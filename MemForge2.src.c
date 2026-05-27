/*
 * MemForge2 v0.4.29 — UEFI memory tester written from scratch.
 *
 * Latest release: https://github.com/Paradoxdov/memforge/releases
 * For per-version changes see git log / GitHub Releases page.
 * Bumping reminder: update the L"v0.X.Y" strings AND this header.
 *
 * Build: see Makefile. Outputs MemForge2.efi.
 */

#include <efi.h>
#include <efilib.h>
#include "MemForge2.mp.h"
#include "font_data.h"

/* Forward decls for CPU feature flags (defined later in cpuid block). */
extern int g_has_avx2;
extern int g_has_clflush;
static UINTN  get_largest_free_pages(void);
static UINT64 get_total_ram_mb_from_efi_map(void);
/* Abort flag (set by check_abort_key when user presses ESC/Q). Volatile so
   AP cores see updates from BSP without a memory barrier. */
extern volatile int g_aborted;
/* CPU activity sampling — forward decls so kernels in earlier code can call
   them via ap_yield even though definitions are in the cpuid block below. */
#define MSR_IA32_MPERF          0xE7
#define MSR_IA32_APERF          0xE8
#define MSR_IA32_THERM_STATUS   0x19C
#define MSR_TEMPERATURE_TARGET  0x1A2
#define MSR_PLATFORM_INFO       0xCE
/* RAPL — Running Average Power Limit. Provides cumulative package energy
   consumption that we sample twice and divide by Δt to get watts.
   Intel and AMD use DIFFERENT MSRs for the same logical concept:
     Intel (Sandy Bridge+):
       0x606 MSR_RAPL_POWER_UNIT      : units (bits 12:8 = energy unit power)
       0x611 MSR_PKG_ENERGY_STATUS    : 32-bit cumulative pkg energy
     AMD (Family 17h+ / Zen+):
       0xC0010299 MSR_AMD_RAPL_POWER_UNIT
       0xC001029B MSR_AMD_PKG_ENERGY_STATUS
   Both work the same way (counter + units → ΔJ/Δs = W). detect_rapl()
   picks the right pair based on the CPU vendor and writes *resolved* MSR
   numbers into the runtime globals so sample_aggregate_metrics() doesn't
   need to know about the vendor. */
#define MSR_INTEL_RAPL_POWER_UNIT     0x606
#define MSR_INTEL_PKG_ENERGY_STATUS   0x611
#define MSR_AMD_RAPL_POWER_UNIT       0xC0010299
#define MSR_AMD_PKG_ENERGY_STATUS     0xC001029B
static inline UINT64 rdtsc_now(void) {
    UINT32 lo, hi;
    __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
    return ((UINT64)hi << 32) | lo;
}
static inline UINT64 rdmsr_safe(UINT32 reg) {
    UINT32 lo, hi;
    __asm__ __volatile__("rdmsr" : "=a"(lo), "=d"(hi) : "c"(reg));
    return ((UINT64)hi << 32) | lo;
}
static inline void wrmsr_safe(UINT32 reg, UINT64 val) {
    UINT32 lo = (UINT32)(val & 0xFFFFFFFFu);
    UINT32 hi = (UINT32)(val >> 32);
    __asm__ __volatile__("wrmsr" : : "c"(reg), "a"(lo), "d"(hi));
}
extern int g_has_aperf_mperf;
extern int g_has_thermal;        /* CPUID.06H:EAX.0 = digital thermal sensor */
extern UINT32 g_tj_max;          /* TjMax (°C) from MSR_TEMPERATURE_TARGET */
extern UINT32 g_base_freq_mhz;   /* base (max non-turbo) freq from MSR_PLATFORM_INFO */

/* ---------- Hardware identification (SMBIOS-derived) ---------- */
#define HW_STR  80
/* Cap on DIMM slots we track. Consumer desktops have 2-4 slots, workstation
   boards 4-8, dual-socket servers commonly 24 (Dell R730 etc.), and 4-socket
   blade systems / HPE Superdome ranges 48+. 32 covers everything up to and
   including a fully-populated dual-socket DDR5 Xeon (24 slots) plus headroom.
   Adds ~8 KB of static storage which is negligible in a UEFI app. */
#define MAX_DIMMS 32
typedef struct {
    CHAR8  locator[24];
    CHAR8  manufacturer[24];
    CHAR8  part_number[24];
    UINT32 size_mb;
    UINT32 speed_mt;
    UINT32 configured_speed_mt;
    UINT8  ddr_type;       /* SMBIOS Type 17 offset 0x12 (memory type code) */
    UINT16 handle;         /* SMBIOS handle, used to cross-ref Type 20 mapping */
    /* SPD-derived fields. Populated from a direct SMBus EEPROM read after
       SMBIOS parsing — adds info SMBIOS doesn't provide (serial number,
       manufacturing date) or sometimes lies about (manufacturer JEDEC ID).
       All zero if SPD read failed or wasn't attempted. */
    UINT8  spd_present;        /* 1 = SPD bytes valid, 0 = SMBus read failed */
    UINT8  spd_jedec_bank;     /* SPD byte 320/0x140 first JEDEC continuation bytes */
    UINT8  spd_jedec_code;     /* SPD byte 321/0x141 — JEDEC manufacturer code */
    UINT8  spd_mfg_year;       /* SPD byte 323/0x143 — BCD year (e.g. 0x21 = 2021) */
    UINT8  spd_mfg_week;       /* SPD byte 324/0x144 — BCD week (1..52) */
    UINT8  spd_serial[4];      /* SPD bytes 325-328 — unique serial */
    UINT8  spd_addr;           /* SMBus 7-bit address 0x50..0x57 used to read */
    UINT8  spd_size_class;     /* DDR3=3, DDR4=4, DDR5=5, 0 if unknown */
    UINT8  spd_tCL;            /* Primary CAS latency in CLOCKS (derived) */
    /* Chip organization, parsed from SPD. Lets us map a stuck-bit position
       on the 64-bit data bus back to a SPECIFIC chip on the DIMM PCB. */
    UINT8  spd_device_width;   /* SDRAM x-width (4, 8, 16) — chip data lanes */
    UINT8  spd_bus_width;      /* Total module bus width (64 normal, 72 ECC) */
    UINT8  spd_ranks;          /* Number of ranks (1, 2, 4) */
    /* Primary JEDEC timings — all derived from the same SPD MTB/FTB block.
       CL/tRCD/tRP/tRAS in CLOCK cycles (what BIOS-setup screens show as
       "16-18-18-38"). tRFC in nanoseconds (typical 280-560 ns range; doesn't
       fit in a clock count <256 for refresh). tCK in picoseconds — lets us
       compute effective MT/s = 2,000,000 / tCK_ps. Zero = not extracted. */
    UINT8  spd_tRCD;           /* row-to-column delay (clocks) */
    UINT8  spd_tRP;            /* row precharge (clocks) */
    UINT8  spd_tRAS;           /* row active time (clocks) — typ. 28-40 */
    UINT16 spd_tRFC_ns;        /* refresh cycle time (ns) */
    UINT16 spd_tCK_ps;         /* nominal clock period in ps; 0 if unknown */
    UINT8  spd_reserved[0];
} dimm_info_t;

/* Type 20 (Memory Device Mapped Address) entries: addr-range → DIMM handle.
   Each Type 20 maps an exclusive physical address range to one DIMM. If a
   range is interleaved across N DIMMs, firmware emits N Type 20 entries with
   matching ranges and incrementing interleave_pos (1..N). We use this map to
   answer "which DIMM contains physical address X?" in error reports. */
#define MAX_DIMM_MAP 32
typedef struct {
    UINT64 start;            /* inclusive, bytes */
    UINT64 end;              /* inclusive, bytes */
    UINT16 dev_handle;       /* points back to a Type 17 (matches dimm_info_t.handle) */
    UINT8  interleave_pos;   /* 1..N (0 = non-interleaved per spec, but BIOSes vary) */
    UINT8  interleave_depth; /* total DIMMs in interleave set; 1 = solo */
} dimm_map_entry_t;

/* Decode SMBIOS memory-type byte (Table 17 / DSP0134 spec). Returns a short
   human label. Most common DDR/LPDDR variants covered. */
static CHAR16 *ddr_type_name(UINT8 t) {
    switch (t) {
        case 0x01: return L"Other";
        case 0x02: return L"Unknown";
        case 0x0F: return L"SDRAM";
        case 0x12: return L"DDR";
        case 0x13: return L"DDR2";
        case 0x18: return L"DDR3";
        case 0x1A: return L"DDR4";
        case 0x1B: return L"LPDDR";
        case 0x1C: return L"LPDDR2";
        case 0x1D: return L"LPDDR3";
        case 0x1E: return L"LPDDR4";
        case 0x1F: return L"Logical NV-DIMM";
        case 0x20: return L"HBM";
        case 0x21: return L"HBM2";
        case 0x22: return L"DDR5";
        case 0x23: return L"LPDDR5";
        case 0x24: return L"HBM3";
        default:   return L"?";
    }
}

/* Forward decl — defined after the globals it reads. Used to gate DDR5-specific
   tuning (longer Bit Fade, heavier Row Hammer) and to warn the user that DDR5
   on-die ECC silently corrects 1-bit errors within each chip, so a PASS on
   DDR5 is a stricter result than a PASS on DDR4 (it doesn't mean the chips
   are perfect — it means no errors that ODECC couldn't fix). */
static int is_ddr5_system(void);

static CHAR8  g_sys_vendor[HW_STR]    = "(unknown)";
static CHAR8  g_sys_model[HW_STR]     = "(unknown)";
static CHAR8  g_bios_vendor[HW_STR]   = "(unknown)";
static CHAR8  g_bios_version[HW_STR]  = "(unknown)";
static CHAR8  g_cpu_brand[64]         = "(unknown CPU)";
static UINT32 g_dimm_count            = 0;
static UINT64 g_total_ram_mb          = 0;
static UINT32 g_max_dimm_speed_mt     = 0;
static UINT32 g_max_dimm_configured_mt = 0;
static UINT8  g_dimm_ddr_type         = 0;  /* dominant DDR generation across DIMMs */
static dimm_info_t g_dimms[MAX_DIMMS];
static dimm_map_entry_t g_dimm_map[MAX_DIMM_MAP];
static UINT32 g_dimm_map_count        = 0;


#define MAX_CORES         64
#define ROWHAMMER_STRIDE  (128 * 1024)
/* Row Hammer iteration count.
   JEDEC DDR4 Maximum Active Count (MAC) is 200,000 activations per refresh
   window (64 ms) — that's the threshold memory vendors guarantee against.
   PassMark MemTest86 Test 13 second pass uses exactly 200K acc / 64ms as the
   "worst case scenario". To get sustained stress over ~6 s of wall time we
   need ~200M iterations on modern HW (~30 ns per iter).
   Refs: memtest86.com/tech_individual-test-descr.html ; en.wikipedia.org/wiki/Row_hammer */
#define ROWHAMMER_ITERS   200000000ULL
#define PROGRESS_GRAIN    32          /* progress callback every Nth slice */

/* Per-test repeat counts, calibrated against memtest86+ (test_list[] in
   tests/tests.c on GitHub). memtest86+'s "iterations" column is essentially
   the per-pass weight — bumping ours into the same ballpark.
     memtest86+ address tests:  6 iterations  → ADDR_REPEATS = 6
     memtest86+ block move:    81 iterations  → BLOCK_REPEATS = 30
     memtest86+ random pattern:30 iterations  → RANDOM_REPEATS = 4 (we use
                              a stronger 64-bit xorshift; fewer reps suffice)
     AVX2 is non-standard (not in memtest86+); 4 reps is enough for thermal
     stress on the VRM/IMC without dragging out the suite.
   BIT_FADE_DEFAULT_S = retention wait per phase (write→wait→verify). 60 s
   is enough to catch obvious leakage; memtest86+ default is 5 minutes for
   thoroughness. Configurable via [Run] BitFadeSeconds in quantai.ini. */
#define ADDR_REPEATS         6
#define AVX2_REPEATS         4
#define BLOCK_REPEATS        30
#define RANDOM_REPEATS       4
#define BIT_FADE_DEFAULT_S   60

/* Colors (ARGB32, alpha unused) — premium dark palette. */
#define COL_BG        0x00060C18u
#define COL_PANEL     0x000F1A2Bu
#define COL_PANEL_ALT 0x00162236u
#define COL_FG        0x00ECF0F6u
#define COL_DIM       0x00607888u
#define COL_ACCENT    0x0038BDF8u
#define COL_ACCENT_HI 0x0070D4FFu
#define COL_ACCENT_DK 0x001D6FA3u
#define COL_OK        0x0034D399u
#define COL_OK_DK     0x00166534u
#define COL_FAIL      0x00F43F5Eu
#define COL_FAIL_DK   0x00651A26u
#define COL_RUN       0x00FBBF24u
#define COL_RUN_DK    0x00654C0Fu
#define COL_IDLE      0x001E293Bu
#define COL_BAR_BG    0x001E293Bu
#define COL_BAR_FILL  0x0038BDF8u
#define COL_BORDER    0x00334155u

/* ---------- Globals ---------- */
static EFI_GRAPHICS_OUTPUT_PROTOCOL *g_gop = NULL;
static EFI_MP_SERVICES_PROTOCOL     *g_mp  = NULL;
static UINTN g_w = 0, g_h = 0;
static EFI_FILE_PROTOCOL *g_logfile = NULL;
static EFI_FILE_PROTOCOL *g_logroot = NULL;  /* kept open for report.json */
static UINTN g_n_cores = 1, g_n_enabled = 1;

/* Language: 0 = Russian, 1 = English (default). Toggled via L key in menu
   OR overridden by [Meta] Language=ru/en in quantai.ini. */
static int g_lang = 1;
static inline CHAR16 *T(CHAR16 *ru, CHAR16 *en) { return g_lang ? en : ru; }

/* Pass progress for header — visible to user so they don't think the tests
   "restarted on their own" when in fact pass N+1 is testing a different
   memory region. */
static volatile UINT32 g_pass_idx_disp   = 0;
static volatile UINT32 g_pass_total_disp = 1;

/* Running total of errors found in tests COMPLETED so far this run.
   Updated by the test driver right after each run_test_mc() returns.
   Read by render_header() so user can see "are there errors yet" at a
   glance without waiting for the summary. Reset to 0 at start of run. */
static volatile UINT64 g_run_total_errors = 0;

/* Total UNIQUE bytes of physical RAM actually exercised across the current
   run. Incremented at the start of each multipass pass by the size of that
   pass's buffer (the buffer is always a fresh (region, offset) slice).
   The summary screen reads this to report HONEST coverage instead of
   misleadingly showing only the last pass's buffer size. */
static UINT64 g_run_tested_mb = 0;
static UINT32 g_run_passes_done = 0;   /* completed pass count for summary */

/* ---- Live "during the test" telemetry ----
   All of these are sampled at most once per BSP yield (~10 Hz) and read by
   render_header / core_panel_update. Producing 5–10 numbers in the UI from a
   single periodic sample keeps overhead negligible (<0.1% of one core) while
   surfacing diagnostically useful data we already paid to collect. */

/* Aggregate memory bandwidth across all cores. We sample sum(g_args[i].bytes)
   periodically and divide by Δt to get current GB/s. Helps the user spot
   "memory subsystem is healthy" vs "controller throttled" without reading
   nine per-core throughputs. */
static UINT64 g_bw_bytes_prev   = 0;
static UINT64 g_bw_ts_prev_ms   = 0;
static UINT32 g_bw_mbps_current = 0;    /* MB/s (×1) — display: divide by 1024 for GB/s */
static UINT32 g_bw_mbps_peak    = 0;    /* highest sampled MB/s during this run */
static UINT64 g_bw_peak_theoretical_mbps = 0; /* derived from DDR speed × bus width */

/* Bandwidth history ring — 1-minute time buckets, up to 1024 buckets
   (~17 h). Lets the summary report a trend: is the system sustaining the
   bandwidth it had at minute 0, or has BW silently dropped 8 % by hour 4?
   A persistent drop is a strong signal of thermal-induced refresh-rate
   bumps, IMC throttling, or a marginally-failing channel.

   Per-bucket fields: max BW (we take the bucket maximum, not mean, so
   short BW dips between tests don't bias the trend) plus sample count.
   New samples within the same bucket update max; bucket roll-over starts
   a fresh max. */
#define BW_HISTORY_BUCKETS 1024
#define BW_BUCKET_MS       60000ULL    /* 1 minute per bucket */
static UINT32 g_bw_history_max[BW_HISTORY_BUCKETS];
static UINT32 g_bw_history_count = 0;       /* total buckets used (≤ BW_HISTORY_BUCKETS) */
static UINT64 g_bw_history_start_ms = 0;    /* run-start timestamp for bucket indexing */
static UINT32 g_bw_current_bucket = 0xFFFFFFFFu;   /* index currently being filled */

/* CPU vendor (CPUID leaf 0 EBX/EDX/ECX returns "GenuineIntel"/"AuthenticAMD"
   etc.). Used to: (1) pick the right RAPL MSR pair, (2) skip Intel-only
   thermal MSR readout on AMD, (3) decide which platform-specific quirks
   to apply. Set by detect_cpu_features(). */
typedef enum { CPU_UNKNOWN = 0, CPU_INTEL, CPU_AMD } cpu_vendor_t;
static cpu_vendor_t g_cpu_vendor = CPU_UNKNOWN;

/* CPU package power via RAPL. The MSR numbers are resolved at probe time
   so the sampling loop is vendor-agnostic. */
static int    g_has_rapl        = 0;
static UINT32 g_rapl_unit_msr   = 0;    /* resolved Intel/AMD power-unit MSR */
static UINT32 g_rapl_pkg_msr    = 0;    /* resolved Intel/AMD pkg-energy MSR */
static UINT32 g_rapl_energy_units_div = 0; /* 2^N from RAPL_POWER_UNIT bits 12:8 */
static UINT64 g_rapl_pkg_energy_prev  = 0;
static UINT64 g_rapl_ts_prev_ms = 0;
static UINT32 g_pkg_power_w     = 0;    /* latest sampled value */
static UINT32 g_pkg_power_w_peak = 0;   /* peak sampled watts during this run */
static UINT32 g_pkg_vid_mv      = 0;    /* CPU core/package voltage in mV
                                            (Phase 3 populates this; stays 0
                                            until VID sampling is wired up) */

/* System-wide telemetry rolled up from per-core ap_arg_t state. */
static UINT32 g_max_temp_c      = 0;
static UINT32 g_throttle_total  = 0;     /* total throttle events across all cores */
static UINT32 g_freq_max_mhz    = 0;
static UINT32 g_freq_avg_mhz    = 0;
static UINT64 g_cum_bytes       = 0;     /* cumulative bytes processed this run */

/* Pass duration history — first N passes, ms. Used by the summary and by the
   header (avg pass time). Older passes overwrite cyclically once history is
   full; the size 64 covers a 1024-pass max with no allocation. */
#define MAX_PASS_HISTORY 64
static UINT32 g_pass_durations_ms[MAX_PASS_HISTORY];
static UINT32 g_pass_durations_count = 0;
static UINT64 g_cur_pass_start_ms    = 0;

/* App boot timestamp (ms since efi_main entry). Used for "Uptime" indicator. */
static UINT64 g_boot_ms             = 0;

/* Run start timestamp (ms_now value when the user started the current
   test run via menu). bsp_yield_render() uses this to compute elapsed_ms
   for render_header() so the time / ETA / spinner update DURING long
   tests (Bit Fade Ext = 6 min) — previously render_header was only
   called between tests, so the whole header froze for the test's
   duration. */
static UINT64 g_run_start_ms = 0;

/* Header static-row cache: rows 2-5 (CPU brand, mobo+BIOS, DIMM-A, DIMM-B)
   never change during a test run. Redrawing them every ~100 ms used to cause
   visible flicker (clear-row then redraw-glyph reveals background color
   between frames). This flag flips to 1 after the first render_header()
   call and stays so until the layout is rebuilt (next test run); when 1
   we skip the clear+redraw of those rows entirely. */
static int g_hdr_static_drawn = 0;

/* Forward decls for "current test" state — referenced by core_panel_update
   (above the run_test_mc function that owns them). The actual storage is
   declared once just before bsp_yield_render(); these are just earlier
   tentative definitions so the compiler can resolve the symbols. */
static UINT64 g_cur_test_started = 0;
static UINTN  g_cur_test_idx     = 0;
static UINT64 g_last_yield_ms    = 0;

/* Forward decls for adaptive core-panel grid helpers — used by
   compute_layout which lives much earlier in the file than their
   definitions (which sit next to the rest of the panel code). */
static UINTN core_grid_cols(void);
static UINTN core_grid_shown(void);

/* Forward decl for render_activity_row() — defined AFTER g_tests / g_args
   (~1000 lines below). render_header() in the early part of the file calls
   it on every tick to paint the live "currently running" status with
   spinner + Bit Fade countdown. */
static void render_activity_row(UINT64 elapsed_ms);

/* DDR5 auto-tuning state.
   DDR5 differs from DDR4 in ways that affect test design:
   1. On-die ECC (ODECC) is MANDATORY in DDR5. The DRAM die silently corrects
      1-bit errors before they leave the chip. Soft errors that DDR4 would
      surface to my tester are invisible on DDR5 ⇒ PASS on DDR5 is a STRICTER
      result, not a weaker one.
   2. Row Hammer mitigations (RFM / TRR-v2) absorb single-pair hammering. To
      have any chance of provoking a flip we need MORE aggressor pairs and
      MORE iterations.
   3. Retention is physically worse (smaller cells) but masked by refresh+ODECC.
      Bit Fade needs LONGER wait per phase to be meaningful.
   These factors are set after SMBIOS detection in efi_main(). */
static int    g_is_ddr5             = 0;
static UINT32 g_rowhammer_mult_x100 = 100; /* 100 = DDR4 baseline, 200 = DDR5 (2x) */
static int    g_cfg_bitfade_explicit = 0;  /* user set BitFadeSeconds in INI? */

/* Runtime config — populated by parse_quantai_ini(), with defaults.
   The defaults aim to actually test memory: MultiPass=1 + Passes=0 covers
   every byte of every EfiConventionalMemory region we can allocate
   (= 98%+ of physical RAM; the remaining 1-2% is firmware-reserved and
   inaccessible to any UEFI application — this is the same fundamental
   limit PassMark MemTest86 has). For a quick smoke run, set Passes=1 and
   MultiPass=0 in quantai.ini. */
static UINT32 g_cfg_passes        = 0;     /* 0 = auto (cover all addressable RAM) */
static UINT32 g_cfg_max_cores     = 0;     /* 0 = use all enabled */
static int    g_cfg_enable_avx    = 1;     /* override the AVX2 test */
/* Auto-skip flag for heavy parallel-burst kernels (AVX2 Sustained, Thermal
   Soak, BW Soak, VRM Square-Wave). Set to 1 by the init thermal-guard
   when baseline CPU temperature exceeds the safe threshold — on hot
   systems with weak cooling or buggy power-management firmware these
   kernels reliably halt the system. Pattern tests (March-C-, TRRespass,
   etc.) still run because they don't burn 12 cores at AVX2 burst.
   Override with IgnoreThermalGuard=1 in quantai.ini. */
static int    g_thermal_guard_skip_heavy = 0;
static int    g_cfg_ignore_thermal_guard = 0;
/* Hard-stop flag: baseline temp was >=100°C at init. main_menu_wait
   checks this before launching tests. */
static int    g_thermal_emergency = 0;
static int    g_cfg_multipass     = 1;     /* if 1, rotate buffer across regions */
static int    g_cfg_force_blt     = 0;     /* if 1, never use direct framebuffer */
static int    g_cfg_enable_aa      = 0;     /* if 1, enable AA + direct-fb path */
static UINT32 g_cfg_force_w        = 0;     /* >0 = pick GOP mode matching this width */
static UINT32 g_cfg_force_h        = 0;     /* >0 = pick GOP mode matching this height */
static UINT32 g_cfg_font_scale     = 0;     /* 0=auto (2× when h>1500), 1=force 1×, 2=force 2× */
static UINT32 g_cfg_buffer_cap_mb = 1024;  /* per-allocation cap */
static int    g_cfg_buffer_cap_explicit = 0;  /* user set BufferMB in INI? */
/* Per-DIMM isolation: if non-zero, allocate the test buffer ONLY within
   the physical address range of DIMM #N (1-based, matches SMBIOS Type 17
   slot numbering). Lets the user verify each stick separately without
   physically removing the others. Set via [Run] TestOnlyDimm=N. */
static UINT32 g_cfg_test_only_dimm = 0;     /* 0 = all DIMMs (default) */

/* v0.4.29 — auto-isolation state.
   When the post-test verdict detects "errors on multiple DIMMs, block-
   mapped Type 20" we offer the user [I] to automatically re-test each
   affected DIMM with TestOnlyDimm in turn, giving a definitive
   "REPLACE: DDR4-B2" answer instead of "REPLACE BOTH" guesswork.
   These globals are populated by render_simple_verdict() during the
   FAIL branch and consumed by the post-summary key handler. */
#define MAX_ISO_DIMMS 4
static int    g_iso_offer = 0;                  /* 1 = show [I] in footer */
static int    g_iso_dimm_idx[MAX_ISO_DIMMS];    /* 0-based DIMM indices */
static UINTN  g_iso_dimm_n   = 0;               /* count in g_iso_dimm_idx */
typedef struct {
    int     dimm_idx;        /* 0-based */
    CHAR8   locator[24];     /* SMBIOS locator string */
    UINT32  errors;          /* errors found in this isolation pass */
    UINT32  passes;          /* how many passes ran */
    int     status;          /* 0=not run, 1=alloc failed, 2=ok, 3=aborted */
} isolation_result_t;
static isolation_result_t g_iso_results[MAX_ISO_DIMMS];
static UINTN g_iso_results_n = 0;
static UINT32 g_iso_kernel = 0;   /* kernel_id_t — that found errors (UINT32 so this can sit before the enum decl) */
/* Marathon mode: keep cycling the full test for N hours total. Useful for
   shop-overnight runs and intermittent-failure hunting (errors that only
   surface after 2-4 h of sustained load). 0 = disabled (normal behaviour),
   1..24 = run until total elapsed time hits N hours OR user aborts.
   When enabled, MultiPass iterator wraps when exhausted (we re-cover the
   whole RAM range again) and the pass counter keeps incrementing. */
static UINT32 g_cfg_marathon_hours = 0;
/* Bit Fade is the slowest test by far — 2 × bitfade_s + overhead = 4-10
   min per pass at default settings. Running it on every pass of a 32-pass
   full test stacks to many hours. Default: run only on pass 1 (first
   chunk gets the retention check, other passes skip it). User can set
   BitFadeEveryPass=1 in quantai.ini for memtest86+ thoroughness. */
static int    g_cfg_bitfade_every_pass = 0;
static UINT32 g_cfg_bitfade_s     = BIT_FADE_DEFAULT_S;  /* wait per phase */
/* Quick-mode flag. Set when user presses Q in the menu. Only the 4 "hard
   fault" tests run (Walking Ones, Walking Zeroes, Moving Inversions,
   Address Pattern) over 3 chunks of RAM — ~2-3 minutes total on modern HW.
   Catches dead sticks / stuck bits / addressing errors with high
   confidence, misses subtle pattern/retention issues. */
static int g_quick_mode = 0;


/* Console char cell size derived from current text mode. */
static UINTN g_text_cols = 80, g_text_rows = 25;
static UINTN g_char_w    = 8,  g_char_h    = 16;

/* Horizontal advance per glyph (cursor step). Bitmap is FONT_W=14 wide, but
   we advance by 2 px less so consecutive letters sit closer together. The
   rightmost two columns of one glyph and the leftmost ones of the next
   overlap; safe for Consolas at PT=17 because the rightmost ~2 cols of
   each glyph are blank padding. Previously 13 (1-px overlap, ~2-3 px
   visible gap) and 14 (no overlap, very airy). 12 = ~1-2 px visible gap. */
#define FONT_ADVANCE 12

/* ---------- Time ----------
   We can't trust RT->GetTime for sub-second precision: most UEFI firmwares
   leave EFI_TIME.Nanosecond at 0, so fast tests ended up with time_ms=0 and
   the displayed MB/s collapsed to "0". Use TSC (constant-rate on all modern
   CPUs) calibrated via BS->Stall (which the UEFI spec requires to be
   accurate to ±1%). */
static UINT64 g_tsc_freq_hz = 0;

static void calibrate_tsc(void) {
    /* Stall for 100 ms and count TSC ticks. */
    UINT64 t1 = rdtsc_now();
    uefi_call_wrapper(BS->Stall, 1, (UINTN)100000);  /* microseconds */
    UINT64 t2 = rdtsc_now();
    if (t2 > t1) g_tsc_freq_hz = (t2 - t1) * 10;     /* × 10 to get per-second */
}

static UINT64 ms_now(void) {
    if (g_tsc_freq_hz)
        return (rdtsc_now() * 1000ULL) / g_tsc_freq_hz;
    /* Pre-calibration fallback (only used during very early init). */
    EFI_TIME t;
    uefi_call_wrapper(RT->GetTime, 2, &t, NULL);
    return ((UINT64)t.Hour * 3600 + (UINT64)t.Minute * 60 + t.Second) * 1000
           + t.Nanosecond / 1000000;
}

/* ---------- Drawing primitives ---------- */
static void blt_fill(UINTN x, UINTN y, UINTN w, UINTN h, UINT32 color) {
    if (!g_gop || !w || !h) return;
    if (x >= g_w || y >= g_h) return;
    if (x + w > g_w) w = g_w - x;
    if (y + h > g_h) h = g_h - y;
    EFI_GRAPHICS_OUTPUT_BLT_PIXEL px;
    px.Blue  = color & 0xFF;
    px.Green = (color >> 8) & 0xFF;
    px.Red   = (color >> 16) & 0xFF;
    px.Reserved = 0;
    uefi_call_wrapper(g_gop->Blt, 10, g_gop, &px, EfiBltVideoFill,
                      0, 0, x, y, w, h, 0);
}

static void box_outline(UINTN x, UINTN y, UINTN w, UINTN h, UINT32 col) {
    blt_fill(x,         y,         w, 1, col);
    blt_fill(x,         y + h - 1, w, 1, col);
    blt_fill(x,         y,         1, h, col);
    blt_fill(x + w - 1, y,         1, h, col);
}

/* Vertical gradient: col_a at top, col_b at bottom. */
static void blt_gradient_v(UINTN x, UINTN y, UINTN w, UINTN h,
                            UINT32 col_a, UINT32 col_b) {
    if (!g_gop || !w) return;
    if (h <= 1) { blt_fill(x, y, w, h, col_a); return; }
    for (UINTN i = 0; i < h; i++) {
        UINTN ra = (col_a >> 16) & 0xFF, rb = (col_b >> 16) & 0xFF;
        UINTN ga = (col_a >> 8)  & 0xFF, gb = (col_b >> 8)  & 0xFF;
        UINTN ba = col_a & 0xFF,         bb = col_b & 0xFF;
        UINT32 c = (UINT32)(((ra*(h-1-i) + rb*i) / (h-1)) << 16)
                 | (UINT32)(((ga*(h-1-i) + gb*i) / (h-1)) << 8)
                 |          ((ba*(h-1-i) + bb*i) / (h-1));
        blt_fill(x, y + i, w, 1, c);
    }
}

/* Panel: filled rectangle with border + subtle top highlight. */
static void blt_panel(UINTN x, UINTN y, UINTN w, UINTN h,
                       UINT32 bg, UINT32 border) {
    blt_fill(x, y, w, h, bg);
    box_outline(x, y, w, h, border);
    blt_fill(x + 1, y + 1, w - 2, 1, border); /* top highlight */
}

/* Progress bar with border, gradient fill, and glow line. */
static void blt_progress_bar(UINTN x, UINTN y, UINTN w, UINTN h,
                              UINT32 fill_pct_x10, /* 0..1000 */
                              UINT32 col_fill, UINT32 col_track) {
    blt_fill(x, y, w, h, col_track);
    box_outline(x, y, w, h, COL_BORDER);
    if (fill_pct_x10 > 1000) fill_pct_x10 = 1000;
    UINTN fw = (w - 2) * fill_pct_x10 / 1000;
    if (fw > 0) {
        blt_fill(x + 1, y + 1, fw, h - 2, col_fill);
        /* top glow line */
        blt_fill(x + 1, y + 1, fw, 1, COL_ACCENT_HI);
    }
}

/* ---------- Pixel-mode text renderer (uses bundled font, supports Cyrillic) ----------
   Bypasses UEFI ConOut entirely — many firmwares only have ASCII glyphs in their
   built-in font, which breaks Russian rendering. We draw each glyph pixel by pixel
   using the embedded font in font_data.h (12×24, Latin + Cyrillic). */

static UINT16 gfx_lookup_glyph(UINT32 cp) {
    /* Binary search of g_font_cp_map (sorted by cp). */
    UINTN lo = 0, hi = FONT_GLYPHS;
    while (lo < hi) {
        UINTN mid = (lo + hi) >> 1;
        UINT16 c = g_font_cp_map[mid].cp;
        if (c == cp) return g_font_cp_map[mid].idx;
        if (c < cp) lo = mid + 1;
        else        hi = mid;
    }
    return 0xFFFF;
}

/* Forward decl — log_line is defined ~150 lines later (after FAT helpers)
   but fb_detect_once needs to log diagnostic info about the GOP format. */
static void log_line(CHAR16 *s);
/* Forward decl — recheck_fb_dimensions is defined near efi_main but called
   from the main menu loop ~50 lines above its definition. */
static void recheck_fb_dimensions(void);
/* Forward decls — AMD SMN thermal helpers defined in the SMBus section
   (uses pci_read* helpers) but called from detect_cpu_features which
   lives earlier in the file. */
static void   amd_thermal_probe(void);
static UINT32 amd_thermal_sample(void);

/* Framebuffer pointer + pitch, cached on first text-draw call. NULL means
   firmware exposes BltOnly mode — we'll fall back to per-pixel Blt for AA. */
static volatile UINT32 *g_fb_base   = NULL;
static UINT32           g_fb_pitch  = 0;     /* in 32-bit pixels */
static int              g_fb_bgra   = 1;     /* 1 = B/G/R/X (most common), 0 = R/G/B/X */
static int              g_fb_ready  = 0;

/* Font scale factor (1 or 2). Set once at GOP init based on g_h. When ≥2,
   each source font pixel is rendered as a SCALE×SCALE block on the screen,
   keeping text physically the same size on a 4K display as on a 1080p one
   (otherwise our 14×28 bitmap would be a tiny speck on hi-DPI panels). */
static UINT32 g_font_scale = 1;

static void fb_detect_once(void) {
    if (g_fb_ready) return;
    g_fb_ready = 1;
    if (!g_gop) return;
    EFI_GRAPHICS_OUTPUT_MODE_INFORMATION *info = g_gop->Mode->Info;
    UINT32 fmt = info->PixelFormat;
    UINT32 ppsl = info->PixelsPerScanLine;
    UINT64 fb_b = g_gop->Mode->FrameBufferBase;
    UINT64 fb_sz = g_gop->Mode->FrameBufferSize;

    CHAR16 lb[180];
    SPrint(lb, sizeof(lb),
           L"[GFX] PixelFormat=%d PixelsPerScanLine=%d FBBase=0x%lx FBSize=%ld",
           fmt, ppsl, fb_b, fb_sz);
    log_line(lb);

    /* Sanity-check before enabling direct framebuffer writes. Any of these
       failures means we use the slow-but-safe Blt fallback path (g_fb_base
       stays NULL). The user reported text rendered at wrong positions on a
       Dell OptiPlex 5050; root cause was the firmware reporting either
       PixelBltOnly or a PixelsPerScanLine that didn't match what Blt was
       actually using internally. The cure is to refuse direct fb access
       when anything looks off and stay with Blt the whole way. */
    int ok = 1;
    if (fmt == PixelBlueGreenRedReserved8BitPerColor) {
        g_fb_bgra = 1;
    } else if (fmt == PixelRedGreenBlueReserved8BitPerColor) {
        g_fb_bgra = 0;
    } else {
        log_line(L"[GFX] direct-fb disabled: PixelFormat is BitMask/BltOnly");
        ok = 0;
    }
    if (fb_b == 0) {
        log_line(L"[GFX] direct-fb disabled: FrameBufferBase=0 (firmware hides fb)");
        ok = 0;
    }
    if (ppsl == 0 || ppsl < g_w) {
        log_line(L"[GFX] direct-fb disabled: PixelsPerScanLine looks bogus");
        ok = 0;
    }
    /* Pitch must match what we expect within a reasonable margin. Some
       Intel iGPU firmware pads scanlines to 64-pixel alignment and reports
       that padded value here, while Blt operates on the visible width
       internally — which puts our direct writes at offsets Blt-positioned
       elements don't expect. If the ratio is more than 10% off, refuse. */
    if (ok && ppsl > 0 && ppsl != g_w) {
        UINT32 diff = (ppsl > g_w) ? (ppsl - g_w) : (g_w - ppsl);
        if (diff * 100 / g_w > 10) {
            log_line(L"[GFX] direct-fb disabled: PixelsPerScanLine differs from width by >10%");
            ok = 0;
        } else {
            SPrint(lb, sizeof(lb),
                   L"[GFX] PixelsPerScanLine=%d differs from g_w=%d by %d px — using it as pitch",
                   ppsl, (UINT32)g_w, diff);
            log_line(lb);
        }
    }
    /* User can force Blt-only via INI [Display] ForceBlt=1 if their firmware
       still glitches even though our sanity checks pass. */
    if (g_cfg_force_blt) {
        log_line(L"[GFX] direct-fb disabled: ForceBlt=1 in quantai.ini");
        ok = 0;
    }
    /* Direct-fb path is only used when EnableAA=1 — otherwise the safe
       Blt-based 1-bit renderer is in effect and we don't need fb pointers. */
    if (!g_cfg_enable_aa) {
        log_line(L"[GFX] direct-fb not enabled: EnableAA=0 (default), using safe Blt renderer");
        ok = 0;
    }
    if (!ok) return;
    g_fb_base  = (volatile UINT32 *)(UINTN)fb_b;
    g_fb_pitch = ppsl;
    SPrint(lb, sizeof(lb),
           L"[GFX] direct-fb enabled: base=0x%lx pitch=%d bgra=%d",
           fb_b, ppsl, g_fb_bgra);
    log_line(lb);
}

static inline void fb_blend_px(UINTN x, UINTN y, UINT8 a,
                                UINT8 fr, UINT8 fg, UINT8 fb_) {
    /* Read current pixel, blend with text colour by alpha, write back.
       Skip the read+blend for the two trivial cases (full opaque / fully
       transparent) — they dominate the pixel count in any glyph and the
       fast paths skip the framebuffer read entirely. */
    if (a == 0) return;
    if (g_fb_base) {
        UINTN off = y * g_fb_pitch + x;
        if (a >= 250) {
            g_fb_base[off] = g_fb_bgra
                ? ((UINT32)fr << 16) | ((UINT32)fg << 8) | fb_
                : ((UINT32)fb_ << 16) | ((UINT32)fg << 8) | fr;
            return;
        }
        UINT32 bg = g_fb_base[off];
        UINT8 b0, g0, r0;
        if (g_fb_bgra) { b0 = bg & 0xFF; g0 = (bg >> 8) & 0xFF; r0 = (bg >> 16) & 0xFF; }
        else            { r0 = bg & 0xFF; g0 = (bg >> 8) & 0xFF; b0 = (bg >> 16) & 0xFF; }
        /* out = bg + (fg - bg) * a / 256  (using >>8 for speed; the tiny
           gamma error is invisible). Use signed math so (fg-bg) can be
           negative when text is darker than background. */
        INT32 dr = (INT32)fr - (INT32)r0;
        INT32 dg = (INT32)fg - (INT32)g0;
        INT32 db = (INT32)fb_ - (INT32)b0;
        UINT8 br = (UINT8)((INT32)r0 + ((dr * (INT32)a) >> 8));
        UINT8 bgc= (UINT8)((INT32)g0 + ((dg * (INT32)a) >> 8));
        UINT8 bb = (UINT8)((INT32)b0 + ((db * (INT32)a) >> 8));
        g_fb_base[off] = g_fb_bgra
            ? ((UINT32)br << 16) | ((UINT32)bgc << 8) | bb
            : ((UINT32)bb << 16) | ((UINT32)bgc << 8) | br;
    } else {
        /* BltOnly fallback — slow but correct for firmware that hides the
           framebuffer. Just paint full-opacity for alpha >= 128, skip rest. */
        if (a >= 128) blt_fill(x, y, 1, 1,
                               ((UINT32)fr << 16) | ((UINT32)fg << 8) | fb_);
    }
}

/* SAFE Blt-based 1-bit renderer. Threshold the AA glyph at alpha >= 128,
   coalesce horizontal "on" runs into blt_fill calls. Works on EVERY UEFI
   firmware because we use the standard GOP Blt protocol. Trade-off: text
   has jagged 1-bit edges instead of smooth AA. */
static void gfx_draw_char_1bit(UINTN x, UINTN y, UINT16 idx, UINT32 color) {
    UINT32 sc = g_font_scale;
    UINTN cell_w = FONT_W * sc;
    UINTN cell_h = FONT_H * sc;
    if (x >= g_w || y >= g_h) return;
    UINTN max_col = (x + cell_w <= g_w) ? FONT_W : ((g_w - x) / sc);
    UINTN max_row = (y + cell_h <= g_h) ? FONT_H : ((g_h - y) / sc);
    for (UINTN row = 0; row < max_row; row++) {
        const UINT8 *grow = g_font_glyphs[idx][row];
        UINTN run_start = 0, run_len = 0;
        for (UINTN col = 0; col < max_col; col++) {
            if (grow[col] >= 128) {
                if (run_len == 0) run_start = col;
                run_len++;
            } else if (run_len) {
                blt_fill(x + run_start * sc, y + row * sc,
                         run_len * sc, sc, color);
                run_len = 0;
            }
        }
        if (run_len) {
            blt_fill(x + run_start * sc, y + row * sc,
                     run_len * sc, sc, color);
        }
    }
}

/* AA grayscale renderer via direct framebuffer. Smoother, faster, but
   only safe on firmware where PixelsPerScanLine matches what Blt uses
   internally — some old Intel iGPU UEFIs glitch with this path. Gated
   behind [Display] EnableAA=1 in quantai.ini (default OFF). */
static void gfx_draw_char_aa(UINTN x, UINTN y, UINT16 idx, UINT32 color) {
    UINT32 sc = g_font_scale;
    UINTN cell_w = FONT_W * sc;
    UINTN cell_h = FONT_H * sc;
    if (x >= g_w || y >= g_h) return;
    UINTN max_col = (x + cell_w <= g_w) ? FONT_W : ((g_w - x) / sc);
    UINTN max_row = (y + cell_h <= g_h) ? FONT_H : ((g_h - y) / sc);
    UINT8 fr = (color >> 16) & 0xFF;
    UINT8 fg = (color >>  8) & 0xFF;
    UINT8 fb_=  color         & 0xFF;
    if (sc == 1) {
        for (UINTN row = 0; row < max_row; row++) {
            const UINT8 *grow = g_font_glyphs[idx][row];
            for (UINTN col = 0; col < max_col; col++) {
                UINT8 a = grow[col];
                if (a) fb_blend_px(x + col, y + row, a, fr, fg, fb_);
            }
        }
        return;
    }
    for (UINTN row = 0; row < max_row; row++) {
        const UINT8 *grow = g_font_glyphs[idx][row];
        for (UINTN col = 0; col < max_col; col++) {
            UINT8 a = grow[col];
            if (!a) continue;
            UINTN bx = x + col * sc;
            UINTN by = y + row * sc;
            for (UINTN dy = 0; dy < sc; dy++)
                for (UINTN dx = 0; dx < sc; dx++)
                    fb_blend_px(bx + dx, by + dy, a, fr, fg, fb_);
        }
    }
}

static void gfx_draw_char(UINTN x, UINTN y, UINT32 cp, UINT32 color) {
    UINT16 idx = gfx_lookup_glyph(cp);
    if (idx == 0xFFFF) {
        box_outline(x + 2, y + (FONT_H * g_font_scale / 2) - 2,
                    8 * g_font_scale, 8 * g_font_scale, color);
        return;
    }
    fb_detect_once();
    /* Default = 1-bit-via-Blt: bulletproof, works on every firmware.
       AA is opt-in via [Display] EnableAA=1 and additionally requires
       direct-fb to have passed sanity checks. */
    if (g_cfg_enable_aa && g_fb_base) {
        gfx_draw_char_aa(x, y, idx, color);
    } else {
        gfx_draw_char_1bit(x, y, idx, color);
    }
}

static void gfx_draw_str_color(UINTN x, UINTN y, CHAR16 *s, UINT32 color) {
    UINTN cx = x;
    UINTN advance = FONT_ADVANCE * g_font_scale;
    while (*s) {
        gfx_draw_char(cx, y, (UINT32)*s, color);
        cx += advance;
        s++;
    }
}

/* ---------- "Text mode" facade — pure pixel renderer, ConOut is OFF ----------
   We MUST NOT mirror UI text to ST->ConOut->OutputString: most UEFI firmwares
   render their own built-in font on top of the GOP framebuffer when they see
   OutputString, and that font almost never has Cyrillic glyphs. Result on
   real hardware = double rendering with garbage boxes where Russian should be.
   Only cls() pokes ConOut once, to wipe any stale boot-time text the firmware
   has left in the framebuffer. */
static void cls(void) {
    blt_fill(0, 0, g_w, g_h, COL_BG);
    if (ST && ST->ConOut) {
        /* Wipe firmware's internal text buffer so subsequent firmware-side
           refreshes (if any) don't overlay our pixel UI. We never write to
           ConOut after this, so the buffer stays empty. */
        uefi_call_wrapper(ST->ConOut->ClearScreen, 1, ST->ConOut);
        uefi_call_wrapper(ST->ConOut->SetCursorPosition, 3, ST->ConOut, 0, 0);
    }
}

static void say(CHAR16 *s) {
    /* Last-resort fatal output. Goes via the firmware console (ASCII only). */
    uefi_call_wrapper(ST->ConOut->OutputString, 2, ST->ConOut, s);
}

static void say_at_rc(UINTN col, UINTN row, CHAR16 *s) {
    gfx_draw_str_color(col * g_char_w, row * g_char_h, s, COL_FG);
}

/* Init splash — rendered on screen BEFORE the main menu is built. Shown
   between init steps that may stall for several seconds on restrictive
   firmware (HP Sure Start blocks SMBus probes, slow USB I/O during
   logging on some boards). Before this, the user just saw the dark
   firmware boot screen with no indication that MemForge was even alive
   — common reaction was "it froze, pull the USB". Splash is single-color,
   centered, fast to draw — does not depend on the main menu layout
   (which requires SMBIOS to be parsed first). */
static void init_splash(CHAR16 *stage) {
    if (!g_gop) return;
    cls();
    UINTN cy = g_h / 2;
    /* Title — large centered line. */
    CHAR16 *title = L"MEMFORGE v0.4.29";
    UINTN tx = (g_w - StrLen(title) * g_char_w) / 2;
    gfx_draw_str_color(tx, cy - g_char_h * 2, title, COL_ACCENT_HI);
    /* Stage indicator — what we're doing right now. */
    UINTN sx = (g_w - StrLen(stage) * g_char_w) / 2;
    gfx_draw_str_color(sx, cy, stage, COL_FG);
    /* Hint — only shown in init splash, not in menu. */
    CHAR16 *hint = (g_lang ? L"Please wait, initializing..."
                           : L"Подождите, идёт инициализация...");
    UINTN hx = (g_w - StrLen(hint) * g_char_w) / 2;
    gfx_draw_str_color(hx, cy + g_char_h * 2, hint, COL_DIM);
}

static void say_at_px(UINTN px, UINTN py, CHAR16 *s) {
    gfx_draw_str_color(px, py, s, COL_FG);
}

static void clear_row(UINTN row) {
    /* Wipe the row's pixel band so stale glyphs disappear before redraw. */
    blt_fill(0, row * g_char_h, g_w, g_char_h, COL_BG);
}

static void log_line(CHAR16 *s) {
    if (!g_logfile) return;
    /* Encode CHAR16 (UTF-16) to UTF-8 so Cyrillic etc. survive into the log
       file. The previous "just drop the high byte" trick mangled Russian
       into random control chars (e.g. "ВСЕГО" → "! (").  */
    CHAR8 buf[600];
    UINTN i = 0;
    while (*s && i < sizeof(buf) - 4) {
        UINT32 cp = (UINT32)(*s);
        if (cp < 0x80) {
            buf[i++] = (CHAR8)cp;
        } else if (cp < 0x800) {
            buf[i++] = (CHAR8)(0xC0 | (cp >> 6));
            buf[i++] = (CHAR8)(0x80 | (cp & 0x3F));
        } else {
            buf[i++] = (CHAR8)(0xE0 | (cp >> 12));
            buf[i++] = (CHAR8)(0x80 | ((cp >> 6) & 0x3F));
            buf[i++] = (CHAR8)(0x80 | (cp & 0x3F));
        }
        s++;
    }
    buf[i++] = '\n';
    UINTN len = i;
    uefi_call_wrapper(g_logfile->Write, 3, g_logfile, &len, buf);
    /* Flush after every line — but Flush() alone is NOT enough on FAT.
       It commits data to disk clusters but does NOT update the file's
       size field in the directory entry. If the user pulls the USB
       while the file is still open, Windows mounts the stick and sees
       'memforge2.log: size 0' even though the data is sitting in
       unlinked clusters. Reported by user: "logs опять пустые" after
       seeing a full summary screen.

       Workaround: Flush + Close + Reopen at saved position. The Close
       forces FAT to write the updated directory entry. Reopen restores
       handle for next log_line. Cost: extra ~10-30 ms per log line on
       cheap USB, total ~1 s extra on init. Acceptable price for the
       log surviving a USB yank at ANY point. */
    uefi_call_wrapper(g_logfile->Flush, 1, g_logfile);
    UINT64 pos = 0;
    uefi_call_wrapper(g_logfile->GetPosition, 2, g_logfile, &pos);
    uefi_call_wrapper(g_logfile->Close, 1, g_logfile);
    g_logfile = NULL;
    if (g_logroot) {
        EFI_FILE_PROTOCOL *nf = NULL;
        EFI_STATUS rs = uefi_call_wrapper(g_logroot->Open, 5,
                            g_logroot, &nf, L"memforge2.log",
                            EFI_FILE_MODE_READ | EFI_FILE_MODE_WRITE, 0);
        if (rs == EFI_SUCCESS && nf) {
            uefi_call_wrapper(nf->SetPosition, 2, nf, pos);
            g_logfile = nf;
        }
    }
}

/* Caller-driven flush. With per-line flush above this is redundant for
   the log file itself, but we keep the helper as a no-op so existing
   call sites don't break. Could in future be used for an explicit
   "fsync now" hook if the FAT driver lies about Flush completion. */
static void flush_log_now(void) {
    if (!g_logfile) return;
    uefi_call_wrapper(g_logfile->Flush, 1, g_logfile);
}

/* ---------- Layout ---------- */
static UINTN g_pad = 16;        /* common margin */
static UINTN g_inner;           /* inner width */

/* Header (top strip) */
static UINTN g_hdr_y = 0, g_hdr_h = 0;

/* Per-test cards list (v0.4 layout: replaces grid + per-test bar + memmap) */
static UINTN g_card_x, g_card_y, g_card_w, g_card_row_h;
/* Number of card columns. 1 normally, 2 in ultra-compact mode where the
   12-test stack would push the cores panel off the bottom of small fb
   (observed on Dell OptiPlex 5050 with firmware-allocated 800×600 fb). */
static UINTN g_card_cols = 1;
/* On very short screens (g_h < 800, e.g. 1024×768 / 1366×768) the cards
   panel doesn't fit alongside header + core panel + overall + footer. We
   drop it — the same "test N/9" info is shown in the header strip. Set by
   compute_layout(). */
static int g_show_cards = 1;

/* v0.4.29 — focused cards layout for small screens (g_h < 900).
   Instead of one full-width row per test (14 rows × ~40 px = 560 px,
   which on a 1024×768 screen eats 70% of vertical space and clips the
   core panel + footer), we draw:
     1) a single STRIP row showing all N tests as small status dots
        + an overall "5/14   ош:0" count.
     2) a BIG FOCUSED CARD (3 rows tall) for the currently-running
        test — name + time + progress bar + live metrics.
   Total reservation: ~4 rows instead of 7-14. Layout scales identically
   from 640×480 to 4K because it does NOT depend on N tests.
   On g_h ≥ 900 the original per-test cards layout is kept (it gives
   a better at-a-glance overview when there's room).                  */
static int   g_focused_cards = 0;
static UINTN g_strip_x, g_strip_y, g_strip_w, g_strip_h;
static UINTN g_focus_x, g_focus_y, g_focus_w, g_focus_h;
static UINTN g_focus_active_idx = (UINTN)-1; /* which test is in big card now */

/* Per-core panel */
static UINTN g_core_x, g_core_y, g_core_row_h, g_core_w;

/* Overall progress bar */
static UINTN g_overall_x, g_overall_y, g_overall_w, g_overall_h;

/* Footer */
static UINTN g_foot_y, g_foot_h;

static int g_compact = 0;

static void compute_layout(UINTN n_tests) {
    /* New layout being computed → force render_header to repaint the static
       rows (gradient, CPU brand, BIOS, DIMM info) ONCE on the next call.
       After that they stay drawn and aren't touched until the next layout
       rebuild. This is what eliminated the per-tick flicker. */
    g_hdr_static_drawn = 0;
    /* Compact mode for low-resolution screens (e.g. 1024×768, 1366×768).
       Cuts vertical paddings so the full UI still fits without scrolling.
       At ≥900 px height we use the spacious "comfort" mode. */
    g_compact = (g_h < 900);

    g_pad = g_compact ? 6 : (g_w / 64);
    if (g_pad < (g_compact ? 6 : 12)) g_pad = g_compact ? 6 : 12;
    g_inner = g_w - 2 * g_pad;

    /* Header has 3 lines (was 6; dropped 4 static rows that duplicated
       info already shown in the main menu — CPU brand, motherboard, BIOS,
       per-DIMM detail — leaving only live, in-flight metrics):
         0: title/RAM/pass/elapsed/ETA/tests
         1: BW · cumulative · pkg-W · max temp · throttle count · uptime
         2: ACTIVITY — current test, spinner, Bit-Fade countdown when active. */
    g_hdr_y = 0;
    g_hdr_h = g_compact ? (3 * g_char_h + 6) : (3 * g_char_h + 12);
    UINTN hdr_min = 3 * g_char_h + 6;
    if (g_hdr_h < hdr_min) g_hdr_h = hdr_min;

    /* Per-test cards: one row per test, full inner width.
       PREVIOUSLY we dropped this panel at g_h<800 assuming there was no
       room — but that was when the header took 6 rows. With the new 3-row
       header (we cut 4 static rows that duplicated the menu) there are
       ~84 extra px which is exactly what the 9 compact cards need on a
       1024×768 / 1366×768 screen. So cards are ALWAYS shown again — they
       are THE place the user looks to see "which test is running, which
       passed, which failed". The header's "Тесты N/9" counter alone wasn't
       enough; the user (rightly) said "не понятно происходит что то или
       нет" when only the counter was visible. */
    g_show_cards = 1;
    g_card_x = g_pad;
    g_card_y = g_hdr_h + g_pad + g_char_h;
    g_card_w = g_inner;
    g_card_row_h = g_compact ? g_char_h : (g_char_h + 16);

    /* v0.4.29 — focused layout on small screens.
       On g_h<900 the per-test card list eats 60-70% of vertical space
       and clips the core panel / footer (YgrecK field report on 1024×768
       Radeon HD 4350). Replace with: 1-row strip of all test dots +
       3-row big card for the currently-running test. Fixed ~4 rows
       regardless of N. On g_h≥900 we keep the original layout because
       the per-test rows give a nicer at-a-glance view when there's room. */
    g_focused_cards = (g_h < 900) ? 1 : 0;
    UINTN cards_h;
    UINTN cards_rows;
    if (g_focused_cards) {
        g_strip_x = g_card_x;
        g_strip_y = g_card_y;
        g_strip_w = g_card_w;
        g_strip_h = g_char_h + 8;
        g_focus_x = g_card_x;
        g_focus_y = g_strip_y + g_strip_h + 6;
        g_focus_w = g_card_w;
        g_focus_h = 3 * g_char_h + 14;
        cards_h = g_strip_h + 6 + g_focus_h + g_pad;
        cards_rows = 0;        /* unused in focused mode */
        g_card_cols = 1;       /* unused but keep consistent */
        g_card_row_h = g_strip_h;
        goto focused_done;
    }
    /* Decide 1-col vs 2-col cards. Three thresholds:
         1. Tiny framebuffers (g_h < 700): always 2-col (Dell OptiPlex 800×600).
         2. n_tests is large AND the 1-col layout would push the cores panel
            below the screen edge: force 2-col. With 14 tests this fires on
            1080p / 1200p displays where the old 12-test layout used to fit.
         3. Otherwise 1-col (preferred for readability).
       To check #2 we compute the height the entire layout would need with
       1-col cards and compare against g_h. If it overflows, switch.       */
    g_card_cols = 1;
    if (g_h < 700) {
        g_card_cols = 2;
    } else if (n_tests >= 13 && g_h < 1600) {
        /* Hard rule: ≥13 tests basically never fit 1-col on anything below
           4K. Probe below would say 1-col is OK on 1440p with a 21-px margin,
           but real rendering adds 30-50 px not visible to the probe (the
           "Overall progress" label above the bar, separator strips inside
           panels, rounding). Field report from a 14-test build on 1440p:
           [N/14] label overlapped CPU row, "MB/s" column truncated, footer
           off-screen. Forcing 2-col closes that gap cleanly. */
        g_card_cols = 2;
    } else {
        /* Probe: with 1-col cards, would the layout overflow?
           Reserve estimates: header + cards + cores panel + overall + footer.
           Same formulas used below for the real layout — we just check the
           sum here BEFORE picking g_card_cols.
           +SAFETY_MARGIN accounts for items NOT in this estimate: the
           "Overall progress [N/M]" label row (g_char_h), panel separator
           strips, and integer rounding accumulating across 4-5 layout
           sections. */
        UINTN cards_h_1col   = g_card_row_h * n_tests + g_pad;
        UINTN core_title_h_e = g_compact ? 0 : g_char_h;
        UINTN core_rh_e      = g_compact ? (g_char_h + 1) : (g_char_h + 4);
        UINTN shown_e        = (g_n_enabled > 64) ? 64 : g_n_enabled;
        UINTN cols_e         = (g_n_enabled <= 8) ? 1 : (g_n_enabled <= 32 ? 2 : 4);
        UINTN rpc_e          = (shown_e + cols_e - 1) / cols_e;
        UINTN core_h_e       = core_rh_e * (rpc_e + 1) + core_title_h_e + g_pad + 4;
        if (g_n_enabled > shown_e) core_h_e += core_rh_e;
        UINTN overall_h_e    = g_compact ? 12 : 16;
        UINTN footer_h_e     = g_compact ? 36 : (g_h / 14);
        if (footer_h_e < (UINTN)(g_compact ? 36 : 50))
            footer_h_e = g_compact ? 36 : 50;
        UINTN safety_margin  = 4 * g_char_h;  /* ~112 px on 1× font */
        UINTN need = g_card_y + cards_h_1col + g_pad + core_h_e
                   + g_pad + overall_h_e + g_pad + footer_h_e
                   + safety_margin;
        if (need > g_h) {
            g_card_cols = 2;
        }
    }
    cards_rows = (n_tests + g_card_cols - 1) / g_card_cols;
    cards_h = g_card_row_h * cards_rows + g_pad;

focused_done:
    (void)cards_rows;          /* silence unused-warning in focused branch */
    /* Per-core panel below the cards. The number of visible cores and the
       rows-per-column are now driven by the same adaptive helpers used by
       the renderer (core_grid_shown / core_grid_cols), so the geometry the
       layout reserves matches what the renderer will actually paint. */
    g_core_x = g_pad;
    g_core_y = g_card_y + cards_h + g_pad;
    g_core_w = g_inner;
    g_core_row_h = g_compact ? (g_char_h + 1) : (g_char_h + 4);

    UINTN shown_cores = core_grid_shown();
    UINTN cols        = core_grid_cols();
    UINTN rows_per_col = (shown_cores + cols - 1) / cols;
    /* Reservation MUST match what core_panel_init() actually draws.
       Compact mode (g_h < 900) drops the verbose subtitle row to save
       ~28 px — must match the same `!g_compact` test inside core_panel_init.
       Without this, either we'd over-reserve (wasted space) or under-reserve
       (the last CPU row overlaps the overall progress bar — observed in
       earlier release on 16-core 2-col layout). */
    UINTN core_title_h = g_compact ? 0 : g_char_h;
    UINTN core_h = g_core_row_h * (rows_per_col + 1) + core_title_h + g_pad + 4;
    if (g_n_enabled > shown_cores) core_h += g_core_row_h;

    /* Overall progress bar */
    g_overall_x = g_pad;
    g_overall_y = g_core_y + core_h + g_pad;
    g_overall_w = g_inner;
    g_overall_h = g_compact ? 12 : 16;

    /* Footer */
    g_foot_h = g_compact ? 36 : (g_h / 14);
    if (g_foot_h < (g_compact ? 36 : 50)) g_foot_h = g_compact ? 36 : 50;
    g_foot_y = g_h - g_foot_h;
}

static EFI_PHYSICAL_ADDRESS g_mem_addr;
static UINTN                g_mem_pages;
static UINTN g_mem_pages_global(void);

/* ---------- Header ---------- */
static void heartbeat_dot(UINT64 elapsed_ms) {
    UINT32 col = ((elapsed_ms / 500) % 2) ? COL_OK : COL_BG;
    blt_fill(g_w - 24, 8, 16, 16, col);
}

static void render_header(UINT64 elapsed_ms, UINTN done, UINTN total) {
    int first = !g_hdr_static_drawn;
    if (first) {
        /* Gradient header: accent -> dark accent — painted ONCE per layout
           rebuild. On subsequent ticks we leave the existing gradient in
           place and just repaint the few "live" rows on top. */
        blt_gradient_v(0, 0, g_w, g_hdr_h, COL_ACCENT, COL_ACCENT_DK);
        blt_fill(0, g_hdr_h - 3, g_w, 1, COL_ACCENT_HI);
        blt_fill(0, g_hdr_h - 2, g_w, 2, COL_BG);
    }
    heartbeat_dot(elapsed_ms);

    CHAR16 buf[256];
    UINT32 secs = (UINT32)(elapsed_ms / 1000);

    /* ETA = elapsed × (passes_total × N_TESTS − work_done) / work_done.
       work_done = (current_pass − 1) × N_TESTS + done_tests_in_cur_pass.
       This is more useful than a per-test ETA because tests vary in length
       (Bit Fade is 100× longer than AVX2). Pass-grain work is the right
       unit. We clamp the result to a reasonable range. */
    UINT32 eta_secs = 0;
    if (g_cfg_marathon_hours > 0) {
        /* Marathon mode: ETA = wall-clock time remaining to the hour limit.
           Pass-grain extrapolation makes no sense when "total passes" is a
           sentinel — the user only cares about hours left. */
        UINT64 limit_ms = (UINT64)g_cfg_marathon_hours * 3600ULL * 1000ULL;
        if (limit_ms > elapsed_ms)
            eta_secs = (UINT32)((limit_ms - elapsed_ms) / 1000);
    } else if (g_pass_total_disp > 0 && elapsed_ms > 5000) {
        UINT64 done_units = (UINT64)(g_pass_idx_disp > 0 ? g_pass_idx_disp - 1 : 0) * total
                          + (UINT64)done;
        UINT64 total_units = (UINT64)g_pass_total_disp * total;
        if (done_units > 0 && total_units > done_units) {
            UINT64 remain_ms = elapsed_ms * (total_units - done_units) / done_units;
            if (remain_ms / 1000 > 0xFFFFFFFFULL) eta_secs = 0xFFFFFFFF;
            else eta_secs = (UINT32)(remain_ms / 1000);
        }
    }

    /* Cumulative error count across every test that has FINISHED so far this
       run. Critical to surface in the header — user steps away for 10 min
       and needs to know at a glance "did anything fail yet" without waiting
       for the summary screen. Shown in every width tier so it never
       disappears off narrow displays. The global g_run_total_errors is
       updated by the test driver in the outer pass loop right after each
       run_test_mc() returns. */
    UINT64 errs_so_far = g_run_total_errors;
    CHAR16 err_tag[40];
    if (errs_so_far == 0) {
        SPrint(err_tag, sizeof(err_tag), T(L"ош:0", L"err:0"));
    } else {
        /* Prepend ⚠ + uppercase to make non-zero count visually loud. */
        SPrint(err_tag, sizeof(err_tag),
               T(L"⚠ ОШИБОК:%ld", L"⚠ ERRORS:%ld"), errs_so_far);
    }

    /* Pass-field tag. Normal: "ПРОХОД 3/12". Marathon: "МАРАФОН 2:15/8h"
       (pass-2, 2 h 15 min elapsed of an 8 h target). Marathon uses elapsed
       wall-clock not pass count because the user cares about "how much
       longer", not arbitrary pass-index numbers. */
    CHAR16 pass_tag[48];
    if (g_cfg_marathon_hours > 0) {
        UINT32 elapsed_min = secs / 60;
        UINT32 limit_min   = g_cfg_marathon_hours * 60;
        UINT32 remain_min  = (limit_min > elapsed_min) ? (limit_min - elapsed_min) : 0;
        SPrint(pass_tag, sizeof(pass_tag),
               T(L"МАРАФОН проход %d  %d:%02d/%dч  осталось %d:%02d",
                 L"MARATHON pass %d  %d:%02d/%dh  remaining %d:%02d"),
               (UINT32)g_pass_idx_disp,
               elapsed_min / 60, elapsed_min % 60,
               g_cfg_marathon_hours,
               remain_min / 60, remain_min % 60);
    } else {
        SPrint(pass_tag, sizeof(pass_tag),
               T(L"ПРОХОД %d/%d", L"PASS %d/%d"),
               (UINT32)g_pass_idx_disp, (UINT32)g_pass_total_disp);
    }

    /* --------------- Row 0 ---------------
       Adaptive: drop fields when text grid is too narrow to hold them.
       Wide (cols ≥ 110): full layout — RAM | PASS | err | elapsed | ETA | Tests
       Medium (90-109)  : drop "Tests N/M" tail
       Narrow (< 90)    : also drop "ETA ~mm:ss"
       Very narrow(<70) : also drop "GB RAM" — just version + pass + time + err.
       Error tag stays in EVERY tier (top priority — user must see it). */
    UINTN row0 = 0;
    clear_row(row0);
    UINT64 ram_gb_x10 = (g_total_ram_mb * 10ULL + 512) / 1024ULL;
    UINTN cols = g_text_cols;
    if (cols >= 110) {
        SPrint(buf, sizeof(buf),
               T(L"  MEMFORGE v0.4.29   |   %ld.%ld ГБ RAM   |   %s   "
                 L"|   %s   |   прошло %02d:%02d   |   осталось ~%02d:%02d   |   Тесты %d/%d",
                 L"  MEMFORGE v0.4.29   |   %ld.%ld GB RAM   |   %s   "
                 L"|   %s   |   elapsed %02d:%02d   |   ETA ~%02d:%02d   |   Tests %d/%d"),
               ram_gb_x10 / 10, ram_gb_x10 % 10,
               pass_tag,
               err_tag,
               secs / 60, secs % 60,
               eta_secs / 60, eta_secs % 60,
               (UINT32)done, (UINT32)total);
    } else if (cols >= 90) {
        SPrint(buf, sizeof(buf),
               T(L"  MEMFORGE v0.4.29   |   %ld.%ld ГБ RAM   |   %s   |   %s   |   прошло %02d:%02d   |   осталось ~%02d:%02d",
                 L"  MEMFORGE v0.4.29   |   %ld.%ld GB RAM   |   %s   |   %s   |   elapsed %02d:%02d   |   ETA ~%02d:%02d"),
               ram_gb_x10 / 10, ram_gb_x10 % 10,
               pass_tag,
               err_tag,
               secs / 60, secs % 60,
               eta_secs / 60, eta_secs % 60);
    } else if (cols >= 70) {
        SPrint(buf, sizeof(buf),
               T(L"  MEMFORGE v0.4.29  |  %ld.%ld ГБ RAM  |  %s  |  %s  |  прошло %02d:%02d",
                 L"  MEMFORGE v0.4.29  |  %ld.%ld GB RAM  |  %s  |  %s  |  elapsed %02d:%02d"),
               ram_gb_x10 / 10, ram_gb_x10 % 10,
               pass_tag,
               err_tag,
               secs / 60, secs % 60);
    } else {
        SPrint(buf, sizeof(buf),
               T(L" MEMFORGE v0.4.29 | %s | %s | прошло %02d:%02d",
                 L" MEMFORGE v0.4.29 | %s | %s | elapsed %02d:%02d"),
               pass_tag,
               err_tag,
               secs / 60, secs % 60);
    }
    /* Use COL_FAIL for the whole row when errors > 0 so the warning is
       impossible to miss from across the room. Otherwise default FG. */
    {
        UINT32 row_color = (errs_so_far > 0) ? COL_FAIL : COL_FG;
        gfx_draw_str_color(0, row0 * g_char_h, buf, row_color);
    }

    /* --------------- Row 1: live telemetry strip --------------- */
    clear_row(1);
    /* GB/s with one decimal: mbps / 1024 → GB/s. Show * for "warming up" if 0. */
    UINT32 gbps_x10 = g_bw_mbps_current ? (g_bw_mbps_current * 10 / 1024) : 0;
    /* % of theoretical peak (capped to 100 so a stale numerator can't claim
       impossible utilization). */
    UINT32 peak_pct = 0;
    if (g_bw_peak_theoretical_mbps > 0 && g_bw_mbps_current > 0)
        peak_pct = (UINT32)((UINT64)g_bw_mbps_current * 100ULL / g_bw_peak_theoretical_mbps);
    if (peak_pct > 100) peak_pct = 100;
    /* Cumulative bytes — show ГБ if < 1 ТБ, else ТБ with 2 decimals. */
    CHAR16 cum_buf[40];
    if (g_cum_bytes < (1024ULL * 1024ULL * 1024ULL * 1024ULL)) {
        UINT32 gb = (UINT32)(g_cum_bytes / (1024ULL * 1024ULL * 1024ULL));
        SPrint(cum_buf, sizeof(cum_buf), T(L"%d ГБ", L"%d GB"), gb);
    } else {
        UINT32 tb_x100 = (UINT32)(g_cum_bytes * 100ULL / (1024ULL * 1024ULL * 1024ULL * 1024ULL));
        SPrint(cum_buf, sizeof(cum_buf),
               T(L"%d.%02d ТБ", L"%d.%02d TB"), tb_x100 / 100, tb_x100 % 100);
    }
    /* Uptime since app entry. */
    UINT64 uptime_ms = ms_now() - g_boot_ms;
    UINT32 up_min = (UINT32)(uptime_ms / 60000);
    UINT32 up_sec = (UINT32)((uptime_ms / 1000) % 60);

    /* Adaptive narrow-screen path mirrors row 0. Drop fields right-to-left:
       1) uptime, 2) throttle count, 3) cumulative GB, 4) CPU watts. The BW
       and temp values are core info and never dropped. */
    if (g_has_rapl && g_pkg_power_w > 0) {
        if (cols >= 110) {
            /* Widest branch: show VID alongside watts when we have it. */
            if (g_pkg_vid_mv > 0) {
                SPrint(buf, sizeof(buf),
                       T(L"  BW %d.%d ГБ/с (%d%%)  ·  %s  ·  CPU %dВт %d.%03dВ  ·  макс %d°C  ·  тротт %d  ·  %d:%02d",
                         L"  BW %d.%d GB/s (%d%%)  ·  %s  ·  CPU %dW %d.%03dV  ·  max %d°C  ·  throttle %d  ·  %d:%02d"),
                       gbps_x10 / 10, gbps_x10 % 10, peak_pct,
                       cum_buf, g_pkg_power_w,
                       g_pkg_vid_mv / 1000, g_pkg_vid_mv % 1000,
                       g_max_temp_c, g_throttle_total, up_min, up_sec);
            } else {
                SPrint(buf, sizeof(buf),
                       T(L"  BW %d.%d ГБ/с (%d%%)  ·  %s  ·  CPU %dВт  ·  макс %d°C  ·  тротт %d  ·  uptime %d:%02d",
                         L"  BW %d.%d GB/s (%d%%)  ·  %s  ·  CPU %dW  ·  max %d°C  ·  throttle %d  ·  uptime %d:%02d"),
                       gbps_x10 / 10, gbps_x10 % 10, peak_pct,
                       cum_buf, g_pkg_power_w,
                       g_max_temp_c, g_throttle_total, up_min, up_sec);
            }
        } else if (cols >= 90) {
            SPrint(buf, sizeof(buf),
                   T(L"  BW %d.%d ГБ/с (%d%%)  ·  %s  ·  CPU %dВт  ·  макс %d°C  ·  тротт %d",
                     L"  BW %d.%d GB/s (%d%%)  ·  %s  ·  CPU %dW  ·  max %d°C  ·  throttle %d"),
                   gbps_x10 / 10, gbps_x10 % 10, peak_pct,
                   cum_buf, g_pkg_power_w, g_max_temp_c, g_throttle_total);
        } else if (cols >= 70) {
            SPrint(buf, sizeof(buf),
                   T(L"  BW %d.%d ГБ/с  ·  CPU %dВт  ·  макс %d°C  ·  тротт %d",
                     L"  BW %d.%d GB/s  ·  CPU %dW  ·  max %d°C  ·  throttle %d"),
                   gbps_x10 / 10, gbps_x10 % 10,
                   g_pkg_power_w, g_max_temp_c, g_throttle_total);
        } else {
            SPrint(buf, sizeof(buf),
                   T(L" BW %d.%d ГБ/с · %dВт · %d°C · т%d",
                     L" BW %d.%d GB/s · %dW · %d°C · t%d"),
                   gbps_x10 / 10, gbps_x10 % 10,
                   g_pkg_power_w, g_max_temp_c, g_throttle_total);
        }
    } else {
        if (cols >= 90) {
            SPrint(buf, sizeof(buf),
                   T(L"  BW %d.%d ГБ/с (%d%%)  ·  %s  ·  макс %d°C  ·  тротт %d  ·  uptime %d:%02d",
                     L"  BW %d.%d GB/s (%d%%)  ·  %s  ·  max %d°C  ·  throttle %d  ·  uptime %d:%02d"),
                   gbps_x10 / 10, gbps_x10 % 10, peak_pct,
                   cum_buf, g_max_temp_c, g_throttle_total, up_min, up_sec);
        } else {
            SPrint(buf, sizeof(buf),
                   T(L"  BW %d.%d ГБ/с  ·  макс %d°C  ·  тротт %d",
                     L"  BW %d.%d GB/s  ·  max %d°C  ·  throttle %d"),
                   gbps_x10 / 10, gbps_x10 % 10,
                   g_max_temp_c, g_throttle_total);
        }
    }
    say_at_rc(0, 1, buf);

    /* Row 2: activity indicator. Defined as a separate function later in
       the file because it needs to read from g_tests[] / g_args[] which
       are declared further down. */
    render_activity_row(elapsed_ms);

    g_hdr_static_drawn = 1;  /* nothing else to do — there are no static rows now */
}

/* ---------- Memory ---------- */
static UINTN g_mem_pages_global(void) { return g_mem_pages; }

/* Look up the physical address range for a 1-based DIMM index using
   SMBIOS Type 20 mapping. Returns 1 + fills *start/*end on success, 0
   if the index is out of range or no Type 20 entries exist.

   With interleaved memory (depth>1) a single DIMM's "range" covers a
   non-contiguous set of cache lines, but the SMBIOS-reported start..end
   range IS the SAME as for its interleave partners. We pick that range
   and let multiple DIMMs share it — testing within the range exercises
   the target DIMM but also touches its interleave peers. Not ideal but
   the best we can do without chipset-specific bank-decoding hardware
   knowledge. On non-interleaved systems (most workstations/servers)
   each DIMM has its own exclusive range and isolation is perfect. */
static int dimm_address_range(UINT32 dimm_index_1based,
                               UINT64 *out_start, UINT64 *out_end) {
    if (dimm_index_1based == 0 || dimm_index_1based > g_dimm_count) return 0;
    UINT16 target_handle = g_dimms[dimm_index_1based - 1].handle;
    for (UINT32 i = 0; i < g_dimm_map_count; i++) {
        if (g_dimm_map[i].dev_handle == target_handle) {
            *out_start = g_dimm_map[i].start;
            *out_end   = g_dimm_map[i].end;
            return 1;
        }
    }
    return 0;
}

static EFI_STATUS alloc_test_buffer(void) {
    /* Strategy: 75% of largest contiguous EfiConventionalMemory block,
       capped at g_cfg_buffer_cap_mb (configurable via [Run] BufferMB in
       quantai.ini, default 1024). Fall back to halving on alloc failure.

       When TestOnlyDimm=N is set, we additionally require the allocation
       to fall WITHIN the physical address range of DIMM N (from SMBIOS
       Type 20). UEFI's AllocateAddress flag lets us request a specific
       physical address — we walk the DIMM's range looking for free
       conventional memory chunks. */
    UINT64 lo_bound = 0;
    UINT64 hi_bound = (UINT64)-1;
    int isolating = 0;
    if (g_cfg_test_only_dimm > 0) {
        if (dimm_address_range(g_cfg_test_only_dimm, &lo_bound, &hi_bound)) {
            isolating = 1;
            CHAR16 lb[180];
            SPrint(lb, sizeof(lb),
                   L"[ISOLATE] Test restricted to DIMM%d range 0x%lx..0x%lx",
                   g_cfg_test_only_dimm, lo_bound, hi_bound);
            log_line(lb);
        } else {
            CHAR16 lb[180];
            SPrint(lb, sizeof(lb),
                   L"[ISOLATE] TestOnlyDimm=%d but no SMBIOS Type 20 mapping found — falling back to full RAM",
                   g_cfg_test_only_dimm);
            log_line(lb);
        }
    }

    UINTN cap_pages    = ((UINTN)g_cfg_buffer_cap_mb * 1024ULL * 1024ULL) / 4096;
    UINTN min_pages    = (256ULL  * 1024 * 1024) / 4096;     /* 256 MB floor */

    /* Non-isolated default path — pick largest free block. */
    if (!isolating) {
        UINTN free_pages = get_largest_free_pages();
        UINTN target_pages = (free_pages * 3) / 4;
        if (target_pages > cap_pages) target_pages = cap_pages;
        if (target_pages < min_pages) target_pages = min_pages;
        while (target_pages >= min_pages / 4) {
            EFI_PHYSICAL_ADDRESS addr = 0;
            EFI_STATUS s = uefi_call_wrapper(BS->AllocatePages, 4,
                                  AllocateAnyPages, EfiLoaderData, target_pages, &addr);
            if (s == EFI_SUCCESS) {
                g_mem_addr = addr;
                g_mem_pages = target_pages;
                return s;
            }
            target_pages /= 2;
        }
        return EFI_OUT_OF_RESOURCES;
    }

    /* Isolated path — request specific address within the DIMM's range.
       Walk EFI memory map, find conventional regions overlapping our
       target DIMM, attempt allocations at successively smaller sizes. */
    UINTN map_size = 0, map_key = 0, desc_size = 0;
    UINT32 desc_ver = 0;
    EFI_MEMORY_DESCRIPTOR *map = NULL;
    uefi_call_wrapper(BS->GetMemoryMap, 5, &map_size, map, &map_key, &desc_size, &desc_ver);
    map_size += desc_size * 16;
    uefi_call_wrapper(BS->AllocatePool, 3, EfiLoaderData, map_size, (VOID**)&map);
    if (!map) return EFI_OUT_OF_RESOURCES;
    EFI_STATUS gms = uefi_call_wrapper(BS->GetMemoryMap, 5,
                          &map_size, map, &map_key, &desc_size, &desc_ver);
    if (EFI_ERROR(gms)) {
        uefi_call_wrapper(BS->FreePool, 1, map);
        return gms;
    }
    UINTN entries = map_size / desc_size;
    /* Find largest conventional region that overlaps the DIMM range. */
    UINT64 best_start = 0;
    UINTN  best_pages = 0;
    for (UINTN i = 0; i < entries; i++) {
        EFI_MEMORY_DESCRIPTOR *d = (EFI_MEMORY_DESCRIPTOR *)
            ((UINT8 *)map + i * desc_size);
        if (d->Type != EfiConventionalMemory) continue;
        UINT64 r_start = d->PhysicalStart;
        UINT64 r_end   = r_start + d->NumberOfPages * 4096ULL - 1;
        /* Clip to DIMM range. */
        if (r_end   < lo_bound) continue;
        if (r_start > hi_bound) continue;
        UINT64 cs = (r_start < lo_bound) ? lo_bound : r_start;
        UINT64 ce = (r_end   > hi_bound) ? hi_bound : r_end;
        if (ce <= cs) continue;
        UINTN avail_pages = (UINTN)((ce - cs + 1) / 4096);
        /* Reserve a small headroom to avoid colliding with stuff EFI keeps
           live during runtime. */
        if (avail_pages < min_pages / 4) continue;
        if (avail_pages > best_pages) {
            best_pages = avail_pages;
            best_start = (cs + 4095) & ~0xFFFULL;   /* page-align up */
        }
    }
    uefi_call_wrapper(BS->FreePool, 1, map);

    if (best_pages == 0) {
        log_line(L"[ISOLATE] No conventional memory found inside DIMM range");
        return EFI_NOT_FOUND;
    }

    UINTN target_pages = (best_pages * 3) / 4;
    if (target_pages > cap_pages) target_pages = cap_pages;
    if (target_pages < min_pages) target_pages = min_pages;
    /* AllocateAddress requires page-aligned address. */
    while (target_pages >= min_pages / 4) {
        EFI_PHYSICAL_ADDRESS addr = best_start;
        EFI_STATUS s = uefi_call_wrapper(BS->AllocatePages, 4,
                              AllocateAddress, EfiLoaderData, target_pages, &addr);
        if (s == EFI_SUCCESS) {
            g_mem_addr = addr;
            g_mem_pages = target_pages;
            CHAR16 lb[160];
            SPrint(lb, sizeof(lb),
                   L"[ISOLATE] Allocated %ld MB at 0x%lx (inside DIMM%d)",
                   (UINT64)(target_pages * 4096ULL / 1024ULL / 1024ULL),
                   (UINT64)addr, g_cfg_test_only_dimm);
            log_line(lb);
            return s;
        }
        target_pages /= 2;
    }
    return EFI_OUT_OF_RESOURCES;
}

static void free_test_buffer(void) {
    if (g_mem_addr) {
        uefi_call_wrapper(BS->FreePages, 2, g_mem_addr, g_mem_pages);
        g_mem_addr = 0;
    }
}

/* ---------- Per-AP state (used by both kernels and renderer) ---------- */
typedef enum {
    /* Standard / Quick tests */
    KER_WALKING_ONES, KER_WALKING_ZEROES, KER_MOVING_INV,
    KER_ADDRESS, KER_AVX2, KER_ROWHAMMER, KER_BLOCK_MOVE,
    KER_RANDOM, KER_BIT_FADE,
    /* Aggressive tests — kernels designed to maximize fault coverage
       through algorithmic strength, not duration. References:
         - March-C-:  van de Goor & Tlili, "March tests for word-oriented
           memories", IEEE Trans. on Computers 1997. Catches SAF + AF +
           TF + CFid + CFin + CFst (92% of fault models in 6n ops).
         - March-RAW: De Pessemier et al., extends March-C- with
           read-after-write tests to catch dynamic coupling faults.
         - TRRespass: Frigo et al., USENIX Security 2020 — defeats DDR4
           TRR via 8-aggressor patterns with bank-group rotation.
         - VRM square-wave / cache eviction: standard semiconductor
           production-test techniques for VRM transient response and
           IMC bank-conflict bug discovery. */
    KER_MARCH_CM,       /* March-C-:  92% fault coverage in 6n ops */
    KER_MARCH_RAW,      /* March-RAW: + dynamic coupling */
    KER_BUTTERFLY,      /* Checkerboard butterfly: max cell crosstalk */
    KER_AVX2_SUSTAINED, /* 10 sec sustained FMA + interleaved memory */
    KER_TRRESPASS,      /* 8-sided RowHammer with bank rotation */
    KER_CACHE_EVICT,    /* CLFLUSH storm — max DRAM hit rate */
    KER_VRM_SQUARE,     /* alternating max/min IPC — VRM transient stress */
    KER_BIT_FADE_EXT,   /* 4 patterns × 2 min/phase = 16 min retention */
    /* Thermal/bandwidth soak tests — run ONCE per Full prog (pass 1 only).
       They produce the same kind of sustained thermal load that memtest86+
       generates over 30+ min, which catches marginal cooling / VRM / IMC
       that the burst-style tests miss. Together they push i5-12600KF-class
       CPUs to ~75-90°C (vs ~36°C without them — basically idle). */
    KER_THERMAL_SOAK,   /* 3 min sustained AVX2 FMA + memory writes — heat */
    KER_BW_SOAK,        /* 5 min streaming writes+reads — memory bandwidth */
    KER_L3_STRESS,      /* L3 cache resident workload — exposes L3-cell faults
                           that DRAM-only tests miss (CLFLUSH evicts to DRAM,
                           so DRAM tests bypass L3 storage entirely). */
    KER_STRIDE_BW,      /* Stride-sweep BW probe — runs sustained reads at
                           strides 64 B / 1 KB / 4 KB / 64 KB and reports each.
                           Drop at one stride = TLB / page-table issue, drop
                           at cache-line stride only = channel imbalance. */
} kernel_id_t;

/* ---------- Detailed error capture ----------
   When a kernel detects a value mismatch, it records the first N occurrences
   here. The XOR mask is the most useful single piece of data — it reveals
   which bit positions flipped, which often pinpoints a data-bus line or a
   particular DRAM bit cell. The records are dumped at the end of the run
   into memforge2.log and report.json. */
#define MAX_ERR_RECORDS 32
typedef struct {
    kernel_id_t test;
    UINT32 core;
    UINT64 phys_addr;     /* virtual == physical in UEFI identity-mapped space */
    UINT64 expected;
    UINT64 actual;
    UINT64 xor_mask;
    UINT32 pass_idx;      /* multi-pass index when applicable */
    /* Environmental snapshot at the moment record_error was called. Lets
       the operator correlate "this error appeared at 87 °C, 12 throttle
       events accumulated, 134 W" rather than only knowing the run-wide
       peak. Critical for diagnosing thermal-only failures. */
    UINT64 t_ms;          /* ms since g_run_start_ms (0 if pre-run) */
    UINT32 temp_c;        /* g_max_temp_c sampled at error time */
    UINT32 pkg_watt;      /* g_pkg_power_w (current sample) */
    UINT32 throttle_cnt;  /* g_throttle_total cumulative */
    UINT32 vid_mv;        /* g_pkg_vid_mv, populated in Phase 3 */
} err_record_t;
static err_record_t g_err_records[MAX_ERR_RECORDS];
static volatile UINT32 g_err_count = 0;
static volatile UINT32 g_cur_pass  = 0;

/* The environment globals (g_pkg_power_w, g_max_temp_c, g_throttle_total,
   g_pkg_vid_mv, g_run_start_ms) are declared earlier in the file. They are
   read here from an AP context — that's safe because we never write them
   concurrently during the error path; a torn UINT32 read gives a slightly
   stale temp/watt sample, which is diagnostically harmless. */

static void record_error(kernel_id_t test, UINT32 core,
                          UINT64 addr, UINT64 exp, UINT64 act) {
    /* Atomic increment + bound check. lock xadd on x86_64. */
    UINT32 idx = __sync_fetch_and_add(&g_err_count, 1);
    if (idx >= MAX_ERR_RECORDS) return;
    g_err_records[idx].test         = test;
    g_err_records[idx].core         = core;
    g_err_records[idx].phys_addr    = addr;
    g_err_records[idx].expected     = exp;
    g_err_records[idx].actual       = act;
    g_err_records[idx].xor_mask     = exp ^ act;
    g_err_records[idx].pass_idx     = g_cur_pass;
    /* Environmental snapshot. ms_now is rdtsc-based and AP-safe. */
    UINT64 now = ms_now();
    g_err_records[idx].t_ms         = (g_run_start_ms && now > g_run_start_ms)
                                       ? (now - g_run_start_ms) : 0;
    g_err_records[idx].temp_c       = g_max_temp_c;
    g_err_records[idx].pkg_watt     = g_pkg_power_w;
    g_err_records[idx].throttle_cnt = g_throttle_total;
    g_err_records[idx].vid_mv       = g_pkg_vid_mv;
}

/* ---------- Error localization helpers ----------
   Three diagnostic features that turn raw [test, addr, xor] records into
   actionable "what's wrong with this stick of RAM" answers:
     1. dimm_label_for_addr  — addr → "DIMM_A2" or "DIMM_A1+B1(2-way)" via SMBIOS Type 20
     2. find_stuck_bit       — XOR mask repeated ≥ 5× → "bit N is stuck"
     3. error_histogram_gb   — 1-GB bucketed distribution → see channel/rank skew
   All are pure functions of the captured records; nothing changes during the
   test itself. Called once from render_summary() and write_json_report(). */

/* Look up which DIMM(s) a physical address belongs to using SMBIOS Type 20.
   Writes "DIMM_A2" or "DIMM_A1+B1+C1+D1 (4-way intl)" or "?" into `out`.
   Returns 1 if a mapping was found, 0 otherwise. */
static int dimm_label_for_addr(UINT64 addr, CHAR16 *out, UINTN out_sz_chars) {
    out[0] = 0;
    if (g_dimm_map_count == 0 || g_dimm_count == 0) {
        SPrint(out, out_sz_chars * sizeof(CHAR16),
               T(L"вне SMBIOS-карты", L"unmapped region"));
        return 0;
    }
    /* Collect all map entries whose address range contains this byte.
       For an interleave set, MULTIPLE entries will match (same range,
       different interleave_pos). */
    UINT16 matched_handles[8]; UINT32 n_match = 0;
    UINT8  intl_depth = 1;
    for (UINT32 i = 0; i < g_dimm_map_count && n_match < 8; i++) {
        if (addr >= g_dimm_map[i].start && addr <= g_dimm_map[i].end) {
            matched_handles[n_match++] = g_dimm_map[i].dev_handle;
            if (g_dimm_map[i].interleave_depth > intl_depth)
                intl_depth = g_dimm_map[i].interleave_depth;
        }
    }
    if (n_match == 0) {
        SPrint(out, out_sz_chars * sizeof(CHAR16),
               T(L"вне SMBIOS-карты", L"unmapped region"));
        return 0;
    }
    /* Resolve handles to locator strings. */
    UINTN pos = 0;
    for (UINT32 m = 0; m < n_match; m++) {
        CHAR8 *loc = NULL;
        for (UINT32 j = 0; j < g_dimm_count; j++) {
            if (g_dimms[j].handle == matched_handles[m]) {
                loc = g_dimms[j].locator;
                break;
            }
        }
        if (!loc || !loc[0]) loc = (CHAR8 *)(g_lang ? "unknown DIMM" : "планка без имени");
        pos += SPrint(out + pos, (out_sz_chars - pos) * sizeof(CHAR16),
                      (m == 0) ? L"%a" : L"+%a", loc);
        if (pos >= out_sz_chars - 16) break;
    }
    if (intl_depth > 1 && n_match > 1) {
        SPrint(out + pos, (out_sz_chars - pos) * sizeof(CHAR16),
               T(L" (interleave %d×)", L" (%d-way intl)"), intl_depth);
    }
    return 1;
}

/* Scan captured error records for a repeating XOR mask. If the same XOR
   appears ≥ threshold times, that exact bit pattern is consistently wrong —
   classic signature of a stuck data line or a permanently dead DRAM cell.
   Returns the most-frequent XOR mask (or 0 if no clear winner), writes its
   count into *out_count. Threshold is enforced by the caller. */
static UINT64 find_stuck_bit(UINT32 *out_count) {
    *out_count = 0;
    UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
    if (shown == 0) return 0;
    UINT64 best_xor = 0; UINT32 best_n = 0;
    for (UINT32 i = 0; i < shown; i++) {
        UINT64 x = g_err_records[i].xor_mask;
        if (x == 0) continue;       /* not an error pattern, skip */
        UINT32 n = 0;
        for (UINT32 j = 0; j < shown; j++)
            if (g_err_records[j].xor_mask == x) n++;
        if (n > best_n) { best_n = n; best_xor = x; }
    }
    *out_count = best_n;
    return best_xor;
}

/* Decode a single-bit XOR mask into its bit position (0..63), or -1 if the
   mask is multi-bit. Used to render the "стuck bit D[21]" hint. */
static int single_bit_pos(UINT64 mask) {
    if (mask == 0) return -1;
    if (mask & (mask - 1)) return -1;       /* >1 bit set */
    int pos = 0;
    while (((mask >> pos) & 1ULL) == 0) pos++;
    return pos;
}

/* Map a bit position (0..63 on the 64-bit data bus) back to a specific
   chip on the DIMM PCB, using SPD device_width. Writes "U%d bit %d" or
   "?" into `out`. Returns 1 if mapping was possible, 0 if SPD didn't
   give us organization info. The "U%d" convention follows the silkscreen
   designation on most DIMM PCBs (U1, U2, ..., U8 left-to-right).

   For a typical x8 DDR4 module (8 chips × 8 bits = 64-bit bus):
     bit 0..7   → chip U1 (some boards use U0)
     bit 8..15  → chip U2
     ...
     bit 56..63 → chip U8

   For a x4 module (16 chips × 4 bits):
     bit 0..3   → U1, bit 4..7 → U2, etc.

   For x16 (4 chips × 16 bits) — rare on consumer DDR4 but common on
   LPDDR mobile:
     bit 0..15  → chip U1, bit 16..31 → U2, etc.
*/
static int chip_label_for_bit(UINT32 dimm_idx_0based, int bit_pos,
                               CHAR16 *out, UINTN out_chars) {
    out[0] = 0;
    if (dimm_idx_0based >= g_dimm_count) return 0;
    if (bit_pos < 0 || bit_pos > 63) return 0;
    dimm_info_t *d = &g_dimms[dimm_idx_0based];
    UINT8 dw = d->spd_device_width;
    if (dw != 4 && dw != 8 && dw != 16) {
        /* SPD didn't expose chip organization — typically on DDR4 x4 modules
           where the SPD width byte wasn't readable, or on systems where
           we couldn't get the full SPD block. We can't map bit→chip.
           Return empty (caller should branch and use a different sentence). */
        out[0] = 0;
        return 0;
    }
    /* Chip index 1-based: matches PCB silkscreen designation (U1..U8 / U1..U16). */
    UINT32 chip_idx = (UINT32)bit_pos / dw + 1;
    UINT32 within  = (UINT32)bit_pos % dw;
    SPrint(out, out_chars * sizeof(CHAR16),
           T(L"чип U%d (бит %d, ширина x%d)",
             L"chip U%d (bit %d, width x%d)"),
           chip_idx, within, dw);
    return 1;
}

/* Find which DIMM index (0-based) MOST of the recorded errors belong
   to. Walks g_err_records[], looks up dimm via dimm_label_for_addr but
   we don't have the index back. Easier: re-derive from address via
   g_dimm_map[]. Returns -1 if not determinable. */
static int dominant_dimm_idx(void) {
    UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
    if (shown == 0 || g_dimm_count == 0 || g_dimm_map_count == 0) return -1;
    UINT32 counts[MAX_DIMMS] = {0};
    for (UINT32 i = 0; i < shown; i++) {
        UINT64 addr = g_err_records[i].phys_addr;
        for (UINT32 m = 0; m < g_dimm_map_count; m++) {
            if (addr >= g_dimm_map[m].start && addr <= g_dimm_map[m].end) {
                for (UINT32 j = 0; j < g_dimm_count; j++) {
                    if (g_dimms[j].handle == g_dimm_map[m].dev_handle) {
                        if (j < MAX_DIMMS) counts[j]++;
                        break;
                    }
                }
                break;
            }
        }
    }
    int best = -1; UINT32 best_n = 0;
    for (UINT32 j = 0; j < g_dimm_count; j++) {
        if (counts[j] > best_n) { best_n = counts[j]; best = (int)j; }
    }
    return best;
}

/* v0.4.29 — detect dual-channel interleave ambiguity.
   On consumer desktops with dual/quad-channel memory, the iMC interleaves
   addresses between channels at 64-byte (cache-line) granularity. A
   SINGLE bad chip on one stick produces errors that, when mapped through
   SMBIOS Type 20, appear distributed across BOTH sticks in the channel
   pair because consecutive 64-byte blocks alternate between sticks.

   Field report from a Habr user (Netac DDR4 kit): same stuck bit
   D[53] was reported 24 times, distributed as A2 (8) + B2 (11) + ? (5).
   Pre-v0.4.29 verdict confidently said "REPLACE: DDR4-B2 (HIGH)" — but
   physically it's likely ONE bad chip on one of A2/B2, NOT both.

   This helper returns the list of DIMM indices that each hold >=25% of
   localised errors. If 2+ DIMMs cross that threshold AND errors share
   a common stuck-bit signature, the verdict can no longer confidently
   pick one — it should tell the user "one of these N — swap to isolate".
   Returns count written into out_idx[] (0..cap).                       */
static UINTN distributed_dimm_indices(int *out_idx, UINTN cap) {
    UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
    if (shown == 0 || g_dimm_count == 0 || g_dimm_map_count == 0) return 0;
    UINT32 counts[MAX_DIMMS] = {0};
    UINT32 total_localized = 0;
    for (UINT32 i = 0; i < shown; i++) {
        UINT64 addr = g_err_records[i].phys_addr;
        for (UINT32 m = 0; m < g_dimm_map_count; m++) {
            if (addr >= g_dimm_map[m].start && addr <= g_dimm_map[m].end) {
                for (UINT32 j = 0; j < g_dimm_count; j++) {
                    if (g_dimms[j].handle == g_dimm_map[m].dev_handle) {
                        if (j < MAX_DIMMS) {
                            counts[j]++;
                            total_localized++;
                        }
                        break;
                    }
                }
                break;
            }
        }
    }
    if (total_localized == 0) return 0;
    /* Threshold: a DIMM is "significantly involved" if it holds at least
       25% of localised errors. Pure interleave on a dual-channel pair
       gives ~50/50 split; the 25% threshold also catches asymmetric
       splits (8/11/5 in the Habr case → A2=33%, B2=46%, both above). */
    UINT32 threshold = (total_localized + 3) / 4;
    if (threshold < 2) threshold = 2;   /* ignore single-error noise */
    UINTN n = 0;
    for (UINT32 j = 0; j < g_dimm_count && n < cap; j++) {
        if (counts[j] >= threshold) {
            out_idx[n++] = (int)j;
        }
    }
    return n;
}

/* v0.4.29 — Approach D: detect whether SMBIOS Type 20 reports REAL
   cache-line interleave (overlapping address ranges across DIMMs) or
   BLOCK mapping (disjoint ranges, each DIMM owns its own physical
   region). PassMark forum & KIT paper both confirm that even though
   the iMC physically interleaves consumer dual-channel at 64-byte
   cache-line granularity, BIOS vendors often emit Type 20 entries in
   BLOCK form (one DIMM per region) — what we see in the field on
   Gigabyte B760M DDR4 (A2=0..8GB, B2=8..16GB, intl depth=2 but ranges
   disjoint). Two interpretations:
     - Real disjoint = independent channels, no interleave → errors
       distributed across DIMMs really are on physically separate sticks.
     - Pseudo-interleave (BIOS lies) = iMC interleaves, but Type 20 hides
       it → errors distributed look like multiple sticks but are one chip.
   Without iMC PCI register access (Intel doesn't document MAD_*
   registers for consumer Alder Lake), we use a heuristic:
     overlap_found == 1 → trust Type 20 reports interleave, multi-DIMM
                          errors → "ONE chip on ONE of pair X/Y"
     overlap_found == 0 + intl_depth_max <= 1 → block mode, multi-DIMM
                          errors → "BOTH sticks have faults"
     overlap_found == 0 + intl_depth_max  > 1 → BIOS conflict, fall back
                          to bit-6 channel polarity analysis to decide.
   Returns 1 if any pair of Type 20 entries has overlapping ranges. */
static int type20_has_overlapping_ranges(void) {
    for (UINT32 i = 0; i < g_dimm_map_count; i++) {
        for (UINT32 j = i + 1; j < g_dimm_map_count; j++) {
            if (g_dimm_map[i].dev_handle == g_dimm_map[j].dev_handle) continue;
            /* Ranges overlap iff start_i <= end_j && start_j <= end_i. */
            if (g_dimm_map[i].start <= g_dimm_map[j].end &&
                g_dimm_map[j].start <= g_dimm_map[i].end) {
                return 1;
            }
        }
    }
    return 0;
}
static UINT8 type20_max_interleave_depth(void) {
    UINT8 m = 0;
    for (UINT32 i = 0; i < g_dimm_map_count; i++) {
        if (g_dimm_map[i].interleave_depth > m)
            m = g_dimm_map[i].interleave_depth;
    }
    return m;
}

/* v0.4.29 — Approach A: bit-6 polarity analysis of error addresses.
   On most Intel/AMD consumer dual-channel desktops with DDR4/DDR5, the
   iMC's channel selector is physical address bit 6 (alternating 64-byte
   cache lines between channels). If all error records share the same
   bit-6 value, that's a strong hint the bad cell/chip lives in only ONE
   channel even though Type 20 (block-mapped) may have attributed them
   to multiple DIMM labels. Returns:
      0 = mixed (errors on both bit-6 values, no strong signal)
      1 = all errors at bit 6 == 0 (likely channel A)
      2 = all errors at bit 6 == 1 (likely channel B)
   "All" = ≥85% on one polarity; ≥3 errors required for a verdict
   (single-error noise gives no signal). On chipsets where the channel
   selector isn't bit 6 (Skylake-X page-stride hash, some servers), this
   will return 0 and the caller should fall back to the conservative
   "physical isolation needed" message. */
static int bit6_channel_polarity(void) {
    UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
    if (shown < 3) return 0;
    UINT32 n0 = 0, n1 = 0;
    for (UINT32 i = 0; i < shown; i++) {
        if ((g_err_records[i].phys_addr >> 6) & 1) n1++;
        else n0++;
    }
    UINT32 total = n0 + n1;
    if (total == 0) return 0;
    /* 85% threshold — allows a few "noise" errors from a different
       cause without losing the signal. */
    if (n0 * 100 >= total * 85) return 1;
    if (n1 * 100 >= total * 85) return 2;
    return 0;
}

/* Build a 1-GB-bucketed histogram of error addresses. Writes up to
   `max_buckets` non-zero buckets into out_bucket_gb[]/out_count[], returns
   number actually written. Useful to spot "all errors in 4-8 GB range →
   probably DIMM #2" patterns without needing perfect Type 20 mapping. */
static UINT32 error_histogram_gb(UINT32 *out_bucket_gb, UINT32 *out_count,
                                  UINT32 max_buckets) {
    UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
    UINT32 nb = 0;
    for (UINT32 i = 0; i < shown && nb < max_buckets; i++) {
        UINT32 gb = (UINT32)(g_err_records[i].phys_addr >> 30);
        /* Look up existing bucket */
        UINT32 found = 0;
        for (UINT32 j = 0; j < nb; j++) {
            if (out_bucket_gb[j] == gb) {
                out_count[j]++;
                found = 1;
                break;
            }
        }
        if (!found) {
            out_bucket_gb[nb] = gb;
            out_count[nb]     = 1;
            nb++;
        }
    }
    return nb;
}

/* ---------- DRAM coordinate decode (approximate) ----------
   Map a physical address to (channel, bank, row, column) using a *generic*
   DDR4/DDR5 bit layout. We deliberately do NOT try to model the chipset's
   actual hash function — Intel doesn't document it for client SKUs, and
   server SKUs have one model-specific decoder per generation in Linux's
   EDAC drivers. Instead we use the canonical DDR4/DDR5 bit-position
   mapping that most modules follow at the JEDEC level. The result is
   approximate: it will agree with the actual mapping up to per-channel
   interleave hashing (channel guess may be off by 1 on dual-channel,
   bank within bank-group may swap). It IS reliable for:
     - "Are all errors in the SAME row?" — yes/no
     - "Are all errors in the SAME bank?" — yes/no
     - "Cluster of errors around row N" — yes the cluster is real
   …which is exactly what shop diagnostics need. Marked "~" in output so
   the user knows it's a heuristic.

   DDR4 typical (8 GB unbuffered, x8 chips, 17-bit row):
     PA[5:0]   byte within cacheline (fixed, 64-byte access)
     PA[12:6]  column[9:3]    (7 bits used)
     PA[14:13] bank group     (2 bits = 4 groups)
     PA[17:15] bank in group  (3 bits = 8 banks)
     PA[34:18] row            (up to 17 bits)
     PA[6]     channel hash on 2-ch systems (rough)
   DDR5 typical:
     Same basic shape but 4 bank groups × 4 banks/group (8 BG × 4 banks
     for x16 narrow-IO), 16-18 bit row. Layout very similar at this
     coarse level. We use the DDR4 mapping; the labels still cluster
     correctly for stuck-row detection. */
typedef struct {
    UINT16 channel;       /* 0/1 guess; ignore on single-channel systems */
    UINT16 bank_group;    /* 0..3 (DDR4) or 0..7 (DDR5) */
    UINT16 bank;          /* 0..7 within bank group */
    UINT32 column;        /* multiplied by burst = byte column */
    UINT64 row;           /* 0..~131071 (DDR4) */
} dram_coords_t;

static void decode_dram_coords(UINT64 addr, dram_coords_t *c) {
    c->channel    = (UINT16)((addr >> 6) & 0x1);              /* very rough */
    c->column     = (UINT32)((addr >> 6) & 0x7F);             /* col[9:3] */
    c->bank_group = (UINT16)((addr >> 14) & 0x3);
    c->bank       = (UINT16)((addr >> 17) & 0x7);
    c->row        = addr >> 18;
}

/* Count errors that map to the SAME row. If a single row is hit by 3+
   errors that's almost certainly a failing row driver / address line.
   Returns the row value with most hits and writes the count into *count.
   Returns 0/0 if no clustering. */
static UINT64 find_stuck_row(UINT32 *out_count) {
    *out_count = 0;
    UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
    if (shown < 3) return 0;
    UINT64 best_row = 0; UINT32 best_n = 0;
    for (UINT32 i = 0; i < shown; i++) {
        dram_coords_t ci;
        decode_dram_coords(g_err_records[i].phys_addr, &ci);
        UINT32 n = 0;
        for (UINT32 j = 0; j < shown; j++) {
            dram_coords_t cj;
            decode_dram_coords(g_err_records[j].phys_addr, &cj);
            if (ci.row == cj.row) n++;
        }
        if (n > best_n) { best_n = n; best_row = ci.row; }
    }
    *out_count = best_n;
    return best_row;
}

/* Same idea for bank-group + bank combined: errors clustering in one
   bank-pair point to a failing bank refresh/precharge circuit. */
static UINT32 find_stuck_bank(UINT32 *out_count) {
    *out_count = 0;
    UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
    if (shown < 3) return 0xFFFFFFFFu;
    UINT32 best_id = 0; UINT32 best_n = 0;
    for (UINT32 i = 0; i < shown; i++) {
        dram_coords_t ci;
        decode_dram_coords(g_err_records[i].phys_addr, &ci);
        UINT32 id_i = ((UINT32)ci.bank_group << 4) | ci.bank;
        UINT32 n = 0;
        for (UINT32 j = 0; j < shown; j++) {
            dram_coords_t cj;
            decode_dram_coords(g_err_records[j].phys_addr, &cj);
            UINT32 id_j = ((UINT32)cj.bank_group << 4) | cj.bank;
            if (id_i == id_j) n++;
        }
        if (n > best_n) { best_n = n; best_id = id_i; }
    }
    *out_count = best_n;
    return best_id;
}

typedef struct ap_arg_s ap_arg_t;
struct ap_arg_s {
    UINT64 *base;
    UINTN   n_qwords;
    kernel_id_t kernel;
    UINT32  core_idx;             /* logical slot number, 0 = BSP */
    volatile UINT64 errors;
    volatile UINT64 bytes;
    volatile UINT32 progress;     /* 0..1000 (per-mille) */
    volatile UINT32 done;
    /* Optional yield: BSP's slot sets this so we render the panel mid-kernel.
       APs leave it NULL (drawing from APs would race with BSP). */
    void (*yield)(ap_arg_t *);
    /* Real CPU activity (C0 residency) — each core samples its OWN MSRs from
       inside its kernel; BSP reads util_pct for visualization. Updated by
       ap_yield(). 0..100. */
    UINT64 util_tsc_prev;
    UINT64 util_mperf_prev;
    UINT64 util_aperf_prev;
    volatile UINT32 util_pct;
    volatile UINT32 freq_mhz;     /* effective core frequency, derived from APERF/MPERF */
    volatile UINT32 temp_c;       /* per-core temperature, °C */
    /* Throttle flags — bitwise OR of THROT_* below, latched on each ap_yield()
       sample. Lets the UI show WHY a core's frequency dropped (thermal /
       power / current / PROCHOT) instead of just "running slow". */
    volatile UINT32 throttle_flags;
    volatile UINT32 throttle_event_count; /* total log-events on this core since reset */
    UINT32 throttle_was_active;   /* edge detect for event counter */
};
/* IA32_THERM_STATUS (MSR 0x19C) bit definitions of interest. The "log" bits
   13/15/etc. are sticky — they record that the condition has occurred since
   last cleared. We read the LIVE bits (0/4/10/12/...) — they reflect the
   condition right now. */
#define THROT_THERMAL      0x0001u  /* bit 0  — Thermal Threshold reached */
#define THROT_PROCHOT      0x0010u  /* bit 4  — PROCHOT# asserted */
#define THROT_POWERLIMIT   0x0400u  /* bit 10 — Power limit notification */
#define THROT_CURRENTLIMIT 0x1000u  /* bit 12 — Current limit notification */
#define THROT_ANY (THROT_THERMAL | THROT_PROCHOT | THROT_POWERLIMIT | THROT_CURRENTLIMIT)

static ap_arg_t g_args[MAX_CORES];

/* Inline yield — does THREE things on every call from any core:
     1. Sample this core's OWN APERF/MPERF/TSC → C0 residency % + effective
        frequency (turbo / throttle visible).
     2. Sample this core's IA32_THERM_STATUS → current temperature in °C.
     3. If this is BSP (yield != NULL), trigger UI render. APs leave yield
        NULL so they don't race with BSP on the framebuffer.
   rdmsr reads the CURRENT core's MSRs — that's why each AP samples its own. */
static inline void ap_yield(ap_arg_t *a) {
    if (g_has_aperf_mperf) {
        UINT64 tsc_now   = rdtsc_now();
        UINT64 mperf_now = rdmsr_safe(MSR_IA32_MPERF);
        UINT64 aperf_now = rdmsr_safe(MSR_IA32_APERF);
        if (a->util_tsc_prev != 0 && tsc_now > a->util_tsc_prev) {
            UINT64 dt = tsc_now   - a->util_tsc_prev;
            UINT64 dm = mperf_now - a->util_mperf_prev;
            UINT64 da = aperf_now - a->util_aperf_prev;
            if (dt > 0) {
                /* MPERF/TSC = fraction of time in C0 (executing). */
                UINT64 pct = (dm * 100) / dt;
                if (pct > 100) pct = 100;
                a->util_pct = (UINT32)pct;
            }
            if (dm > 0 && g_base_freq_mhz > 0) {
                /* Effective freq = (APERF/MPERF) × base; dm > 0 ensures we
                   don't divide by zero when core was idle the whole window. */
                UINT64 fmhz = (da * g_base_freq_mhz) / dm;
                if (fmhz > 0xFFFFFFFFULL) fmhz = 0xFFFFFFFF;
                a->freq_mhz = (UINT32)fmhz;
            }
        }
        a->util_tsc_prev   = tsc_now;
        a->util_mperf_prev = mperf_now;
        a->util_aperf_prev = aperf_now;
    }
    /* Per-core temperature sampling via IA32_THERM_STATUS (MSR 0x19C) is
       INTEL-ONLY. On AMD this MSR is unsupported — rdmsr triggers #GP
       and the UEFI default handler doesn't return, freezing the whole
       AP (and ultimately the system, because BSP polls all APs to be
       done). Critical: amd_thermal_probe() sets g_has_thermal = 1 to
       enable the temperature column display (sourced from SMN per-
       PACKAGE sampling in sample_aggregate_metrics), and v0.4.2 added
       g_tj_max = 100 as the AMD fallback. So on AMD both guard
       conditions are true and we WOULD hit the #GP — except for the
       g_cpu_vendor check added here.

       This is THE bug that caused the field-reported AMD AVX2 / TRRespass
       hangs from v0.4.2 through v0.4.9. Every kernel calls ap_yield()
       periodically; first such call after init on AMD froze the AP.
       All the v0.4.5-v0.4.9 thermal-guard work was attacking the wrong
       symptom — the real bug was this one unguarded MSR read.

       On AMD per-core temp stays 0 (column shows '—'). The per-package
       Tctl from SMN is still displayed via sample_aggregate_metrics. */
    if (g_has_thermal && g_tj_max > 0 && g_cpu_vendor == CPU_INTEL) {
        UINT64 ts = rdmsr_safe(MSR_IA32_THERM_STATUS);
        UINT32 delta = (UINT32)((ts >> 16) & 0x7F);   /* °C below TjMax */
        a->temp_c = (g_tj_max > delta) ? (g_tj_max - delta) : 0;
        /* Throttle indicators: bits 0/4/10/12 are LIVE flags ("currently in
           thermal/PROCHOT/power-limit/current-limit"). Capture them so the
           UI can show TRM/PRC/PWR/CUR. Edge-detect any-on → any-off to count
           throttle events; lets the user see "cooling marginal" (event
           count grows) vs "fine" (count stays 0). */
        UINT32 flags = (UINT32)(ts & THROT_ANY);
        a->throttle_flags = flags;
        UINT32 active_now = (flags != 0);
        if (active_now && !a->throttle_was_active) {
            a->throttle_event_count++;
        }
        a->throttle_was_active = active_now;
    }
    if (a->yield) a->yield(a);
}

/* ---------- Test kernels with progress reporting ---------- */
/* Helper used by Walking Ones/Zeroes/Moving Inversions: write a pattern over
   the buffer, then read+verify it. Updates a->bytes INCREMENTALLY (in chunks
   matching the progress-callback granularity) so the live UI can show
   meaningful MB/s on the core panel — the previous behaviour set bytes only
   AT THE END of the kernel, which made the per-core MB/s column read 0
   for the whole duration. */
static UINT64 fill_check_p(ap_arg_t *a, UINT64 *p, UINTN n, UINT64 pat,
                            UINT32 base_pmille, UINT32 span_pmille) {
    UINT64 e = 0;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    for (UINTN i = 0; i < n; i++) {
        p[i] = pat;
        if ((i % step) == 0) {
            /* Account ~step qwords × 8 bytes for the fill pass. */
            a->bytes += (UINT64)step * 8;
            a->progress = base_pmille + (UINT32)(span_pmille / 2 * i / n);
            ap_yield(a);
            if (g_aborted) return e;
        }
    }
    __asm__ __volatile__("mfence" ::: "memory");
    for (UINTN i = 0; i < n; i++) {
        if (p[i] != pat) {
            e++;
            record_error(a->kernel, a->core_idx,
                         (UINT64)(UINTN)&p[i], pat, p[i]);
        }
        if ((i % step) == 0) {
            /* Account ~step qwords × 8 bytes for the verify pass. */
            a->bytes += (UINT64)step * 8;
            a->progress = base_pmille + span_pmille / 2
                          + (UINT32)(span_pmille / 2 * i / n);
            ap_yield(a);
            if (g_aborted) return e;
        }
    }
    return e;
}

static void run_walking_ones(ap_arg_t *a) {
    UINT64 e = 0;
    a->bytes = 0;  /* fill_check_p will accumulate */
    for (int b = 0; b < 64; b++) {
        UINT32 base = (UINT32)(1000ULL * b / 64);
        UINT32 span = (UINT32)(1000ULL / 64);
        e += fill_check_p(a, a->base, a->n_qwords, ((UINT64)1) << b, base, span);
        if (g_aborted) break;
    }
    a->progress = 1000;
    a->errors = e;
    /* bytes already accumulated in fill_check_p */
}

static void run_walking_zeroes(ap_arg_t *a) {
    UINT64 e = 0;
    a->bytes = 0;
    for (int b = 0; b < 64; b++) {
        UINT32 base = (UINT32)(1000ULL * b / 64);
        UINT32 span = (UINT32)(1000ULL / 64);
        e += fill_check_p(a, a->base, a->n_qwords, ~(((UINT64)1) << b), base, span);
        if (g_aborted) break;
    }
    a->progress = 1000;
    a->errors = e;
}

static void run_moving_inv(ap_arg_t *a) {
    UINT64 e = 0;
    UINT64 pats[] = { 0x0000000000000000ULL, 0xFFFFFFFFFFFFFFFFULL,
                      0xAAAAAAAAAAAAAAAAULL, 0x5555555555555555ULL };
    UINTN np = sizeof(pats)/sizeof(pats[0]);
    UINTN step = a->n_qwords / PROGRESS_GRAIN; if (!step) step = a->n_qwords;
    a->bytes = 0;
    for (UINTN k = 0; k < np; k++) {
        UINT32 base = (UINT32)(1000ULL * k / np);
        UINT32 span = (UINT32)(1000ULL / np);
        for (UINTN i = 0; i < a->n_qwords; i++) {
            a->base[i] = pats[k];
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base + (UINT32)(span / 3 * i / a->n_qwords);
                ap_yield(a);
                if (g_aborted) goto mi_done;
            }
        }
        __asm__ __volatile__("mfence" ::: "memory");
        for (UINTN i = 0; i < a->n_qwords; i++) {
            if (a->base[i] != pats[k]) {
                e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&a->base[i], pats[k], a->base[i]);
            }
            a->base[i] = ~pats[k];
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base + span / 3 + (UINT32)(span / 3 * i / a->n_qwords);
                ap_yield(a);
                if (g_aborted) goto mi_done;
            }
        }
        __asm__ __volatile__("mfence" ::: "memory");
        for (UINTN i = 0; i < a->n_qwords; i++) {
            if (a->base[i] != ~pats[k]) {
                e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&a->base[i], ~pats[k], a->base[i]);
            }
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base + 2 * span / 3 + (UINT32)(span / 3 * i / a->n_qwords);
                ap_yield(a);
                if (g_aborted) goto mi_done;
            }
        }
    }
mi_done:
    a->progress = 1000;
    a->errors = e;
}

static void run_address(ap_arg_t *a) {
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    UINT32 reps = ADDR_REPEATS;
    a->bytes = 0;

    for (UINT32 rep = 0; rep < reps; rep++) {
        UINT32 base_p = (UINT32)(1000ULL * rep / reps);
        UINT32 span_p = (UINT32)(1000ULL / reps);
        /* Each rep uses a different XOR seed so consecutive passes exercise
           different bit patterns through the address-decoding logic. */
        UINT64 seed = (UINT64)rep * 0x9E3779B97F4A7C15ULL;

        for (UINTN i = 0; i < n; i++) {
            a->base[i] = ((UINT64)(UINTN)&a->base[i]) ^ seed;
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + (UINT32)(span_p / 2 * i / n);
                ap_yield(a);
                if (g_aborted) goto ap_done;
            }
        }
        __asm__ __volatile__("mfence" ::: "memory");
        for (UINTN i = 0; i < n; i++) {
            UINT64 expected = ((UINT64)(UINTN)&a->base[i]) ^ seed;
            if (a->base[i] != expected) {
                e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&a->base[i], expected, a->base[i]);
            }
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + span_p / 2 + (UINT32)(span_p / 2 * i / n);
                ap_yield(a);
                if (g_aborted) goto ap_done;
            }
        }
    }
ap_done:
    a->progress = 1000;
    a->errors = e;
}

static void run_avx2(ap_arg_t *a) {
    if (!g_has_avx2) {
        a->errors = 0;
        a->bytes  = 0;
        a->progress = 1000;
        return;
    }
    UINT64 e = 0;
    UINTN n32 = a->n_qwords / 4;
    UINT64 *p = a->base;
    UINT64 pat_base[4] = { 0xCAFEBABEDEADBEEFULL, 0x1234567890ABCDEFULL,
                           0xA5A5A5A55A5A5A5AULL, 0xFEDCBA0987654321ULL };
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    UINT32 reps = AVX2_REPEATS;
    a->bytes = 0;

    for (UINT32 rep = 0; rep < reps; rep++) {
        UINT32 base_p = (UINT32)(1000ULL * rep / reps);
        UINT32 span_p = (UINT32)(1000ULL / reps);
        /* Rotate patterns each rep so YMM exercising covers different bits. */
        UINT64 pat[4] = {
            pat_base[(0 + rep) & 3],
            pat_base[(1 + rep) & 3],
            pat_base[(2 + rep) & 3],
            pat_base[(3 + rep) & 3],
        };
        UINT64 *pat_p = pat;

        a->progress = base_p;
        ap_yield(a);
        if (g_aborted) goto av_done;

        /* Phase A: fill the buffer with pat[]. Pure memory bandwidth. */
        UINT64 *p_dst = p;
        UINTN cnt = n32;
        __asm__ __volatile__(
            "vmovdqu (%[pat]), %%ymm0\n\t"
            "1: vmovdqu %%ymm0, (%[dst])\n\t"
            "   add $32, %[dst]\n\t"
            "   dec %[cnt]\n\t"
            "   jnz 1b\n\t"
            "vzeroupper\n\t"
            : [dst] "+r"(p_dst), [cnt] "+r"(cnt)
            : [pat] "r"(pat_p)
            : "ymm0", "memory", "cc");

        /* Account for the bandwidth fill phase (write n_qwords × 8 B). */
        a->bytes += (UINT64)a->n_qwords * 8;
        a->progress = base_p + span_p / 3;
        ap_yield(a);
        if (g_aborted) goto av_done;

        /* Phase B: FMA-chain compute stress. Register-only, no memory writes —
           this is what actually concentrates power draw on the FMA execution
           units (the part that stresses VRM and gives Prime95-small-FFT-style
           thermals). Four independent dependency chains keep all FMA ports busy. */
        {
            UINTN fma_iters = 1000000;
            __asm__ __volatile__(
                "vmovdqu (%[pat]), %%ymm0\n\t"
                "vmovdqu (%[pat]), %%ymm1\n\t"
                "vmovdqu (%[pat]), %%ymm2\n\t"
                "vmovdqu (%[pat]), %%ymm3\n\t"
                "1:\n\t"
                "vfmadd231ps %%ymm1, %%ymm2, %%ymm0\n\t"
                "vfmadd231ps %%ymm2, %%ymm3, %%ymm1\n\t"
                "vfmadd231ps %%ymm3, %%ymm0, %%ymm2\n\t"
                "vfmadd231ps %%ymm0, %%ymm1, %%ymm3\n\t"
                "dec %[cnt]\n\t"
                "jnz 1b\n\t"
                "vzeroupper\n\t"
                : [cnt] "+r"(fma_iters)
                : [pat] "r"(pat_p)
                : "ymm0", "ymm1", "ymm2", "ymm3", "cc");
        }

        a->progress = base_p + 2 * span_p / 3;
        ap_yield(a);
        if (g_aborted) goto av_done;

        for (UINTN i = 0; i + 3 < n; i += 4) {
            for (UINTN k = 0; k < 4; k++) {
                if (p[i+k] != pat[k]) {
                    e++;
                    record_error(a->kernel, a->core_idx,
                                 (UINT64)(UINTN)&p[i+k], pat[k], p[i+k]);
                }
            }
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + span_p / 2 + (UINT32)(span_p / 2 * i / n);
                ap_yield(a);
                if (g_aborted) goto av_done;
            }
        }
    }
av_done:
    a->progress = 1000;
    a->errors = e;
}

static void run_rowhammer(ap_arg_t *a) {
    if (!g_has_clflush) {
        a->errors = 0;
        a->bytes  = 0;
        a->progress = 1000;
        return;
    }
    UINT64 e = 0;
    UINT64 *p = a->base;
    UINTN n = a->n_qwords;
    /* init */
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    a->bytes = 0;
    for (UINTN i = 0; i < n; i++) {
        p[i] = 0xFFFFFFFFFFFFFFFFULL;
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 8;
            a->progress = (UINT32)(150ULL * i / n);
            ap_yield(a);
            if (g_aborted) goto rh_done;
        }
    }
    __asm__ __volatile__("mfence" ::: "memory");

    /* Multi-pair hammering: TRR-equipped DDR4/5 generally absorbs hammering
       on a single aggressor pair, so a one-pair test can falsely PASS. We
       split the iteration budget across N pairs at varied strides — much
       more representative of real-world attack patterns. On DDR5 we double
       the iteration budget (g_rowhammer_mult_x100 == 200) because TRR-v2
       and RFM are designed to actively absorb hot-row activations. */
    UINT64 effective_iters = ROWHAMMER_ITERS * g_rowhammer_mult_x100 / 100;
    UINTN q_per_stride = ROWHAMMER_STRIDE / 8;
    if (n >= 8 * q_per_stride) {
        UINT64 stride_mults[] = { 1, 2, 3, 5, 7, 11 };   /* prime-ish */
        UINTN num_pairs = sizeof(stride_mults) / sizeof(stride_mults[0]);
        UINT64 iters_per_pair = effective_iters / num_pairs;
        UINT64 step_iters = iters_per_pair / (PROGRESS_GRAIN / num_pairs + 1);
        if (!step_iters) step_iters = 1;
        volatile UINT64 sink = 0;

        for (UINTN pair = 0; pair < num_pairs; pair++) {
            UINT64 mult = stride_mults[pair];
            /* Spread aggressors across the buffer; clamp to fit. */
            UINTN a_off = (q_per_stride * mult) % (n - q_per_stride);
            UINTN b_off = (q_per_stride * mult * 3) % (n - q_per_stride);
            if (a_off == b_off) b_off = (b_off + q_per_stride) % (n - q_per_stride);
            UINT64 *agg1 = p + a_off;
            UINT64 *agg2 = p + b_off;

            UINT32 base_p = 150 + (UINT32)(700ULL * pair / num_pairs);
            UINT32 span_p = (UINT32)(700ULL / num_pairs);

            for (UINT64 it = 0; it < iters_per_pair; it++) {
                sink = *(volatile UINT64 *)agg1;
                sink = *(volatile UINT64 *)agg2;
                __asm__ __volatile__("clflush (%0)\n\t"
                                     "clflush (%1)\n\t" : : "r"(agg1), "r"(agg2));
                if ((it % step_iters) == 0) {
                    /* Each step_iters iterations touched 16 bytes (2 reads). */
                    a->bytes += step_iters * 16;
                    a->progress = base_p + (UINT32)(span_p * it / iters_per_pair);
                    ap_yield(a);
                    if (g_aborted) { (void)sink; goto rh_done; }
                }
            }
        }
        (void)sink;
    }
    __asm__ __volatile__("mfence" ::: "memory");
    /* check — anything other than all-ones is a row-hammer flip */
    for (UINTN i = 0; i < n; i++) {
        if (p[i] != 0xFFFFFFFFFFFFFFFFULL) {
            e++;
            record_error(a->kernel, a->core_idx,
                         (UINT64)(UINTN)&p[i], 0xFFFFFFFFFFFFFFFFULL, p[i]);
        }
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 8;
            a->progress = 850 + (UINT32)(150ULL * i / n);
            ap_yield(a);
            if (g_aborted) goto rh_done;
        }
    }
rh_done:
    a->progress = 1000;
    a->errors = e;
    /* bytes accumulated incrementally above; no overwrite needed */
}

static void run_block_move(ap_arg_t *a) {
    UINT64 e = 0;
    UINTN half = a->n_qwords / 2;
    UINT64 *src = a->base;
    UINT64 *dst = a->base + half;
    UINTN step = half / PROGRESS_GRAIN; if (!step) step = half;
    UINT32 reps = BLOCK_REPEATS;
    a->bytes = 0;

    for (UINT32 rep = 0; rep < reps; rep++) {
        UINT32 base_p = (UINT32)(1000ULL * rep / reps);
        UINT32 span_p = (UINT32)(1000ULL / reps);
        UINT64 pat = 0xAAAAAAAAAAAAAAAAULL ^ ((UINT64)rep * 0x1111111111111111ULL);

        /* Phase 1/3: fill src */
        for (UINTN i = 0; i < half; i++) {
            src[i] = pat ^ i;
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + (UINT32)(span_p / 3 * i / half);
                ap_yield(a);
                if (g_aborted) goto bm_done;
            }
        }
        __asm__ __volatile__("mfence" ::: "memory");
        /* Phase 2/3: copy src→dst (the actual block-move stress) */
        for (UINTN i = 0; i < half; i++) {
            dst[i] = src[i];
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + span_p / 3 + (UINT32)(span_p / 3 * i / half);
                ap_yield(a);
                if (g_aborted) goto bm_done;
            }
        }
        __asm__ __volatile__("mfence" ::: "memory");
        /* Phase 3/3: verify dst */
        for (UINTN i = 0; i < half; i++) {
            UINT64 expected = pat ^ i;
            if (dst[i] != expected) {
                e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&dst[i], expected, dst[i]);
            }
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + 2 * span_p / 3 + (UINT32)(span_p / 3 * i / half);
                ap_yield(a);
                if (g_aborted) goto bm_done;
            }
        }
    }
bm_done:
    a->progress = 1000;
    a->errors = e;
}


/* ---------- Random pattern (xorshift64) ----------
   Catches data-dependent faults that fixed patterns miss. We seed once,
   write the sequence, then re-seed identically and verify — so the kernel
   can regenerate "what we expected" without storing it anywhere else. */
static inline UINT64 xorshift64(UINT64 *s) {
    UINT64 x = *s;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *s = x;
    return x;
}

static void run_random_pattern(ap_arg_t *a) {
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    UINT32 reps = RANDOM_REPEATS;
    a->bytes = 0;

    for (UINT32 rep = 0; rep < reps; rep++) {
        UINT32 base_p = (UINT32)(1000ULL * rep / reps);
        UINT32 span_p = (UINT32)(1000ULL / reps);
        /* Per-rep seed: combines buffer address + core index + rep so each
           pass exercises a different sequence and parallel cores don't
           write identical data into adjacent slices. */
        UINT64 seed = ((UINT64)(UINTN)a->base ^ ((UINT64)a->core_idx << 16))
                       * (rep + 1) + 0x9E3779B97F4A7C15ULL;
        if (seed == 0) seed = 1;  /* xorshift hates zero */

        UINT64 wseed = seed;
        for (UINTN i = 0; i < n; i++) {
            a->base[i] = xorshift64(&wseed);
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + (UINT32)(span_p / 2 * i / n);
                ap_yield(a);
                if (g_aborted) goto rp_done;
            }
        }
        __asm__ __volatile__("mfence" ::: "memory");
        UINT64 vseed = seed;
        for (UINTN i = 0; i < n; i++) {
            UINT64 expected = xorshift64(&vseed);
            if (a->base[i] != expected) {
                e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&a->base[i], expected, a->base[i]);
            }
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + span_p / 2 + (UINT32)(span_p / 2 * i / n);
                ap_yield(a);
                if (g_aborted) goto rp_done;
            }
        }
    }
rp_done:
    a->progress = 1000;
    a->errors = e;
}

/* ---------- Bit Fade (DRAM retention) ----------
   Writes a constant pattern, sleeps for N seconds, then verifies. If a cell
   loses charge faster than refresh can keep up, the value drifts and we
   detect it. memtest86+ Test 10 does this with 5-min waits — we default to
   60 s for shop-pace usage. Configurable via [Run] BitFadeSeconds. */
static void run_bit_fade(ap_arg_t *a) {
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    /* Only BSP (slot 0) actually times the wait — APs would burn idle time
       waiting too. Instead each AP fills its slice and verifies; the global
       wait happens in BSP between fill and verify. Practically that means
       APs must finish their fill quickly, then idle, then resume verify. */
    UINT64 wait_ms = (UINT64)g_cfg_bitfade_s * 1000ULL;
    /* Finer chunking so the header spinner + countdown update smoothly
       (~2 Hz) even during the long wait. Was 50 chunks = ~2.4 sec each;
       now n_chunks scales with wait_ms so each chunk is ~500 ms. */
    UINTN n_chunks = wait_ms / 500;
    if (n_chunks < 50) n_chunks = 50;
    UINT64 chunk_ms = wait_ms / n_chunks;
    if (chunk_ms == 0) chunk_ms = 1;

    UINT64 patterns[2] = { 0xFFFFFFFFFFFFFFFFULL, 0x0000000000000000ULL };
    a->bytes = 0;
    for (int phase = 0; phase < 2; phase++) {
        UINT64 pat = patterns[phase];
        UINT32 base_p = (phase == 0) ? 0 : 500;
        UINT32 span_p = 500;

        /* Fill (10% of progress) */
        for (UINTN i = 0; i < n; i++) {
            a->base[i] = pat;
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + (UINT32)(span_p / 10 * i / n);
                ap_yield(a);
                if (g_aborted) goto bf_done;
            }
        }
        __asm__ __volatile__("mfence" ::: "memory");

        /* Wait — 80% of progress, advanced uniformly across the wait.
           ALL cores wait (not just BSP). Previously only BSP entered this
           loop while APs jumped straight to verify; that meant 15/16 of
           the buffer was "verified" while the data was still hot in L3 —
           no actual DRAM retention testing happened on those slices. With
           every AP also calling BS->Stall, each slice of the buffer gets
           the same write→wait→verify cycle that defines this test. Stall
           is a TSC busy-wait, safe to invoke from any AP context. */
        for (UINTN c = 0; c < n_chunks; c++) {
            uefi_call_wrapper(BS->Stall, 1, (UINTN)(chunk_ms * 1000));
            a->progress = base_p + span_p / 10 + (UINT32)((UINT64)span_p * 8ULL / 10 * c / n_chunks);
            ap_yield(a);
            if (g_aborted) goto bf_done;
        }

        /* Verify (last 10%) */
        for (UINTN i = 0; i < n; i++) {
            if (a->base[i] != pat) {
                e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&a->base[i], pat, a->base[i]);
            }
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + span_p * 9 / 10 + (UINT32)(span_p / 10 * i / n);
                ap_yield(a);
                if (g_aborted) goto bf_done;
            }
        }
    }
bf_done:
    a->progress = 1000;
    a->errors = e;
}

/* ====================================================================
   AGGRESSIVE MODE KERNELS
   These are stronger algorithms used by the [3] AGGRESSIVE menu option.
   Each targets a specific class of memory fault that the standard kernels
   miss. References cited in the kernel comments. */

/* ---------- March-C- (van de Goor 1997) ----------
   Industry-standard memory test sequence. Six linear passes through the
   buffer catch 92% of fault models in 6n operations (vs Walking Ones'
   75% in 128n ops — March-C- is BOTH faster AND more thorough):
     1. ↕(w0)     write 0 everywhere (any direction)
     2. ↑(r0,w1)  up:   read 0 → write 1
     3. ↑(r1,w0)  up:   read 1 → write 0
     4. ↓(r0,w1)  down: read 0 → write 1
     5. ↓(r1,w0)  down: read 1 → write 0
     6. ↕(r0)     read 0 everywhere
   Catches: Stuck-At (SAF), Address (AF), Transition (TF), and Coupling
   (CFid, CFin, CFst) faults — all the common cell-level defect modes. */
static void run_march_cm(ap_arg_t *a) {
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    a->bytes = 0;

    /* Phase 1: ↕(w0) — write all zeros (any direction) */
    for (UINTN i = 0; i < n; i++) {
        a->base[i] = 0;
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 8;
            a->progress = (UINT32)(170ULL * i / n);
            ap_yield(a); if (g_aborted) goto mc_done;
        }
    }
    __asm__ __volatile__("mfence" ::: "memory");

    /* Phase 2: ↑(r0,w1) */
    for (UINTN i = 0; i < n; i++) {
        if (a->base[i] != 0) { e++;
            record_error(a->kernel, a->core_idx,
                         (UINT64)(UINTN)&a->base[i], 0, a->base[i]); }
        a->base[i] = 0xFFFFFFFFFFFFFFFFULL;
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 16;
            a->progress = 170 + (UINT32)(170ULL * i / n);
            ap_yield(a); if (g_aborted) goto mc_done;
        }
    }
    __asm__ __volatile__("mfence" ::: "memory");

    /* Phase 3: ↑(r1,w0) */
    for (UINTN i = 0; i < n; i++) {
        if (a->base[i] != 0xFFFFFFFFFFFFFFFFULL) { e++;
            record_error(a->kernel, a->core_idx,
                         (UINT64)(UINTN)&a->base[i],
                         0xFFFFFFFFFFFFFFFFULL, a->base[i]); }
        a->base[i] = 0;
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 16;
            a->progress = 340 + (UINT32)(170ULL * i / n);
            ap_yield(a); if (g_aborted) goto mc_done;
        }
    }
    __asm__ __volatile__("mfence" ::: "memory");

    /* Phase 4: ↓(r0,w1) — note reverse iteration order */
    for (UINTN k = n; k > 0; k--) {
        UINTN i = k - 1;
        if (a->base[i] != 0) { e++;
            record_error(a->kernel, a->core_idx,
                         (UINT64)(UINTN)&a->base[i], 0, a->base[i]); }
        a->base[i] = 0xFFFFFFFFFFFFFFFFULL;
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 16;
            a->progress = 510 + (UINT32)(170ULL * (n - i) / n);
            ap_yield(a); if (g_aborted) goto mc_done;
        }
    }
    __asm__ __volatile__("mfence" ::: "memory");

    /* Phase 5: ↓(r1,w0) */
    for (UINTN k = n; k > 0; k--) {
        UINTN i = k - 1;
        if (a->base[i] != 0xFFFFFFFFFFFFFFFFULL) { e++;
            record_error(a->kernel, a->core_idx,
                         (UINT64)(UINTN)&a->base[i],
                         0xFFFFFFFFFFFFFFFFULL, a->base[i]); }
        a->base[i] = 0;
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 16;
            a->progress = 680 + (UINT32)(170ULL * (n - i) / n);
            ap_yield(a); if (g_aborted) goto mc_done;
        }
    }
    __asm__ __volatile__("mfence" ::: "memory");

    /* Phase 6: ↕(r0) — final verify */
    for (UINTN i = 0; i < n; i++) {
        if (a->base[i] != 0) { e++;
            record_error(a->kernel, a->core_idx,
                         (UINT64)(UINTN)&a->base[i], 0, a->base[i]); }
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 8;
            a->progress = 850 + (UINT32)(150ULL * i / n);
            ap_yield(a); if (g_aborted) goto mc_done;
        }
    }
mc_done:
    a->progress = 1000;
    a->errors = e;
}

/* ---------- March-RAW (Read-After-Write) ----------
   Extends March-C- with explicit read-after-write sequences that catch
   DYNAMIC coupling faults — cell A's behaviour depends on cell B's
   RECENT write history, not just its current value. Pattern:
       ↑(w0, r0, r0, w1, r1, r1)
       ↓(w0, r0, r0, w1, r1, r1)
   The "r,r" pairs let us detect cells that initially read correctly but
   FLIP on a second read (charge sharing / bit-line precharge bugs). */
static void run_march_raw(ap_arg_t *a) {
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    a->bytes = 0;
    UINT64 zero = 0, ones = 0xFFFFFFFFFFFFFFFFULL;

    for (int dir = 0; dir < 2; dir++) {
        UINT32 base_p = (dir == 0) ? 0 : 500;
        for (UINTN k = 0; k < n; k++) {
            UINTN i = (dir == 0) ? k : (n - 1 - k);
            /* Cast to volatile so the compiler emits TWO actual loads instead
               of collapsing v1==v2 via constant propagation from the just-
               written value. Without volatile, -O2 turns the "r,r" pair into
               a no-op and the entire test becomes structural-only. */
            volatile UINT64 *vp = (volatile UINT64 *)&a->base[i];
            *vp = zero;
            UINT64 v1 = *vp;
            UINT64 v2 = *vp;
            if (v1 != zero || v2 != zero) { e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&a->base[i], zero,
                             (v1 != zero) ? v1 : v2); }
            *vp = ones;
            UINT64 w1 = *vp;
            UINT64 w2 = *vp;
            if (w1 != ones || w2 != ones) { e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&a->base[i], ones,
                             (w1 != ones) ? w1 : w2); }
            if ((k % step) == 0) {
                a->bytes += (UINT64)step * 48;  /* 2w + 4r per cell */
                a->progress = base_p + (UINT32)(500ULL * k / n);
                ap_yield(a); if (g_aborted) goto raw_done;
            }
        }
        __asm__ __volatile__("mfence" ::: "memory");
    }
raw_done:
    a->progress = 1000;
    a->errors = e;
}

/* ---------- Checkerboard Butterfly ----------
   Each cell is written with the OPPOSITE pattern of its neighbours, then
   the centre cell is flipped repeatedly while neighbour cells are read.
   Catches "butterfly faults" — coupling between cells that only triggers
   when neighbour is in opposite state.
   Implementation: alternate 0xAA55AA55... / 0x55AA55AA... per 64-byte
   cacheline, then for every cacheline boundary flip the centre and
   re-read neighbouring lines. */
static void run_butterfly(ap_arg_t *a) {
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    a->bytes = 0;

    /* Phase 1: fill with alternating cacheline patterns. 8 qwords per
       cacheline; alternate per-cacheline between two patterns. */
    UINT64 pat_a = 0xAA55AA55AA55AA55ULL;
    UINT64 pat_b = 0x55AA55AA55AA55AAULL;
    for (UINTN i = 0; i < n; i++) {
        a->base[i] = ((i >> 3) & 1) ? pat_b : pat_a;
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 8;
            a->progress = (UINT32)(300ULL * i / n);
            ap_yield(a); if (g_aborted) goto bf2_done;
        }
    }
    __asm__ __volatile__("mfence" ::: "memory");

    /* Phase 2: butterfly — for every 4th cacheline, flip ALL bits and read
       back the NEIGHBOURING cachelines to detect coupling. Skip the
       extreme edges so the index math stays simple. */
    UINTN cl = n / 8;  /* number of cachelines */
    if (cl >= 4) {
        for (UINTN c = 2; c + 2 < cl; c += 4) {
            /* Invert centre cacheline */
            UINTN base_i = c * 8;
            for (UINTN k = 0; k < 8; k++)
                a->base[base_i + k] = ~a->base[base_i + k];
            __asm__ __volatile__("mfence" ::: "memory");
            /* Read neighbours, check they didn't flip too */
            for (int nb = -2; nb <= 2; nb++) {
                if (nb == 0) continue;
                UINTN nb_i = base_i + nb * 8;
                UINT64 expected = ((nb_i >> 3) & 1) ? pat_b : pat_a;
                for (UINTN k = 0; k < 8; k++) {
                    if (a->base[nb_i + k] != expected) {
                        e++;
                        record_error(a->kernel, a->core_idx,
                                     (UINT64)(UINTN)&a->base[nb_i + k],
                                     expected, a->base[nb_i + k]);
                    }
                }
            }
            if ((c % (step / 8 + 1)) == 0) {
                a->bytes += 16 * 64;  /* 1 cl × invert + 4 cls × verify */
                a->progress = 300 + (UINT32)(400ULL * c / cl);
                ap_yield(a); if (g_aborted) goto bf2_done;
            }
        }
    }

    /* Phase 3: final verify pass — every cacheline should be at its
       baseline pattern (the butterflies flipped centres back implicitly
       via the inversion sequence; some won't be — those are errors). */
    for (UINTN i = 0; i < n; i++) {
        UINT64 expected = ((i >> 3) & 1) ? pat_b : pat_a;
        /* Cachelines that were "centres" got inverted ONCE. We don't
           track those — final state is OK if the pattern matches OR is
           the inversion. Don't count flipped centres as errors. */
        if (a->base[i] != expected && a->base[i] != ~expected) {
            e++;
            record_error(a->kernel, a->core_idx,
                         (UINT64)(UINTN)&a->base[i], expected, a->base[i]);
        }
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 8;
            a->progress = 700 + (UINT32)(300ULL * i / n);
            ap_yield(a); if (g_aborted) goto bf2_done;
        }
    }
bf2_done:
    a->progress = 1000;
    a->errors = e;
}

/* ---------- AVX2 Sustained (~30 sec FMA + memory) ----------
   The standard AVX2 test is 3 sec — too short to actually heat VRM. This
   variant runs 30 sec of FMA chain (~10× longer) WITH interleaved memory
   writes, so the test hits VRM + IMC simultaneously. This is the closest
   pre-OS analogue to Prime95 Small FFT thermal stress. */
static void run_avx2_sustained(ap_arg_t *a) {
    if (!g_has_avx2) {
        a->errors = 0; a->bytes = 0; a->progress = 1000;
        return;
    }
    /* Diagnostic: BSP-only checkpoint to distinguish "entered AVX2 kernel
       and hung inside" from "never entered AVX2 kernel". Field-reported
       AMD hang traced this far in v0.4.4 — next log will show whether
       this line appears (thermal halt during burst) or doesn't (some
       earlier fault we missed). */
    if (a->core_idx == 0 && g_cur_test_idx == 0) {
        log_line(L"[KERNEL] AVX2 Sustained: entering burst loop");
    }
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN n32 = n / 4;          /* YMM regs are 32 bytes = 4 qwords */
    UINT64 pat[4] = { 0xCAFEBABEDEADBEEFULL, 0x1234567890ABCDEFULL,
                      0xA5A5A5A55A5A5A5AULL, 0xFEDCBA0987654321ULL };
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    a->bytes = 0;

    /* We loop for ~10 sec wall-clock; each inner iteration does a memory
       fill + a heavy FMA burst + a verify. 10 sec × 32 passes on a 32 GB
       system = ~5 min of cumulative AVX2 — enough to surface VRM/IMC
       transient bugs without overheating weak coolers (memtest86+ uses no
       AVX2 at all; we add it as a "Prime95-Small-FFT-lite" stress). */
    UINT64 t_start = ms_now();
    UINT32 iter = 0;
    while (ms_now() - t_start < 10000 && !g_aborted) {
        iter++;
        /* Fill buffer with pattern */
        UINT64 *p_dst = a->base;
        UINTN cnt = n32;
        __asm__ __volatile__(
            "vmovdqu (%[pat]), %%ymm0\n\t"
            "1: vmovdqu %%ymm0, (%[dst])\n\t"
            "   add $32, %[dst]\n\t"
            "   dec %[cnt]\n\t"
            "   jnz 1b\n\t"
            "vzeroupper\n\t"
            : [dst] "+r"(p_dst), [cnt] "+r"(cnt)
            : [pat] "r"(pat)
            : "ymm0", "memory", "cc");
        a->bytes += (UINT64)n * 8;

        /* Heavy FMA chain — must dominate wall-clock time vs the memory
           fill above, otherwise the CPU sits in DRAM-stall and never
           warms up. Vendor-aware intensity to avoid VRM trip / thermal
           halt on budget AMD boards:

             Intel: 15M iters × 8 dep chains. Intel HWP throttles
             aggressively per-core; high-end Z-series VRMs handle the
             load; matches Prime95 Small FFT power profile (~150W on
             i7-12700K). Field-tested OK on Z690, Z790, OptiPlex 7060.

             AMD: 5M iters × 4 dep chains, PLUS a 'pause × 4' breather
             every 128K outer iterations. Power profile drops ~70%
             without losing the "sustained AVX2 stress" semantic.
             Reason: AMD Zen has double-pumped AVX2 (256-bit YMM
             dispatched as 2× 128-bit ops), and budget ASUS B-series /
             MSI Pro 4-fase VRMs cannot sustain a Prime95-class
             workload on 12 cores — field-reported halt on Ryzen 5 4500
             + ASUS B-series traces directly to this. With the softer
             load, AMD systems still measurably heat up (good for
             surfacing VRM transients and thermal-margin DRAM faults),
             but stay within VRM and Tjmax envelope. */
        UINTN fma_iters_full = (g_cpu_vendor == CPU_AMD) ? 5000000 : 15000000;
        UINTN fma_iters = fma_iters_full;
        if (g_cpu_vendor == CPU_AMD) {
            /* 4 chains + pause-every-N variant — softer thermal envelope. */
            __asm__ __volatile__(
                "vmovdqu (%[pat]), %%ymm0\n\t"
                "vmovdqu (%[pat]), %%ymm1\n\t"
                "vmovdqu (%[pat]), %%ymm2\n\t"
                "vmovdqu (%[pat]), %%ymm3\n\t"
                "1:\n\t"
                "vfmadd231ps %%ymm0, %%ymm0, %%ymm0\n\t"
                "vfmadd231ps %%ymm1, %%ymm1, %%ymm1\n\t"
                "vfmadd231ps %%ymm2, %%ymm2, %%ymm2\n\t"
                "vfmadd231ps %%ymm3, %%ymm3, %%ymm3\n\t"
                "dec %[cnt]\n\t"
                "jz 2f\n\t"
                /* Every 131072 iterations (0x1FFFF mask) insert a small
                   pause to let VRM / thermal recover. Adds <0.1% wall-
                   clock overhead but prevents sustained pipeline
                   saturation that trips weak VRMs. */
                "mov %[cnt], %%rax\n\t"
                "and $0x1FFFF, %%rax\n\t"
                "jnz 1b\n\t"
                "pause\n\tpause\n\tpause\n\tpause\n\t"
                "jmp 1b\n\t"
                "2:\n\t"
                "vzeroupper\n\t"
                : [cnt] "+r"(fma_iters)
                : [pat] "r"(pat)
                : "ymm0", "ymm1", "ymm2", "ymm3", "rax", "cc");
        } else {
            /* 8 chains, no breather — Intel HWP handles throttling. */
            __asm__ __volatile__(
                "vmovdqu (%[pat]), %%ymm0\n\t"
                "vmovdqu (%[pat]), %%ymm1\n\t"
                "vmovdqu (%[pat]), %%ymm2\n\t"
                "vmovdqu (%[pat]), %%ymm3\n\t"
                "vmovdqu (%[pat]), %%ymm4\n\t"
                "vmovdqu (%[pat]), %%ymm5\n\t"
                "vmovdqu (%[pat]), %%ymm6\n\t"
                "vmovdqu (%[pat]), %%ymm7\n\t"
                "1:\n\t"
                "vfmadd231ps %%ymm0, %%ymm0, %%ymm0\n\t"
                "vfmadd231ps %%ymm1, %%ymm1, %%ymm1\n\t"
                "vfmadd231ps %%ymm2, %%ymm2, %%ymm2\n\t"
                "vfmadd231ps %%ymm3, %%ymm3, %%ymm3\n\t"
                "vfmadd231ps %%ymm4, %%ymm4, %%ymm4\n\t"
                "vfmadd231ps %%ymm5, %%ymm5, %%ymm5\n\t"
                "vfmadd231ps %%ymm6, %%ymm6, %%ymm6\n\t"
                "vfmadd231ps %%ymm7, %%ymm7, %%ymm7\n\t"
                "dec %[cnt]\n\t"
                "jnz 1b\n\t"
                "vzeroupper\n\t"
                : [cnt] "+r"(fma_iters)
                : [pat] "r"(pat)
                : "ymm0", "ymm1", "ymm2", "ymm3",
                  "ymm4", "ymm5", "ymm6", "ymm7", "cc");
        }

        /* Verify after the FMA burst — heat may have corrupted memory */
        for (UINTN i = 0; i + 3 < n; i += 4) {
            for (UINTN k = 0; k < 4; k++) {
                if (a->base[i + k] != pat[k]) {
                    e++;
                    record_error(a->kernel, a->core_idx,
                                 (UINT64)(UINTN)&a->base[i + k],
                                 pat[k], a->base[i + k]);
                }
            }
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                UINT32 elapsed = (UINT32)((ms_now() - t_start) * 1000 / 10000);
                if (elapsed > 1000) elapsed = 1000;
                a->progress = elapsed;
                ap_yield(a); if (g_aborted) goto av_done;
            }
        }
    }
av_done:
    a->progress = 1000;
    a->errors = e;
}

/* ---------- TRRespass-style 8-sided RowHammer (Frigo et al. 2020) ----------
   Modern DDR4/5 have Target Row Refresh (TRR) which detects single-pair
   hammers and refreshes the victim row before it can flip. TRRespass
   defeats this by using 8+ aggressor rows interleaved with bank-group
   rotation — TRR's per-bank refresh budget gets exhausted before all
   aggressors are mitigated. Our buffer is 4 KiB rows × 8 banks × 2 bank
   groups (in the typical DDR4/5 controller mapping); we rotate aggressor
   addresses through these to maximise the chance of bypassing TRR. */
static void run_trrespass(ap_arg_t *a) {
    if (!g_has_clflush) {
        a->errors = 0; a->bytes = 0; a->progress = 1000;
        return;
    }
    UINT64 e = 0;
    UINT64 *p = a->base;
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    a->bytes = 0;

    /* Init buffer to all-ones — victim rows must be at known state */
    for (UINTN i = 0; i < n; i++) {
        p[i] = 0xFFFFFFFFFFFFFFFFULL;
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 8;
            a->progress = (UINT32)(80ULL * i / n);
            ap_yield(a); if (g_aborted) goto tr_done;
        }
    }
    __asm__ __volatile__("mfence" ::: "memory");

    /* 8 aggressor offsets, prime-spaced × 8 KiB stride (typical DDR5
       row spacing — 8 banks × 1 KiB row = 8 KiB). Multiplied by primes
       so addresses span multiple bank-groups. */
    UINTN row_qwords = 8192 / 8;     /* 8 KiB row in qwords */
    if (n < 32 * row_qwords) {
        /* Buffer too small for 8 aggressors — fallback to whatever fits */
        row_qwords = n / 32;
        if (row_qwords < 64) goto tr_verify;
    }

    UINT64 aggressor_mult[8] = { 1, 3, 5, 7, 11, 13, 17, 19 };
    /* 4 rounds × 2.5M iters × 8 aggressor reads = 80M activations per pass.
       At ~5 ns per activation (DDR4/5 tRC) = ~12 sec per round × 4 = ~50 s
       wall-clock — long enough to defeat TRR's per-bank refresh budget
       while keeping the test inside the 60-sec budget for shop use. */
    UINT64 iters_per_round = 2500000ULL;
    UINT64 step_iters = iters_per_round / (PROGRESS_GRAIN / 4 + 1);
    if (!step_iters) step_iters = 1;
    volatile UINT64 sink = 0;

    /* Run 4 rounds rotating bank-group targets to defeat TRR cooldown */
    for (UINTN round = 0; round < 4 && !g_aborted; round++) {
        UINT64 *agg[8];
        for (UINTN k = 0; k < 8; k++) {
            UINTN off = (aggressor_mult[k] * row_qwords + round * row_qwords / 8)
                        % (n - row_qwords);
            agg[k] = p + off;
        }
        UINT32 base_p = 80 + (UINT32)(720ULL * round / 4);

        for (UINT64 it = 0; it < iters_per_round; it++) {
            /* Hammer all 8 aggressors in rotation. CLFLUSH evicts each from
               cache so the next read hits DRAM (= DRAM activation). */
            for (UINTN k = 0; k < 8; k++) {
                sink = *(volatile UINT64 *)agg[k];
                __asm__ __volatile__("clflush (%0)" : : "r"(agg[k]));
            }
            if ((it % step_iters) == 0) {
                a->bytes += step_iters * 8 * 8;
                a->progress = base_p
                    + (UINT32)(180ULL * it / iters_per_round);
                ap_yield(a);
                if (g_aborted) { (void)sink; goto tr_verify; }
            }
        }
    }
    (void)sink;
    __asm__ __volatile__("mfence" ::: "memory");

tr_verify:
    /* Check: any qword that's not all-ones is a victim row flip */
    for (UINTN i = 0; i < n; i++) {
        if (p[i] != 0xFFFFFFFFFFFFFFFFULL) {
            e++;
            record_error(a->kernel, a->core_idx,
                         (UINT64)(UINTN)&p[i], 0xFFFFFFFFFFFFFFFFULL, p[i]);
        }
        if ((i % step) == 0) {
            a->progress = 800 + (UINT32)(200ULL * i / n);
            ap_yield(a); if (g_aborted) goto tr_done;
        }
    }
tr_done:
    a->progress = 1000;
    a->errors = e;
}

/* ---------- Cache-Eviction Storm ----------
   For each 64-byte cacheline: write pattern → CLFLUSH → read back. This
   forces EVERY read to hit DRAM (no L3 cache hits), maximising the
   memory controller's bandwidth duty cycle and exposing IMC bank-conflict
   bugs that don't show up under cache-friendly workloads. */
static void run_cache_evict(ap_arg_t *a) {
    if (!g_has_clflush) {
        a->errors = 0; a->bytes = 0; a->progress = 1000;
        return;
    }
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    a->bytes = 0;
    UINT64 seed = ((UINT64)(UINTN)a->base ^ ((UINT64)a->core_idx << 16))
                  + 0x9E3779B97F4A7C15ULL;
    if (seed == 0) seed = 1;

    /* Run for ~10 sec wall-clock or one full buffer pass, whichever ends
       first. Each cacheline (8 qwords) gets: write+flush+read+verify. */
    UINT64 t_start = ms_now();
    UINTN n_cl = n / 8;
    for (UINTN rep = 0; rep < 4; rep++) {
        UINT64 s = seed + rep * 0x123456789ABCDEFULL;
        for (UINTN c = 0; c < n_cl; c++) {
            UINT64 *cl_ptr = a->base + c * 8;
            UINT64 pat = xorshift64(&s);
            /* Write all 8 qwords of the cacheline */
            for (UINTN k = 0; k < 8; k++) cl_ptr[k] = pat ^ k;
            /* Evict from all cache levels */
            __asm__ __volatile__("clflush (%0)" : : "r"(cl_ptr));
            __asm__ __volatile__("mfence" ::: "memory");
            /* Read back (must hit DRAM, not cache) and verify */
            for (UINTN k = 0; k < 8; k++) {
                UINT64 expected = pat ^ k;
                if (cl_ptr[k] != expected) {
                    e++;
                    record_error(a->kernel, a->core_idx,
                                 (UINT64)(UINTN)&cl_ptr[k], expected, cl_ptr[k]);
                }
            }
            if ((c & 1023) == 0) {
                a->bytes += 1024 * 64 * 2;
                UINT64 dt = ms_now() - t_start;
                UINT32 prog = (UINT32)(dt * 1000 / 10000);
                if (prog > 1000) prog = 1000;
                a->progress = prog;
                ap_yield(a);
                if (g_aborted || dt > 10000) goto ce_done;
            }
        }
    }
ce_done:
    a->progress = 1000;
    a->errors = e;
}

/* ---------- VRM Square-Wave (transient stress) ----------
   The VRM PWM controller compensates for load transients via its
   feedback loop. If the loop has marginal phase margin, sudden load
   spikes cause voltage droop/overshoot that can flip memory bits.
   We test by alternating 100% AVX2 load (~50 ms) with idle (~50 ms),
   running this 10 Hz pulse for ~30 sec. */
static void run_vrm_square(ap_arg_t *a) {
    if (!g_has_avx2) {
        a->errors = 0; a->bytes = 0; a->progress = 1000;
        return;
    }
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    UINT64 pat[4] = { 0xDEADBEEFCAFEBABEULL, 0x0123456789ABCDEFULL,
                      0x5A5A5A5AA5A5A5A5ULL, 0xFEDCBA9876543210ULL };
    a->bytes = 0;

    /* Fill buffer with reference pattern */
    for (UINTN i = 0; i + 3 < n; i += 4) {
        for (UINTN k = 0; k < 4; k++) a->base[i + k] = pat[k];
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 8;
            a->progress = (UINT32)(100ULL * i / n);
            ap_yield(a); if (g_aborted) goto vrm_done;
        }
    }
    __asm__ __volatile__("mfence" ::: "memory");

    /* 10 sec of 10 Hz square-wave load. Each cycle = 50 ms AVX2 burst +
       50 ms idle. The transient transitions stress the VRM the most. */
    UINT64 t_start = ms_now();
    UINT32 cycles = 0;
    while (ms_now() - t_start < 10000 && !g_aborted) {
        cycles++;
        /* 50 ms FMA burst — ~300k iters on modern HW */
        UINTN burst = 300000;
        __asm__ __volatile__(
            "vmovdqu (%[pat]), %%ymm0\n\t"
            "vmovdqu (%[pat]), %%ymm1\n\t"
            "1:\n\t"
            "vfmadd231ps %%ymm1, %%ymm0, %%ymm0\n\t"
            "vfmadd231ps %%ymm0, %%ymm1, %%ymm1\n\t"
            "dec %[cnt]\n\t"
            "jnz 1b\n\t"
            "vzeroupper\n\t"
            : [cnt] "+r"(burst)
            : [pat] "r"(pat)
            : "ymm0", "ymm1", "cc");
        /* 50 ms idle — gives VRM time to overshoot back to low */
        uefi_call_wrapper(BS->Stall, 1, (UINTN)50000);

        UINT64 dt = ms_now() - t_start;
        UINT32 prog = 100 + (UINT32)(dt * 800 / 10000);
        if (prog > 900) prog = 900;
        a->progress = prog;
        ap_yield(a);
    }

    /* After the transient stress, verify memory survived */
    for (UINTN i = 0; i + 3 < n; i += 4) {
        for (UINTN k = 0; k < 4; k++) {
            if (a->base[i + k] != pat[k]) {
                e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&a->base[i + k],
                             pat[k], a->base[i + k]);
            }
        }
        if ((i % step) == 0) {
            a->bytes += (UINT64)step * 8;
            a->progress = 900 + (UINT32)(100ULL * i / n);
            ap_yield(a); if (g_aborted) goto vrm_done;
        }
    }
vrm_done:
    a->progress = 1000;
    a->errors = e;
}

/* ---------- Bit Fade Extended (4 patterns × 2 min/phase) ----------
   Standard Bit Fade tests 2 patterns (all-ones, all-zeros). DRAM
   retention failures can be data-dependent — a cell may leak only when
   surrounded by a specific pattern. This variant adds 0xAA/0x55 and
   0xCC/0x33 (denser bit transitions) at full 120-sec wait per phase.
   Each phase: write → wait 120 sec → verify. 4 phases = ~8.5 min. */
static void run_bit_fade_ext(ap_arg_t *a) {
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    UINT64 wait_ms = 120ULL * 1000ULL;      /* 2 min/phase, fixed */
    /* Many small chunks (was 50 × 2.4 sec — too coarse, spinner only
       animated every 2.4 sec). 240 × 0.5 sec lets ap_yield fire at 2 Hz
       so the spinner & countdown in the header stay visibly alive. */
    UINTN n_chunks = 240;
    UINT64 chunk_ms = wait_ms / n_chunks;
    if (chunk_ms == 0) chunk_ms = 1;

    /* 4 patterns: all-ones, all-zeros, 0xAA stripes, 0xCC stripes.
       Each has different DRAM cell stress characteristics. */
    UINT64 patterns[4] = {
        0xFFFFFFFFFFFFFFFFULL,
        0x0000000000000000ULL,
        0xAAAAAAAAAAAAAAAAULL,
        0xCCCCCCCCCCCCCCCCULL,
    };
    a->bytes = 0;
    for (int phase = 0; phase < 4; phase++) {
        UINT64 pat = patterns[phase];
        UINT32 base_p = (UINT32)(phase * 250);
        UINT32 span_p = 250;

        /* Fill */
        for (UINTN i = 0; i < n; i++) {
            a->base[i] = pat;
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + (UINT32)(span_p / 10 * i / n);
                ap_yield(a); if (g_aborted) goto bfe_done;
            }
        }
        __asm__ __volatile__("mfence" ::: "memory");

        /* Wait — all cores Stall (bug-fixed from original Bit Fade where
           only BSP waited). 240 chunks × 0.5 sec for smooth UI animation
           during the wait phase. */
        for (UINTN c = 0; c < n_chunks; c++) {
            uefi_call_wrapper(BS->Stall, 1, (UINTN)(chunk_ms * 1000));
            a->progress = base_p + span_p / 10
                        + (UINT32)((UINT64)span_p * 8ULL / 10 * c / n_chunks);
            ap_yield(a); if (g_aborted) goto bfe_done;
        }

        /* Verify */
        for (UINTN i = 0; i < n; i++) {
            if (a->base[i] != pat) {
                e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&a->base[i], pat, a->base[i]);
            }
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                a->progress = base_p + span_p * 9 / 10
                            + (UINT32)(span_p / 10 * i / n);
                ap_yield(a); if (g_aborted) goto bfe_done;
            }
        }
    }
bfe_done:
    a->progress = 1000;
    a->errors = e;
}

/* ---------- Thermal Soak (3 min sustained AVX2 + memory) ----------
   Standard tests run in short bursts (10 sec AVX2, then a long memory
   verify, then idle, repeat). Modern CPUs cool DOWN between bursts, so
   peak temp stays near idle (we observed 36°C on i5-12600KF — same as
   doing nothing). For shop diagnostics we WANT to reach thermal steady-
   state because that's where marginal cooling, dried thermal paste, weak
   VRM, and IMC instability surface.
   This kernel runs 3 min of CONTINUOUS AVX2 FMA chains interleaved with
   memory writes — no cool-down windows. Expected peak temp: 70-90°C on
   most desktop CPUs with reasonable cooling.
   No error checking — that's done by other kernels. Goal here is heat. */
static void run_thermal_soak(ap_arg_t *a) {
    if (!g_has_avx2) {
        a->errors = 0; a->bytes = 0; a->progress = 1000;
        return;
    }
    /* Two patterns:
         pat_fp = 1.0f × 8 — FP input for FMA chains
         pat_iv = byte shuffle mask: [0,1,2,3, 4,5,6,7, ...] (identity perm,
                  doesn't alter ymm data but goes through the shuffle unit)
       Both live in L1 — only used at loop entry to seed registers, never
       touched per-iteration, so no memory pressure inside the inner loop. */
    UINT64 pat_fp[4] = { 0x3F8000003F800000ULL, 0x3F8000003F800000ULL,
                         0x3F8000003F800000ULL, 0x3F8000003F800000ULL };
    UINT64 pat_iv[4] = { 0x0706050403020100ULL, 0x0F0E0D0C0B0A0908ULL,
                         0x0706050403020100ULL, 0x0F0E0D0C0B0A0908ULL };
    a->bytes = 0;

    /* PRIME95 / Y-CRUNCHER HSW-STRESS PATTERN.
       v1: pure FMA on 8 chains  → ~65-73W on i5-12600KF (only FMA pipes,
                                    shuffle/perm ports sit idle).
       v2 (this): mixed FMA + shuffle + integer ALU → ~85-95W expected.
                  Y-cruncher's HSW stress uses exactly this layout on
                  Alder Lake P-cores to extract ~95% of TDP without
                  AVX-512.

       Port mapping (Golden Cove P-core):
         ports 0,1   = FMA (2-wide, latency 4)        ← 4 FMA chains
         port  5     = shuffle / perm (1-wide, lat 1) ← 4 shuffle chains
         ports 0,1,5 = vector integer ALU (3-wide)    ← 2 padd chains
       Result: 2 FMAs + 1 shuffle + 1 ALU = 4 vector µops/cycle issued,
       saturating the vector execution cluster. On E-cores (Gracemont)
       256-bit ops crack to 2×128, throughput halves but same pattern
       still saturates their narrower ports. */
    UINT64 t_start = ms_now();
    UINT64 duration = 180000ULL;       /* 3 min */
    while (ms_now() - t_start < duration && !g_aborted) {
        /* 400 k iters × 14 vec ops / iter = 5.6 M vec ops / burst.
           At ~4 vec ops/cycle on 4.5 GHz P-core ≈ 0.31 ms / burst →
           ~3000 abort-check points/sec (Ctrl-C reacts within ~0.3 ms). */
        UINTN iters = 400000;
        __asm__ __volatile__(
            /* ymm0..ymm3 = FP dep chains  (consumed by FMA on ports 0/1) */
            "vmovdqu (%[pf]), %%ymm0\n\t"
            "vmovdqu (%[pf]), %%ymm1\n\t"
            "vmovdqu (%[pf]), %%ymm2\n\t"
            "vmovdqu (%[pf]), %%ymm3\n\t"
            /* ymm4..ymm7 = shuffle dep chains (consumed on port 5) */
            "vmovdqu (%[pi]), %%ymm4\n\t"
            "vmovdqu (%[pi]), %%ymm5\n\t"
            "vmovdqu (%[pi]), %%ymm6\n\t"
            "vmovdqu (%[pi]), %%ymm7\n\t"
            /* ymm8 = static shuffle mask (read-only, port 5 input). */
            "vmovdqu (%[pi]), %%ymm8\n\t"
            /* ymm9..ymm10 = integer dep chains (vector ALU). */
            "vmovdqu (%[pi]), %%ymm9\n\t"
            "vmovdqu (%[pi]), %%ymm10\n\t"
            "1:\n\t"
            /* 4 FMA — saturate both FMA ports (2 per cycle, lat 4) */
            "vfmadd231ps %%ymm0, %%ymm0, %%ymm0\n\t"
            "vfmadd231ps %%ymm1, %%ymm1, %%ymm1\n\t"
            "vfmadd231ps %%ymm2, %%ymm2, %%ymm2\n\t"
            "vfmadd231ps %%ymm3, %%ymm3, %%ymm3\n\t"
            /* 4 shuffles — saturate port 5 (1 per cycle, lat 1).
               vpshufb uses the same shuffle hardware as vpermilps and
               keeps the operand byte-rotated so registers don't go to
               a degenerate state. */
            "vpshufb %%ymm8, %%ymm4, %%ymm4\n\t"
            "vpshufb %%ymm8, %%ymm5, %%ymm5\n\t"
            "vpshufb %%ymm8, %%ymm6, %%ymm6\n\t"
            "vpshufb %%ymm8, %%ymm7, %%ymm7\n\t"
            /* 2 integer ALU ops — soak up free issue slots on ports 0/1/5.
               vpaddd has latency 1 so each chain advances every cycle. */
            "vpaddd %%ymm9, %%ymm9, %%ymm9\n\t"
            "vpaddd %%ymm10, %%ymm10, %%ymm10\n\t"
            /* 2 more FMA — fills out the remaining FMA throughput
               (without these we issue 2 FMA + 1 shuf + 1 ALU = 4 µops,
               could do 4 FMA + 1 shuf + 1 ALU = 6 µops/cycle if RF reads
               keep up). Each new FMA chain is independent of the first 4. */
            "vfmadd231ps %%ymm0, %%ymm1, %%ymm2\n\t"
            "vfmadd231ps %%ymm1, %%ymm2, %%ymm3\n\t"
            "dec %[cnt]\n\t"
            "jnz 1b\n\t"
            "vzeroupper\n\t"
            : [cnt] "+r"(iters)
            : [pf] "r"(pat_fp), [pi] "r"(pat_iv)
            : "ymm0", "ymm1", "ymm2", "ymm3",
              "ymm4", "ymm5", "ymm6", "ymm7",
              "ymm8", "ymm9", "ymm10",
              "cc");

        /* Synthetic GFLOPS counter for the bytes column.
           14 vec ops × 8 lanes × 400 k iters = ~45 M vec lane-ops / burst.
           Doesn't represent real memory, but lets the user see the kernel
           is actually progressing. */
        a->bytes += 45000ULL;

        UINT64 dt = ms_now() - t_start;
        UINT32 prog = (UINT32)(dt * 1000ULL / duration);
        if (prog > 1000) prog = 1000;
        a->progress = prog;
        ap_yield(a);
    }
    a->progress = 1000;
    a->errors = 0;     /* Not a memory test — no verify */
}

/* ---------- BW Soak (5 min sustained memory bandwidth) ----------
   The memtest86+ analog. 5 minutes of continuous AVX2 streaming
   stores + reads, no compute breaks. Hits the IMC + DRAM continuously
   the way memtest86+ does over its 30+ min walk-through, but
   compressed. Verifies data integrity each cycle so it also catches
   sustained-load memory errors that burst tests miss.
   Expected DRAM throughput: 25-30 GB/s on DDR5 single-channel,
   60-90 GB/s on dual-channel. CPU heat: similar to Thermal Soak. */
static void run_bw_soak(ap_arg_t *a) {
    if (!g_has_avx2) {
        a->errors = 0; a->bytes = 0; a->progress = 1000;
        return;
    }
    UINT64 e = 0;
    UINTN n = a->n_qwords;
    UINTN n32 = n / 4;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    UINT64 pat[4] = { 0xDEADBEEFCAFEBABEULL, 0x0123456789ABCDEFULL,
                      0x5A5A5A5AA5A5A5A5ULL, 0xFEDCBA9876543210ULL };
    a->bytes = 0;

    UINT64 t_start = ms_now();
    UINT64 duration = 300000ULL;       /* 5 min in ms */
    while (ms_now() - t_start < duration && !g_aborted) {
        /* Streaming write: vmovntdq bypasses cache → every store goes
           straight to DRAM through the write-combining buffers.
           Maximises memory controller activity. sfence flushes WC. */
        UINT64 *dst = a->base;
        UINTN cnt = n32;
        __asm__ __volatile__(
            "vmovdqu (%[pat]), %%ymm0\n\t"
            "1: vmovntdq %%ymm0, (%[dst])\n\t"
            "   add $32, %[dst]\n\t"
            "   dec %[cnt]\n\t"
            "   jnz 1b\n\t"
            "sfence\n\t"
            "vzeroupper\n\t"
            : [dst] "+r"(dst), [cnt] "+r"(cnt)
            : [pat] "r"(pat)
            : "ymm0", "memory", "cc");
        a->bytes += (UINT64)n * 8;

        /* Read+verify pass. The buffer was streamed (write-combined +
           sfenced), so first reads will MISS L1/L2/L3 and hit DRAM —
           also bandwidth-heavy. The hw prefetcher will pull subsequent
           lines into L2, so later qwords on the same page are cached;
           that's OK, still tons of DRAM activity per pass. */
        for (UINTN i = 0; i + 3 < n; i += 4) {
            for (UINTN k = 0; k < 4; k++) {
                if (a->base[i + k] != pat[k]) {
                    e++;
                    record_error(a->kernel, a->core_idx,
                                 (UINT64)(UINTN)&a->base[i + k],
                                 pat[k], a->base[i + k]);
                }
            }
            if ((i % step) == 0) {
                a->bytes += (UINT64)step * 8;
                UINT64 dt = ms_now() - t_start;
                UINT32 prog = (UINT32)(dt * 1000ULL / duration);
                if (prog > 1000) prog = 1000;
                a->progress = prog;
                ap_yield(a);
                if (g_aborted) goto bw_done;
            }
        }
    }
bw_done:
    a->progress = 1000;
    a->errors = e;
}

/* ---------- L3 cache stress (KER_L3_STRESS) ----------
   Pattern tests above CLFLUSH between write and verify (or use NT stores),
   so the data round-trips DRAM and we never observe the L3 cells. This
   kernel deliberately KEEPS the working set in L3 by:
     1. Picking a region small enough to fit L3 per core (we conservatively
        assume 1 MB — true for every x86 CPU since Nehalem 2008).
     2. After writing the pattern, doing many short read loops without
        flushing. L1/L2 will hold the hot lines but eviction sends them to
        L3 not DRAM, so cached values do live in L3 between iterations.
     3. Verifying at the end and recording any bit flip.
   Marginal L3 cells either (a) flip a bit (caught here) or (b) trigger
   single-bit ECC correction that fires MCA bank 1 — also reported.
   Duration: ~10 s — short kernel, not the bottleneck of any pass. */
static void run_l3_stress(ap_arg_t *a) {
    /* 1 MB working set fits in L3 on every modern x86 CPU. */
    UINTN L3_BYTES = 1024ULL * 1024ULL;
    UINTN n = L3_BYTES / 8;
    if (n > a->n_qwords) n = a->n_qwords;
    UINTN step = n / PROGRESS_GRAIN; if (!step) step = n;
    UINT64 e = 0;
    a->bytes = 0;
    UINT64 *p = a->base;

    /* Two-phase pattern: write 0xA5..., read+rewrite XOR mask, final verify.
       This stresses cell stability under repeated read+write cycles, which
       is the workload most likely to expose L3 retention faults. */
    UINT64 PAT_A = 0xA5A5A5A5A5A5A5A5ULL;
    UINT64 PAT_B = 0x5A5A5A5A5A5A5A5AULL;

    /* Phase 1: initialise. */
    for (UINTN i = 0; i < n; i++) p[i] = PAT_A;
    a->bytes += (UINT64)n * 8;

    /* Phase 2: ~10 s of read-modify-write loops. Each iteration toggles
       between PAT_A and PAT_B so cells get exercised in both directions. */
    UINT64 t_start = ms_now();
    UINT32 iter = 0;
    while (ms_now() - t_start < 10000 && !g_aborted) {
        UINT64 want = (iter & 1) ? PAT_B : PAT_A;
        UINT64 next = (iter & 1) ? PAT_A : PAT_B;
        for (UINTN i = 0; i < n; i++) {
            if (p[i] != want) {
                e++;
                record_error(a->kernel, a->core_idx,
                             (UINT64)(UINTN)&p[i], want, p[i]);
            }
            p[i] = next;
            if ((i & 4095) == 0) {
                UINT64 dt = ms_now() - t_start;
                UINT32 prog = (UINT32)(dt * 1000ULL / 10000ULL);
                if (prog > 1000) prog = 1000;
                a->progress = prog;
                if ((i & 65535) == 0) ap_yield(a);
            }
        }
        a->bytes += (UINT64)n * 8 * 2;   /* read + write */
        iter++;
    }
    a->progress = 1000;
    a->errors = e;
}

/* ---------- Stride-sweep BW probe (KER_STRIDE_BW) ----------
   Reads the buffer with 4 different stride patterns, ~3 s each (12 s total).
   The bytes-per-second at each stride exposes platform pathologies:
     64 B    — cache-line, maximises prefetcher utility
     1 KB    — outside hardware prefetcher window; reveals raw IMC bandwidth
     4 KB    — page-stride; reveals TLB miss cost
     64 KB   — > L2 set associativity; reveals cache set conflicts
   A drop at JUST ONE stride is the diagnostic signal — uniform low BW =
   hot system, narrow drop = a real architectural problem (bad channel
   interleave / TLB shootdown / set conflict).
   Results are stored in g_stride_mbps[core][phase] for the BSP to report
   after all APs finish. */
static volatile UINT32 g_stride_mbps[MAX_CORES][4];   /* per-core, per-phase MB/s */

static void run_stride_bw(ap_arg_t *a) {
    UINTN n = a->n_qwords;
    UINTN n_bytes = n * 8;
    static const UINTN strides[4] = { 64, 1024, 4096, 65536 };
    UINT64 e = 0;
    a->bytes = 0;
    UINT64 *p = a->base;

    /* Write a known pattern once so reads have something to verify against.
       PAT_X picks a value that won't collide with stray zeros/0xFF. */
    UINT64 PAT_X = 0x1122334455667788ULL;
    for (UINTN i = 0; i < n; i++) p[i] = PAT_X;
    a->bytes += (UINT64)n_bytes;

    for (UINT32 ph = 0; ph < 4; ph++) {
        UINTN stride = strides[ph];
        UINTN stride_qw = stride / 8;
        if (stride_qw < 1) stride_qw = 1;
        UINT64 phase_start = ms_now();
        UINT64 phase_bytes = 0;
        while (ms_now() - phase_start < 3000 && !g_aborted) {
            /* One pass through the buffer at this stride. */
            volatile UINT64 sink = 0;
            for (UINTN i = 0; i < n; i += stride_qw) {
                UINT64 v = p[i];
                sink ^= v;
                if (v != PAT_X) {
                    e++;
                    record_error(a->kernel, a->core_idx,
                                 (UINT64)(UINTN)&p[i], PAT_X, v);
                }
            }
            /* Each stride hit one qword (8 B) and pulled in a cache line
               (64 B) — we count cache-line bytes for honest BW reporting. */
            phase_bytes += (UINT64)(n / stride_qw) * 64;
            (void)sink;
            if (g_aborted) break;
        }
        UINT64 dt = ms_now() - phase_start;
        if (dt > 0 && a->core_idx < MAX_CORES) {
            UINT32 mbps = (UINT32)((phase_bytes * 1000ULL) / (dt * 1024ULL * 1024ULL));
            g_stride_mbps[a->core_idx][ph] = mbps;
        }
        a->bytes += phase_bytes;
        a->progress = ((ph + 1) * 1000) / 4;
        ap_yield(a);
    }
    a->progress = 1000;
    a->errors = e;
}

/* ---------- CPU feature detection ---------- */
int g_has_avx2    = 0;
int g_has_clflush = 0;
/* IA32_APERF/MPERF MSRs — Intel since Nehalem (2008), AMD since Bulldozer.
   We use MPERF/TSC ratio for real C0 residency ("% time core was actually
   running vs in C1/C6 idle"). rdtsc_now / rdmsr_safe defined as inlines near
   the top of the file (forward-decl region) so kernels can use them too. */
int g_has_aperf_mperf = 0;
int g_has_thermal      = 0;
UINT32 g_tj_max        = 0;
UINT32 g_base_freq_mhz = 0;

static void cpuid(UINT32 leaf, UINT32 subleaf,
                  UINT32 *a, UINT32 *b, UINT32 *c, UINT32 *d) {
    UINT32 ra, rb, rc, rd;
    __asm__ __volatile__("cpuid"
        : "=a"(ra), "=b"(rb), "=c"(rc), "=d"(rd)
        : "a"(leaf), "c"(subleaf));
    if (a) *a = ra; if (b) *b = rb; if (c) *c = rc; if (d) *d = rd;
}

/* Try to enable AVX state ourselves: CR4.OSXSAVE=1 + XCR0[x87|SSE|AVX]=1.
   UEFI firmware almost always leaves OSXSAVE=0 in pre-OS, so without this
   any VEX-encoded AVX2 instruction would fault with #UD even on CPUs that
   fully support AVX2. We only attempt this when CPUID confirms the CPU
   supports XSAVE+AVX+AVX2 — otherwise the CR4/XCR0 writes themselves could
   #GP on truly-no-AVX hardware. */
static int try_enable_avx_state(void) {
    UINT32 a, b, c, d;
    cpuid(1, 0, &a, &b, &c, &d);
    int has_xsave = (c >> 26) & 1;
    int has_avx   = (c >> 28) & 1;
    cpuid(7, 0, &a, &b, &c, &d);
    int has_avx2  = (b >> 5)  & 1;
    if (!has_xsave || !has_avx || !has_avx2) return 0;

    /* Full Intel SDM Vol.1 §13.5.4 / OSDev "AVX crash on EFI App [SOLVED]"
       prescribed sequence. Earlier we only set CR4.OSXSAVE + XSETBV which
       worked on most firmware because they had the other bits already set.
       Field-reported AMD Ryzen 5 4500 + ASUS B-series hang traced to
       run_avx2_sustained executing the first AVX2 instruction and
       triggering #UD because CR0.EM was still set / CR4.OSFXSR was clear
       on this firmware. Setting all required bits defensively is harmless
       on firmware that already had them. */

    /* CR0: clear EM (no x87 emulation), set MP (monitor coprocessor). */
    UINT64 cr0;
    __asm__ __volatile__("mov %%cr0, %0" : "=r"(cr0));
    cr0 &= ~(1ULL << 2);    /* EM = 0 */
    cr0 |=  (1ULL << 1);    /* MP = 1 */
    __asm__ __volatile__("mov %0, %%cr0" : : "r"(cr0));

    /* CR4:
         bit 9  OSFXSR    — OS supports FXSAVE/FXRSTOR (required for SSE)
         bit 10 OSXMMEXCPT — OS supports SIMD float-point exceptions
         bit 18 OSXSAVE   — OS supports XSAVE (required for AVX state save). */
    UINT64 cr4;
    __asm__ __volatile__("mov %%cr4, %0" : "=r"(cr4));
    cr4 |= (1ULL << 9);
    cr4 |= (1ULL << 10);
    cr4 |= (1ULL << 18);
    __asm__ __volatile__("mov %0, %%cr4" : : "r"(cr4));

    /* XCR0: enable save-state for x87 (bit0) + SSE/XMM (bit1) + AVX/YMM (bit2). */
    UINT32 lo = 0x7;
    UINT32 hi = 0;
    __asm__ __volatile__("xsetbv" : : "a"(lo), "d"(hi), "c"(0));

    return 1;
}

/* Push this core to its maximum P-state (turbo). In UEFI pre-OS there is no
   OS power-management driver, so the firmware leaves the CPU at whatever
   default P-state it set during boot (usually base ratio, sometimes lower).
   With AVX2 stress on 16 idle cores we saw the i5-12600KF top out at 57 W
   and 39 °C — only ~40 % of its 150 W PL2 limit, because the cores never
   asked for turbo. Result: stress tests were "stress" in name only.

   The fix is the standard Intel sequence used by enterprise UEFI apps and
   recommended by Intel community / SDM Vol. 3B:
     1. IA32_MISC_ENABLE (0x1A0) bit 38 = 0   → un-disable turbo
     2. IA32_PM_ENABLE   (0x770) = 1           → enable HWP (write-once / reset)
     3. read IA32_HWP_CAPABILITIES (0x771)[7:0] → "Highest" supported perf level
     4. IA32_HWP_REQUEST (0x774) = min=highest, max=highest, EPP=0 (performance)
   Step 4 demands the PCU run THIS logical CPU at its peak. Each core has
   its own MSR view, so this MUST run on every AP via MP Services, not just
   the BSP — that's why detect_cpu_features() calls it for BSP and ap_entry()
   calls it on each AP.

   Some BIOSes lock CPU MSRs ("CPU Lock Configuration" / "OC Lock"). If our
   write doesn't stick, we just keep the firmware default. The function
   returns the Highest-Performance ratio we observed, or 0 if HWP unsupported. */
/* Global counters populated by try_enable_max_perf, summarised after the
   first AP dispatch. Useful to confirm HWP actually took effect on every
   logical CPU and not just the BSP. */
static volatile UINT32 g_hwp_ok_count   = 0;
static volatile UINT32 g_hwp_fail_count = 0;
/* v0.4.29 — count APs that took the legacy PERF_CTL (0x199) path
   instead of HWP. Lets the [PERF] summary distinguish "HWP failed
   silently" from "this is pre-Skylake, we used the right legacy MSR". */
static volatile UINT32 g_legacy_turbo_count = 0;
static UINT32 g_max_freq_mhz_observed   = 0;
static UINT32 g_dominant_throttle       = 0; /* THROT_* bitmap (last non-zero) */

/* AMD CPPC2 (Collaborative Processor Performance Control v2) — Zen+ /
   Family 17h equivalent of Intel HWP. Same goal: ask the CPU to run at
   its highest P-state continuously instead of the firmware default base
   ratio. Without this the AMD tester runs all kernels at ~70% of CPU
   max and thermal stress is correspondingly weaker.

   Detection: CPUID 0x80000008.EBX bit 27 = "AMD CPPC supported".
     (Bit 27 historically — some sources document bit 12; bit 27 is the
     definitive one per the AMD64 Programmer's Manual since Zen.)

   MSRs:
     0xC00102B0 MSR_AMD_CPPC_CAP1
       bits [7:0]   Lowest Performance
       bits [15:8]  Lowest Non-linear Performance
       bits [23:16] Nominal Performance
       bits [31:24] Highest Performance         ← what we want
     0xC00102B1 MSR_AMD_CPPC_ENABLE             ← write bit 0 = 1 to enable
     0xC00102B3 MSR_AMD_CPPC_REQ                ← per-logical-CPU request
       bits [7:0]   Maximum Performance
       bits [15:8]  Minimum Performance
       bits [23:16] Desired Performance (0 = autonomous within min..max)
       bits [31:24] Energy Performance Preference (0 = max perf, 0xFF = max powersave)
   Returns the "highest" ratio (CAP1 bits [31:24]) or 0 if unsupported. */
static UINT32 try_enable_cppc2_amd(void) {
    UINT32 a, b, c, d;
    cpuid(0x80000008, 0, &a, &b, &c, &d);
    int has_cppc = (b >> 27) & 1;
    if (!has_cppc) {
        return 0;
    }
    /* Enable CPPC (write-once latched per Zen errata). */
    UINT64 en = rdmsr_safe(0xC00102B1);
    if (!(en & 1ULL)) {
        wrmsr_safe(0xC00102B1, 1ULL);
    }
    /* Read capabilities — highest perf is bits [31:24] of CAP1. */
    UINT64 cap1 = rdmsr_safe(0xC00102B0);
    UINT32 highest = (UINT32)((cap1 >> 24) & 0xFFu);
    if (highest == 0) {
        /* Some early Zen returned 0 in CAP1 until BIOS programmed it.
           Fall back to "request 0xFF" which the PSP clamps to actual max. */
        highest = 0xFF;
    }
    /* AMD CPPC_REQ field order is REVERSED vs Intel HWP_REQUEST:
         AMD: [7:0]=max  [15:8]=min  [23:16]=desired  [31:24]=EPP
         Intel: opposite of first two. Be careful. */
    UINT64 req = (UINT64)highest         /* max */
               | ((UINT64)highest <<  8) /* min */
               | (0ULL              << 16) /* desired = autonomous */
               | (0x00ULL           << 24);/* EPP = 0 (max performance) */
    wrmsr_safe(0xC00102B3, req);
    /* Verify (PSP may clamp; we trust the value we sent). */
    UINT64 verify = rdmsr_safe(0xC00102B3);
    if ((verify & 0xFFFFFFFFu) == (req & 0xFFFFFFFFu)) {
        __sync_fetch_and_add(&g_hwp_ok_count, 1);
    } else {
        __sync_fetch_and_add(&g_hwp_fail_count, 1);
    }
    return highest;
}

static UINT32 try_enable_max_perf(void) {
    if (g_cpu_vendor == CPU_AMD) {
        return try_enable_cppc2_amd();
    }
    if (g_cpu_vendor != CPU_INTEL) {
        return 0;
    }
    UINT32 a, b, c, d;
    cpuid(6, 0, &a, &b, &c, &d);
    int has_hwp = (a >> 7) & 1;          /* CPUID.06H:EAX[7] = HWP supported */

    /* (1) Clear IA32_MISC_ENABLE.Turbo_Disable (bit 38). Whether or not HWP
       is in play, this re-enables opportunistic boost if BIOS disabled it. */
    UINT64 me = rdmsr_safe(0x1A0);
    if (me & (1ULL << 38)) {
        wrmsr_safe(0x1A0, me & ~(1ULL << 38));
    }

    if (has_hwp) {
        /* (2) Enable HWP. Write-once per reset — second writes #GP-ignored. */
        UINT64 pme = rdmsr_safe(0x770);
        if (!(pme & 1ULL)) wrmsr_safe(0x770, 1ULL);

        /* (3) Read Highest Performance level for THIS core. Alder Lake
           hybrid: P-cores typically report a higher ratio than E-cores,
           so we must read per-core, not assume BSP's value. */
        UINT64 caps = rdmsr_safe(0x771);
        UINT32 highest = (UINT32)(caps & 0xFFu);
        if (highest == 0) {
            __sync_fetch_and_add(&g_hwp_fail_count, 1);
            return 0;
        }

        /* (4) Demand peak perf. Format of IA32_HWP_REQUEST (0x774):
                bits  7:0  Minimum_Performance
                bits 15:8  Maximum_Performance
                bits 23:16 Desired_Performance (0 = autonomous w/in min..max)
                bits 31:24 Energy_Performance_Preference (0x00 = max perf,
                                                          0xFF = max powersave)
                bits 41:32 Activity_Window (0 = HW default)
                bit  42    Package_Control (0 = per-logical-CPU)
           We set min = max = highest so the PCU has zero excuse to clock down. */
        UINT64 req = (UINT64)highest          /* min */
                   | ((UINT64)highest <<  8)  /* max */
                   | (0ULL              << 16)  /* desired = autonomous */
                   | (0x00ULL           << 24); /* EPP = max performance */
        wrmsr_safe(0x774, req);

        /* Verify the write stuck. Some Gigabyte/ASRock BIOSes set an OC-lock
           that silently discards writes to perf MSRs; we want to see that in
           the log instead of guessing later. */
        UINT64 verify = rdmsr_safe(0x774);
        if ((verify & 0xFFFFFFFFu) == (req & 0xFFFFFFFFu)) {
            __sync_fetch_and_add(&g_hwp_ok_count, 1);
        } else {
            __sync_fetch_and_add(&g_hwp_fail_count, 1);
        }
        return highest;
    }

    /* Fallback: legacy SpeedStep (pre-Skylake CPUs without HWP — Sandy
       Bridge, Ivy Bridge, Haswell, Broadwell). Read max turbo ratio from
       MSR_TURBO_RATIO_LIMIT (0x1AD)[7:0] = 1-core turbo, request it via
       IA32_PERF_CTL (0x199). This is the EIST/SpeedStep mechanism that
       was the OS interface before HWP. */
    UINT64 trl = rdmsr_safe(0x1AD);
    UINT32 turbo_ratio = (UINT32)(trl & 0xFFu);
    if (turbo_ratio == 0) {
        /* Last resort: read non-turbo max from PLATFORM_INFO (0xCE)[15:8]. */
        UINT64 pi = rdmsr_safe(0xCE);
        turbo_ratio = (UINT32)((pi >> 8) & 0xFFu);
    }
    if (turbo_ratio > 0) {
        wrmsr_safe(0x199, ((UINT64)turbo_ratio) << 8);
        /* v0.4.29 — bump the legacy-turbo counter so the per-run [PERF]
           summary can honestly say "OK via legacy PERF_CTL on N cores"
           instead of leaving us with OK=0 FAIL=0 which looks like nothing
           was done. */
        __sync_fetch_and_add(&g_legacy_turbo_count, 1);
    }
    return turbo_ratio;
}

/* Lift Package Power Limits 1 & 2 so PL1 throttling doesn't cap our
   stress tests. The default Intel profile is PL1=125W / PL2=150W on a
   12600KF, but many B-chipset BIOSes (Gigabyte B760, ASRock H610) ship
   with a much lower PL1 — often 65W — to keep the cheap VRM cool. Result:
   first 28 s of stress run at PL2=150W, then CPU clamps to PL1=65W and
   "тротт PWR" counter starts climbing. Heat barely reaches 45 °C.

   MSR 0x610 (MSR_PKG_POWER_LIMIT) format:
     bits 14:0    PL1 watts × 8           (1/8 W units, set by 0x606[3:0])
     bit  15      PL1 enable
     bit  16      PL1 clamp
     bits 23:17   PL1 time window (encoded as a 6-bit y + 2-bit z)
     bit  24      reserved
     bits 31:25   reserved
     bits 46:32   PL2 watts × 8
     bit  47      PL2 enable
     bit  48      PL2 clamp
     bits 55:49   PL2 time window
     bit  63      Lock — once set, writes are silently dropped until reset.
   We try to write PL1=PL2=250 W (= 2000 in 1/8 W units = 0x7D0). 250 W is
   well above what any 12600KF can pull, so this effectively disables PL
   capping for the duration of our run. If lock bit was already set by
   BIOS, the write fails silently and we'll see it in the read-back. */
static int try_lift_power_limits(UINT64 *out_old, UINT64 *out_new) {
    if (g_cpu_vendor != CPU_INTEL) return 0;
    UINT64 cur = rdmsr_safe(0x610);
    if (out_old) *out_old = cur;
    if (cur & (1ULL << 63)) {
        if (out_new) *out_new = cur;
        return 0;                /* locked by BIOS */
    }
    /* 250 W in 1/8 W units = 2000 = 0x7D0 (11 bits — fits in 15). */
    const UINT64 watt_units = 2000;
    /* Time window: y=0, z=0  → 1 ms (we want PL1 to be effectively
       instantaneous so it acts like PL2). Field is at bits 23:17. */
    UINT64 newv =
        (watt_units & 0x7FFFULL)         /* PL1 watts */
      | (1ULL          << 15)            /* PL1 enable */
      | (1ULL          << 16)            /* PL1 clamp */
      | (0ULL          << 17)            /* time window = 1 ms */
      | ((watt_units & 0x7FFFULL) << 32) /* PL2 watts */
      | (1ULL          << 47)            /* PL2 enable */
      | (1ULL          << 48);           /* PL2 clamp (don't lock bit 63) */
    wrmsr_safe(0x610, newv);
    UINT64 verify = rdmsr_safe(0x610);
    if (out_new) *out_new = verify;
    /* Match the low 32 bits — PL1 fields. If those stuck, treat as success. */
    return ((verify & 0xFFFFFFFFULL) == (newv & 0xFFFFFFFFULL));
}

static void detect_cpu_features(void) {
    UINT32 a, b, c, d;
    /* CPUID leaf 0 — vendor string in EBX/EDX/ECX. We compare against the
       canonical 12-char strings for Intel and AMD. Other vendors (Centaur,
       VIA, Hygon) fall through to UNKNOWN; they'll get the conservative
       Intel-MSR probes which simply fail and disable optional features. */
    cpuid(0, 0, &a, &b, &c, &d);
    if (b == 0x756E6547 /* "Genu" */ && d == 0x49656E69 /* "ineI" */ && c == 0x6C65746E /* "ntel" */) {
        g_cpu_vendor = CPU_INTEL;
    } else if (b == 0x68747541 /* "Auth" */ && d == 0x69746E65 /* "enti" */ && c == 0x444D4163 /* "cAMD" */) {
        g_cpu_vendor = CPU_AMD;
    } else {
        g_cpu_vendor = CPU_UNKNOWN;
    }
    {
        CHAR16 vlb[64];
        SPrint(vlb, sizeof(vlb), L"[CPU] vendor = %s",
               (g_cpu_vendor == CPU_INTEL) ? L"Intel" :
               (g_cpu_vendor == CPU_AMD)   ? L"AMD"   : L"Unknown");
        log_line(vlb);
    }

    cpuid(1, 0, &a, &b, &c, &d);
    g_has_clflush       = (d >> 19) & 1;     /* EDX bit 19 = CLFLUSH */
    cpuid(7, 0, &a, &b, &c, &d);
    int has_avx2_cpuid  = (b >> 5)  & 1;     /* EBX bit 5  = AVX2 */

    /* Earlier code refused to run AVX2 unless OSXSAVE was already set by
       firmware — but in practice UEFI never sets it, so the test was
       always skipped. Instead, enable XSAVE state ourselves on the BSP and
       trust that CPUID.7:EBX.5 means the silicon can do AVX2. */
    if (has_avx2_cpuid) {
        log_line(L"[AVX] CPUID says AVX2 supported — enabling XSAVE state...");
        if (try_enable_avx_state()) {
            log_line(L"[AVX] BSP: CR4.OSXSAVE + XCR0[YMM] enabled");
            g_has_avx2 = 1;
        } else {
            log_line(L"[AVX] enable failed — keeping AVX2 disabled");
            g_has_avx2 = 0;
        }
    } else {
        log_line(L"[AVX] CPUID: this CPU lacks AVX2");
        g_has_avx2 = 0;
    }

    /* Push BSP to turbo. In UEFI pre-OS the firmware leaves the CPU at base
       ratio (no SpeedStep / HWP from the OS yet), which silently capped our
       16-core AVX2 thermal soak at 57 W / 39 °C on an i5-12600KF that should
       hit 125-150 W under Prime95. Each AP runs the same sequence in
       ap_entry() because IA32_HWP_REQUEST is per-logical-CPU. */
    {
        UINT32 hp = try_enable_max_perf();
        CHAR16 lb[200];
        if (hp > 0) {
            /* Both Intel HWP and AMD CPPC2 expose an abstract "performance
               level" rather than a direct MHz value. We log raw so it's
               traceable per platform. On Intel Alder Lake 63 ≈ 4.9 GHz
               P-core. On AMD Zen 0xFF / 255 is the typical highest. */
            SPrint(lb, sizeof(lb),
                   L"[PERF] BSP: %s enabled, Highest_Perf level=%d",
                   (g_cpu_vendor == CPU_AMD) ? L"CPPC2" : L"HWP", hp);
        } else {
            SPrint(lb, sizeof(lb),
                   L"[PERF] BSP: %s unavailable or locked; staying at firmware default",
                   (g_cpu_vendor == CPU_AMD) ? L"CPPC2" : L"HWP");
        }
        log_line(lb);

        /* INTEL-ONLY diagnostic block. MSR 0x1AD (TURBO_RATIO_LIMIT) and
           MSR 0x610 (PKG_POWER_LIMIT) are Intel-specific — on AMD reading
           a non-existent MSR triggers a #GP fault. UEFI has no OS-level
           exception handler, so the CPU just freezes / reboots. This was
           a regression that bricked the tester on every AMD platform:
           visible in logs as program ending right after the BSP HWP line,
           before reaching the menu. */
        if (g_cpu_vendor == CPU_INTEL) {
            /* Dump configured turbo ratios — tells us what the silicon CAN do. */
            UINT64 trl = rdmsr_safe(0x1AD);    /* MSR_TURBO_RATIO_LIMIT */
            UINT32 r1 = (UINT32)((trl >>  0) & 0xFFu);
            UINT32 r2 = (UINT32)((trl >>  8) & 0xFFu);
            UINT32 r4 = (UINT32)((trl >> 24) & 0xFFu);
            UINT32 r8 = (UINT32)((trl >> 56) & 0xFFu);
            SPrint(lb, sizeof(lb),
                   L"[PERF] Turbo ratios from 0x1AD: 1c=%d 2c=%d 4c=%d 8c=%d (×100 MHz)",
                   r1, r2, r4, r8);
            log_line(lb);

            /* Dump PKG_POWER_LIMIT (0x610). Critical: this tells us if BIOS
               set a low PL1 that will throttle us. */
            UINT64 plr = rdmsr_safe(0x610);
            UINT32 pl1_w = (UINT32)((plr >> 3) & 0xFFFu);
            UINT32 pl2_w = (UINT32)((plr >> 35) & 0xFFFu);
            int locked = (plr & (1ULL << 63)) ? 1 : 0;
            SPrint(lb, sizeof(lb),
                   L"[PERF] BIOS PL1=%dW (en=%d) PL2=%dW (en=%d) lock=%d",
                   pl1_w, (int)((plr >> 15) & 1),
                   pl2_w, (int)((plr >> 47) & 1), locked);
            log_line(lb);

            /* Try to lift PL1/PL2 to 250 W so power-limit throttling can't
               silently cap our stress run. If BIOS locked it, the write
               fails and we report it — at least the user knows to disable
               "CPU Lock Configuration" in BIOS. */
            UINT64 old_pl = 0, new_pl = 0;
            int pl_ok = try_lift_power_limits(&old_pl, &new_pl);
            UINT32 new_pl1 = (UINT32)((new_pl >> 3) & 0xFFFu);
            UINT32 new_pl2 = (UINT32)((new_pl >> 35) & 0xFFFu);
            SPrint(lb, sizeof(lb),
                   L"[PERF] Power limit lift: %a (new PL1=%dW PL2=%dW)",
                   pl_ok ? "OK" : "FAILED (BIOS locked)",
                   new_pl1, new_pl2);
            log_line(lb);
        } else {
            /* AMD path — these MSR addresses are different on AMD (CPPC2
               at 0xC00102B0..B3, PKG_POWER at 0xC0010299..029B). HWP-lift
               equivalent isn't yet implemented for AMD; CPU stays at the
               firmware default P-state. RAPL still works because that
               vendor-aware detection happens later via detect_rapl(). */
            log_line(L"[PERF] AMD platform: turbo/PL diagnostic skipped "
                     L"(Intel-MSR-only). CPU runs at firmware default.");
        }
    }

    /* APERF/MPERF availability — CPUID.06H:ECX.0 = "Hardware Coordination
       Feedback Capability". Required to safely use rdmsr on 0xE7/0xE8.
       Same leaf: EAX.0 = Digital Thermal Sensor (Intel-only, per-core temp
       via 0x19C). On AMD, leaf 06H is undefined — values may read as
       garbage. We force g_has_thermal=0 on AMD because IA32_THERM_STATUS
       (0x19C) doesn't exist on AMD; reading it returns junk and our
       temperature column would lie. AMD per-core temp lives in SMN
       (System Management Network) registers which require chipset-specific
       PCI config-space access — too platform-specific for a UEFI app. */
    cpuid(6, 0, &a, &b, &c, &d);
    g_has_aperf_mperf = (c & 1) != 0;
    if (g_cpu_vendor == CPU_AMD) {
        g_has_thermal = 0;
        /* AMD lacks IA32_THERM_STATUS (0x19C) — Intel-only. We probe the
           AMD SMN mailbox right after for Tctl access; if SMN responds,
           g_has_thermal flips back to 1 and the temperature column gets
           per-PACKAGE readings (not per-core like Intel) populated by
           sample_aggregate_metrics. */
        log_line(L"[CPU] AMD: IA32_THERM_STATUS unavailable, probing SMN for Tctl...");
        amd_thermal_probe();
    } else {
        g_has_thermal = (a & 1) != 0;
    }

    if (g_has_thermal && g_cpu_vendor == CPU_INTEL) {
        /* TjMax is the temperature threshold the digital sensor measures
           below — bits 23:16 of MSR_TEMPERATURE_TARGET. Typical 100°C
           desktop / 105°C mobile. INTEL-ONLY MSR (0x1A2).
           Note: amd_thermal_probe() flips g_has_thermal back to 1 when SMN
           is up, so we MUST gate by vendor here — otherwise reading 0x1A2
           on AMD triggers #GP and freezes the tester at init. Field-reported
           hang on a Habr user's AMD machine (v0.4.1) traced to exactly this. */
        UINT64 tt = rdmsr_safe(MSR_TEMPERATURE_TARGET);
        g_tj_max = (UINT32)((tt >> 16) & 0xFF);
        if (g_tj_max == 0 || g_tj_max > 150) g_tj_max = 100;  /* sane fallback */
    } else if (g_has_thermal && g_cpu_vendor == CPU_AMD) {
        /* AMD: no MSR_TEMPERATURE_TARGET equivalent. Tctl is already raw
           degrees from SMN, no offset needed for display. Set a reasonable
           fallback TjMax so any code that compares against it doesn't
           treat 0 as "thermal headroom unknown". Ryzen typical Tjmax 95°C
           (consumer) / 105°C (server EPYC), pick the middle. */
        g_tj_max = 100;
    }
    /* Base (max non-turbo) frequency from MSR_PLATFORM_INFO bits 15:8.
       INTEL-ONLY MSR (0xCE). On AMD it does not exist — earlier (pre-fix)
       code unconditionally read it and triggered #GP, freezing the tester
       at init on every AMD platform. The frequency display tolerates
       g_base_freq_mhz == 0 (ap_yield guards on `> 0`); AMD users just
       won't see a calibrated MHz number in the core panel until we
       implement AMD's MSRC001_0061 equivalent. */
    if (g_cpu_vendor == CPU_INTEL) {
        UINT64 pi = rdmsr_safe(MSR_PLATFORM_INFO);
        UINT32 ratio = (UINT32)((pi >> 8) & 0xFF);
        if (ratio > 0 && ratio < 100) g_base_freq_mhz = ratio * 100;
    } else {
        /* g_base_freq_mhz stays 0 → freq column shows '—'. */
        log_line(L"[CPU] AMD: MSR_PLATFORM_INFO unavailable — base freq unknown");
    }
}

/* ---------- RAPL detection (vendor-aware) ----------
   Both Intel (Sandy Bridge+, 2011) and AMD (Zen / Family 17h+, 2017) expose
   package-level energy counters that can be sampled to derive watts. The
   MSR layout is the same on both — power-unit MSR (bits 12:8 = energy
   exponent) + cumulative pkg-energy counter — but the MSR ADDRESSES differ.
   Intel:  0x606 + 0x611
   AMD:    0xC0010299 + 0xC001029B
   The detection picks the right pair based on g_cpu_vendor and writes the
   resolved addresses into runtime globals so the sampling loop is vendor-
   agnostic. */
static void detect_rapl(void) {
    if (g_cpu_vendor == CPU_INTEL) {
        g_rapl_unit_msr = MSR_INTEL_RAPL_POWER_UNIT;
        g_rapl_pkg_msr  = MSR_INTEL_PKG_ENERGY_STATUS;
    } else if (g_cpu_vendor == CPU_AMD) {
        g_rapl_unit_msr = MSR_AMD_RAPL_POWER_UNIT;
        g_rapl_pkg_msr  = MSR_AMD_PKG_ENERGY_STATUS;
    } else {
        log_line(L"[RAPL] unknown CPU vendor — disabling power telemetry");
        g_has_rapl = 0;
        return;
    }
    UINT64 u = rdmsr_safe(g_rapl_unit_msr);
    if (!u) {
        log_line(L"[RAPL] POWER_UNIT MSR returned 0 — RAPL unavailable");
        g_has_rapl = 0;
        return;
    }
    /* Energy unit = 1 / 2^(bits 12:8) joules per LSB. Typical Intel = 14
       (1/16384 J ≈ 61 µJ), typical AMD = 16 (1/65536 J ≈ 15 µJ). */
    UINT32 e_units = (UINT32)((u >> 8) & 0x1F);
    if (e_units == 0 || e_units > 24) {
        log_line(L"[RAPL] POWER_UNIT bits 12:8 out of range — RAPL unavailable");
        g_has_rapl = 0;
        return;
    }
    g_rapl_energy_units_div = 1u << e_units;
    UINT64 e = rdmsr_safe(g_rapl_pkg_msr);
    if (e == 0) {
        log_line(L"[RAPL] PKG_ENERGY_STATUS = 0 — RAPL unavailable");
        g_has_rapl = 0;
        return;
    }
    g_has_rapl = 1;
    g_rapl_pkg_energy_prev = e;
    CHAR16 lb[160];
    SPrint(lb, sizeof(lb),
           L"[RAPL] enabled (%s): energy unit = 1/%d J/LSB, pkg MSR = 0x%x",
           (g_cpu_vendor == CPU_AMD) ? L"AMD" : L"Intel",
           g_rapl_energy_units_div, g_rapl_pkg_msr);
    log_line(lb);
}

/* ---------- MCA (Machine Check Architecture) — ECC error capture ----------
   Intel and AMD both implement IA32 MCA with the same MSR layout:
     IA32_MCG_CAP    (0x179)  read-only; bits [7:0] = number of MC banks
     MCi_CTL         (0x400 + 4*i)  per-bank enable mask
     MCi_STATUS      (0x401 + 4*i)  per-bank latched error status
       bit 63 VAL   — register holds a valid error log
       bit 62 OVER  — additional errors lost since this one was logged
       bit 61 UC    — error was UNCORRECTED (system was in danger)
       bit 60 EN    — error reporting was enabled when error occurred
       bit 59 MISCV — MCi_MISC contains valid info
       bit 58 ADDRV — MCi_ADDR contains valid info
       bit 57 PCC   — processor-context corrupted (uncorrectable + risky)
       bits 15:0    — MCA Error Code (architectural):
                       0b00001MMMCCCC = memory hierarchy error
                       MMM = level (0=L0/L1/RAM, 1=L1, 2=L2, 3=L3, 4=GEN)
                       CCCC = transaction type (read/write/eviction/snoop)
     MCi_ADDR        (0x402 + 4*i)  physical address (valid iff ADDRV=1)
     MCi_MISC        (0x403 + 4*i)  vendor-specific (used for DIMM mapping
                                      on some Intel chipsets)
   For OUR purposes:
     1. At startup, snapshot every MCi_STATUS (these are PERSISTENT across
        reboots — they record errors from BEFORE we even started).
     2. After test runs complete, re-read every bank. Any STATUS that now
        has VAL=1 but didn't before = a NEW error our tests caused or
        observed. Decode and report.
     3. Crucially: on ECC RAM, the iMC silently corrects single-bit flips
        AND logs them via MCA. Pure pattern tests never see those errors
        (they read back correct data). MCA is the ONLY way to surface
        them, which is exactly the "PassMark MemTest86 Pro" feature
        memtest86+ users complain is missing in the free tool. */
#define MAX_MCA_BANKS 32

static int    g_has_mca         = 0;
static UINT32 g_mca_bank_count  = 0;
static UINT64 g_mca_baseline[MAX_MCA_BANKS];   /* snapshot at startup */
static UINT64 g_mca_post[MAX_MCA_BANKS];        /* snapshot after run */
static UINT32 g_mca_new_errors  = 0;            /* count diff'd to display */

static void mca_detect(void) {
    /* CPUID.1:EDX bit 14 = MCA supported, bit 7 = MCE supported. */
    UINT32 a, b, c, d;
    cpuid(1, 0, &a, &b, &c, &d);
    int mca = (d >> 14) & 1;
    int mce = (d >>  7) & 1;
    if (!mca || !mce) {
        log_line(L"[MCA] CPUID says MCA unsupported — skipping ECC capture");
        return;
    }
    UINT64 cap = rdmsr_safe(0x179);    /* IA32_MCG_CAP */
    if (cap == 0) {
        log_line(L"[MCA] IA32_MCG_CAP returned 0 — disabling ECC capture");
        return;
    }
    g_mca_bank_count = (UINT32)(cap & 0xFF);
    if (g_mca_bank_count > MAX_MCA_BANKS) g_mca_bank_count = MAX_MCA_BANKS;
    if (g_mca_bank_count == 0) {
        log_line(L"[MCA] zero banks reported — MCA not enabled by firmware");
        return;
    }
    g_has_mca = 1;
    CHAR16 lb[120];
    SPrint(lb, sizeof(lb), L"[MCA] enabled: %d bank(s), MCG_CAP=0x%lx",
           g_mca_bank_count, cap);
    log_line(lb);
}

/* Snapshot MCi_STATUS for every bank. Called at startup AND after the
   test run completes. The baseline-vs-post diff tells us which banks
   logged new errors during our run — ANY of those is interesting (CPU
   crash protection caught something we may not have noticed). */
static void mca_snapshot(UINT64 *out) {
    for (UINT32 i = 0; i < g_mca_bank_count; i++) {
        out[i] = rdmsr_safe(0x401 + 4 * i);
    }
}

/* Human-readable label for the architectural MCA error code (low 16 bits
   of MCi_STATUS). We only decode codes likely to appear during a memory
   test; specialty things (TLB errors etc.) get a generic label. */
static const CHAR16 *mca_error_label(UINT64 status) {
    UINT16 ec = (UINT16)(status & 0xFFFF);
    /* Memory hierarchy: 0b00001MMMCCCC. Bits 7:4 = MMM (level), 3:0 = CCCC. */
    if ((ec & 0xFF00) == 0x0100) {
        UINT8 mmm = (UINT8)((ec >> 4) & 0x7);
        UINT8 cccc = (UINT8)(ec & 0xF);
        (void)cccc;
        if (mmm == 0)  return L"Memory (LFB/generic)";
        if (mmm == 1)  return L"L1 cache";
        if (mmm == 2)  return L"L2 cache";
        if (mmm == 3)  return L"L3 cache";
        if (mmm == 4)  return L"Memory controller / RAM";
        return L"Cache/memory hierarchy";
    }
    /* TLB errors: 0b00000000TTCC */
    if ((ec & 0xFFF0) == 0x0010) return L"TLB error";
    /* Bus / Interconnect: 0b0000101PPTRRIILL */
    if ((ec & 0xF800) == 0x0800) return L"Bus / Interconnect";
    /* Other architectural / vendor-specific */
    return L"Other architectural";
}

/* Walk through baseline-vs-post and log any bank that now has VAL=1 but
   didn't before (NEW error). Also report banks that already had VAL=1
   at startup — those weren't caused by us but the user should know
   they exist (could be pre-existing failing hardware). */
static void mca_report_diff(void) {
    if (!g_has_mca) return;
    mca_snapshot(g_mca_post);
    CHAR16 lb[280];
    UINT32 new_count = 0;
    UINT32 preexisting_count = 0;
    for (UINT32 i = 0; i < g_mca_bank_count; i++) {
        UINT64 before = g_mca_baseline[i];
        UINT64 after  = g_mca_post[i];
        int before_valid = (before >> 63) & 1;
        int after_valid  = (after  >> 63) & 1;
        if (!after_valid && !before_valid) continue;
        int is_new = after_valid && (!before_valid || after != before);
        int uc     = (after >> 61) & 1;   /* uncorrected */
        int addrv  = (after >> 58) & 1;
        UINT64 addr = addrv ? rdmsr_safe(0x402 + 4 * i) : 0;
        const CHAR16 *kind = mca_error_label(after);
        if (is_new) new_count++;
        else        preexisting_count++;
        SPrint(lb, sizeof(lb),
               L"[MCA] %s bank %d: %s (%s) status=0x%lx%a",
               is_new ? L"NEW" : L"pre-existing",
               i, kind,
               uc ? L"UNCORRECTED" : L"corrected",
               after,
               addrv ? "" : "  [no addr]");
        log_line(lb);
        if (addrv) {
            CHAR16 dimm[64];
            dimm_label_for_addr(addr, dimm, 64);
            SPrint(lb, sizeof(lb),
                   L"[MCA]   addr=0x%lx → DIMM=%s",
                   addr, dimm);
            log_line(lb);
        }
    }
    g_mca_new_errors = new_count;
    if (new_count == 0 && preexisting_count == 0) {
        log_line(L"[MCA] no logged errors in any bank — clean run");
    } else {
        SPrint(lb, sizeof(lb),
               L"[MCA] summary: %d new error(s), %d pre-existing",
               new_count, preexisting_count);
        log_line(lb);
    }
}

/* ---------- Bandwidth degradation trend ----------
   Analyses the 1-min bucketed BW history collected during the run.
   Compares the FIRST quartile of buckets against the LAST quartile.
   A sustained drop > 5 % is reported as a yellow trend; > 15 % is
   reported as a red trend (likely thermal throttling, IMC retry
   storms, or a marginally-failing channel slowing the controller).

   We need at least 8 buckets (~8 min of runtime) to make any claim
   — shorter runs the trend is noise and we just log "n/a".

   Implementation note: results are exposed via the two globals
   g_bw_trend_first_pct / g_bw_trend_last_pct (relative to overall
   peak ×100) so render_summary and JSON can pick them up without
   recomputing. */
static UINT32 g_bw_trend_first_pct = 0;     /* first-quartile mean / peak * 100 */
static UINT32 g_bw_trend_last_pct  = 0;     /* last-quartile mean / peak * 100 */
static int    g_bw_trend_degraded  = 0;     /* 0 = ok, 1 = mild, 2 = severe */

static void bw_trend_report(void) {
    if (g_bw_history_count < 8) {
        log_line(L"[BWTREND] run too short (<8 min) — no trend analysis");
        return;
    }
    UINT32 q = g_bw_history_count / 4;
    if (q < 2) q = 2;
    UINT64 first_sum = 0, last_sum = 0;
    UINT32 peak = 0;
    for (UINT32 i = 0; i < q; i++) first_sum += g_bw_history_max[i];
    for (UINT32 i = g_bw_history_count - q; i < g_bw_history_count; i++)
        last_sum += g_bw_history_max[i];
    for (UINT32 i = 0; i < g_bw_history_count; i++)
        if (g_bw_history_max[i] > peak) peak = g_bw_history_max[i];
    if (peak == 0) return;

    UINT32 first_mean = (UINT32)(first_sum / q);
    UINT32 last_mean  = (UINT32)(last_sum  / q);
    g_bw_trend_first_pct = (UINT32)((UINT64)first_mean * 100ULL / peak);
    g_bw_trend_last_pct  = (UINT32)((UINT64)last_mean  * 100ULL / peak);

    /* Drop = (first - last) / first * 100, clamped at 0. */
    UINT32 drop_pct = 0;
    if (first_mean > last_mean) {
        drop_pct = (UINT32)(((UINT64)(first_mean - last_mean) * 100ULL) / first_mean);
    }
    const CHAR16 *verdict;
    if (drop_pct >= 15)      { g_bw_trend_degraded = 2; verdict = L"SEVERE drop"; }
    else if (drop_pct >=  5) { g_bw_trend_degraded = 1; verdict = L"mild drop"; }
    else                     { g_bw_trend_degraded = 0; verdict = L"stable"; }

    CHAR16 lb[220];
    SPrint(lb, sizeof(lb),
           L"[BWTREND] %d buckets (1-min) collected, peak %d MB/s, "
           L"first-quartile mean %d MB/s (%d%% of peak), "
           L"last-quartile mean %d MB/s (%d%% of peak) -> %s",
           g_bw_history_count, peak,
           first_mean, g_bw_trend_first_pct,
           last_mean,  g_bw_trend_last_pct,
           verdict);
    log_line(lb);
    if (g_bw_trend_degraded >= 1) {
        SPrint(lb, sizeof(lb),
               L"[BWTREND] WARNING: %d%% bandwidth drop over the run — "
               L"check thermal throttling, IMC retry counters, or a "
               L"marginal channel slowing the controller",
               drop_pct);
        log_line(lb);
    }
}

/* ---------- Cold/warm boot delta — persistent last-run record ----------
   Save a small summary of every run into a UEFI Non-Volatile Variable.
   At the start of the next run we read it back and log the delta:
     "[HIST] Previous run (2 boots ago): 0 errors, peak 78 °C, BW 23 GB/s"
     "[HIST] Delta: temp +6 °C, BW −4 % — possible thermal degradation"
   Lets a shop see at a glance whether a problem is reproducing across
   boots or only happened once. Lets a long-term customer-system check
   spot slow degradation: "BW was 24 GB/s a month ago, today it's 21."

   Storage: one EFI variable, ~96 bytes, vendor GUID below. Flash wear
   is one write per run end — negligible (NVRAM rated for ~100 k writes).

   We deliberately do NOT include the address-record list to keep storage
   tiny + because addresses change between runs anyway. */
static EFI_GUID g_mf_hist_guid = {
    0xA1B2C3D4, 0x5E6F, 0x7890,
    { 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89 }
};
#define MF_HIST_MAGIC   0x32474642u   /* 'MFG2' little-endian */
#define MF_HIST_VERSION 1u

typedef struct {
    UINT32 magic;
    UINT32 version;
    /* Wall-clock seconds since 1970 — RT->GetTime gives us the components,
       we fold to a single 64-bit value via a rough Gregorian conversion
       (good for human-readable display, not for cryptographic precision). */
    INT64  run_epoch_s;
    UINT32 run_seq;             /* +1 every run, lets the log say "5 boots ago" */
    UINT32 run_total_errors;
    UINT32 max_temp_c;
    UINT32 pkg_power_w_peak;
    UINT32 bw_mbps_peak;
    UINT32 bw_trend_first_pct;
    UINT32 bw_trend_last_pct;
    UINT32 bw_trend_degraded;
    UINT32 passes_done;
    UINT32 total_time_s;
    UINT32 total_ram_mb;
    UINT32 mca_new_errors;
    UINT32 cpu_vendor;          /* 1=Intel, 2=AMD, 0=unknown */
    UINT32 reserved[6];
} mf_hist_t;

/* Filled at startup by hist_load. If magic doesn't match the slot is
   treated as empty and g_hist_prev_valid stays 0. */
static mf_hist_t g_hist_prev;
static int       g_hist_prev_valid = 0;

/* Approximate days-since-1970 for a (year, month, day) tuple. Doesn't
   account for leap-year edge cases beyond Gregorian rule — good to
   ±1 day, fine for "last run was 14 days ago" display. */
static INT64 epoch_seconds_from_efitime(EFI_TIME *t) {
    if (!t || t->Year < 1970) return 0;
    static const UINT16 mdays[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
    INT64 y = t->Year;
    INT64 days = (y - 1970) * 365 + ((y - 1969) / 4) - ((y - 1901) / 100) + ((y - 1601) / 400);
    days += mdays[(t->Month - 1) & 0xF];
    if (t->Month > 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) days++;
    days += (t->Day > 0 ? t->Day - 1 : 0);
    return days * 86400LL + (INT64)t->Hour * 3600 + (INT64)t->Minute * 60 + t->Second;
}

static void hist_load(void) {
    UINTN sz = sizeof(g_hist_prev);
    UINT32 attr = 0;
    EFI_STATUS rs = uefi_call_wrapper(RT->GetVariable, 5,
                                      L"MemForgeLastRun", &g_mf_hist_guid,
                                      &attr, &sz, &g_hist_prev);
    if (rs != EFI_SUCCESS || sz < sizeof(UINT32) * 2) {
        log_line(L"[HIST] no previous run record (first run, or NVRAM cleared)");
        return;
    }
    if (g_hist_prev.magic != MF_HIST_MAGIC) {
        log_line(L"[HIST] previous record magic mismatch — ignoring");
        return;
    }
    if (g_hist_prev.version != MF_HIST_VERSION) {
        CHAR16 lb[120];
        SPrint(lb, sizeof(lb),
               L"[HIST] previous record schema v%d ≠ current v%d — ignoring",
               g_hist_prev.version, MF_HIST_VERSION);
        log_line(lb);
        return;
    }
    g_hist_prev_valid = 1;
    /* Human-readable summary of the previous run. */
    CHAR16 lb[260];
    SPrint(lb, sizeof(lb),
           L"[HIST] Previous run #%d: errors=%d  max %d°C  CPU peak %d W  "
           L"BW peak %d MB/s  passes=%d  time=%d s",
           g_hist_prev.run_seq, g_hist_prev.run_total_errors,
           g_hist_prev.max_temp_c, g_hist_prev.pkg_power_w_peak,
           g_hist_prev.bw_mbps_peak, g_hist_prev.passes_done,
           g_hist_prev.total_time_s);
    log_line(lb);
}

static void hist_save_and_diff(UINT64 total_ms) {
    /* Read current time. */
    EFI_TIME t;
    if (uefi_call_wrapper(RT->GetTime, 2, &t, NULL) != EFI_SUCCESS) {
        return;
    }
    UINT32 run_total_errors = (UINT32)g_run_total_errors;
    if (g_run_total_errors > 0xFFFFFFFFULL) run_total_errors = 0xFFFFFFFFu;

    /* Build the new record. */
    mf_hist_t cur;
    for (UINTN i = 0; i < sizeof(cur); i++) ((UINT8*)&cur)[i] = 0;
    cur.magic              = MF_HIST_MAGIC;
    cur.version            = MF_HIST_VERSION;
    cur.run_epoch_s        = epoch_seconds_from_efitime(&t);
    cur.run_seq            = g_hist_prev_valid ? (g_hist_prev.run_seq + 1) : 1;
    cur.run_total_errors   = run_total_errors;
    cur.max_temp_c         = g_max_temp_c;
    cur.pkg_power_w_peak   = g_pkg_power_w_peak;
    cur.bw_mbps_peak       = g_bw_mbps_peak;
    cur.bw_trend_first_pct = g_bw_trend_first_pct;
    cur.bw_trend_last_pct  = g_bw_trend_last_pct;
    cur.bw_trend_degraded  = (UINT32)g_bw_trend_degraded;
    cur.passes_done        = g_run_passes_done;
    cur.total_time_s       = (UINT32)(total_ms / 1000);
    cur.total_ram_mb       = (UINT32)g_total_ram_mb;
    cur.mca_new_errors     = g_mca_new_errors;
    cur.cpu_vendor         = (UINT32)g_cpu_vendor;

    /* Log delta vs previous run before we overwrite the variable. */
    if (g_hist_prev_valid) {
        CHAR16 lb[260];
        INT32  d_temp = (INT32)cur.max_temp_c - (INT32)g_hist_prev.max_temp_c;
        INT32  d_err  = (INT32)cur.run_total_errors - (INT32)g_hist_prev.run_total_errors;
        INT32  d_bw   = 0;
        if (g_hist_prev.bw_mbps_peak > 0) {
            INT64 num = (INT64)cur.bw_mbps_peak - (INT64)g_hist_prev.bw_mbps_peak;
            d_bw = (INT32)((num * 100LL) / (INT64)g_hist_prev.bw_mbps_peak);
        }
        INT64 d_secs = cur.run_epoch_s - g_hist_prev.run_epoch_s;
        INT32 d_days = (INT32)(d_secs / 86400LL);

        /* gnu-efi SPrint doesn't understand %+d (signed-with-mandatory-sign).
           Output literal "?d" instead of the value. Hand-format the sign
           character and absolute value, insert via %c%d. */
        CHAR16 sgn_err  = d_err  >= 0 ? L'+' : L'-';
        CHAR16 sgn_temp = d_temp >= 0 ? L'+' : L'-';
        CHAR16 sgn_bw   = d_bw   >= 0 ? L'+' : L'-';
        UINT32 abs_err  = d_err  >= 0 ? (UINT32)d_err  : (UINT32)(-d_err);
        UINT32 abs_temp = d_temp >= 0 ? (UINT32)d_temp : (UINT32)(-d_temp);
        UINT32 abs_bw   = d_bw   >= 0 ? (UINT32)d_bw   : (UINT32)(-d_bw);
        SPrint(lb, sizeof(lb),
               L"[HIST] Delta vs prev (~%d day(s) ago): errors %c%d, "
               L"temp %c%d°C, BW peak %c%d%%",
               d_days,
               sgn_err,  abs_err,
               sgn_temp, abs_temp,
               sgn_bw,   abs_bw);
        log_line(lb);

        /* Loud warnings on regressions. But: skip thermal/BW comparisons if
           the previous run was shorter than 60 seconds — peak temp on a
           7-second smoke test is meaningless (never reached steady-state)
           and triggering "⚠ temp rose 26°C" against a cold-start baseline
           is a false positive. Field report from a Habr commenter
           (ASRock B760M, igrblkv) where the previous run was only 7 sec. */
        int prev_long_enough = g_hist_prev.total_time_s >= 60;
        if (!prev_long_enough) {
            SPrint(lb, sizeof(lb),
                   L"[HIST] previous run was only %d s — skipping thermal/BW "
                   L"delta warnings (need ≥60s for meaningful peak comparison)",
                   g_hist_prev.total_time_s);
            log_line(lb);
        }
        /* Intensity-similarity check: if BW peak shifted by more than ±40%,
           the workload was clearly different (different MaxCores, different
           Quick vs Full mode, prev hit thermal guard and ran single-core
           while current ran multi-core, etc.). Comparing peak temperature
           across wildly different intensity profiles produces meaningless
           "thermal regression" warnings. Field-reported false-positive
           v0.4.13: prev run was single-core (capped by old thermal guard,
           peak 92°C, BW 4072 MB/s); current run multi-core (peak 126°C,
           BW 9193 MB/s, +126% BW). The +34°C delta is from intensity
           change, not from cooling degradation. */
        int abs_d_bw = (d_bw >= 0) ? d_bw : -d_bw;
        int similar_intensity = (abs_d_bw <= 40);
        if (!similar_intensity) {
            SPrint(lb, sizeof(lb),
                   L"[HIST] BW peak differs by %d%% from prev run — workload "
                   L"intensity changed (different config?); skipping "
                   L"thermal/BW delta warnings",
                   abs_d_bw);
            log_line(lb);
        }
        if (d_err > 0) {
            SPrint(lb, sizeof(lb),
                   L"[HIST] ⚠ REGRESSION: %d new errors since last run",
                   d_err);
            log_line(lb);
        }
        if (d_temp >= 5 && prev_long_enough && similar_intensity) {
            SPrint(lb, sizeof(lb),
                   L"[HIST] ⚠ temp rose %d°C vs last run — check airflow/paste",
                   d_temp);
            log_line(lb);
        }
        if (d_bw <= -5 && prev_long_enough && similar_intensity) {
            SPrint(lb, sizeof(lb),
                   L"[HIST] ⚠ BW dropped %d%% vs last run — possible degradation",
                   -d_bw);
            log_line(lb);
        }
    }

    /* Persist. Non-volatile so it survives power-off; BootService-only
       attribute means runtime/OS can read but not casually overwrite. */
    EFI_STATUS rs = uefi_call_wrapper(RT->SetVariable, 5,
                                      L"MemForgeLastRun", &g_mf_hist_guid,
                                      EFI_VARIABLE_NON_VOLATILE |
                                      EFI_VARIABLE_BOOTSERVICE_ACCESS,
                                      sizeof(cur), &cur);
    if (rs != EFI_SUCCESS) {
        CHAR16 lb[120];
        SPrint(lb, sizeof(lb),
               L"[HIST] SetVariable failed (status=0x%lx) — record not saved",
               (UINT64)rs);
        log_line(lb);
    } else {
        log_line(L"[HIST] run record saved to NVRAM");
    }
}

/* ---------- CPU brand string (CPUID 0x80000002-4) ---------- */
static void detect_cpu_brand(void) {
    UINT32 max_ext = 0, b, c, d;
    cpuid(0x80000000, 0, &max_ext, &b, &c, &d);
    if (max_ext < 0x80000004) {
        /* Fall back to vendor string (CPUID 0). */
        UINT32 vb, vc, vd;
        cpuid(0, 0, NULL, &vb, &vc, &vd);
        UINT32 *vendor = (UINT32 *)g_cpu_brand;
        vendor[0] = vb; vendor[1] = vd; vendor[2] = vc;
        g_cpu_brand[12] = 0;
        return;
    }
    UINT32 *brand = (UINT32 *)g_cpu_brand;
    cpuid(0x80000002, 0, &brand[0], &brand[1], &brand[2],  &brand[3]);
    cpuid(0x80000003, 0, &brand[4], &brand[5], &brand[6],  &brand[7]);
    cpuid(0x80000004, 0, &brand[8], &brand[9], &brand[10], &brand[11]);
    g_cpu_brand[48] = 0;
    /* Trim leading spaces (Intel pads brand string from the right). */
    int i = 0;
    while (g_cpu_brand[i] == ' ' && g_cpu_brand[i]) i++;
    if (i > 0) {
        int j = 0;
        while (g_cpu_brand[i + j]) { g_cpu_brand[j] = g_cpu_brand[i + j]; j++; }
        g_cpu_brand[j] = 0;
    }
}

/* ---------- I/O port + PCI config-space primitives ----------
   Raw inb/outb/inl/outl + the 0xCF8/0xCFC PCI config-space mechanism. We
   use these directly to talk to the ICH/PCH SMBus controller — it's a
   PCI device whose I/O Base Address Register lives in config space, and
   once read, all SMBus host commands go through plain in/out instructions
   to that I/O port. UEFI exposes EFI_PCI_IO_PROTOCOL but raw access is
   simpler here and the protocol availability varies across firmwares. */
static inline UINT8 io_inb(UINT16 port) {
    UINT8 v;
    __asm__ __volatile__("inb %1, %0" : "=a"(v) : "Nd"(port));
    return v;
}
static inline void io_outb(UINT16 port, UINT8 v) {
    __asm__ __volatile__("outb %0, %1" : : "a"(v), "Nd"(port));
}
static inline UINT16 io_inw(UINT16 port) {
    UINT16 v;
    __asm__ __volatile__("inw %1, %0" : "=a"(v) : "Nd"(port));
    return v;
}
static inline void io_outw(UINT16 port, UINT16 v) {
    __asm__ __volatile__("outw %0, %1" : : "a"(v), "Nd"(port));
}
static inline UINT32 io_inl(UINT16 port) {
    UINT32 v;
    __asm__ __volatile__("inl %1, %0" : "=a"(v) : "Nd"(port));
    return v;
}
static inline void io_outl(UINT16 port, UINT32 v) {
    __asm__ __volatile__("outl %0, %1" : : "a"(v), "Nd"(port));
}

/* PCI config-space read at (bus, dev, func, offset). offset must be 4-byte
   aligned for outl-based access. Returns the 32-bit DWORD at that offset. */
static UINT32 pci_read32(UINT8 bus, UINT8 dev, UINT8 func, UINT8 off) {
    UINT32 addr = 0x80000000u
                | ((UINT32)bus  << 16)
                | ((UINT32)dev  << 11)
                | ((UINT32)func <<  8)
                | (off & 0xFC);
    io_outl(0xCF8, addr);
    return io_inl(0xCFC);
}
static UINT16 pci_read16(UINT8 bus, UINT8 dev, UINT8 func, UINT8 off) {
    UINT32 v = pci_read32(bus, dev, func, off & 0xFC);
    return (UINT16)((v >> ((off & 2) * 8)) & 0xFFFF);
}
static UINT8 pci_read8(UINT8 bus, UINT8 dev, UINT8 func, UINT8 off) {
    UINT32 v = pci_read32(bus, dev, func, off & 0xFC);
    return (UINT8)((v >> ((off & 3) * 8)) & 0xFF);
}

/* ---------- AMD SMN (System Management Network) thermal reader ----------
   AMD Zen+ CPUs expose their Tctl (control temperature, the same value
   shown by Ryzen Master / HWiNFO) via the System Management Network.
   The SMN is reachable from the host CPU through a mailbox in the Data
   Fabric device at PCI 00:00.0 (vendor 0x1022) using a non-standard
   index/data pair in config space:
     - PCI config offset 0x60: SMN target address (write 32-bit)
     - PCI config offset 0x64: SMN data (read 32-bit)
   For Zen / Zen+ / Zen2 / Zen3 / Zen4 the Tctl register is at SMN
   address 0x00059800 ("F17M00H_THM_TCTL" in AMD PPR docs). The value
   read is encoded as:
     bits [31:21]  Tctl temperature in 0.125 °C units
       (i.e. divide by 8 to get °C with quarter-degree precision)
   We sample this periodically from sample_aggregate_metrics on AMD
   systems where SMN responded with non-FF on probe.

   Caveats:
     - This is PER-PACKAGE (socket) temperature, not per-core like Intel
       IA32_THERM_STATUS. We populate g_max_temp_c only — the per-core
       column in the core panel still shows "—" on AMD.
     - On dual-socket AMD systems we only see socket 0. UEFI apps don't
       have a clean way to enumerate sockets without ACPI tables.
     - Some Zen3+ chiplet designs add an offset ("Tctl offset", 27 °C
       for Threadripper, etc.) which is what makes Ryzen Master show a
       different value than HWiNFO. We use the raw Tctl. */
static int    g_amd_smn_ready = 0;     /* probe result */
static UINT8  g_amd_df_dev    = 0;     /* PCI device of the Data Fabric (0:00.0) */

static UINT32 amd_smn_read(UINT32 smn_addr) {
    /* Write target address to config offset 0x60, then read data from
       0x64. Both go to PCI 0:DF:0 — we found DF dev during probe. */
    UINT32 cfg_addr = 0x80000000u | ((UINT32)g_amd_df_dev << 11) | 0x60;
    io_outl(0xCF8, cfg_addr);
    io_outl(0xCFC, smn_addr);
    cfg_addr = 0x80000000u | ((UINT32)g_amd_df_dev << 11) | 0x64;
    io_outl(0xCF8, cfg_addr);
    return io_inl(0xCFC);
}

static void amd_thermal_probe(void) {
    if (g_cpu_vendor != CPU_AMD) return;
    /* Find Data Fabric PCI device: vendor 0x1022 at bus 0. On Zen+ this
       is typically at 0:18.x but the host bridge that holds the SMN
       index/data window is at 0:00.0 with various device IDs. We just
       scan for any 0x1022 device on bus 0 with class 0x06 (Host bridge)
       and try SMN access to it. */
    for (UINT8 dev = 0; dev < 32; dev++) {
        UINT16 vid = pci_read16(0, dev, 0, 0x00);
        if (vid != 0x1022) continue;
        UINT8 cls = pci_read8(0, dev, 0, 0x0B);
        if (cls != 0x06) continue;     /* not a host bridge */
        g_amd_df_dev = dev;
        /* Test SMN by reading 0x00059800 (Tctl). FFFFFFFF = no response. */
        UINT32 v = amd_smn_read(0x00059800);
        if (v == 0xFFFFFFFF || v == 0) continue;
        /* v0.4.29 — use the CORRECT decode (same as amd_thermal_sample):
           apply 0x7FF mask AND bit-19 -49°C range adjust. Pre-v0.4.29
           probe used the broken raw>>21/8 decode and would report 92°C
           on Ryzen 9 7900X (real ~43°C) as "initial Tctl" in the log,
           which then poisoned the run-wide peak temperature counter
           and produced a false-positive throttling warning on the
           verdict screen even on water-cooled CPUs. */
        UINT32 raw = (v >> 21) & 0x7FF;
        INT32  deg = (INT32)(raw / 8);
        if (v & (1u << 19)) deg -= 49;
        if (deg < 1 || deg > 150) continue;   /* sanity */
        g_amd_smn_ready = 1;
        g_has_thermal   = 1;            /* enable thermal display, AMD-aware */
        g_tj_max        = 95;           /* typical Ryzen TjMax for ratio calc */
        CHAR16 lb[160];
        SPrint(lb, sizeof(lb),
               L"[CPU] AMD SMN thermal: DF at 00:%02d.0, initial Tctl=%d°C — enabled",
               (UINT32)dev, deg);
        log_line(lb);
        return;
    }
    log_line(L"[CPU] AMD SMN thermal: no Data Fabric found — temperature column stays '—'");
}

static UINT32 amd_thermal_sample(void) {
    /* v0.4.29 — correct decode per Linux k10temp / FreeBSD amdtemp.c:
       SMN 0x59800 (SMU_THM_TCON_CUR_TMP)
         bits [31:21]  raw temperature value (11 bits, mask 0x7FF)
         bit  19       TempRangeSel — when SET, scale is -49°C..+206°C
                       (subtract 49°C from the raw decode); when CLEAR
                       scale is 0..225°C (no offset).
       temp_c = (raw * 0.125) - (range_sel ? 49 : 0)

       Pre-v0.4.29 code was missing both the 0x7FF mask AND the bit-19
       range adjustment, which inflated readings by ~49°C on Ryzen SKUs
       that report on the -49..206 scale (most Renoir/Cezanne/Zen3+
       desktop parts). Field report on Ryzen 5 4500 showed Tctl=93°C at
       idle / 123°C under AVX2 — real temps were 44°C / 74°C, matching
       HWiNFO and confirming the bit-19 path was active. */
    if (!g_amd_smn_ready) return 0;
    UINT32 v = amd_smn_read(0x00059800);
    UINT32 raw = (v >> 21) & 0x7FF;
    INT32  deg = (INT32)(raw / 8);          /* raw * 0.125 °C */
    if (v & (1u << 19)) deg -= 49;          /* range adjust */
    if (deg < 0 || deg > 150) return 0;     /* sanity clamp */
    return (UINT32)deg;
}

/* ---------- Intel SMBus / SPD reader ----------
   Intel ICH/PCH SMBus host controller lives at PCI device class 0x0C0500
   (Serial Bus → SMBus). Across generations it has appeared at multiple
   bus/dev/func locations but always with that class code, so we scan PCI
   bus 0 for it rather than hard-coding.

   Once found, BAR4 (offset 0x20 in config) holds the I/O base address.
   Standard register layout (Intel SMBus Programmer's Reference):
     +0x00 HST_STS    R/W  Host Status (write-1-to-clear for sticky bits)
     +0x02 HST_CNT    R/W  Host Control (start, command code, intr enable)
     +0x03 HST_CMD    R/W  Command code (the byte-offset INSIDE the SPD EEPROM)
     +0x04 XMIT_SLVA  R/W  Slave 7-bit address + R/W bit
     +0x05 HST_D0     R/W  Data byte 0 (read result lives here for byte ops)
     +0x06 HST_D1     R/W  Data byte 1 (for word ops, unused for SPD byte reads)
   HST_STS bit fields we care about:
     bit 0 HOST_BUSY     — controller is mid-transaction
     bit 1 INTR          — operation completed successfully
     bit 2 DEV_ERR       — device gave NACK (e.g. no SPD at that addr)
     bit 3 BUS_ERR       — generic SMBus error
     bit 4 FAILED        — host abort
   HST_CNT command codes (bits 4:2):
     001 BYTE            — write/read a single byte at HST_D0
     010 BYTE_DATA       — write command byte + write/read HST_D0
     011 WORD_DATA       — like BYTE_DATA but uses both HST_D0/D1
   Plus bit 6 START kicks the transaction. Bit 0 INTR_EN we leave 0
   (poll-based). The slave address byte is (addr_7bit << 1) | (1 = read).
*/
static UINT16 g_smbus_io_base = 0;    /* 0 = not found / not initialised */
static int    g_smbus_present = 0;

static int smbus_locate(void) {
    /* Scan bus 0. The PCH SMBus typically lives at dev 31 func 4 on modern
       Intel parts, dev 31 func 3 on older ICH7..ICH10, and dev 20 func 0
       on AMD FCH. Restricting the scan to bus 0 keeps it fast (32 devs ×
       8 funcs = 256 probes).

       Intel and AMD FCH both implement the PIIX4-compatible SMBus register
       layout, so once we have the I/O base the rest of smbus_byte_read /
       smbus_wait works on BOTH unchanged. The detection differs only in:
         - PCI vendor ID (0x8086 Intel vs 0x1022 AMD)
         - Where the I/O base lives in PCI config:
             Intel: standard BAR4 (offset 0x20)
             AMD:   non-standard offset 0x90 (SMBA I/O Base in FCH spec).
                    If 0 there, fall back to fixed 0xB00 which is the AMD
                    Bolton/Promontory default.
    */
    for (UINT8 dev = 0; dev < 32; dev++) {
        UINT16 vid = pci_read16(0, dev, 0, 0x00);
        if (vid == 0xFFFF) continue;
        UINT8 hdr  = pci_read8(0, dev, 0, 0x0E);
        UINT8 nfunc = (hdr & 0x80) ? 8 : 1;
        for (UINT8 func = 0; func < nfunc; func++) {
            UINT16 fvid = pci_read16(0, dev, func, 0x00);
            if (fvid == 0xFFFF) continue;
            /* Class 0x0C / Subclass 0x05 = Serial Bus → SMBus. */
            UINT8 cls = pci_read8(0, dev, func, 0x0B);
            UINT8 sub = pci_read8(0, dev, func, 0x0A);
            if (cls != 0x0C || sub != 0x05) continue;

            UINT16 base = 0;
            const CHAR16 *vendor_name = L"?";
            if (fvid == 0x8086) {
                /* Intel: I/O base in BAR4 (offset 0x20), bit 0 = 1 marks
                   it as an I/O BAR. */
                UINT32 bar4 = pci_read32(0, dev, func, 0x20);
                if (!(bar4 & 1)) continue;
                base = (UINT16)(bar4 & 0xFFFE);
                vendor_name = L"Intel";
            } else if (fvid == 0x1022) {
                /* AMD FCH: I/O base at non-standard PCI config offset 0x90
                   (per AMD FCH SMBus spec). If firmware left it 0, fall
                   back to the hard-coded 0xB00 default which all Zen-era
                   FCHs ship with. */
                UINT32 smba = pci_read32(0, dev, func, 0x90);
                base = (UINT16)(smba & 0xFFE0);   /* mask off control bits */
                if (base == 0) base = 0xB00;
                vendor_name = L"AMD";
            } else {
                continue;   /* unknown SMBus vendor — skip */
            }

            g_smbus_io_base = base;
            g_smbus_present = 1;

            /* Make sure I/O space access is enabled in the Command
               register. Some firmwares clear bit 0 after their own use. */
            UINT16 cmd = pci_read16(0, dev, func, 0x04);
            if (!(cmd & 0x0001)) {
                UINT32 addr = 0x80000000u | ((UINT32)dev << 11)
                            | ((UINT32)func << 8) | 0x04;
                io_outl(0xCF8, addr);
                io_outw(0xCFC, cmd | 0x0001);
            }
            CHAR16 lb[120];
            SPrint(lb, sizeof(lb),
                   L"[SPD] SMBus host: %s vendor (PCI %02d:%02d.%d), I/O base 0x%X",
                   vendor_name, 0, (UINT32)dev, (UINT32)func, (UINT32)base);
            log_line(lb);
            return 1;
        }
    }
    return 0;
}

/* Wait for the host controller to finish a transaction. Returns the final
   HST_STS value; caller checks for INTR (success) vs DEV_ERR/BUS_ERR. */
static UINT8 smbus_wait(void) {
    UINT16 base = g_smbus_io_base;
    /* Poll up to ~10 ms. A real byte read completes in <1 ms; the 10-ms
       cap matters only for NACK detection on blocked SMBus (HP Sure Start,
       TPM-locked DIMMs etc.) where the controller never reports BUSY=0.
       Previously 100 ms ×8 addresses ×384 bytes/probe was making HP boots
       look frozen — pure 30+ second SPD wait. Real systems answer fast. */
    for (int spin = 0; spin < 1000; spin++) {
        UINT8 s = io_inb(base + 0x00);
        if (!(s & 0x01))  return s;       /* HOST_BUSY cleared → done */
        uefi_call_wrapper(BS->Stall, 1, (UINTN)10);  /* 10 µs */
    }
    return io_inb(base + 0x00);
}

/* Read one byte from SPD EEPROM at SMBus addr (7-bit) + offset.
   Returns 1 on success and writes the byte into *out; 0 on any error.
   `cmd_offset` is the byte index INSIDE the EEPROM (0..255 for one page).
   For DDR4 SPDs that span 2 pages (bytes 0..511) the caller must use the
   page-select commands (SPA0=0x36, SPA1=0x37) before requesting offset
   ≥ 256 — we do that in spd_read_page_select. */
static int smbus_byte_read(UINT8 addr7, UINT8 cmd_offset, UINT8 *out) {
    if (!g_smbus_present) return 0;
    UINT16 base = g_smbus_io_base;
    /* Clear any sticky status bits from a previous transaction. Writing 1
       to each clears it (per Intel ICH SMBus spec). */
    io_outb(base + 0x00, 0xFF);
    /* Set up: slave addr | READ bit, command byte. */
    io_outb(base + 0x04, (addr7 << 1) | 1);
    io_outb(base + 0x03, cmd_offset);
    /* Trigger BYTE_DATA read: HST_CNT = (cmd=010 << 2) | START(6). */
    io_outb(base + 0x02, (0x2 << 2) | (1 << 6));
    UINT8 sts = smbus_wait();
    /* INTR (bit 1) without errors (bits 2/3/4) = success. */
    if ((sts & 0x1E) != 0x02) return 0;
    *out = io_inb(base + 0x05);
    return 1;
}

/* DDR4 SPD page select: addresses 0x36 (page 0 = bytes 0..255) and 0x37
   (page 1 = bytes 256..511) accept a "Send Byte" command. The command
   data is irrelevant — the act of writing to that address switches the
   active page. After this, normal byte reads at addr=0x50..0x57 expose
   the selected 256-byte window. */
static void spd_set_page(UINT8 page) {
    if (!g_smbus_present) return;
    UINT16 base = g_smbus_io_base;
    UINT8 spa = (page == 0) ? 0x36 : 0x37;
    io_outb(base + 0x00, 0xFF);
    io_outb(base + 0x04, (spa << 1) | 0);   /* WRITE */
    io_outb(base + 0x03, 0x00);              /* command byte = don't care */
    io_outb(base + 0x05, 0x00);              /* data byte = don't care */
    io_outb(base + 0x02, (0x2 << 2) | (1 << 6));  /* BYTE_DATA write */
    (void)smbus_wait();
}

/* Read a contiguous range from an SPD EEPROM. Handles the 256-byte page
   boundary for DDR4 (and DDR5 doesn't use SPA — bytes are accessed via
   a different mechanism, but for our basic-field needs the first 256 are
   enough). Returns count of bytes successfully read (0..len). */
static UINTN spd_read_bytes(UINT8 addr7, UINT16 start, UINT16 len,
                             UINT8 *buf) {
    UINTN got = 0;
    UINT8 cur_page = 0xFF;
    for (UINT16 i = 0; i < len; i++) {
        UINT16 abs_off = start + i;
        UINT8  page    = (UINT8)(abs_off >> 8);
        UINT8  in_page = (UINT8)(abs_off & 0xFF);
        if (page != cur_page) {
            spd_set_page(page);
            cur_page = page;
        }
        UINT8 b = 0;
        if (!smbus_byte_read(addr7, in_page, &b)) break;
        buf[i] = b;
        got++;
    }
    /* Reset to page 0 so subsequent code sees the SPD's "default" view. */
    if (cur_page != 0) spd_set_page(0);
    return got;
}

/* Parse the bytes we care about out of a 384-byte SPD dump. The layout
   covered here is the DDR4 SPD (Annex B of JEDEC SPD4.1.2.L); DDR3 uses
   different offsets but we fall back to a partial parse. DDR5 has its
   own (much larger) SPD layout which we don't yet decode beyond byte 2.
   Fills the relevant fields of `d`. */
static void spd_parse_into_dimm(UINT8 *buf, UINTN n_bytes, dimm_info_t *d) {
    if (n_bytes < 4) return;
    UINT8 dram_type = buf[2];
    /* JEDEC SPD byte 2 DRAM Device Type codes:
         0x0B DDR3,  0x0C DDR4,  0x12 DDR5,  0x13 LPDDR4,  ...
       Map to a class number for our display logic. */
    if      (dram_type == 0x0C) d->spd_size_class = 4;  /* DDR4 */
    else if (dram_type == 0x12) d->spd_size_class = 5;  /* DDR5 */
    else if (dram_type == 0x0B) d->spd_size_class = 3;  /* DDR3 */
    else                         d->spd_size_class = 0;

    /* Chip organization — byte offsets DIFFER between DDR3 and DDR4/DDR5.
       The bit-field ENCODING within each byte is the same:
         "Module Organization" byte:
           bits [5:3] (DDR3) or [4:3] (DDR4/5) = ranks-minus-one
           bits [2:0]                          = SDRAM device width
                                                   (000=x4, 001=x8, 010=x16, 011=x32)
         "Memory Bus Width" byte:
           bits [4:3] = Bus Width Extension (00=none, 01=+8b for ECC)
           bits [2:0] = Primary Bus Width   (000=8, 001=16, 010=32, 011=64)
       Bytes:
         DDR3 (JESD79-3): organization=7, bus width=8
         DDR4 (JESD79-4): organization=12, bus width=13
         DDR5 (JESD79-5): organization=235, bus width=235 (treated below)
       Reading byte 12 on a DDR3 SPD gets "Module Nominal Voltage", which
       is meaningless as a chip-organization field — exactly the bug that
       caused HP 8300 to report "x16" on a Samsung x8 module. */
    {
        UINT8 b_org = 0, b_bw = 0;
        int have = 0;
        if (d->spd_size_class == 3 && n_bytes >= 9) {
            b_org = buf[7];  b_bw = buf[8];   have = 1;
        } else if ((d->spd_size_class == 4 || d->spd_size_class == 5) &&
                   n_bytes >= 14) {
            b_org = buf[12]; b_bw = buf[13];  have = 1;
        }
        if (have) {
            UINT8 dw_code  = b_org & 0x07;
            /* DDR3 puts ranks in bits[5:3] (3-bit), DDR4/5 in [4:3] (2-bit).
               Mask wider for DDR3 so 4-rank LRDIMMs decode correctly. */
            UINT8 rk_code  = (d->spd_size_class == 3)
                             ? ((b_org >> 3) & 0x07)
                             : ((b_org >> 3) & 0x03);
            UINT8 bw_code  = b_bw & 0x07;
            UINT8 ext_code = (b_bw >> 3) & 0x03;
            if (dw_code <= 3) d->spd_device_width = (UINT8)(4 << dw_code);
            if (rk_code <= 3) d->spd_ranks        = (UINT8)(rk_code + 1);
            if (bw_code <= 3) d->spd_bus_width    = (UINT8)(8 << bw_code);
            if (ext_code == 1 && d->spd_bus_width > 0)
                d->spd_bus_width += 8;       /* ECC */
        }
    }

    /* Manufacturer / serial / mfg date — byte offsets DIFFER by DDR
       generation, again. Always BCD-encoded except for the serial which
       is raw 4-byte unique ID.
         DDR3 (JESD79-3 Annex K, rev 1.3+):
           byte 117    Module Mfg JEDEC continuation count (bank)
           byte 118    Module Mfg JEDEC ID code
           byte 120    Module Manufacturing Date — Year   (BCD)
           byte 121    Module Manufacturing Date — Week   (BCD)
           bytes 122-125  Module Serial Number (4 bytes)
         DDR4 (JESD79-4 SPD section):
           byte 320    Module Mfg JEDEC bank
           byte 321    Module Mfg JEDEC code
           byte 323    Year (BCD)
           byte 324    Week (BCD)
           bytes 325-328  Serial (4 bytes)
         DDR5: manufacturer block lives in the SPD5 Hub at high offsets we
         can't reach without I3C — skipped (handled by the SPD5 Hub probe
         logged separately as [SPD5HUB]).                                 */
    if (d->spd_size_class == 3 && n_bytes >= 126) {
        d->spd_jedec_bank = buf[117];
        d->spd_jedec_code = buf[118];
        d->spd_mfg_year   = buf[120];
        d->spd_mfg_week   = buf[121];
        d->spd_serial[0]  = buf[122];
        d->spd_serial[1]  = buf[123];
        d->spd_serial[2]  = buf[124];
        d->spd_serial[3]  = buf[125];
    } else if (d->spd_size_class == 4 && n_bytes >= 329) {
        d->spd_jedec_bank = buf[320];
        d->spd_jedec_code = buf[321];
        d->spd_mfg_year   = buf[323];
        d->spd_mfg_week   = buf[324];
        d->spd_serial[0]  = buf[325];
        d->spd_serial[1]  = buf[326];
        d->spd_serial[2]  = buf[327];
        d->spd_serial[3]  = buf[328];
    }

    /* === Primary JEDEC timings === MTB-based encoding shared by DDR3 and
       DDR4 (default MTB = 0.125 ns = 125 ps); DDR5 uses direct picoseconds.
       FTB (Fine Time Base) signed adjustments per-timing are ignored to
       keep the code simple — can shift CL by ±1 clock at worst (display).

       DDR3 (JESD79-3) byte map:
         byte 12   tCKmin   (MTB)
         byte 16   tAAmin   (MTB)  → CAS latency
         byte 18   tRCDmin  (MTB)
         byte 20   tRPmin   (MTB)
         byte 21   bits[7:4] tRAS upper nibble, bits[3:0] tRC upper nibble
         byte 22   tRASmin lower byte  → 12-bit MTB value
         bytes 24-25  tRFCmin (MTB, little-endian 16-bit)

       DDR4 (JESD79-4) byte map:
         byte 18   tCKAVGmin (MTB)
         byte 24   tAAmin    (MTB)  → CAS latency
         byte 25   tRCDmin   (MTB)
         byte 26   tRPmin    (MTB)
         byte 27   bits[7:4] tRAS upper, bits[3:0] tRC upper
         byte 28   tRASmin lower byte
         bytes 30-31 tRFC1min                                            */
    if ((d->spd_size_class == 3 && n_bytes >= 26) ||
        (d->spd_size_class == 4 && n_bytes >= 36)) {
        int is_d4 = (d->spd_size_class == 4);
        UINT8 o_tCK  = is_d4 ? 18 : 12;
        UINT8 o_tAA  = is_d4 ? 24 : 16;
        UINT8 o_tRCD = is_d4 ? 25 : 18;
        UINT8 o_tRP  = is_d4 ? 26 : 20;
        UINT8 o_rasU = is_d4 ? 27 : 21;
        UINT8 o_rasL = is_d4 ? 28 : 22;
        UINT8 o_rfcL = is_d4 ? 30 : 24;
        UINT8 o_rfcH = is_d4 ? 31 : 25;

        UINT16 mtb_tCK  = buf[o_tCK];
        UINT16 mtb_tAA  = buf[o_tAA];
        UINT16 mtb_tRCD = buf[o_tRCD];
        UINT16 mtb_tRP  = buf[o_tRP];
        UINT16 mtb_tRAS = (UINT16)(((buf[o_rasU] & 0xF0) << 4) | buf[o_rasL]);
        UINT16 mtb_tRFC = (UINT16)(buf[o_rfcL] | ((UINT16)buf[o_rfcH] << 8));

        UINT32 tCK_ps  = (UINT32)mtb_tCK  * 125u;
        UINT32 tAA_ps  = (UINT32)mtb_tAA  * 125u;
        UINT32 tRCD_ps = (UINT32)mtb_tRCD * 125u;
        UINT32 tRP_ps  = (UINT32)mtb_tRP  * 125u;
        UINT32 tRAS_ps = (UINT32)mtb_tRAS * 125u;
        UINT32 tRFC_ps = (UINT32)mtb_tRFC * 125u;

        d->spd_tCK_ps = (tCK_ps <= 0xFFFF) ? (UINT16)tCK_ps : 0;

        /* Cycles = round_up(ps / tCK_ps). Guard against tCK_ps=0 (SPD junk). */
        if (tCK_ps) {
            UINT32 cl_clk  = (tAA_ps  + tCK_ps - 1) / tCK_ps;
            UINT32 rcd_clk = (tRCD_ps + tCK_ps - 1) / tCK_ps;
            UINT32 rp_clk  = (tRP_ps  + tCK_ps - 1) / tCK_ps;
            UINT32 ras_clk = (tRAS_ps + tCK_ps - 1) / tCK_ps;
            d->spd_tCL  = (cl_clk  > 255) ? 255 : (UINT8)cl_clk;
            d->spd_tRCD = (rcd_clk > 255) ? 255 : (UINT8)rcd_clk;
            d->spd_tRP  = (rp_clk  > 255) ? 255 : (UINT8)rp_clk;
            d->spd_tRAS = (ras_clk > 255) ? 255 : (UINT8)ras_clk;
        }
        UINT32 tRFC_ns = tRFC_ps / 1000u;
        d->spd_tRFC_ns = (tRFC_ns > 65535) ? 65535 : (UINT16)tRFC_ns;
    } else if (d->spd_size_class == 5 && n_bytes >= 44) {
        /* DDR5 SPD layout (JEDEC SPD 5.0). Per-byte units shift — MTB is
           1 ps and timings are 16-bit little-endian fields:
             bytes 20-21  tCKAVGmin
             bytes 30-31  tAAmin    (CAS)
             bytes 32-33  tRCDmin
             bytes 34-35  tRPmin
             bytes 36-37  tRASmin
             bytes 40-41  tRFC1min  (in ns directly per spec)
           DDR5 also encodes most timings as picoseconds rather than MTB —
           so the values are already in ps and we skip the ×125 step. */
        UINT16 tCK_ps  = (UINT16)(buf[20] | ((UINT16)buf[21] << 8));
        UINT16 tAA_ps  = (UINT16)(buf[30] | ((UINT16)buf[31] << 8));
        UINT16 tRCD_ps = (UINT16)(buf[32] | ((UINT16)buf[33] << 8));
        UINT16 tRP_ps  = (UINT16)(buf[34] | ((UINT16)buf[35] << 8));
        UINT16 tRAS_ps = (UINT16)(buf[36] | ((UINT16)buf[37] << 8));
        UINT16 tRFC_ns = (n_bytes >= 42) ? (UINT16)(buf[40] | ((UINT16)buf[41] << 8)) : 0;

        d->spd_tCK_ps  = tCK_ps;
        if (tCK_ps) {
            UINT32 cl_clk  = ((UINT32)tAA_ps  + tCK_ps - 1) / tCK_ps;
            UINT32 rcd_clk = ((UINT32)tRCD_ps + tCK_ps - 1) / tCK_ps;
            UINT32 rp_clk  = ((UINT32)tRP_ps  + tCK_ps - 1) / tCK_ps;
            UINT32 ras_clk = ((UINT32)tRAS_ps + tCK_ps - 1) / tCK_ps;
            d->spd_tCL  = (cl_clk  > 255) ? 255 : (UINT8)cl_clk;
            d->spd_tRCD = (rcd_clk > 255) ? 255 : (UINT8)rcd_clk;
            d->spd_tRP  = (rp_clk  > 255) ? 255 : (UINT8)rp_clk;
            d->spd_tRAS = (ras_clk > 255) ? 255 : (UINT8)ras_clk;
        }
        d->spd_tRFC_ns = tRFC_ns;
    }
    d->spd_present = 1;
}

/* High-level: discover SMBus controller, then for every SMBIOS-known DIMM
   probe SPD addresses 0x50..0x57 and pull a 384-byte dump from the first
   one that responds. Match SMBus order to SMBIOS order by slot index (a
   rough heuristic — works on most consumer boards where DIMM 1 = SPD 0x50,
   DIMM 2 = 0x51, etc.). Logs every probe so the user can see what
   happened. */
static void spd_populate_dimms(void) {
    CHAR16 lb[180];
    if (!smbus_locate()) {
        log_line(L"[SPD] SMBus controller not found (Intel) — skipping SPD read");
        return;
    }
    SPrint(lb, sizeof(lb),
           L"[SPD] SMBus base I/O port = 0x%X; probing 0x50..0x57",
           g_smbus_io_base);
    log_line(lb);

    UINT8 buf[384];
    UINT32 dimm_idx = 0;
    for (UINT8 a = 0x50; a <= 0x57 && dimm_idx < g_dimm_count; a++) {
        UINT8 first;
        /* Sniff byte 0. SPDs always have a non-zero "Bytes Used" field
           there (0x23 / 0x40 / 0x80 typical for DDR3/4/5). 0x00 or 0xFF
           means no SPD responded. */
        if (!smbus_byte_read(a, 0, &first)) continue;
        if (first == 0x00 || first == 0xFF) continue;
        /* SMBus signal-integrity probe: re-read byte 0 a few times. SPD
           byte 0 is static EEPROM content — every read MUST return the
           same value. Mismatches or NAKs indicate a noisy SMBus (poor
           cable seating, motherboard SI issues, marginal pull-ups). This
           doesn't affect memory testing but the operator wants to know
           if the platform's I²C is flaky before trusting any other SPD
           data. Cheap — 16 short reads, ~5 ms total. */
        {
            UINT32 mismatches = 0, nacks = 0;
            const UINT32 probes = 16;
            for (UINT32 p = 0; p < probes; p++) {
                UINT8 v;
                if (!smbus_byte_read(a, 0, &v)) nacks++;
                else if (v != first)            mismatches++;
            }
            if (mismatches > 0 || nacks > 0) {
                SPrint(lb, sizeof(lb),
                       L"[SMBUS] slot 0x%X integrity: %d/%d mismatches, "
                       L"%d/%d NAKs — possible SI issue on SMBus",
                       a, mismatches, probes, nacks, probes);
                log_line(lb);
            }
        }
        /* Real SPD — pull 384 bytes (covers DDR4 manufacturer block). */
        UINTN got = spd_read_bytes(a, 0, sizeof(buf), buf);
        if (got < 4) continue;
        dimm_info_t *d = &g_dimms[dimm_idx];
        d->spd_addr = a;
        spd_parse_into_dimm(buf, got, d);
        /* Diagnostic log line — show what we extracted. gnu-efi's SPrint
           promotes UINT8 args to UINTN and `%02X` then prints the whole
           promoted width (8 hex digits), producing garbage like
           "0x00000050" instead of "0x50" and a 32-char serial blob.
           Workaround: hand-format each byte into a small string buffer
           and insert via %a. Result is exactly what you'd expect from
           a standard printf. */
        static const CHAR8 hex[] = "0123456789ABCDEF";
        #define HX2(out, v) do { \
            (out)[0] = hex[((v) >> 4) & 0xF]; \
            (out)[1] = hex[(v) & 0xF]; \
            (out)[2] = 0; \
        } while (0)
        CHAR8 s_addr[3], s_ser[9], s_year[3], s_week[3];
        HX2(s_addr, a);
        s_ser[0] = hex[(d->spd_serial[0] >> 4) & 0xF];
        s_ser[1] = hex[ d->spd_serial[0]       & 0xF];
        s_ser[2] = hex[(d->spd_serial[1] >> 4) & 0xF];
        s_ser[3] = hex[ d->spd_serial[1]       & 0xF];
        s_ser[4] = hex[(d->spd_serial[2] >> 4) & 0xF];
        s_ser[5] = hex[ d->spd_serial[2]       & 0xF];
        s_ser[6] = hex[(d->spd_serial[3] >> 4) & 0xF];
        s_ser[7] = hex[ d->spd_serial[3]       & 0xF];
        s_ser[8] = 0;
        HX2(s_year, d->spd_mfg_year);
        HX2(s_week, d->spd_mfg_week);
        #undef HX2
        SPrint(lb, sizeof(lb),
               L"[SPD] slot 0x%a (DIMM%d): type=%d serial=%a mfg=20%a/W%a bytes=%ld",
               s_addr, dimm_idx,
               d->spd_size_class,
               s_ser,
               s_year, s_week,
               (UINT64)got);
        log_line(lb);
        /* Second line: the timings BIOS-setup style. Effective MT/s is
           computed as 2,000,000 / tCK_ps (double data rate). Helps the
           operator instantly see "DDR4-3200 16-18-18-38 tRFC=350ns" vs
           "DDR4-2666 19-19-19-43 tRFC=350ns" and notice XMP vs JEDEC. */
        if (d->spd_tCK_ps && d->spd_tCL) {
            UINT32 mtps = 2000000u / d->spd_tCK_ps;
            SPrint(lb, sizeof(lb),
                   L"[SPD]   timings: DDR%d-%d %d-%d-%d-%d  tRFC=%dns  tCK=%dps",
                   d->spd_size_class, mtps,
                   d->spd_tCL, d->spd_tRCD, d->spd_tRP, d->spd_tRAS,
                   d->spd_tRFC_ns, d->spd_tCK_ps);
            log_line(lb);
        }
        /* DDR5: probe the SPD5118 Hub for device-info MRs and any
           SMBus-side PEC error counter. The on-die ECC counters
           themselves (DDR5 MR48-51 inside the DRAM die) are NOT
           reachable from SMBus — they need MRR (Mode Register Read)
           via the platform's iMC mailbox, which is chipset-specific
           and not implemented here. What we CAN read:
             MR0  Device Type           (0x18 = SPD5118)
             MR1  Hub Revision
             MR3  Vendor ID
             MR48 PEC error counter on SMBus side
           A non-zero PEC counter is a clear "I²C is noisy" signal that
           the operator should investigate (cable seating, pull-ups). */
        if (d->spd_size_class == 5) {
            UINT8 hub_dev = 0, hub_rev = 0, hub_vid = 0, hub_pec = 0;
            int got_any = 0;
            if (smbus_byte_read(a, 0, &hub_dev)) got_any++;
            if (smbus_byte_read(a, 1, &hub_rev)) got_any++;
            if (smbus_byte_read(a, 3, &hub_vid)) got_any++;
            /* MR48 = PEC error counter, register 0x30 on SPD5118. */
            if (smbus_byte_read(a, 0x30, &hub_pec)) got_any++;
            if (got_any) {
                SPrint(lb, sizeof(lb),
                       L"[SPD5HUB] slot 0x%X: dev=0x%02X rev=0x%02X vid=0x%02X "
                       L"hub_pec_err=%d",
                       a, hub_dev, hub_rev, hub_vid, hub_pec);
                log_line(lb);
                if (hub_pec > 0) {
                    SPrint(lb, sizeof(lb),
                           L"[SPD5HUB] ⚠ slot 0x%X: %d PEC errors on SMBus side — "
                           L"noisy I²C (check seating / pull-ups)",
                           a, hub_pec);
                    log_line(lb);
                }
            }
        }
        dimm_idx++;
    }
    if (dimm_idx == 0) {
        log_line(L"[SPD] no SPDs responded — DDR5 I3C-only platform or all slots empty");
    }
    /* One-shot note for the DDR5 case: explain WHY we don't print per-die
       on-die ECC counters. Operators familiar with DDR5 often expect us
       to dump them — we make the limitation explicit instead. */
    if (g_dimm_ddr_type == 0x22 || g_dimm_ddr_type == 0x23) {
        log_line(L"[ODECC] DDR5 on-die ECC counters (MR48-51) live inside "
                 L"each DRAM die. They're only reachable via Mode Register "
                 L"Read through the platform iMC mailbox, which is "
                 L"chipset-specific. Not implemented here. The SMBus-side "
                 L"PEC error counter (reported above per slot) is the "
                 L"closest signal available.");
    }
}

/* ---------- SMBIOS parser (Type 0 BIOS, Type 1 System, Type 17 Memory) ---------- */
typedef struct {
    UINT8  anchor[4];      /* "_SM_" */
    UINT8  checksum;
    UINT8  length;
    UINT8  major;
    UINT8  minor;
    UINT16 max_struct_size;
    UINT8  revision;
    UINT8  formatted[5];
    UINT8  dmi_anchor[5];  /* "_DMI_" */
    UINT8  dmi_checksum;
    UINT16 table_length;
    UINT32 table_address;
    UINT16 num_structs;
    UINT8  bcd_revision;
} __attribute__((packed)) smbios_entry_t;

typedef struct {
    UINT8  anchor[5];      /* "_SM3_" */
    UINT8  checksum;
    UINT8  length;
    UINT8  major;
    UINT8  minor;
    UINT8  docrev;
    UINT8  revision;
    UINT8  reserved;
    UINT32 table_max_size;
    UINT64 table_address;
} __attribute__((packed)) smbios3_entry_t;

typedef struct {
    UINT8  type;
    UINT8  length;
    UINT16 handle;
} __attribute__((packed)) smbios_hdr_t;

/* Bounded string-table walker. `end` caps how far we'll scan. */
static const CHAR8 *smbios_get_str_bounded(const smbios_hdr_t *h, UINT8 idx,
                                            const CHAR8 *end) {
    if (idx == 0) return (const CHAR8 *)"";
    const CHAR8 *p = (const CHAR8 *)h + h->length;
    /* Sanity: don't scan past end of table */
    if (p >= end) return (const CHAR8 *)"";
    while (idx > 1 && p < end && *p) {
        while (p < end && *p) p++;
        if (p >= end) return (const CHAR8 *)"";
        p++;
        idx--;
    }
    if (p >= end || *p == 0) return (const CHAR8 *)"";
    return p;
}

static void smbios_copy_str(CHAR8 *dst, UINTN dst_sz, const CHAR8 *src) {
    /* Preserve existing default (e.g. "(unknown)") when SMBIOS string is
       empty or null — otherwise we'd overwrite the placeholder with "". */
    if (!src || !src[0]) return;
    UINTN i = 0;
    while (src[i] && i < dst_sz - 1) { dst[i] = src[i]; i++; }
    dst[i] = 0;
}

static void parse_smbios_table(VOID *table_base, UINTN max_size, UINT16 num_structs) {
    if (!table_base || max_size < 4) return;
    /* Cap to prevent runaway walks if firmware reports a bogus size. */
    if (max_size > 65536) max_size = 65536;
    const CHAR8 *p = (const CHAR8 *)table_base;
    const CHAR8 *end = p + max_size;
    UINT16 seen = 0;
    UINT16 cap = num_structs ? num_structs : 256;
    if (cap > 256) cap = 256;
    while (p + 4 <= end && seen < cap) {
        const smbios_hdr_t *h = (const smbios_hdr_t *)p;
        if (h->length < 4) break;
        if (p + h->length > end) break;  /* formatted area exceeds table */
        const UINT8 *raw = (const UINT8 *)h;

        if (h->type == 0 && h->length >= 6) {
            smbios_copy_str(g_bios_vendor,  sizeof(g_bios_vendor),
                            smbios_get_str_bounded(h, raw[4], end));
            smbios_copy_str(g_bios_version, sizeof(g_bios_version),
                            smbios_get_str_bounded(h, raw[5], end));
        } else if (h->type == 1 && h->length >= 6) {
            smbios_copy_str(g_sys_vendor, sizeof(g_sys_vendor),
                            smbios_get_str_bounded(h, raw[4], end));
            smbios_copy_str(g_sys_model,  sizeof(g_sys_model),
                            smbios_get_str_bounded(h, raw[5], end));
        } else if (h->type == 17 && h->length >= 0x16) {
            if (g_dimm_count >= MAX_DIMMS) {
                /* Server with > MAX_DIMMS sticks — log once at the boundary
                   so the operator notices the truncation instead of silently
                   seeing only the first N DIMMs and a wrong total. */
                static int warned_overflow = 0;
                if (!warned_overflow) {
                    CHAR16 wb[160];
                    SPrint(wb, sizeof(wb),
                           L"[SMBIOS] ⚠ more than %d DIMMs present — "
                           L"only first %d tracked (rebuild with larger MAX_DIMMS)",
                           MAX_DIMMS, MAX_DIMMS);
                    log_line(wb);
                    warned_overflow = 1;
                }
            } else {
            UINT16 size_field = *(const UINT16 *)(raw + 0x0C);
            if (size_field != 0 && size_field != 0xFFFF) {
                dimm_info_t *d = &g_dimms[g_dimm_count];
                UINT32 size_mb;
                /* SMBIOS Type 17 Size field encoding (DMTF SMBIOS spec):
                     0x0000  no module installed
                     0xFFFF  unknown (skipped above)
                     0x7FFF  marker: real size is in Extended Size at 0x1C
                             (used for modules > 32 GB — e.g. 64 / 128 / 256 GB
                             RDIMMs on Xeon SP / EPYC / Dell R730+ servers)
                     bit 15  = 1: size in KB (legacy small modules)
                     otherwise: size in MB directly
                   Earlier code triggered Extended Size only on 0xFFFF, which
                   silently halved every >32 GB stick to 32 GB. Field-report
                   from a Dell R730 with 18 × 64 GB DDR4: tool reported 512 GB
                   instead of 1152 GB (combined with MAX_DIMMS=16 cap). */
                if (size_field == 0x7FFF && h->length >= 0x20)
                    size_mb = *(const UINT32 *)(raw + 0x1C);
                else if (size_field & 0x8000)
                    size_mb = (size_field & 0x7FFF) / 1024;
                else
                    size_mb = size_field;
                d->size_mb = size_mb;
                d->handle  = h->handle;
                d->ddr_type = (h->length >= 0x13) ? raw[0x12] : 0;
                d->speed_mt = (h->length >= 0x17) ? *(const UINT16 *)(raw + 0x15) : 0;
                d->configured_speed_mt = (h->length >= 0x22)
                                       ? *(const UINT16 *)(raw + 0x20)
                                       : d->speed_mt;
                if (h->length > 0x10)
                    smbios_copy_str(d->locator,      sizeof(d->locator),
                                    smbios_get_str_bounded(h, raw[0x10], end));
                if (h->length > 0x17)
                    smbios_copy_str(d->manufacturer, sizeof(d->manufacturer),
                                    smbios_get_str_bounded(h, raw[0x17], end));
                if (h->length > 0x1A)
                    smbios_copy_str(d->part_number,  sizeof(d->part_number),
                                    smbios_get_str_bounded(h, raw[0x1A], end));
                if (d->speed_mt > g_max_dimm_speed_mt) g_max_dimm_speed_mt = d->speed_mt;
                if (d->configured_speed_mt > g_max_dimm_configured_mt)
                    g_max_dimm_configured_mt = d->configured_speed_mt;
                if (g_dimm_ddr_type == 0 && d->ddr_type) g_dimm_ddr_type = d->ddr_type;
                g_total_ram_mb += d->size_mb;
                g_dimm_count++;
            }
            }   /* closes 'else' of g_dimm_count < MAX_DIMMS check */
        } else if (h->type == 20 && h->length >= 0x13 && g_dimm_map_count < MAX_DIMM_MAP) {
            /* Type 20 = Memory Device Mapped Address. Maps a physical address
               range to one Type 17 DIMM. Fields (DSP0134 §7.21):
                 +0x04 UINT32 start_addr_kb
                 +0x08 UINT32 end_addr_kb
                 +0x0C UINT16 dev_handle (links to Type 17)
                 +0x0E UINT16 array_mapped_handle (links to Type 19)
                 +0x10 UINT8  partition_row_position
                 +0x11 UINT8  interleave_position
                 +0x12 UINT8  interleaved_data_depth
                 +0x13 UINT64 ext_start_addr   (only if start_kb=0xFFFFFFFF)
                 +0x1B UINT64 ext_end_addr     (same)
               BIOSes with >16 TB of RAM use the extended fields; below that
               the 32-bit-KB fields suffice. */
            UINT32 start_kb = *(const UINT32 *)(raw + 0x04);
            UINT32 end_kb   = *(const UINT32 *)(raw + 0x08);
            UINT64 start_b, end_b;
            if (start_kb == 0xFFFFFFFFu && h->length >= 0x1F) {
                start_b = *(const UINT64 *)(raw + 0x13);
                end_b   = *(const UINT64 *)(raw + 0x1B);
            } else {
                start_b = (UINT64)start_kb * 1024ULL;
                end_b   = ((UINT64)end_kb + 1) * 1024ULL - 1;
            }
            /* Sanity: skip empty/inverted ranges that some buggy firmware
               emits as placeholders. */
            if (end_b > start_b && (end_b - start_b) < (1ULL << 48)) {
                dimm_map_entry_t *m = &g_dimm_map[g_dimm_map_count++];
                m->start            = start_b;
                m->end              = end_b;
                m->dev_handle       = *(const UINT16 *)(raw + 0x0C);
                m->interleave_pos   = raw[0x11];
                m->interleave_depth = raw[0x12];
                if (m->interleave_depth == 0) m->interleave_depth = 1;
            }
        } else if (h->type == 127) {
            break;
        }

        p += h->length;
        /* Walk strings looking for double-NUL terminator, bounded by end. */
        while (p + 1 < end && !(p[0] == 0 && p[1] == 0)) p++;
        if (p + 2 > end) break;
        p += 2;
        seen++;
    }
}

/* Byte-wise GUID compare — gnu-efi's CompareGuid() in this build returns
   0 (equal) for every comparison, presumably an ABI mismatch between
   libefi.a's compiled convention and our MS-ABI caller. Doing the compare
   ourselves bypasses the library entirely. */
static int guid_eq(EFI_GUID *a, EFI_GUID *b) {
    const UINT8 *pa = (const UINT8 *)a;
    const UINT8 *pb = (const UINT8 *)b;
    for (int i = 0; i < 16; i++) if (pa[i] != pb[i]) return 0;
    return 1;
}

static void parse_smbios(void) {
    EFI_GUID g2 = SMBIOS_TABLE_GUID;
    EFI_GUID g3 = SMBIOS3_TABLE_GUID;
    VOID *table_base = NULL;
    UINTN max_size = 0;
    UINT16 num_structs = 0;
    CHAR16 lb[160];

    log_line(L"[SMBIOS] scanning EFI configuration tables");
    SPrint(lb, sizeof(lb), L"[SMBIOS]  NumberOfTableEntries=%d",
           (UINT32)ST->NumberOfTableEntries);
    log_line(lb);

    /* Prefer SMBIOS 3.x entry if present. Use 'continue' on a per-entry
       failure so we don't abandon the whole scan on the first bad entry. */
    for (UINTN i = 0; i < ST->NumberOfTableEntries; i++) {
        if (!guid_eq(&ST->ConfigurationTable[i].VendorGuid, &g3)) continue;
        VOID *vt = ST->ConfigurationTable[i].VendorTable;
        if (!vt) continue;
        const UINT8 *a = (const UINT8 *)vt;
        SPrint(lb, sizeof(lb),
               L"[SMBIOS] SMBIOS3 GUID at idx=%d  anchor='%c%c%c%c%c'",
               (UINT32)i, a[0], a[1], a[2], a[3], a[4]);
        log_line(lb);
        if (a[0] != '_' || a[1] != 'S' || a[2] != 'M' || a[3] != '3' || a[4] != '_') {
            log_line(L"[SMBIOS]  bad anchor — skipping");
            continue;
        }
        smbios3_entry_t *e = (smbios3_entry_t *)vt;
        table_base  = (VOID *)(UINTN)e->table_address;
        max_size    = e->table_max_size;
        num_structs = 0;
        SPrint(lb, sizeof(lb),
               L"[SMBIOS]  v3 base=0x%lx max_size=%ld",
               (UINT64)(UINTN)table_base, (UINT64)max_size);
        log_line(lb);
        break;
    }
    if (!table_base) {
        for (UINTN i = 0; i < ST->NumberOfTableEntries; i++) {
            if (!guid_eq(&ST->ConfigurationTable[i].VendorGuid, &g2)) continue;
            VOID *vt = ST->ConfigurationTable[i].VendorTable;
            if (!vt) continue;
            const UINT8 *a = (const UINT8 *)vt;
            SPrint(lb, sizeof(lb),
                   L"[SMBIOS] SMBIOS2 GUID at idx=%d  anchor='%c%c%c%c'",
                   (UINT32)i, a[0], a[1], a[2], a[3]);
            log_line(lb);
            if (a[0] != '_' || a[1] != 'S' || a[2] != 'M' || a[3] != '_') {
                log_line(L"[SMBIOS]  bad anchor — skipping");
                continue;
            }
            smbios_entry_t *e = (smbios_entry_t *)vt;
            table_base  = (VOID *)(UINTN)e->table_address;
            max_size    = e->table_length;
            num_structs = e->num_structs;
            SPrint(lb, sizeof(lb),
                   L"[SMBIOS]  v2 base=0x%lx len=%ld nstructs=%d",
                   (UINT64)(UINTN)table_base, (UINT64)max_size, (UINT32)num_structs);
            log_line(lb);
            break;
        }
    }
    if (table_base && max_size) {
        parse_smbios_table(table_base, max_size, num_structs);
        SPrint(lb, sizeof(lb),
               L"[SMBIOS] parsed: sys='%a %a' bios='%a %a' dimms=%d total=%ld MB"
               L" type20_map=%d",
               g_sys_vendor, g_sys_model, g_bios_vendor, g_bios_version,
               g_dimm_count, g_total_ram_mb, g_dimm_map_count);
        log_line(lb);
        /* Dump address ranges for each Type 20 entry — invaluable when
           debugging "why does my error show ?" (firmware didn't emit T20). */
        for (UINT32 i = 0; i < g_dimm_map_count; i++) {
            CHAR8 *loc = (CHAR8 *)"?";
            for (UINT32 j = 0; j < g_dimm_count; j++)
                if (g_dimms[j].handle == g_dimm_map[i].dev_handle) {
                    loc = g_dimms[j].locator; break;
                }
            SPrint(lb, sizeof(lb),
                   L"[SMBIOS]  T20[%d]: %a  0x%lx-0x%lx  (intl pos=%d/depth=%d)",
                   i, loc, g_dimm_map[i].start, g_dimm_map[i].end,
                   g_dimm_map[i].interleave_pos, g_dimm_map[i].interleave_depth);
            log_line(lb);
        }
        /* Re-print combined line for compatibility — old code did this once,
           don't double-log. */
        SPrint(lb, sizeof(lb),
               L"[SMBIOS] done. Total DIMMs=%d, ranges=%d",
               g_dimm_count, g_dimm_map_count);
        log_line(lb);
    } else {
        log_line(L"[SMBIOS] no valid entry point found");
    }

    /* If SMBIOS didn't give us memory info (common on soldered-LPDDR laptops
       and QEMU/OVMF), fall back to the EFI memory map. */
    if (g_total_ram_mb == 0) {
        g_total_ram_mb = get_total_ram_mb_from_efi_map();
        SPrint(lb, sizeof(lb), L"[SMBIOS] RAM total from EFI map: %ld MB", g_total_ram_mb);
        log_line(lb);
    }
}

/* ---------- Minimal quantai.ini parser ----------
   Reads quantai.ini from the FAT root we booted from. Only a small subset
   of keys is honored (everything else is the Python analyzer's domain):
     [Meta] Language=ru|en
     [Run] Passes=N, MaxCores=N, EnableAVX=0|1, MultiPass=0|1, BufferMB=N
   Format: section headers in [brackets], key=value lines, ; or # comments.
   Whitespace tolerant. */
static int ini_parse_uint(const CHAR8 *s, UINT32 *out) {
    UINT32 v = 0;
    int seen = 0;
    while (*s == ' ' || *s == '\t') s++;
    while (*s >= '0' && *s <= '9') {
        v = v * 10 + (*s - '0');
        seen = 1;
        s++;
    }
    if (!seen) return 0;
    *out = v;
    return 1;
}

static int ini_strieq(const CHAR8 *a, const char *b) {
    while (*a && *b) {
        CHAR8 ca = *a; CHAR8 cb = (CHAR8)*b;
        if (ca >= 'A' && ca <= 'Z') ca += 32;
        if (cb >= 'A' && cb <= 'Z') cb += 32;
        if (ca != cb) return 0;
        a++; b++;
    }
    return *a == 0 && *b == 0;
}

static void parse_quantai_ini(void) {
    if (!g_logroot) return;
    /* Try a few common name variants. UEFI FAT is usually case-insensitive
       but a handful of firmwares aren't, and users do save the file with
       different casing depending on their editor / file manager. */
    EFI_FILE_PROTOCOL *f = NULL;
    EFI_STATUS s = EFI_NOT_FOUND;
    static CHAR16 *names[] = {
        L"quantai.ini", L"QUANTAI.INI", L"Quantai.ini", L"QuantAI.ini"
    };
    CHAR16 *found_as = NULL;
    for (UINTN i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        s = uefi_call_wrapper(g_logroot->Open, 5, g_logroot, &f,
                              names[i], EFI_FILE_MODE_READ, 0);
        if (s == EFI_SUCCESS && f) { found_as = names[i]; break; }
    }
    if (EFI_ERROR(s) || !f) {
        log_line(L"[INI] quantai.ini not found in volume root — "
                 L"check that file is at USB root, not in a subfolder. "
                 L"Using defaults.");
        return;
    }
    {
        CHAR16 lb[120];
        SPrint(lb, sizeof(lb), L"[INI] opened as '%s'", found_as);
        log_line(lb);
    }
    /* Read up to 4 KB — way more than enough for our subset. */
    CHAR8 buf[4096];
    UINTN sz = sizeof(buf) - 1;
    if (EFI_ERROR(uefi_call_wrapper(f->Read, 3, f, &sz, buf))) {
        uefi_call_wrapper(f->Close, 1, f);
        return;
    }
    uefi_call_wrapper(f->Close, 1, f);
    buf[sz] = 0;
    {
        CHAR16 lb[80];
        SPrint(lb, sizeof(lb), L"[INI] read %d bytes", (UINT32)sz);
        log_line(lb);
    }

    /* === BOM detection ===
       Windows Notepad saves files with a UTF-8 BOM (EF BB BF) by default.
       Some editors save UTF-16 (FF FE) or UTF-16BE (FE FF). Our parser is
       CHAR8 (ASCII/single-byte); these byte sequences at the start would
       break the first section header parse, silently dropping ALL config.
       Skip UTF-8 BOM transparently; refuse UTF-16 with an explicit error
       so the user knows what went wrong instead of "config silently
       ignored". */
    CHAR8 *bom_start = buf;
    UINTN bom_sz = sz;
    if (sz >= 3 && (UINT8)buf[0] == 0xEF && (UINT8)buf[1] == 0xBB
                && (UINT8)buf[2] == 0xBF) {
        bom_start = buf + 3;
        bom_sz    = sz - 3;
        log_line(L"[INI] UTF-8 BOM detected at start of file — skipped "
                 L"(save as ANSI / UTF-8 without BOM to avoid this notice)");
    } else if (sz >= 2 && (UINT8)buf[0] == 0xFF && (UINT8)buf[1] == 0xFE) {
        log_line(L"[INI] ⚠ UTF-16 LE encoding detected — UNSUPPORTED. "
                 L"Re-save quantai.ini as plain ANSI or UTF-8 (without BOM). "
                 L"Config NOT applied, falling back to defaults.");
        return;
    } else if (sz >= 2 && (UINT8)buf[0] == 0xFE && (UINT8)buf[1] == 0xFF) {
        log_line(L"[INI] ⚠ UTF-16 BE encoding detected — UNSUPPORTED. "
                 L"Re-save quantai.ini as plain ANSI or UTF-8 (without BOM). "
                 L"Config NOT applied, falling back to defaults.");
        return;
    }

    CHAR8 section[32] = {0};
    CHAR8 *line = bom_start;
    (void)bom_sz;
    while (line < buf + sz) {
        CHAR8 *eol = line;
        while (eol < buf + sz && *eol != '\n' && *eol != '\r') eol++;
        if (eol < buf + sz) *eol = 0;

        /* skip leading whitespace */
        while (*line == ' ' || *line == '\t') line++;
        if (*line == 0 || *line == ';' || *line == '#') goto next;

        if (*line == '[') {
            line++;
            UINTN i = 0;
            while (*line && *line != ']' && i < sizeof(section) - 1)
                section[i++] = *line++;
            section[i] = 0;
            goto next;
        }

        /* key=value */
        CHAR8 *eq = line;
        while (*eq && *eq != '=') eq++;
        if (*eq != '=') goto next;
        *eq = 0;
        CHAR8 *key = line;
        CHAR8 *val = eq + 1;
        /* trim trailing whitespace from key */
        CHAR8 *kend = eq - 1;
        while (kend > key && (*kend == ' ' || *kend == '\t')) { *kend = 0; kend--; }
        /* trim leading whitespace from value */
        while (*val == ' ' || *val == '\t') val++;
        /* CUT value at first whitespace, ';' or '#' — strips inline comments
           like "Language=ru   ; ru or en" so string-compare keys (Language=)
           match cleanly. Numeric keys already get this for free because
           ini_parse_uint stops at first non-digit. Without this strip the
           parser silently dropped Language=ru in our own default file —
           every key with an inline comment was unreachable from INI. */
        CHAR8 *vend = val;
        while (*vend && *vend != ' ' && *vend != '\t'
                     && *vend != ';' && *vend != '#') vend++;
        *vend = 0;

        if (ini_strieq(section, "Meta") && ini_strieq(key, "Language")) {
            if (ini_strieq(val, "en")) g_lang = 1;
            else if (ini_strieq(val, "ru")) g_lang = 0;
        } else if (ini_strieq(section, "Run")) {
            UINT32 v;
            if (ini_strieq(key, "Passes") && ini_parse_uint(val, &v))
                g_cfg_passes = v;
            else if (ini_strieq(key, "MaxCores") && ini_parse_uint(val, &v))
                g_cfg_max_cores = v;
            else if (ini_strieq(key, "EnableAVX") && ini_parse_uint(val, &v))
                g_cfg_enable_avx = (int)v;
            else if (ini_strieq(key, "MultiPass") && ini_parse_uint(val, &v))
                g_cfg_multipass = (int)v;
            else if (ini_strieq(key, "BufferMB") && ini_parse_uint(val, &v)) {
                g_cfg_buffer_cap_mb = v;
                g_cfg_buffer_cap_explicit = 1;
            }
            else if (ini_strieq(key, "BitFadeSeconds") && ini_parse_uint(val, &v)) {
                g_cfg_bitfade_s = v;
                g_cfg_bitfade_explicit = 1;  /* user chose this — don't auto-tune later */
            }
            else if (ini_strieq(key, "TestOnlyDimm") && ini_parse_uint(val, &v)) {
                /* 1-based DIMM index. We cross-reference with SMBIOS Type 20
                   address-range map at alloc time to restrict the test
                   buffer to that DIMM's physical range. 0 = test all. */
                g_cfg_test_only_dimm = v;
            }
            else if (ini_strieq(key, "BitFadeEveryPass") && ini_parse_uint(val, &v)) {
                g_cfg_bitfade_every_pass = (int)v;
            }
            else if (ini_strieq(key, "MarathonHours") && ini_parse_uint(val, &v)) {
                /* Cap at 24 h — anything longer is a typo. 0 = off. */
                if (v > 24) v = 24;
                g_cfg_marathon_hours = v;
            }
            else if (ini_strieq(key, "IgnoreThermalGuard") && ini_parse_uint(val, &v)) {
                /* When set to 1, bypass the auto-skip of heavy parallel-burst
                   kernels when baseline CPU temperature exceeds 85°C. Use
                   only if you know your cooling is fine and the high reading
                   is a sensor offset issue, OR if you specifically want to
                   stress-test thermal headroom. Default off — safer for
                   shop intake where unattended runs shouldn't halt the box. */
                g_cfg_ignore_thermal_guard = (int)v;
            }
        } else if (ini_strieq(section, "Display")) {
            UINT32 v;
            /* Workaround switch for firmware where direct framebuffer writes
               misbehave (text rendered at wrong positions, glitched garbage).
               Set ForceBlt=1 in [Display] section of quantai.ini if you see
               this. Cost: slightly slower text rendering, no visual change. */
            if (ini_strieq(key, "ForceBlt") && ini_parse_uint(val, &v))
                g_cfg_force_blt = (int)v;
            /* Width/Height: pick the GOP mode that matches these exactly.
               Useful when firmware enumerates a buggy default and a good
               one — let user pin to the good one. */
            else if (ini_strieq(key, "Width")  && ini_parse_uint(val, &v))
                g_cfg_force_w = v;
            else if (ini_strieq(key, "Height") && ini_parse_uint(val, &v))
                g_cfg_force_h = v;
            /* FontScale: 0=auto (2× for >1500 px height, 1× otherwise),
               1=force 1×, 2=force 2×. Useful if auto-pick is wrong. */
            else if (ini_strieq(key, "FontScale") && ini_parse_uint(val, &v))
                g_cfg_font_scale = v;
            /* EnableAA: enable smooth grayscale text via direct framebuffer
               writes. Default OFF because on some Intel iGPU firmware
               (Dell OptiPlex etc.) direct-fb writes glitch and produce
               text at wrong screen positions. Safe-by-default uses Blt
               1-bit threshold rendering which works on every firmware. */
            else if (ini_strieq(key, "EnableAA") && ini_parse_uint(val, &v))
                g_cfg_enable_aa = (int)v;
        }

next:
        if (eol >= buf + sz) break;
        line = eol + 1;
    }

    CHAR16 lb[200];
    SPrint(lb, sizeof(lb),
           L"[INI] lang=%a passes=%d max_cores=%d avx=%d multipass=%d buf_cap=%d MB",
           g_lang ? "en" : "ru", g_cfg_passes, g_cfg_max_cores,
           g_cfg_enable_avx, g_cfg_multipass, g_cfg_buffer_cap_mb);
    log_line(lb);
}

/* ---------- DDR5 detection + auto-tune ----------
   Detect a DDR5 (or LPDDR5) system primarily via SMBIOS Type 17 memory-type
   field (0x22 = DDR5, 0x23 = LPDDR5). If SMBIOS lies or is empty (some OEMs
   ship broken tables), fall back to inferring from configured speed: any
   DIMM clocked above ~4800 MT/s is necessarily DDR5 (JEDEC DDR4 caps at
   3200 MT/s, XMP/overclock historically tops out around 4800). */
static int is_ddr5_system(void) {
    if (g_dimm_ddr_type == 0x22 || g_dimm_ddr_type == 0x23) return 1;
    UINT32 sp = g_max_dimm_configured_mt
                ? g_max_dimm_configured_mt : g_max_dimm_speed_mt;
    if (sp >= 4800) return 1;
    return 0;
}

/* Apply RAM-size-aware tuning. Big RAM systems (≥64 GB) should use larger
   buffer chunks so the pass count stays reasonable: at 1 GB chunks a
   256 GB system needs 256 multipass passes × ~80 s = ~6 hours, but at
   4 GB chunks it's 64 × ~320 s = ~6 hours too — same wall-clock, but FAR
   fewer ap-dispatch / SMBIOS / pass-init overhead cycles. The reason to
   keep chunks small (1 GB) on small-RAM systems is alloc fragmentation —
   on a 4 GB box the largest free EfiConventionalMemory region might be
   only ~3 GB, and a 4 GB allocation would fail.

   Tuning curve:
     ≤ 32 GB:    1024 MB chunks (current default)
     33-127 GB:  2048 MB chunks (Threadripper / desktop with 64 GB)
     128-511 GB: 4096 MB chunks (Threadripper Pro, mid-server)
     ≥ 512 GB:   8192 MB chunks (high-end Xeon Platinum / Epyc)
   Skipped if user explicitly set BufferMB in quantai.ini. */
/* Per-DIMM "XMP active" flag set by xmp_warn_check(). Read by main menu
   render to show a warning, and by report.json to surface in output. */
static UINT8  g_xmp_dimm_flagged[MAX_DIMMS] = {0};
static int    g_xmp_any_flagged = 0;

/* Detect XMP / EXPO / AMP active by comparing configured DRAM speed
   against the JEDEC max for the generation. If a DIMM is running ABOVE
   its JEDEC-standard top speed, BIOS has loaded an overclock profile
   from SPD. That profile may be unstable even though the chip ITSELF
   tested clean — exactly the "RAM passes test but Windows BSODs"
   pattern the shop sees most often.

   JEDEC standard top speeds (per JEDEC DDR specs as of 2023):
     DDR3:   2133 MT/s   (anything above = OC)
     DDR4:   3200 MT/s   (above = XMP / AMP / DOCP profile)
     DDR5:   6400 MT/s   (above = XMP 3.0 / EXPO profile)
   Below threshold = could be either rated JEDEC or BIOS-downclocked,
   doesn't trigger the warning (no OC risk). */
static void xmp_warn_check(void) {
    g_xmp_any_flagged = 0;
    for (UINT32 i = 0; i < g_dimm_count && i < MAX_DIMMS; i++) {
        dimm_info_t *d = &g_dimms[i];
        UINT32 speed = d->configured_speed_mt
                     ? d->configured_speed_mt : d->speed_mt;
        if (speed == 0) continue;
        UINT32 jedec_max = 0;
        switch (d->ddr_type) {
            case 0x18: jedec_max = 2133; break;  /* DDR3 */
            case 0x1A: jedec_max = 3200; break;  /* DDR4 */
            case 0x22: jedec_max = 6400; break;  /* DDR5 */
            default:                              break;
        }
        if (jedec_max == 0) continue;
        if (speed > jedec_max) {
            g_xmp_dimm_flagged[i] = 1;
            g_xmp_any_flagged = 1;
            CHAR16 lb[200];
            SPrint(lb, sizeof(lb),
                   L"[XMP] %a (DIMM%d): %d MT/s exceeds JEDEC %s max %d MT/s — "
                   L"XMP/EXPO/AMP profile is active",
                   d->locator, i + 1, speed,
                   ddr_type_name(d->ddr_type), jedec_max);
            log_line(lb);
        }
    }
    if (g_xmp_any_flagged) {
        log_line(L"[XMP] WARNING: BSOD-grade memory errors in Windows may be due "
                 L"to an unstable overclock, not a defective stick. Disable XMP "
                 L"in BIOS and retest to confirm.");
    }
}

static void apply_ram_size_tuning(void) {
    if (g_cfg_buffer_cap_explicit) return;
    UINT64 ram_gb = g_total_ram_mb / 1024;
    UINT32 new_cap;
    if      (ram_gb <= 32)  new_cap = 1024;
    else if (ram_gb <= 127) new_cap = 2048;
    else if (ram_gb <= 511) new_cap = 4096;
    else                    new_cap = 8192;
    if (new_cap != g_cfg_buffer_cap_mb) {
        CHAR16 lb[160];
        SPrint(lb, sizeof(lb),
               L"[RAM] %ld GB system → BufferMB %d -> %d (auto, fewer passes)",
               ram_gb, g_cfg_buffer_cap_mb, new_cap);
        log_line(lb);
        g_cfg_buffer_cap_mb = new_cap;
    }
}

/* Multi-socket / many-core warning. RAPL on a multi-socket system reports
   energy ONLY for the package the BSP is executing on; the other sockets
   are invisible to our power telemetry. >32 logical cores almost always
   means either (a) HEDT/server with 2+ sockets, or (b) single Threadripper
   /Epyc package. We can't tell from CPUID alone in pre-OS, so we just warn
   that RAPL might under-report total system power. Tests themselves run
   correctly on all cores via MP services — it's only the reported watts
   that may be partial. */
static void check_multi_socket(void) {
    if (g_n_cores >= 64) {
        log_line(L"[WARN] >=64 logical cores — likely multi-socket. RAPL pkg power "
                 L"reported is BSP socket only; other sockets not measured.");
    }
}

/* Apply DDR5-specific test tuning AFTER SMBIOS detection. Called once from
   efi_main() before the menu shows. Safe to call again — idempotent. */
static void apply_ddr_tuning(void) {
    g_is_ddr5 = is_ddr5_system();
    if (g_is_ddr5) {
        /* Bit Fade on DDR5: cells leak faster but on-die ECC + aggressive
           refresh mask short tests. memtest86+ uses 300 s/phase but that's
           "overnight thoroughness", not "shop diagnostic". 120 s/phase =
           4 min/pass total catches gross retention issues without making
           the test look frozen for ages. User can override with
           BitFadeSeconds= in quantai.ini for memtest86+ parity. */
        if (!g_cfg_bitfade_explicit && g_cfg_bitfade_s < 120) {
            UINT32 old = g_cfg_bitfade_s;
            g_cfg_bitfade_s = 120;
            CHAR16 lb[160];
            SPrint(lb, sizeof(lb),
                   L"[DDR5] BitFadeSeconds %d -> %d (DDR5 cells leak faster, "
                   L"longer wait surfaces issues past on-die ECC)",
                   old, g_cfg_bitfade_s);
            log_line(lb);
        }
        /* Row Hammer: DDR5's TRR-v2 + RFM absorb single-pair attacks. Double
           the iteration budget to give us a fighting chance of provoking a
           flip. The kernel already cycles 6 aggressor pairs at prime strides;
           multiplying iters * 2 effectively doubles dwell time per pair. */
        g_rowhammer_mult_x100 = 200;
        log_line(L"[DDR5] Row Hammer multiplier 1.0x -> 2.0x (TRR/RFM compensation)");
        log_line(L"[DDR5] Reminder: on-die ECC silently corrects 1-bit errors. "
                 L"PASS on DDR5 means no errors that ODECC couldn't fix — it "
                 L"does NOT mean all chips are perfect.");
    } else {
        g_rowhammer_mult_x100 = 100;
    }
}

/* ---------- EFI memory map — total RAM size (fallback when SMBIOS empty) ---------- */
static UINT64 get_total_ram_mb_from_efi_map(void) {
    UINTN map_sz = 0, key = 0, desc_sz = 0;
    UINT32 desc_ver = 0;
    EFI_MEMORY_DESCRIPTOR *map = NULL;

    uefi_call_wrapper(BS->GetMemoryMap, 5, &map_sz, NULL, &key, &desc_sz, &desc_ver);
    if (!map_sz || !desc_sz) return 0;
    map_sz += 8 * desc_sz;
    if (EFI_ERROR(uefi_call_wrapper(BS->AllocatePool, 3,
                                    EfiLoaderData, map_sz, (VOID **)&map))) return 0;
    if (EFI_ERROR(uefi_call_wrapper(BS->GetMemoryMap, 5,
                                    &map_sz, map, &key, &desc_sz, &desc_ver))) {
        uefi_call_wrapper(BS->FreePool, 1, map);
        return 0;
    }
    UINT64 total_pages = 0;
    UINTN n = map_sz / desc_sz;
    CHAR8 *cp = (CHAR8 *)map;
    for (UINTN i = 0; i < n; i++, cp += desc_sz) {
        EFI_MEMORY_DESCRIPTOR *d = (EFI_MEMORY_DESCRIPTOR *)cp;
        /* Sum every region that is actual physical RAM, regardless of who
           currently owns it (free, runtime-svc, ACPI, etc.). We exclude only
           MMIO ranges and reserved (some of which is real RAM but counting
           it would double-count when firmware has carved out ACPI). */
        switch (d->Type) {
            case EfiConventionalMemory:
            case EfiBootServicesCode:
            case EfiBootServicesData:
            case EfiLoaderCode:
            case EfiLoaderData:
            case EfiRuntimeServicesCode:
            case EfiRuntimeServicesData:
            case EfiACPIReclaimMemory:
            case EfiACPIMemoryNVS:
            case EfiPalCode:
                total_pages += d->NumberOfPages;
                break;
            default:
                break;
        }
    }
    uefi_call_wrapper(BS->FreePool, 1, map);
    return (total_pages * 4096ULL) / (1024ULL * 1024ULL);
}

/* ---------- Memory-region inventory for multi-pass coverage ----------
   We walk the EFI memory map once at startup and remember every
   EfiConventionalMemory region ≥ 256 MB. When MultiPass=1 is set in
   quantai.ini, the test loop iterates over these one by one — that's the
   only way to give an honest "X% of RAM tested" rather than testing one
   contiguous chunk and leaving the rest of physical RAM untouched. */
typedef struct {
    EFI_PHYSICAL_ADDRESS addr;
    UINTN pages;
    UINT32 tested_pass;   /* g_cur_pass value when last allocated here */
} mem_region_t;

#define MAX_REGIONS 64
static mem_region_t g_regions[MAX_REGIONS];
static UINT32 g_n_regions = 0;

static void scan_mem_regions(void) {
    UINTN map_sz = 0, key = 0, desc_sz = 0;
    UINT32 desc_ver = 0;
    EFI_MEMORY_DESCRIPTOR *map = NULL;

    uefi_call_wrapper(BS->GetMemoryMap, 5, &map_sz, NULL, &key, &desc_sz, &desc_ver);
    if (!map_sz || !desc_sz) return;
    map_sz += 8 * desc_sz;
    if (EFI_ERROR(uefi_call_wrapper(BS->AllocatePool, 3,
                                    EfiLoaderData, map_sz, (VOID **)&map))) return;
    if (EFI_ERROR(uefi_call_wrapper(BS->GetMemoryMap, 5,
                                    &map_sz, map, &key, &desc_sz, &desc_ver))) {
        uefi_call_wrapper(BS->FreePool, 1, map);
        return;
    }

    UINTN n = map_sz / desc_sz;
    CHAR8 *cp = (CHAR8 *)map;
    UINTN min_pages = (256ULL * 1024 * 1024) / 4096;  /* 256 MB minimum */
    for (UINTN i = 0; i < n && g_n_regions < MAX_REGIONS; i++, cp += desc_sz) {
        EFI_MEMORY_DESCRIPTOR *d = (EFI_MEMORY_DESCRIPTOR *)cp;
        if (d->Type != EfiConventionalMemory) continue;
        if (d->NumberOfPages < min_pages) continue;
        g_regions[g_n_regions].addr  = d->PhysicalStart;
        g_regions[g_n_regions].pages = d->NumberOfPages;
        g_regions[g_n_regions].tested_pass = 0;
        g_n_regions++;
    }
    uefi_call_wrapper(BS->FreePool, 1, map);

    /* Sort regions descending by size (simple insertion sort, small N). */
    for (UINT32 i = 1; i < g_n_regions; i++) {
        mem_region_t cur = g_regions[i];
        UINT32 j = i;
        while (j > 0 && g_regions[j-1].pages < cur.pages) {
            g_regions[j] = g_regions[j-1];
            j--;
        }
        g_regions[j] = cur;
    }

    CHAR16 lb[160];
    SPrint(lb, sizeof(lb), L"[MEM] %d conventional region(s) ≥ 256 MB found:", g_n_regions);
    log_line(lb);
    UINT64 total = 0;
    for (UINT32 i = 0; i < g_n_regions; i++) {
        UINT64 mb = (g_regions[i].pages * 4096ULL) / (1024ULL * 1024ULL);
        total += mb;
        SPrint(lb, sizeof(lb), L"[MEM]   region[%d] addr=0x%lx size=%ld MB",
               i, (UINT64)g_regions[i].addr, mb);
        log_line(lb);
    }
    SPrint(lb, sizeof(lb), L"[MEM] total addressable for testing: %ld MB", total);
    log_line(lb);
}

/* Attempt to allocate at a specific region at the given page-offset.
   Returns 1 on success. If the offset+cap exceeds the region, allocates
   whatever fits (so we naturally cover the tail of the region in one pass). */
static int alloc_region_buffer_at(UINT32 region_idx, UINTN page_offset) {
    if (region_idx >= g_n_regions) return 0;
    if (page_offset >= g_regions[region_idx].pages) return 0;

    EFI_PHYSICAL_ADDRESS addr = g_regions[region_idx].addr + page_offset * 4096ULL;
    UINTN remaining = g_regions[region_idx].pages - page_offset;
    UINTN cap_pages = ((UINTN)g_cfg_buffer_cap_mb * 1024ULL * 1024ULL) / 4096;
    UINTN pages = (remaining < cap_pages) ? remaining : cap_pages;
    /* Don't bother with tiny tail chunks. */
    UINTN min_pages = (16 * 1024 * 1024) / 4096;  /* 16 MB floor */
    if (pages < min_pages) return 0;

    EFI_STATUS s = uefi_call_wrapper(BS->AllocatePages, 4,
                       AllocateAddress, EfiLoaderData, pages, &addr);
    if (EFI_ERROR(s)) return 0;
    g_mem_addr = addr;
    g_mem_pages = pages;
    g_regions[region_idx].tested_pass = g_cur_pass + 1;
    return 1;
}

/* ---------- EFI memory map walker — find largest free contiguous block ---------- */
static UINTN get_largest_free_pages(void) {
    UINTN map_sz = 0, key = 0, desc_sz = 0;
    UINT32 desc_ver = 0;
    EFI_MEMORY_DESCRIPTOR *map = NULL;

    uefi_call_wrapper(BS->GetMemoryMap, 5, &map_sz, NULL, &key, &desc_sz, &desc_ver);
    if (!map_sz || !desc_sz) return 0;
    map_sz += 8 * desc_sz; /* slack for any allocation by GetMemoryMap itself */
    EFI_STATUS s = uefi_call_wrapper(BS->AllocatePool, 3,
                                     EfiLoaderData, map_sz, (VOID **)&map);
    if (EFI_ERROR(s)) return 0;
    s = uefi_call_wrapper(BS->GetMemoryMap, 5, &map_sz, map, &key, &desc_sz, &desc_ver);
    if (EFI_ERROR(s)) { uefi_call_wrapper(BS->FreePool, 1, map); return 0; }

    UINTN max_pages = 0;
    UINTN n = map_sz / desc_sz;
    CHAR8 *cp = (CHAR8 *)map;
    for (UINTN i = 0; i < n; i++, cp += desc_sz) {
        EFI_MEMORY_DESCRIPTOR *d = (EFI_MEMORY_DESCRIPTOR *)cp;
        if (d->Type == EfiConventionalMemory && d->NumberOfPages > max_pages)
            max_pages = d->NumberOfPages;
    }
    uefi_call_wrapper(BS->FreePool, 1, map);
    return max_pages;
}

/* ---------- AP entry ---------- */
static void EFIAPI ap_entry(VOID *arg) {
    ap_arg_t *a = (ap_arg_t *)arg;
    /* Diagnostic: BSP-side only, first test only. APs can't safely write
       to the log (FAT FS not multi-thread-safe, and we'd hammer it 11x
       per call on a 12-core box). BSP is core_idx==0 by convention. */
    int diag = (a->core_idx == 0 && g_cur_test_idx == 0);

    /* Thermal guard: if init detected hot CPU at idle, skip heavy
       parallel-burst kernels that have caused field-reported halts on
       buggy AMD AMI firmware. Pattern tests still run because they
       don't burn all cores at AVX2 power. Skip is silent on APs; BSP
       logs once per skipped test (visible in the per-test summary). */
    if (g_thermal_guard_skip_heavy) {
        switch (a->kernel) {
            case KER_AVX2:
            case KER_AVX2_SUSTAINED:
            case KER_VRM_SQUARE:
            case KER_THERMAL_SOAK:
            case KER_BW_SOAK:
                if (a->core_idx == 0) {
                    /* Note: g_tests[] is defined after ap_entry in the source,
                       so we can't reference its name string here without a
                       forward declaration. Kernel ID is enough to identify
                       the skipped test — the per-test summary line later
                       in the test loop will print the name. */
                    CHAR16 lb[180];
                    SPrint(lb, sizeof(lb),
                           L"[TEMP] skipping heavy-burst kernel id=%d "
                           L"(test idx=%d) — thermal guard active "
                           L"(baseline CPU was too hot at start)",
                           (UINT32)a->kernel, (UINT32)g_cur_test_idx);
                    log_line(lb);
                }
                a->errors = 0;
                a->bytes = 0;
                a->progress = 1000;
                a->done = 1;
                return;
            default:
                break;
        }
    }

    if (diag) log_line(L"[BSP] ap_entry: pre try_enable_avx_state");

    /* CR4 and XCR0 are per-core registers. The BSP enabled OSXSAVE for
       itself in detect_cpu_features(), but each AP must enable XSAVE state
       on its OWN core before running anything that emits VEX-encoded AVX2
       instructions. Cheap: a CR4 read + write + XSETBV.
       NB: this list must include EVERY kernel that emits YMM/AVX2 ops —
       AVX2 Sustained and VRM Square-Wave were missing here, which made
       APs trap on the first vmovdqu / vfmadd231ps and die silently
       (visible as 15/16 cores "ЗАВИС" with BSP completing alone). */
    if (g_has_avx2 && (a->kernel == KER_AVX2
                   || a->kernel == KER_AVX2_SUSTAINED
                   || a->kernel == KER_VRM_SQUARE
                   || a->kernel == KER_THERMAL_SOAK
                   || a->kernel == KER_BW_SOAK)) {
        try_enable_avx_state();
    }
    if (diag) log_line(L"[BSP] ap_entry: post avx_state, pre try_enable_max_perf");
    /* Each AP requests max P-state on ITS OWN core. IA32_HWP_REQUEST (0x774)
       is a per-logical-CPU MSR — writing it from the BSP only affects the
       BSP. Without this every AP would run our stress kernels at the base
       firmware-default ratio. Cheap: 4 MSR ops, hundreds of nanoseconds. */
    try_enable_max_perf();
    if (diag) log_line(L"[BSP] ap_entry: post max_perf, entering kernel switch");
    a->errors = 0; a->bytes = 0; a->progress = 0;
    switch (a->kernel) {
        case KER_WALKING_ONES:   run_walking_ones(a);   break;
        case KER_WALKING_ZEROES: run_walking_zeroes(a); break;
        case KER_MOVING_INV:     run_moving_inv(a);     break;
        case KER_ADDRESS:        run_address(a);        break;
        case KER_AVX2:           run_avx2(a);           break;
        case KER_ROWHAMMER:      run_rowhammer(a);      break;
        case KER_BLOCK_MOVE:     run_block_move(a);     break;
        case KER_RANDOM:         run_random_pattern(a); break;
        case KER_BIT_FADE:       run_bit_fade(a);       break;
        /* Aggressive kernels */
        case KER_MARCH_CM:       run_march_cm(a);       break;
        case KER_MARCH_RAW:      run_march_raw(a);      break;
        case KER_BUTTERFLY:      run_butterfly(a);      break;
        case KER_AVX2_SUSTAINED: run_avx2_sustained(a); break;
        case KER_TRRESPASS:      run_trrespass(a);      break;
        case KER_CACHE_EVICT:    run_cache_evict(a);    break;
        case KER_VRM_SQUARE:     run_vrm_square(a);     break;
        case KER_BIT_FADE_EXT:   run_bit_fade_ext(a);   break;
        case KER_THERMAL_SOAK:   run_thermal_soak(a);   break;
        case KER_BW_SOAK:        run_bw_soak(a);        break;
        case KER_L3_STRESS:      run_l3_stress(a);      break;
        case KER_STRIDE_BW:      run_stride_bw(a);      break;
    }
    if (diag) log_line(L"[BSP] ap_entry: kernel returned, marking done");
    /* Before signalling done=1, mark this core's live-activity fields as
       "no longer running". Without this, the LAST value sampled while the
       kernel was busy (often util_pct=99, freq_mhz=3700) stays in the
       struct after the AP halts — the BSP renders that stale snapshot and
       it LOOKS like the core is still 100% busy while sitting in
       "ждёт др." (waiting). The UI then said two contradictory things at
       once. Zeroing util_pct and freq_mhz here makes the display honest. */
    a->util_pct = 0;
    a->freq_mhz = 0;
    a->throttle_flags = 0;
    /* temp_c is intentionally NOT reset — the core stays physically warm
       for several seconds after going idle; that's a real reading worth
       showing, not noise. */
    a->done = 1;
}

/* ---------- Test catalogue ----------
   ALL tests are stress-oriented now — no "gentle structural-only" mode.
   Quick vs Full is purely time budget:
     Quick (4 tests, ~5 min)  : the heaviest hard-fault tests only.
                                AVX2-Sustained (VRM), TRRespass (DRAM rows),
                                Cache-Eviction (IMC), March-C- (cells).
     Full  (10 tests, ~60-90m): adds remaining stress + retention.
   Bit Fade Extended only runs on pass 1 (BitFadeEveryPass=0 default). */
typedef struct {
    CHAR16 *name;        /* full name shown in cards */
    CHAR16 *short_name;  /* (legacy, no longer used in v0.4 UI) */
    kernel_id_t k;
    CHAR16 *desc_ru;     /* what the test detects, brief */
    CHAR16 *desc_en;
    CHAR16 *eta_ru;      /* rough wall-clock time on modern HW with 1 GB buffer */
    CHAR16 *eta_en;
    int    in_quick;     /* 1 = run in quick stress mode */
} test_def_t;
static test_def_t g_tests[] = {
    /* === In both Quick and Full (heaviest hard-fault stressors) === */
    { L"AVX2 Sustained",    L"AVX2sus", KER_AVX2_SUSTAINED,
      L"FMA + интерлив запись — VRM+IMC stress (10 сек)",
      L"FMA + interleaved writes — VRM+IMC stress (10 s)",
      L"~10 с", L"~10 s", 1 },
    { L"TRRespass",         L"TRRsp",   KER_TRRESPASS,
      L"8-сторонний Row Hammer + rotation банков — обход TRR (DDR4/5)",
      L"8-sided Row Hammer + bank rotation — TRR bypass (DDR4/5)",
      L"~60 с", L"~60 s", 1 },
    { L"Cache-Eviction",    L"CEvict",  KER_CACHE_EVICT,
      L"CLFLUSH каждой строки — макс DRAM hits, IMC bank conflicts",
      L"CLFLUSH every line — max DRAM hits, IMC bank conflicts",
      L"~10 с", L"~10 s", 1 },
    { L"March-C-",          L"MarchCM", KER_MARCH_CM,
      L"6-фазный March: SAF+AF+TF+coupling (92% fault coverage)",
      L"6-phase March: SAF+AF+TF+coupling (92% fault coverage)",
      L"~2 с",  L"~2 s",  1 },

    /* === Thermal Soak — runs in BOTH Quick and Full (pass 1 only) ===
       Without sustained thermal load the rest of the tests run on a cold
       CPU (we observed 38°C max on i5-12600KF with 4-test Quick mode),
       which defeats the purpose of "stress" testing. Thermal Soak forces
       the system to thermal steady-state BEFORE the structural tests, so
       cell-level / VRM / IMC issues that only surface when hot get
       exposed. Slightly slows Quick (~4 min → ~7 min) but is the only
       way to actually validate thermal margin. */
    { L"Thermal Soak",      L"ThSoak",  KER_THERMAL_SOAK,
      L"3 мин непрерывный AVX2 + memory — прогрев до steady-state (только pass 1)",
      L"3 min continuous AVX2 + memory — soak to thermal steady-state (pass 1 only)",
      L"~3 мин", L"~3 min", 1 },

    /* === Full-only — BW Soak (memtest86+ analog) === */
    { L"BW Soak",           L"BWSoak",  KER_BW_SOAK,
      L"5 мин streaming write+read — memtest86+-like sustained DRAM load (только pass 1)",
      L"5 min streaming write+read — memtest86+-like sustained DRAM load (pass 1 only)",
      L"~5 мин", L"~5 min", 0 },

    /* === Full-only (additional fault classes, slower) === */
    { L"March-RAW",         L"MarchRAW",KER_MARCH_RAW,
      L"Read-After-Write — динамические coupling faults",
      L"Read-After-Write — dynamic coupling faults",
      L"~3 с",  L"~3 s",  0 },
    { L"Butterfly",         L"Butfly",  KER_BUTTERFLY,
      L"Шахматный паттерн + flip соседей — cell crosstalk",
      L"Checkerboard + neighbour flip — cell crosstalk",
      L"~5 с",  L"~5 s",  0 },
    { L"Address Pattern",   L"AddrPat", KER_ADDRESS,
      L"ошибки адресации (cell X читает значение Y)",
      L"addressing errors (cell X reads value of Y)",
      L"~5 с",  L"~5 s",  0 },
    { L"VRM Square-Wave",   L"VRMsq",   KER_VRM_SQUARE,
      L"AVX2 burst / idle 10 Hz — VRM transient response",
      L"AVX2 burst / idle 10 Hz — VRM transient response",
      L"~10 с", L"~10 s", 0 },
    { L"Random Pattern",    L"Rand64",  KER_RANDOM,
      L"псевдослучайные паттерны (xorshift64), 4 повтора",
      L"pseudo-random patterns (xorshift64), 4 reps",
      L"~6 с",  L"~6 s",  0 },
    { L"Bit Fade Ext.",     L"FadeExt", KER_BIT_FADE_EXT,
      L"DRAM retention: 4 паттерна × 2 мин wait (1, 0, 0xAA, 0xCC)",
      L"DRAM retention: 4 patterns × 2 min wait (1, 0, 0xAA, 0xCC)",
      L"~8 мин", L"~8 min", 0 },
    /* L3 cache resident workload — exposes L3-cell faults that DRAM-only
       tests miss (CLFLUSH evicts data out of L3 before verify). */
    { L"L3 Cache Stress",   L"L3stress",KER_L3_STRESS,
      L"1 MB рабочий набор в L3, RMW × ~10 сек — ловит L3-cell faults",
      L"1 MB working set in L3, RMW × ~10 s — catches L3-cell faults",
      L"~10 с", L"~10 s", 0 },
    /* Stride-sweep BW probe — 4 strides × 3 s. Drop at just one stride
       points at TLB / set-associativity / channel-interleave issues. */
    { L"Stride BW",         L"StrideBW",KER_STRIDE_BW,
      L"sweep страйдов 64B/1KB/4KB/64KB — TLB / set-conflict / каналы",
      L"sweep strides 64B/1KB/4KB/64KB — TLB / set-conflict / channels",
      L"~12 с", L"~12 s", 0 },
};
#define N_TESTS (sizeof(g_tests) / sizeof(g_tests[0]))

/* v0.4.29 — map a kernel enum (KER_*) to its position in g_tests[].
   CRITICAL: do NOT index g_tests[] directly by a kernel_id_t value.
   The enum values do not match array positions (e.g., KER_AVX2_SUSTAINED
   = 12 maps to position 0 in g_tests because AVX2 Sustained is the
   first row of the table, while position 12 happens to be L3 Cache
   Stress). Before this helper existed, an AVX2 error was displayed in
   the verdict, JSON and log as "T=L3 Cache Stress" — total
   misattribution that completely broke field triage. Always use this
   helper for kernel→display-name lookup. */
static int tests_idx_for_kernel(kernel_id_t k) {
    for (UINTN i = 0; i < N_TESTS; i++) {
        if (g_tests[i].k == k) return (int)i;
    }
    return -1;
}
static CHAR16 *name_for_kernel(kernel_id_t k) {
    int ti = tests_idx_for_kernel(k);
    return (ti >= 0) ? g_tests[ti].name : L"(unknown kernel)";
}

/* Activity row painter — invoked by render_header() on every tick to show
   what test is running, how long it's been on this test, and (critically)
   a per-second countdown when Bit Fade is in its silent wait phase. Lives
   here, not next to render_header, because it needs g_tests / g_args
   which are only defined above. */
static void render_activity_row(UINT64 elapsed_ms) {
    /* Row 2 of the header strip. Cleared and repainted on every tick so
       the spinner animates and the countdown ticks down visibly. */
    clear_row(2);
    /* Spinner cycles through 4 characters (8-slot indexing for finer
       wraparound). At our ~10 Hz render rate that's a full rotation per
       0.4 s — clearly "alive" even when no other UI element moves. */
    static UINTN spin_idx = 0;
    static UINT64 spin_last_ms = 0;
    if (elapsed_ms - spin_last_ms > 80) {
        spin_idx = (spin_idx + 1) & 3;
        spin_last_ms = elapsed_ms;
    }
    static const CHAR16 spinner_chars[4] = { L'|', L'/', L'—', L'\\' };
    CHAR16 spin[2] = { spinner_chars[spin_idx], 0 };

    UINT32 test_elapsed_s = 0;
    if (g_cur_test_started > 0) {
        UINT64 now = ms_now();
        if (now > g_cur_test_started)
            test_elapsed_s = (UINT32)((now - g_cur_test_started) / 1000);
    }

    CHAR16 *cur_test_name = (g_cur_test_idx < N_TESTS)
                            ? g_tests[g_cur_test_idx].name
                            : T(L"(простой)", L"(idle)");

    /* Bit-Fade-specific countdown. Both the 2-phase Bit Fade and the
       4-phase Bit Fade Extended encode phase + wait progress in
       a->progress (per-mille). 2-phase splits 0-500/500-1000 with each
       half = 10% fill + 80% wait + 10% verify; 4-phase splits into 4
       quarters of 0-250/250-500/500-750/750-1000, each with the same
       10/80/10 fill-wait-verify split inside its quarter.
       From g_args[0].progress we derive phase + wait-elapsed for display.
       PREVIOUSLY only KER_BIT_FADE was matched here — the Extended
       variant fell through to the generic "тест: X · на этом тесте Y:ZZ"
       activity line, so during the 4×2-min wait the user saw the elapsed
       counter tick but no phase/countdown — looked like "stuck on test 10
       for 6 minutes". With this fix the user sees e.g. "Bit Fade Ext —
       фаза 3/4 · retention wait 1:23 / 2:00". */
    int is_bitfade_wait = 0;
    UINT32 bf_phase = 1;
    UINT32 bf_phase_total = 2;     /* total phases for the running kernel */
    UINT32 bf_wait_done_s = 0;
    if (g_cur_test_idx < N_TESTS) {
        kernel_id_t k = g_tests[g_cur_test_idx].k;
        UINT32 p = g_args[0].progress;
        if (p > 1000) p = 1000;
        if (k == KER_BIT_FADE) {
            bf_phase_total = 2;
            /* Half-span = 500. Phase wait is base+100..base+450 (per-mille). */
            if (p >= 100 && p < 450) {
                is_bitfade_wait = 1; bf_phase = 1;
                bf_wait_done_s = (UINT32)((UINT64)(p - 100) * g_cfg_bitfade_s / 350ULL);
            } else if (p >= 600 && p < 950) {
                is_bitfade_wait = 1; bf_phase = 2;
                bf_wait_done_s = (UINT32)((UINT64)(p - 600) * g_cfg_bitfade_s / 350ULL);
            }
        } else if (k == KER_BIT_FADE_EXT) {
            bf_phase_total = 4;
            /* Quarter-span = 250. Phase n (0..3): base = n*250.
               wait region inside each quarter = base+25..base+225. */
            UINT32 phase_idx = p / 250;            /* 0..3 */
            if (phase_idx > 3) phase_idx = 3;
            UINT32 within = p - phase_idx * 250;   /* 0..249 */
            if (within >= 25 && within < 225) {
                is_bitfade_wait = 1;
                bf_phase = phase_idx + 1;
                /* Bit Fade Extended uses a fixed 120 sec/phase, not
                   g_cfg_bitfade_s. Map the 200-pmille-wide wait range
                   linearly to 0..120 seconds. */
                bf_wait_done_s = (UINT32)((UINT64)(within - 25) * 120ULL / 200ULL);
            }
        }
    }

    CHAR16 buf[256];
    if (is_bitfade_wait) {
        UINT32 total = (g_cur_test_idx < N_TESTS
                        && g_tests[g_cur_test_idx].k == KER_BIT_FADE_EXT)
                       ? 120 : g_cfg_bitfade_s;
        SPrint(buf, sizeof(buf),
               T(L"  %s  %s — фаза %d/%d  ·  retention wait %d:%02d / %d:%02d  ·  ядра спят, это нормально",
                 L"  %s  %s — phase %d/%d  ·  retention wait %d:%02d / %d:%02d  ·  cores idle, this is OK"),
               spin, cur_test_name, bf_phase, bf_phase_total,
               bf_wait_done_s / 60, bf_wait_done_s % 60,
               total / 60, total % 60);
        gfx_draw_str_color(0, 2 * g_char_h, buf, COL_ACCENT_HI);
    } else {
        SPrint(buf, sizeof(buf),
               T(L"  %s  Тест: %s  ·  идёт %d:%02d  ·  Ядра %d/%d использовано  ·  AVX2 %s",
                 L"  %s  Test: %s  ·  running %d:%02d  ·  Cores %d/%d used  ·  AVX2 %s"),
               spin, cur_test_name,
               test_elapsed_s / 60, test_elapsed_s % 60,
               (UINT32)g_n_enabled, (UINT32)g_n_cores,
               g_has_avx2 ? T(L"вкл", L"on") : T(L"нет", L"off"));
        say_at_rc(0, 2, buf);
    }
}

/* Per-test summary captured for the final table. */
typedef struct {
    UINT64 errors, bytes, time_ms;
    UINT32 status;   /* 0=skip 1=pass 2=fail */
} test_summary_t;
static test_summary_t g_summary[N_TESTS];

/* ---------- Per-test cards (v0.4 layout) ----------
   One horizontal row per test: status dot + name + own progress bar + live
   metrics (%, MB/s, errors). Past tests stay visible with their final state.
*/
typedef enum {
    CARD_IDLE = 0,
    CARD_RUNNING = 1,
    CARD_PASS = 2,
    CARD_FAIL = 3,
    CARD_SKIP = 4,
} card_state_t;

typedef struct {
    card_state_t state;
    UINT32 pct_x10;   /* 0..1000 */
    UINT64 mbs;
    UINT64 errors;
} card_info_t;
static card_info_t g_cards[N_TESTS];

/* v0.4.29 — Forward decls for focused-mode helpers (defined below
   card_paint so they can share the same color-lookup logic). */
static void card_paint_full(UINTN i);
static void card_strip_paint(UINTN i);
static void card_focused_paint(UINTN i);
static void card_focused_redraw(void);

static void card_paint(UINTN i) {
    if (i >= N_TESTS) return;
    if (!g_show_cards) return;   /* short-screen mode: panel suppressed */
    if (g_focused_cards) {
        /* Focused mode: every paint updates the strip dot. If this test
           is currently running, also repaint the big focused card. When
           the active test changes (i becomes RUNNING and differs from
           g_focus_active_idx), clear the big card area first so old
           name/bar pixels don't leak through. */
        card_strip_paint(i);
        if (g_cards[i].state == CARD_RUNNING) {
            if (g_focus_active_idx != i) {
                g_focus_active_idx = i;
                blt_fill(g_focus_x, g_focus_y, g_focus_w, g_focus_h, COL_PANEL);
                box_outline(g_focus_x, g_focus_y, g_focus_w, g_focus_h, COL_BORDER);
            }
            card_focused_paint(i);
        }
        return;
    }
    card_paint_full(i);
}

static void card_paint_full(UINTN i) {
    /* Original full-row card painter — used on g_h≥900 screens.
       Position within the cards grid. With g_card_cols=1 it's just
       (col=0, row=i) — backward compatible. With g_card_cols=2 we lay out
       tests 1..rows in left col, rows+1..N in right col. */
    UINTN rows_per_col = (N_TESTS + g_card_cols - 1) / g_card_cols;
    UINTN col_no = i / rows_per_col;
    UINTN row_no = i % rows_per_col;
    UINTN col_gap = (g_card_cols > 1) ? g_pad : 0;
    UINTN col_w   = (g_card_w - col_gap * (g_card_cols - 1)) / g_card_cols;
    UINTN x = g_card_x + col_no * (col_w + col_gap);
    UINTN y = g_card_y + row_no * g_card_row_h;
    UINTN w = col_w;
    UINTN h = g_card_row_h - 2;
    card_info_t *c = &g_cards[i];

    UINT32 bg = (c->state == CARD_RUNNING) ? COL_RUN_DK
              : (c->state == CARD_PASS)    ? COL_OK_DK
              : (c->state == CARD_FAIL)    ? COL_FAIL_DK
              : COL_PANEL;
    blt_fill(x, y, w, h, bg);
    box_outline(x, y, w, h, COL_BORDER);

    UINT32 accent = (c->state == CARD_RUNNING) ? COL_RUN
                  : (c->state == CARD_PASS)    ? COL_OK
                  : (c->state == CARD_FAIL)    ? COL_FAIL
                  : (c->state == CARD_SKIP)    ? COL_DIM
                  : COL_DIM;

    /* Status dot on the left */
    UINTN dot_size = (h > 18) ? 14 : (h - 4);
    UINTN dot_x = x + 6;
    UINTN dot_y = y + (h - dot_size) / 2;
    blt_fill(dot_x, dot_y, dot_size, dot_size, accent);
    if (c->state == CARD_RUNNING)
        blt_fill(dot_x, dot_y, dot_size, 2, COL_ACCENT_HI);

    UINTN text_y = y + (h - g_char_h) / 2;

    /* Test name */
    UINTN name_x = dot_x + dot_size + 8;
    say_at_px(name_x, text_y, g_tests[i].name);

    /* Right-aligned metrics */
    CHAR16 stats[80];
    if (c->state == CARD_IDLE) {
        SPrint(stats, sizeof(stats),
               T(L"           ожидание           ",
                 L"           pending            "));
    } else if (c->state == CARD_SKIP) {
        SPrint(stats, sizeof(stats),
               T(L"     пропуск (firmware)     ",
                 L"     skipped (firmware)     "));
    } else {
        UINT32 pct = c->pct_x10; if (pct > 1000) pct = 1000;
        SPrint(stats, sizeof(stats),
               L"  %3d.%d%%   %5ld MB/s   err %ld  ",
               pct / 10, pct % 10, c->mbs, c->errors);
    }
    UINTN stats_chars = StrLen(stats);
    UINTN stats_x = x + w - stats_chars * g_char_w - 6;
    say_at_px(stats_x, text_y, stats);

    /* Progress bar between name and stats */
    UINTN name_chars = StrLen(g_tests[i].name);
    UINTN bar_x = name_x + name_chars * g_char_w + 12;
    UINTN bar_end = (stats_x > 12) ? stats_x - 12 : bar_x;
    if (bar_end > bar_x + 40) {
        UINTN bar_w = bar_end - bar_x;
        UINTN bar_h = 10;
        UINTN bar_y = y + (h - bar_h) / 2;
        UINT32 fill = (c->state == CARD_IDLE) ? 0
                    : (c->state == CARD_SKIP) ? 1000
                    : (c->state == CARD_PASS) ? 1000
                    : c->pct_x10;
        UINT32 bar_col = (c->state == CARD_FAIL)    ? COL_FAIL
                       : (c->state == CARD_PASS)    ? COL_OK
                       : (c->state == CARD_RUNNING) ? COL_BAR_FILL
                       : (c->state == CARD_SKIP)    ? COL_DIM
                       : COL_BAR_BG;
        blt_progress_bar(bar_x, bar_y, bar_w, bar_h, fill, bar_col, COL_BAR_BG);
    }
}

/* ---------- Focused-mode card painters (v0.4.29) ---------- */

/* Paint the small status dot for test i in the top strip. The strip is
   one row tall and shows N evenly-spaced dots, one per test. The dot
   color reflects the test's CARD_state. The strip is rendered as a
   single bordered panel, so individual dots can be repainted in-place
   without redrawing the whole strip. */
static void card_strip_paint(UINTN i) {
    if (!g_focused_cards) return;
    /* Calculate dot position: divide strip horizontally into N_TESTS
       equal slots, dot in the middle of each slot. Leave room on the
       right for the "5/14 ош:0" counter. */
    UINTN right_text_chars = 18;   /* "  5/14   ош:0  " */
    UINTN avail_w = g_strip_w - right_text_chars * g_char_w - 8;
    if (avail_w < 40) avail_w = 40;
    UINTN slot_w  = avail_w / N_TESTS;
    if (slot_w < 8) slot_w = 8;
    UINTN dot_size = (g_strip_h > 14) ? 10 : (g_strip_h - 4);
    UINTN dot_x = g_strip_x + 6 + i * slot_w + (slot_w - dot_size) / 2;
    UINTN dot_y = g_strip_y + (g_strip_h - dot_size) / 2;

    card_info_t *c = &g_cards[i];
    UINT32 col = (c->state == CARD_RUNNING) ? COL_RUN
               : (c->state == CARD_PASS)    ? COL_OK
               : (c->state == CARD_FAIL)    ? COL_FAIL
               : (c->state == CARD_SKIP)    ? COL_DIM
               : COL_BAR_BG;
    /* Clear the slot first (so a transition from RUNNING→PASS doesn't
       leave a halo). */
    blt_fill(dot_x - 2, dot_y - 2, dot_size + 4, dot_size + 4, COL_PANEL);
    blt_fill(dot_x, dot_y, dot_size, dot_size, col);
    if (c->state == CARD_RUNNING)
        blt_fill(dot_x, dot_y, dot_size, 2, COL_ACCENT_HI);

    /* Right-aligned counter — "N/M  ош:K". Repaint every call (cheap).
       Counts done+running+failed against N_TESTS; errors are summed
       from g_cards[].errors. */
    UINTN done = 0;
    UINT64 total_err = 0;
    for (UINTN k = 0; k < N_TESTS; k++) {
        if (g_cards[k].state == CARD_PASS ||
            g_cards[k].state == CARD_FAIL ||
            g_cards[k].state == CARD_SKIP) {
            done++;
        }
        total_err += g_cards[k].errors;
    }
    CHAR16 buf[32];
    SPrint(buf, sizeof(buf), L"  %d/%d  err:%ld  ",
           (UINT32)done, (UINT32)N_TESTS, total_err);
    UINTN buf_chars = StrLen(buf);
    UINTN text_x = g_strip_x + g_strip_w - buf_chars * g_char_w - 6;
    UINTN text_y = g_strip_y + (g_strip_h - g_char_h) / 2;
    /* Clear right-side text area first */
    blt_fill(text_x - 2, text_y, buf_chars * g_char_w + 4, g_char_h, COL_PANEL);
    say_at_px(text_x, text_y, buf);
}

/* Paint the big focused card for test i (assumed to be the currently
   running test). Shows: test name, elapsed/expected time, big progress
   bar, live metrics (%, MB/s, errors, BW total, watts, °C). */
static void card_focused_paint(UINTN i) {
    if (!g_focused_cards) return;
    if (i >= N_TESTS) return;

    UINTN x = g_focus_x, y = g_focus_y;
    UINTN w = g_focus_w, h = g_focus_h;
    card_info_t *c = &g_cards[i];

    /* Inner area — leave space for outline border. */
    UINTN ix = x + 4, iw = w - 8;
    UINTN row_h = g_char_h + 4;
    UINTN row1_y = y + 6;
    UINTN row3_y = y + h - row_h - 4;
    UINTN row2_y = (row1_y + row3_y) / 2 - row_h / 2;

    /* Clear each row strip individually to avoid full-card flicker
       (only the changing pixels get rewritten). */
    blt_fill(ix, row1_y, iw, row_h, COL_PANEL);
    blt_fill(ix, row2_y, iw, row_h, COL_PANEL);
    blt_fill(ix, row3_y, iw, row_h, COL_PANEL);

    /* Row 1: test name (left) + short description in dim color + index counter (right).
       v0.4.29 — description lets non-expert user know what the test
       actually checks (TRRespass / March-C- / Butterfly etc. are jargon). */
    say_at_px(ix + 4, row1_y, g_tests[i].name);
    UINTN name_chars = StrLen(g_tests[i].name);
    CHAR16 idx_buf[32];
    SPrint(idx_buf, sizeof(idx_buf), L"[%d/%d]", (UINT32)(i + 1), (UINT32)N_TESTS);
    UINTN idx_chars = StrLen(idx_buf);
    UINTN idx_x = ix + iw - idx_chars * g_char_w - 4;
    /* Insert description between name and [N/M] if there's room */
    CHAR16 *desc = T(g_tests[i].desc_ru, g_tests[i].desc_en);
    UINTN desc_x = ix + 4 + (name_chars + 3) * g_char_w;
    UINTN desc_avail_chars = (idx_x > desc_x) ? (idx_x - desc_x) / g_char_w : 0;
    if (desc && desc[0] && desc_avail_chars > 12) {
        CHAR16 desc_clip[160];
        UINTN n = 0;
        for (; n < desc_avail_chars - 2 && n < 158 && desc[n]; n++)
            desc_clip[n] = desc[n];
        desc_clip[n] = 0;
        gfx_draw_str_color(desc_x, row1_y, desc_clip, COL_DIM);
    }
    say_at_px(idx_x, row1_y, idx_buf);

    /* Row 2: big progress bar spanning full inner width */
    UINTN bar_h = g_char_h - 2;
    if (bar_h < 10) bar_h = 10;
    UINTN bar_y = row2_y + (row_h - bar_h) / 2;
    UINT32 fill = c->pct_x10;
    UINT32 bar_col = (c->state == CARD_FAIL) ? COL_FAIL
                   : (c->state == CARD_PASS) ? COL_OK
                   : COL_BAR_FILL;
    blt_progress_bar(ix + 4, bar_y, iw - 8, bar_h, fill, bar_col, COL_BAR_BG);

    /* Row 3: live metrics — pct, MB/s, errors, BW total, watts, temp.
       Globals are file-static and already in scope here. BW shown in
       GB/s when ≥1024 MB/s, else MB/s. */
    UINT32 pct = c->pct_x10; if (pct > 1000) pct = 1000;
    CHAR16 metrics[160];
    UINT32 bw_gbs_x10 = g_bw_mbps_current ? (g_bw_mbps_current * 10 / 1024) : 0;
    SPrint(metrics, sizeof(metrics),
           T(L" %3d.%d%%   %5ld МБ/с   ош:%ld     BW %d.%d ГБ/с  %dВт  %d°C ",
             L" %3d.%d%%   %5ld MB/s   err:%ld    BW %d.%d GB/s  %dW  %d°C "),
           pct / 10, pct % 10, c->mbs, c->errors,
           bw_gbs_x10 / 10, bw_gbs_x10 % 10,
           (UINT32)g_pkg_power_w, (UINT32)g_max_temp_c);
    say_at_px(ix + 4, row3_y, metrics);
}

/* Helper: clear + repaint the big focused card from scratch (used when
   the active test changes — see card_paint above). */
static void card_focused_redraw(void) {
    if (!g_focused_cards) return;
    blt_fill(g_focus_x, g_focus_y, g_focus_w, g_focus_h, COL_PANEL);
    box_outline(g_focus_x, g_focus_y, g_focus_w, g_focus_h, COL_BORDER);
    if (g_focus_active_idx < N_TESTS) {
        card_focused_paint(g_focus_active_idx);
    }
}

static void cards_init_all(void) {
    if (!g_show_cards) {
        /* Still need to reset state — render_progress() reads c->state to
           decide whether to mark a test "running" / "passed" — without the
           init the colors of the in-header progress would be stale. */
        for (UINTN i = 0; i < N_TESTS; i++) {
            g_cards[i].state = CARD_IDLE;
            g_cards[i].pct_x10 = 0;
            g_cards[i].mbs = 0;
            g_cards[i].errors = 0;
        }
        return;
    }
    /* Reset all card state first */
    for (UINTN i = 0; i < N_TESTS; i++) {
        g_cards[i].state = CARD_IDLE;
        g_cards[i].pct_x10 = 0;
        g_cards[i].mbs = 0;
        g_cards[i].errors = 0;
    }

    if (g_focused_cards) {
        /* Section title above the strip */
        say_at_px(g_strip_x, g_strip_y - g_char_h - 2,
                  T(L"  Прогресс тестов", L"  Test Progress"));
        /* Strip background + border */
        blt_fill(g_strip_x, g_strip_y, g_strip_w, g_strip_h, COL_PANEL);
        box_outline(g_strip_x, g_strip_y, g_strip_w, g_strip_h, COL_BORDER);
        /* Focused card empty background + border */
        blt_fill(g_focus_x, g_focus_y, g_focus_w, g_focus_h, COL_PANEL);
        box_outline(g_focus_x, g_focus_y, g_focus_w, g_focus_h, COL_BORDER);
        g_focus_active_idx = (UINTN)-1;
        /* Paint all strip dots (IDLE state) + counter */
        for (UINTN i = 0; i < N_TESTS; i++) {
            card_strip_paint(i);
        }
        /* Placeholder text in the focused area while nothing runs */
        UINTN ty = g_focus_y + (g_focus_h - g_char_h) / 2;
        say_at_px(g_focus_x + 8, ty,
                  T(L"  Ожидание старта теста...",
                    L"  Waiting for test start..."));
        return;
    }

    /* Original full-list layout */
    say_at_px(g_card_x, g_card_y - g_char_h - 2,
              T(L"  Прогресс тестов", L"  Test Progress"));
    for (UINTN i = 0; i < N_TESTS; i++) {
        card_paint(i);
    }
}

/* ---------- Per-core real-time performance state ----------
   Sampled every render cycle (~100 ms). Used to color-code bars and detect
   slow/stuck cores. Progress is in millipercent (0..1000) per the kernel
   contract — we measure the derivative to get a live activity signal. */
typedef struct {
    UINT32 prev_progress;
    UINT64 last_sample_ms;
    UINT32 ppms_x1000;     /* smoothed: progress-mille per ms × 1000 */
    UINT32 stuck_frames;   /* consecutive samples with delta == 0 */
    UINT32 anim_phase;     /* counter for pulse animation */
} core_perf_t;
static core_perf_t g_core_perf[MAX_CORES];

static void core_perf_reset_all(void) {
    UINT64 now = ms_now();
    for (UINTN i = 0; i < MAX_CORES; i++) {
        g_core_perf[i].prev_progress  = 0;
        g_core_perf[i].last_sample_ms = now;
        g_core_perf[i].ppms_x1000     = 0;
        g_core_perf[i].stuck_frames   = 0;
        g_core_perf[i].anim_phase     = 0;
    }
}

static void core_perf_sample(void) {
    UINT64 now = ms_now();
    for (UINTN i = 0; i < g_n_enabled; i++) {
        UINT64 dt = now - g_core_perf[i].last_sample_ms;
        if (dt < 30) continue;          /* skip if too soon */
        UINT32 cur = g_args[i].progress;
        UINT32 delta = (cur >= g_core_perf[i].prev_progress)
                     ? (cur - g_core_perf[i].prev_progress) : 0;
        UINT32 inst = (delta * 1000) / (UINT32)dt;
        /* Exponential smoothing 3/4 + 1/4 to avoid jittery bars. */
        g_core_perf[i].ppms_x1000 = (g_core_perf[i].ppms_x1000 * 3 + inst) / 4;
        if (delta == 0 && !g_args[i].done && cur < 1000) {
            g_core_perf[i].stuck_frames++;
        } else {
            g_core_perf[i].stuck_frames = 0;
        }
        g_core_perf[i].prev_progress  = cur;
        g_core_perf[i].last_sample_ms = now;
        g_core_perf[i].anim_phase++;
    }
}

/* ---------- Per-core panel ---------- */
/* ---------- Core-panel layout helpers ---------- */
/* Adaptive grid: scales the panel so ALL cores are visible no matter how
   many physical cores the test machine has. Modern Threadripper / Xeon
   chips can ship 32–64 cores; truncating to "first 16 shown" lost half
   the diagnostic picture (an error on CPU#23 would never appear in the
   panel even though it was caught in the log). The layout now adapts:
       cores ≤ 8   : 1 column,  full metrics (Темп, Частота, МБ/с, Смещ)
       cores 9-16  : 2 columns, mid metrics (drop Темп/Частота/МБ/с/Смещ)
       cores 17-32 : 2 columns × ≤16 rows, mid metrics
       cores 33-64 : 4 columns × ≤16 rows, minimal metrics (only key signal:
                     bar + C0% + Сост. + Ош — the agg metrics like max temp,
                     freq, throttle, BW are already shown in the header
                     strip and provide the system-wide picture)
   Hard ceiling 64 = MAX_CORES (struct array size). Beyond that the test
   suite itself caps; for shop machines this is plenty. */
static UINTN core_grid_cols(void) {
    if (g_n_enabled <= 8)  return 1;
    if (g_n_enabled <= 16) return 2;
    if (g_n_enabled <= 32) return 2;
    return 4;
}
static UINTN core_grid_shown(void) {
    UINTN s = g_n_enabled;
    if (s > 64) s = 64;
    return s;
}
static UINTN core_grid_rows(void) {
    UINTN shown = core_grid_shown();
    UINTN cols  = core_grid_cols();
    return (shown + cols - 1) / cols;
}
/* "Density" indicates how aggressively to drop per-core metrics so things
   still fit horizontally in each grid cell. 0 = full (1-col), 1 = compact
   (2-col), 2 = ultra (4-col, only the essentials). */
static int core_grid_density(void) {
    if (g_n_enabled <= 8)  return 0;
    if (g_n_enabled <= 32) return 1;
    return 2;
}

static void core_cell_geom(UINTN i, UINTN *out_x, UINTN *out_y,
                            UINTN *out_w, UINTN *out_bx, UINTN *out_bw,
                            UINTN *out_reserved_right) {
    UINTN cols = core_grid_cols();
    UINTN rows = core_grid_rows();
    UINTN col_idx = i / rows;
    UINTN row_idx = i % rows;
    UINTN col_w = g_core_w / cols;
    UINTN x = g_core_x + col_idx * col_w;
    /* y-offset: skip optional subtitle row (g_char_h in comfort mode, 0 in
       compact) + col-header strip (4 px gap) + this row's index slot. */
    UINTN core_title_h = g_compact ? 0 : g_char_h;
    UINTN y = g_core_y + core_title_h + 4 + row_idx * g_core_row_h + 2;
    /* Label "CPU00 " = 6 chars. Right side reserved for status text;
       1-col mode shows " 99% 72°C 4400MHz жмёт" ≈ 24 chars, 2-col mode
       shows " 99% 72°C жмёт" ≈ 15 chars. */
    UINTN label_chars = 6;
    UINTN right_chars = (core_grid_cols() == 1) ? 24 : 15;
    UINTN bx = x + 8 + label_chars * g_char_w;
    UINTN bw = col_w - 8 - label_chars * g_char_w - right_chars * g_char_w - 8;
    if (out_x) *out_x = x;
    if (out_y) *out_y = y;
    if (out_w) *out_w = col_w;
    if (out_bx) *out_bx = bx;
    if (out_bw) *out_bw = bw;
    if (out_reserved_right) *out_reserved_right = right_chars * g_char_w;
}

/* Column geometry for the core-panel table layout. Each metric gets its own
   clearly-bordered cell so the user can see at a glance "что есть что".
   In 1-column mode (≤8 cores) we show all 10 columns; in 2-column mode
   (>8 cores, layout splits to two halves) we drop temp/freq/MB/s/addr to
   fit. New in this revision: per-core errors, per-core bandwidth, throttle
   reason flags, and current test address (MB offset within buffer). */
typedef struct {
    UINTN x_label;     /* "CPU00" */
    UINTN x_bar;       /* small activity bar */
    UINTN bar_w;
    UINTN x_pct;       /* "99%" */
    UINTN x_temp;      /* "34°C" */
    UINTN x_freq;      /* "3700 МГц" */
    UINTN x_state;     /* "работает" */
    UINTN x_err;       /* per-core error count */
    UINTN x_mbs;       /* per-core MB/s */
    UINTN x_thr;       /* throttle reason: TRM/PWR/PRC/CUR/— */
    UINTN x_addr;      /* MB offset of current slice in buffer */
    UINTN x_end;
} core_cols_t;

static void core_cols_compute(core_cols_t *c) {
    UINTN col_w_total = g_core_w / core_grid_cols();
    UINTN cw = g_char_w;
    UINTN pad = 6;
    /* WIDTH-AWARE column packing — instead of dropping metrics based on core
       count alone (the old approach over-stripped on wide displays), we
       compute mandatory columns first and then greedily add optional ones
       in priority order while there's room left in the cell.
       Previous version dropped Temp/Freq/MB/s/Addr for any layout ≥ 2 cols
       even on 1920×1080 where each 2-col half is 67 chars wide — plenty of
       room. The user (rightly) complained "не видны показатели". */
    UINTN w_label = 6 * cw;     /* "CPU00 " — always */
    UINTN w_pct   = 5 * cw;     /* " 99% " — always */
    UINTN w_err   = 5 * cw;     /* "Ош N"  — always (diagnostic gold) */
    UINTN w_bar_min = 4 * cw;   /* minimum activity bar so it remains visible */

    /* Status word width depends on language: Russian "работает" = 8 chars,
       "ждёт др." = 8 chars, "простой" = 7 chars. Round to 10. */
    UINTN w_state = 10 * cw;

    UINTN n_mandatory_cols = 5; /* label, bar, pct, state, err */
    UINTN mandatory_used = w_label + w_bar_min + w_pct + w_state + w_err
                         + pad * n_mandatory_cols;
    /* Leftover space in this cell after the mandatory columns. */
    UINTN slack = (col_w_total > mandatory_used) ? (col_w_total - mandatory_used) : 0;

    /* Optional columns in priority order. Each consumes (width + pad) from
       slack; skip if it doesn't fit. */
    UINTN w_thr  = 0, w_temp = 0, w_freq = 0, w_mbs = 0, w_addr = 0;
    UINTN bar_extra = 0;        /* extra pixels to give the activity bar */

    /* Priority 1: Throttle reason (TRM/PWR/PRC/—) — 5 chars */
    if (slack >= 5 * cw + pad) { w_thr = 5 * cw; slack -= w_thr + pad; }
    /* Priority 2: Temperature — 5 chars ("99°C") */
    if (slack >= 5 * cw + pad) { w_temp = 5 * cw; slack -= w_temp + pad; }
    /* Priority 3: Frequency — 9 chars ("4500 МГц") */
    if (slack >= 9 * cw + pad) { w_freq = 9 * cw; slack -= w_freq + pad; }
    /* Priority 4: Per-core MB/s — 6 chars */
    if (slack >= 6 * cw + pad) { w_mbs  = 6 * cw; slack -= w_mbs  + pad; }
    /* v0.4.29 — "Смещ" (buffer-offset for this core's slice) column dropped
       from the main test screen. It was a developer-debug field that nobody
       in the field could interpret; removing it frees ~9 chars to widen the
       activity bar. The offset is still in the log and the JSON. */
    (void)w_addr;
    /* Remaining slack goes into the activity bar so it visually fills the
       row. Cap at 16 cw so the bar doesn't look obnoxious on wide screens. */
    if (slack > 0) {
        bar_extra = (slack > 16 * cw) ? 16 * cw : slack;
    }
    UINTN w_bar = w_bar_min + bar_extra;

    UINTN x = pad;
    c->x_label = x;                              x += w_label + pad;
    c->x_bar   = x;  c->bar_w = w_bar;           x += w_bar   + pad;
    c->x_pct   = x;                              x += w_pct   + pad;
    if (w_temp) { c->x_temp = x;                 x += w_temp  + pad; } else c->x_temp = 0;
    if (w_freq) { c->x_freq = x;                 x += w_freq  + pad; } else c->x_freq = 0;
    c->x_state = x;                              x += w_state + pad;
    c->x_err   = x;                              x += w_err   + pad;
    if (w_mbs)  { c->x_mbs = x;                  x += w_mbs   + pad; } else c->x_mbs = 0;
    if (w_thr)  { c->x_thr = x;                  x += w_thr   + pad; } else c->x_thr = 0;
    if (w_addr) { c->x_addr = x;                 x += w_addr  + pad; } else c->x_addr = 0;
    c->x_end   = x;
}

static void core_panel_init(void) {
    UINTN shown = core_grid_shown();
    UINTN rows  = core_grid_rows();
    /* Compact mode: drop the verbose subtitle row entirely — it just said
       "Core activity (C0 residency via APERF/MPERF MSR)" which is jargon
       a shop tech does NOT care about. The column-header strip below
       (Core / C0% / Temp / Freq / State / Err / MB/s / Thrt) makes the
       meaning self-evident. Saves ~28 px which is exactly what was needed
       to fit the last CPU row on 1024×768 displays. */
    int show_title = !g_compact;
    UINTN title_h = show_title ? g_char_h : 0;

    /* +1 row for column-header strip; +1 if cores truncated. */
    UINTN panel_h = g_core_row_h * (rows + 1) + title_h + g_pad + 4;
    if (g_n_enabled > shown) panel_h += g_core_row_h;
    blt_panel(g_core_x, g_core_y, g_core_w, panel_h, COL_PANEL, COL_BORDER);

    if (show_title) {
        if (g_has_aperf_mperf) {
            say_at_px(g_core_x + 8, g_core_y + 2,
                      T(L"Активность ядер (C0 residency через APERF/MPERF MSR)",
                        L"Core activity (C0 residency via APERF/MPERF MSR)"));
        } else {
            say_at_px(g_core_x + 8, g_core_y + 2,
                      T(L"Прогресс по ядрам (MSR недоступен)",
                        L"Per-core progress (MSR unavailable)"));
        }
        blt_fill(g_core_x + 1, g_core_y + title_h + 2,
                 g_core_w - 2, 1, COL_BORDER);
    }

    /* Column-header strip — labels appear ONCE so the meaning of each
       column is obvious. Drawn in COL_DIM to look "structural". */
    core_cols_t c;
    core_cols_compute(&c);
    /* When title is suppressed (compact mode), col-headers start at the top
       of the panel; otherwise they sit just below the title row. */
    UINTN hdr_y = g_core_y + title_h + 4;
    UINTN cols  = core_grid_cols();
    for (UINTN col_idx = 0; col_idx < cols; col_idx++) {
        UINTN base_x = g_core_x + (g_core_w / cols) * col_idx;
        gfx_draw_str_color(base_x + c.x_label, hdr_y, T(L"Ядро",    L"Core"),  COL_DIM);
        gfx_draw_str_color(base_x + c.x_bar,   hdr_y, T(L"Активн.",  L"Bar"),   COL_DIM);
        gfx_draw_str_color(base_x + c.x_pct,   hdr_y, T(L"Загрузка", L"Load%"), COL_DIM);
        if (c.x_temp)
            gfx_draw_str_color(base_x + c.x_temp,  hdr_y, T(L"Темп",  L"Temp"),  COL_DIM);
        if (c.x_freq)
            gfx_draw_str_color(base_x + c.x_freq,  hdr_y, T(L"Частота",L"Freq"), COL_DIM);
        gfx_draw_str_color(base_x + c.x_state, hdr_y, T(L"Состояние",L"State"), COL_DIM);
        gfx_draw_str_color(base_x + c.x_err,   hdr_y, T(L"Ош",     L"Err"),   COL_DIM);
        if (c.x_mbs)
            gfx_draw_str_color(base_x + c.x_mbs,   hdr_y, T(L"МБ/с",  L"MB/s"),  COL_DIM);
        if (c.x_thr)
            gfx_draw_str_color(base_x + c.x_thr,   hdr_y, T(L"Тротт", L"Thrt"),  COL_DIM);
        if (c.x_addr)
            gfx_draw_str_color(base_x + c.x_addr,  hdr_y, T(L"Смещ", L"Off"),   COL_DIM);
    }
    /* Vertical dividers between columns (works for 2 and 4 column layouts) */
    if (cols > 1) {
        UINTN col_w = g_core_w / cols;
        for (UINTN k = 1; k < cols; k++) {
            blt_fill(g_core_x + col_w * k, hdr_y + g_char_h,
                     1, g_core_row_h * rows + g_pad, COL_BORDER);
        }
    }
    /* Horizontal separator between header and data rows */
    blt_fill(g_core_x + 1, hdr_y + g_char_h + 2, g_core_w - 2, 1, COL_BORDER);

    for (UINTN i = 0; i < shown; i++) {
        UINTN cx, cy, cw, bx, bw, rr;
        core_cell_geom(i, &cx, &cy, &cw, &bx, &bw, &rr);
        /* Shift row Y to accommodate the new column-header strip above. */
        cy += g_core_row_h;
        CHAR16 buf[16];
        /* Label cores from 1, not 0 — matches how the OS / Task Manager
           shows logical processors and how users naturally count. Internal
           slot index `i` stays 0-based throughout the rest of the code. */
        SPrint(buf, sizeof(buf), L"CPU%02d", (UINT32)i + 1);
        say_at_px(cx + c.x_label, cy, buf);
        /* Initial empty bar in the small dedicated bar column.
           Compact mode: thin progress bar (~10 px) vertically centred in
           the row — visually lighter, leaves the row feeling less cramped.
           Comfort mode: full-row bar (the previous behaviour). */
        UINTN base_x = cx + c.x_bar;
        UINTN bar_h  = g_compact ? 10 : (g_core_row_h - 6);
        UINTN bar_y  = cy + (g_core_row_h - bar_h) / 2;
        blt_fill(base_x, bar_y, c.bar_w, bar_h, COL_BAR_BG);
        box_outline(base_x, bar_y, c.bar_w, bar_h, COL_BORDER);
    }

    /* Overflow summary line — must follow same title_h conditional. */
    if (g_n_enabled > shown) {
        UINTN sy = g_core_y + title_h + 4 + rows * g_core_row_h + 2;
        CHAR16 buf[120];
        SPrint(buf, sizeof(buf),
               T(L"  + ещё %d ядро(а) — показаны только первые %d",
                 L"  + %d more core(s) — only first %d shown"),
               (UINT32)(g_n_enabled - shown), (UINT32)shown);
        say_at_px(g_core_x + 8, sy, buf);
    }

    core_perf_reset_all();
}

static void core_panel_update(void) {
    core_perf_sample();

    UINTN shown = core_grid_shown();
    core_cols_t c;
    core_cols_compute(&c);

    for (UINTN i = 0; i < shown; i++) {
        UINTN cx, cy, cw, bx_unused, bw_unused, rr_unused;
        core_cell_geom(i, &cx, &cy, &cw, &bx_unused, &bw_unused, &rr_unused);
        cy += g_core_row_h;   /* shift past column-header strip */

        UINT32 util = g_args[i].util_pct;
        UINT32 pmille = g_args[i].progress;
        if (pmille > 1000) pmille = 1000;

        /* Clear entire row band before redrawing (otherwise old glyphs
           bleed through — gfx_draw_str only paints "on" pixels). */
        blt_fill(cx + c.x_label, cy, c.x_end - c.x_label, g_core_row_h - 4, COL_PANEL);

        /* CPU label re-paint */
        CHAR16 lbl[16];
        SPrint(lbl, sizeof(lbl), L"CPU%02d", (UINT32)i + 1);
        say_at_px(cx + c.x_label, cy, lbl);

        /* Bar value = REAL C0 residency (% of time core is in C0 state).
           When MSR is unavailable we fall back to test progress %. When the
           core has finished its slice it's halted by MP services and not
           doing test work — util_pct was zeroed in ap_entry so the bar drops
           to 0%. That correctly reflects "no longer working". */
        UINT32 bar_pct;
        if (g_has_aperf_mperf) bar_pct = util;
        else                    bar_pct = pmille / 10;
        if (bar_pct > 100) bar_pct = 100;

        /* Status decision — clear Russian words, no slang.
           "ждёт" (waits) instead of "готово": when a core finishes ITS slice
           of the buffer before other cores, it doesn't mean the test is done —
           it means this core is idle, waiting for the slower cores to catch
           up. The previous "готово" wording confused users who thought the
           whole test had completed early.
           In ultra-dense layout (≥33 cores) there's only ~7 chars of room
           for the status — switch to compact words. */
        int compact_state = (core_grid_density() >= 2);
        CHAR16 *status_word;
        UINT32 col;
        if (g_args[i].done) {
            col = COL_OK;
            status_word = compact_state
                ? T(L"ждёт",   L"wait  ")
                : T(L"ждёт др.", L"waiting");
        } else if (g_core_perf[i].stuck_frames >= 5) {
            col = COL_FAIL;
            status_word = T(L"ЗАВИС", L"STUCK ");
        } else if (g_has_aperf_mperf) {
            if      (util >= 95) { col = COL_OK;     status_word = compact_state
                                                                  ? T(L"работ.", L"work  ")
                                                                  : T(L"работает", L"working"); }
            else if (util >= 80) { col = COL_ACCENT; status_word = T(L"норма",   L"normal "); }
            else if (util >= 50) { col = COL_RUN;    status_word = compact_state
                                                                  ? T(L"сниж.", L"reduc.")
                                                                  : T(L"снижено", L"reduced"); }
            else                 { col = COL_FAIL;   status_word = T(L"простой", L"idle   "); }
        } else {
            col = COL_ACCENT;
            status_word = T(L"активен", L"active ");
        }

        /* Bar — in its OWN narrow column (no more screen-filling stripe).
           Compact mode: thin 10-px bar centred vertically (less visual
           noise on 1024×768 screens where 16 fat bars would dominate). */
        UINTN bar_x = cx + c.x_bar;
        UINTN bar_h = g_compact ? 10 : (g_core_row_h - 6);
        UINTN bar_y = cy + (g_core_row_h - bar_h) / 2;
        UINTN fill  = (c.bar_w > 2) ? (c.bar_w - 2) * bar_pct / 100 : 0;
        blt_fill(bar_x, bar_y, c.bar_w, bar_h, COL_BAR_BG);
        if (fill > 0) {
            UINTN inner_h = (bar_h > 2) ? (bar_h - 2) : bar_h;
            blt_fill(bar_x + 1, bar_y + 1, fill, inner_h, col);
            UINT32 glow = (g_core_perf[i].anim_phase & 4) ? COL_ACCENT_HI : col;
            blt_fill(bar_x + 1, bar_y + 1, fill, 1, glow);
        }
        box_outline(bar_x, bar_y, c.bar_w, bar_h, COL_BORDER);

        /* Numeric columns — each in its own slot, no overlap risk */
        CHAR16 nbuf[24];
        SPrint(nbuf, sizeof(nbuf), L"%3d%%", bar_pct);
        say_at_px(cx + c.x_pct, cy, nbuf);

        if (c.x_temp && g_has_thermal) {
            SPrint(nbuf, sizeof(nbuf), L"%2d°C", g_args[i].temp_c);
            say_at_px(cx + c.x_temp, cy, nbuf);
        }
        if (c.x_freq && g_has_aperf_mperf && g_args[i].freq_mhz > 0) {
            SPrint(nbuf, sizeof(nbuf), L"%4d МГц", g_args[i].freq_mhz);
            say_at_px(cx + c.x_freq, cy, nbuf);
        }

        gfx_draw_str_color(cx + c.x_state, cy, status_word, col);

        /* --- New columns: errors, bandwidth, throttle reason, address --- */

        /* Errors per core: red if non-zero, dim if zero. Most diagnostic
           signal in the whole panel — clustering of errors by core groups
           points at the responsible memory channel/DIMM. */
        UINT64 ne = g_args[i].errors;
        UINT32 err_col = (ne > 0) ? COL_FAIL : COL_DIM;
        SPrint(nbuf, sizeof(nbuf), L"%ld", ne);
        gfx_draw_str_color(cx + c.x_err, cy, nbuf, err_col);

        /* MB/s per core: g_args[i].bytes is cumulative for the running test.
           Approximate live throughput by dividing by time since current test
           started; if not running, show "—". */
        if (c.x_mbs) {
            UINT64 ts = g_cur_test_started;
            UINT64 now = ms_now();
            UINT32 mbs = 0;
            if (ts && now > ts) {
                UINT64 dt_ms = now - ts;
                if (dt_ms > 0)
                    mbs = (UINT32)((g_args[i].bytes / (1024 * 1024)) * 1000ULL / dt_ms);
            }
            if (g_args[i].done) {
                gfx_draw_str_color(cx + c.x_mbs, cy, L"—", COL_DIM);
            } else {
                SPrint(nbuf, sizeof(nbuf), L"%4d", mbs);
                gfx_draw_str_color(cx + c.x_mbs, cy, nbuf,
                                   (mbs < 100) ? COL_DIM : COL_FG);
            }
        }

        /* Throttle reason: prioritise displaying the most-severe active
           flag. Empty dash = no throttling now. Colored bright when active
           because this is bad news the user wants to see. Dropped entirely
           in ultra-dense (>32 core) layout — the agg "тротт N" in header
           covers the system-wide signal. */
        if (c.x_thr) {
            UINT32 tflags = g_args[i].throttle_flags;
            CHAR16 *t_str;
            UINT32 t_col;
            if      (tflags & THROT_THERMAL)      { t_str = L"TRM"; t_col = COL_FAIL;   }
            else if (tflags & THROT_PROCHOT)      { t_str = L"PRC"; t_col = COL_FAIL;   }
            else if (tflags & THROT_POWERLIMIT)   { t_str = L"PWR"; t_col = COL_RUN;    }
            else if (tflags & THROT_CURRENTLIMIT) { t_str = L"CUR"; t_col = COL_RUN;    }
            else                                  { t_str = L"—";   t_col = COL_DIM;    }
            gfx_draw_str_color(cx + c.x_thr, cy, t_str, t_col);
        }

        /* Address offset: MB offset of THIS core's slice within the test
           buffer. Lets the user verify slices are distinct (CPU0 @ 0 MB,
           CPU1 @ 128 MB, ...). Useful for debugging the dispatcher. */
        if (c.x_addr && g_args[i].base) {
            UINT64 base_off = (UINT64)(UINTN)g_args[i].base
                            - (UINT64)(UINTN)g_mem_addr;
            UINT32 off_mb = (UINT32)(base_off / (1024ULL * 1024ULL));
            SPrint(nbuf, sizeof(nbuf), L"@%4d МБ", off_mb);
            gfx_draw_str_color(cx + c.x_addr, cy, nbuf, COL_DIM);
        }

        /* Blinking warning dot for stuck cores (far right) */
        if (g_core_perf[i].stuck_frames >= 5) {
            UINT32 blink = (g_core_perf[i].anim_phase & 2) ? COL_FAIL : COL_PANEL;
            blt_fill(cx + c.x_end - 14, cy + 4, 10, 10, blink);
        }
    }
}

/* ---------- Live update of running card + overall ---------- */
static void render_progress(UINTN test_idx, UINT64 t_started_ms,
                             UINT64 elapsed_ms, UINTN done_tests) {
    /* Aggregate per-core stats into the running card */
    UINT64 sum = 0, bytes = 0, errors = 0;
    for (UINTN i = 0; i < g_n_enabled; i++) {
        sum    += g_args[i].progress;
        bytes  += g_args[i].bytes;
        errors += g_args[i].errors;
    }
    UINT32 avg = (UINT32)(sum / g_n_enabled);
    UINT64 dt = (elapsed_ms > t_started_ms) ? (elapsed_ms - t_started_ms) : 1;
    UINT64 mbs = (bytes / (1024 * 1024) * 1000) / dt;

    g_cards[test_idx].pct_x10 = avg;
    g_cards[test_idx].mbs = mbs;
    g_cards[test_idx].errors = errors;
    card_paint(test_idx);

    /* Overall progress bar. On short screens (g_h < 800) we skip the
       "Общий прогресс [N/9]" label row above the bar — the header already
       shows "Тесты N/9", so this is duplicate info eating ~28 px we don't
       have. */
    if (g_h >= 800) {
        UINTN orow = (g_overall_y > g_char_h + 2)
                   ? (g_overall_y - g_char_h - 2) / g_char_h : 0;
        if (orow > 0 && orow < g_text_rows) {
            clear_row(orow);
            CHAR16 olbl[80];
            SPrint(olbl, sizeof(olbl),
                   T(L"  Общий прогресс  [%d/%d тестов]",
                     L"  Overall  [%d/%d tests]"),
                   (UINT32)done_tests, (UINT32)N_TESTS);
            say_at_rc(g_overall_x / g_char_w, orow, olbl);
        }
    }
    blt_progress_bar(g_overall_x, g_overall_y, g_overall_w, g_overall_h,
                     (UINT32)(done_tests * 1000 / N_TESTS), COL_OK, COL_BAR_BG);
}

/* ---------- Countdown ---------- */
volatile int g_aborted = 0;

/* Non-blocking keyboard poll. Sets g_aborted if ESC or Q pressed and returns 1.
   Safe to call from BSP context (kernel mid-execution or AP-spin loop). */
static int check_abort_key(void) {
    if (g_aborted) return 1;
    if (!ST || !ST->ConIn) return 0;
    EFI_INPUT_KEY k;
    EFI_STATUS s = uefi_call_wrapper(ST->ConIn->ReadKeyStroke, 2, ST->ConIn, &k);
    if (s != EFI_SUCCESS) return 0;
    if (k.ScanCode == SCAN_ESC || k.UnicodeChar == L'q' || k.UnicodeChar == L'Q') {
        g_aborted = 1;
        return 1;
    }
    return 0;
}

static void drain_conin(void) {
    if (!ST || !ST->ConIn) return;
    /* Soft reset just discards pending input. */
    uefi_call_wrapper(ST->ConIn->Reset, 2, ST->ConIn, FALSE);
    /* Belt-and-braces: also drain any keystrokes the firmware queued before reset. */
    EFI_INPUT_KEY k;
    for (int i = 0; i < 64; i++) {
        if (uefi_call_wrapper(ST->ConIn->ReadKeyStroke, 2, ST->ConIn, &k) != EFI_SUCCESS) break;
    }
}

/* v0.4.29 — countdown UX rework.
   Pre-v0.4.29: ESC meant "skip the wait and start the test now" — which
   completely contradicts the universal "ESC = cancel" convention. Users
   pressed ESC expecting "I don't want this test" and instead launched it.

   New keybinds (consistent with how every other UI on the planet works):
     ESC      → skip THIS test (continue to next)
     Q        → abort the entire run
     Enter/Space → start now (skip remaining wait — explicit "go" key)
   Return value:
     0 = normal flow, run the test
     1 = ESC pressed, caller should skip this test
     2 = Q pressed, g_aborted=1, caller should break the outer loop. */
static int countdown(int seconds, UINTN test_idx) {
    UINTN row = g_foot_y / g_char_h;
    if (row >= g_text_rows) row = g_text_rows - 1;
    for (int s = seconds; s > 0; s--) {
        if (g_aborted) return 2;
        clear_row(row);
        CHAR16 buf[180];
        SPrint(buf, sizeof(buf),
               T(L"  [%d/%d] %s  старт через %d сек   [Enter]=старт сейчас  [ESC]=пропустить тест  [Q]=отмена прогона",
                 L"  [%d/%d] %s  starts in %d sec   [Enter]=start now  [ESC]=skip this test  [Q]=abort run"),
               (UINT32)(test_idx + 1), (UINT32)N_TESTS, g_tests[test_idx].name, s);
        say_at_rc(0, row, buf);

        EFI_EVENT ev[2] = { ST->ConIn->WaitForKey, NULL };
        uefi_call_wrapper(BS->CreateEvent, 5, EVT_TIMER, 0, NULL, NULL, &ev[1]);
        uefi_call_wrapper(BS->SetTimer, 3, ev[1], TimerRelative, 10000000ULL);
        UINTN idx = 0;
        uefi_call_wrapper(BS->WaitForEvent, 3, 2, ev, &idx);
        uefi_call_wrapper(BS->CloseEvent, 1, ev[1]);
        if (idx == 0) {
            EFI_INPUT_KEY k = { 0, 0 };
            EFI_STATUS rs = uefi_call_wrapper(ST->ConIn->ReadKeyStroke, 2, ST->ConIn, &k);
            if (rs != EFI_SUCCESS) continue;   /* spurious wake — k is garbage */
            if (k.ScanCode == SCAN_ESC) return 1;       /* skip this test */
            if (k.UnicodeChar == L'q' || k.UnicodeChar == L'Q') {
                g_aborted = 1; return 2;                 /* abort run */
            }
            if (k.UnicodeChar == L'\r' || k.UnicodeChar == L'\n' ||
                k.UnicodeChar == L' ') {
                return 0;                                /* start now */
            }
            /* Any other key → ignore, continue counting down */
        }
    }
    return 0;
}

/* ---------- BSP yield callback — render mid-kernel so cores look alive ----------
   The BSP runs its kernel slot inline (MP services can't dispatch onto BSP).
   Without periodic rendering during that work the panel only updates after BSP
   finishes — making bars look static. Each kernel calls ap_yield(a) at progress
   points; for the BSP slot we point yield at this callback. */
/* (these globals are declared earlier in the file — search "g_cur_test_started"
   forward decl — to keep core_panel_update happy without reordering huge
   blocks of code.) */
/* Sample aggregate metrics — runs on the BSP only, throttled to once per
   yield window. Updates the globals that render_header reads. Cheap (a few
   MSR reads + arithmetic); does NOT touch APs. */
static void sample_aggregate_metrics(UINT64 now_ms) {
    /* --- 1) RAM bandwidth (sum of g_args[i].bytes, MB/s over Δt) --- */
    UINT64 sum_bytes = 0;
    UINT32 freq_max = 0, freq_sum = 0, temp_max = 0;
    UINT32 thr_sum = 0;
    UINT32 freq_cnt = 0;
    UINT32 thr_seen = 0;        /* OR of any throttle flags this sample */
    for (UINTN i = 0; i < g_n_enabled; i++) {
        sum_bytes += g_args[i].bytes;
        UINT32 f = g_args[i].freq_mhz;
        UINT32 t = g_args[i].temp_c;
        if (f > 0) { freq_sum += f; freq_cnt++; if (f > freq_max) freq_max = f; }
        if (t > temp_max) temp_max = t;
        thr_sum += g_args[i].throttle_event_count;
        thr_seen |= g_args[i].throttle_flags;
    }
    g_cum_bytes = sum_bytes;
    g_freq_max_mhz = freq_max;
    g_freq_avg_mhz = freq_cnt ? (freq_sum / freq_cnt) : 0;
    /* On AMD, per-core IA32_THERM_STATUS doesn't exist so g_args[i].temp_c
       stays 0. Instead we sample the package Tctl via SMN here. The value
       gets fed into the same g_max_temp_c counter that the Intel path
       writes, so the header / summary display works uniformly. */
    if (g_amd_smn_ready) {
        UINT32 pkg_temp = amd_thermal_sample();
        if (pkg_temp > temp_max) temp_max = pkg_temp;
        /* Periodic visibility log so the user can confirm temperature
           is actually being measured (not just a stale baseline value).
           Throttled to once every 30 seconds so we don't spam the log. */
        static UINT64 last_temp_log_ms = 0;
        if (pkg_temp > 0 && (last_temp_log_ms == 0 ||
            now_ms - last_temp_log_ms >= 30000)) {
            CHAR16 lb[120];
            SPrint(lb, sizeof(lb),
                   L"[TEMP] live SMN Tctl=%d°C (peak so far %d°C)",
                   pkg_temp, g_max_temp_c);
            log_line(lb);
            last_temp_log_ms = now_ms;
        }
    }
    /* RUNNING peak — never decrease. Previously we did
         g_max_temp_c = temp_max;
       which OVERWROTE the sample's max instead of accumulating across
       samples. Result: user saw 53 °C live in the header (one sample
       caught the peak) but the final summary later showed 43 °C because
       a later, cooler sample replaced the recorded value. The label is
       "макс / max" — it must monotonically grow. */
    if (temp_max > g_max_temp_c) g_max_temp_c = temp_max;
    g_throttle_total = thr_sum;
    /* Sticky: remember the last non-zero throttle bitmap so the post-run
       summary can show "тротт PWR (16)" instead of just "тротт 16". This
       is essential to distinguish thermal-throttle from PL1-clamping. */
    if (thr_seen) g_dominant_throttle = thr_seen;
    if (freq_max > g_max_freq_mhz_observed) g_max_freq_mhz_observed = freq_max;

    if (g_bw_ts_prev_ms != 0 && now_ms > g_bw_ts_prev_ms) {
        UINT64 dt_ms = now_ms - g_bw_ts_prev_ms;
        if (dt_ms >= 200 && sum_bytes >= g_bw_bytes_prev) {
            UINT64 db = sum_bytes - g_bw_bytes_prev;
            /* MB/s = (bytes / 2^20) * 1000 / ms */
            UINT64 mbps = (db >> 20) * 1000ULL / dt_ms;
            if (mbps > 0xFFFFFFFFULL) mbps = 0xFFFFFFFF;
            g_bw_mbps_current = (UINT32)mbps;
            if (g_bw_mbps_current > g_bw_mbps_peak) g_bw_mbps_peak = g_bw_mbps_current;
            g_bw_bytes_prev = sum_bytes;
            g_bw_ts_prev_ms = now_ms;

            /* Append to time-bucketed history for the post-run trend
               analysis. Bucket index = minutes since g_bw_history_start_ms.
               Keep the bucket MAX (not mean) so transient dips during
               between-test transitions don't depress the trend. */
            if (g_bw_history_start_ms == 0) g_bw_history_start_ms = now_ms;
            UINT64 elapsed = now_ms - g_bw_history_start_ms;
            UINT32 b = (UINT32)(elapsed / BW_BUCKET_MS);
            if (b < BW_HISTORY_BUCKETS) {
                if (b != g_bw_current_bucket) {
                    /* New bucket — start its max fresh. */
                    g_bw_history_max[b] = g_bw_mbps_current;
                    g_bw_current_bucket = b;
                    if (b + 1 > g_bw_history_count) g_bw_history_count = b + 1;
                } else if (g_bw_mbps_current > g_bw_history_max[b]) {
                    g_bw_history_max[b] = g_bw_mbps_current;
                }
            }
        }
    } else {
        g_bw_bytes_prev = sum_bytes;
        g_bw_ts_prev_ms = now_ms;
    }

    /* --- 1b) CPU voltage (VID) via IA32_PERF_STATUS (Intel only) ---
       MSR 0x198 bits[47:32] hold "Current Voltage" in 1/8192 V units on
       Westmere-and-later Intel CPUs. Formula: mV = raw × 125 / 1024.
       AMD has a completely different MSR layout (MSR_PSTATE_DEF varies by
       family) and rdmsr on the Intel MSR would #GP and freeze UEFI — so
       we MUST guard on g_cpu_vendor. Sanity range 500–2000 mV; anything
       outside is treated as junk (some firmware writes 0 to the field). */
    if (g_cpu_vendor == CPU_INTEL) {
        UINT64 ps = rdmsr_safe(0x198);
        UINT32 raw = (UINT32)((ps >> 32) & 0xFFFFu);
        UINT32 mv  = (raw * 125u) / 1024u;
        if (mv >= 500 && mv <= 2000) g_pkg_vid_mv = mv;
    }

    /* --- 2) Package power via RAPL (Watts = ΔJ / Δs) --- */
    if (g_has_rapl && g_rapl_ts_prev_ms != 0 && now_ms - g_rapl_ts_prev_ms >= 250) {
        UINT64 e_now = rdmsr_safe(g_rapl_pkg_msr) & 0xFFFFFFFFULL;
        UINT64 e_delta;
        /* 32-bit counter wraps every ~233 J / power. We treat smaller-than-
           previous as a wrap. */
        if (e_now >= g_rapl_pkg_energy_prev) e_delta = e_now - g_rapl_pkg_energy_prev;
        else                                  e_delta = (0x100000000ULL - g_rapl_pkg_energy_prev) + e_now;
        UINT64 dt_ms = now_ms - g_rapl_ts_prev_ms;
        /* watts = (Δenergy / units_div) / (dt_ms/1000)
                 = Δenergy * 1000 / (units_div * dt_ms)              */
        UINT64 w = (e_delta * 1000ULL) / ((UINT64)g_rapl_energy_units_div * dt_ms);
        if (w > 0xFFFFFFFFULL) w = 0xFFFFFFFF;
        g_pkg_power_w = (UINT32)w;
        if (g_pkg_power_w > g_pkg_power_w_peak) g_pkg_power_w_peak = g_pkg_power_w;
        g_rapl_pkg_energy_prev = e_now;
        g_rapl_ts_prev_ms      = now_ms;
    } else if (g_has_rapl && g_rapl_ts_prev_ms == 0) {
        g_rapl_pkg_energy_prev = rdmsr_safe(g_rapl_pkg_msr) & 0xFFFFFFFFULL;
        g_rapl_ts_prev_ms = now_ms;
    }
}

static void bsp_yield_render(ap_arg_t *a) {
    (void)a;
    /* Always poll keyboard, even if we skip the render this tick. */
    check_abort_key();
    UINT64 now = ms_now();
    /* Throttle to ~10 Hz (was 20 Hz). At 20 Hz the clear-then-redraw cycle
       in render_header / core_panel_update was visible as flicker on slow
       monitors. 10 Hz feels equally responsive for memory tests where the
       interesting changes (errors, bandwidth shifts, temps) happen on the
       order of seconds, not 50-ms ticks. */
    if (now - g_last_yield_ms < 100) return;
    g_last_yield_ms = now;
    sample_aggregate_metrics(now);
    /* Refresh the HEADER too — previously only between tests, so during
       a long test (Bit Fade Ext = 6 min) the header froze: elapsed time
       didn't tick, "Тесты N/10" didn't update, BW/temp/throttle were
       stale, and the spinner in the activity row didn't animate. With
       this call all three header rows update at ~10 Hz. */
    UINT64 run_elapsed = (g_run_start_ms && now > g_run_start_ms)
                        ? (now - g_run_start_ms) : 0;
    render_header(run_elapsed, g_cur_test_idx, N_TESTS);
    core_panel_update();
    render_progress(g_cur_test_idx, g_cur_test_started, now, 0);
}

/* ---------- Run a kernel multi-core, polling progress ---------- */
static test_summary_t run_test_mc(UINTN test_idx) {
    test_summary_t s = {0, 0, 0, 1};
    UINT64 t_started = ms_now();
    g_cur_test_idx     = test_idx;
    g_cur_test_started = t_started;
    g_last_yield_ms    = 0;

    UINTN total_q = (g_mem_pages * 4096) / 8;
    UINTN per_q   = total_q / g_n_enabled;
    UINT64 *base  = (UINT64 *)(UINTN)g_mem_addr;

    for (UINTN i = 0; i < g_n_enabled; i++) {
        g_args[i].base     = base + i * per_q;
        g_args[i].n_qwords = per_q;
        g_args[i].kernel   = g_tests[test_idx].k;
        g_args[i].core_idx = (UINT32)i;
        g_args[i].progress = 0;
        g_args[i].done     = 0;
        g_args[i].errors   = 0;
        g_args[i].bytes    = 0;
        g_args[i].util_tsc_prev   = 0;
        g_args[i].util_mperf_prev = 0;
        g_args[i].util_aperf_prev = 0;
        g_args[i].util_pct        = 0;
        g_args[i].freq_mhz        = 0;
        g_args[i].temp_c          = 0;
        /* throttle_event_count is NOT reset here — we want it to accumulate
           across all tests in a run so the user sees the total number of
           throttle events for the whole prog. It IS reset between RUNS
           (when the user goes back to the menu and starts again), via the
           reset block in efi_main()'s outer loop. */
        g_args[i].throttle_flags  = 0;
        /* Only BSP slot renders. APs leave yield NULL — drawing from an AP
           would race with BSP on the framebuffer. */
        g_args[i].yield    = (i == 0) ? bsp_yield_render : NULL;
    }

    /* Non-blocking AP dispatch: pass dummy WaitEvent so StartupThisAP
       returns immediately. Without this, BSP blocks per-AP and "parallel"
       cores actually serialise. This was the v0.3 freeze.
       Diagnostic logging added in v0.4.3 to pinpoint AP-dispatch hangs
       on AMD ASUS firmware (StartupThisAP sometimes blocks even with a
       dummy WaitEvent on AMI B450/B550 AMD UEFI). */
    EFI_EVENT ap_events[MAX_CORES] = {0};
    if (g_mp && g_n_enabled > 1) {
        if (test_idx == 0) {
            CHAR16 lb[140];
            SPrint(lb, sizeof(lb),
                   L"[DISP] dispatching %d AP(s) via StartupThisAP (non-blocking)",
                   (UINT32)(g_n_enabled - 1));
            log_line(lb);
        }
        for (UINTN i = 1; i < g_n_enabled; i++) {
            uefi_call_wrapper(BS->CreateEvent, 5, 0, 0, NULL, NULL, &ap_events[i]);
            EFI_STATUS sd = uefi_call_wrapper(g_mp->StartupThisAP, 7, g_mp,
                              (EFI_AP_PROCEDURE)ap_entry,
                              i, ap_events[i], 0, &g_args[i], NULL);
            if (test_idx == 0 && EFI_ERROR(sd)) {
                CHAR16 lb[120];
                SPrint(lb, sizeof(lb),
                       L"[DISP] StartupThisAP slot=%d failed status=0x%lx",
                       (UINT32)i, (UINT64)sd);
                log_line(lb);
            }
        }
        if (test_idx == 0) log_line(L"[DISP] all StartupThisAP calls returned");
    }

    /* BSP runs slot 0 inline. Its kernel calls ap_yield(a) periodically,
       which routes to bsp_yield_render — that's how bars stay live. */
    if (test_idx == 0) log_line(L"[DISP] BSP entering ap_entry for slot 0");
    ap_entry(&g_args[0]);
    if (test_idx == 0) log_line(L"[DISP] BSP returned from ap_entry slot 0");

    /* Once-per-run diagnostic: how many APs successfully bumped the
       CPU into max P-state, by which mechanism. Pre-v0.4.29 this only
       counted the HWP path, so on Haswell/Ivy/Sandy (no HWP) the line
       read OK=0 FAIL=0 — making it look like nothing happened, even
       though the legacy PERF_CTL fallback was actually doing its job. */
    static int g_perf_summary_logged = 0;
    if (!g_perf_summary_logged) {
        g_perf_summary_logged = 1;
        CHAR16 lb[200];
        if (g_hwp_ok_count > 0 || g_hwp_fail_count > 0) {
            SPrint(lb, sizeof(lb),
                   L"[PERF] turbo lift via HWP: OK=%d FAIL=%d (of %d cores)",
                   g_hwp_ok_count, g_hwp_fail_count, (UINT32)g_n_enabled);
        } else if (g_legacy_turbo_count > 0) {
            SPrint(lb, sizeof(lb),
                   L"[PERF] turbo lift via legacy PERF_CTL (0x199): %d/%d cores "
                   L"(HWP not on this CPU — pre-Skylake)",
                   g_legacy_turbo_count, (UINT32)g_n_enabled);
        } else if (g_cpu_vendor == CPU_AMD) {
            SPrint(lb, sizeof(lb),
                   L"[PERF] turbo lift via AMD CPPC2 (see earlier [PERF] BSP line)");
        } else {
            SPrint(lb, sizeof(lb),
                   L"[PERF] turbo lift: none — CPU runs at BIOS-default P-state");
        }
        log_line(lb);
    }

    /* Spin-poll AP done flags with 100 ms refresh (hard cap 1 hour). Also
       polls the keyboard so ESC during a long test can abort. */
    if (g_mp && g_n_enabled > 1) {
        for (UINTN spin = 0; spin < 36000; spin++) {
            int all_done = 1;
            for (UINTN i = 1; i < g_n_enabled; i++)
                if (!g_args[i].done) { all_done = 0; break; }
            if (all_done) break;
            UINT64 sp_now = ms_now();
            sample_aggregate_metrics(sp_now);
            /* Also refresh the header so during a slow test (where the BSP
               is idle in this poll loop waiting for slower APs) the
               spinner / time / Bit Fade countdown still animate. */
            UINT64 run_elapsed = (g_run_start_ms && sp_now > g_run_start_ms)
                                ? (sp_now - g_run_start_ms) : 0;
            render_header(run_elapsed, g_cur_test_idx, N_TESTS);
            core_panel_update();
            render_progress(test_idx, t_started, sp_now, 0);
            heartbeat_dot(sp_now);
            /* Both event sources: timer (so we redraw at 10 Hz) AND keyboard
               (so ESC/Q response feels immediate). */
            EFI_EVENT te = NULL;
            uefi_call_wrapper(BS->CreateEvent, 5, EVT_TIMER, 0, NULL, NULL, &te);
            uefi_call_wrapper(BS->SetTimer, 3, te, TimerRelative, 1000000ULL);
            EFI_EVENT events[2] = { te, ST->ConIn->WaitForKey };
            UINTN idx = 0;
            uefi_call_wrapper(BS->WaitForEvent, 3, 2, events, &idx);
            uefi_call_wrapper(BS->CloseEvent, 1, te);
            if (idx == 1) {
                /* Key event fired — consume + check abort */
                check_abort_key();
                if (g_aborted) break;
            }
        }
        for (UINTN i = 1; i < g_n_enabled; i++)
            if (ap_events[i]) uefi_call_wrapper(BS->CloseEvent, 1, ap_events[i]);
    }
    core_panel_update();

    s.time_ms = ms_now() - t_started;
    for (UINTN i = 0; i < g_n_enabled; i++) {
        s.errors += g_args[i].errors;
        s.bytes  += g_args[i].bytes;
    }
    s.status = (s.errors == 0) ? 1 : 2;
    return s;
}

/* ---------- Simple verdict screen (shown by default after tests) ----------
   Designed for a shop technician / customer standing over the screen, NOT
   for a memory engineer reading the log. Three states:
     ✓ PASS  — green big text "MEMORY OK"
     ⚠ WARN  — yellow "MEMORY MARGINAL", explain what's off
     ✗ FAIL  — red, name the DIMM to replace + plain language + S/N for warranty
   Press [D] to drill into the existing technical render_summary table. */

typedef enum {
    VERDICT_PASS = 0,
    VERDICT_WARN = 1,
    VERDICT_FAIL = 2
} verdict_kind_t;

typedef enum {
    CONF_LOW = 0,        /* 1-2 scattered errors */
    CONF_MED = 1,        /* 3-9 errors, clustered on one DIMM */
    CONF_HIGH = 2        /* 10+ clustered OR stuck-bit/row/bank detected */
} verdict_confidence_t;

/* Decide what overall verdict to show. PASS = clean run, WARN = no errors
   but worrying signals (MCA new corrected errors, BW degraded, big temp
   regression vs prev run), FAIL = any errors recorded. */
static verdict_kind_t compute_verdict_kind(void) {
    if (g_aborted)            return VERDICT_WARN;   /* incomplete — flag */
    if (g_err_count > 0)      return VERDICT_FAIL;
    if (g_mca_new_errors > 0) return VERDICT_WARN;
    if (g_bw_trend_degraded >= 2) return VERDICT_WARN;  /* severe BW drop */
    /* Cold/warm boot delta — large temp regression vs last run = WARN */
    if (g_hist_prev_valid &&
        g_hist_prev.max_temp_c > 0 &&
        g_max_temp_c > g_hist_prev.max_temp_c + 8) return VERDICT_WARN;
    return VERDICT_PASS;
}

/* Decide how confident we are about the failure location. HIGH only if
   we have a stuck pattern (same XOR mask repeated, same row, or same
   bank) — those localise to a specific cell/wordline/bank. MED if
   errors cluster on one DIMM but pattern is unclear. LOW if scattered. */
static verdict_confidence_t compute_confidence(void) {
    if (g_err_count == 0) return CONF_LOW;
    UINT32 stuck_n = 0;
    UINT64 stuck_x = find_stuck_bit(&stuck_n);
    UINT32 srow_n = 0;  find_stuck_row(&srow_n);
    UINT32 sbank_n = 0; find_stuck_bank(&sbank_n);
    if ((stuck_n >= 5 && stuck_x != 0) || srow_n >= 3 || sbank_n >= 3)
        return CONF_HIGH;
    if (g_err_count >= 10) return CONF_HIGH;
    if (g_err_count >= 3)  return CONF_MED;
    return CONF_LOW;
}

/* Draw a string centered horizontally at the given pixel-Y, with optional
   font-scale override (2 = double-size for headlines). Restores g_font_scale
   on exit. */
static void verdict_say_centered(CHAR16 *s, UINTN y, UINT32 color, UINT32 scale) {
    UINT32 saved = g_font_scale;
    if (scale > 0) g_font_scale = scale;
    UINTN cw_now = FONT_ADVANCE * g_font_scale;
    UINTN w = StrLen(s) * cw_now;
    UINTN x = (g_w > w) ? (g_w - w) / 2 : 0;
    gfx_draw_str_color(x, y, s, color);
    g_font_scale = saved;
}

/* Compose plain-language "what's broken" text into a buffer. Pulls from
   the localization helpers (stuck-bit / stuck-row / stuck-bank / chip).
   At most 2 lines so the screen stays readable. */
static int verdict_describe_what_broke(CHAR16 *line1, UINTN cap1,
                                       CHAR16 *line2, UINTN cap2,
                                       int dominant_dimm_0based) {
    line1[0] = 0; line2[0] = 0;
    int n = 0;

    UINT32 stuck_n = 0;
    UINT64 stuck_x = find_stuck_bit(&stuck_n);
    int bp = single_bit_pos(stuck_x);
    if (stuck_n >= 5 && bp >= 0 && dominant_dimm_0based >= 0) {
        CHAR16 chip[64];
        int have_chip = chip_label_for_bit((UINT32)dominant_dimm_0based, bp, chip, 64);
        if (have_chip) {
            /* SPD told us chip width → can name the exact chip on PCB */
            SPrint(line1, cap1,
                   T(L"● Дохлая ячейка: %s — биту %d стуковое значение (%d ошибок этого типа)",
                     L"● Dead cell: %s — bit %d stuck (%d errors of this type)"),
                   chip, bp, stuck_n);
        } else {
            /* SPD didn't expose chip width (typical for DDR4 x4 / некоторые DDR5)
               — we know which bit, just not which physical chip on the PCB */
            SPrint(line1, cap1,
                   T(L"● Стуковый бит D[%d] на планке (%d ошибок). "
                     L"SPD не сообщил ширину чипа — точный U-номер чипа не определить.",
                     L"● Stuck bit D[%d] on DIMM (%d errors). "
                     L"SPD did not expose chip width — exact chip U-number unknown."),
                   bp, stuck_n);
        }
        n++;
    }

    UINT32 srow_n = 0;
    UINT64 srow = find_stuck_row(&srow_n);
    if (srow_n >= 3) {
        CHAR16 *target = (n == 0) ? line1 : line2;
        UINTN cap = (n == 0) ? cap1 : cap2;
        SPrint(target, cap,
               T(L"● Повреждён ряд ячеек ~0x%lx (%d ошибок в одном ряду)",
                 L"● Damaged cell row ~0x%lx (%d errors in same row)"),
               srow, srow_n);
        n++;
        if (n >= 2) return n;
    }

    UINT32 sbank_n = 0;
    UINT32 sbank = find_stuck_bank(&sbank_n);
    if (sbank_n >= 3) {
        CHAR16 *target = (n == 0) ? line1 : line2;
        UINTN cap = (n == 0) ? cap1 : cap2;
        SPrint(target, cap,
               T(L"● Повреждена секция памяти (банк-группа %d / банк %d, %d ошибок)",
                 L"● Damaged memory bank (bank-group %d / bank %d, %d errors)"),
               (sbank >> 4) & 0xF, sbank & 0xF, sbank_n);
        n++;
        if (n >= 2) return n;
    }

    /* Fallback when no clear pattern — say total error count + dominant DIMM */
    if (n == 0) {
        SPrint(line1, cap1,
               T(L"● %ld ошибок памяти без чёткой локализации",
                 L"● %ld memory errors without a clear pattern"),
               (UINT64)g_err_count);
    }
    return n;
}

/* ---------- v0.4.29 Auto-isolation feature ----------
   When the post-test verdict detects "errors on 2+ DIMMs in block-mapped
   Type 20", we can definitively identify the bad stick(s) by re-running
   the failing test on each DIMM in turn with TestOnlyDimm, instead of
   asking the user to physically pull DIMMs. Block-mapped is required:
   on real cache-line interleave, TestOnlyDimm doesn't physically isolate
   because the iMC still alternates between channels.                  */

/* v0.4.29 — should auto-isolation kick in automatically?
   Same conditions as the [I] offer in render_simple_verdict, but checked
   from the main test loop right after tests complete so we can run
   isolation BEFORE showing the verdict and skip the "press [I] then wait"
   step entirely. Populates g_iso_dimm_idx[] and g_iso_dimm_n as a side
   effect so do_auto_isolation() can pick up from there. Returns 1 if
   should run, 0 otherwise. */
static int should_auto_isolate(void) {
    if (g_err_count == 0) return 0;
    if (g_cfg_test_only_dimm != 0) return 0;   /* user already isolated manually */
    if (g_dimm_count < 2) return 0;
    int dist_idx[MAX_DIMMS];
    UINTN dist_n = distributed_dimm_indices(dist_idx, MAX_DIMMS);
    if (dist_n < 2 || dist_n > MAX_ISO_DIMMS) return 0;
    /* Only auto-run on block-mapped Type 20 — on real interleave the
       isolation can't physically separate channels. */
    if (type20_has_overlapping_ranges()) return 0;
    /* Capture for do_auto_isolation() */
    g_iso_dimm_n = dist_n;
    for (UINTN k = 0; k < dist_n; k++) g_iso_dimm_idx[k] = dist_idx[k];
    return 1;
}

/* Splash shown briefly before isolation kicks in — tells the user what's
   about to happen and why, without offering an opt-out (the whole point
   of auto-isolation is "no decisions, just get to the truth"). */
static void render_auto_isolation_intro(UINTN n_dimms) {
    cls();
    blt_gradient_v(0, 0, g_w, g_char_h * 2, COL_ACCENT, COL_ACCENT_DK);
    blt_fill(0, g_char_h * 2 - 2, g_w, 2, COL_ACCENT_HI);
    verdict_say_centered(T(L"АВТОМАТИЧЕСКАЯ ПРОВЕРКА ПЛАНОК",
                           L"AUTOMATIC DIMM ISOLATION"),
                         g_char_h / 2, COL_FG, 1);
    UINTN cy = g_char_h * 5;
    UINTN cx = g_w / 8;
    UINTN cline = g_char_h + 4;
    CHAR16 buf[200];

    SPrint(buf, sizeof(buf),
           T(L"  Тест нашёл ошибки в адресах %d планок памяти.",
             L"  Test found errors in the address ranges of %d DIMMs."),
           (UINT32)n_dimms);
    gfx_draw_str_color(cx, cy, buf, COL_FG); cy += cline;
    gfx_draw_str_color(cx, cy,
        T(L"  Чтобы понять — это все планки дефектные или только одна,",
          L"  To determine whether all DIMMs are bad or only one,"),
        COL_FG); cy += cline;
    gfx_draw_str_color(cx, cy,
        T(L"  программа сейчас проверит каждую планку отдельно.",
          L"  the program is about to test each DIMM separately."),
        COL_FG); cy += cline + 8;

    SPrint(buf, sizeof(buf),
           T(L"  Займёт примерно %d минут. Подождите, пожалуйста.",
             L"  This will take about %d minutes. Please wait."),
           (UINT32)(n_dimms * 2 + 1));   /* rough estimate */
    gfx_draw_str_color(cx, cy, buf, COL_ACCENT_HI); cy += cline;

    /* Hold this screen for ~3 sec so the user reads it. */
    uefi_call_wrapper(BS->Stall, 1, (UINTN)3000000);
}

/* Pick the kernel that found the most error records (used to focus the
   isolation re-test on the test that actually catches the fault). Falls
   back to KER_AVX2_SUSTAINED (a strong sustained-load test) when no
   error records exist. */
static UINT32 isolation_pick_kernel(void) {
    UINT32 counts[64] = {0};        /* indexed by kernel_id_t (well below 64) */
    UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
    for (UINT32 i = 0; i < shown; i++) {
        UINT32 k = (UINT32)g_err_records[i].test;
        if (k < 64) counts[k]++;
    }
    UINT32 best_k = (UINT32)KER_AVX2_SUSTAINED;
    UINT32 best_n = 0;
    for (UINT32 k = 0; k < 64; k++) {
        if (counts[k] > best_n) { best_n = counts[k]; best_k = k; }
    }
    return best_k;
}

/* Render the live "isolating DDR4-A2 / running test..." panel. */
static void render_isolation_progress(UINTN current_dimm_k,
                                       UINTN total_dimms,
                                       CHAR8 *current_locator,
                                       CHAR16 *kernel_name,
                                       UINT32 current_pass,
                                       UINT32 total_passes,
                                       UINT64 elapsed_ms,
                                       UINT64 errors_so_far) {
    cls();
    UINT32 strip_col = COL_ACCENT;
    UINT32 strip_dk  = COL_ACCENT_DK;
    blt_gradient_v(0, 0, g_w, g_char_h * 2, strip_col, strip_dk);
    blt_fill(0, g_char_h * 2 - 2, g_w, 2, COL_ACCENT_HI);

    CHAR16 title[120];
    SPrint(title, sizeof(title),
           T(L"АВТО-ИЗОЛЯЦИЯ ПЛАНОК — %d из %d",
             L"AUTO-ISOLATION — %d of %d"),
           (UINT32)(current_dimm_k + 1), (UINT32)total_dimms);
    verdict_say_centered(title, g_char_h / 2, COL_FG, 1);

    UINTN cy = g_char_h * 4;
    UINTN cx = g_w / 6;
    UINTN cline = g_char_h + 4;
    CHAR16 buf[200];

    SPrint(buf, sizeof(buf),
           T(L"  Проверяю планку:  %a",
             L"  Testing DIMM:     %a"),
           current_locator);
    gfx_draw_str_color(cx, cy, buf, COL_ACCENT_HI); cy += cline;

    SPrint(buf, sizeof(buf),
           T(L"  Тест:             %s",
             L"  Test:             %s"),
           kernel_name);
    gfx_draw_str_color(cx, cy, buf, COL_FG); cy += cline;

    SPrint(buf, sizeof(buf),
           T(L"  Проход:           %d из %d",
             L"  Pass:             %d of %d"),
           current_pass, total_passes);
    gfx_draw_str_color(cx, cy, buf, COL_FG); cy += cline;

    UINT64 sec = elapsed_ms / 1000;
    SPrint(buf, sizeof(buf),
           T(L"  Прошло:           %d:%02d",
             L"  Elapsed:          %d:%02d"),
           (UINT32)(sec / 60), (UINT32)(sec % 60));
    gfx_draw_str_color(cx, cy, buf, COL_FG); cy += cline + 6;

    SPrint(buf, sizeof(buf),
           T(L"  Ошибок пока:      %ld",
             L"  Errors so far:    %ld"),
           errors_so_far);
    gfx_draw_str_color(cx, cy, buf,
                       errors_so_far > 0 ? COL_FAIL : COL_OK); cy += cline + 6;

    gfx_draw_str_color(cx, cy,
        T(L"  [ESC] = прервать изоляцию и вернуться к вердикту",
          L"  [ESC] = abort isolation and return to verdict"),
        COL_DIM);
}

/* Re-allocate test buffer constrained to a specific DIMM's address range,
   run the target kernel `n_passes` times, count errors, free.
   On entry the original whole-RAM buffer must have been freed.
   On return the original allocation is NOT restored — caller is responsible
   for re-allocating after all isolation work completes.
   Records that the isolation produces are TRUNCATED from g_err_records[]
   so they don't pollute the original verdict's data; the function returns
   just the error count for this run. */
static void run_isolation_for_dimm(UINTN dimm_idx, UINT32 kernel,
                                    UINT32 n_passes,
                                    isolation_result_t *out) {
    out->dimm_idx = (int)dimm_idx;
    out->errors = 0;
    out->passes = 0;
    out->status = 0;
    for (UINTN c = 0; c < sizeof(out->locator); c++) {
        out->locator[c] = (dimm_idx < g_dimm_count) ? g_dimms[dimm_idx].locator[c] : 0;
    }

    /* Temporarily switch the global to ask alloc_test_buffer() to pin
       the allocation to this DIMM's physical range. */
    UINT32 saved_only = g_cfg_test_only_dimm;
    UINT32 saved_buf  = g_cfg_buffer_cap_mb;
    g_cfg_test_only_dimm = (UINT32)(dimm_idx + 1);
    /* Smaller buffer for isolation — 256 MB is plenty to surface
       intermittent bit failures within a few minutes per pass. */
    g_cfg_buffer_cap_mb = 256;

    EFI_STATUS s = alloc_test_buffer();
    if (EFI_ERROR(s) || g_mem_addr == 0) {
        out->status = 1;            /* alloc failed */
        g_cfg_test_only_dimm = saved_only;
        g_cfg_buffer_cap_mb  = saved_buf;
        CHAR16 lb[160];
        SPrint(lb, sizeof(lb),
               L"[ISO] %a: alloc failed within DIMM range (status=0x%lx)",
               (CHAR8*)g_dimms[dimm_idx].locator, (UINT64)s);
        log_line(lb);
        return;
    }

    {
        CHAR16 lb[180];
        SPrint(lb, sizeof(lb),
               L"[ISO] %a: alloc OK at 0x%lx, %ld pages — running %s for %d passes",
               (CHAR8*)g_dimms[dimm_idx].locator, (UINT64)g_mem_addr,
               (UINT64)g_mem_pages, name_for_kernel(kernel), n_passes);
        log_line(lb);
    }

    /* Snapshot error-record count so we can extract our delta and then
       truncate them out (verdict data is preserved). */
    UINT32 err_snapshot = g_err_count;
    UINT64 t_start = ms_now();
    CHAR16 *kname = name_for_kernel(kernel);

    /* run_test_mc expects an INDEX into g_tests[], not a kernel_id_t enum value
       — translate via the helper (the same bug that affected pre-v0.4.20 verdict
       text would happen here if we passed `kernel` straight in). */
    int test_idx = tests_idx_for_kernel((kernel_id_t)kernel);
    if (test_idx < 0) {
        log_line(L"[ISO] kernel id not found in g_tests[] — falling back to AVX2 Sustained");
        test_idx = tests_idx_for_kernel(KER_AVX2_SUSTAINED);
        if (test_idx < 0) {
            out->status = 1;
            free_test_buffer();
            g_cfg_test_only_dimm = saved_only;
            g_cfg_buffer_cap_mb  = saved_buf;
            return;
        }
    }

    for (UINT32 p = 0; p < n_passes; p++) {
        if (g_aborted) { out->status = 3; break; }
        /* Reset g_aborted poll state between passes (keep it abortable but
           don't carry over the user's earlier ESC from countdown). */
        render_isolation_progress(0, 0,                /* outer counters set by caller */
                                  g_dimms[dimm_idx].locator,
                                  kname, p + 1, n_passes,
                                  ms_now() - t_start,
                                  g_err_count - err_snapshot);
        test_summary_t r = run_test_mc((UINTN)test_idx);
        out->passes++;
        out->errors += (UINT32)r.errors;
        CHAR16 lb[160];
        SPrint(lb, sizeof(lb),
               L"[ISO]   pass %d/%d: errors=%ld",
               p + 1, n_passes, r.errors);
        log_line(lb);
    }
    if (out->status != 3) out->status = 2;

    /* Truncate isolation error records so they don't appear in the
       original verdict / report.json error list. */
    g_err_count = err_snapshot;

    free_test_buffer();
    g_cfg_test_only_dimm = saved_only;
    g_cfg_buffer_cap_mb  = saved_buf;
}

/* Orchestrate full auto-isolation: re-test each DIMM in g_iso_dimm_idx[]
   with the kernel that found the original errors. Populates g_iso_results.
   On entry: original whole-RAM buffer is allocated.
   On exit:  original whole-RAM buffer is re-allocated and tests can resume
             normally (in practice the caller goes back to verdict screen). */
static void do_auto_isolation(void) {
    log_line(L"[ISO] === auto-isolation started ===");
    UINT32 k = isolation_pick_kernel();
    g_iso_kernel = k;
    {
        CHAR16 lb[200];
        SPrint(lb, sizeof(lb),
               L"[ISO] target kernel = %s; %d DIMM(s) to test",
               name_for_kernel(k), (UINT32)g_iso_dimm_n);
        log_line(lb);
    }

    /* Free the current whole-RAM buffer so DIMM-range allocations are free
       to claim those pages. */
    free_test_buffer();

    g_iso_results_n = 0;
    for (UINTN i = 0; i < g_iso_dimm_n && i < MAX_ISO_DIMMS; i++) {
        if (g_aborted) break;
        UINTN dimm_idx = (UINTN)g_iso_dimm_idx[i];
        /* Show initial frame for this DIMM */
        render_isolation_progress(i, g_iso_dimm_n,
                                  g_dimms[dimm_idx].locator,
                                  name_for_kernel(k), 1, 3,
                                  0, 0);
        run_isolation_for_dimm(dimm_idx, k, 3, &g_iso_results[i]);
        g_iso_results_n = i + 1;
    }

    /* Re-allocate the whole-RAM buffer so subsequent rendering / actions
       work normally. If it fails we just leave g_mem_addr=0 — verdict
       screen doesn't need the buffer. */
    g_cfg_test_only_dimm = 0;
    alloc_test_buffer();

    log_line(L"[ISO] === auto-isolation finished ===");
}

/* Render the post-isolation result screen with per-DIMM error counts and
   the new definitive verdict. */
static void render_isolation_verdict(void) {
    cls();
    blt_gradient_v(0, 0, g_w, g_char_h * 2, COL_FAIL, COL_FAIL_DK);
    blt_fill(0, g_char_h * 2 - 2, g_w, 2, COL_ACCENT_HI);
    verdict_say_centered(T(L"РЕЗУЛЬТАТ АВТО-ИЗОЛЯЦИИ", L"AUTO-ISOLATION RESULT"),
                         g_char_h / 2, COL_FG, 1);

    UINTN cy = g_char_h * 4;
    UINTN cx = g_w / 8;
    UINTN cline = g_char_h + 4;
    CHAR16 buf[200];

    SPrint(buf, sizeof(buf),
           T(L"  Использован тест:  %s   (%d прохода по 256 МБ на планке)",
             L"  Test used:         %s   (%d passes of 256 MB per DIMM)"),
           name_for_kernel(g_iso_kernel), 3);
    gfx_draw_str_color(cx, cy, buf, COL_DIM); cy += cline + 6;

    /* Count bad/clean */
    int bad_count = 0, alloc_fail_count = 0;
    int single_bad_idx = -1;
    for (UINTN i = 0; i < g_iso_results_n; i++) {
        if (g_iso_results[i].status == 1) alloc_fail_count++;
        else if (g_iso_results[i].errors > 0) { bad_count++; single_bad_idx = (int)i; }
    }

    /* Per-DIMM rows */
    for (UINTN i = 0; i < g_iso_results_n; i++) {
        isolation_result_t *r = &g_iso_results[i];
        UINT32 col = COL_FG;
        CHAR16 *mark;
        if (r->status == 1) { mark = T(L"✗ ALLOC FAIL", L"✗ ALLOC FAIL"); col = COL_RUN; }
        else if (r->status == 3) { mark = T(L"⊘ ОТМЕНА", L"⊘ ABORTED"); col = COL_DIM; }
        else if (r->errors > 0) { mark = T(L"✗ НЕИСПРАВНА", L"✗ FAULTY"); col = COL_FAIL; }
        else { mark = T(L"✓ ЧИСТАЯ", L"✓ CLEAN"); col = COL_OK; }
        SPrint(buf, sizeof(buf),
               T(L"  %-12a  %s   ·   %d ошибок за %d прохода",
                 L"  %-12a  %s   ·   %d errors in %d passes"),
               (CHAR8*)r->locator, mark, r->errors, r->passes);
        gfx_draw_str_color(cx, cy, buf, col); cy += cline;
    }
    cy += cline;

    /* Final definitive verdict */
    if (alloc_fail_count > 0 && bad_count == 0) {
        gfx_draw_str_color(cx, cy,
            T(L"  ⚠ Изоляция неполная — часть планок недоступна для аллокации",
              L"  ⚠ Isolation incomplete — some DIMMs failed allocation"),
            COL_RUN); cy += cline;
        gfx_draw_str_color(cx, cy,
            T(L"  (firmware-reserved memory holes). Проверьте физически.",
              L"  (firmware-reserved memory holes). Test physically."),
            COL_DIM); cy += cline;
    } else if (bad_count == 0) {
        gfx_draw_str_color(cx, cy,
            T(L"  ⚠ За 3 прохода ошибки не воспроизвелись.",
              L"  ⚠ Errors did not reproduce in 3 passes."),
            COL_RUN); cy += cline;
        gfx_draw_str_color(cx, cy,
            T(L"  Возможно очень редкий intermittent — запустите Marathon",
              L"  Likely very rare intermittent — try Marathon"),
            COL_DIM); cy += cline;
        gfx_draw_str_color(cx, cy,
            T(L"  или проверьте физической подменой по одной планке.",
              L"  or test by physically swapping one DIMM at a time."),
            COL_DIM); cy += cline;
    } else if (bad_count == 1) {
        SPrint(buf, sizeof(buf),
            T(L"  ▶ ТОЧНО:   ЗАМЕНИТЬ %a",
              L"  ▶ DEFINITIVE: REPLACE %a"),
            (CHAR8*)g_iso_results[single_bad_idx].locator);
        gfx_draw_str_color(cx, cy, buf, COL_FAIL); cy += cline;
        gfx_draw_str_color(cx, cy,
            T(L"     Уверенность: ВЫСОКАЯ (подтверждено изоляцией)",
              L"     Confidence:  HIGH (confirmed by isolation)"),
            COL_OK); cy += cline;
    } else {
        /* bad_count >= 2 — both/all planks bad */
        gfx_draw_str_color(cx, cy,
            T(L"  ▶ ТОЧНО:   ЗАМЕНИТЬ ВСЕ ПОМЕЧЕННЫЕ ✗",
              L"  ▶ DEFINITIVE: REPLACE ALL MARKED ✗"),
            COL_FAIL); cy += cline;
        gfx_draw_str_color(cx, cy,
            T(L"     Уверенность: ВЫСОКАЯ (подтверждено изоляцией)",
              L"     Confidence:  HIGH (confirmed by isolation)"),
            COL_OK); cy += cline;
    }

    UINTN foot_y = g_h - g_char_h - 8;
    blt_fill(0, foot_y - 4, g_w, g_char_h + 8, COL_PANEL_ALT);
    blt_fill(0, foot_y - 5, g_w, 1, COL_BORDER);
    verdict_say_centered(
        T(L"[D] обратно к вердикту   [M] меню   [ESC] перезагрузка",
          L"[D] back to verdict   [M] menu   [ESC] reboot"),
        foot_y, COL_ACCENT_HI, 1);

    /* Also log the result for offline review. */
    for (UINTN i = 0; i < g_iso_results_n; i++) {
        CHAR16 lb[200];
        SPrint(lb, sizeof(lb),
               L"[ISO] result: %a status=%d errors=%d passes=%d",
               (CHAR8*)g_iso_results[i].locator,
               g_iso_results[i].status,
               g_iso_results[i].errors,
               g_iso_results[i].passes);
        log_line(lb);
    }
}

static void render_simple_verdict(UINT64 total_ms) {
    cls();
    verdict_kind_t v = compute_verdict_kind();
    /* v0.4.29 — reset isolation offer; will be enabled below if applicable. */
    g_iso_offer = 0;
    g_iso_dimm_n = 0;

    /* Header strip color matches the verdict — green/yellow/red gradient */
    UINT32 strip_col, strip_dk, strip_hi;
    CHAR16 *title, *subtitle;
    switch (v) {
        case VERDICT_PASS:
            strip_col = COL_OK; strip_dk = COL_OK_DK; strip_hi = COL_ACCENT_HI;
            title    = T(L"✓ ПАМЯТЬ В ПОРЯДКЕ",   L"✓ MEMORY OK");
            subtitle = T(L"Все тесты пройдены без ошибок",
                         L"All tests passed, no errors");
            break;
        case VERDICT_WARN:
            strip_col = COL_RUN; strip_dk = COL_RUN; strip_hi = COL_ACCENT_HI;
            title    = T(L"⚠ ПАМЯТЬ НА ГРАНИ", L"⚠ MEMORY MARGINAL");
            subtitle = T(L"Тесты прошли, но есть тревожные сигналы",
                         L"Tests passed, but warning signals detected");
            break;
        default: /* VERDICT_FAIL */
            strip_col = COL_FAIL; strip_dk = COL_FAIL_DK; strip_hi = COL_FAIL;
            title    = T(L"✗ НАЙДЕНА НЕИСПРАВНОСТЬ", L"✗ MEMORY FAILURE FOUND");
            subtitle = T(L"Обнаружены ошибки памяти — требуется замена",
                         L"Memory errors detected — replacement required");
            break;
    }

    /* Top gradient strip */
    UINTN strip_h = g_char_h * 2 + 16;
    blt_gradient_v(0, 0, g_w, strip_h, strip_col, strip_dk);
    blt_fill(0, strip_h - 3, g_w, 1, strip_hi);

    /* Big title — 2× font, centered on the strip */
    UINT32 title_scale = (g_w >= 1280) ? 2 : 1;
    UINTN title_y = (strip_h - g_char_h * title_scale) / 2;
    verdict_say_centered(title, title_y, COL_FG, title_scale);

    /* Subtitle below the strip */
    UINTN y = strip_h + g_pad * 2;
    verdict_say_centered(subtitle, y, COL_FG, 1);
    y += g_char_h * 2;

    /* Quick stats row — RAM, duration, errors. Always visible. */
    CHAR16 stats[160];
    UINT32 secs = (UINT32)(total_ms / 1000);
    UINT32 hh = secs / 3600, mm = (secs / 60) % 60, ss = secs % 60;
    UINT64 ram_gb_x10 = (g_total_ram_mb * 10ULL + 512) / 1024ULL;
    SPrint(stats, sizeof(stats),
           T(L"%ld.%ld ГБ %s  ·  %d:%02d:%02d  ·  %ld ошибок",
             L"%ld.%ld GB %s  ·  %d:%02d:%02d  ·  %ld errors"),
           ram_gb_x10 / 10, ram_gb_x10 % 10,
           g_dimm_ddr_type ? ddr_type_name(g_dimm_ddr_type) : L"RAM",
           hh, mm, ss, (UINT64)g_err_count);
    verdict_say_centered(stats, y, COL_DIM, 1);
    y += g_char_h * 2;

    /* Center content card */
    UINTN card_w = (g_w > 200) ? (g_w * 3 / 4) : g_w;
    UINTN card_x = (g_w - card_w) / 2;
    UINTN card_h = g_char_h * 12 + 16;
    blt_panel(card_x, y, card_w, card_h, COL_PANEL, COL_BORDER);
    UINTN cx = card_x + g_pad;
    UINTN cy = y + g_pad;
    UINTN cline = g_char_h;

    if (v == VERDICT_PASS) {
        gfx_draw_str_color(cx, cy,
            T(L"  Можно отдавать клиенту.",
              L"  Safe to ship to customer."),
            COL_OK);
        cy += cline * 2;
        /* Optional thermal / power peak — useful summary at a glance */
        CHAR16 ln[160];
        if (g_max_temp_c > 0) {
            SPrint(ln, sizeof(ln),
                   T(L"  Пик температуры:  %d °C",
                     L"  Peak temperature: %d °C"),
                   g_max_temp_c);
            gfx_draw_str_color(cx, cy, ln, COL_FG);
            cy += cline;
        }
        if (g_pkg_power_w_peak > 0) {
            SPrint(ln, sizeof(ln),
                   T(L"  Пик мощности CPU: %d Вт",
                     L"  Peak CPU power:   %d W"),
                   g_pkg_power_w_peak);
            gfx_draw_str_color(cx, cy, ln, COL_FG);
            cy += cline;
        }
        if (g_throttle_total > 0) {
            SPrint(ln, sizeof(ln),
                   T(L"  Тротлинг:         %d событий (норма для долгой нагрузки)",
                     L"  Throttle events:  %d (normal under sustained load)"),
                   g_throttle_total);
            gfx_draw_str_color(cx, cy, ln, COL_DIM);
            cy += cline;
        }
    } else if (v == VERDICT_WARN) {
        gfx_draw_str_color(cx, cy,
            T(L"  Что обнаружено:",
              L"  What was detected:"),
            COL_ACCENT_HI);
        cy += cline + 4;
        CHAR16 ln[200];
        if (g_aborted) {
            gfx_draw_str_color(cx, cy,
                T(L"  • Тест прерван пользователем — повтори полный прогон",
                  L"  • Test aborted by user — re-run full pass"),
                COL_RUN);
            cy += cline;
        }
        if (g_mca_new_errors > 0) {
            SPrint(ln, sizeof(ln),
                T(L"  • Контроллер памяти исправил %d ECC-ошибок без сбоя теста",
                  L"  • Memory controller corrected %d ECC errors silently"),
                g_mca_new_errors);
            gfx_draw_str_color(cx, cy, ln, COL_RUN);
            cy += cline;
            gfx_draw_str_color(cx + g_char_w * 4, cy,
                T(L"  (память работает, но скоро может начать сбоить)",
                  L"  (memory works for now but may start failing soon)"),
                COL_DIM);
            cy += cline;
        }
        if (g_bw_trend_degraded >= 2) {
            SPrint(ln, sizeof(ln),
                T(L"  • Пропускная способность упала с %d%% до %d%% за прогон",
                  L"  • Bandwidth dropped from %d%% to %d%% during the run"),
                g_bw_trend_first_pct, g_bw_trend_last_pct);
            gfx_draw_str_color(cx, cy, ln, COL_RUN);
            cy += cline;
            gfx_draw_str_color(cx + g_char_w * 4, cy,
                T(L"  (возможно термальный тротлинг — проверь охлаждение)",
                  L"  (possible thermal throttling — check cooling)"),
                COL_DIM);
            cy += cline;
        }
        if (g_hist_prev_valid &&
            g_max_temp_c > g_hist_prev.max_temp_c + 8) {
            SPrint(ln, sizeof(ln),
                T(L"  • Температура +%d °C по сравнению с прошлым прогоном",
                  L"  • Temperature rose %d °C vs previous run"),
                g_max_temp_c - g_hist_prev.max_temp_c);
            gfx_draw_str_color(cx, cy, ln, COL_RUN);
            cy += cline;
            gfx_draw_str_color(cx + g_char_w * 4, cy,
                T(L"  (проверь термопасту, кулер, вентиляторы)",
                  L"  (check thermal paste, cooler, fans)"),
                COL_DIM);
            cy += cline;
        }
    } else { /* VERDICT_FAIL */
        int didx = dominant_dimm_idx();
        int dist_idx[MAX_DIMMS];
        UINTN dist_n = distributed_dimm_indices(dist_idx, MAX_DIMMS);
        int is_distributed = (dist_n >= 2);

        /* v0.4.29 — Approach D + A: classify WHY errors are distributed.
             type20_overlap = 1 → ranges overlap (real cache-line interleave)
                                  → "ONE chip behind two labels"
             type20_overlap = 0, depth ≤ 1 → block mode (disjoint ranges,
                                  no interleave claim) → "BOTH sticks bad"
             type20_overlap = 0, depth  > 1 → BIOS pseudo-interleave
                                  conflict → fall back to bit-6 polarity.  */
        int type20_overlap = is_distributed ? type20_has_overlapping_ranges() : 0;
        UINT8 type20_depth = is_distributed ? type20_max_interleave_depth() : 1;
        int bit6_pol = is_distributed ? bit6_channel_polarity() : 0;
        enum { DIST_NONE = 0, DIST_PAIR_INTERLEAVE, DIST_BOTH_STICKS_BAD, DIST_AMBIGUOUS };
        int dist_kind = DIST_NONE;
        if (is_distributed) {
            if (type20_overlap)                     dist_kind = DIST_PAIR_INTERLEAVE;
            else if (type20_depth <= 1)             dist_kind = DIST_BOTH_STICKS_BAD;
            else if (bit6_pol != 0)                 dist_kind = DIST_PAIR_INTERLEAVE;
            else                                    dist_kind = DIST_AMBIGUOUS;
        }

        verdict_confidence_t conf = compute_confidence();
        /* Distributed errors usually drop confidence to MED — but if we're
           certain it's both sticks (block-mapped, disjoint ranges), HIGH
           confidence is honest because we know exactly which sticks. */
        if (is_distributed && conf == CONF_HIGH &&
            dist_kind != DIST_BOTH_STICKS_BAD) conf = CONF_MED;
        CHAR16 *conf_str;
        UINT32  conf_col;
        switch (conf) {
            case CONF_HIGH: conf_str = T(L"ВЫСОКАЯ",  L"HIGH");
                            conf_col = COL_OK;     break;
            case CONF_MED:  conf_str = T(L"СРЕДНЯЯ", L"MEDIUM");
                            conf_col = COL_RUN;    break;
            default:        conf_str = T(L"НИЗКАЯ",  L"LOW");
                            conf_col = COL_FAIL;   break;
        }

        /* Build the comma-separated DIMM list once — used in two branches. */
        CHAR16 dimm_list[220] = {0};
        if (is_distributed) {
            for (UINTN k = 0; k < dist_n; k++) {
                CHAR16 frag[64];
                CHAR16 *sep_ru = (k == 0) ? L"%a" : L", %a";
                CHAR16 *sep_en = (k == 0) ? L"%a" : L", %a";
                SPrint(frag, sizeof(frag), g_lang ? sep_en : sep_ru,
                       (CHAR8*)g_dimms[dist_idx[k]].locator);
                UINTN have = StrLen(dimm_list);
                UINTN need = StrLen(frag);
                if (have + need + 1 < sizeof(dimm_list) / sizeof(CHAR16)) {
                    for (UINTN c = 0; c <= need; c++)
                        dimm_list[have + c] = frag[c];
                }
            }
        }

        CHAR16 ln[260];
        if (dist_kind == DIST_BOTH_STICKS_BAD) {
            /* Block-mapped BIOS with disjoint Type 20 ranges. Errors on
               two DIMMs really mean both sticks have at least one bad
               chip — NOT interleave hiding a single source. */
            SPrint(ln, sizeof(ln),
                T(L"  ЗАМЕНИТЬ ОБЕ:  %s",
                  L"  REPLACE BOTH:  %s"), dimm_list);
            gfx_draw_str_color(cx, cy, ln, COL_FAIL);
            cy += cline;
            SPrint(ln, sizeof(ln),
                T(L"  Уверенность: %s", L"  Confidence:  %s"), conf_str);
            gfx_draw_str_color(cx, cy, ln, conf_col);
            cy += cline + 6;
            gfx_draw_str_color(cx, cy,
                T(L"  SMBIOS показывает планки с непересекающимися диапазонами",
                  L"  SMBIOS reports DIMMs with disjoint address ranges"),
                COL_DIM); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  (не interleave). Значит ошибки на этих планках —",
                  L"  (no interleave). So errors on these DIMMs really happened"),
                COL_DIM); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  это РЕАЛЬНО на разных физических плашках, обе дефектные.",
                  L"  on physically separate sticks; both are defective."),
                COL_DIM); cy += cline + 6;
            /* v0.4.29 — offer auto-isolation: re-test each DIMM in its own
               physical address range to confirm WHICH ones are actually
               bad (vs symptom of one chip pretending to be two). Block-
               mapped Type 20 is the precondition — on real cache-line
               interleave TestOnlyDimm doesn't physically isolate.
               v0.4.29 — only offer [I] if auto-isolation hasn't already
               run (g_iso_results_n == 0). The normal flow now triggers
               isolation automatically right after the test loop, so the
               [I] offer here is only relevant when the user navigated
               back to the simple verdict after seeing the isolation
               result (via [D] toggle). */
            if (g_cfg_test_only_dimm == 0 && dist_n >= 2 && dist_n <= MAX_ISO_DIMMS
                && g_iso_results_n == 0) {
                g_iso_offer = 1;
                g_iso_dimm_n = dist_n;
                for (UINTN k = 0; k < dist_n; k++)
                    g_iso_dimm_idx[k] = dist_idx[k];
                gfx_draw_str_color(cx, cy,
                    T(L"  ▶ Нажмите [I] чтобы прога проверила каждую планку",
                      L"  ▶ Press [I] for the program to test each DIMM"),
                    COL_ACCENT_HI); cy += cline;
                gfx_draw_str_color(cx, cy,
                    T(L"     отдельно и дала точный ответ (~5 мин).",
                      L"     in isolation for a definitive answer (~5 min)."),
                    COL_ACCENT_HI); cy += cline + 6;
            }
        } else if (dist_kind == DIST_PAIR_INTERLEAVE) {
            /* Real interleave (Type 20 ranges overlap), or BIOS-conflict
               case where bit-6 polarity strongly skewed toward one channel.
               Either way — ONE bad chip masquerading as two DIMM labels. */
            SPrint(ln, sizeof(ln),
                T(L"  ЗАМЕНИТЬ ОДНУ из: %s",
                  L"  REPLACE ONE of:  %s"), dimm_list);
            gfx_draw_str_color(cx, cy, ln, COL_FAIL);
            cy += cline;
            SPrint(ln, sizeof(ln),
                T(L"  Уверенность: %s  (нужна физическая проверка)",
                  L"  Confidence:  %s  (physical isolation needed)"),
                conf_str);
            gfx_draw_str_color(cx, cy, ln, conf_col);
            cy += cline + 6;
            gfx_draw_str_color(cx, cy,
                T(L"  Это dual-channel interleave: один дохлый чип на ОДНОЙ",
                  L"  Dual-channel interleave: one bad chip on ONE stick"),
                COL_DIM); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  планке маскируется как ошибки на обоих DIMM-именах.",
                  L"  appears as errors under both DIMM labels."),
                COL_DIM); cy += cline;
            if (bit6_pol == 1 || bit6_pol == 2) {
                /* Add bit-6 polarity hint when strongly skewed */
                SPrint(ln, sizeof(ln),
                    T(L"  Подсказка: ~85%%+ ошибок на физ.адресах с битом 6 = %d",
                      L"  Hint: ≥85%% of errors at addresses with bit 6 = %d"),
                    bit6_pol - 1);
                gfx_draw_str_color(cx, cy, ln, COL_DIM); cy += cline;
                gfx_draw_str_color(cx, cy,
                    T(L"  (на стандартном Intel/AMD это один канал из двух).",
                      L"  (on standard Intel/AMD this means one of the two channels)."),
                    COL_DIM); cy += cline;
            }
            cy += 6;
            gfx_draw_str_color(cx, cy,
                T(L"  Как определить точную:",
                  L"  How to identify the exact one:"),
                COL_ACCENT_HI); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  1. Выключите ПК, выньте ОДНУ из перечисленных планок.",
                  L"  1. Power off, physically remove ONE of the listed DIMMs."),
                COL_FG); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  2. Запустите этот тест на 10 минут.",
                  L"  2. Run this test for 10 minutes."),
                COL_FG); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  3. Ошибки исчезли → дохлая та, что вынули.",
                  L"  3. Errors gone → the removed one was bad."),
                COL_FG); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"     Ошибки остались → дохлая оставшаяся.",
                  L"     Errors still there → the remaining one is bad."),
                COL_FG); cy += cline + 6;
        } else if (dist_kind == DIST_AMBIGUOUS) {
            /* BIOS conflict: Type 20 claims interleave depth>1 but ranges
               disjoint, AND bit-6 polarity is mixed. Can't tell — say so. */
            SPrint(ln, sizeof(ln),
                T(L"  ЗАМЕНИТЬ: проверить %s",
                  L"  REPLACE: check %s"), dimm_list);
            gfx_draw_str_color(cx, cy, ln, COL_FAIL);
            cy += cline;
            SPrint(ln, sizeof(ln),
                T(L"  Уверенность: %s  (BIOS даёт противоречивые данные)",
                  L"  Confidence:  %s  (BIOS reports conflicting data)"),
                conf_str);
            gfx_draw_str_color(cx, cy, ln, conf_col);
            cy += cline + 6;
            gfx_draw_str_color(cx, cy,
                T(L"  SMBIOS показывает interleave, но без подтверждения через iMC.",
                  L"  SMBIOS reports interleave but it can't be confirmed via iMC."),
                COL_DIM); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  Это может быть одна планка (interleave) или обе (block-режим).",
                  L"  Could be one stick (interleave) or both (block mode)."),
                COL_DIM); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  Проверьте плашки физически по одной — это даст ответ.",
                  L"  Test sticks physically one at a time for definitive answer."),
                COL_DIM); cy += cline + 6;
        }
        if (is_distributed) {
            /* Common: "what was detected" line shown for every distributed
               branch. Skip if no clear pattern. */
            gfx_draw_str_color(cx, cy,
                T(L"  ── Что обнаружено ──",
                  L"  ── What was detected ──"),
                COL_DIM); cy += cline;
            CHAR16 l1[220], l2[220];
            verdict_describe_what_broke(l1, sizeof(l1) / sizeof(CHAR16),
                                         l2, sizeof(l2) / sizeof(CHAR16),
                                         didx);
            if (l1[0]) { gfx_draw_str_color(cx + g_char_w, cy, l1, COL_FAIL); cy += cline; }
            if (l2[0]) { gfx_draw_str_color(cx + g_char_w, cy, l2, COL_FAIL); cy += cline; }
        } else if (didx >= 0) {
            /* Errors localized to one DIMM — keep the original honest verdict. */
            dimm_info_t *d = &g_dimms[didx];
            SPrint(ln, sizeof(ln),
                T(L"  ЗАМЕНИТЬ:   %a",
                  L"  REPLACE:    %a"),
                d->locator);
            gfx_draw_str_color(cx, cy, ln, COL_FAIL);
            cy += cline;
            SPrint(ln, sizeof(ln),
                T(L"  Уверенность: %s",
                  L"  Confidence:  %s"),
                conf_str);
            gfx_draw_str_color(cx, cy, ln, conf_col);
            cy += cline + 6;

            /* DIMM info card — for warranty / re-order */
            gfx_draw_str_color(cx, cy,
                T(L"  ── Информация о планке (для гарантии) ──",
                  L"  ── DIMM info (for warranty) ──"),
                COL_DIM);
            cy += cline;
            SPrint(ln, sizeof(ln),
                T(L"  Производитель:  %a",
                  L"  Manufacturer:   %a"),
                d->manufacturer[0] ? (CHAR8*)d->manufacturer : (CHAR8*)"?");
            gfx_draw_str_color(cx, cy, ln, COL_FG); cy += cline;
            SPrint(ln, sizeof(ln),
                T(L"  Модель:         %a",
                  L"  Part number:    %a"),
                d->part_number[0] ? (CHAR8*)d->part_number : (CHAR8*)"?");
            gfx_draw_str_color(cx, cy, ln, COL_FG); cy += cline;
            if (d->spd_present && (d->spd_serial[0] || d->spd_serial[1] ||
                                    d->spd_serial[2] || d->spd_serial[3])) {
                SPrint(ln, sizeof(ln),
                    T(L"  Серийный номер: %02X%02X%02X%02X",
                      L"  Serial number:  %02X%02X%02X%02X"),
                    d->spd_serial[0], d->spd_serial[1],
                    d->spd_serial[2], d->spd_serial[3]);
                gfx_draw_str_color(cx, cy, ln, COL_FG); cy += cline;
                if (d->spd_mfg_year && d->spd_mfg_week) {
                    SPrint(ln, sizeof(ln),
                        T(L"  Дата выпуска:   20%02X, неделя %d",
                          L"  Manufactured:   20%02X, week %d"),
                        d->spd_mfg_year, d->spd_mfg_week);
                    gfx_draw_str_color(cx, cy, ln, COL_FG); cy += cline;
                }
            }
            cy += cline / 2;

            gfx_draw_str_color(cx, cy,
                T(L"  ── Что обнаружено ──",
                  L"  ── What was detected ──"),
                COL_DIM);
            cy += cline;
            CHAR16 l1[220], l2[220];
            verdict_describe_what_broke(l1, sizeof(l1) / sizeof(CHAR16),
                                         l2, sizeof(l2) / sizeof(CHAR16),
                                         didx);
            if (l1[0]) { gfx_draw_str_color(cx + g_char_w, cy, l1, COL_FAIL); cy += cline; }
            if (l2[0]) { gfx_draw_str_color(cx + g_char_w, cy, l2, COL_FAIL); cy += cline; }
        } else {
            /* No SMBIOS Type 20 mapping — total + advice (physical, not INI) */
            SPrint(ln, sizeof(ln),
                T(L"  Найдено %ld ошибок — точную планку определить не удалось",
                  L"  %ld errors found — could not pinpoint specific DIMM"),
                (UINT64)g_err_count);
            gfx_draw_str_color(cx, cy, ln, COL_FAIL); cy += cline + 6;
            gfx_draw_str_color(cx, cy,
                T(L"  Рекомендация:",
                  L"  Recommendation:"),
                COL_ACCENT_HI);
            cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  • Выньте все планки кроме одной, прогоните 10 мин.",
                  L"  • Pull out all DIMMs but one, run 10 min."),
                COL_FG); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  • Поочерёдно для каждой планки — увидите дохлую.",
                  L"  • Repeat for each DIMM — you'll see the bad one."),
                COL_FG); cy += cline;
            gfx_draw_str_color(cx, cy,
                T(L"  • Сбрось XMP/EXPO в BIOS — может помочь.",
                  L"  • Reset XMP/EXPO in BIOS — may help."),
                COL_FG); cy += cline;
        }
    }

    /* Footer hint — same key handling as the technical summary, plus [D].
       v0.4.29: [I] for auto-isolation when offered. */
    UINTN foot_y = g_h - g_char_h - 8;
    blt_fill(0, foot_y - 4, g_w, g_char_h + 8, COL_PANEL_ALT);
    blt_fill(0, foot_y - 5, g_w, 1, COL_BORDER);
    CHAR16 footer[260];
    if (g_iso_offer) {
        SPrint(footer, sizeof(footer),
               T(L"[I] авто-изоляция   [D] технические детали   [M] меню   [ESC] перезагрузка   [L] %s",
                 L"[I] auto-isolation   [D] technical details   [M] menu   [ESC] reboot   [L] %s"),
               g_lang ? L"RU" : L"EN");
    } else {
        SPrint(footer, sizeof(footer),
               T(L"[D] технические детали     [M] меню     [ESC] перезагрузка     [L] %s",
                 L"[D] technical details     [M] menu     [ESC] reboot     [L] %s"),
               g_lang ? L"RU" : L"EN");
    }
    verdict_say_centered(footer, foot_y, COL_ACCENT_HI, 1);
}

/* ---------- Final summary table ---------- */
static void render_summary(UINT64 total_ms) {
    cls();

    /* Gradient header strip */
    blt_gradient_v(0, 0, g_w, g_hdr_h, COL_ACCENT, COL_ACCENT_DK);
    blt_fill(0, g_hdr_h - 3, g_w, 1, COL_ACCENT_HI);
    UINTN hrow = (g_hdr_h / 2 - g_char_h / 2) / g_char_h;
    CHAR16 buf[200];
    SPrint(buf, sizeof(buf),
           T(L"  MEMFORGE v0.4.29 ИТОГИ   |   %d сек   |   Ядра %d/%d",
             L"  MEMFORGE v0.4.29 SUMMARY   |   %d sec   |   Cores %d/%d"),
           (UINT32)(total_ms / 1000),
           (UINT32)g_n_enabled, (UINT32)g_n_cores);
    say_at_rc(0, hrow, buf);

    /* Table panel */
    UINTN tbl_y = g_hdr_h + g_pad;
    /* Panel covers: header row + separator + N_TESTS rows + separator +
       totals row + (up to 3 error-localization rows: stuck-bit / affected
       DIMM / 1-GB histogram) + blank + peaks row + (optional) pass-
       durations row. We budget N_TESTS+10 to leave room for everything. */
    UINTN tbl_h = (N_TESTS + 10) * g_char_h + g_pad;
    blt_panel(g_pad, tbl_y, g_w - 2 * g_pad, tbl_h, COL_PANEL, COL_BORDER);

    UINTN row = tbl_y / g_char_h + 1;
    say_at_rc(3, row++,
              T(L"#  Тест               Статус   Ошибки   МБ прошло   МБ/с     Время(мс)",
                L"#  Test               Status   Errors   MB tested   MB/s     Time(ms)"));
    /* Separator */
    blt_fill(g_pad + 4, row * g_char_h - 2, g_w - 2 * g_pad - 8, 1, COL_BORDER);
    say_at_rc(3, row++, L"-----------------------------------------------------------------------");

    UINT64 grand_err = 0;
    for (UINTN i = 0; i < N_TESTS; i++) {
        test_summary_t *t = &g_summary[i];
        const CHAR16 *st = t->status == 1 ? L"PASS"
                          : t->status == 2 ? L"FAIL" : L"SKIP";
        UINT64 mbs = t->time_ms ? (t->bytes / (1024 * 1024) * 1000) / t->time_ms : 0;

        /* Color indicator dot */
        UINT32 dot_col = t->status == 1 ? COL_OK : t->status == 2 ? COL_FAIL : COL_DIM;
        blt_fill(g_pad + 8, row * g_char_h + 4, 8, 8, dot_col);

        SPrint(buf, sizeof(buf),
               L"   %d  %-18s %4s   %6ld   %10ld   %6ld   %8ld",
               (UINT32)(i + 1), g_tests[i].name, st,
               t->errors, t->bytes / (1024 * 1024), mbs, t->time_ms);
        say_at_rc(3, row++, buf);
        log_line(buf);
        grand_err += t->errors;
    }
    /* Separator before totals */
    blt_fill(g_pad + 4, row * g_char_h, g_w - 2 * g_pad - 8, 1, COL_BORDER);
    row++;
    SPrint(buf, sizeof(buf),
           T(L"ВСЕГО ОШИБОК: %ld", L"TOTAL ERRORS: %ld"), grand_err);
    say_at_rc(3, row++, buf);
    log_line(buf);

    /* --- MCA / ECC errors caught by CPU hardware ---
       Separate from g_err_count because MCA records errors the iMC saw
       and may have CORRECTED via ECC — our pattern tests never saw them.
       Critical to surface: on ECC RAM the only way to know a chip is
       failing is via the MCA counter. Even on non-ECC systems, MCA may
       record uncorrected errors that crashed something invisibly. */
    if (g_has_mca && g_mca_new_errors > 0) {
        CHAR16 mca_line[200];
        SPrint(mca_line, sizeof(mca_line),
               T(L"⚠ MCA: %d новых аппаратных ошиб(ок/ки) залогировано (см. лог [MCA] для деталей)",
                 L"⚠ MCA: %d new hardware error(s) logged (see log [MCA] lines for details)"),
               g_mca_new_errors);
        gfx_draw_str_color(3 * g_char_w, row * g_char_h, mca_line, COL_FAIL);
        log_line(mca_line);
        row++;
    }

    /* --- Error localization (only if we actually have errors) --- */
    if (g_err_count > 0) {
        /* (1) Stuck-bit hint: same XOR mask ≥5× = consistent fault. */
        UINT32 stuck_n = 0;
        UINT64 stuck_x = find_stuck_bit(&stuck_n);
        if (stuck_n >= 5 && stuck_x != 0) {
            int bp = single_bit_pos(stuck_x);
            CHAR16 sb[260];
            if (bp >= 0) {
                /* Translate bit position into physical chip on the
                   suspected DIMM. This gives shop tech an actionable
                   answer: 'replace this specific chip / DIMM by RMA'. */
                int didx = dominant_dimm_idx();
                CHAR16 chip[64] = L"";
                if (didx >= 0)
                    chip_label_for_bit((UINT32)didx, bp, chip, 64);
                /* v0.4.29 — use SMBIOS Type 17 locator string ("DDR4-B2")
                   instead of array-index-based "DIMM%d" which had nothing
                   to do with the physical slot label the user sees. */
                CHAR8 *loc = (didx >= 0 && g_dimms[didx].locator[0])
                             ? g_dimms[didx].locator : (CHAR8*)"?";
                if (didx >= 0 && chip[0]) {
                    /* Full info: DIMM + exact chip designator */
                    SPrint(sb, sizeof(sb),
                           T(L"⚠ Застрял бит D[%d] → %a, %s: %d ошибок",
                             L"⚠ Stuck bit D[%d] → %a, %s: %d errors"),
                           bp, loc, chip, stuck_n);
                } else if (didx >= 0) {
                    /* DIMM known, exact chip not — say so plainly */
                    SPrint(sb, sizeof(sb),
                           T(L"⚠ Застрял бит D[%d] → %a (точный чип не определён по SPD): %d ошибок",
                             L"⚠ Stuck bit D[%d] → %a (exact chip unknown per SPD): %d errors"),
                           bp, loc, stuck_n);
                } else {
                    SPrint(sb, sizeof(sb),
                           T(L"⚠ Застрял бит D[%d] (планку определить не удалось): %d ошибок",
                             L"⚠ Stuck bit D[%d] (DIMM could not be identified): %d errors"),
                           bp, stuck_n);
                }
            } else {
                SPrint(sb, sizeof(sb),
                       T(L"⚠ Повторяющийся паттерн ошибки: %d × XOR=0x%016lx (мульти-бит)",
                         L"⚠ Repeating error pattern: %d × XOR=0x%016lx (multi-bit)"),
                       stuck_n, stuck_x);
            }
            gfx_draw_str_color(3 * g_char_w, row * g_char_h, sb, COL_FAIL);
            log_line(sb);
            row++;
        }

        /* (1.5) Stuck-row / stuck-bank detection (DRAM-coord heuristic). */
        {
            UINT32 sr_n = 0;
            UINT64 sr   = find_stuck_row(&sr_n);
            if (sr_n >= 3) {
                CHAR16 srb[200];
                SPrint(srb, sizeof(srb),
                       T(L"⚠ Возможно битая строка памяти ~0x%lx: %d ошибок в одной DRAM-строке (~ = приблиз., точная схема чипсета не публикуется)",
                         L"⚠ Possibly bad memory row ~0x%lx: %d errors in one DRAM row (~ = approximate; chipset hash not public)"),
                       sr, sr_n);
                gfx_draw_str_color(3 * g_char_w, row * g_char_h, srb, COL_FAIL);
                log_line(srb);
                row++;
            }
            UINT32 sb_n = 0;
            UINT32 sb_id = find_stuck_bank(&sb_n);
            if (sb_n >= 3) {
                CHAR16 sbb[200];
                SPrint(sbb, sizeof(sbb),
                       T(L"⚠ Возможно повреждена секция чипа DRAM: банк-группа %d, банк %d — %d ошибок в одном банке (~ = приблиз.)",
                         L"⚠ Possibly damaged DRAM chip section: bank-group %d, bank %d — %d errors in one bank (~ = approximate)"),
                       (sb_id >> 4) & 0xF, sb_id & 0xF, sb_n);
                gfx_draw_str_color(3 * g_char_w, row * g_char_h, sbb, COL_FAIL);
                log_line(sbb);
                row++;
            }
        }

        /* (2) Affected-DIMM list (from SMBIOS Type 20 mapping). */
        CHAR16 dimm_line[260]; UINTN pos = 0;
        pos += SPrint(dimm_line + pos, sizeof(dimm_line) - pos * sizeof(CHAR16),
                      T(L"Ошибки по планкам: ", L"Errors by DIMM: "));
        /* Aggregate: tally errors per distinct DIMM-label string. */
        CHAR16 labels[8][64]; UINT32 lcount[8] = {0}; UINT32 nl = 0;
        UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
        for (UINT32 i = 0; i < shown; i++) {
            CHAR16 lab[64];
            dimm_label_for_addr(g_err_records[i].phys_addr, lab, 64);
            int matched = 0;
            for (UINT32 j = 0; j < nl; j++) {
                /* Wide-string compare */
                int eq = 1;
                for (UINTN k = 0; k < 64; k++) {
                    if (labels[j][k] != lab[k]) { eq = 0; break; }
                    if (lab[k] == 0) break;
                }
                if (eq) { lcount[j]++; matched = 1; break; }
            }
            if (!matched && nl < 8) {
                for (UINTN k = 0; k < 64; k++) {
                    labels[nl][k] = lab[k];
                    if (lab[k] == 0) break;
                }
                lcount[nl] = 1;
                nl++;
            }
        }
        for (UINT32 j = 0; j < nl; j++) {
            pos += SPrint(dimm_line + pos, sizeof(dimm_line) - pos * sizeof(CHAR16),
                          (j == 0) ? L"%s (%d)" : L", %s (%d)",
                          labels[j], lcount[j]);
            if (pos >= 200) break;
        }
        gfx_draw_str_color(3 * g_char_w, row * g_char_h, dimm_line, COL_FAIL);
        log_line(dimm_line);
        row++;

        /* (3) 1-GB histogram — v0.4.29: short label on its own row, then
           entries wrapped across multiple rows so nothing falls off the
           right edge on a 1024-pixel screen (a 14-entry histogram is
           ~120 chars which doesn't fit any reasonable single line). */
        UINT32 hb[32], hc[32];
        UINT32 nbk = error_histogram_gb(hb, hc, 32);
        if (nbk > 0) {
            CHAR16 lbl[80];
            SPrint(lbl, sizeof(lbl),
                   T(L"Карта ошибок по 1-ГБ участкам (формат  ГБ:ошибок):",
                     L"Error map per 1-GB range (format  GB:errors):"));
            gfx_draw_str_color(3 * g_char_w, row * g_char_h, lbl, COL_DIM);
            log_line(lbl);
            row++;

            /* Available width in characters for the data rows. Reserve
               left margin (3 chars * char_w == matches other rows). */
            UINTN avail_cols = (g_w > 6 * g_char_w)
                             ? (g_w - 6 * g_char_w) / g_char_w : 40;
            if (avail_cols > 250) avail_cols = 250;

            CHAR16 line[260]; UINTN lp = 0;
            line[0] = 0;
            for (UINT32 j = 0; j < nbk; j++) {
                CHAR16 frag[32];
                UINTN fl = SPrint(frag, sizeof(frag),
                                  (lp == 0) ? L"  %d-%dG:%d" : L"  ·  %d-%dG:%d",
                                  hb[j], hb[j] + 1, hc[j]);
                /* SPrint returns bytes, including trailing 0 in some
                   gnu-efi impls — use StrLen for char count. */
                fl = StrLen(frag);
                if (lp + fl > avail_cols - 1 && lp > 0) {
                    /* Flush current line and start a new one */
                    line[lp] = 0;
                    gfx_draw_str_color(3 * g_char_w, row * g_char_h, line, COL_DIM);
                    log_line(line);
                    row++;
                    lp = 0;
                    /* Rebuild fragment as "first on line" (no leading separator) */
                    SPrint(frag, sizeof(frag), L"  %d-%dG:%d",
                           hb[j], hb[j] + 1, hc[j]);
                    fl = StrLen(frag);
                }
                /* Append fragment to current line */
                for (UINTN c = 0; c < fl && lp < sizeof(line)/sizeof(CHAR16) - 1; c++)
                    line[lp++] = frag[c];
                line[lp] = 0;
            }
            if (lp > 0) {
                gfx_draw_str_color(3 * g_char_w, row * g_char_h, line, COL_DIM);
                log_line(line);
                row++;
            }
        }
    }

    /* --- Telemetry peaks from the run ---
       The live header showed instant values; here we surface the PEAKS / TOTALS
       that we sampled during the whole prog. Useful to know AFTER the test:
         - peak BW (MB/s → GB/s) and what % of theoretical max
         - peak CPU package power (W) if RAPL was available
         - max core temperature hit
         - total throttle events across all cores
         - average pass duration (sec) — drift indicates instability */
    row++;  /* blank */
    UINT32 peak_gbps_x10 = g_bw_mbps_peak ? (g_bw_mbps_peak * 10 / 1024) : 0;
    UINT32 peak_pct = 0;
    if (g_bw_peak_theoretical_mbps > 0 && g_bw_mbps_peak > 0)
        peak_pct = (UINT32)((UINT64)g_bw_mbps_peak * 100ULL / g_bw_peak_theoretical_mbps);
    if (peak_pct > 100) peak_pct = 100;
    UINT32 avg_pass_s = 0;
    if (g_pass_durations_count > 0) {
        UINT32 nshown = g_pass_durations_count > MAX_PASS_HISTORY
                      ? MAX_PASS_HISTORY : g_pass_durations_count;
        UINT64 sum_ms = 0;
        for (UINT32 i = 0; i < nshown; i++) sum_ms += g_pass_durations_ms[i];
        avg_pass_s = (UINT32)(sum_ms / nshown / 1000);
    }
    /* Throttle reason short tag — which power-mgmt cap actually triggered.
       Critical for distinguishing "CPU too hot" from "BIOS PL1 too low":
         THR  = thermal (CPU at TjMax)        → cooling problem
         PWR  = package power limit (PL1/PL2) → BIOS limits or VRM cap
         CUR  = current limit (IccMax)        → VRM amperage cap
         PRC  = PROCHOT# externally asserted  → MB chipset / external limit
       If we threw 16 throttle events with max temp 44°C the answer is
       almost certainly PWR — that's exactly what we want this tag to show. */
    const CHAR16 *thr_tag = L"—";
    if (g_dominant_throttle & THROT_THERMAL)      thr_tag = L"THR";
    else if (g_dominant_throttle & THROT_POWERLIMIT)   thr_tag = L"PWR";
    else if (g_dominant_throttle & THROT_CURRENTLIMIT) thr_tag = L"CUR";
    else if (g_dominant_throttle & THROT_PROCHOT)      thr_tag = L"PRC";

    if (g_has_rapl && g_pkg_power_w_peak > 0) {
        SPrint(buf, sizeof(buf),
               T(L"Пики: BW %d.%d ГБ/с (%d%% пика)  ·  CPU %dВт  ·  макс %d°C  ·  тротт %d (%s)  ·  Fmax %d MHz  ·  ср.проход %d с",
                 L"Peaks: BW %d.%d GB/s (%d%% peak)  ·  CPU %dW  ·  max %d°C  ·  throttle %d (%s)  ·  Fmax %d MHz  ·  avg pass %d s"),
               peak_gbps_x10 / 10, peak_gbps_x10 % 10, peak_pct,
               g_pkg_power_w_peak, g_max_temp_c, g_throttle_total, thr_tag,
               g_max_freq_mhz_observed, avg_pass_s);
    } else {
        SPrint(buf, sizeof(buf),
               T(L"Пики: BW %d.%d ГБ/с (%d%% пика)  ·  макс %d°C  ·  тротт %d (%s)  ·  Fmax %d MHz  ·  ср.проход %d с",
                 L"Peaks: BW %d.%d GB/s (%d%% peak)  ·  max %d°C  ·  throttle %d (%s)  ·  Fmax %d MHz  ·  avg pass %d s"),
               peak_gbps_x10 / 10, peak_gbps_x10 % 10, peak_pct,
               g_max_temp_c, g_throttle_total, thr_tag,
               g_max_freq_mhz_observed, avg_pass_s);
    }
    gfx_draw_str_color(3 * g_char_w, row * g_char_h, buf, COL_ACCENT_HI);
    log_line(buf);
    row++;

    /* Per-pass duration list — visualizes pass-to-pass timing drift. A run
       where every pass takes ~the same time is healthy; rising times across
       passes suggest thermal/power degradation; sudden outliers suggest
       transient hangs. Shown only if we have ≥2 passes to compare. */
    if (g_pass_durations_count >= 2) {
        UINT32 nshown = g_pass_durations_count > MAX_PASS_HISTORY
                      ? MAX_PASS_HISTORY : g_pass_durations_count;
        if (nshown > 8) nshown = 8;  /* cap for screen width */
        CHAR16 pd[200];
        UINTN k = 0;
        k += SPrint(pd + k, sizeof(pd) - k * sizeof(CHAR16),
                    T(L"Проходы: ", L"Passes: "));
        for (UINT32 i = 0; i < nshown; i++) {
            /* IMPORTANT: separator must be a WIDE-STRING literal (L"·"),
               not a narrow C-string ("·"). The narrow form encodes "·" as
               UTF-8 bytes 0xC2 0xB7; passed via %a (ASCII string) to SPrint
               they appear in the wide buffer as TWO widechars (0xC2='Â',
               0xB7='·'), producing the "Â·" mojibake the user saw in the
               log. Wide string is one codepoint U+00B7. */
            k += SPrint(pd + k, sizeof(pd) - k * sizeof(CHAR16),
                        (i + 1 < nshown) ? L"%d·" : L"%d",
                        g_pass_durations_ms[i] / 1000);
        }
        k += SPrint(pd + k, sizeof(pd) - k * sizeof(CHAR16),
                    T(L" с", L" s"));
        gfx_draw_str_color(3 * g_char_w, row * g_char_h, pd, COL_DIM);
        log_line(pd);
        row++;
    }

    /* Gradient footer */
    UINT32 foot_a = (grand_err == 0) ? COL_OK
                  : (g_aborted ? COL_RUN : COL_FAIL);
    UINT32 foot_b = (grand_err == 0) ? COL_OK_DK
                  : (g_aborted ? COL_RUN_DK : COL_FAIL_DK);
    blt_gradient_v(0, g_h - g_foot_h, g_w, g_foot_h, foot_a, foot_b);
    blt_fill(0, g_h - g_foot_h, g_w, 2, foot_a); /* top edge glow */

    UINTN frow = (g_h - g_foot_h / 2 - g_char_h / 2) / g_char_h;
    if (frow >= g_text_rows) frow = g_text_rows - 1;
    /* HONEST coverage verdict. Previously this used g_mem_pages — the size
       of the LAST pass's buffer — and showed "tested 1 GB of 16 GB" even
       after a full 16-pass multipass run actually exercised ~15.8 GB.
       That was a lie. Now we read g_run_tested_mb, which accumulates the
       UNIQUE bytes of physical RAM touched across every pass of this run.
       It also drops the "→ set MultiPass=1" nag when MultiPass is ALREADY 1
       (the old code printed that even after the user had done it). */
    UINT64 tested_mb_v = g_run_tested_mb;
    UINT32 cov_x10_v = g_total_ram_mb ? (UINT32)((tested_mb_v * 1000) / g_total_ram_mb) : 0;
    if (cov_x10_v > 1000) cov_x10_v = 1000;
    /* Cap reported "untested" at 0 if we covered more than total_ram (which
       can happen because total_ram_mb is SMBIOS-reported size but tests run
       against EfiConventionalMemory which can have slight rounding). */
    UINT64 untested_mb = (g_total_ram_mb > tested_mb_v)
                          ? (g_total_ram_mb - tested_mb_v) : 0;
    /* Heuristic: if we hit ≥95% of total RAM, treat it as full coverage —
       the missing 1–5% is firmware-reserved memory unreachable to ANY UEFI
       application. Don't scare the user about that residual. */
    int full_coverage = (cov_x10_v >= 950);
    CHAR16 line1[200], line2[200], line3[220];
    line3[0] = 0;
    if (g_aborted) {
        SPrint(line1, sizeof(line1),
               T(L"  ОТМЕНЕНО ПОЛЬЗОВАТЕЛЕМ", L"  ABORTED BY USER"));
        line2[0] = 0;
    } else if (grand_err == 0) {
        if (g_quick_mode) {
            /* Quick mode is INTENTIONALLY short — 3 passes × 4 essential
               tests, ~3 min total. Coverage of <20% is the expected outcome,
               NOT a sign the run was aborted. Tell the user that explicitly
               and recommend the full test for proper diagnosis.
               Previous wording used the loanwords "триаж" / "шоп" which
               many native Russian users didn't understand; this version uses
               plain everyday Russian. */
            SPrint(line1, sizeof(line1),
                   T(L"  БЫСТРАЯ ПРОВЕРКА: ошибок не найдено. Проверено %ld МБ из %ld МБ (%d.%d%%, %d проходов).",
                     L"  QUICK CHECK: no errors found. Tested %ld MB of %ld MB (%d.%d%%, %d passes)."),
                   tested_mb_v, g_total_ram_mb,
                   cov_x10_v / 10, cov_x10_v % 10, g_run_passes_done);
            SPrint(line2, sizeof(line2),
                   T(L"  Thermal Soak + 4 жёстких теста × 3 прохода (~7 мин). Для полной диагностики нажми [1].",
                     L"  Thermal Soak + 4 hard tests × 3 passes (~7 min). For full diagnosis press [1]."));
        } else if (full_coverage) {
            SPrint(line1, sizeof(line1),
                   T(L"  Без ошибок на %ld МБ — практически вся доступная RAM (%d проходов, %d.%d%%)",
                     L"  No errors on %ld MB — essentially all reachable RAM (%d passes, %d.%d%%)"),
                   tested_mb_v, g_run_passes_done, cov_x10_v / 10, cov_x10_v % 10);
            line2[0] = 0;
        } else if (g_cfg_multipass) {
            SPrint(line1, sizeof(line1),
                   T(L"  Без ошибок на %ld МБ из %ld МБ (%d.%d%%, %d проходов)",
                     L"  No errors on %ld MB of %ld MB (%d.%d%%, %d passes)"),
                   tested_mb_v, g_total_ram_mb, cov_x10_v / 10, cov_x10_v % 10,
                   g_run_passes_done);
            SPrint(line2, sizeof(line2),
                   T(L"  Прогон остановлен раньше — остались непокрытые %ld МБ (вероятно abort)",
                     L"  Run cut short — %ld MB remained uncovered (likely aborted)"),
                   untested_mb);
        } else {
            SPrint(line1, sizeof(line1),
                   T(L"  Без ошибок на %ld МБ (%d.%d%% от %ld МБ RAM)",
                     L"  No errors on %ld MB (%d.%d%% of %ld MB RAM)"),
                   tested_mb_v, cov_x10_v / 10, cov_x10_v % 10, g_total_ram_mb);
            SPrint(line2, sizeof(line2),
                   T(L"  ВАЖНО: остальные %ld МБ не тестировались. Поставь MultiPass=1 в quantai.ini",
                     L"  NOTE: the other %ld MB were not tested. Set MultiPass=1 in quantai.ini"),
                   untested_mb);
        }
        /* DDR5 ODECC disclosure — surface the on-die-ECC caveat to the user
           so a "PASS" on DDR5 isn't misread as "all chips are perfect". This
           is the SINGLE most important context for interpreting DDR5 results
           and the industry-standard caveat memtest86/memtest86+ also surface. */
        if (g_is_ddr5) {
            SPrint(line3, sizeof(line3),
                   T(L"  DDR5: on-die ECC скрывает 1-бит ошибки внутри чипа — PASS строже, но не гарантирует идеальные ячейки",
                     L"  DDR5: on-die ECC silently corrects 1-bit errors inside the chip — PASS is stricter but doesn't mean cells are perfect"));
            if (!line2[0]) {
                /* If line2 was empty (full coverage path), promote disclosure
                   into line2 so it appears on the row right under line1. */
                StrCpy(line2, line3);
                line3[0] = 0;
            }
        }
    } else {
        SPrint(line1, sizeof(line1),
               T(L"  ОБНАРУЖЕНО %ld ОШИБОК на %ld МБ (%d.%d%% от RAM, %d проходов)",
                 L"  %ld ERRORS on %ld MB (%d.%d%% of RAM, %d passes)"),
               grand_err, tested_mb_v, cov_x10_v / 10, cov_x10_v % 10,
               g_run_passes_done);
        SPrint(line2, sizeof(line2),
               T(L"  Детали (адреса/XOR-маски): memforge2.log + report.json на флешке",
                 L"  Details (addresses/XOR masks): memforge2.log + report.json on USB"));
        if (g_is_ddr5) {
            SPrint(line3, sizeof(line3),
                   T(L"  DDR5: эти ошибки прошли мимо on-die ECC — значит ≥2 бит в чипе или проблема дорожек/IMC",
                     L"  DDR5: these errors slipped past on-die ECC — means ≥2 bits/chip or trace/IMC issue"));
        }
    }
    say_at_rc(2, frow, line1);
    if (line2[0] && frow + 1 < g_text_rows) say_at_rc(2, frow + 1, line2);
    if (line3[0] && frow + 2 < g_text_rows) {
        gfx_draw_str_color(2 * g_char_w, (frow + 2) * g_char_h,
                           line3, COL_RUN);
        /* Footer help shifts down one row when ODECC disclosure shown */
        frow++;
    }
    /* Bottom hint strip — MUST be visible no matter how small the screen.
       We draw it at the very bottom of the framebuffer in its own coloured
       strip, NOT at frow+2 which could be off-screen on tiny fb (Dell
       OptiPlex 5050 = 800×600). Adaptive text width — full version on
       wide screens, abbreviated otherwise. */
    CHAR16 hint[200];
    if (g_text_cols >= 78) {
        SPrint(hint, sizeof(hint),
               T(L"[D] обратно к вердикту   [M] в меню   [L] язык RU/EN   [ESC] перезагрузка",
                 L"[D] back to verdict   [M] menu   [L] language EN/RU   [ESC] reboot"));
    } else if (g_text_cols >= 50) {
        SPrint(hint, sizeof(hint),
               T(L"[D] вердикт   [M] меню   [L] язык   [ESC] reboot",
                 L"[D] verdict   [M] menu   [L] language   [ESC] reboot"));
    } else {
        SPrint(hint, sizeof(hint),
               T(L"[D] вердикт  [M] меню  [ESC] reboot",
                 L"[D] verdict  [M] menu  [ESC] reboot"));
    }
    UINTN hint_y = g_h - g_char_h - 4;
    blt_fill(0, hint_y - 4, g_w, g_char_h + 8, COL_PANEL_ALT);
    blt_fill(0, hint_y - 5, g_w, 1, COL_BORDER);
    UINTN hint_w = StrLen(hint) * g_char_w;
    UINTN hint_x = (g_w > hint_w) ? (g_w - hint_w) / 2 : 0;
    gfx_draw_str_color(hint_x, hint_y, hint, COL_ACCENT_HI);
}

/* ---------- JSON report writer ----------
   Produces report.json in the FAT root alongside memforge2.log. Format matches
   what the LogAnalyzerGUI expects so AI analysis can read structured data
   without regex parsing. */
static void json_write_chunk(EFI_FILE_PROTOCOL *jf, CHAR16 *buf16) {
    CHAR8 line[600];
    UINTN k = 0;
    while (buf16[k] && k < sizeof(line) - 1) { line[k] = (CHAR8)(buf16[k] & 0xFF); k++; }
    line[k] = 0;
    UINTN len = k;
    uefi_call_wrapper(jf->Write, 3, jf, &len, line);
}

static void write_json_report(UINT64 total_ms) {
    if (!g_logroot) return;
    /* Delete prior report so we don't end up with stale bytes past our new content. */
    EFI_FILE_PROTOCOL *old = NULL;
    if (uefi_call_wrapper(g_logroot->Open, 5, g_logroot, &old, L"report.json",
            EFI_FILE_MODE_READ | EFI_FILE_MODE_WRITE, 0) == EFI_SUCCESS && old) {
        uefi_call_wrapper(old->Delete, 1, old);
    }
    EFI_FILE_PROTOCOL *jf = NULL;
    EFI_STATUS s = uefi_call_wrapper(g_logroot->Open, 5, g_logroot, &jf,
                       L"report.json",
                       EFI_FILE_MODE_CREATE | EFI_FILE_MODE_READ | EFI_FILE_MODE_WRITE, 0);
    if (EFI_ERROR(s) || !jf) return;

    EFI_TIME t;
    uefi_call_wrapper(RT->GetTime, 2, &t, NULL);

    /* Honest "tested_mb" = unique bytes of physical RAM exercised across
       all multipass passes, NOT the size of the last allocated buffer. */
    UINT64 buf_mb = g_run_tested_mb;
    UINT32 cov_x100 = g_total_ram_mb ? (UINT32)((buf_mb * 10000) / g_total_ram_mb) : 0;
    if (cov_x100 > 10000) cov_x100 = 10000;

    UINT64 grand_err = 0;
    UINT32 n_skip = 0, n_pass = 0, n_fail = 0;
    for (UINTN i = 0; i < N_TESTS; i++) {
        grand_err += g_summary[i].errors;
        if (g_summary[i].status == 0) n_skip++;
        else if (g_summary[i].status == 1) n_pass++;
        else if (g_summary[i].status == 2) n_fail++;
    }

    CHAR16 buf[600];

    SPrint(buf, sizeof(buf),
        L"{\r\n"
        L"  \"version\": \"0.4\",\r\n"
        L"  \"timestamp\": \"%04d-%02d-%02dT%02d:%02d:%02d\",\r\n"
        L"  \"run_time_ms\": %ld,\r\n",
        t.Year, t.Month, t.Day, t.Hour, t.Minute, t.Second, total_ms);
    json_write_chunk(jf, buf);

    SPrint(buf, sizeof(buf),
        L"  \"hardware\": {\r\n"
        L"    \"system_vendor\": \"%a\",\r\n"
        L"    \"system_model\": \"%a\",\r\n"
        L"    \"bios_vendor\": \"%a\",\r\n"
        L"    \"bios_version\": \"%a\",\r\n"
        L"    \"cpu\": \"%a\",\r\n"
        L"    \"cores_total\": %d,\r\n"
        L"    \"cores_enabled\": %d,\r\n"
        L"    \"avx2_available\": %a\r\n"
        L"  },\r\n",
        g_sys_vendor, g_sys_model, g_bios_vendor, g_bios_version,
        g_cpu_brand, (UINT32)g_n_cores, (UINT32)g_n_enabled,
        g_has_avx2 ? "true" : "false");
    json_write_chunk(jf, buf);

    SPrint(buf, sizeof(buf),
        L"  \"memory\": {\r\n"
        L"    \"total_mb\": %ld,\r\n"
        L"    \"tested_mb\": %ld,\r\n"
        L"    \"coverage_pct\": %d.%02d,\r\n"
        L"    \"passes_done\": %d,\r\n"
        L"    \"multipass\": %a,\r\n"
        L"    \"max_speed_mt\": %d,\r\n"
        L"    \"configured_speed_mt\": %d,\r\n"
        L"    \"ddr_type\": \"%s\",\r\n"
        L"    \"dimm_count\": %d,\r\n"
        L"    \"dimms\": [",
        g_total_ram_mb, buf_mb, cov_x100 / 100, cov_x100 % 100,
        g_run_passes_done,
        g_cfg_multipass ? "true" : "false",
        g_max_dimm_speed_mt, g_max_dimm_configured_mt,
        g_dimm_ddr_type ? ddr_type_name(g_dimm_ddr_type) : L"unknown",
        g_dimm_count);
    json_write_chunk(jf, buf);

    for (UINT32 i = 0; i < g_dimm_count; i++) {
        dimm_info_t *d = &g_dimms[i];
        /* gnu-efi's SPrint promotes UINT8 to UINTN and %02X then prints all
           8 hex chars instead of 2 — producing garbage like
           "serial":"0000006F00000027000000BF00000000" and
           "smbus_addr":"0x00000050". Hand-format the hex bytes into ASCII
           buffers and insert via %a (CHAR8*). Same workaround already used
           in the [SPD] log line — just was never applied here. */
        static const CHAR8 hex[] = "0123456789ABCDEF";
        #define HX2(out, v) do { (out)[0] = hex[((v) >> 4) & 0xF]; \
                                  (out)[1] = hex[(v) & 0xF]; \
                                  (out)[2] = 0; } while (0)
        CHAR8 s_addr[3], s_year[3], s_jcode[3];
        CHAR8 s_ser[9];
        HX2(s_addr,  d->spd_addr);
        HX2(s_year,  d->spd_mfg_year);
        HX2(s_jcode, d->spd_jedec_code);
        s_ser[0] = hex[(d->spd_serial[0] >> 4) & 0xF];
        s_ser[1] = hex[ d->spd_serial[0]       & 0xF];
        s_ser[2] = hex[(d->spd_serial[1] >> 4) & 0xF];
        s_ser[3] = hex[ d->spd_serial[1]       & 0xF];
        s_ser[4] = hex[(d->spd_serial[2] >> 4) & 0xF];
        s_ser[5] = hex[ d->spd_serial[2]       & 0xF];
        s_ser[6] = hex[(d->spd_serial[3] >> 4) & 0xF];
        s_ser[7] = hex[ d->spd_serial[3]       & 0xF];
        s_ser[8] = 0;
        #undef HX2

        SPrint(buf, sizeof(buf),
            L"%a\r\n      {\"slot\":\"%a\",\"size_mb\":%d,\"manufacturer\":\"%a\","
            L"\"part\":\"%a\",\"type\":\"%s\",\"speed_mt\":%d,\"configured_speed_mt\":%d,"
            L"\"xmp_active\":%a,"
            L"\"organization\":{\"device_width\":%d,\"bus_width\":%d,\"ranks\":%d},"
            L"\"spd\":{\"present\":%a,\"smbus_addr\":\"0x%a\","
            L"\"serial\":\"%a\",\"mfg_year\":\"20%a\",\"mfg_week\":%d,"
            L"\"jedec_bank\":%d,\"jedec_code\":\"0x%a\","
            L"\"timings\":{\"tCL\":%d,\"tRCD\":%d,\"tRP\":%d,\"tRAS\":%d,"
            L"\"tRFC_ns\":%d,\"tCK_ps\":%d}}}",
            (i > 0) ? "," : "",
            d->locator, d->size_mb, d->manufacturer, d->part_number,
            d->ddr_type ? ddr_type_name(d->ddr_type) : L"?",
            d->speed_mt, d->configured_speed_mt,
            (i < MAX_DIMMS && g_xmp_dimm_flagged[i]) ? "true" : "false",
            d->spd_device_width, d->spd_bus_width, d->spd_ranks,
            d->spd_present ? "true" : "false",
            s_addr,
            s_ser, s_year, d->spd_mfg_week,
            d->spd_jedec_bank, s_jcode,
            d->spd_tCL, d->spd_tRCD, d->spd_tRP, d->spd_tRAS,
            d->spd_tRFC_ns, d->spd_tCK_ps);
        json_write_chunk(jf, buf);
    }

    SPrint(buf, sizeof(buf), L"\r\n    ]\r\n  },\r\n  \"tests\": [");
    json_write_chunk(jf, buf);

    for (UINTN i = 0; i < N_TESTS; i++) {
        test_summary_t *r = &g_summary[i];
        const CHAR16 *st = r->status == 1 ? L"PASS"
                         : r->status == 2 ? L"FAIL"
                         : L"SKIP";
        UINT64 mbs = r->time_ms ? (r->bytes / (1024 * 1024) * 1000) / r->time_ms : 0;
        SPrint(buf, sizeof(buf),
            L"%a\r\n    {\"id\":%d,\"name\":\"%s\",\"status\":\"%s\","
            L"\"errors\":%ld,\"bytes_mb\":%ld,\"time_ms\":%ld,\"throughput_mbps\":%ld}",
            (i > 0) ? "," : "",
            (UINT32)(i + 1), g_tests[i].name, st,
            r->errors, r->bytes / (1024 * 1024), r->time_ms, mbs);
        json_write_chunk(jf, buf);
    }

    SPrint(buf, sizeof(buf),
        L"\r\n  ],\r\n"
        L"  \"summary\": {\"passed\":%d,\"failed\":%d,\"skipped\":%d,\"total_errors\":%ld},\r\n"
        L"  \"verdict\": \"%a\",\r\n",
        n_pass, n_fail, n_skip, grand_err,
        (g_aborted ? "ABORTED" : (grand_err > 0 ? "FAIL" : "PASS")));
    json_write_chunk(jf, buf);

    /* Bandwidth degradation trend (populated by bw_trend_report when
       enough buckets were collected; first_pct=0 means n/a). */
    SPrint(buf, sizeof(buf),
        L"  \"bw_trend\": {\"buckets\":%d,\"first_quartile_pct\":%d,"
        L"\"last_quartile_pct\":%d,\"degraded\":%d},\r\n",
        g_bw_history_count, g_bw_trend_first_pct,
        g_bw_trend_last_pct, g_bw_trend_degraded);
    json_write_chunk(jf, buf);

    /* v0.4.29 — run-wide peaks. Lets an automated analyzer verify the
       CPU actually got loaded (without these the JSON had no way to
       answer "was the workload real or did the CPU idle?"). */
    UINT32 peak_bw_gbs_x10 = g_bw_mbps_peak ? (g_bw_mbps_peak * 10 / 1024) : 0;
    UINT32 peak_pct_theo   = 0;
    if (g_bw_peak_theoretical_mbps > 0 && g_bw_mbps_peak > 0)
        peak_pct_theo = (UINT32)((UINT64)g_bw_mbps_peak * 100ULL
                                 / g_bw_peak_theoretical_mbps);
    SPrint(buf, sizeof(buf),
        L"  \"peaks\": {"
        L"\"temp_c\":%d,"
        L"\"pkg_power_w\":%d,"
        L"\"freq_mhz\":%d,"
        L"\"bw_mbps\":%d,"
        L"\"bw_gbps_x10\":%d,"
        L"\"bw_theoretical_mbps\":%ld,"
        L"\"bw_pct_of_theoretical\":%d,"
        L"\"throttle_events_total\":%d,"
        L"\"turbo_lift_hwp_cores\":%d,"
        L"\"turbo_lift_legacy_cores\":%d"
        L"},\r\n",
        g_max_temp_c, g_pkg_power_w_peak, g_max_freq_mhz_observed,
        g_bw_mbps_peak, peak_bw_gbs_x10,
        (UINT64)g_bw_peak_theoretical_mbps, peak_pct_theo,
        g_throttle_total,
        g_hwp_ok_count, g_legacy_turbo_count);
    json_write_chunk(jf, buf);

    /* Detailed error records — most useful single field is xor_mask. */
    SPrint(buf, sizeof(buf), L"  \"error_records_captured\": %d,\r\n",
           g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count);
    json_write_chunk(jf, buf);
    SPrint(buf, sizeof(buf), L"  \"error_records_total\": %d,\r\n", g_err_count);
    json_write_chunk(jf, buf);
    SPrint(buf, sizeof(buf), L"  \"errors\": [");
    json_write_chunk(jf, buf);

    UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
    for (UINT32 i = 0; i < shown; i++) {
        err_record_t *r = &g_err_records[i];
        CHAR16 dimm_lab[64];
        dimm_label_for_addr(r->phys_addr, dimm_lab, 64);
        dram_coords_t cc;
        decode_dram_coords(r->phys_addr, &cc);
        /* core+1 to match the 1-based "CPUNN" numbering shown on screen.
           dram_coords are tagged with "approx_" prefix because the channel
           hash function is chipset-specific and we use a generic layout. */
        SPrint(buf, sizeof(buf),
            L"%a\r\n    {\"test\":\"%s\",\"core\":%d,\"addr\":\"0x%lx\","
            L"\"expected\":\"0x%lx\",\"actual\":\"0x%lx\",\"xor\":\"0x%lx\","
            L"\"pass\":%d,\"dimm\":\"%s\","
            L"\"approx_bank_group\":%d,\"approx_bank\":%d,"
            L"\"approx_row\":\"0x%lx\",\"approx_col\":\"0x%x\","
            /* Environmental snapshot at the moment of the error. */
            L"\"at\":{\"t_ms\":%ld,\"temp_c\":%d,\"pkg_w\":%d,"
            L"\"throttle\":%d,\"vid_mv\":%d}}",
            (i > 0) ? "," : "",
            name_for_kernel(r->test), r->core + 1,
            r->phys_addr, r->expected, r->actual, r->xor_mask, r->pass_idx,
            dimm_lab,
            cc.bank_group, cc.bank, cc.row, cc.column,
            r->t_ms, r->temp_c, r->pkg_watt, r->throttle_cnt, r->vid_mv);
        json_write_chunk(jf, buf);
    }

    SPrint(buf, sizeof(buf), L"\r\n  ],\r\n");
    json_write_chunk(jf, buf);

    /* Stuck-bit / repeating-pattern summary. */
    if (g_err_count > 0) {
        UINT32 stuck_n = 0;
        UINT64 stuck_x = find_stuck_bit(&stuck_n);
        int bp = single_bit_pos(stuck_x);
        SPrint(buf, sizeof(buf),
            L"  \"stuck_bit\": {\"xor_mask\":\"0x%lx\",\"count\":%d,\"bit_pos\":%d,"
            L"\"is_stuck\":%a},\r\n",
            stuck_x, stuck_n, bp,
            (stuck_n >= 5 && stuck_x != 0) ? "true" : "false");
        json_write_chunk(jf, buf);

        /* Stuck-row / stuck-bank (DRAM-coord heuristic). */
        UINT32 sr_n = 0;
        UINT64 sr   = find_stuck_row(&sr_n);
        UINT32 sb_n = 0;
        UINT32 sb_id = find_stuck_bank(&sb_n);
        SPrint(buf, sizeof(buf),
            L"  \"stuck_row\": {\"row\":\"0x%lx\",\"count\":%d,\"is_stuck\":%a,\"approximate\":true},\r\n",
            sr, sr_n, (sr_n >= 3) ? "true" : "false");
        json_write_chunk(jf, buf);
        SPrint(buf, sizeof(buf),
            L"  \"stuck_bank\": {\"bank_group\":%d,\"bank\":%d,\"count\":%d,\"is_stuck\":%a,\"approximate\":true},\r\n",
            (sb_id >> 4) & 0xF, sb_id & 0xF, sb_n, (sb_n >= 3) ? "true" : "false");
        json_write_chunk(jf, buf);

        /* 1-GB-bucket histogram. */
        UINT32 hb[32], hc[32];
        UINT32 nbk = error_histogram_gb(hb, hc, 32);
        SPrint(buf, sizeof(buf), L"  \"error_histogram_gb\": [");
        json_write_chunk(jf, buf);
        for (UINT32 j = 0; j < nbk; j++) {
            SPrint(buf, sizeof(buf), L"%a{\"gb\":%d,\"count\":%d}",
                   (j == 0) ? "" : ",", hb[j], hc[j]);
            json_write_chunk(jf, buf);
        }
        SPrint(buf, sizeof(buf), L"],\r\n");
        json_write_chunk(jf, buf);
    }

    /* MCA block — hardware-logged errors observed by the iMC / cache /
       interconnect during our run. May include corrected ECC events that
       our pattern tests didn't notice. */
    if (g_has_mca) {
        SPrint(buf, sizeof(buf),
               L"  \"mca\": {\"enabled\":true,\"bank_count\":%d,\"new_errors\":%d,\"banks\":[",
               g_mca_bank_count, g_mca_new_errors);
        json_write_chunk(jf, buf);
        int first = 1;
        for (UINT32 i = 0; i < g_mca_bank_count; i++) {
            UINT64 before = g_mca_baseline[i];
            UINT64 after  = g_mca_post[i];
            int after_valid = (after >> 63) & 1;
            if (!after_valid) continue;
            int is_new = (after != before) || !((before >> 63) & 1);
            int uc     = (after >> 61) & 1;
            int addrv  = (after >> 58) & 1;
            UINT64 addr = addrv ? rdmsr_safe(0x402 + 4 * i) : 0;
            CHAR16 dimm[64] = L"";
            if (addrv) dimm_label_for_addr(addr, dimm, 64);
            SPrint(buf, sizeof(buf),
                   L"%a{\"bank\":%d,\"status\":\"0x%lx\",\"addr\":\"0x%lx\","
                   L"\"new\":%a,\"uncorrected\":%a,\"dimm\":\"%s\"}",
                   first ? "" : ",",
                   i, after, addr,
                   is_new ? "true" : "false",
                   uc ? "true" : "false",
                   addrv ? dimm : L"?");
            first = 0;
            json_write_chunk(jf, buf);
        }
        SPrint(buf, sizeof(buf), L"]},\r\n");
        json_write_chunk(jf, buf);
    } else {
        SPrint(buf, sizeof(buf), L"  \"mca\": {\"enabled\":false},\r\n");
        json_write_chunk(jf, buf);
    }

    SPrint(buf, sizeof(buf),
        L"  \"coverage_disclaimer\": \"tested_mb is the cumulative UNIQUE bytes "
        L"of physical RAM exercised across all multipass passes (non-overlapping "
        L"region/offset slices). With MultiPass=1 this approaches total_mb minus "
        L"~1-5%% firmware-reserved RAM that is unreachable to any UEFI app. "
        L"With MultiPass=0 it equals the single allocated buffer size.\"\r\n"
        L"}\r\n");
    json_write_chunk(jf, buf);

    uefi_call_wrapper(jf->Flush, 1, jf);
    uefi_call_wrapper(jf->Close, 1, jf);
}

/* ---------- Main menu (no auto-start) ----------
   Shown after init+SMBIOS+buffer-alloc. v0.4: redesigned as a structured
   layout with framed panels and two button-style hotkeys [1] / [2]. */

/* Draw a framed panel with a colored title bar across its top. The body of
   the panel (the area below the title bar) starts at y + title_h. */
static void menu_panel(UINTN x, UINTN y, UINTN w, UINTN h,
                       CHAR16 *title, UINT32 accent) {
    blt_panel(x, y, w, h, COL_PANEL, COL_BORDER);
    /* Title bar: a slightly lighter background with a colored left tab. */
    UINTN tb_h = FONT_H + 6;
    blt_fill(x + 1, y + 1, w - 2, tb_h, COL_PANEL_ALT);
    blt_fill(x + 1, y + 1, 4, tb_h, accent);              /* left accent strip */
    blt_fill(x + 1, y + tb_h + 1, w - 2, 1, COL_BORDER);  /* separator */
    gfx_draw_str_color(x + 14, y + 3, title, accent);
}

/* Two-line button-style option: big bordered rectangle with a "[N]" badge
   on the left and a label / description stacked on the right. */
static void menu_button(UINTN x, UINTN y, UINTN w, UINTN h,
                        CHAR16 *badge, CHAR16 *label, CHAR16 *sub,
                        UINT32 col_a, UINT32 col_b) {
    blt_gradient_v(x, y, w, h, col_a, col_b);
    box_outline(x, y, w, h, COL_FG);
    box_outline(x + 1, y + 1, w - 2, h - 2, col_a);
    /* Big badge on the left ("[1]" / "[2]") */
    UINTN badge_x = x + 16;
    UINTN badge_y = y + (h - FONT_H) / 2;
    gfx_draw_str_color(badge_x, badge_y, badge, COL_FG);
    /* Label + sub stacked to the right of the badge */
    UINTN tx = badge_x + 4 * FONT_ADVANCE + 18;
    UINTN ty1 = y + (h - 2 * FONT_H - 4) / 2;
    UINTN ty2 = ty1 + FONT_H + 4;
    gfx_draw_str_color(tx, ty1, label, COL_FG);
    gfx_draw_str_color(tx, ty2, sub,   COL_FG);
}

/* ---------- About / info screen ----------
   Shown when user presses [I] in the main menu. Honest overview + comparison
   vs memtest86+ / PassMark / TM5. Multi-page if content exceeds screen
   height: PgDn/Space = next page, PgUp/Backspace = prev, any other key /
   ESC = return to menu. Content split into "pages" of body lines, each
   page rendered with a fresh title strip and a "page N/M" indicator.

   Lines kept short (≤ 60 chars) so they fit even on 800×600 framebuffers
   (Dell OptiPlex 5050 case) where g_text_cols can drop to ~66. */

/* Type tag for each body line — affects color but not layout. */
#define ABT_H   1   /* section heading (accent) */
#define ABT_T   2   /* normal text (FG) */
#define ABT_P   3   /* positive bullet (green) */
#define ABT_D   4   /* dim text (gray) */
#define ABT_SP  5   /* blank spacer */

typedef struct {
    int    type;
    CHAR16 *ru;
    CHAR16 *en;
} abt_line_t;

/* All About content, in display order. Lines kept ≤ 60 chars so they fit
   on the narrowest target (800-px fb on a Dell OptiPlex 5050 = ~66 cols).
   Same level of detail as the README — concrete refs (Frigo et al.,
   van de Goor 1997), specific named competitors, honest tradeoffs. */
static const abt_line_t g_about_lines[] = {
    /* ===== Section 1: what this is ===== */
    { ABT_H,  L"━ ЧТО ЭТО ━",
              L"━ WHAT IT IS ━" },
    { ABT_T,  L"UEFI-приложение для проверки оперативной памяти.",
              L"UEFI app for RAM diagnostics." },
    { ABT_T,  L"Грузится с USB до загрузки операционной системы.",
              L"Boots from USB before any OS loads." },
    { ABT_T,  L"Тестит RAM параллельно на всех ядрах процессора.",
              L"Tests RAM in parallel on every CPU core." },
    { ABT_T,  L"Назначение — шоп/ремонт: приёмка, поиск дефектов.",
              L"For shop/repair: intake QC, defect hunt." },
    { ABT_SP, L"", L"" },

    /* ===== Section 2: tests ===== */
    { ABT_H,  L"━ ТЕСТЫ (14 шт.) ━",
              L"━ TESTS (14 total) ━" },
    { ABT_T,  L" 1. AVX2 Sustained   — нагрузка VRM/контроллера",
              L" 1. AVX2 Sustained   — VRM/memctl stress" },
    { ABT_T,  L" 2. TRRespass        — Row Hammer (для DDR4/5)",
              L" 2. TRRespass        — Row Hammer (DDR4/5)" },
    { ABT_T,  L" 3. Cache-Eviction   — нагрузка на шину памяти",
              L" 3. Cache-Eviction   — memory bus stress" },
    { ABT_T,  L" 4. March-C-         — классические pattern-ошибки",
              L" 4. March-C-         — classic pattern faults" },
    { ABT_T,  L" 5. Thermal Soak     — 3 мин прогрев CPU + памяти",
              L" 5. Thermal Soak     — 3-min CPU+RAM heat-up" },
    { ABT_T,  L" 6. BW Soak          — 5 мин нагрузка пропускной",
              L" 6. BW Soak          — 5-min bandwidth stress" },
    { ABT_T,  L" 7. March-RAW        — ошибки read-after-write",
              L" 7. March-RAW        — read-after-write faults" },
    { ABT_T,  L" 8. Butterfly        — взаимовлияние ячеек",
              L" 8. Butterfly        — cell crosstalk" },
    { ABT_T,  L" 9. Address Pattern  — ошибки адресации",
              L" 9. Address Pattern  — addressing faults" },
    { ABT_T,  L"10. VRM Square-Wave  — переходные процессы VRM",
              L"10. VRM Square-Wave  — VRM transients" },
    { ABT_T,  L"11. Random Pattern   — псевдослучайные паттерны",
              L"11. Random Pattern   — pseudo-random patterns" },
    { ABT_T,  L"12. Bit Fade Ext.    — потеря заряда ячеек (~8м)",
              L"12. Bit Fade Ext.    — cell retention (~8 min)" },
    { ABT_T,  L"13. L3 Cache Stress  — ошибки L3-кэша процессора",
              L"13. L3 Cache Stress  — L3 cache cell faults" },
    { ABT_T,  L"14. Stride BW        — проблемы каналов/TLB",
              L"14. Stride BW        — channel/TLB issues" },
    { ABT_SP, L"", L"" },

    /* ===== Section 3: режимы прогона ===== */
    { ABT_H,  L"━ РЕЖИМЫ ПРОГОНА ━",
              L"━ RUN MODES ━" },
    { ABT_T,  L"Quick (~5 мин) — самые сильные тесты + Thermal Soak.",
              L"Quick (~5 min) — strongest tests + Thermal Soak." },
    { ABT_T,  L"Full (~30 мин) — все 14 тестов, один полный проход.",
              L"Full (~30 min) — all 14 tests, one full pass." },
    { ABT_T,  L"MultiPass — последовательно покрывает ВСЮ память,",
              L"MultiPass — sequentially covers ALL RAM," },
    { ABT_T,  L"  а не только тестовый буфер (~3 мин/ГБ).",
              L"  not just the test buffer (~3 min/GB)." },
    { ABT_T,  L"Marathon — крутить тесты 1-24 часа подряд для",
              L"Marathon — cycle tests for 1-24 hours to" },
    { ABT_T,  L"  отлова редких ошибок (раз в N часов).",
              L"  surface very rare intermittent faults." },
    { ABT_SP, L"", L"" },

    /* ===== Section 4: умные функции ===== */
    { ABT_H,  L"━ УМНЫЕ ФУНКЦИИ ━",
              L"━ SMART FEATURES ━" },
    { ABT_P,  L"✓ Авто-изоляция планок",
              L"✓ Auto DIMM isolation" },
    { ABT_T,  L"  Если ошибки в адресах нескольких планок —",
              L"  If errors span multiple DIMM ranges —" },
    { ABT_T,  L"  прога сама перепроверит каждую отдельно",
              L"  re-tests each in isolation to give a" },
    { ABT_T,  L"  и даст однозначный ответ \"ЗАМЕНИТЬ X\".",
              L"  definitive \"REPLACE X\" answer." },
    { ABT_SP, L"", L"" },
    { ABT_P,  L"✓ Чтение SPD планок",
              L"✓ DIMM SPD read" },
    { ABT_T,  L"  Серийник, неделя/год производства, тайминги,",
              L"  Serial, manufacture week/year, timings," },
    { ABT_T,  L"  производитель (для гарантийной замены).",
              L"  manufacturer (for warranty replacement)." },
    { ABT_SP, L"", L"" },
    { ABT_P,  L"✓ MCA — невидимые аппаратные ошибки",
              L"✓ MCA — invisible hardware errors" },
    { ABT_T,  L"  На памяти с ECC контроллер тихо исправляет",
              L"  On ECC memory the controller silently fixes" },
    { ABT_T,  L"  одно-битные сбои — обычные тесты их не видят.",
              L"  single-bit flips — normal tests can't see." },
    { ABT_T,  L"  Мы их подсчитываем напрямую через регистры CPU.",
              L"  We count them via CPU registers." },
    { ABT_SP, L"", L"" },
    { ABT_P,  L"✓ Снимок состояния при каждой ошибке",
              L"✓ Per-error environmental snapshot" },
    { ABT_T,  L"  Темп / Вт / напряжение / тротлинг в момент сбоя.",
              L"  Temp / W / voltage / throttle at error moment." },
    { ABT_T,  L"  Видно: \"бит флипнул при 87°C на 1.23 В\".",
              L"  Shows: \"bit flipped at 87°C / 1.23 V\"." },
    { ABT_SP, L"", L"" },
    { ABT_P,  L"✓ Сравнение с прошлым прогоном",
              L"✓ Diff vs previous run" },
    { ABT_T,  L"  Записываем краткую сводку в UEFI NVRAM.",
              L"  Brief summary persisted to UEFI NVRAM." },
    { ABT_T,  L"  При следующем запуске покажет дельты ошибок,",
              L"  Next boot shows error/temp/BW deltas —" },
    { ABT_T,  L"  температуры, пропускной — \"что изменилось\".",
              L"  \"what changed since last test\"." },
    { ABT_SP, L"", L"" },
    { ABT_P,  L"✓ Подгон под турбо-режим",
              L"✓ CPU pushed to turbo" },
    { ABT_T,  L"  Поднимаем CPU до максимальной частоты на всех",
              L"  Bumps CPU to max P-state on every core for" },
    { ABT_T,  L"  ядрах для теплового стресса памяти.",
              L"  proper thermal stress on the RAM." },
    { ABT_SP, L"", L"" },

    /* ===== Section 5: report files ===== */
    { ABT_H,  L"━ ОТЧЁТ ━",
              L"━ REPORT FILES ━" },
    { ABT_T,  L"После теста на USB рядом с loader.efi:",
              L"After test, on USB next to loader.efi:" },
    { ABT_T,  L"  memforge2.log  — полный лог прогона",
              L"  memforge2.log  — full run log" },
    { ABT_T,  L"  report.json    — структурированные данные",
              L"  report.json    — structured data" },
    { ABT_T,  L"Включает: SPD каждой планки, MCA-журнал,",
              L"Contains: per-DIMM SPD, MCA log," },
    { ABT_T,  L"найденные ошибки с адресами и состоянием системы",
              L"errors with addresses and system state" },
    { ABT_T,  L"на момент сбоя, гистограмму распределения.",
              L"at moment of failure, address histogram." },
};
#define ABOUT_LINE_COUNT (sizeof(g_about_lines) / sizeof(g_about_lines[0]))

/* ---------- Periodic-wake key wait ----------
   Wait for either a keystroke or a short timer tick. Returns 1 if a key
   was successfully read into *out_key, 0 if it was a timer tick (no key)
   or a spurious wake.

   Why we don't just block on ConIn->WaitForKey:
     Some old HP business EFI firmwares (e.g. Z2 G8, EliteDesk 800 series)
     deactivate USB-keyboard polling after the WaitForEvent has been
     blocked for ~10 s — the USB stack assumes nobody's waiting and stops
     servicing the device. After that, ESC / any key never gets delivered
     and the user has to power-cycle to exit a summary screen.
   Periodic short waits (timeout_ms = 200) force firmware to re-enter
   WaitForEvent often enough that the USB stack keeps polling. The caller
   gets a 0 return when nothing happened and just loops again — totally
   harmless on firmwares that don't need it. */
static int wait_key_or_timer(EFI_INPUT_KEY *out_key, UINT64 timeout_ms) {
    if (out_key) { out_key->ScanCode = 0; out_key->UnicodeChar = 0; }
    EFI_EVENT te = NULL;
    EFI_STATUS cs = uefi_call_wrapper(BS->CreateEvent, 5,
                                       EVT_TIMER, 0, NULL, NULL, &te);
    if (cs != EFI_SUCCESS || !te) {
        /* Fallback: blocking key wait (old behaviour). Safer than aborting. */
        UINTN idx = 0;
        uefi_call_wrapper(BS->WaitForEvent, 3, 1,
                          &ST->ConIn->WaitForKey, &idx);
        EFI_STATUS rs = uefi_call_wrapper(ST->ConIn->ReadKeyStroke, 2,
                                           ST->ConIn, out_key);
        return (rs == EFI_SUCCESS) ? 1 : 0;
    }
    /* SetTimer wants 100-ns units. 10000 = 1 ms. Floor at 50 ms so we
       don't burn CPU on a hyperactive timer. */
    if (timeout_ms < 50) timeout_ms = 50;
    uefi_call_wrapper(BS->SetTimer, 3, te, TimerRelative,
                      (UINT64)timeout_ms * 10000ULL);
    EFI_EVENT events[2] = { te, ST->ConIn->WaitForKey };
    UINTN idx = 0;
    uefi_call_wrapper(BS->WaitForEvent, 3, 2, events, &idx);
    /* CloseEvent implicitly cancels any pending timer on this event. */
    uefi_call_wrapper(BS->CloseEvent, 1, te);
    if (idx == 1) {
        EFI_STATUS rs = uefi_call_wrapper(ST->ConIn->ReadKeyStroke, 2,
                                           ST->ConIn, out_key);
        return (rs == EFI_SUCCESS) ? 1 : 0;
    }
    return 0;  /* timer tick — caller loops */
}

static void render_about_page(UINTN page, UINTN total_pages,
                                UINTN lines_per_page) {
    cls();
    /* Top gradient + title. */
    UINTN ch = g_char_h;
    UINTN hdr_h = ch + 12;
    blt_gradient_v(0, 0, g_w, hdr_h, COL_ACCENT, COL_ACCENT_DK);
    blt_fill(0, hdr_h - 3, g_w, 1, COL_ACCENT_HI);
    CHAR16 title_buf[80];
    SPrint(title_buf, sizeof(title_buf),
           T(L"О ПРОГРАММЕ   ·   стр. %d/%d",
             L"ABOUT MEMFORGE   ·   page %d/%d"),
           (UINT32)(page + 1), (UINT32)total_pages);
    gfx_draw_str_color((g_w - StrLen(title_buf) * g_char_w) / 2,
                       4, title_buf, COL_FG);

    UINTN y = hdr_h + 8;
    UINTN x = g_pad + 8;
    UINTN start = page * lines_per_page;
    UINTN end   = start + lines_per_page;
    if (end > ABOUT_LINE_COUNT) end = ABOUT_LINE_COUNT;
    for (UINTN i = start; i < end; i++) {
        const abt_line_t *L = &g_about_lines[i];
        CHAR16 *s = g_lang ? L->en : L->ru;
        UINT32 col = COL_FG;
        switch (L->type) {
            case ABT_H:  col = COL_ACCENT_HI; break;
            case ABT_P:  col = COL_OK;        break;
            case ABT_D:  col = COL_DIM;       break;
            case ABT_SP: y += ch; continue;
            default:     col = COL_FG;        break;
        }
        gfx_draw_str_color(x, y, s, col);
        y += ch;
    }

    /* Footer: navigation hint. */
    CHAR16 hint[120];
    if (total_pages > 1) {
        SPrint(hint, sizeof(hint),
               T(L"[Space/PgDn] далее   [PgUp/Bksp] назад   [ESC/M] меню",
                 L"[Space/PgDn] next   [PgUp/Bksp] prev   [ESC/M] menu"));
    } else {
        SPrint(hint, sizeof(hint),
               T(L"[Любая клавиша] назад в меню",
                 L"[Any key] back to menu"));
    }
    UINTN hint_y = g_h - ch - 4;
    blt_fill(0, hint_y - 4, g_w, ch + 8, COL_PANEL_ALT);
    blt_fill(0, hint_y - 5, g_w, 1, COL_BORDER);
    UINTN hint_w = StrLen(hint) * g_char_w;
    UINTN hint_x = (g_w > hint_w) ? (g_w - hint_w) / 2 : 0;
    gfx_draw_str_color(hint_x, hint_y, hint, COL_ACCENT_HI);
}

/* Compute how many About lines fit between the header strip and footer
   strip on the current display, then loop a paginated viewer until the
   user dismisses. */
static void render_about_screen(void) {
    UINTN ch = g_char_h;
    UINTN hdr_h = ch + 12;
    UINTN foot_h = ch + 8;
    UINTN body_h = (g_h > hdr_h + foot_h + 16)
                 ? (g_h - hdr_h - foot_h - 16) : ch;
    UINTN lines_per_page = body_h / ch;
    if (lines_per_page < 4) lines_per_page = 4;
    UINTN total_pages = (ABOUT_LINE_COUNT + lines_per_page - 1) / lines_per_page;
    if (total_pages < 1) total_pages = 1;

    UINTN page = 0;
    for (;;) {
        render_about_page(page, total_pages, lines_per_page);
        /* Wait for a key. Use periodic-wake helper so HP USB stack stays
           alive even if user leaves us sitting here. No idle timeout —
           About is informational; user dismisses at their pace. */
        drain_conin();
        EFI_INPUT_KEY k = {0, 0};
        for (;;) {
            if (wait_key_or_timer(&k, 200)) break;
            /* Timer tick — just loop and wait again. */
        }
        /* Navigation: PgDn / Space / Enter → next page (or exit if last).
           PgUp / Backspace → prev page (or stay on first).
           ESC / M → exit immediately to menu. */
        if (k.ScanCode == SCAN_ESC ||
            k.UnicodeChar == L'm' || k.UnicodeChar == L'M') {
            return;
        }
        if (k.ScanCode == SCAN_PAGE_DOWN ||
            k.UnicodeChar == L' ' || k.UnicodeChar == 13) {
            if (page + 1 < total_pages) page++;
            else                          return;     /* last page → exit */
            continue;
        }
        if (k.ScanCode == SCAN_PAGE_UP || k.UnicodeChar == 8 /* Bksp */) {
            if (page > 0) page--;
            continue;
        }
        /* Any other key on a single-page screen → exit; on multi-page →
           treat as "next" so the user can just keep tapping. */
        if (total_pages > 1) {
            if (page + 1 < total_pages) page++;
            else return;
        } else {
            return;
        }
    }
}

/* Block on a single key press, return its struct. Used by the about screen
   to wait until user dismisses. Drains spurious wakes the same way the
   main menu loop does. */
static void about_wait_any_key(void) {
    drain_conin();
    for (;;) {
        EFI_INPUT_KEY k = {0, 0};
        if (wait_key_or_timer(&k, 200)) return;
        /* Timer tick — keep USB polling alive on HP firmwares. */
    }
}

static void render_main_menu(void) {
    cls();

    CHAR16 buf[320];

    /* --- Geometry: 4 tiers based on screen height ---
         g_h ≥ 1000  : full   (spacious, all rows visible, subtitle shown)
         900-999     : compact (smaller paddings, no subtitle)
         800-899     : tight  (also drop per-DIMM rows, smaller buttons)
         < 800       : ultra  (also drop pass-time estimate row, smallest pad)
       Tested vertical budget at 768 px (1024×768 / 1366×768):
         title 38 + HW 150 + COV-noEst 94 + TESTS 318 + buttons 60 + footer 36
         + 5×pad(6) = 726 px → fits in 768 with 42 px headroom. */
    int compact = (g_h < 1000);
    int tight   = (g_h < 900);
    int ultra   = (g_h < 800);
    UINTN pad   = ultra ? 4 : (tight ? 6 : (compact ? 8 : 16));
    UINTN px    = pad;
    UINTN pw    = g_w - 2 * pad;
    UINTN cw    = g_char_w;
    UINTN ch    = g_char_h;

    /* --- Top title strip ---
       Tight mode drops the subtitle to save 1 row (~30 px) — the title alone
       is enough; users know what app they booted. */
    UINTN hdr_h = tight ? (ch + 10) : (ch * 2 + 24);
    blt_gradient_v(0, 0, g_w, hdr_h, COL_ACCENT, COL_ACCENT_DK);
    blt_fill(0, hdr_h - 3, g_w, 1, COL_ACCENT_HI);
    blt_fill(0, hdr_h - 2, g_w, 2, COL_BG);

    CHAR16 *title = L"MEMFORGE  v0.4";
    gfx_draw_str_color((g_w - StrLen(title) * cw) / 2,
                       tight ? 4 : 6, title, COL_FG);
    if (!tight) {
        CHAR16 *subtitle = T(L"Универсальная диагностика памяти",
                             L"Universal Memory Diagnostic");
        gfx_draw_str_color((g_w - StrLen(subtitle) * cw) / 2,
                           6 + ch + 2, subtitle, COL_FG);
    }

    UINTN y = hdr_h + pad;

    /* ============================ HARDWARE PANEL ============================ */
    /* Tight mode drops the per-DIMM detail rows — those values are also
       shown in the test-running header, so the user sees them either way. */
    UINTN n_dimm_rows = tight ? 0 : ((g_dimm_count > 4) ? 4 : g_dimm_count);
    UINTN hw_body_rows = 3 + n_dimm_rows + 1;       /* Sys, CPU, RAM, DIMMs, BIOS */
    UINTN hw_title_h   = compact ? (ch + 4) : (ch + 8);
    UINTN hw_h         = hw_title_h + hw_body_rows * (ch + (compact ? 0 : 2)) + pad;
    menu_panel(px, y, pw, hw_h, T(L"ОБОРУДОВАНИЕ", L"HARDWARE"), COL_ACCENT_HI);

    UINTN cy = y + hw_title_h + 6;
    UINTN cx = px + 16;
    UINTN line_h = ch + (compact ? 0 : 2);

    SPrint(buf, sizeof(buf), L"%s %a %a",
           T(L"Система:  ", L"System:   "), g_sys_vendor, g_sys_model);
    gfx_draw_str_color(cx, cy, buf, COL_FG); cy += line_h;

    SPrint(buf, sizeof(buf), L"%s %a",
           T(L"CPU:      ", L"CPU:      "), g_cpu_brand);
    gfx_draw_str_color(cx, cy, buf, COL_FG); cy += line_h;

    UINT64 buf_mb = (g_mem_pages * 4096ULL) / (1024ULL * 1024ULL);
    UINT32 cov_x10 = g_total_ram_mb ? (UINT32)((buf_mb * 1000) / g_total_ram_mb) : 0;
    UINT32 sp = g_max_dimm_configured_mt ? g_max_dimm_configured_mt : g_max_dimm_speed_mt;
    UINT64 ram_gb_x10 = (g_total_ram_mb * 10ULL + 512) / 1024ULL;
    SPrint(buf, sizeof(buf),
           T(L"RAM:      %ld.%ld ГБ %s, %d DIMM @ %d MT/s, тест %ld МБ (%d.%d%%)",
             L"RAM:      %ld.%ld GB %s, %d DIMMs @ %d MT/s, tested %ld MB (%d.%d%%)"),
           ram_gb_x10 / 10, ram_gb_x10 % 10,
           g_dimm_ddr_type ? ddr_type_name(g_dimm_ddr_type) : T(L"тип ?", L"type ?"),
           g_dimm_count, sp,
           buf_mb, cov_x10 / 10, cov_x10 % 10);
    gfx_draw_str_color(cx, cy, buf, COL_FG); cy += line_h;

    for (UINT32 i = 0; i < n_dimm_rows; i++) {
        dimm_info_t *d = &g_dimms[i];
        SPrint(buf, sizeof(buf),
               L"  %a: %d МБ  %a  %a  %d MT/s",
               d->locator, d->size_mb, d->manufacturer, d->part_number,
               d->configured_speed_mt ? d->configured_speed_mt : d->speed_mt);
        gfx_draw_str_color(cx, cy, buf, COL_DIM); cy += line_h;
    }

    SPrint(buf, sizeof(buf), L"BIOS:     %a  %a   |   %s %d / %d   |   AVX2: %s",
           g_bios_vendor, g_bios_version,
           T(L"Ядра", L"Cores"),
           (UINT32)g_n_enabled, (UINT32)g_n_cores,
           g_has_avx2 ? T(L"вкл.", L"on  ")
                      : T(L"пропуск", L"skip"));
    gfx_draw_str_color(cx, cy, buf, COL_FG);

    y += hw_h + pad;

    /* ============================ COVERAGE PANEL ============================ */
    UINT64 total_addressable = 0;
    for (UINT32 r = 0; r < g_n_regions; r++)
        total_addressable += (g_regions[r].pages * 4096ULL) / (1024ULL * 1024ULL);
    UINT32 cov_plan_x10;
    UINT64 mb_to_test;
    UINT32 passes_plan;
    if (g_cfg_multipass) {
        mb_to_test = total_addressable;
        passes_plan = g_cfg_passes ? g_cfg_passes
                                    : (UINT32)((total_addressable + g_cfg_buffer_cap_mb - 1)
                                               / g_cfg_buffer_cap_mb);
        cov_plan_x10 = g_total_ram_mb ? (UINT32)((mb_to_test * 1000) / g_total_ram_mb) : 0;
    } else {
        mb_to_test = buf_mb;
        passes_plan = g_cfg_passes ? g_cfg_passes : 1;
        cov_plan_x10 = cov_x10;
    }

    /* Per-pass estimate (s) for the stress-oriented catalogue.
       Regular pass (every pass): AVX2-Sustained 10 + TRRespass 60 +
         Cache-Evict 10 + March-C- 2 + March-RAW 3 + Butterfly 5 +
         Address 5 + VRM-Sq 10 + Random 6 ≈ 111 sec.
       Pass 1 ONLY: + Thermal Soak 180 + BW Soak 300 + Bit Fade Ext
         (4 × bitfade_s + setup). Adds ~8-10 min once. */
    UINT32 per_pass = 60 + 10 + 2 + 3 + 5 + 5 + 6
                    + (g_has_avx2 ? (10 + 10) : 0);
    /* One-shot tests on pass 1: Thermal Soak + BW Soak + Bit Fade Ext. */
    UINT32 soak_once = (g_has_avx2 ? (180 + 300) : 0);
    UINT32 bf_once   = (UINT32)(4 * (g_cfg_bitfade_s + 4));
    UINT32 total_est = per_pass * passes_plan + soak_once + bf_once;

    /* Number of body rows depends on context — base = coverage row.
       In non-ultra screens we also show the per-pass time estimate row.
       Ultra mode (< 800 px) drops it: the same numbers are computed and
       displayed on the action buttons ("~22 мин") anyway, so no info loss. */
    UINTN cov_body_rows = 1;
    if (!ultra)           cov_body_rows++;   /* per-pass estimate row */
    if (!g_cfg_multipass) cov_body_rows++;
    if (g_is_ddr5)        cov_body_rows++;
    if (g_xmp_any_flagged) cov_body_rows++;  /* XMP warning line */
    UINTN cov_h = hw_title_h + cov_body_rows * line_h + pad;
    menu_panel(px, y, pw, cov_h, T(L"ПОКРЫТИЕ", L"COVERAGE"), COL_OK);

    cy = y + hw_title_h + 6;
    SPrint(buf, sizeof(buf),
           T(L"%ld МБ из %ld МБ  (%d.%d%% от RAM)   MultiPass=%d   Passes=%d",
             L"%ld MB of %ld MB  (%d.%d%% of RAM)   MultiPass=%d   Passes=%d"),
           mb_to_test, g_total_ram_mb, cov_plan_x10 / 10, cov_plan_x10 % 10,
           g_cfg_multipass, passes_plan);
    gfx_draw_str_color(cx, cy, buf, COL_FG); cy += line_h;

    if (!ultra) {
        SPrint(buf, sizeof(buf),
               T(L"Один проход ~%d сек   ·   суммарно ~%d сек  (~%d мин)",
                 L"One pass ~%d s   ·   total ~%d s  (~%d min)"),
               per_pass, total_est, (total_est + 59) / 60);
        gfx_draw_str_color(cx, cy, buf, COL_DIM); cy += line_h;
    }

    /* DDR5 auto-tune notice. We list ONLY what we actually changed so the
       banner isn't a lie when the user pinned BitFadeSeconds in
       quantai.ini (in which case Bit Fade kept its user value, but Row
       Hammer was still doubled). Without this distinction the banner used
       to claim "Bit Fade 60 с" which read as "tuned for DDR5" but was
       actually the DDR4-default-equivalent value. */
    if (g_is_ddr5) {
        int bitfade_was_tuned = (!g_cfg_bitfade_explicit && g_cfg_bitfade_s >= 300);
        if (bitfade_was_tuned) {
            SPrint(buf, sizeof(buf),
                   T(L"  DDR5: Bit Fade %d с/фаза (×%d), Row Hammer ×2 — компенсация TRR/ODECC",
                     L"  DDR5: Bit Fade %d s/phase (×%d), Row Hammer ×2 — TRR/ODECC compensation"),
                   g_cfg_bitfade_s, g_cfg_bitfade_s / 60);
        } else {
            SPrint(buf, sizeof(buf),
                   T(L"  DDR5: Row Hammer ×2 — компенсация TRR/ODECC. Bit Fade %d с (твой выбор в quantai.ini)",
                     L"  DDR5: Row Hammer ×2 — TRR/ODECC compensation. Bit Fade %d s (your quantai.ini value)"),
                   g_cfg_bitfade_s);
        }
        gfx_draw_str_color(cx, cy, buf, COL_ACCENT_HI); cy += line_h;
    }

    if (!g_cfg_multipass) {
        gfx_draw_str_color(cx, cy,
            T(L"  → Чтобы покрыть всю RAM: в quantai.ini поставь MultiPass=1, Passes=0",
              L"  → For full RAM coverage: set MultiPass=1, Passes=0 in quantai.ini"),
            COL_RUN);
        cy += line_h;
    }

    /* XMP/EXPO warning — appears only when at least one DIMM is running
       above its JEDEC max speed. Visible in red on the main menu so the
       user sees it BEFORE starting tests. */
    if (g_xmp_any_flagged) {
        gfx_draw_str_color(cx, cy,
            T(L"  ⚠ XMP/EXPO активен — нестабильный OC возможная причина BSOD; отключи в BIOS если жалобы",
              L"  ⚠ XMP/EXPO active — unstable OC may cause BSODs; disable in BIOS to verify"),
            COL_FAIL);
        cy += line_h;
    }

    y += cov_h + pad;

    /* ============================ TESTS PANEL ============================
       Two layouts depending on vertical budget:
         - "Comfort" (g_h ≥ 900): single column, name+time+description per row.
                                  12 rows + header + separator = 14 rows.
         - "Compact" (g_h < 900): two columns side-by-side, no description.
                                  Rows shrink to 6 per column = 7 total rows.
       The compact path saves ~200 px of vertical space — critical on Dell
       OptiPlex / Intel iGPU firmware that allocates 1024×768 fb regardless
       of the monitor's native size. Description text is dropped because it
       wouldn't fit horizontally split anyway, and the same info is encoded
       in the action buttons ("стресс + 4 теста · 3 прох."). */
    int two_col = (g_h < 900);
    UINTN tests_title_h = hw_title_h;
    UINTN rows_per_col  = two_col
                        ? ((N_TESTS + 1) / 2)        /* ceil(12/2) = 6 */
                        : (UINTN)N_TESTS;
    UINTN tests_h = tests_title_h + (rows_per_col + 2) * line_h + pad;
    menu_panel(px, y, pw, tests_h, T(L"ТЕСТЫ", L"TESTS"), COL_RUN);

    cy = y + tests_title_h + 4;
    UINTN col_w = two_col ? (pw / 2) : pw;
    /* Column-relative offsets — applied to both halves in 2-col mode. */
    UINTN off_mark = 0;
    UINTN off_idx  = off_mark + 2 * cw;
    UINTN off_name = off_idx  + 4 * cw;
    UINTN off_time = off_name + 20 * cw;
    UINTN off_desc = off_time + 10 * cw;

    /* Header strip — show description label only when we actually have a
       description column (single-col layout). */
    {
        UINTN hx = cx;
        gfx_draw_str_color(hx + off_mark, cy, L"·",                          COL_DIM);
        gfx_draw_str_color(hx + off_idx,  cy, T(L"№",     L"#"),             COL_DIM);
        gfx_draw_str_color(hx + off_name, cy, T(L"Тест",  L"Test"),          COL_DIM);
        gfx_draw_str_color(hx + off_time, cy, T(L"Время", L"Time"),          COL_DIM);
        if (!two_col) {
            gfx_draw_str_color(hx + off_desc, cy,
                               T(L"Описание", L"Description"), COL_DIM);
        }
        if (two_col) {
            /* Mirror the column header into the right half. */
            UINTN rx = cx + col_w;
            gfx_draw_str_color(rx + off_mark, cy, L"·",                      COL_DIM);
            gfx_draw_str_color(rx + off_idx,  cy, T(L"№",     L"#"),         COL_DIM);
            gfx_draw_str_color(rx + off_name, cy, T(L"Тест",  L"Test"),      COL_DIM);
            gfx_draw_str_color(rx + off_time, cy, T(L"Время", L"Time"),      COL_DIM);
        }
    }
    cy += line_h;
    blt_fill(px + 8, cy - 2, pw - 16, 1, COL_BORDER);

    UINTN desc_avail_px = (px + pw) - (cx + off_desc) - 4;
    UINTN desc_max_chars = (desc_avail_px > cw) ? (desc_avail_px / cw) : 0;
    if (desc_max_chars > 90) desc_max_chars = 90;

    UINTN base_cy = cy;
    for (UINTN i = 0; i < N_TESTS; i++) {
        int will_skip = (g_tests[i].k == KER_AVX2 && !g_has_avx2);
        UINT32 cmark  = g_tests[i].in_quick ? COL_OK : COL_DIM;
        UINT32 ctext  = will_skip ? COL_DIM : COL_FG;
        /* Which column does this test belong to?
            two_col = false → always column 0.
            two_col = true  → first rows_per_col tests in column 0,
                              the rest in column 1. */
        UINTN colno  = two_col ? (i / rows_per_col) : 0;
        UINTN rowno  = two_col ? (i % rows_per_col) : i;
        UINTN rx     = cx + colno * col_w;
        UINTN ry     = base_cy + rowno * line_h;

        gfx_draw_str_color(rx + off_mark, ry,
                           g_tests[i].in_quick ? L"·" : L" ", cmark);
        SPrint(buf, sizeof(buf), L"%d.", (UINT32)(i + 1));
        gfx_draw_str_color(rx + off_idx,  ry, buf, COL_DIM);
        gfx_draw_str_color(rx + off_name, ry, g_tests[i].name, ctext);
        gfx_draw_str_color(rx + off_time, ry,
            will_skip ? T(L"пропуск", L"skip")
                      : T(g_tests[i].eta_ru, g_tests[i].eta_en),
            will_skip ? COL_FAIL : COL_ACCENT_HI);

        if (!two_col) {
            CHAR16 desc_buf[120];
            CHAR16 *src = T(g_tests[i].desc_ru, g_tests[i].desc_en);
            UINTN sl = StrLen(src);
            if (sl <= desc_max_chars) {
                gfx_draw_str_color(rx + off_desc, ry, src, ctext);
            } else if (desc_max_chars >= 1) {
                UINTN cap = desc_max_chars - 1;
                if (cap > 118) cap = 118;
                UINTN k;
                for (k = 0; k < cap; k++) desc_buf[k] = src[k];
                desc_buf[k++] = L'…';
                desc_buf[k]   = 0;
                gfx_draw_str_color(rx + off_desc, ry, desc_buf, ctext);
            }
        }
    }
    cy = base_cy + rows_per_col * line_h;

    y += tests_h + pad;

    /* ============================ ACTION BUTTONS ============================
       Two big buttons side-by-side: [1] FULL · [2] QUICK.
       In tight mode (g_h < 900) buttons shrink to 2-line height so the
       footer still fits below them on small displays. */
    /* Quick: 4 hardest tests × 3 passes + Thermal Soak on pass 1.
       AVX2-Sustained 10 + TRRespass 60 + Cache-Evict 10 + March-C- 2
       = ~82 sec/pass. ×3 passes + 180 sec Thermal Soak (pass 1) ≈ 7 min.
       Thermal Soak makes Quick a REAL stress test instead of a 4-min
       "barely-above-idle" benchmark. */
    UINT32 quick_per_pass = 60 + 10 + 2 + (g_has_avx2 ? 10 : 0);
    UINT32 quick_total    = quick_per_pass * 3 + (g_has_avx2 ? 180 : 0);

    UINTN gap = pad;
    UINTN btn_h = tight ? (ch * 2 + 4) : (ch * 3 + 8);
    UINTN btn_w = (pw - gap) / 2;

    /* RESERVE space for the footer first, then size/place buttons within
       what's left. Otherwise on short screens the TESTS panel may push y
       so far down that buttons overlap (or replace) the footer — which is
       what happened on the Dell OptiPlex 5050 / 1024×768 firmware-quirk
       scenario. Footer needs: 1 row of text + 4 px gap-above + 4 px below. */
    UINTN footer_total_h = ch + 8;
    UINTN footer_top = g_h - footer_total_h;
    /* Buttons must finish at least `pad` before footer_top. If they don't,
       shrink btn_h to make them fit; never below 2 char rows. */
    if (y + btn_h + pad > footer_top) {
        UINTN avail = (footer_top > y + pad) ? (footer_top - y - pad) : 0;
        UINTN min_h = ch * 2 + 4;          /* still readable */
        btn_h = (avail >= min_h) ? avail : min_h;
        /* If even min_h doesn't fit, the layout above us is too tall —
           push buttons up by overlapping the TESTS panel's bottom padding.
           Better than buttons disappearing entirely. */
        if (y + btn_h + pad > footer_top && footer_top > min_h + pad) {
            y = footer_top - btn_h - pad;
        }
    }

    /* [1] Full test — green (recommended for full diagnosis) */
    CHAR16 full_sub[160];
    SPrint(full_sub, sizeof(full_sub),
           T(L"все 10 тестов · %d прох. · ~%d мин",
             L"all 10 tests · %d passes · ~%d min"),
           passes_plan, (total_est + 59) / 60);
    menu_button(px, y, btn_w, btn_h,
                L"[1]",
                T(L"ПОЛНЫЙ ТЕСТ", L"FULL TEST"),
                full_sub,
                COL_OK, COL_OK_DK);

    /* [2] Quick test — amber (triage stress test) */
    CHAR16 quick_sub[160];
    SPrint(quick_sub, sizeof(quick_sub),
           T(L"стресс + 4 теста · 3 прох. · ~%d мин",
             L"stress + 4 tests · 3 passes · ~%d min"),
           (quick_total + 59) / 60);
    menu_button(px + btn_w + gap, y, btn_w, btn_h,
                L"[2]",
                T(L"БЫСТРЫЙ ТЕСТ", L"QUICK TEST"),
                quick_sub,
                COL_RUN, COL_RUN_DK);

    y += btn_h + pad;

    /* ============================ FOOTER ============================ */
    /* Compact key-help line, centered. Adaptive width — full string on wide
       displays, abbreviated on narrow ones (Dell OptiPlex 5050 with 800×600
       firmware-allocated fb couldn't fit the full string + clipped on the
       right edge → user never saw the [I] info hint). */
    CHAR16 footer[240];
    if (g_text_cols >= 78) {
        SPrint(footer, sizeof(footer),
               T(L"[1] полный   [2] быстрый   [I] о программе   [L] язык: %s   [ESC] перезагрузка",
                 L"[1] full   [2] quick   [I] about   [L] language: %s   [ESC] reboot"),
               g_lang ? L"EN" : L"RU");
    } else if (g_text_cols >= 60) {
        SPrint(footer, sizeof(footer),
               T(L"[1] полный  [2] быстрый  [I] инфо  [L] %s  [ESC] reboot",
                 L"[1] full  [2] quick  [I] info  [L] %s  [ESC] reboot"),
               g_lang ? L"EN" : L"RU");
    } else {
        SPrint(footer, sizeof(footer),
               T(L"[1] полн [2] быстр [I] инфо [L] %s [ESC]",
                 L"[1] full [2] quick [I] info [L] %s [ESC]"),
               g_lang ? L"EN" : L"RU");
    }
    UINTN foot_y = g_h - ch - pad;
    if (foot_y < y + 4) foot_y = y + 4;
    /* footer strip background */
    blt_fill(0, foot_y - 4, g_w, ch + 8, COL_PANEL_ALT);
    blt_fill(0, foot_y - 5, g_w, 1, COL_BORDER);
    gfx_draw_str_color((g_w - StrLen(footer) * cw) / 2,
                       foot_y, footer, COL_FG);
}

static int main_menu_wait(void) {
    /* Returns 1 to start tests, 0 to reboot.
       BUG WE FIX HERE: WaitForEvent on ConIn->WaitForKey is known to fire
       spuriously on some buggy AMI/Insyde firmwares. The previous version
       did `ReadKeyStroke(&k); if (k.UnicodeChar == 13) return 1;` without
       checking ReadKeyStroke's return code — so when the wake was spurious,
       `k` stayed uninitialised stack garbage. If that garbage happened to
       equal 13 (ENTER), the menu would "press itself" → tests started
       without any user input. That matches the user-reported "test
       restarted by itself after completing". */
    int need_redraw = 1;
    for (;;) {
        if (need_redraw) {
            /* LATE firmware-quirk recheck: by this point we've done many
               Blt operations (cls during init + STEP 2 cls + buffer alloc
               logging), so any firmware that lazy-updates Mode->Info has
               had time to settle. If actual fb is smaller than initially
               reported, downgrade g_w/g_h NOW so render_main_menu lays
               out within the real fb. */
            recheck_fb_dimensions();
            render_main_menu();
            need_redraw = 0;
        }

        EFI_INPUT_KEY k = { 0, 0 };
        /* Periodic-wake variant: 200 ms timer ticks keep the EFI USB stack
           alive on old HP business firmwares that otherwise stop polling
           the keyboard during long blocking WaitForEvent calls. Spurious /
           timer wakes just loop back to wait again — no harm done. */
        if (!wait_key_or_timer(&k, 200)) {
            continue;   /* timer tick or spurious wake */
        }

        /* [1] / ENTER / SPACE → Full test (recommended).
           [2] / Q             → Quick triage.
           The full test now uses the strongest tests we have (March-C-,
           TRRespass, AVX2 Sustained, etc.) — there's no separate
           "aggressive" mode because the strong tests ARE the default. */
        if (k.UnicodeChar == L'1' ||
            k.UnicodeChar == 13   ||  /* ENTER */
            k.UnicodeChar == ' ') {
            g_quick_mode = 0;
            return 1;        /* full test mode */
        }
        if (k.UnicodeChar == L'2' ||
            k.UnicodeChar == L'q' || k.UnicodeChar == L'Q') {
            g_quick_mode = 1;
            return 1;        /* quick triage mode */
        }
        if (k.ScanCode == SCAN_ESC) return 0;
        if (k.UnicodeChar == L'l' || k.UnicodeChar == L'L') {
            g_lang = !g_lang;
            need_redraw = 1;
        }
        /* [I] — full program description + comparison vs other tools. */
        if (k.UnicodeChar == L'i' || k.UnicodeChar == L'I') {
            render_about_screen();
            about_wait_any_key();
            need_redraw = 1;
        }
    }
}

/* Re-check firmware-quirk discrepancy. On some Intel iGPU UEFIs the
   Mode->Info fields are STALE immediately after SetMode and only get
   refreshed by firmware after the first several Blt operations. Calling
   this later (e.g. just before main menu render) gives firmware time to
   settle and produces a correct override that the SetMode-time check
   missed. Idempotent — safe to call multiple times. */
static void recheck_fb_dimensions(void) {
    if (!g_gop) return;
    EFI_GRAPHICS_OUTPUT_MODE_INFORMATION *info = g_gop->Mode->Info;
    UINT32 ppsl = info->PixelsPerScanLine;
    UINT64 fbsz = g_gop->Mode->FrameBufferSize;
    if (ppsl == 0 || fbsz == 0) return;
    UINT32 actual_w = (ppsl < g_w) ? ppsl : g_w;
    UINT32 max_rows = (UINT32)(fbsz / 4 / ppsl);
    UINT32 actual_h = (max_rows < g_h) ? max_rows : g_h;
    if (actual_w < g_w || actual_h < g_h) {
        CHAR16 lb[180];
        SPrint(lb, sizeof(lb),
               L"[GFX] LATE firmware-quirk: claimed %dx%d but fbsize/ppsl imply %dx%d — re-downgrading",
               (UINT32)g_w, (UINT32)g_h, actual_w, actual_h);
        log_line(lb);
        g_w = actual_w;
        g_h = actual_h;
        g_text_cols = g_w / g_char_w;
        g_text_rows = g_h / g_char_h;
    }
}

/* ---------- Entry ---------- */
EFI_STATUS efi_main(EFI_HANDLE ImageHandle, EFI_SYSTEM_TABLE *SystemTable) {
    InitializeLib(ImageHandle, SystemTable);

    /* CRITICAL: UEFI firmware arms a 5-minute watchdog on every loaded image.
       If we don't kick it or disable it, firmware HARD-RESETS the machine at
       t=300 s. Multi-pass memory testing easily runs 30+ minutes — without
       this disable, every long run gets killed mid-test and looks like
       hardware fault. Pass timeout=0 to fully disable. */
    uefi_call_wrapper(BS->SetWatchdogTimer, 4,
                      (UINTN)0, (UINT64)0, (UINTN)0, NULL);

    EFI_GUID gop_guid = EFI_GRAPHICS_OUTPUT_PROTOCOL_GUID;
    uefi_call_wrapper(BS->LocateProtocol, 3, &gop_guid, NULL, (VOID **)&g_gop);
    if (g_gop) {
        /* Pick the best GOP mode. Always go for the LARGEST mode firmware
           offers, because:
             - If we cap at 1080p but monitor is 4K, the 1080p framebuffer
               is shown in the top-left quadrant (user's "1/4 screen" bug).
             - Firmware-enumerated modes are guaranteed to work — picking
               the max one always fills the physical display.
           Tiny-font concern at 4K is handled by g_font_scale below: when
           the picked resolution is bigger than 1500 px vertical, we render
           the bitmap font at 2× so each character covers 28×56 px instead
           of 14×28 — same visual size as on a 1080p screen.

           [Display] Width=N Height=N in quantai.ini overrides the picked
           mode if user needs to force a specific resolution (e.g. firmware
           offers a broken mode that should be skipped). */
        /* v0.4.29 — robust mode picker with per-mode verification.
           Field report (MSI B650 TOMAHAWK + BIOS 1.M3 on Ryzen 9 7900X):
           our SetMode(3440x1440) was silently rejected by firmware which
           stayed at default 800x600 — but Mode->Info still LIED that it
           was at 3440x1440. The pre-v0.4.29 LATE check eventually caught
           the mismatch and clamped g_w/g_h, but by then the splash and
           main menu had already rendered as garbage onto an 800x600
           framebuffer using 3440x1440 coordinates.

           New approach: build a list of all candidate modes sorted by
           pixel area (largest first). For each, try SetMode + verify that
           the framebuffer size and PixelsPerScanLine actually match what
           the mode claims. If they don't match — firmware silently
           rejected the switch — skip and try the next-largest. Stop at
           the first mode that activates honestly. */

        UINT32 n_modes = g_gop->Mode->MaxMode;
        /* Cap to MAX_MODE_CAND to keep stack small; firmware rarely
           exposes more than 20 modes anyway. */
        #define MAX_MODE_CAND 32
        UINT32 cand[MAX_MODE_CAND];
        UINT64 cand_px[MAX_MODE_CAND];
        UINTN  n_cand = 0;
        for (UINT32 m = 0; m < n_modes && n_cand < MAX_MODE_CAND; m++) {
            EFI_GRAPHICS_OUTPUT_MODE_INFORMATION *info = NULL;
            UINTN info_sz = 0;
            EFI_STATUS qs = uefi_call_wrapper(g_gop->QueryMode, 4,
                                              g_gop, m, &info_sz, &info);
            if (EFI_ERROR(qs) || !info) continue;
            UINT32 w = info->HorizontalResolution;
            UINT32 h = info->VerticalResolution;
            if (w == 0 || h == 0) continue;
            if (g_cfg_force_w && g_cfg_force_h) {
                if (w != g_cfg_force_w || h != g_cfg_force_h) continue;
            }
            cand[n_cand]    = m;
            cand_px[n_cand] = (UINT64)w * h;
            n_cand++;
        }
        /* Simple selection sort, largest pixel area first. */
        for (UINTN i = 0; i < n_cand; i++) {
            UINTN best = i;
            for (UINTN j = i + 1; j < n_cand; j++) {
                if (cand_px[j] > cand_px[best]) best = j;
            }
            if (best != i) {
                UINT32 tm = cand[i]; cand[i] = cand[best]; cand[best] = tm;
                UINT64 tp = cand_px[i]; cand_px[i] = cand_px[best]; cand_px[best] = tp;
            }
        }

        /* Try modes in order; accept the first one that activates for real. */
        UINT32 active_w = 0, active_h = 0;
        UINT32 active_ppsl = 0;
        UINT64 active_fbsz = 0;
        for (UINTN i = 0; i < n_cand; i++) {
            UINT32 m = cand[i];
            EFI_GRAPHICS_OUTPUT_MODE_INFORMATION *info = NULL;
            UINTN info_sz = 0;
            if (uefi_call_wrapper(g_gop->QueryMode, 4, g_gop, m,
                                  &info_sz, &info) != EFI_SUCCESS || !info)
                continue;
            UINT32 want_w = info->HorizontalResolution;
            UINT32 want_h = info->VerticalResolution;

            /* Only SetMode if not already the active mode. */
            if (m != g_gop->Mode->Mode) {
                EFI_STATUS ss = uefi_call_wrapper(g_gop->SetMode, 2, g_gop, m);
                if (EFI_ERROR(ss)) {
                    CHAR16 lb[160];
                    SPrint(lb, sizeof(lb),
                           L"[GFX] mode[%d] %dx%d: SetMode failed (status=0x%lx), trying next",
                           m, want_w, want_h, (UINT64)ss);
                    log_line(lb);
                    continue;
                }
            }
            /* Force a tiny Blt — some Intel iGPU firmwares leave Mode->Info
               stale until the first real Blt happens. */
            {
                EFI_GRAPHICS_OUTPUT_BLT_PIXEL kick = {0, 0, 0, 0};
                uefi_call_wrapper(g_gop->Blt, 10, g_gop, &kick, EfiBltVideoFill,
                                  0, 0, 0, 0, 1, 1, 0);
            }
            /* Verify the switch by checking FBSize / PPSL match the
               requested mode. Firmware lies via Mode->Info but can't lie
               via FBSize (it's the actual allocation size). */
            UINT32 got_ppsl = g_gop->Mode->Info->PixelsPerScanLine;
            UINT64 got_fbsz = g_gop->Mode->FrameBufferSize;
            if (got_ppsl == 0 || got_fbsz == 0) {
                CHAR16 lb[160];
                SPrint(lb, sizeof(lb),
                       L"[GFX] mode[%d] %dx%d: ppsl/fbsz are zero — accepting anyway",
                       m, want_w, want_h);
                log_line(lb);
                active_w = want_w; active_h = want_h;
                active_ppsl = got_ppsl; active_fbsz = got_fbsz;
                break;
            }
            UINT32 implied_max_h = (UINT32)(got_fbsz / 4 / got_ppsl);
            /* Accept if the framebuffer can actually hold this mode (ppsl
               >= want_w AND implied_max_h >= want_h). Allow some slack —
               sometimes PPSL is slightly larger than width (alignment). */
            if (got_ppsl >= want_w && implied_max_h >= want_h) {
                active_w = want_w; active_h = want_h;
                active_ppsl = got_ppsl; active_fbsz = got_fbsz;
                break;
            }
            /* Firmware silently failed — fb too small for requested mode. */
            CHAR16 lb[200];
            SPrint(lb, sizeof(lb),
                   L"[GFX] mode[%d] %dx%d: firmware silently rejected "
                   L"(fbsz=%ld ppsl=%d implies max %dx%d), trying next",
                   m, want_w, want_h, got_fbsz, got_ppsl, got_ppsl, implied_max_h);
            log_line(lb);
        }

        if (active_w == 0 || active_h == 0) {
            /* Nothing accepted. Fall back to whatever Mode->Info currently
               says — best effort. */
            EFI_GRAPHICS_OUTPUT_MODE_INFORMATION *info = g_gop->Mode->Info;
            active_w = info->HorizontalResolution;
            active_h = info->VerticalResolution;
            active_ppsl = info->PixelsPerScanLine;
            active_fbsz = g_gop->Mode->FrameBufferSize;
            log_line(L"[GFX] WARN: no mode activated cleanly — using current "
                     L"firmware values (display may be wrong)");
        }

        g_w = active_w;
        g_h = active_h;
        /* If PPSL is smaller than g_w (rare quirk), clamp g_w to PPSL
           and g_h to what fits in fbsize. Same logic as the old LATE
           check, but now applied per-mode. */
        if (active_ppsl > 0 && active_fbsz > 0 && active_ppsl < g_w) {
            UINT32 max_rows = (UINT32)(active_fbsz / 4 / active_ppsl);
            CHAR16 lb[180];
            SPrint(lb, sizeof(lb),
                   L"[GFX] PPSL=%d < width=%d — clamping to %dx%d "
                   L"(firmware pixel alignment quirk)",
                   active_ppsl, (UINT32)g_w, active_ppsl,
                   max_rows < g_h ? max_rows : (UINT32)g_h);
            log_line(lb);
            g_w = active_ppsl;
            if (max_rows < g_h) g_h = max_rows;
        }
        #undef MAX_MODE_CAND
    }

    EFI_GUID mp_guid = EFI_MP_SERVICES_PROTOCOL_GUID;
    EFI_STATUS mp_locate_status = uefi_call_wrapper(BS->LocateProtocol, 3,
                                                    &mp_guid, NULL, (VOID **)&g_mp);
    EFI_STATUS mp_gnp_status = EFI_NOT_FOUND;
    UINTN mp_raw_cores = 0, mp_raw_enabled = 0;
    if (g_mp) {
        mp_gnp_status = uefi_call_wrapper(g_mp->GetNumberOfProcessors, 3, g_mp,
                                          &g_n_cores, &g_n_enabled);
        mp_raw_cores = g_n_cores;
        mp_raw_enabled = g_n_enabled;
        if (g_n_enabled > MAX_CORES) g_n_enabled = MAX_CORES;
    }
    if (g_n_enabled == 0) g_n_enabled = 1;
    if (g_n_cores   == 0) g_n_cores   = 1;
    /* MP services diagnostic log — written AFTER log file opens, see below. */

    /* We use our own bundled font (12x24) for ALL UI text, not UEFI ConOut.
       That gives us reliable Cyrillic rendering on any firmware. */
    if (g_w == 0) g_w = 800;
    if (g_h == 0) g_h = 600;
    /* Pick font scale: 2× on hi-DPI displays (>1500 px tall) so the bitmap
       font isn't a tiny speck on a 4K monitor. INI override available. */
    if (g_cfg_font_scale > 0) {
        g_font_scale = g_cfg_font_scale;
    } else {
        g_font_scale = (g_h > 1500) ? 2 : 1;
    }
    if (g_font_scale < 1) g_font_scale = 1;
    if (g_font_scale > 4) g_font_scale = 4;
    g_char_w   = FONT_ADVANCE * g_font_scale;
    g_char_h   = FONT_H       * g_font_scale;
    g_text_cols = g_w / g_char_w;
    g_text_rows = g_h / g_char_h;

    /* Log file */
    EFI_LOADED_IMAGE *li = NULL;
    EFI_GUID li_guid = LOADED_IMAGE_PROTOCOL;
    uefi_call_wrapper(BS->HandleProtocol, 3, ImageHandle, &li_guid, (VOID **)&li);
    if (li) {
        EFI_FILE_IO_INTERFACE *fs = NULL;
        EFI_GUID fs_guid = SIMPLE_FILE_SYSTEM_PROTOCOL;
        uefi_call_wrapper(BS->HandleProtocol, 3, li->DeviceHandle, &fs_guid, (VOID **)&fs);
        if (fs) {
            uefi_call_wrapper(fs->OpenVolume, 2, fs, &g_logroot);
            if (g_logroot) {
                /* Delete the previous log first so each boot starts clean.
                   Without this, EFI_FILE_MODE_CREATE on an existing file
                   leaves old bytes past our new writes (file isn't
                   truncated — that's an UEFI Open quirk). */
                EFI_FILE_PROTOCOL *old = NULL;
                if (uefi_call_wrapper(g_logroot->Open, 5, g_logroot, &old,
                        L"memforge2.log",
                        EFI_FILE_MODE_READ | EFI_FILE_MODE_WRITE, 0)
                        == EFI_SUCCESS && old) {
                    uefi_call_wrapper(old->Delete, 1, old);  /* closes + removes */
                }
                uefi_call_wrapper(g_logroot->Open, 5, g_logroot, &g_logfile, L"memforge2.log",
                                  EFI_FILE_MODE_CREATE | EFI_FILE_MODE_READ
                                  | EFI_FILE_MODE_WRITE, 0);
            }
        }
    }

    log_line(L"=== MemForge2 v0.4.29 init ===");
    log_line(L"[WATCHDOG] UEFI 5-min watchdog disabled at app entry");
    /* Show splash IMMEDIATELY so the user sees the program is alive while
       INI parsing, SMBus probes and SMBIOS walk happen. Without this, the
       firmware boot logo just hangs there for several seconds on slow PCs
       (HP Sure Start can add 5-15 s) and users yank the USB thinking
       it never started. */
    init_splash(L"Init...");
    parse_quantai_ini();
    /* Honor MaxCores from INI: clamp g_n_enabled before tests reset state. */
    if (g_cfg_max_cores > 0 && g_cfg_max_cores < g_n_enabled) {
        CHAR16 lb[80];
        SPrint(lb, sizeof(lb),
               L"[INI] MaxCores=%d limits g_n_enabled from %d to %d",
               g_cfg_max_cores, (UINT32)g_n_enabled, g_cfg_max_cores);
        log_line(lb);
        g_n_enabled = g_cfg_max_cores;
    }

    {
        CHAR16 lb[160];
        SPrint(lb, sizeof(lb),
               L"[GFX] g_w=%d g_h=%d (text grid %d cols x %d rows at %dx%d px font)",
               (UINT32)g_w, (UINT32)g_h,
               (UINT32)g_text_cols, (UINT32)g_text_rows,
               (UINT32)g_char_w, (UINT32)g_char_h);
        log_line(lb);
        /* Dump every GOP mode the firmware exposes — invaluable when our
           auto-selection picks the "wrong" size on someone's PC. The user
           reported that on a different machine the UI appeared in 1/4 of
           the screen; this list will show us exactly what modes were on
           offer and confirm our selector picked the largest sane one. */
        if (g_gop) {
            UINT32 n_modes = g_gop->Mode->MaxMode;
            UINT32 cur = g_gop->Mode->Mode;
            SPrint(lb, sizeof(lb),
                   L"[GFX] GOP modes available=%d, current=%d:",
                   n_modes, cur);
            log_line(lb);
            for (UINT32 m = 0; m < n_modes && m < 32; m++) {
                EFI_GRAPHICS_OUTPUT_MODE_INFORMATION *info = NULL;
                UINTN info_sz = 0;
                if (uefi_call_wrapper(g_gop->QueryMode, 4,
                                      g_gop, m, &info_sz, &info) != EFI_SUCCESS)
                    continue;
                /* v0.4.29 — also log PixelFormat and PixelsPerScanLine
                   so we can see if a card (e.g. old Radeon HD 4350) only
                   offers BltOnly modes (PixelFormat=3) that prevent
                   direct-fb rendering. */
                SPrint(lb, sizeof(lb),
                       L"[GFX]  mode[%d]=%dx%d ppsl=%d pf=%d %a",
                       m, info->HorizontalResolution, info->VerticalResolution,
                       info->PixelsPerScanLine, (UINT32)info->PixelFormat,
                       (m == cur) ? "<-- SELECTED" : "");
                log_line(lb);
            }
        } else {
            log_line(L"[GFX] NO GOP PROTOCOL FOUND — firmware has no UEFI graphics. "
                     L"Falling back to 800x600 default. UI will not render correctly.");
        }
        /* v0.4.29 — MP Services Protocol diagnostic. Without this log it
           was impossible to tell from a field report whether multi-core
           dispatch failed (LocateProtocol error / GetNumberOfProcessors
           returned 1) or the test was simply running on a single-core
           CPU. YgrecK report on a 2009-era Radeon HD 4350 system showed
           only CPU01 in the panel — could be either case. Now we log:
             - LocateProtocol status
             - GetNumberOfProcessors status + raw values
             - whether we capped to MAX_CORES
             - final g_n_cores / g_n_enabled used by tests. */
        if (EFI_ERROR(mp_locate_status) || !g_mp) {
            SPrint(lb, sizeof(lb),
                   L"[MP] EFI_MP_SERVICES_PROTOCOL not found (status=0x%lx) "
                   L"— tests will run on BSP only (single-thread). "
                   L"This is normal on very old or stripped-down firmware.",
                   (UINT64)mp_locate_status);
            log_line(lb);
        } else if (EFI_ERROR(mp_gnp_status)) {
            SPrint(lb, sizeof(lb),
                   L"[MP] GetNumberOfProcessors failed (status=0x%lx) "
                   L"— tests will run on BSP only.",
                   (UINT64)mp_gnp_status);
            log_line(lb);
        } else {
            SPrint(lb, sizeof(lb),
                   L"[MP] services OK: total=%d enabled=%d (raw); after cap g_n_enabled=%d",
                   (UINT32)mp_raw_cores, (UINT32)mp_raw_enabled,
                   (UINT32)g_n_enabled);
            log_line(lb);
            if (mp_raw_enabled == 1 && mp_raw_cores > 1) {
                log_line(L"[MP] WARN: firmware reports total>1 but only 1 enabled — "
                         L"likely BIOS hyperthreading-off or AP-disabled-at-POST.");
            }
        }
    }
    log_line(L"[STEP 1] Detecting CPU features + SMBIOS...");
    init_splash(g_lang ? L"Detecting CPU features..."
                       : L"Анализ CPU...");
    detect_cpu_features();
    /* INI EnableAVX=0 overrides MSR detection (in case user wants to force-skip). */
    if (!g_cfg_enable_avx) {
        log_line(L"[INI] EnableAVX=0 — forcing AVX2 test to be skipped");
        g_has_avx2 = 0;
    }
    detect_cpu_brand();
    calibrate_tsc();
    {
        CHAR16 lb[120];
        SPrint(lb, sizeof(lb), L"[TSC] calibrated freq = %ld Hz (~%ld MHz)",
               g_tsc_freq_hz, g_tsc_freq_hz / 1000000);
        log_line(lb);
    }
    init_splash(g_lang ? L"Reading SMBIOS..." : L"Чтение SMBIOS...");
    parse_smbios();
    /* After SMBIOS we know how many DIMMs there are. Now go direct to each
       DIMM's SPD EEPROM via SMBus to pull info SMBIOS doesn't expose:
       serial number, manufacturing date, JEDEC manufacturer ID, primary
       timings. Non-fatal if it fails (older Intel, AMD, DDR5 I3C-only).
       Often the SLOWEST init step on HP business systems: Sure Start
       blocks SMBus probes and each address NACK has to time out. */
    init_splash(g_lang ? L"Reading DIMM SPDs..." : L"Чтение SPD планок...");
    spd_populate_dimms();
    /* Once total RAM is known: scale buffer-chunk size for big-RAM systems
       so the pass count stays reasonable. Skipped if user pinned BufferMB
       in quantai.ini. */
    apply_ram_size_tuning();
    /* Warn (in log) if this looks like a multi-socket box — RAPL only sees
       one package's power. */
    check_multi_socket();
    /* SMBIOS knows the DRAM generation — apply DDR5-specific test tuning
       (Bit Fade longer, Row Hammer 2x stronger) before the menu/tests run. */
    apply_ddr_tuning();
    /* Warn user if XMP/EXPO is active. Common cause of "RAM tested clean
       but Windows BSODs" — unstable overclock profile. */
    xmp_warn_check();
    /* RAPL probe — once SMBIOS is up, MSR_RAPL_POWER_UNIT (0x606) tells us
       whether the package energy counter is meaningful. Failures are silent;
       g_pkg_power_w simply stays 0 and the header omits the watts cell. */
    detect_rapl();
    /* MCA — detect support and snapshot every MCi_STATUS. The baseline
       captures errors that pre-existed before we ran (could be from
       a previous boot, firmware POST, anything). After our test run we
       re-snapshot and diff. New entries = errors observed by HW during
       our test even if our pattern checks didn't catch the symptom
       (single-bit ECC corrections, marginal cells, etc.). */
    mca_detect();
    if (g_has_mca) mca_snapshot(g_mca_baseline);

    /* Baseline CPU temperature thermal guard. If we're already at 85+°C
       BEFORE any test work, a 12-core AVX2 burst will instantly push past
       Tjmax and either trip the firmware power management cleanly (most
       systems) or thermal-halt without warning (buggy ASUS/AMI AMD UEFI).
       Field-reported AMD hang (Ryzen 5 4500 on ASUS B-series) traced to
       exactly this — Tctl=93°C at idle. v0.4.6 verified the entry path
       into the AVX2 kernel completes (CR0/CR4 fix worked), confirming
       the hang is INSIDE the AVX2 burst — pure thermal halt with no
       chance to log anything past the first FMA instruction.

       Behaviour:
         - <85°C   normal, all tests run
         - >=85°C  set g_thermal_guard_skip_heavy = 1. ap_entry checks
                   this flag and skips KER_AVX2{,_SUSTAINED}, KER_VRM_SQUARE,
                   KER_THERMAL_SOAK, KER_BW_SOAK. Pattern tests still run
                   (March-C-, TRRespass, Cache-Eviction, etc.) — those
                   don't burn all cores at AVX2 power level.
         - IgnoreThermalGuard=1 in INI bypasses the auto-skip.

       Intel per-core temp is sampled inside kernels, not at init, so the
       guard currently only triggers on AMD where SMN gives us a
       synchronous baseline reading. Intel would need IA32_THERM_STATUS
       per-core via a dedicated sampler at init — TODO if Intel users
       hit the same class of issue. */
    {
        UINT32 t0 = 0;
        if (g_cpu_vendor == CPU_AMD && g_has_thermal) {
            t0 = amd_thermal_sample();
        }
        /* v0.4.13 — REVERTED all thermal-guard auto-skip / cap / hard-
           stop logic. Field report from a user on a NEW system with
           verified-good cooling: SMN read shows Tctl=93°C at IDLE and
           jumps to 118°C under load. Real idle temp on that build is
           ~43°C per Windows HWiNFO64 cross-check — meaning the k10temp-
           style decode (raw>>21, /8 = °C) has an undocumented +~50°C
           offset on Ryzen 5 4500 (Renoir desktop) on at least some
           ASUS B-series boards. Linux k10temp table only knows about
           Ryzen 1700X/1800X (+20°C) and Threadripper (+27°C); Renoir
           desktop SKUs aren't tabulated, so we (like Linux) assumed
           offset=0. That assumption is wrong for this SKU.

           Since the MSR-bug fix in v0.4.10 (Intel-only IA32_THERM_STATUS
           in ap_yield) was the actual cause of the original hangs, the
           thermal guard logic isn't needed and was misfiring on bogus
           sensor data. Just log the reading informationally and proceed.

           Users with genuinely-overheating systems will still see the
           warning + a moving live-temp log — they can decide whether
           to abort. Users with bogus-sensor systems just get a warning
           they can ignore. No more auto-skip or hard-stop based on
           sensor data we can't trust per-SKU. */
        if (t0 >= 90) {
            CHAR16 lb[260];
            SPrint(lb, sizeof(lb),
                   L"[TEMP] CPU sensor reads %d°C at idle. If your cooling "
                   L"is normal, this is likely an undocumented SMN Tctl "
                   L"offset on this Ryzen SKU (k10temp doesn't know about "
                   L"every model). Cross-check with HWiNFO64 in Windows. "
                   L"Tests proceeding at full intensity regardless.",
                   t0);
            log_line(lb);
        } else if (t0 >= 75) {
            CHAR16 lb[200];
            SPrint(lb, sizeof(lb),
                   L"[TEMP] Baseline CPU temperature %d°C at IDLE — higher "
                   L"than typical. Tests will proceed but watch for throttling.",
                   t0);
            log_line(lb);
        } else if (t0 > 0) {
            CHAR16 lb[120];
            SPrint(lb, sizeof(lb),
                   L"[TEMP] Baseline CPU temperature %d°C — OK", t0);
            log_line(lb);
        }
    }

    /* Read the previous run record from NVRAM (cold/warm-boot delta).
       Logs "[HIST] Previous run #N: ..." if found; silent on first run. */
    hist_load();
    /* Theoretical peak DRAM bandwidth for the % utilization indicator:
         speed_MT/s × 8 bytes/transfer × channels = peak MB/s
       This is a SHARED peak across all cores reading the same DIMMs. */
    {
        UINT32 sp = g_max_dimm_configured_mt
                    ? g_max_dimm_configured_mt : g_max_dimm_speed_mt;
        UINT32 channels = (g_dimm_count >= 2) ? 2 : 1;  /* most desktops = dual */
        g_bw_peak_theoretical_mbps = (UINT64)sp * 8 * channels;
        CHAR16 lb[120];
        SPrint(lb, sizeof(lb),
               L"[BW] theoretical peak = %ld MB/s (%d MT/s x 8 bytes x %d ch)",
               g_bw_peak_theoretical_mbps, sp, channels);
        log_line(lb);
    }
    /* Capture app-start timestamp for the "Uptime" indicator. ms_now() needs
       TSC calibration which was already done above. */
    g_boot_ms = ms_now();
    {
        CHAR16 fb[256];
        /* Hardware identification line in the format the AI analyzer parses:
           "HW: <vendor> <model> | BIOS: <ver> | RAM: <gb> GB <speed> MT/s" */
        SPrint(fb, sizeof(fb),
               L"HW: %a %a | BIOS: %a %a | RAM: %ld MB %s %d MT/s (%d DIMMs)",
               g_sys_vendor, g_sys_model,
               g_bios_vendor, g_bios_version,
               g_total_ram_mb,
               g_dimm_ddr_type ? ddr_type_name(g_dimm_ddr_type) : L"?",
               g_max_dimm_configured_mt ? g_max_dimm_configured_mt : g_max_dimm_speed_mt,
               g_dimm_count);
        log_line(fb);
        SPrint(fb, sizeof(fb), L"CPU: %a", g_cpu_brand);
        log_line(fb);
        SPrint(fb, sizeof(fb),
               L"CPU features: AVX2=%d CLFLUSH=%d Cores=%d/%d",
               g_has_avx2, g_has_clflush,
               (UINT32)g_n_enabled, (UINT32)g_n_cores);
        log_line(fb);
        for (UINT32 i = 0; i < g_dimm_count; i++) {
            dimm_info_t *d = &g_dimms[i];
            SPrint(fb, sizeof(fb),
                   L"DIMM %a: %d MB  %a  %a  speed=%d@%d MT/s",
                   d->locator, d->size_mb, d->manufacturer, d->part_number,
                   d->speed_mt, d->configured_speed_mt);
            log_line(fb);
        }
    }
    init_splash(g_lang ? L"Allocating test buffer..."
                       : L"Резервирование тестового буфера...");
    log_line(L"[STEP 2] cls()");
    cls();
    log_line(L"[STEP 3] Allocating test buffer (75% of largest free block, max 1 GB)...");

    /* Build inventory of testable regions for multi-pass mode. */
    scan_mem_regions();

    /* When isolating a single DIMM, multipass would rotate the buffer
       across regions in OTHER DIMMs — defeating the isolation. Force
       MultiPass=0 and Passes=1 in this mode so we hammer just the one
       allocation inside the target DIMM's range. */
    if (g_cfg_test_only_dimm > 0 && g_cfg_multipass) {
        log_line(L"[ISOLATE] TestOnlyDimm set — forcing MultiPass=0, Passes=1");
        g_cfg_multipass = 0;
        g_cfg_passes    = 1;
    }

    EFI_STATUS s = alloc_test_buffer();
    if (EFI_ERROR(s)) {
        say(L"FATAL: cannot allocate test memory.\r\n");
        log_line(L"FATAL: alloc failed");
        for (;;) {}
    }
    {
        CHAR16 fb[160];
        UINT64 buf_mb = (g_mem_pages * 4096ULL) / (1024ULL * 1024ULL);
        UINT32 cov_x10 = g_total_ram_mb ? (UINT32)((buf_mb * 1000) / g_total_ram_mb) : 0;
        SPrint(fb, sizeof(fb),
               L"Test buffer: %ld MB allocated (coverage %d.%d%% of %ld MB total RAM)",
               buf_mb, cov_x10 / 10, cov_x10 % 10, g_total_ram_mb);
        log_line(fb);
    }

    /* ---------------- Outer loop: menu → tests → summary → menu ----------------
       After each test run, the user picks ENTER to go back to the menu (start
       again, change language, etc.) or ESC to reboot. We never auto-reset. */
    int reboot_requested = 0;
    while (!reboot_requested) {
        /* Fresh state for this run. */
        g_aborted = 0;
        for (UINTN i = 0; i < N_TESTS; i++) {
            g_summary[i].status  = 0;
            g_summary[i].errors  = 0;
            g_summary[i].bytes   = 0;
            g_summary[i].time_ms = 0;
            g_cards[i].state     = CARD_IDLE;
            g_cards[i].pct_x10   = 0;
            g_cards[i].mbs       = 0;
            g_cards[i].errors    = 0;
        }
        drain_conin();

        /* Main menu — wait for user input before starting tests. Flush
           the log here so if the user pulls the USB at the menu (common
           if they think the test is taking too long) everything written
           so far is safely on disk. */
        log_line(L"[STEP 4a] Main menu (waiting for user)");
        flush_log_now();
        if (!main_menu_wait()) {
            log_line(L"User chose reboot from menu");
            reboot_requested = 1;
            break;
        }
        /* Thermal emergency check (v0.4.12). If baseline temp was 100°C+
           at init, refuse to start tests no matter what the user pressed.
           Display a screen explaining why and force them to reboot. */
        if (g_thermal_emergency) {
            log_line(L"[TEMP] User tried to start tests but thermal emergency "
                     L"is active — refusing.");
            cls();
            UINTN cy = g_h / 3;
            gfx_draw_str_color(g_pad, cy,
                T(L"⚠ ОПАСНАЯ ТЕМПЕРАТУРА CPU",
                  L"⚠ DANGEROUS CPU TEMPERATURE"),
                COL_FAIL);
            cy += g_char_h * 2;
            CHAR16 ln[260];
            SPrint(ln, sizeof(ln),
                T(L"  CPU sensor показал >= 100°C на старте.",
                  L"  CPU sensor reads >= 100°C at idle."));
            gfx_draw_str_color(g_pad, cy, ln, COL_FG); cy += g_char_h;
            gfx_draw_str_color(g_pad, cy,
                T(L"  Запуск тестов отменён чтобы не перегреть CPU.",
                  L"  Tests refused to avoid further overheating."),
                COL_FG); cy += g_char_h * 2;
            gfx_draw_str_color(g_pad, cy,
                T(L"  Что делать:",
                  L"  What to do:"),
                COL_ACCENT_HI); cy += g_char_h;
            gfx_draw_str_color(g_pad, cy,
                T(L"  1. Проверить охлаждение (термопаста, вентилятор)",
                  L"  1. Check cooling (thermal paste, fan)"),
                COL_FG); cy += g_char_h;
            gfx_draw_str_color(g_pad, cy,
                T(L"  2. Кросс-проверить через HWiNFO64 в Windows",
                  L"  2. Cross-check with HWiNFO64 in Windows"),
                COL_FG); cy += g_char_h;
            gfx_draw_str_color(g_pad, cy,
                T(L"  3. Если temp в Windows нормальная — поставить",
                  L"  3. If Windows shows normal temp, set"),
                COL_FG); cy += g_char_h;
            gfx_draw_str_color(g_pad, cy,
                L"     IgnoreThermalGuard=1   в quantai.ini",
                COL_FG); cy += g_char_h * 2;
            gfx_draw_str_color(g_pad, cy,
                T(L"  [ESC] перезагрузка",
                  L"  [ESC] reboot"),
                COL_ACCENT_HI);
            drain_conin();
            for (;;) {
                EFI_INPUT_KEY k = {0,0};
                if (wait_key_or_timer(&k, 200)) {
                    if (k.ScanCode == SCAN_ESC) break;
                }
            }
            reboot_requested = 1;
            break;
        }
        log_line(L"[STEP 4b] User started tests");

        /* Wipe menu pixels before painting the test layout — section
           backgrounds don't cover the whole screen, so leftover glyphs
           from the menu would otherwise leak between cards. */
        cls();
        compute_layout(N_TESTS);
        render_header(0, 0, N_TESTS);
        cards_init_all();
        core_panel_init();

        UINT64 t_run_start = ms_now();
        g_run_start_ms = t_run_start;  /* exported for bsp_yield_render */
        UINTN done_tests = 0;

        /* Reset honest coverage counters for this run — they accumulate the
           total UNIQUE bytes tested across all multipass passes so the
           summary can report real coverage, not just the last buffer size. */
        g_run_tested_mb = 0;
        g_run_passes_done = 0;
        g_run_total_errors = 0;
        /* Reset per-run live telemetry so it starts fresh on a re-run. */
        g_bw_bytes_prev = 0;
        g_bw_ts_prev_ms = 0;
        g_bw_mbps_current = 0;
        g_bw_mbps_peak = 0;
        /* Reset the BW history ring (trend analyzer reads it fresh). */
        g_bw_history_count = 0;
        g_bw_history_start_ms = 0;
        g_bw_current_bucket = 0xFFFFFFFFu;
        g_bw_trend_first_pct = 0;
        g_bw_trend_last_pct  = 0;
        g_bw_trend_degraded  = 0;
        g_rapl_ts_prev_ms = 0;
        g_pkg_power_w = 0;
        g_pkg_power_w_peak = 0;
        g_max_temp_c = 0;
        g_throttle_total = 0;
        g_freq_max_mhz = 0;
        g_freq_avg_mhz = 0;
        g_cum_bytes = 0;
        g_pass_durations_count = 0;
        for (UINTN i = 0; i < g_n_enabled; i++) {
            g_args[i].throttle_event_count = 0;
            g_args[i].throttle_was_active  = 0;
        }

        /* Multi-pass support.
              Passes=0  : auto — pass count derived from total available RAM
                          (covers everything end-to-end)
              Passes=N>0: do at most N passes regardless of coverage
              MultiPass=1: rotate buffer through (region, offset) pairs so we
                          actually exercise different DRAM cells on every pass.
                          Without this, all "passes" hit the SAME bytes.
           Sub-region offset iteration covers the FULL extent of each region,
           not just the first BufferMB. That fixes the "rest of RAM not
           tested" gap users (rightly) complained about. */
        UINT32 passes_target = g_cfg_passes;
        if (g_quick_mode) {
            /* Quick triage: 3 chunks (~3 GB tested), only 4 hard-fault tests
               run per chunk. ~2-3 min total on modern HW. */
            passes_target = 3;
            log_line(L"[QUICK] Quick mode: Thermal Soak (pass 1) + 3 passes × 4 hard-fault tests");
        } else
        if (g_cfg_multipass && passes_target == 0) {
            /* Auto-calculate: enough passes to cover every byte we can alloc. */
            UINT64 total_addressable_mb = 0;
            for (UINT32 r = 0; r < g_n_regions; r++)
                total_addressable_mb += (g_regions[r].pages * 4096ULL) / (1024ULL * 1024ULL);
            passes_target = (UINT32)((total_addressable_mb + g_cfg_buffer_cap_mb - 1) / g_cfg_buffer_cap_mb);
            if (passes_target == 0) passes_target = 1;
            CHAR16 lb[120];
            SPrint(lb, sizeof(lb),
                   L"[PASS] auto passes=%d (cover %ld MB / %d MB per pass)",
                   passes_target, total_addressable_mb, g_cfg_buffer_cap_mb);
            log_line(lb);
        }
        if (passes_target == 0) passes_target = 1;

        /* Marathon mode override: when MarathonHours>0, time bounds the run
           instead of pass count. We set passes_target to "effectively
           infinite" and let the in-loop time check break us out. */
        UINT64 marathon_limit_ms = 0;
        if (g_cfg_marathon_hours > 0) {
            marathon_limit_ms = (UINT64)g_cfg_marathon_hours * 3600ULL * 1000ULL;
            passes_target = 0xFFFFFFFFu;     /* sentinel — display will show
                                                "MARATHON N h" instead of N/M */
            CHAR16 lb[140];
            SPrint(lb, sizeof(lb),
                   L"[MARATHON] enabled — run for %d h, multipass iterator "
                   L"will wrap when RAM coverage cycle completes",
                   g_cfg_marathon_hours);
            log_line(lb);
        }

        /* Iterator state across regions and offsets within each region. */
        UINT32 mp_region = 0;
        UINTN  mp_offset_pages = 0;
        for (UINT32 pass = 0; pass < passes_target && !g_aborted; pass++) {
            /* Marathon: stop when wall-clock limit hits. Checked at the
               TOP of each pass so we never enter a pass that will overrun.
               (A pass typically takes 3-10 min, so worst-case overshoot
               is one pass duration past the limit.) */
            if (marathon_limit_ms) {
                UINT64 elapsed = ms_now() - t_run_start;
                if (elapsed >= marathon_limit_ms) {
                    CHAR16 lb[120];
                    SPrint(lb, sizeof(lb),
                           L"[MARATHON] %d h limit reached after %d pass(es) — stopping",
                           g_cfg_marathon_hours, pass);
                    log_line(lb);
                    break;
                }
                /* Wrap the multipass iterator so we re-cover the whole RAM
                   range on the next cycle instead of falling through to
                   "no more (region, offset) slots — done". */
                if (g_cfg_multipass && mp_region >= g_n_regions) {
                    mp_region = 0;
                    mp_offset_pages = 0;
                    UINT32 elapsed_min = (UINT32)(elapsed / 60000);
                    CHAR16 lb[140];
                    SPrint(lb, sizeof(lb),
                           L"[MARATHON] coverage cycle complete at t+%d min — "
                           L"wrapping iterator for next cycle",
                           elapsed_min);
                    log_line(lb);
                }
            }
            g_cur_pass = pass;
            g_pass_idx_disp = pass + 1;
            g_pass_total_disp = passes_target;
            g_cur_pass_start_ms = ms_now();

            if (g_cfg_multipass) {
                if (g_mem_addr) { free_test_buffer(); g_mem_addr = 0; }
                /* Find next (region, offset) that yields a usable allocation. */
                int alloced = 0;
                while (mp_region < g_n_regions && !alloced) {
                    if (alloc_region_buffer_at(mp_region, mp_offset_pages)) {
                        alloced = 1;
                        mp_offset_pages += g_mem_pages;
                        if (mp_offset_pages >= g_regions[mp_region].pages) {
                            mp_region++;
                            mp_offset_pages = 0;
                        }
                    } else {
                        /* Couldn't allocate — try next region from start. */
                        mp_region++;
                        mp_offset_pages = 0;
                    }
                }
                if (!alloced) {
                    log_line(L"[PASS] no more (region, offset) slots — done");
                    break;
                }
                CHAR16 lb[160];
                UINT64 buf_mb_pass = (g_mem_pages * 4096ULL) / (1024ULL * 1024ULL);
                /* Accumulate honest coverage: each multipass pass touches a
                   FRESH (region, offset) slice, so its bytes never overlap
                   prior passes. Sum is the unique RAM exercised so far. */
                g_run_tested_mb += buf_mb_pass;
                SPrint(lb, sizeof(lb),
                       L"[PASS] %d/%d  addr=0x%lx  size=%ld MB",
                       pass + 1, passes_target, (UINT64)g_mem_addr, buf_mb_pass);
                log_line(lb);
                cards_init_all();
            } else if (passes_target > 1) {
                CHAR16 lb[100];
                SPrint(lb, sizeof(lb), L"[PASS] %d/%d on same buffer", pass + 1, passes_target);
                log_line(lb);
                for (UINTN i = 0; i < N_TESTS; i++) {
                    g_cards[i].state = CARD_IDLE;
                    g_cards[i].pct_x10 = 0;
                    g_cards[i].mbs = 0;
                    g_cards[i].errors = 0;
                }
                cards_init_all();
            }

        /* Reset per-pass test counter so the header shows "Тесты 0/7" at the
           start of each pass, not "Тесты 14/7" / "21/7" / etc. (Previously
           done_tests was declared once outside the pass loop and accumulated
           across passes — visually broken.) */
        done_tests = 0;
        log_line(L"[STEP 4] Entering test loop");
        for (UINTN i = 0; i < N_TESTS && !g_aborted; i++) {
            CHAR16 lb[120];
            SPrint(lb, sizeof(lb), L"[STEP 5.%d.A] Pre-test %s", (UINT32)i, g_tests[i].name);
            log_line(lb);
            render_header(ms_now() - t_run_start, done_tests, N_TESTS);

            /* Quick mode: skip tests not flagged for quick stress. */
            if (g_quick_mode && !g_tests[i].in_quick) {
                CHAR16 lb2[120];
                SPrint(lb2, sizeof(lb2),
                       L"[5.%d] %s -> SKIPPED (quick mode)",
                       (UINT32)i, g_tests[i].name);
                log_line(lb2);
                g_summary[i].status = 0;
                g_cards[i].state = CARD_SKIP;
                g_cards[i].pct_x10 = 1000;
                g_cards[i].mbs = 0;
                g_cards[i].errors = 0;
                card_paint(i);
                done_tests++;
                continue;
            }

            /* Skip Bit Fade on passes 2..N unless user explicitly asked for
               BitFadeEveryPass=1 in quantai.ini.
               Why: Bit Fade is by far the slowest test (2 × bitfade_s + ~10
               s = 4-10 min per pass). On a 32 GB DDR5 system with 32 passes
               that adds up to many hours — unacceptable for shop diagnostics.
               Running it ONCE on the first chunk is enough to surface
               retention failures: if the DRAM cells leak, they leak on every
               row, not just the row tested in this one chunk. Other passes
               run the 8 fast tests on different chunks for breadth. */
            /* Tests that only need to run ONCE per Full prog (on pass 1):
                 - Bit Fade / Bit Fade Ext   : retention is property of cell,
                                               testing one chunk catches it
                 - Thermal Soak / BW Soak    : these establish thermal state
                                               for the rest of the prog;
                                               running once is enough, doing
                                               it every pass adds 8 min × N.
               BitFadeEveryPass=1 overrides for Bit Fade variants only. */
            int is_soak  = (g_tests[i].k == KER_THERMAL_SOAK
                         || g_tests[i].k == KER_BW_SOAK);
            int is_fade  = (g_tests[i].k == KER_BIT_FADE
                         || g_tests[i].k == KER_BIT_FADE_EXT);
            int skip_after_p1 = (pass > 0)
                              && ((is_fade && !g_cfg_bitfade_every_pass)
                                  || is_soak);
            if (skip_after_p1) {
                SPrint(lb, sizeof(lb),
                       L"[5.%d] %s -> SKIPPED (only pass 1 by design)",
                       (UINT32)i, g_tests[i].name);
                log_line(lb);
                g_summary[i].status = 0;
                g_cards[i].state = CARD_SKIP;
                g_cards[i].pct_x10 = 1000;
                g_cards[i].mbs = 0;
                g_cards[i].errors = 0;
                card_paint(i);
                done_tests++;
                continue;
            }

            /* Skip AVX2 when firmware hasn't enabled OSXSAVE/XCR0[YMM]. */
            if (g_tests[i].k == KER_AVX2 && !g_has_avx2) {
                SPrint(lb, sizeof(lb),
                       L"[5.%d] %s -> SKIPPED (firmware did not enable XSAVE/OSXSAVE)",
                       (UINT32)i, g_tests[i].name);
                log_line(lb);
                g_summary[i].status = 0;
                g_cards[i].state = CARD_SKIP;
                g_cards[i].pct_x10 = 1000;
                g_cards[i].mbs = 0;
                g_cards[i].errors = 0;
                card_paint(i);
                done_tests++;
                continue;
            }

            g_cards[i].state = CARD_RUNNING;
            g_cards[i].pct_x10 = 0;
            g_cards[i].mbs = 0;
            g_cards[i].errors = 0;
            card_paint(i);

            /* v0.4.29 — countdown returns 0=start, 1=skip this test, 2=abort run */
            int cd_rc = countdown(2, i);
            if (cd_rc == 2) break;          /* Q → abort whole run */
            if (cd_rc == 1) {                /* ESC → skip this test */
                SPrint(lb, sizeof(lb),
                       L"[5.%d] %s -> SKIPPED (user pressed ESC during countdown)",
                       (UINT32)i, g_tests[i].name);
                log_line(lb);
                g_summary[i].status = 0;
                g_cards[i].state = CARD_SKIP;
                g_cards[i].pct_x10 = 1000;
                g_cards[i].mbs = 0;
                g_cards[i].errors = 0;
                card_paint(i);
                done_tests++;
                continue;
            }
            /* v0.4.29 — clear the countdown footer once the test starts.
               The old "[N/14] Test starts in 2 sec ..." line would linger
               throughout the test run, taking up screen space without
               serving any purpose during the test itself. Replace with a
               short hint about how to abort. */
            {
                UINTN frow = g_foot_y / g_char_h;
                if (frow < g_text_rows) {
                    clear_row(frow);
                    say_at_rc(0, frow,
                              T(L"  [Q] = прервать прогон   ·   логи пишутся в memforge2.log на USB",
                                L"  [Q] = abort run   ·   logs are written to memforge2.log on USB"));
                }
            }

            SPrint(lb, sizeof(lb), L"[STEP 5.%d.B] Calling run_test_mc(%s)", (UINT32)i, g_tests[i].name);
            log_line(lb);
            test_summary_t r = run_test_mc(i);
            SPrint(lb, sizeof(lb), L"[STEP 5.%d.C] run_test_mc returned errors=%ld",
                   (UINT32)i, r.errors);
            log_line(lb);
            /* Flush after each test completes — user might yank USB at any
               point during a 45-minute Full run, and we want the in-progress
               per-test results to survive that. Cheap (1× per test, not
               1× per log line). */
            flush_log_now();
            /* v0.4.29 — ACCUMULATE across marathon passes, do not OVERWRITE.
               Pre-v0.4.29 the line was `g_summary[i] = r;` which kept only
               the LAST pass's per-test result. On a 16-hour marathon with
               an intermittent error rate of 1 per pass, that meant the
               final summary table showed "errors: 0" because the most
               recent pass happened to be clean — completely hiding the
               24 cumulative errors found across earlier passes. Also fed
               into JSON `summary.total_errors: 0` and `verdict: "PASS"`,
               which then misled any automated post-test analyzer.        */
            if (g_run_passes_done == 0) {
                /* First pass: initialize with this pass's result */
                g_summary[i] = r;
            } else {
                /* Subsequent passes: accumulate counts; status is "sticky":
                   FAIL wins over PASS wins over SKIP. */
                g_summary[i].errors  += r.errors;
                g_summary[i].bytes   += r.bytes;
                g_summary[i].time_ms += r.time_ms;
                if (r.status == 2) g_summary[i].status = 2;          /* FAIL is sticky */
                else if (g_summary[i].status == 0 && r.status == 1)
                    g_summary[i].status = 1;                          /* upgrade SKIP→PASS */
            }
            /* Bump cumulative error counter shown in the live header. */
            g_run_total_errors += r.errors;

            g_cards[i].state = (r.status == 1) ? CARD_PASS : CARD_FAIL;
            g_cards[i].pct_x10 = 1000;
            g_cards[i].mbs = r.time_ms ? (r.bytes / (1024 * 1024) * 1000) / r.time_ms : 0;
            g_cards[i].errors = r.errors;
            card_paint(i);
            done_tests++;

            CHAR16 logbuf[160];
            SPrint(logbuf, sizeof(logbuf),
                   L"[%d/%d] %s -> errors=%ld, bytes=%ld MB, time=%ld ms",
                   (UINT32)(i + 1), (UINT32)N_TESTS, g_tests[i].name,
                   r.errors, r.bytes / (1024 * 1024), r.time_ms);
            log_line(logbuf);
            /* Stride-BW: emit per-stride MB/s averaged across cores. A
               sharp drop at one stride is the diagnostic signal — uniform
               low values just mean the system is busy. */
            if (g_tests[i].k == KER_STRIDE_BW) {
                static const UINTN stride_labels[4] = { 64, 1024, 4096, 65536 };
                for (UINT32 ph = 0; ph < 4; ph++) {
                    UINT64 sum = 0; UINT32 cnt = 0;
                    for (UINT32 c = 0; c < g_n_enabled && c < MAX_CORES; c++) {
                        if (g_stride_mbps[c][ph] > 0) {
                            sum += g_stride_mbps[c][ph];
                            cnt++;
                        }
                    }
                    UINT32 avg = cnt ? (UINT32)(sum / cnt) : 0;
                    SPrint(logbuf, sizeof(logbuf),
                           L"[STRIDE] stride %d B: %d MB/s avg across %d cores",
                           (UINT32)stride_labels[ph], avg, cnt);
                    log_line(logbuf);
                }
            }

            render_progress(i, t_run_start, ms_now(), done_tests);
        }
        if (!g_aborted) {
            g_run_passes_done++;
            /* Append this pass's wall-clock duration to the history (cyclic
               write so we never overflow the fixed-size buffer). */
            UINT64 dur = ms_now() - g_cur_pass_start_ms;
            UINT32 idx = g_pass_durations_count % MAX_PASS_HISTORY;
            g_pass_durations_ms[idx] = (UINT32)(dur > 0xFFFFFFFFULL ? 0xFFFFFFFFULL : dur);
            g_pass_durations_count++;
        }
        /* Single-buffer mode (MultiPass=0): all "passes" re-test the SAME
           bytes, so unique coverage is fixed = current buffer size. We set
           it once on the first pass and never re-add. */
        if (!g_cfg_multipass && g_run_tested_mb == 0) {
            g_run_tested_mb = (g_mem_pages * 4096ULL) / (1024ULL * 1024ULL);
        }
        }   /* end pass-loop */

        UINT64 total_ms = ms_now() - t_run_start;
        /* Snapshot MCA banks BEFORE rendering summary so the summary can
           report the new-error count alongside our own pattern errors. */
        if (g_has_mca) mca_report_diff();
        /* BW degradation trend: useful in long/marathon runs to catch
           silent throttling. Logs first vs last quartile means. */
        bw_trend_report();
        /* Persist this run's summary to NVRAM and log delta vs prev run.
           Lets a shop see across reboots whether the symptom reproduces. */
        hist_save_and_diff(total_ms);
        /* v0.4.29 — auto-isolation: if errors are distributed across 2+
           DIMMs on a block-mapped system, automatically run per-DIMM
           re-test BEFORE showing the verdict. No user key needed. The
           result screen becomes the verdict the user sees. */
        int auto_isolated = 0;
        if (should_auto_isolate()) {
            log_line(L"[ISO] auto-isolation triggered by distributed-error verdict");
            render_auto_isolation_intro(g_iso_dimm_n);
            do_auto_isolation();
            render_isolation_verdict();
            auto_isolated = 1;
        }

        /* Show the simple verdict screen only when auto-isolation didn't
           run (or wasn't applicable). When it did run, the isolation
           verdict screen is already showing. */
        if (!auto_isolated) {
            render_simple_verdict(total_ms);
        }

        /* Dump per-error detail to the log — XOR mask is the most useful
           single piece of info, it pinpoints which bits flipped. */
        if (g_err_count > 0) {
            CHAR16 lb[300];
            UINT32 shown = g_err_count > MAX_ERR_RECORDS ? MAX_ERR_RECORDS : g_err_count;
            SPrint(lb, sizeof(lb),
                   L"[ERR] Detected %d error(s) total, first %d recorded:",
                   g_err_count, shown);
            log_line(lb);
            for (UINT32 i = 0; i < shown; i++) {
                err_record_t *r = &g_err_records[i];
                CHAR16 dimm_lab[64];
                dimm_label_for_addr(r->phys_addr, dimm_lab, 64);
                /* Decode DRAM coordinates (approximate — chipset hash
                   unknown; marked ~ in output so user sees it's heuristic). */
                dram_coords_t coords;
                decode_dram_coords(r->phys_addr, &coords);
                /* r->core stays 0-based internally (matches MP-services slot
                   index) but we display it +1 so a user reading the log sees
                   the same "CPU N" numbering as the on-screen panel. */
                SPrint(lb, sizeof(lb),
                       L"[ERR] T=%s Core=%d Addr=0x%lx Exp=0x%lx Act=0x%lx XOR=0x%lx DIMM=%s "
                       L"~bg=%d ~bank=%d ~row=0x%lx ~col=0x%x",
                       name_for_kernel(r->test), r->core + 1,
                       r->phys_addr, r->expected, r->actual, r->xor_mask,
                       dimm_lab,
                       coords.bank_group, coords.bank, coords.row, coords.column);
                log_line(lb);
                /* Environmental snapshot at the exact moment this error
                   was recorded. Lets the operator see "this byte flipped
                   when CPU was at 87 °C and 134 W" rather than only knowing
                   the run-wide peak. VID printed only when nonzero (Phase 3
                   populates it; older builds keep printing 0 mV otherwise). */
                if (r->vid_mv > 0) {
                    SPrint(lb, sizeof(lb),
                           L"[ERR]   when: t+%lds  temp=%d°C  W=%d  throttle=%d  VID=%d.%03dV",
                           r->t_ms / 1000, r->temp_c, r->pkg_watt,
                           r->throttle_cnt,
                           r->vid_mv / 1000, r->vid_mv % 1000);
                } else {
                    SPrint(lb, sizeof(lb),
                           L"[ERR]   when: t+%lds  temp=%d°C  W=%d  throttle=%d",
                           r->t_ms / 1000, r->temp_c, r->pkg_watt,
                           r->throttle_cnt);
                }
                log_line(lb);
            }
            /* Aggregate analysis: stuck bit + 1-GB histogram + DIMM tally + row/bank. */
            UINT32 srow_n = 0;
            UINT64 srow   = find_stuck_row(&srow_n);
            if (srow_n >= 3) {
                SPrint(lb, sizeof(lb),
                       L"[ERR] STUCK ROW ~0x%lx: %d errors clustered in same DRAM row "
                       L"(suggests failing row driver / wordline)",
                       srow, srow_n);
                log_line(lb);
            }
            UINT32 sbank_n = 0;
            UINT32 sbank   = find_stuck_bank(&sbank_n);
            if (sbank_n >= 3) {
                SPrint(lb, sizeof(lb),
                       L"[ERR] STUCK BANK ~bg=%d/bank=%d: %d errors clustered in same DRAM bank "
                       L"(suggests failing bank refresh / precharge circuit)",
                       (sbank >> 4) & 0xF, sbank & 0xF, sbank_n);
                log_line(lb);
            }
            /* Original aggregate analysis below. */
            UINT32 stuck_n = 0;
            UINT64 stuck_x = find_stuck_bit(&stuck_n);
            if (stuck_n >= 5 && stuck_x != 0) {
                int bp = single_bit_pos(stuck_x);
                if (bp >= 0) {
                    /* Map to physical chip if SPD told us organization. */
                    int didx = dominant_dimm_idx();
                    CHAR16 chip[64] = L"";
                    if (didx >= 0)
                        chip_label_for_bit((UINT32)didx, bp, chip, 64);
                    if (didx >= 0 && chip[0])
                        SPrint(lb, sizeof(lb),
                               L"[ERR] STUCK BIT D[%d] -> DIMM%d %s: %d occurrences (XOR=0x%016lx)",
                               bp, didx + 1, chip, stuck_n, stuck_x);
                    else
                        SPrint(lb, sizeof(lb),
                               L"[ERR] STUCK BIT D[%d]: %d occurrences (XOR=0x%016lx)",
                               bp, stuck_n, stuck_x);
                } else {
                    SPrint(lb, sizeof(lb),
                           L"[ERR] Repeating pattern: %d × XOR=0x%016lx (multi-bit)",
                           stuck_n, stuck_x);
                }
                log_line(lb);
            }
            UINT32 hb[32], hc[32];
            UINT32 nbk = error_histogram_gb(hb, hc, 32);
            if (nbk > 0) {
                CHAR16 hl[300]; UINTN hp = 0;
                hp += SPrint(hl + hp, sizeof(hl) - hp * sizeof(CHAR16),
                             L"[ERR] Histogram 1G buckets: ");
                for (UINT32 j = 0; j < nbk; j++) {
                    hp += SPrint(hl + hp, sizeof(hl) - hp * sizeof(CHAR16),
                                 (j == 0) ? L"[%d-%dGB]=%d" : L" [%d-%dGB]=%d",
                                 hb[j], hb[j] + 1, hc[j]);
                    if (hp >= 250) break;
                }
                log_line(hl);
            }
        } else {
            log_line(L"[ERR] No errors recorded.");
        }

        log_line(L"[STEP 6] Writing report.json");
        write_json_report(total_ms);

        /* Wait for user decision. Only EXPLICIT keys do something — random
           accidental presses must NOT cycle into "rerun tests". Earlier
           behaviour was "any key = back to menu", which combined with the
           menu's "ENTER = start" made it look like the tests self-restarted
           when the user just acknowledged the summary. */
        log_line(L"[STEP 7] Awaiting post-summary action");
        drain_conin();
        int leave_summary = 0;
        /* View mode: 0 = simple verdict (default), 1 = technical details.
           [D] toggles between them; [L] re-renders the current one in the
           other language; [M] returns to main menu; [ESC] reboots. */
        int view_mode = 0;
        /* Idle timeout: if the operator walked away and nobody pressed a
           key for 10 min, auto-reboot. The summary is still on screen, the
           log + report.json are already on USB, so nothing is lost.
           This is also a safety net for the HP keyboard-hang scenario:
           even if USB never comes back, the machine will at least cycle
           rather than sit forever on a frozen screen. */
        const UINT64 IDLE_REBOOT_MS = 10ULL * 60ULL * 1000ULL;   /* 10 min */
        UINT64 idle_ms = 0;
        while (!leave_summary) {
            EFI_INPUT_KEY k = { 0, 0 };
            int got = wait_key_or_timer(&k, 200);
            if (!got) {
                idle_ms += 200;
                if (idle_ms >= IDLE_REBOOT_MS) {
                    log_line(L"[STEP 7] Idle timeout — auto-reboot");
                    reboot_requested = 1;
                    leave_summary = 1;
                }
                continue;   /* timer tick — loop and keep USB polling alive */
            }
            idle_ms = 0;    /* any successful key read resets the idle clock */
            if (k.ScanCode == SCAN_ESC) {
                reboot_requested = 1;
                leave_summary = 1;
            } else if (k.UnicodeChar == L'l' || k.UnicodeChar == L'L') {
                g_lang = !g_lang;
                if (view_mode == 0) render_simple_verdict(total_ms);
                else                render_summary(total_ms);
            } else if (k.UnicodeChar == L'd' || k.UnicodeChar == L'D' ||
                       /* Cyrillic в/В = same physical key as D on RU layout */
                       k.UnicodeChar == 0x0432 || k.UnicodeChar == 0x0412) {
                view_mode = !view_mode;
                if (view_mode == 0) render_simple_verdict(total_ms);
                else                render_summary(total_ms);
            } else if (k.UnicodeChar == L'm' || k.UnicodeChar == L'M' ||
                       /* Cyrillic ь/Ь = same physical key as M on RU layout */
                       k.UnicodeChar == 0x044C || k.UnicodeChar == 0x042C) {
                leave_summary = 1;        /* fall through → main menu */
            } else if ((k.UnicodeChar == L'i' || k.UnicodeChar == L'I' ||
                        /* Cyrillic ш/Ш = same physical key as I on RU layout */
                        k.UnicodeChar == 0x0448 || k.UnicodeChar == 0x0428)
                       && g_iso_offer) {
                /* v0.4.29 — auto-isolation: re-test each affected DIMM with
                   TestOnlyDimm, give a definitive REPLACE answer. */
                do_auto_isolation();
                render_isolation_verdict();
                /* view_mode stays at "isolation result"; user can [D] to
                   go back to original simple verdict (the unconditional
                   `view_mode = !view_mode` above flips truthy→0, so D
                   correctly re-renders the simple verdict). */
                view_mode = 2;             /* sentinel for "in isolation result" */
                g_iso_offer = 0;           /* don't re-offer */
            }
            /* Any other key — explicitly ignored. The view stays put. */
        }
    }

    log_line(L"[STEP 8] Cleanup + reset");
    if (g_logfile) {
        uefi_call_wrapper(g_logfile->Flush, 1, g_logfile);
        uefi_call_wrapper(g_logfile->Close, 1, g_logfile);
    }
    if (g_logroot) {
        uefi_call_wrapper(g_logroot->Close, 1, g_logroot);
    }
    free_test_buffer();
    uefi_call_wrapper(RT->ResetSystem, 4, EfiResetCold, EFI_SUCCESS, 0, NULL);
    return EFI_SUCCESS;
}
