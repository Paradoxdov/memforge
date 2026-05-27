# MemForge2

**Latest release: [v0.4.24](https://github.com/Paradoxdov/memforge/releases/latest)** — download `MemForge2.efi`, copy to `EFI/BOOT/loader.efi` on a FAT32 USB.

UEFI memory diagnostic tool for shop / repair use. Boots from USB before any
OS loads, runs 14 stress and pattern tests in parallel on every CPU core,
captures hardware ECC events via MCA registers, snapshots environmental
context at every error, and writes a structured report to the USB stick for
offline review.

![Memory failure detected — DDR3 stuck-bit + damaged bank, pinpointed to Samsung DIMM1](docs/screenshot-failure.jpg)

*Real run on a refurbished HP EliteDesk 8300 build: 396 errors in 54 seconds,
pinpointed to **DIMM1**, with stuck-bit and damaged-bank patterns visible.
SPD readout identified the part as Samsung M378B5773CH0-CK0 for warranty
replacement. (Per-chip mapping in this screenshot pre-dates a DDR3 SPD-offset
fix — see commit history; current builds give the correct chip U-number for
DDR3 modules.) The simple verdict screen (shown above) is the default after
every run; press **[D]** for the technical breakdown — full 14-test table,
per-error address/DIMM/DRAM-coord records, MCA bank diff, SPD timings, and
BW degradation trend.*

## What it actually does

- 14 memory tests covering pattern faults, retention, row hammering, address
  decoding, sustained thermal stress, L3-cache cell faults, and stride-based
  bandwidth profiling.
- Direct SPD EEPROM read via SMBus — pulls serial number, manufacturing
  date, JEDEC manufacturer ID and **full primary timings** (CL / tRCD /
  tRP / tRAS / tRFC / tCK) from each DIMM (info SMBIOS Type 17 does not
  expose).
- MCA (Machine Check Architecture) snapshot before/after the run — surfaces
  ECC errors that pattern tests cannot see by design (silently corrected by
  the iMC).
- Heuristic DRAM coordinate decode (bank-group / bank / row / column) with
  stuck-row and stuck-bank cluster detection.
- HWP + PL1/PL2 lift on Intel, CPPC2 lift on AMD — actually pushes the CPU
  to turbo P-state inside UEFI, which most memory testers do not do (their
  tests run at base frequency).
- y-cruncher-style mixed-port Thermal Soak: saturates FMA, shuffle and
  integer ALU ports simultaneously instead of FMA alone.
- **Per-error environmental snapshot** — every recorded error captures
  temp/CPU watts/throttle/VID at the exact moment, so the operator sees
  "this byte flipped at t+182 s, temp 87 °C, 1.235 V" instead of only
  the run-wide peak.
- **Marathon mode** — run for 1-24 hours with multipass-iterator wrap so
  every cycle covers fresh memory. Catches intermittent failures that only
  surface after 2-4 h of sustained thermal load.
- **Cold/warm boot delta** — persists a 96-byte summary to UEFI NVRAM
  after every run; on next boot logs deltas: `errors +3, temp +6 °C, BW
  peak −8 %`. Lets a shop see across reboots whether the symptom
  reproduces.
- **Bandwidth degradation trend** — 1-min BW buckets, first-quartile vs
  last-quartile compare flags >5 % drop as mild / >15 % as severe.
  Catches silent thermal throttling, IMC retry storms, marginal channels.

## Tests

| #  | Test              | Time   | What it catches |
|----|-------------------|--------|-----------------|
|  1 | AVX2 Sustained    | ~10 s  | VRM / IMC sustained-load faults |
|  2 | TRRespass         | ~60 s  | DDR4/5 Row Hammer (Frigo et al., USENIX Sec 2020) |
|  3 | Cache-Eviction    | ~10 s  | DRAM bus errors via CLFLUSH |
|  4 | March-C-          | ~2 s   | 6-phase JEDEC-grade March (van de Goor 1997, 92% coverage) |
|  5 | Thermal Soak      | 3 min  | Errors that only surface at thermal steady-state |
|  6 | BW Soak           | 5 min  | Sustained DRAM bandwidth (memtest86+ analog) |
|  7 | March-RAW         | ~3 s   | Read-after-write dynamic coupling faults |
|  8 | Butterfly         | ~5 s   | Cell crosstalk via checkerboard + neighbour flip |
|  9 | Address Pattern   | ~5 s   | Address-line faults (cell X reads value of Y) |
| 10 | VRM Square-Wave   | ~10 s  | VRM transient response under 10 Hz load square |
| 11 | Random Pattern    | ~6 s   | xorshift64 pseudo-random patterns × 4 |
| 12 | Bit Fade Extended | ~8 min | DRAM retention (write → wait → verify, 2 patterns) |
| 13 | **L3 Cache Stress** | ~10 s | L3 cell faults invisible to DRAM-only tests |
| 14 | **Stride BW**     | ~12 s  | TLB / set-associativity / channel-interleave issues |

## Strengths vs other tools

- **Per-error environmental snapshot** — every error record includes the
  ms-timestamp, peak temperature, CPU power and VID at the moment the
  byte flipped. Lets you distinguish "fails cold at 45 °C" (stuck cell)
  from "fails only above 85 °C" (IMC thermal margin) without re-running.
- **TRRespass** — 8-sided Row Hammer with bank rotation. Bypasses TRR/RFM
  that the naive 2-aggressor Row Hammer in memtest86+ gets defeated by.
  Reference: Frigo et al., *TRRespass: Exploiting the Many Sides of Target
  Row Refresh*, USENIX Security 2020.
- **March-C-** — formal industrial March algorithm. Most consumer testers
  still use walking-1s/0s from the 1980s.
- **L3 Cache Stress** — resident workload tests L3 cells directly. Every
  other tester CLFLUSHes between write and verify, so the L3 storage
  layer never gets observed; marginal L3 cells flip bits we catch here
  or trigger silent ECC fires in MCA bank 1.
- **MCA capture** — reads `MCi_STATUS` MSRs before and after the test, diffs
  to surface ECC corrections that pattern tests by design cannot see.
- **Full SPD via SMBus** — direct ICH/PCH SMBus EEPROM read on Intel,
  FCH SMBus on AMD. Pulls serial number, manufacturing date, JEDEC
  bank/code, and the complete primary timing set
  (CL-tRCD-tRP-tRAS-tRFC). Detects XMP/EXPO vs JEDEC at a glance.
- **DDR5 SPD5 Hub probe** — reads SPD5118 hub device-info MRs and the
  SMBus-side PEC error counter. (On-die ECC counters per DDR5 die live in
  MR48-51 inside the DRAM and require an iMC mailbox we don't implement
  — but the SMBus PEC counter is the next-best signal.)
- **HWP + PL1/PL2 lift** (Intel) / **CPPC2 lift** (AMD) — programs the
  per-logical-CPU performance-state MSRs and lifts package power limits
  so the CPU actually runs at turbo during the stress kernels. PassMark
  explicitly admit on their own forum that their tool does not do this.
- **Error localization** — 5 mechanisms:
  1. DIMM name via SMBIOS Type 20 (physical address → DIMM slot)
  2. Per-chip stuck-bit mapping (bit position → chip U-designator)
  3. Stuck-bit detection via XOR-mask repetition
  4. Stuck-row / stuck-bank detection via DRAM-coord heuristic
  5. 1-GB histogram of error addresses

  **Honest scope of (1)**: SMBIOS Type 20 maps each physical address to
  the slot the firmware says owns it. On servers / NUMA / single-channel
  / non-interleaved consumer setups this gives the exact bad DIMM. On a
  typical dual-channel desktop the iMC interleaves addresses at 64-byte
  granularity between the channel pair, so a single bad chip on ONE stick
  produces errors that appear distributed across both sticks in the pair.
  In that case (auto-detected since v0.4.18) the verdict tells the user
  plainly "REPLACE ONE of A2 / B2 — physically remove one and retest to
  identify the bad stick" instead of confidently picking one. We narrow
  from "1 of 4-8 sticks" to "1 of the 2 in this pair"; that's worse than
  the README screenshot's "DIMM1 pinpointed" (that was a non-interleaved
  DDR3 build) but still a lot more than memtest86+ which only reports
  raw addresses.
- **Marathon mode** — `MarathonHours=N` (1-24) keeps cycling tests for N
  hours. Multipass iterator wraps when RAM coverage cycle completes, so
  every cycle covers fresh (region, offset) pairs. Catches the
  intermittent-failure class that 30-min runs miss but real customers hit
  once a week.
- **Cold/warm boot delta** — UEFI NVRAM persistent record of each run.
  Surfaces regressions across reboots with explicit warnings (`⚠ 3 new
  errors since last run`, `⚠ temp rose 6 °C — check airflow/paste`).
- **Bandwidth degradation trend** — 1-min BW buckets, first-vs-last
  quartile compare. Catches silent thermal throttling and IMC retry
  storms even when no pattern errors fire.
- **Stride-sweep BW** — drop at exactly one stride pinpoints TLB issues,
  cache set conflicts, or channel-interleave bugs.
- **SMBus signal integrity probe** — 16 repeated SPD-byte reads per
  slot; mismatches/NAKs flagged as I²C SI warning.
- **Per-DIMM isolation** — `TestOnlyDimm=N` restricts the test buffer to
  one DIMM's physical address range (via SMBIOS Type 20 + UEFI
  `AllocateAddress`). Lets you verify each stick separately without
  removing the others. Perfect isolation on non-interleaved memory; on
  dual-channel desktops you isolate per channel pair.
- **DDR5-aware** — auto-tunes Bit Fade for on-die ECC, applies 2×
  multiplier to Row Hammer activations to compensate for TRR/RFM,
  detects XMP/EXPO over-JEDEC speeds.

## Output

After the test, on the USB next to `loader.efi`:

- `memforge2.log` — full run log with timestamps, per-test results, SPD
  per DIMM with full timings, MCA bank diffs, per-error records with
  environmental snapshots, BW trend verdict, cold/warm boot delta, stride
  per-stride MB/s, SMBus signal integrity.
- `report.json` — structured data including everything above. Each error
  record carries `at: {t_ms, temp_c, pkg_w, throttle, vid_mv}` for
  context-aware AI analysis. Plain JSON, easy to feed into any downstream
  tool.

## Building

Requires MSYS2 + mingw-w64 GCC + gnu-efi headers/library.

```bash
make
```

Produces `MemForge2.efi`. Copy onto a FAT-formatted USB at
`EFI/BOOT/loader.efi`, along with `quantai.ini` at root. Wrap with the
Linux Foundation's `PreLoader.efi` if you want to keep Secure Boot enabled
(MOK enrollment via `HashTool.efi`).

If Windows Defender Smart App Control blocks gcc, run `WHITELIST_MSYS2.bat`
once as admin.

## Configuration

`quantai.ini` on the USB root holds runtime knobs:

```ini
[Run]
Passes=0            ; 0 = auto from RAM size / BufferMB
MultiPass=1         ; rotate buffer across regions to cover all RAM
MaxCores=0          ; 0 = use all enabled cores
EnableAVX=1
;BufferMB=1024      ; auto-scaled by RAM size if commented out
;BitFadeSeconds=60  ; DDR4 default; DDR5 auto-bumped to 120
;BitFadeEveryPass=0 ; only on pass 1 by default
;TestOnlyDimm=0     ; 1..N = isolate to that DIMM slot
;MarathonHours=0    ; 0 = off, 1..24 = run for N hours

[Meta]
Version=0.4.24
Language=en         ; "ru" or "en"

[Display]
;EnableAA=0         ; 1 = AA via direct framebuffer (faster, less compatible)
;ForceBlt=0         ; 1 = always use GOP Blt (slower, max compatibility)
;Width=1920         ; manual GOP mode override
;Height=1080
;FontScale=0        ; 0=auto, 1=1×, 2=2× (for 4K displays)
```

The `[AI]` section that may appear in your local `quantai.ini` is consumed
by an external post-test analyzer (not part of this repo) and is ignored by
the tester itself.

## How this was built

This project is a collaboration: code written by Claude (Anthropic LLM)
under direction from a PC-assembly tech who provides the domain expertise,
use cases, and real-hardware validation. The repository owner is not a
systems programmer; Claude handles C, UEFI APIs, MSRs, SMBus protocol.
See the article on Habr for the longer story.

## License

MIT — see [`LICENSE`](LICENSE). Permits commercial use, modification,
distribution, and private use; provided as-is, no warranty.

## Acknowledgements

- Frigo, Giuffrida, Bos, Razavi — TRRespass attack (USENIX Sec 2020)
- A. J. van de Goor — March-C- algorithm (1997)
- Linux Foundation — `PreLoader.efi` / `HashTool.efi` Secure Boot shims
- gnu-efi project — UEFI headers and library
