# How MemForge2 names the faulty DIMM without swapping

This document explains the method MemForge2 uses to attribute a memory error to
the **exact physical stick** (by SPD serial number and slot) on a fully
populated, interleaved system — without removing or swapping modules.

It builds directly on the row-conflict timing side-channel from
**Pessl, Gruss, Maurice, Schwarz, Mangard — "DRAMA: Exploiting DRAM Addressing
for Cross-CPU Attacks" (USENIX Security 2016).** We use the same idea for the
opposite, benign purpose: field diagnostics.

> Code excerpts below are accurate **as of v0.4.66**. The source of truth is
> `MemForge2.src.c`; if the code has moved on, trust the file, not this doc.

---

## 1. The problem

Memory errors are caught by **physical address**. Translating "address → slot"
is the hard part:

- **SMBIOS Type 20** maps an address range to a DIMM locator string, but on a
  dual-channel desktop the integrated memory controller (iMC) **interleaves**
  addresses between the two channels. Type 20 doesn't model the interleave, so a
  single bad chip on one stick gets reported against whichever locator owns the
  low address range — the same wrong slot every time, no matter which physical
  slot holds the faulty module.
- **Per-rank / per-DIMM error counters** exist on server iMCs but **not** on
  Intel client parts (Haswell etc.).

So the slot label is the weak link. The fix: recover the real
address-mapping functions **from the hardware itself** via timing, then decode
each error's address to (channel, DIMM) → SPD slot → serial.

In UEFI pre-boot everything is identity-mapped (physical == virtual), so we
skip the `/proc/pagemap` machinery the original paper needed under Linux.

---

## 2. Primitive: row-conflict latency

Two addresses in the **same bank but different rows** force a row conflict — the
access serialises and is slow. Different banks → parallel → fast. `clflush`
evicts both lines so the read actually reaches DRAM. We take the **minimum** of
16 probes to reject scheduler/interrupt noise:

```c
static UINT64 probe_pair_lat(volatile UINT64 *a, volatile UINT64 *b) {
    UINT64 best = ~0ULL;
    for (int i = 0; i < 16; i++) {
        __asm__ __volatile__("clflush (%0)" :: "r"(a) : "memory");
        __asm__ __volatile__("clflush (%0)" :: "r"(b) : "memory");
        __asm__ __volatile__("mfence\n\tlfence" ::: "memory");
        UINT64 t0 = rdtsc_now();
        __asm__ __volatile__("lfence" ::: "memory");
        (void)*a; (void)*b;
        __asm__ __volatile__("lfence" ::: "memory");
        UINT64 t1 = rdtsc_now();
        if (t1 - t0 < best) best = t1 - t0;
    }
    return best;
}
```

A calibration pass (`timing_probe_calibrate`) histograms 4000 random pairs and
requires a **clean bimodal gap** (fast cluster vs slow cluster). On the Haswell
reference board the fast cluster sat around ~233 cyc and the slow (row-conflict)
cluster around ~390 cyc, with an empty band between them — a textbook gap. No
gap → the channel is too noisy on this platform and the method bails out. This
is the first go/no-go gate.

---

## 3. Sample across ALL of physical RAM

Critical: the probe must span **every 4 GB block**, or the channel/DIMM-select
function (which lives in high or hashed bits) never becomes observable. We walk
the UEFI memory map, collect the large `EfiConventionalMemory` regions, and
scatter the address pool size-weighted across all of them:

```c
/* collect EfiConventionalMemory regions >= 64 MB into regs[]/rpages[] via GetMemoryMap */
UINT64 totpg = 0; for (int i = 0; i < nr; i++) totpg += rpages[i];
UINT64 rng = rdtsc_now() | 1ULL;
for (int i = 0; i < FN_POOL; i++) {
    rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;   /* xorshift64 */
    UINT64 pg = rng % totpg, acc = 0; int r = 0;           /* size-weighted region pick */
    for (; r < nr; r++) { if (pg < acc + rpages[r]) break; acc += rpages[r]; }
    if (r >= nr) r = nr - 1;
    UINT64 a = regs[r] + (pg - acc) * 4096ULL + ((rng >> 20) & (4096ULL - 64ULL));
    g_fn_pool[i] = a & ~63ULL;                             /* cache-line aligned */
    g_fn_setid[i] = 0xFF;                                  /* 0xFF = ungrouped */
}
```

```c
#define FN_POOL   512
#define FN_BIT_LO 6
#define FN_BIT_HI 34   /* a32/a33 are the 4 GB block boundaries on a 16 GB box */
```

The probe is read-only (clflush + load) on free memory.

---

## 4. Group addresses into same-bank sets

Threshold = midpoint of the min/max latency of `pool[0]` vs the rest. Then a
greedy union: for each ungrouped address, start a new set and pull in everyone
that row-conflicts with it (latency ≥ threshold = same physical bank):

```c
UINT64 thresh = (lo + hi) / 2;
g_fn_nsets = 0;
for (int i = 0; i < FN_POOL; i++) {
    if (g_fn_setid[i] != 0xFF) continue;
    g_fn_setid[i] = (UINT8)g_fn_nsets;
    volatile UINT64 *bi = (volatile UINT64 *)(UINTN)g_fn_pool[i];
    for (int j = i + 1; j < FN_POOL; j++) {
        if (g_fn_setid[j] != 0xFF) continue;
        if (probe_pair_lat(bi, (volatile UINT64 *)(UINTN)g_fn_pool[j]) >= thresh)
            g_fn_setid[j] = (UINT8)g_fn_nsets;
    }
    g_fn_nsets++;
}
```

The number of sets equals the number of distinct physical banks observed
(2ch × 2DIMM × 8 banks = 32 on single-rank kits; more with dual-rank). It's a
sanity number.

---

## 5. The predicate: "is this XOR of address bits a selector?"

A bitmask `m` is a real addressing function (channel / DIMM / rank / bank
selector) iff its parity (XOR of the chosen address bits) is **constant within
every same-bank set** AND **differs across sets**. This is validated against the
live hardware — which is what makes the whole thing safe:

```c
static int fn_is_addr_func(UINT64 m) {
    UINT8 sv[64]; for (int s = 0; s < 64; s++) sv[s] = 0xFF;
    int gmin = 2, gmax = -1;
    for (int k = 0; k < FN_POOL; k++) {
        UINT8 s = g_fn_setid[k]; if (s >= 64) continue;
        int v = __builtin_parityll(g_fn_pool[k] & m) & 1;
        if (sv[s] == 0xFF) sv[s] = (UINT8)v;
        else if (sv[s] != (UINT8)v) return 0;   /* not constant within a set */
    }
    for (int s = 0; s < g_fn_nsets && s < 64; s++)
        if (sv[s] != 0xFF) { if (sv[s] < gmin) gmin = sv[s]; if (sv[s] > gmax) gmax = sv[s]; }
    return (gmax > gmin);                        /* varies across sets => real function */
}
```

A blind 1–2-bit brute force only finds the **bank** bits. The **channel**
function is a 7-bit XOR — invisible to a small brute force — so we test it from
a table of published candidates.

---

## 6. Address-map table + self-validation

We keep a table of published `(channel-hash, DIMM-in-channel-bit)` pairs and
accept the **first row whose both functions actually validate** on the live
silicon. A row that doesn't fit the platform simply fails `fn_is_addr_func` and
is skipped — so a wrong row can never select the wrong **addressing function**.
(This guard is at the function level only. The function-value → physical-slot
step in §7 still *assumes* the standard SMBus layout, so "never" applies to the
math, not blindly to the whole chain — see the limitation in §10.)

```c
static const struct { const CHAR16 *nm; UINT64 ch; int dbit; } g_addr_maps[] = {
    { L"Intel DDR3 2ch/2DIMM (IvyBridge/Haswell)",
      (1ULL<<7)|(1ULL<<8)|(1ULL<<9)|(1ULL<<12)|(1ULL<<13)|(1ULL<<18)|(1ULL<<19), 16 },
    { L"Intel DDR3 2ch (SandyBridge)", (1ULL<<6), 16 },
};
for (UINTN mi = 0; mi < sizeof(g_addr_maps)/sizeof(g_addr_maps[0]); mi++) {
    if (fn_is_addr_func(g_addr_maps[mi].ch) &&
        fn_is_addr_func(1ULL << g_addr_maps[mi].dbit)) {
        g_intl_chan_mask = g_addr_maps[mi].ch;
        g_intl_dimm_bit  = g_addr_maps[mi].dbit;
        g_intl_valid     = 1;     /* "address map CONFIRMED" */
        break;
    }
}
/* none matched -> g_intl_valid = 0 -> fall back to SMBIOS Type-20 */
```

DRAMA Table 2a, Haswell/Ivy Bridge DDR3, 2 channels / 2 DIMMs per channel
(validated here):

| Selector | Function |
|----------|----------|
| Channel | `a7 ^ a8 ^ a9 ^ a12 ^ a13 ^ a18 ^ a19` |
| DIMM-in-channel | `a16` |
| Rank | `a17 ^ a21` |
| Bank | `a14^a19`, `a15^a20`, `a18^a22` |

Sandy Bridge dual-channel: `Channel = a6`.

---

## 7. Decode: address → (channel, DIMM) → SPD slot → serial

Standard Intel SMBus layout: `0x50/0x51` = channel 0 DIMM 0/1, `0x52/0x53` =
channel 1 DIMM 0/1. The SPD scan fills `g_dimms[]` in probe order `0x50..0x53`,
so on a fully populated board the slot index equals the `g_dimms[]` index:

```c
static inline int addr_to_intl_slot(UINT64 a) {
    int ch   = __builtin_parityll(a & g_intl_chan_mask) & 1;
    int dimm = (int)((a >> g_intl_dimm_bit) & 1);
    return ch * 2 + dimm;          /* 0..3 = SPD 0x50..0x53 = g_dimms[] index */
}
```

Every recorded error increments a per-slot counter; the verdict reports the
dominant slot's SPD serial, part number and slot label. (Note: gnu-efi `SPrint`
pads `%02X` to 32-bit width, so the serial is formatted by hand, nibble by
nibble.)

---

## 8. Validation methodology (the method is not "proven" without this)

1. **Ground truth, one stick at a time.** With `TestOnlyDimm` forcing the buffer
   into a single DIMM's range, test each module alone. On the reference kit
   (4× Samsung DDR3 on an OptiPlex 7020/9020) exactly one serial failed —
   `214649E0` — and the other three (`173B6958`, `17A31121`, `16A0656B`) passed.
   That is the independent truth, established without any address decode.
2. **Decoder anchor — concrete.** A real recorded error sat at physical address
   `0x4803C080`. Decode it with the confirmed map:
   - channel = parity of bits {7,8,9,12,13,18,19} of the address = **1**
   - DIMM-in-channel = a16 = **1**
   - SPD slot = `0x50 + 2·channel + DIMM` = `0x50 + 2 + 1` = **`0x53`**
   - the SPD serial read at `0x53` = **`214649E0`** — the exact module step 1 had
     flagged. The decode and the independent ground truth agree.
3. **"It's the RAM, not the test" — two controls.**
   - *Different kit:* the same machine with a different (good) kit passed the
     identical tests with zero errors — same binary, same board. The errors
     track the memory, not the code.
   - *Different write path:* re-filling the same buffer with plain **scalar**
     64-bit stores produced the same byte-lane mismatches as the AVX2 fill
     (`AVX2 mismatches ≈ scalar mismatches`). So the corruption is a real memory
     fault, **not an artifact of the AVX2 instruction** — a separate proof from
     the kit-swap above.
4. **It follows the stick, not the address.** Moving the faulty module to another
   slot makes the verdict name the new slot **and the same serial** (validated
   `DIMM2 → DIMM4`, serial unchanged).

---

## 9. Contributing a map for a new platform

The current table covers Intel DDR3 dual-channel (Sandy/Ivy/Haswell). Other
platforms (DDR4, DDR5, Skylake+, AMD) fall back safely to SMBIOS Type-20 today.
To add one:

1. Run on the target platform (ideally with a known-bad module).
2. Read the `[FUNC]` log lines: did a channel/DIMM candidate confirm? How many
   same-bank sets? Are addresses in one bank spread across all of RAM
   (interleaved) or confined to one block (block-mapped)?
3. Obtain that controller's addressing functions (DRAMA-style tables per
   generation, or recover them with the candidate probe).
4. Add one row to `g_addr_maps`. `fn_is_addr_func` self-validation guarantees a
   wrong row can't fire on the wrong silicon.
5. Confirm with the anchor (step 8.2) against ground truth before claiming it
   works.

---

## 10. Limitations

- Needs `CLFLUSH` and a clean bimodal timing gap; a noisy controller defeats the
  probe (the tool then falls back to Type-20).
- The address functions must be known/tabulated for the platform.
- Assumes the standard Intel SMBus slot layout and a 2-channel × 2-DIMM,
  fully-populated topology for the slot mapping.
- **DDR5 on-die ECC** silently corrects single-bit errors, masking the very
  faults (and complicating timing) — this method is not a substitute for it.
- The labelling step (which recovered function is "the channel", and which value
  is which physical slot) is the part that is platform-specific; the timing
  recovery itself is general, the slot anchor is not free.

When the address map is not confirmed, the tool reverts to its previous,
honest behaviour — it just doesn't pin the exact slot. Fallback is never wrong,
only less precise.

---

## 11. Field notes (hard-won, read before you trust a verdict)

These cost real debugging time; they are part of the method, not trivia.

- **"A scalar write-back fixes the byte" is a *measurement* question first, not
  proof of a dead cell.** The same symptom can be a transient or a read-side
  artifact. Here it turned out to be a genuinely bad byte-lane — but that was
  only established by the controls in §8 (good-kit swap + scalar-vs-AVX2), not
  by the symptom alone. Don't shortcut to "bad cell".
- **Don't conclude a bad socket / bad lane / hardware fault from logs alone.**
  Always cross-check against single-stick ground truth (§8.1). On consumer
  dual-channel, SMBIOS Type-20 will confidently point at the wrong slot, and
  the "always DIMM3" symptom is a *mapping* artifact, not a socket fault.
- **The probe must span all of RAM.** An early version sampled one 4 GB block
  and only ever found the bank bits — the channel/DIMM functions were invisible.
  Span everything (§3) or the recovery silently under-reports.
- **The channel function is a multi-bit XOR.** A 1–2-bit brute force will not
  find it. That is why the candidate table (§6) exists.
- **Reproduce the anchor (§8.2) before claiming a new platform works.** Matching
  the decode to independent ground truth is the only thing that turns "looks
  right" into "is right".

---

### Reference

Pessl, Gruss, Maurice, Schwarz, Mangard. *DRAMA: Exploiting DRAM Addressing for
Cross-CPU Attacks.* USENIX Security Symposium, 2016.
