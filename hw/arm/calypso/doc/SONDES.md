# Probing the emulated Calypso DSP

**First hypothesis to test: the probe is wrong, not the emulated DSP.** A
broken probe usually produces plausible numbers; check the measurement before
building a hypothesis on it.

## Rules

1. **Prove the probe is active in the mode being run.** Never print a counter
   that is filled in only one branch without printing "not measured" in the other.
2. **Locate a buffer by RAM difference** (snapshot before, diff after), never by
   assuming an address.
3. **Respect types.** DSP memory is a `uint16_t` array holding signed `int16_t`
   values. Cast for every magnitude comparison, and for a signed quantity print
   the maximum absolute value NEXT TO the signed maximum.
4. **Go through the core's address translation**, not the raw arrays. `prog[]`
   and `data[]` are not what the DSP reads: the OVLY alias (program reads in
   0x0060-0x27FF go to `data[]` when `PMST_OVLY` is set), XPC banking and MMRs
   sit in between. Reproduce the translation or call the core function.
5. **Read compiler warnings, not only errors** (`-Wformat` catches argument
   shifts in long trace lines).
6. **Qualify success in code, not by reading the log.** On this bench a CRC OK
   counts as a decode only if it returns the INJECTED BSIC, T3 <= 50 and an FN
   equal to the frame: print both numbers (`CRC OK: N, dont VRAIES: M`).

## Zero witness

Define a region known to weigh zero and measure it in every campaign. For the
SB, the 21 samples before and after the burst carry no information: perturbing
one must not change the output. If a margin sample starts to weigh, the probe
or the protocol is broken. Measure in max |delta| of value (not sign changes),
and check that the witness floor is separate from the useful-region floor.

## PC reported by memory probes

The dispatcher advances the PC before the probe sees it, so memory-write probes
in `c54x_mem.c` usually report the PC of the NEXT instruction (the writer is
`pc-1`), but not for every instruction family. Never name a faulty instruction
from a memory probe's PC: confirm with a single-step trace that prints the PC
BEFORE execution with the real opcode (`CALYPSO_PISTE_LO/HI`), and decode the
opcode against `doc/opcodes/tic54x-opc.c`.

## Verbosity filter of `c54x_exe`

`c54x_exe` (`src/verbosite.c`) pipes stderr and classifies each line by a
keyword of the probe name: `WATCH`, `DUMP`, `SCAN`, `PROBE`, `MAP` are level 4,
`TRACE`, `LOOP`, `CYCLE` level 5. Without `-vvvv` / `-vvvvv` those probes print
nothing. Before concluding from silence, rerun with `-vvvvvv`; for an
unconditional probe, choose a name without these keywords.

## Bench pitfalls

- `getenv("X")` is TRUE for `X=0`: use a helper that treats `0`, `""`, `off`,
  `non` as false.
- `REJEU_SCH_PARTOUT=1` corrupts the reference: `cellule.c` computes
  `t3p = p51 / 10`, only correct for p51 in {1, 11, 21, 31, 41}. FN cannot be
  validated with this flag.
- The GMSK sampling offset must be **0.5** (symbol centre); at 0.0 the offset
  becomes undetermined and everything downstream is unstable.
- The SB window is **380 words** (190 complex samples). Follow the length the
  DSP programs (ALGTH) instead of hard-coding it.

## Probes

| Variable | Effect |
|---|---|
| `CALYPSO_WATCH_WR_LO/HI/N` | chronological log of every write in a data range (index, value, old value, PC, insn) |
| `CALYPSO_PISTE_LO/HI/N` | single-step over a PC range: PC BEFORE execution, real opcode, next word, A/B/T/AR0-7/BRC, words pointed to by AR2..AR5, DP/CPL/SP and candidate direct addresses |
| `CALYPSO_PISTE_DUMP_LO/HI` | dump of a data range at each pass through the PC range |
| `CALYPSO_T_LO/HI` | within an instruction window: every change of T (`PISTE-T`) and every clearing of a non-zero accumulator (`PISTE-NUL`) |

All of them need `-vvvv` / `-vvvvv` and stay inert until their bounds are set.
