# qosmo - QEMU model of the TI Calypso baseband

QEMU machine for the **TI Calypso GSM baseband** (Compal E88 / Motorola C1xx).
The unmodified [osmocom-bb](https://osmocom.org/projects/baseband) firmware
boots and runs on it. One platform, with the layer 1 selected at configure time:

```bash
../configure --target-list=arm-softmmu                      # bare platform, no L1
../configure --target-list=arm-softmmu --enable-l1-grgsm    # gr-gsm demodulation
../configure --target-list=arm-softmmu --enable-l1-dsp      # TI mask ROM on an emulated C54x
```

The two `--enable-l1-*` options are mutually exclusive; meson rejects the combination.

## Build and install

```bash
./install.sh --check          # prerequisites and state, changes nothing
sudo ./install.sh --with-deps # missing apt packages, then every step
./install.sh --list           # steps (venv configure build install bin verify) and values
```

Main options (and their environment variables): `--prefix` (`PREFIX`, default
`$GSM_ROOT/qemu-install`), `--build-dir` (`BUILD_DIR`), `--venv` (`VENV`),
`--bin-dir` (`BIN_DIR`, default `/usr/local/bin`), `--destdir` (`DESTDIR`),
`--cc` (`QOSMO_CC`). `./install.sh --help` lists them all.

Manual build:

```bash
mkdir build && cd build
../configure --target-list=arm-softmmu --disable-werror
ninja qemu-system-arm
./qemu-system-arm -M help | grep calypso   # names the linked L1: none, grgsm or c54x
```

Without an L1 the firmware boots and scans but never finds a cell. This is the
expected way to test the platform alone: boot, IRQs, TDMA, UART and SIM.

## Running the bench

`run.sh` starts the full bench (Osmocom core network, Asterisk, QEMU, osmocon,
mobile, BTS side-cars) through the numbered steps in `run_modules/`, and lays
out the panes with `tmux_modules/`. Its environment comes from
`environnement/bench.env` (see `environnement/README.md`).

## Layer-1 interface (`CalypsoL1Ops`)

[`calypso_l1_ops.h`](include/hw/arm/calypso/calypso_l1_ops.h) defines nine
callbacks from the platform to the L1:

```c
static const CalypsoL1Ops my_ops = {
    .name              = "grgsm",
    .init              = my_l1_init,           /* end of machine_init */
    .frame_tick        = my_l1_frame_tick,     /* once per TDMA frame */
    .api_read_override = my_l1_read_override,  /* API RAM read */
    .si_valid          = my_l1_si_valid,
    .l1s_fn            = my_l1s_fn,
    .burst_written     = my_l1_burst_written,  /* d_burst_d */
    .rach_written      = my_l1_rach_written,   /* d_rach */
    .page_written      = my_l1_page_written,   /* d_dsp_page */
    .uart_tx_byte      = my_tap_l1ctl,         /* optional */
};
static void reg(void) { calypso_l1_register(&my_ops); }
type_init(reg)
```

The L1 calls the platform directly (`calypso_api_ram()`, `calypso_trx_get_fn()`,
`calypso_trx_autosync_fn()`, `calypso_tsp_move()`, `calypso_tpu_run_scenario()`).
`calypso_l1_disable(why)` turns the linked L1 off when `CALYPSO_DSP_EXTERN` is
set; the DCCH tap ([`calypso_dcch_tap.c`](hw/arm/calypso/calypso_dcch_tap.c))
then publishes the dedicated channel.

## External DSP bridge

This is the mode used on the bench: the TI mask ROM runs in `c54x_exe`, outside
QEMU, and `calypso_trx.c` talks to it through
[`calypso_dsp_pont.h`](include/hw/arm/calypso/calypso_dsp_pont.h).

- Start `c54x_exe --arm` first, then QEMU with `CALYPSO_DSP_EXTERN=1` (or a
  socket path). QEMU exits if the shared memory does not exist yet.
- API RAM shared through `/dev/shm/calypso_api_ram` (16 KiB); frame-by-frame
  synchronisation on `/tmp/calypso_dsp.sock` (`SOCK_SEQPACKET`).
- Messages: `HELLO`/`HELLO_OK` (magic `'C54X'`), `RESET`, `TICK` (a = fn,
  b = page and flags), `DONE` (IRQ/IDLE/INIT flags), `GO`, `DCCH`.
- `TICK.b` bit 16: frame IRQ armed by the ARM (`TPU_CTRL_DSP_EN`).
  Bit 17: two-phase tick (ROM ISR, `DONE|PONT_DONE_PHASE_A`, ARM frame IRQ and
  `l1_sync()`, then `PONT_GO` and the burst).
- `PONT_DCCH` carries TN, type and sub-channel of SDCCH/4 (0), SDCCH/8 (1) or
  released (`0xFF`). `calypso_dcch_tap.c` learns it from the L1CTL stream and
  also publishes `/dev/shm/calypso_dcch_cfg`, which announces TCH/F (2) and
  TCH/H (3); the TCH switch itself follows the firmware task in `c54x_exe`.

The `l1-dsp/` sources also build outside QEMU with
[`contrib/hors-qemu/cales-qemu.c`](contrib/hors-qemu/cales-qemu.c) and the
headers in `contrib/hors-qemu/doublures/`; `c54x_exe` and `grgsm_exe` compile
them from this tree (`QOSMO ?= /opt/GSM/qosmo`).

### QEMU-side environment variables

| Variable (on `qemu-system-arm`) | Effect |
|---|---|
| `CALYPSO_DSP_EXTERN` | `1` or socket path: bridge to `c54x_exe` |
| `CALYPSO_PONT_LOCKSTEP=1` | QEMU waits for `DONE` each frame (required: the emulated C54x is slower than real time) |
| `CALYPSO_PONT_ARM_FIRST=0` | single-phase tick |
| `CALYPSO_PONT_RETRY_DIV` | resend to the DSP every frame/N (default 16; `c54x_exe/run.sh` sets 64) |
| `CALYPSO_CPU_KICK_NS` | CPU kick period (default 5 ms) |
| `CALYPSO_PACER_RATTRAPAGE` | late frames the TDMA pacer catches up (default 20; `0` = strict grid) |

`CALYPSO_BSP_STREAM`, `CALYPSO_A5`, `CALYPSO_SONDES`, `CALYPSO_C54X_OVM`,
`CALYPSO_MVKD_DMAD_AVANT` and `CALYPSO_HACK_SOFT_SCALE` are read by `l1-dsp/`:
with the external DSP, set them on `c54x_exe`, not on QEMU.

## Downlink noise injector

Off by default. [`tools/injecteur_bruit.py`](tools/injecteur_bruit.py) is a
transparent UDP proxy between the bridge and the L1 that degrades the downlink;
the uplink passes unchanged. [`run_modules/38-bruit.sh`](run_modules/38-bruit.sh)
starts it before `40-qemu` when `BRUIT_MODE` is set; `c54x_exe/run.sh` does the
same for the external DSP.

| Variable | Effect |
|---|---|
| `BRUIT_MODE` | `ber` (bit flips), `souple` (AWGN + hard decision), `iq` (GMSK I/Q + AWGN, experimental, DSP only, refused with `CALYPSO_BSP_STREAM=1`), `relais` (pass-through) |
| `BRUIT_BER`, `BRUIT_RAFALES` | flip rate 0..1; mean burst length in bits (Gilbert-Elliott) |
| `BRUIT_SNR_DB` | SNR in dB; without `BRUIT_MODE` it exports `CALYPSO_BSP_SNR_DB` (AWGN in the C54x core) |
| `BRUIT_PERTE`, `BRUIT_TN`, `BRUIT_GRAINE` | erasures, targeted timeslots, random seed |
| `BRUIT_STATS`, `BRUIT_OPTS`, `BRUIT_PY` | statistics period (10 s), extra options, injector path |
| `BRUIT_CIBLE`, `BRUIT_PORT_DSP`, `BRUIT_PORT_GRGSM` | target L1 (`auto` reads `build/meson-info`); relocated ports (16702, 14730) |

With the gr-gsm L1, QEMU listens for GSMTAP on port 4730; the bridge then sends
to 14730 and only `ber` and `relais` apply (the stream carries decoded L2
blocks). With the DSP L1 the module exports `CALYPSO_BSP_PORT=16702` and the
injector takes 6702. Log: `$LOG_DIR/bruit.log`.

## Status

| Part | State |
|---|---|
| Platform (SoC, TPU, TSP, SIM, ULPD, UART, timers, SPI/I2C, INTH, TRF6151, L1 dispatch, DCCH tap) | Builds and boots alone |
| `CalypsoL1Ops` interface | 9 callbacks, used by the gr-gsm L1 |
| `--enable-l1-grgsm` | Working |
| `--enable-l1-dsp` (C54x inside QEMU) | `l1-dsp/` compiles and links; the ops table is empty except `.name = "c54x"` |
| External DSP (`CALYPSO_DSP_EXTERN`) | SCH, BCCH SI1-4, location update, SMS and TCH/F calls end to end; A5 ciphering on the XIO ports |
| E88 board devices (`calypso_iota.c`, `calypso_rf3166.c`, `calypso_asm4532.c`, `calypso_xio.c`, `calypso_debug.c`, `calypso_invariants.c`, `fw_console.c`, `sercomm_gate.c`, `l1ctl_sock.c`) | Built only with `--enable-l1-dsp`; not wired: their entry point `calypso_pcb_init()` (in `l1-dsp/calypso_full_pcb.c`) is not called from `calypso_soc_realize()` |

## Known issues

1. **`B_BFI` on every speech frame** with the external DSP. The ROM reports
   roughly 3 to 20 % bit errors on the 456 coded bits; FR speech stays
   intelligible because the firmware does not forward the BFI. Unknown whether
   the cause is the signal path (BSP, I/Q, equalisation) or the C54x core
   (Viterbi, re-encoding); a bit-by-bit comparison with the BTS frames is needed.
2. **Occasional LOS during a call**: the SACCH downlink stops being accepted,
   the ACCH counter drops to 0 and the mobile reports "LOS during dedicated mode".
   Not reproducible on every run.
3. **SDCCH/8 SACCH downlink**: about half of the blocks are dropped. Suspect:
   the 45.002 table of the BSP for SACCH/8 over 102 frames.
4. **Real-time margin**: in TCH the DSP work takes about 4.2 to 4.6 ms of the
   4.62 ms frame, which leaves almost no headroom.
5. Pacer catch-up (`CALYPSO_PACER_RATTRAPAGE`) is not validated for speech.
6. The "kept in qosmo-dsp" block in `hw/arm/calypso/meson.build` is outdated.

## Repository layout

| Path | Content |
|---|---|
| `hw/arm/calypso/` | Calypso platform devices, L1 dispatch, DCCH tap |
| `hw/arm/calypso/l1-grgsm/` | gr-gsm layer 1 and its registration |
| `hw/arm/calypso/l1-dsp/` | C54x core (exec, decode, mem, irq, probes), BSP, A5, mailbox, full PCB |
| `hw/arm/calypso/doc/SONDES.md` | C54x core probes |
| `include/hw/arm/calypso/` | Public headers (`calypso_l1_ops.h`, `calypso_dsp_pont.h`, ...) |
| `hw/intc/calypso_inth.c` | Interrupt controller |
| `contrib/hors-qemu/` | POSIX shims to build `l1-dsp/` outside QEMU |
| `run.sh`, `run_modules/` | Bench launcher and its numbered steps |
| `tmux_modules/` | tmux layout of the bench panes |
| `environnement/` | Bench environment file (`bench.env`) |
| `cfgs/` | mobile configuration for group runs |
| `opt-gsm-scripts/` | Helper scripts installed into `/opt/GSM` |
| `tools/` | `injecteur_bruit.py`, `rapport-run.sh` (run report), `qosmo-launch`, gdb helpers |
| `install.sh` | Build and install script |
| `make-overlay.sh`, `tout-en-un.py` | Source overlay and single-file dump helpers |
| everything else | Upstream QEMU 9.2.4 |

## License

Derived from QEMU 9.2.4: GPL-2.0-or-later (SPDX headers, `COPYING`). The
repository also carries a GPLv3 `LICENSE` ("qosmo Copyright (C) 2026 Bastien Baranoff").
