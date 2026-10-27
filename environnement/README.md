# `environnement/` - bench configuration

`run.sh` sources `bench.env` with `set -a` before running the steps in
`run_modules/`. Every value uses `: "${VAR:=default}"`, so a value given on the
command line or in the caller's environment always wins:

```bash
LOG_DIR=/var/tmp/calypso ENCRYPTION="a5 1" ./run.sh
./run.sh --check-paths        # check the paths below on this machine
```

## Variables in `bench.env`

| Group | Variables |
|---|---|
| Paths | `GSM_ROOT`, `QEMU_TREE`, `QEMU_BIN`, `QEMU_CFGS`, `FIRMWARE_DIR`, `FIRMWARE_ELF`, `FIRMWARE_BIN`, `OSMOCOM_BB`, `OSMOCON`, `OSMOCOM_CFG`, `OSMOCOM_HOME`, `NITB_ROOT` |
| Run and logs | `RUN_DIR` (default `$XDG_RUNTIME_DIR/calypso`), `LOG_DIR`, `CAPTURE_DIR`, `TMUX_SESSION` |
| Core network | `ENCRYPTION` (default `a5 0`) |
| Mobiles | `CALYPSO_MOBILE_DEBUG` (log categories), `CALYPSO_PULSE_LATENCY_MSEC` |
| Display | `CALYPSO_LANG` (`en`), `CALYPSO_NO_ATTACH` (`1` = do not attach tmux) |

`FIRMWARE_ELF` defaults to `layer1.highram_patched.elf` when it exists (B_BFI
frames, FACCH steals included, passed at zero gain and masked by the GAPK ECU),
otherwise to `layer1.highram.elf`.

The noise injector variables (`BRUIT_*`) are not set here; see the "Downlink
noise injector" section of the repository README.

## Gate idioms in the C code

A `CALYPSO_*` variable does not always turn off with `=0`:

| Idiom | Active when | How to turn off |
|---|---|---|
| `getenv("X") ? 1 : 0` | the variable **exists**, even as `0` | `unset X` |
| `atoi(...) > 0` | value > 0 | `X=0` |
| `*e == '1'` | exactly `"1"` | any other value |
| default ON + `X_OFF` | by default | set `X_OFF` |

QEMU logs the effective configuration as `calypso-manifest` lines in `qemu.log`;
check there rather than trusting the command line.
