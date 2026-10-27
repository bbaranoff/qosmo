# tmux_modules - tmux layout of the bench

`run_modules/35-tmux.sh` sources `_commun.sh`, then `bench.sh`, and calls
`tmux_layout`. The session (`TMUX_SESSION`, default `calypso`) has six windows:
radio, core, ms1, ms2 (side-car), voice, shell.

```
_commun.sh    colours, _w / _split / _tail helpers, core/voice/shell windows, styling
bench.sh      the bench layout (TMUX_FENETRE_PREMIERE, tmux_layout_premiere, tmux_layout)
```

## Writing a layout

A layout file defines:

| Name | Purpose |
|---|---|
| `TMUX_FENETRE_PREMIERE` | name of the window created with the session |
| `tmux_layout_premiere()` | command run in that window |
| `tmux_layout()` | every other window, built with `_w` / `_split` |

## Rules

- **Colour carries the role**, not the position (see the header of `_commun.sh`).
- **Core panes follow `/var/log/osmocom/osmo-*.log`**, the service logs, not the
  module start-up traces in `$LOG_DIR/mod/`.
- **`tail -F`**, never `-f`: the log may not exist yet, and `40-qemu` truncates
  `qemu.log` beyond its size cap.
- **`|| sleep infinity`** as fallback, so a missing log does not kill the pane.
- A layout starts **no service**: tmux is only for watching; the bench runs
  without it.
