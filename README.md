# nad-navigation

Live navigation controller written in Go. It reads camera point data from a UDP endpoint and publishes steering commands over UDP.

## Quick Start

Testing config (restricted modes):

```bash
go run ./cmd/nad --config config.testing.json
```

Official config (full navigation modes):

```bash
go run ./cmd/nad --config config.official.json
```

Overrides (optional):

```bash
go run ./cmd/nad --config config.testing.json --live-addr 0.0.0.0:9001 --output-addr 127.0.0.1:9002
```

## Input Format (UDP)

Send CSV packets to `live.udp_addr`:

```
t,detected,confidence,cx,cy,size
```

- `t` is optional. If omitted, send 5 fields and the controller will use wall-clock time.
- `detected` accepts `true/false`, `1/0`, `yes/no`.
- `cx, cy` are normalized in `[-1, 1]` where `(0, 0)` is the image center.
- `size` is a `[0, 1]` proxy for distance.

## Output Format (UDP)

The controller sends a CSV payload to `output.udp_addr`:

```
yaw,vertical,forward,mode
```

`mode` is the mode name (for example `TRACK` or `LATERAL_ONLY`).

## Simulation Feed (UDP)

Use the existing replay script to feed a CSV log into the live UDP input:

```bash
python scripts/send_udp.py --log iva-log-2.log --addr 127.0.0.1:9001
```

Options:

- `--loop` to loop playback
- `--speed 2.0` to run 2x real-time

## Visualization (jplot)

This project exposes live values over `expvar` at:

```
http://<viz.addr>/debug/vars
```

Install jplot if needed:

```bash
go install github.com/rs/jplot@latest
```

Example: plot input `cx/cy` and output commands:

```bash
jplot --url http://127.0.0.1:7070/debug/vars --interval 200ms input_cx input_cy
jplot --url http://127.0.0.1:7070/debug/vars --interval 200ms output_yaw output_vertical output_forward
```

If you change the visualization address, update `viz.addr` in your config.

## Modes

Modes define how the controller converts camera error into steering commands. Each mode has a clear use case so we can test safely and then enable full behavior.

**Testing-only modes**

- `LATERAL_ONLY` for calibration or safe indoor tests when you only want left/right alignment. It outputs yaw corrections only and keeps `forward=0` and `vertical=0`.
- `FLY_STRAIGHT` for drivetrain checks and timing tests. It outputs constant commands for `fly_straight_seconds`, then switches to `fly_straight_after_mode`.

**Operational modes**

- `SEARCH` when the target is missing. It scans with a slow yaw pattern and keeps forward at zero.
- `TRACK` when the target is visible but not centered. It uses PD control to center yaw/vertical.
- `APPROACH` when the target is centered for several frames. It adds forward motion while maintaining alignment.
- `CAPTURE` when the target is centered and large enough. It stops or holds steady for the capture event.

## Configuration Profiles

Two JSON configs are provided to keep testing safe and explicit (comments are allowed):

- `config.testing.json` limits modes to `LATERAL_ONLY` and `FLY_STRAIGHT` and pins `mode_override` so the controller cannot enter operational modes while testing.
- `config.official.json` enables `SEARCH/TRACK/APPROACH/CAPTURE` and clears `mode_override` so the controller can transition normally.

If you need a custom profile, copy one of these and adjust `controller.allowed_modes` and `controller.mode_override`.
