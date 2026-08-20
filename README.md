# micro_sp_gui

A web GUI for a [micro_sp](https://github.com/endre90/micro_sp) system: browse and edit the whole
state, walk and edit the transform tree, drive a UR robot through
[`ur_redis_driver`](https://github.com/sequenceplanner/ur_redis_driver), read the activity log, and
send goals and orders.

It is one egui application, compiled to WebAssembly, served by one Rust binary.

## Why there is a server

The GUI is egui compiled to `wasm32-unknown-unknown`, and wasm has no TCP sockets, so the browser
cannot speak the Redis protocol. `micro_sp` is not wasm-buildable either — it depends on `tokio`
with `full` and `redis` with `tokio-comp`, both unconditionally — so the frontend cannot even borrow
its types.

So the binary holds the one `ConnectionManager`, and the frontend talks to it:

```
browser (egui/wasm) ──── /ws     push feed ────► two pollers ───► Redis
                    └─── /api/*  POSTs     ────► StateManager / TransformsManager
```

Two pollers, because they are not the same shape of read. One scans the whole keyspace every
`--poll-ms`; the other `MGET`s only the robot's ~21 variables every `--robot-poll-ms`, so the Robot
tab can keep up with a driver that republishes every 5 ms without paying for a full scan twenty
times a second.

Reads are pushed over a websocket; writes are plain `POST`s. `crates/protocol` mirrors `SPValue`'s
serde representation exactly, so a value deserialises straight out of Redis and back again with no
translation layer — `crates/server/tests/mirror.rs` fails the build if that mirror ever drifts from
`micro_sp`.

## Layout

| crate | what it is |
|---|---|
| `crates/protocol` | wire types. `serde` only, so it compiles to wasm. Mirrors `SPValue`, and duplicates the constants `ur_redis_driver` cannot share. |
| `crates/server` | `axum` + `micro_sp`. Polls Redis, tails the activity log, serves `dist/`, and applies writes. Binary: `micro_sp_gui`. |
| `crates/web` | the egui frontend. Also builds a native dev shell (`micro_sp_gui_dev`) that talks to the same server. |

## Running it

```bash
rustup target add wasm32-unknown-unknown
cargo install --locked trunk

docker run -p 6379:6379 -d redis           # or point REDIS_HOST/REDIS_PORT elsewhere
trunk build --release                      # produces ./dist
cargo run --release -p micro_sp_gui_server -- \
    --log-dir /tmp/msp_logs \
    --frames-dir /tmp/msp_frames
# open http://localhost:8080
```

### Developing

`trunk serve` rebuilds the frontend on save and proxies `/api` and `/ws` to the server, so only the
server needs restarting when you touch it:

```bash
cargo run -p micro_sp_gui_server -- --log-dir /tmp/msp_logs   # :8080
trunk serve                                                    # :8081, open this one
```

For compiler errors and a debugger without a wasm round trip, run the same UI natively:

```bash
cargo run -p micro_sp_gui_web --bin micro_sp_gui_dev
```

### Configuration

Redis comes from `micro_sp`'s own `REDIS_HOST` / `REDIS_PORT` (default `127.0.0.1:6379`), so the GUI
agrees with every other process in the system by default. Everything else has a flag and an
environment variable:

| flag | variable | default | meaning |
|---|---|---|---|
| `--bind` | `MICRO_SP_GUI_BIND` | `0.0.0.0:8080` | where to serve |
| `--dist` | `MICRO_SP_GUI_DIST` | `dist` | the built frontend |
| `--poll-ms` | `MICRO_SP_GUI_POLL_MS` | `250` | Redis poll period — one poller serves every browser |
| `--robot-poll-ms` | `MICRO_SP_GUI_ROBOT_POLL_MS` | `50` | how often to re-read the robot's own variables — one small `MGET`, no keyspace scan |
| `--log-dir` | `MICRO_SP_ACTIVITY_LOG_DIR` | — | where micro_sp writes its activity log |
| `--log-stem` | `MICRO_SP_GUI_LOG_STEM` | `micro_sp` | log file base name |
| `--log-ring` | `MICRO_SP_GUI_LOG_RING` | `20000` | lines kept for backfill |
| `--frames-dir` | `MICRO_SP_GUI_FRAMES_DIR` | — | where the Transforms tab exports frame JSON |
| `--sp-id` | `SP_ID` | — | seed the sp_id picker before discovery finds one |
| `--robot-id` | `ROBOT_ID` | — | seed the robot picker |

> **There is no authentication.** The server exposes unrestricted state writes and robot commands.
> Bind it to localhost or a trusted network.

## The tabs

**State** — the whole keyspace as a filterable table (substring or regex, over keys *and* values),
with a typed editor for every `SPValue`: scalars, arrays, maps, transforms, and the `UNKNOWN`
variant of each. Edits are staged and applied as one batch, so dragging a number does not write to
Redis on every frame. A value Redis holds that is not valid `SPValue` JSON is shown as unparsed
rather than silently dropped, which is what `micro_sp`'s own `build_state` does to it.

**Robot** — builds a `ur_redis_driver` request and shows what the driver publishes back:
`safety_mode`, `robot_mode`, connection lights, joint states, TCP pose, and the
`request_state` → `request_result` / `request_feedback` chain with the fail counters.
A connection light has three states, not two: grey means nothing has written the variable at all,
which is a different fault from the driver reporting that it is disconnected — a driver older than
`v0.1.1` never writes them, and painting that red sends you hunting for a network problem that does
not exist. All 19 URScript
templates and all 24 dashboard commands are available. A cancelled goal is shown as cancelled, not
failed — the driver terminates it as `succeeded` with result `cancelled`.

**Transforms** — the tree as collapsible nodes, with Move, Reparent, Snap-to-parent, Remove and
Insert, plus a lookup between any two frames. Frames `ur_redis_driver` owns are marked, because it
reasserts their parent and pose on every tick and an edit there will not stick. Export writes a
frame as a scenario file, with the robot's joint configuration and gantry position as metadata, in
the layout `load_transforms_from_path` reads.

**Logs** — micro_sp no longer publishes log blobs to Redis; the `{sp_id}_logger_*` keys are gone. The
server tails the on-disk activity log instead (rotation-aware) and the tab filters it by kind
(operations / transitions / SOPs / variables), by source, and by grep or regex. Timestamps are shown
in *your* timezone: micro_sp writes records with `chrono::Local` and no offset on them, so a runner
in a container with no `TZ` logs UTC while one on the host logs local time. The file's banner is the
only line that records which, so the server resolves every record against it and the browser renders
the instant — hover a timestamp to see what the file literally says. Alongside it, the
live *information* pane shows the current value of every `_information` variable — what each runner,
operation and SOP is saying right now — which needs no extra Redis traffic because it is already in
the state feed.

**Goals** and **Orders** — send goal predicates, or item counts that become
`var:count_picked_{item} == {n}` goals. Both **append** to `{sp_id}_incoming_goals` rather than
overwriting it, so a batch sent before the goal runner drains the previous one is not lost; `replace`
is opt-in. Both show the same read-only pipeline: current goal and its state, planner and plan state,
the plan with the current step marked, and the incoming and scheduled queues as read back out of
Redis.

## Tests

```bash
cargo test --workspace          # 39 tests, no Redis needed
cargo clippy --workspace --all-targets
cargo check -p micro_sp_gui_web --target wasm32-unknown-unknown --lib
```

The two that matter most are in `crates/server/tests/mirror.rs`: one round-trips every `SPValue`
variant through the protocol crate and asserts the JSON is byte-identical, and one checks that every
key the server writes for a motion request is a key `ur_redis_driver` actually seeds. Both fail
loudly if an upstream crate changes.
