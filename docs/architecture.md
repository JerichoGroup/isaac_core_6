# Architecture

How the packages fit together, and why the boundaries are where they are. Read this if you are
extending `isaac_core`; you do not need it to use it.

## The shape

```
src/isaac_core/
├── contracts/   frames, packet spec, pose types, angles, gimbal kernel, ports, topics
├── config/      pydantic schema, layered loader, sources
├── geo/         LLA <-> ECEF <-> ENU, rotations, distance
├── protocol/    the 51-byte UDP pose codec
├── vehicle/     kinematics, trajectories, limits
├── control/     JSON-RPC 2.0 over TCP: server, client, messages, errors
├── cli/         isaac-core run / doctor / config / completion
├── install.py   locating and validating an Isaac Sim install
├── devkit/      Sim.launch / Sim.attach, transports, recording, MAVLink bridge
├── sim/         everything that imports Isaac: composer, configurator, runtime, planner
├── sidecar/     supervisor for out-of-process services
└── debug/       inspector and pose-sender GUI
```

Plus, outside the package:

```
extensions/      OmniGraph nodes (three extensions, seven nodes, one template)
usd/             scenes and feature layers, authored in the Isaac Sim GUI
ros2/            the isaac_core_ros2_msgs package
config/          default.toml, the documented configuration reference
```

## The one rule that matters

**The kernel never imports Isaac Sim, ROS 2 or GStreamer.**

`contracts`, `config`, `geo`, `protocol` and `vehicle` are pure Python. That is why the whole unit
suite runs in under twenty seconds with no GPU, no Isaac Sim and no ROS 2 installed, and it is the
single biggest difference from the previous generation, where the geodesy lived inside OmniGraph nodes
and could only be exercised by launching the simulator.

Twenty-three import-linter contracts enforce this. They are not decoration: `PYTHONPATH=src
lint-imports` fails the commit if `sim` starts importing `geo`, or if a kernel module reaches for
`omni`.

A practical consequence: when a pure computation is needed on both sides of the Isaac boundary, it
goes in `contracts/`. That is why the gimbal composition kernel and the angle helpers live there
rather than in `geo/` — `sim` may import `contracts` but not `geo`.

## Where Isaac actually gets touched

Only `sim/` and `extensions/`.

- `sim/composer.py` references USD layers onto the stage and applies semantics.
- `sim/configurator.py` turns resolved config into concrete prim attribute writes.
- `sim/runtime.py` owns the frame loop, the control-plane handlers and the main-thread task queue.
- `sim/planner.py` decides which layers to compose, as a dependency-ordered fixed point.
- `extensions/` holds the OmniGraph nodes, which are thin adapters over the kernel.

Everything Isaac-facing is deliberately concentrated so the rest can be tested and reasoned about
without a GPU.

## Threading

Isaac owns the main thread and USD is not thread-safe. The control-plane server therefore runs on its
own thread and **must not** touch USD or Kit directly. Handlers that need to either:

- record intent and let the simulation loop act on it (this is what `set_gimbal` does), or
- dispatch a task to the main thread and wait for it (`_on_main_thread`).

Getting this wrong does not produce a nice error. Calling the timeline from the server thread raises
`RuntimeError: There is no current event loop in thread 'isaac-core-control-server'`, and writing a
relationship to a live OmniGraph node aborts the whole process with exit 0 and no traceback.

## Layers

A feature is a directory with a `layer.toml` manifest and usually a `.usda` file. The manifest
declares three things:

- `requires` — what must already exist on the stage (may be satisfied by another layer)
- `provides` — what this layer contributes
- `[[bindings]]` — which config key feeds which prim attribute

Manifests contain *wiring*, never values. Values come from config. Planning resolves `requires`
against both the stage and other layers as a fixed point, which is also what makes composition order
deterministic.

This is the extension seam: a layer on `assets.layer_search_paths` composes with no change to
`isaac_core`. See [authoring_layers.md](authoring_layers.md).

## Control plane

JSON-RPC 2.0 over TCP, newline-delimited, on `127.0.0.1:8760` by default. Fifteen methods, all listed
in the `Method` enum in `control/messages.py` — that enum is the complete surface, and a test asserts
every member has a handler and every handler has a caller.

The devkit is a typed client for it. `Sim.attach` needs nothing but the host and port, which is why it
works against another machine; `Sim.launch` additionally resolves the Isaac install and owns the
process.

A token is required when binding to anything other than loopback.

## Two interpreters

Isaac Sim bundles Python 3.12. ROS 2 Humble ships C extensions for 3.10. So `rclpy` cannot be
imported inside Isaac, and all ROS I/O is done by Isaac's own C++ bridge nodes. Host-side tools
(`isaac-core-mavlink`, recording) run on system Python where `rclpy` works.

The boundary is *construction*, not publication: generated message modules are pure Python and import
fine inside Isaac, but the C typesupport needed to put one on the wire is built for 3.10. See
[ros2_and_python.md](ros2_and_python.md).

## Why the sidecar exists with nothing in it

`sidecar/` is a service supervisor — registry, restart policy, health polling — with no registered
services. RTSP moved into Isaac natively, which is what it used to supervise.

It is kept for v2 deliberately rather than deleted: the supervisor is the part that was hard to get
right, and an out-of-process service is expected again in v3. If v3 closes without one, it goes.
