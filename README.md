# Akai — a modular robot control, simulation and training stack

> **Status:** very early / experimental. Nodes are being ported here one by one
> from an earlier prototype; nothing lands before it is reviewed and tested.
> The [Nodes](#nodes) table tracks progress.

The goal is to have one fully open-source, easy-to-install and customizable
stack for robotics: the same repository running on a Raspberry Pi controlling
an SO-101 and on a server with hundreds of GPUs, covering the whole pipeline —
controlling robots, collecting data, running simulations, hosting a database
and training models.

## Philosophy

ROS gives you a lot of independent nodes that control different aspects of the
robot, but it is very hardware-specific and doesn't give you a ready-to-use
stack to wire them together or train a policy.

LeRobot, on the other hand, covers the whole loop — from anything to do with
the robot to anything to do with training. But it is very hard to customize:
if you want to add something or change the behavior of core components, the
code is not very modular, and — much worse — you will have to keep fixing
conflicts with future updates, which isn't maintainable.

Dora looks like a great choice for modern robotics. It gives you modularity
like ROS together with out-of-the-box readiness for training and simulation.
But dora itself is only a framework, not a ready stack — and a ready stack is
what this project is about. Any node, any data format, any dataflow can be
easily customized and changed. This project is a good starting point that lets
you work on and improve one thing while reusing the existing components. It
assumes some canonical communication patterns, conventions, and project and
technology choices, described below.

## Quickstart

> Only the first nodes are ported; the camera → rerun program below runs today,
> the teleoperation programs are the target workflow.

The node API (`dora-rs`) is the **1.0.1** wheel from PyPI. The CLI is built
from source at a commit pinned in `pyproject.toml`, because the repo layout
depends on two module-resolution fixes that landed after the 1.0.1 release
(dora-rs/dora#3350 and #3465). Building it needs Rust >= 1.95
(`rustup update stable`); the first `uv sync` takes a few minutes.

```sh
uv sync
uv run dora --version
```

### Simple run

```sh
uv run dora build dataflows/tests/camera_rerun/camera_rerun.yml   # first time: builds each node's venv
uv run dora run   dataflows/tests/camera_rerun/camera_rerun.yml
```

This records your webcam to `out/rrd/*.rrd` as H.264 and shows it live in a
rerun viewer, with the `manager` owning the lifecycle (`boot` → `running` →
`disconnect` on Ctrl-C). The recorder serves the stream over gRPC and the
viewer only connects, so the viewer can just as well run on another machine.

## Architecture

### Node and dataflow pattern

- **Nodes exchange plain Apache Arrow messages** — no shared code, no language
  lock-in. Every data message is recommended to carry a captured
  `capture_time` in metadata so a consumer can align streams that arrive at
  different rates. (Not `timestamp`: dora already puts its own HLC send-time
  there.)
- **Producers (cameras, robot) are lightweight and never block the dora
  loop.** Heavy or blocking work — camera capture, MuJoCo rendering — runs on
  a background thread that keeps a latest-sample slot. The node publishes
  that latest sample when its `tick` input from a `dora/timer` fires, once
  per new sample (the slot is emptied when taken, so a fast tick only lowers
  latency). See `opencv_camera`.
- **A node opens its resources before it joins the dataflow.** The mode is
  built first (camera, recording, viewer) and only then `Node()` is created
  and handed to `mode.start(node)`. dora starts the timers once every node has
  joined, so nothing ticks into a node that is still opening hardware.
- **Per-node config** is a pydantic-settings model populated from the
  dataflow's `env:` block.
- **Nodes often have multiple modes** they can load, to serve different use
  cases from one node.
- **The manager node owns the program state.** It collects state reports from
  the other nodes (`<name>_node_state`, edge-triggered tokens such as `ready`
  and `finished`) and emits the new `program_state`; the other nodes receive
  that state as an input, tear down on `disconnect`, and report important
  changes back to the manager — for example connecting to a robot, or
  switching between teleoperation and policy control.

### Messaging formats

General conventions: TODO. Each node documents its own message formats in its
`README.md` under `nodes/<node name>/`.

### Robot control

Every station comes with an MJCF model of the robot with cameras and a small
YAML config describing how to control it (EE or joint control, or anything
else that isn't covered by the MJCF and is required for robot control).

## Layout

> dora treats the directory of the dataflow file you run as the project root.
> Node paths may leave it (`../../../nodes/<node>/…` from a program directory
> works), but a `module:` path may not: it has to resolve inside the project
> root, and since dora-rs/dora#3247 (fixed by #3350 and #3465, both in the
> pinned CLI) an in-tree symlink counts as inside. So only programs that use
> shared modules carry a `modules -> ../../modules` symlink and reference
> `modules/<subdir>/<module>.yml`. Node paths *inside* a module are relative to
> the module file's physical directory (`../../nodes/<node>/…` from
> `modules/<subdir>/`), so a module is written once and works from any program.

```
assets/
  <robot family>/                  # e.g. trossen
    <robot name>/                  # e.g. trossen_arm — a single arm
      meshes/                      # STL mesh files
      schemas/                     # MJCF schemas using the meshes
      scenes/                      # YAML scene descriptions with extra robot-control parameters
    <robot name>/                  # e.g. trossen_stationary — meshes/ empty, schemas reuse the
                                   # single arm: two arms at a distance, correctly oriented
    <robot name>/                  # e.g. trossen_mobile — same reuse, different distance/orientation
modules/                           # reusable dora modules (sub-graphs), mounted into programs
  robots/                          # planned: ready-to-plug robot nodes with cameras
    <robot family>/                # e.g. trossen
      stationary.yml
      stationary_mujoco.yml
  software/                        # planned: reusable dataflow logic
    teleoperation_spine/           # planned: teleoperation spine with the teleoperator side
    benchmark_spine/               # planned
    training_spine/                # planned
dataflows/                         # runnable programs, one directory each
  tests/                           # small programs exercising a few nodes together
    camera_rerun/
      camera_rerun.yml             # one webcam -> rerun viewer, manager-owned lifecycle
  teleoperation/                   # planned
    <robot family>/                # e.g. trossen
      <robot name>/                # e.g. trossen_stationary
        modules -> ../../../../modules   # symlink, only needed for `module:` references (see note above)
        trossen_stationary_local.yml
        trossen_stationary_remote.yml
        trossen_stationary_mujoco_local.yml
nodes/
  <node name>/                     # own uv project (its .venv is built by the dataflow's `build:`)
    pyproject.toml                 # plus uv.lock
    README.md                      # node docs, including its message formats and config
    src/
      main.py                      # plain script: config -> mode -> Node -> loop, identical in every node
      config.py                    # pydantic-settings config (mode, name, fps, resolution, ...)
      modes/
        <mode name>.py             # one mode = one behaviour; the default is the node's main job
```

A dataflow launches a node as `path: ../../nodes/<node>/.venv/bin/python` with
`args: ../../nodes/<node>/src/main.py` (dora runs a bare `.py` path with the
system python, not the node's venv), after `build: uv sync --project
../../nodes/<node>`. Assets are referenced the same way (`../../assets/…`).

## Nodes

Planned nodes. Columns track the review pipeline: a node counts as landed only
when all three are checked.

| node | what it does | unit tests | implemented | human-verified |
| --- | --- | --- | --- | --- |
| `manager` | State machine that collects `node_state` tokens and emits `program_state`. `simple` mode: `boot` → `running` → `disconnect`. | - | x | x |
| `opencv_camera` | Publishes frames from an OpenCV capture (webcam or file) as `<name>_image` at the configured FPS, rgb8 or jpeg. | - | x | x |
| `realsense-camera` | Intel RealSense color + optional depth. | - | - | - |
| `pinocchio` | Whole-robot **IK + collision safety** (Pinocchio + Coal): consumes the `command` + `state` bundles, runs a synchronized whole-robot solve (self-collision + plane constraints), emits per-part `<part>_joint_target` + per-arm `measured/solution_pose` (model-frame FK). Generic, simulator-independent; reads the scene descriptor. | - | - | - |
| `mujoco-sim` | **MuJoCo** sim driven by per-part `<part>_joint_target`, emits the `state` bundle + per-part `tcp_pose` + cameras — the **fast local** backend (realtime, bg-thread render). Reads the descriptor. | - | - | - |
| `sync` | Reusable aggregator: collects N event-driven inputs, emits one concatenated bundle the moment every input has a fresh sample (no tick), `log.warning` on component-timestamp skew. Bundles the per-arm hardware nodes (no built-in dora join). | - | - | - |
| `trossen-robot` | The **real** Trossen robot, **one arm per node** (`NAME`+`IP`, `MODE`=follower/leader; `base` mode TODO for the `trossen-slate` mobile base): a follower streams per-part `joint_target` from pinocchio (joint control only); a leader publishes its hand-moved state. | - | - | - |
| `ur5e-robot` | The **real** UR5e, **one arm per node** (`NAME`+`IP`): servoJ streaming of per-part `joint_target` with joint-jump guards; optional Robotiq gripper. | - | - | - |
| `lerobot` | Records cameras + the `state` bundle + the `command` bundle into a `LeRobotDataset` (with video). | - | - | - |
| `rerun` | One node, two modes: `record` writes every stream to an `.rrd` (cameras as H.264 video) and serves it live over gRPC (cameras as JPEG, ~6 ms behind capture); `visualize` opens a viewer on that server, locally or from another machine. | - | x | - |
| `web-controller` | `manual` mode builds + emits the `command` bundle (closed-loop page, +/- buttons); `episode` mode = episode/task + disconnect only. | - | - | - |

## Roadmap

Might be delusional.

1. Be able to control any type of robot: single- or multi-armed, stationary or
   mobile, static, legged or wheeled. Right now the primary target is bimanual
   manipulation.
2. Control a fleet of robots with remote teleoperation — a crucial feature for
   real VLA robotics applications.
3. A robotics database. Not only collecting episodes, but a real database with
   secure transactions, ready for continuous updates and querying. Since rerun
   provides the closest open-source thing to a robotics database (rerun
   server), plus nice visualizations and a flexible dataset format, it is the
   primary choice for storing data and visualizations.
4. Train and run inference with big models: train VLAs and other models on
   large datasets, run inference in benchmarks and on real hardware.
