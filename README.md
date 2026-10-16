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

> Only the first nodes are ported; the camera → rerun and the simulated robot
> programs below run today, the teleoperation programs are the target workflow.

Robot meshes and textures under `assets/` are stored in
[Git LFS](https://git-lfs.com). Install it before cloning (`brew install
git-lfs` / `apt install git-lfs`, then `git lfs install` once per machine); in a
clone made without it, run `git lfs pull`. Without the real files MuJoCo fails
with a mesh parse error, because it gets LFS pointer files instead.

The node API (`dora-rs`) and the CLI (`dora-rs-cli`) are the **1.1.0** wheels
from PyPI, pinned in `pyproject.toml`. 1.1.0 is the first release with the two
module-resolution fixes the repo layout depends on (dora-rs/dora#3350 and
#3465), so no Rust toolchain is needed.

```sh
uv sync
uv run dora --version
```

### Simple run

```sh
uv run dora build dataflows/examples/camera_rerun/camera_rerun.yml   # first time: builds each node's venv
uv run dora run   dataflows/examples/camera_rerun/camera_rerun.yml
```

This records your webcam to `out/rrd/*.rrd` as H.264 and shows it live in a
rerun viewer, with the `manager` owning the lifecycle (`boot` → `running` →
`disconnect` on Ctrl-C). The recorder serves the stream over gRPC and the
viewer only connects, so the viewer can just as well run on another machine.

### Simulated robot

```sh
uv run dora build dataflows/examples/mujoco_web_rerun/mujoco_web_rerun.yml
uv run dora run   dataflows/examples/mujoco_web_rerun/mujoco_web_rerun.yml
```

A UR10e with a Robotiq 2F-85 in MuJoCo. Open http://127.0.0.1:8000 to drive
its joints and gripper; the rerun viewer shows both sim cameras, the robot in
3D and one plot per part (measured vs commanded joints), and everything is
recorded to `out/rrd/`. `mujoco_web_rerun_depth` is the same with both cameras'
depth images too (about 37 MB/s more in the recording).

To replay a recording, open it with the robot model as an asset:

```sh
uv run --project nodes/rerun rerun --asset <out>/rrd/ur10e_tabletop.model.rrd <out>/rrd/<recording>*.rrd
```

## Architecture

### Node and dataflow pattern

- **Nodes exchange plain Apache Arrow messages** — no shared code, no language
  lock-in. Every data message is recommended to carry a captured
  `capture_time` in metadata so a consumer can align streams that arrive at
  different rates. (Not `timestamp`: dora already puts its own HLC send-time
  there.) `capture_time` is the wall clock of the machine that captured the
  data, so streams are only comparable on one clock: for now everything a robot
  program captures (the robot, its cameras) is assumed to run on one computer.
  Splitting producers across machines needs their clocks synchronised (chrony,
  PTP) first.
- **Observations are aligned by capture time, on one observation clock.** A
  `dora/timer` (e.g. 33 ms) ticks the producers and a `sync` node in `align`
  mode. A producer that can capture on the tick does (the sim; later, triggered
  cameras); one that cannot (a free-running webcam) sends each sample as it is
  captured. Either way every sample carries its true `capture_time`, and the
  aligner builds one observation per tick from every input's sample captured
  nearest the tick. Delivery delays then cost latency, never alignment, and
  each element's own `capture_time` is kept, so every offset can be checked.
  Heavy or blocking work (camera capture, MuJoCo rendering) runs on a
  background thread, so the dora loop stays responsive.
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

Messages are flat Apache Arrow arrays; their metadata says what they are, so a
consumer (the recorder) needs no config or model to handle
a stream it has never seen. Each node documents its own messages in its
`README.md` under `nodes/<node name>/`; the shared keys are:

| key | type | meaning |
| --- | --- | --- |
| `capture_time` | float | wall-clock seconds (`time.time()`) when the data was captured; not `timestamp`, dora owns that key |
| `names` | list[str] | one label per value; for joint vectors the URDF joint names |
| `encoding`, `width`, `height` | str, int, int | images: `rgb8` (raveled H×W×3) or `jpeg`; depth: `uint16` (millimetres) |
| `camera` | str | on images and depth: the camera's name, as in the cell's calibration file |
| `step` | int | the observation index a message belongs to: set on each tuple by `sync`, copied by every node answering it (a controller's target, the safety filter's command), so a recording puts them on the observation's row |

List values must be plain Python numbers: dora turns a list holding numpy scalars
into its string form, without an error.

Images and depth say what they are by their `encoding`, so a consumer like the
recorder needs no config for them. Joint states and targets look alike (both
named vectors), so a node that must tell them apart is told by its config (the
recorder's `joint_states`). Each stream type requires its keys; a
consumer may rely on them and fails loudly without them. A *named vector* is a
flat numeric array whose `names` label every value (one name per value): joint
states and targets are named vectors, and several of them can travel
together as a `sync` tuple.

| stream | payload | required metadata |
| --- | --- | --- |
| image | `uint8[N]` | `encoding`, `width`, `height`, `capture_time`; `camera` to find its calibration |
| depth | `uint16[w*h]` millimetres | `encoding` (`uint16`), `width`, `height`, `capture_time`; `camera` |
| joint state | `float64[n]` measured positions (a named vector) | `names` (n), `capture_time` |
| joint target | `float64[n]` position setpoints (a named vector) | `names` (n), `capture_time` |

Static facts about a stream stay out of its messages. A camera's calibration
(its frame, extrinsics and intrinsics) is an input from the calibration
process, a file per cell (`assets/cells/<cell>/calibration.yaml`), which a
consumer reads by config and looks up by the message's `camera`.

Consumers never parse meaning out of input or output ids: an id is just the
name a dataflow wires. Robot streams are per *part* (a named
set of joints: an arm, a gripper, ...), and their ids follow one naming
convention so dataflows read alike:

| id | payload | direction |
| --- | --- | --- |
| `<part>_joint_state` | `float64[n]` measured joint positions | robot/sim → consumers |
| `joint_target` | joint position setpoints for any joints of the robot: a named vector (`names` says which) | controller → robot/sim, `queue_size: 1, queue_policy: drop_oldest`; one input for the whole robot |
| `node_state` / `<name>_node_state` | `utf8[1]` token (`ready`, `finished`, `disconnect`) | every node → manager, edge-triggered |
| `program_state` | `utf8[1]` (`boot`, `running`, `disconnect`) | manager → every node |

Poses come with the `pink` node's IK mode.

### Robot control

Every station comes with an MJCF model of the robot with cameras and a small
YAML config describing how to control it (EE or joint control, or anything
else that isn't covered by the MJCF and is required for robot control).

## Layout

> dora treats the directory of the dataflow file you run as the project root.
> Node paths may leave it (`../../../nodes/<node>/…` from a program directory
> works), but a `module:` path may not: it has to resolve inside the project
> root, and since dora-rs/dora#3247 (fixed by #3350 and #3465, both in
> 1.1.0) an in-tree symlink counts as inside. So only programs that use
> shared modules carry a `modules -> ../../modules` symlink and reference
> `modules/<subdir>/<module>.yml`. Node paths *inside* a module are relative to
> the module file's physical directory (`../../nodes/<node>/…` from
> `modules/<subdir>/`), so a module is written once and works from any program.

```
assets/                            # formats and how to make them: assets/README.md
  <robot family>/                  # e.g. universal_robots, trossen
    <robot name>/                  # e.g. ur10e (with a 2F-85), trossen_arm — a single arm
      meshes/                      # mesh files (STL/OBJ), in Git LFS (.gitattributes)
      schemas/                     # the robot's MJCF using the meshes (and its generator)
      urdf/                        # the robot's URDF and its meshes (and its generator)
  props/                           # reusable things a cell holds: table, floor, ... (a URDF each)
  cells/                           # robots and props placed in a world: one directory per cell
    build_cell.py                  # composes a cell's scene file into its URDF and MJCF
    <cell>/                        # e.g. ur10e_tabletop; later two arms, or an arm on a mobile base
      scene.yaml                   # its robots, props and the files the nodes load
      calibration.yaml             # its cameras
      <cell>.urdf, <cell>.xml      # built: its kinematics and simulation
libs/                              # Python packages shared by nodes (uv path dependencies)
  akai_scene/                      # the scene file format (a package the nodes depend on)
.claude/skills/                    # agent skills; rerun-* are vendored from rerun (scripts/sync_rerun_skills.py)
.agents/skills -> .claude/skills   # the same skills for Codex, Cursor and other agents
scripts/                           # repo maintenance scripts
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
  examples/                        # small programs exercising a few nodes together
    camera_rerun/
      camera_rerun.yml             # one webcam -> rerun viewer, manager-owned lifecycle
    mujoco_web_rerun/
      mujoco_web_rerun.yml         # UR10e sim driven from a web page through the safety filter, recorded + viewed in rerun
    mujoco_web_rerun_depth/
      mujoco_web_rerun_depth.yml   # the same, with depth images from the RGB-D cameras
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

## Agent skills

Rerun's own [agent skills](https://github.com/rerun-io/rerun/tree/main/skills)
(data model, URDF, blueprints, dataset conversion, ...) are vendored into
`.claude/skills/` at the release of the `rerun-sdk` locked by the rerun node, so
they describe the SDK the stack runs. After bumping `rerun-sdk` in
`nodes/rerun/pyproject.toml` (and `uv lock` there):

```sh
uv run scripts/sync_rerun_skills.py          # re-vendor them at the new version
uv run scripts/sync_rerun_skills.py --check  # exit 1 if they are out of date
```

Don't edit the vendored skills; a sync overwrites them. `.rerun-upstream.json`
records the tag and commit they came from.

## Nodes

Planned nodes. Columns track the review pipeline: a node counts as landed only
when all three are checked.

| node | what it does | unit tests | implemented | human-verified |
| --- | --- | --- | --- | --- |
| `manager` | State machine that collects `node_state` tokens and emits `program_state`. `simple` mode: `boot` → `running` → `disconnect`. | - | x | x |
| `opencv_camera` | Publishes frames from an OpenCV capture (webcam or file) as `<name>_image` at the configured FPS, rgb8 or jpeg. | - | x | x |
| `realsense-camera` | Intel RealSense color + optional depth. | - | - | - |
| `pink` | Robot kinematics on the URDF with **pink** (on Pinocchio + Coal), one IK step per input. `joints` mode: a safety filter answering each step's target (joined with its observation by `sync`) with a command within the velocity and joint limits, kept off the robot itself and the table by a collision barrier. Planned: a pose (IK) mode. Simulator-independent; reads the scene file. | - | x | - |
| `mujoco` | **MuJoCo** sim of an MJCF, in real time, the only clocked node: captures on each tick: `joint_target` in (a named vector), `<part>_joint_state` (URDF joint names) and MJCF camera images (built from the cell's calibration) out, all of one instant; physics and rendering on background threads. | - | x | - |
| `sync` | Joins streams into one message, a *tuple* (an Arrow struct, a field per input; their metadata as `<input>.<key>`). `tuple` mode: the moment every input has a new message. `align` mode: per reference message, every input's sample captured nearest it, for aligned observations (cameras + state for a policy or recorder). `step` mode: the messages of one `step`, e.g. an observation and the target answering it; a tuple input joins in flat. Dora has no built-in join. | - | x | x |
| `trossen-robot` | The **real** Trossen robot, **one arm per node** (`NAME`+`IP`, `MODE`=follower/leader; `base` mode TODO for the `trossen-slate` mobile base): a follower streams per-part `joint_target` from pink (joint control only); a leader publishes its hand-moved state. | - | - | - |
| `ur5e-robot` | The **real** UR5e, **one arm per node** (`NAME`+`IP`): servoJ streaming of per-part `joint_target` with joint-jump guards; optional Robotiq gripper. | - | - | - |
| `lerobot` | Records cameras + the `state` bundle + the `command` bundle into a `LeRobotDataset` (with video). | - | - | - |
| `rerun` | One node, two modes: `record` writes every stream to an `.rrd` (cameras as H.264 video) and serves it live over gRPC (cameras as JPEG, ~6 ms behind capture), dispatching on message metadata: named joint plots per part, an optional URDF robot in 3D (`rerun.urdf.UrdfTree`) moved by joint states, cameras placed in it from a calibration file, its model written once as a shared asset and its forward kinematics as a layer; `visualize` opens a viewer on that server, locally or from another machine. | - | x | x |
| `web_controller` | A web page (FastAPI + WebSocket, plain JS). `joint` mode: sliders and +/- per joint of every part it sees, closed-loop on the observation tuple, answers each one with a `joint_target` for all parts (carrying its `step`), Disconnect button. Planned: `cartesian` (needs pink), `episode`. | - | x | - |

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
