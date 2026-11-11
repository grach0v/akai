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
its joints and gripper; rerun shows the cameras, the robot in 3D and joint
plots, and records everything to `out/rrd/`. `mujoco_web_rerun_depth` adds depth
images; `mujoco_web_pose` jogs the tool centre point instead, with IK in pink.

To replay a recording, open it with the robot model as an asset:

```sh
uv run --project nodes/rerun rerun --asset <out>/rrd/ur10e_tabletop.model.rrd <out>/rrd/<recording>*.rrd
```

## Architecture

### Node and dataflow pattern

- **Nodes exchange plain Apache Arrow messages** — no shared code, no language
  lock-in. Captured data should carry `capture_time` (wall clock at capture) in
  its metadata: it helps `sync` align streams and the recorder log them, but it
  is a recommendation, not a requirement. Capture times compare only on one
  clock, so for now a robot's producers run on one computer.
- **One observation per tick of the robot clock.** A `dora/timer` ticks the
  producers; a `sync` node in `align` mode joins what they captured into one
  observation per tick.
- **Keep the dora loop short.** Slow or unbounded work (waiting for a camera
  frame, video encoding) runs on a background thread. Short work may block: the
  real-time sim waits ~7 ms per tick for its render.
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

Messages are flat Apache Arrow arrays whose metadata says what they are. Each
node documents its messages in its README; the shared keys are:

| key | type | meaning |
| --- | --- | --- |
| `capture_time` | float | wall-clock seconds (`time.time()`) when the data was captured (dora owns `timestamp`) |
| `names` | list[str] | one label per value; for joint vectors the URDF joint names |
| `encoding`, `width`, `height` | str, int, int | images: `rgb8` (raveled H×W×3) or `jpeg`; depth: `uint16` (millimetres) |
| `camera` | str | on images and depth: the camera's name, as in the cell's calibration file |
| `step` | int | the observation a message belongs to: set by `sync`, copied by every answer to it (target, command) |

List values must be plain Python numbers: dora turns a list holding numpy scalars
into its string form, without an error.

A *named vector* is a numeric array with one `names` entry per value; joint
states and targets are named vectors. Each stream requires its keys, and a
consumer fails loudly without them.

| stream | payload | required metadata |
| --- | --- | --- |
| image | `uint8[N]` | `encoding`, `width`, `height`, `capture_time`; `camera` to find its calibration |
| depth | `uint16[w*h]` millimetres | `encoding` (`uint16`), `width`, `height`, `capture_time`; `camera` |
| joint state | `float64[n]` measured positions (a named vector) | `names` (n), `capture_time` |
| joint target | `float64[n]` position setpoints (a named vector) | `names` (n), `capture_time` |
| pose target | `float64[7]` `[x, y, z, qx, qy, qz, qw]`: the pose of URDF frame `frame` in URDF frame `reference` | `frame`, `reference`, `capture_time` |

Static facts stay out of messages: a camera's calibration is a file per cell
(`assets/cells/<cell>/calibration.yaml`), looked up by the message's `camera`.
Nodes never read meaning from ids; ids just follow one convention, per *part*
(an arm, a gripper):

| id | payload | direction |
| --- | --- | --- |
| `<part>_joint_state` | `float64[n]` measured joint positions | robot/sim → consumers |
| `joint_target` | joint position setpoints for any joints of the robot: a named vector (`names` says which) | controller → robot/sim, `queue_size: 1, queue_policy: drop_oldest`; one input for the whole robot |
| `pose_target` | a pose target (`frame` says which frame) | controller → pink |
| `node_state` / `<name>_node_state` | `utf8[1]` token (`ready`, `finished`, `disconnect`) | every node → manager, edge-triggered |
| `program_state` | `utf8[1]` (`boot`, `running`, `disconnect`) | manager → every node |

Poses name their frames by URDF link, so every node that loads the URDF reads
them the same way.

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
    mujoco_web_pose/
      mujoco_web_pose.yml          # the same robot driven by its TCP pose (jogged on the page), IK in pink
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
| `pink` | IK on the URDF with **pink**: answers each step's joint or pose targets with a command one step toward them, speed-limited and collision-free. | - | x | - |
| `mujoco` | **MuJoCo** sim: `joint_target` in, joint states and camera images of one instant out on each tick. `sim` mode runs in real time, `step` mode one fixed period per tick. | - | x | - |
| `sync` | Joins streams into one *tuple* (an Arrow struct, a field per input). `tuple`: when every input has a new message. `align`: every input's sample nearest a reference time (observations). `step`: the messages of one `step`. | - | x | x |
| `trossen-robot` | The **real** Trossen robot, **one arm per node** (`NAME`+`IP`, `MODE`=follower/leader; `base` mode TODO for the `trossen-slate` mobile base): a follower streams per-part `joint_target` from pink (joint control only); a leader publishes its hand-moved state. | - | - | - |
| `ur5e-robot` | The **real** UR5e, **one arm per node** (`NAME`+`IP`): servoJ streaming of per-part `joint_target` with joint-jump guards; optional Robotiq gripper. | - | - | - |
| `lerobot` | Records cameras + the `state` bundle + the `command` bundle into a `LeRobotDataset` (with video). | - | - | - |
| `rerun` | `record` writes every stream to an `.rrd` (cameras as H.264) and serves it live: joint plots, the URDF robot in 3D with its cameras and poses. `visualize` opens a viewer on that server. | - | x | x |
| `web_controller` | A web page answering each observation with a target. `joint` mode: a slider per joint. `pose` mode: jog buttons for the TCP pose. Planned: `episode`. | - | x | - |

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
