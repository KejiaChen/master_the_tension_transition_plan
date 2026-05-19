# `master_the_tension_transition_plan`

Paper-release package for the central planning workflow used in the tension transition experiments.

## What This Package Contains

This package integrates the workflow logic that was previously split across:

- `mtc_tutorial`
- `moveit2_tutorials` for `isaac_demo_dualarms.launch.py`
- `solution_subscriber_pkg` for `subtrajectory_subscriber`

The robot configuration package is still kept external:

- `dual_arm_panda_moveit_config`

Scene assets are also still external in the current release:

- `~/ws_humble/scene/trans`

## Before Running

Always use the ROS Humble environment and deactivate Conda before building or running ROS packages.

Example:

```bash
conda deactivate
cd ~/ws_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Workflow

### 1. Launch RViz and the central planning scene

```bash
ros2 launch master_the_tension_transition_plan central_planning_bringup.launch.py
```

Notes:

- default hardware mode is `mock_components`
- if needed, you can still pass `ros2_control_hardware_type:=isaac`

### 2. Load the online DLO model as piecewise linear markers

Use:

```bash
ros2 launch master_the_tension_transition_plan load_dlo.launch.py
```

### 3. Launch the planner

Use:

```bash
ros2 launch master_the_tension_transition_plan pick_place_demo_dual.launch.py clip_ids:="[6, 7, 8]" exe:=dual_mtc_routing alter_finger_left:=false clip_added_from_blender:=false attach_pull_cable:=true attach_transport_cable:=true
```

When planning for objects added from Blender, set:

```bash
clip_added_from_blender:=true
```

### 4. Load obstacles and clip fixtures

For the plane setup with obstacles:

```bash
ros2 launch master_the_tension_transition_plan load_scene.launch.py \
scene_file:=/home/tp2/ws_humble/scene/trans/trans_env_adapt_z.scene \
mesh_file:=/home/tp2/ws_humble/scene/trans/mesh/qb_board_plane_with_obs_2.stl \
clip_file:=/home/tp2/ws_humble/scene/trans/target_shape_plane_7_clip_qb.scene \
use_qb_board_coordinate:=true \
add_clip_hats:=true
```

For the BMW shape loaded from Blender:

```bash
ros2 launch master_the_tension_transition_plan load_scene.launch.py \
scene_file:=/home/tp2/ws_humble/scene/trans/trans_env_blender_plane.scene \
mesh_file:=/home/tp2/ws_humble/scene/trans/mesh/qb_board_bmw_with_clip_high.stl \
clip_file:=/home/tp2/ws_humble/scene/trans/target_shape_bmw_clip_high_qb.scene \
use_qb_board_coordinate:=true \
add_clip_hats:=false
```

Notes:

- set `use_qb_board_coordinate:=false` when using `_world.scene`
- the launch file defaults use `~/ws_humble/...`, but the explicit examples above keep the original absolute paths for clarity

### 5. Send planned trajectories to robot PCs through the trajectory server

Use:

```bash
ros2 run master_the_tension_transition_plan subtrajectory_subscriber --left_mios_ip 10.157.174.87 --right_mios_ip 10.157.174.97
```

If you only want the trajectories to be saved locally on the central PC, use:

```bash
ros2 run master_the_tension_transition_plan subtrajectory_subscriber --left_mios_ip local --right_mios_ip local
```

Optional:

- by default, the subscriber writes under `~/ws_humble/trajectories_follower` and `~/ws_humble/trajectories_leader`
- you can override the workspace root with:

```bash
ros2 run master_the_tension_transition_plan subtrajectory_subscriber --workspace_dir /path/to/workspace
```

### 6. Step control in planning

After steps 1 to 5 are launched:

- click `Continue` in `RvizVisualToolsGui` to plan the next step
- click it again when the planner terminal shows `Waiting to continue`

### 7. Save trajectories on the robot PCs

By default, planned trajectories received by the trajectory server are saved in:

- `<mios-folder>/traj_folder`

They can be overwritten when new trajectories with the same name arrive, so move them to a permanent subfolder if you want to keep them.

## Old To New Command Mapping

Old:

```bash
ros2 launch moveit2_tutorials isaac_demo_dualarms.launch.py
ros2 launch mtc_tutorial load_dlo.launch.py
ros2 launch mtc_tutorial pick_place_demo_dual.launch.py ...
ros2 launch mtc_tutorial load_scene.launch.py ...
ros2 run solution_subscriber_pkg subtrajectory_subscriber ...
```

New:

```bash
ros2 launch master_the_tension_transition_plan central_planning_bringup.launch.py
ros2 launch master_the_tension_transition_plan load_dlo.launch.py
ros2 launch master_the_tension_transition_plan pick_place_demo_dual.launch.py ...
ros2 launch master_the_tension_transition_plan load_scene.launch.py ...
ros2 run master_the_tension_transition_plan subtrajectory_subscriber ...
```

## Current Release Boundary

This package currently does not vendor:

- `dual_arm_panda_moveit_config`
- `moveit_resources_panda_description`
- scene assets under `~/ws_humble/scene/trans`

Those are still required in the workspace for the full workflow to run.

## Verification Status

Current status:

- package build succeeded
- launch and runtime verification are intended to be checked manually next
