# arm_primitives
Author: Johnalbert Garnica

One set of motion, gripper and planning-scene primitives for different robot arms, on top of
MoveIt 2 (ROS 2 Lyrical). The same code drives the **Franka FER** (with the Franka Hand) and the
**UR7e**. Each arm is described by a YAML file, so supporting another arm means adding a config,
not writing new code.

```
                 your script / node            ros2 service call / other languages
                        |                                     |
                  Robot (Python API)  <------------  primitives_node (ROS services)
             arm  |  gripper  |  scene  |  pick/place
                        |
        MoveIt move_group (move_action, execute_trajectory, compute_ik/fk,
                           compute_cartesian_path, apply_planning_scene)
                        |
      config/arms/fer.yaml  ->  franka_ros2 (mock or real)   + franka_gripper
      config/arms/ur7e.yaml ->  ur_mock.launch.py (mock)     | ur_robot_driver (real)
```

## Status
Tested on ros2_control mock hardware with MoveIt for both arms (`ros2 run arm_primitives demo`
passes on `fer` and `ur7e`: named/joint/pose/relative/Cartesian moves land within 2 mm and
0.015 rad, IK/FK, planning scene, Franka gripper, pick and place, attach/detach) and through
every service. Not yet run on the real robots: the `real` bringups include the vendors' launch
files (`franka_fer_moveit_config/real.launch.py`, `ur_robot_driver/ur_control.launch.py` +
`ur_moveit_config/ur_moveit.launch.py`) and are untested.

## Packages
| Package | Contents |
|---|---|
| `arm_primitives` | Python API (`Robot`, `Arm`, `Gripper`, `Scene`), the `primitives_node` service server, the `demo` node, arm configs and launch files |
| `arm_primitives_interfaces` | Service definitions for `primitives_node` |

## Dependencies
ROS 2 Lyrical with MoveIt 2 and ros2_control (`ros-lyrical-moveit`, `ros-lyrical-ros2-controllers`).

**Franka FER**: [franka_ros2-jasgcode](https://github.com/jasgcode/franka_ros2-jasgcode) (a copy of
m-elwin's Lyrical fork of `franka_ros2`), plus `franka_description` and `libfranka` from
[m-elwin](https://github.com/m-elwin). Build them as their own workspace in `src/franka`:
```bash
cd src/franka && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

**UR7e**: `ur_description` and `ur_moveit_config`. With apt:
```bash
sudo apt install ros-lyrical-ur-description ros-lyrical-ur-moveit-config
```
or from source in `src/ur` (the same versions as apt):
```bash
mkdir -p src/ur && cd src/ur
git clone --depth 1 --branch 4.3.1 https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description
git clone --depth 1 --branch 6.1.0 https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
colcon build --merge-install --packages-select ur_description ur_moveit_config --cmake-args -DBUILD_TESTING=OFF
```
A real UR7e also needs the driver: `sudo apt install ros-lyrical-ur-robot-driver`.

## Build
This repo lives in `<ws>/src/arm_primitives`. With the arm workspaces as underlays (deactivate
any Python venv first):
```bash
source /opt/ros/lyrical/setup.bash
source src/franka/install/setup.bash
source src/ur/install/setup.bash        # skip if the UR packages came from apt
colcon build --packages-select arm_primitives_interfaces arm_primitives
source install/setup.bash
```

## Quick start
Bring up an arm on mock hardware (rviz and the service server start too):
```bash
ros2 launch arm_primitives arm.launch.py arm:=fer
```
```bash
ros2 launch arm_primitives arm.launch.py arm:=ur7e
```
Real hardware: `hardware:=real robot_ip:=<address>`. Other arguments: `rviz:=false`,
`server:=false`, and `arm:=` also takes a path to your own YAML file.

Run every primitive on whichever arm is up (it checks where the arm ends up after each one):
```bash
ros2 run arm_primitives demo --ros-args -p arm:=fer
```

## Python API
```python
from arm_primitives import Robot, run


async def main(robot: Robot) -> None:
    arm = robot.arm
    await arm.move_to_named('ready')
    print(await arm.ee_pose())  # Pose of ee_link in base_frame
    await arm.move_relative((0.0, 0.0, -0.05))  # straight down 5 cm
    await arm.move_relative(rotation=(0.0, 0.0, 0.5), frame='tool')
    grasp = arm.tool_down_pose(0.45, 0.1, 0.15)
    await robot.scene.add_box('cube', grasp, (0.04, 0.04, 0.04))
    if robot.gripper.available:
        await robot.pick(grasp, 'cube', width=0.04)
        await robot.place(arm.tool_down_pose(0.45, -0.1, 0.15), 'cube')


run(main, arm='ur7e')  # or: --ros-args -p arm:=fer
```
Inside your own node, create `Robot(node, 'fer')` and await the primitives from callbacks while
the node spins in a `MultiThreadedExecutor`.

| Object | Primitives |
|---|---|
| `robot.arm` | `move_to_named`, `move_to_joints`, `move_to_pose` (`linear=True` for a straight line), `move_cartesian`, `move_relative` (`frame='base'` or `'tool'`), `execute`, `stop`, `ee_pose`, `fk`, `ik`, `joint_positions`, `named_poses`, `tool_down_pose`, `pose_constraints` |
| `robot.gripper` | `open`, `close`, `move(width)`, `grasp(width, force)`, `home`, `available` |
| `robot.scene` | `add_box`, `add_cylinder`, `add_sphere`, `load`, `remove`, `clear`, `attach`, `detach` |
| `robot` | `pick`, `place` |
| `arm_primitives.geometry` | `make_pose`, `offset_pose`, `from_rpy`, `to_rpy`, `position_error`, `orientation_error`, ... |

Every `move_*` plans, executes (`execute=False` to only plan) and returns the
`moveit_msgs/RobotTrajectory`. Speeds default to the config and can be set per call with
`velocity_scaling` / `acceleration_scaling`. Failures raise `PrimitiveError` (or `GripperError`).

## Services
`primitives_node` (started by `arm.launch.py`) serves the same primitives under `/arm_primitives/`:

| Service | Type |
|---|---|
| `move_to_named`, `move_to_joints`, `move_to_pose`, `move_relative` | `arm_primitives_interfaces/srv/MoveTo*`, `MoveRelative` |
| `gripper` | `arm_primitives_interfaces/srv/Gripper` (open, close, grasp, move, home) |
| `scene` | `arm_primitives_interfaces/srv/SceneObject` (add_box, add_cylinder, add_sphere, remove, clear, attach, detach) |
| `pick_place` | `arm_primitives_interfaces/srv/PickPlace` |
| `get_state` | `arm_primitives_interfaces/srv/GetArmState` |
| `stop` | `std_srvs/srv/Trigger` (answered while a motion runs) |

```bash
ros2 service call /arm_primitives/move_relative arm_primitives_interfaces/srv/MoveRelative "{translation: {z: -0.05}, frame: base, linear: true}"
```

## Adding an arm
Copy `arm_primitives/config/arms/ur7e.yaml` and change:
- `planning_group`, `base_frame`, `ee_link`, `joints`: from the arm's MoveIt config (SRDF).
- `named_poses`: extra joint configurations (SRDF `group_state`s are picked up automatically).
- `tool_down`: end-effector orientation, as a quaternion, with the tool pointing down.
- `gripper`: `none`, `franka`, or `gripper_command` (any `control_msgs/GripperCommand` server).
- `bringup`: the launch files to include for `mock` and `real`; `{robot_ip}` and `{rviz}` are
  filled in from the `arm.launch.py` arguments.

Then use `arm:=<name>` after rebuilding, or `arm:=/path/to/arm.yaml` without rebuilding.

## Development
Lint and type-check with the workspace's ruff and ty (configs in this repo):
```bash
ruff format "$PWD" && ruff check "$PWD" && ty check
```
Unit tests: `colcon test --packages-select arm_primitives` or `python3 -m pytest arm_primitives/test`.
