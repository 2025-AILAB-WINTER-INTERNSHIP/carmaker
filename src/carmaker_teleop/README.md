# carmaker_teleop (ROS1)

Keyboard teleoperation package for CarMaker manual driving.

## Publish modes

- `mode:=carmaker_control` publishes `carmaker_msgs/Control_Signal` (default)
- `mode:=twist` publishes `geometry_msgs/Twist`
- `mode:=ackermann` publishes `ackermann_msgs/AckermannDriveStamped`

## Run

```bash
# build
cd /workspace
cbp carmaker_teleop
source /workspace/install/setup.bash

# default: /carmaker/control_signal as carmaker_msgs/Control_Signal
roslaunch carmaker_teleop keyboard_teleop.launch
```

## Explicit CarMaker control mode

```bash
roslaunch carmaker_teleop keyboard_teleop.launch \
  mode:=carmaker_control \
  topic:=/carmaker/control_signal \
  default_gear:=2 \
  drive_gear:=2 neutral_gear:=1 reverse_gear:=0 park_gear:=-9
```

## If CarMaker expects Ackermann topic

```bash
roslaunch carmaker_teleop keyboard_teleop.launch mode:=ackermann topic:=/ackermann_cmd
```

## Key mapping (Game-like)

- `w`: forward (auto drive gear + gas)
- `s`: reverse (auto rear gear + gas)
- `x`: coast (release gas/brake)
- `a/d`: steer left/right
- `space`: brake
- `r/n/f/p`: gear drive/neutral/rear/parking
- `q/z`: increase/decrease max pedal
- `e/c`: increase/decrease max steer angle
- `Ctrl+C`: quit

## Useful params

- `topic` (default: `/carmaker/control_signal`)
- `rate` (default: `20.0`)
- `max_speed` (default: `3.0`)
- `max_steer` (default: `0.5`)
- `max_pedal` (default: `0.7`)
- `max_accel` (default: `3.0`)
- `default_gear` (default: `2`)
- `drive_gear` (default: `2`)
- `neutral_gear` (default: `1`)
- `reverse_gear` (default: `0`)
- `park_gear` (default: `-9`)
- `gas_step` (default: `0.08`)
- `brake_step` (default: `0.10`)
- `steer_return_rate` (default: `0.15`)
- `speed_step` (default: `0.2`)
- `steer_step` (default: `0.05`)
