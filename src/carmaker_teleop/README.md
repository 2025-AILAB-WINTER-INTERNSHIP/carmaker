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

# default: /control_signal as carmaker_msgs/Control_Signal
roslaunch carmaker_teleop keyboard_teleop.launch
```

## Explicit CarMaker control mode

```bash
roslaunch carmaker_teleop keyboard_teleop.launch \
  mode:=carmaker_control \
  topic:=/control_signal \
  default_gear:=1
```

## If CarMaker expects Ackermann topic

```bash
roslaunch carmaker_teleop keyboard_teleop.launch mode:=ackermann topic:=/ackermann_cmd
```

## Key mapping

- `w/x`: speed up/down
- `a/d`: steer left/right
- `s` or `space`: full stop
- `r/n/f/p`: gear drive/neutral/rear/parking (1/0/-1/-9)
- `q/z`: increase/decrease max speed
- `e/c`: increase/decrease max steer angle
- `Ctrl+C`: quit

## Useful params

- `topic` (default: `/control_signal`)
- `rate` (default: `20.0`)
- `max_speed` (default: `3.0`)
- `max_steer` (default: `0.5`)
- `max_pedal` (default: `0.7`)
- `max_accel` (default: `3.0`)
- `default_gear` (default: `1`)
- `speed_step` (default: `0.2`)
- `steer_step` (default: `0.05`)
