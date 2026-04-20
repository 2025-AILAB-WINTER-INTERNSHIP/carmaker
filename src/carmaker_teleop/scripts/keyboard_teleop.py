#!/usr/bin/python3
import select
import sys
import termios

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

from carmaker_msgs.msg import Control_Signal

HELP = """
Keyboard Teleop
---------------------------
Game-like WASD (carmaker_control mode):
  w : forward (auto drive gear + gas)
  s : reverse (auto rear gear + gas)
  a : steer left
  d : steer right
  q : forward-left  (w+a)
  e : forward-right (w+d)
  z : reverse-left  (s+a)
  c : reverse-right (s+d)
  x : coast (release gas/brake)
  space : emergency brake
  m : toggle precision parking mode

Gear (carmaker_control mode):
  r : drive
  n : neutral
  f : rear
  p : parking

CTRL-C to quit
"""


class KeyboardTeleop(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "carmaker_control").strip().lower()
        self.topic = rospy.get_param("~topic", "/carmaker/control_signal")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))

        # Common limits
        self.speed_step = float(rospy.get_param("~speed_step", 0.2))
        self.steer_step = float(rospy.get_param("~steer_step", 0.08))
        self.max_speed = float(rospy.get_param("~max_speed", 3.0))
        self.max_steer = float(rospy.get_param("~max_steer", 0.65))
        self.max_pedal = float(rospy.get_param("~max_pedal", 0.7))
        self.max_accel = float(rospy.get_param("~max_accel", 3.0))
        self.default_gear = int(rospy.get_param("~default_gear", 1))

        # CarMaker12 gear mapping can vary by project; keep it configurable.
        self.drive_gear = int(rospy.get_param("~drive_gear", 1))
        self.neutral_gear = int(rospy.get_param("~neutral_gear", 0))
        self.reverse_gear = int(rospy.get_param("~reverse_gear", -1))
        self.park_gear = int(rospy.get_param("~park_gear", -9))

        # Game-like control response
        self.gas_step = float(rospy.get_param("~gas_step", 0.08))
        self.brake_step = float(rospy.get_param("~brake_step", 0.10))
        self.gas_decay_step = float(rospy.get_param("~gas_decay_step", 0.06))
        self.brake_decay_step = float(rospy.get_param("~brake_decay_step", 0.08))
        self.steer_return_rate = float(rospy.get_param("~steer_return_rate", 0.10))
        self.precision_gas_scale = float(rospy.get_param("~precision_gas_scale", 0.10))
        self.precision_steer_scale = float(
            rospy.get_param("~precision_steer_scale", 1.60)
        )
        self.precision_steer_return_scale = float(
            rospy.get_param("~precision_steer_return_scale", 0.50)
        )
        self.precision_max_pedal = float(
            rospy.get_param("~precision_max_pedal", 0.10)
        )

        self.speed = 0.0
        self.steer = 0.0
        self.gas = 0.0
        self.brake = 0.0
        self.gear = self.default_gear
        self.precision_mode = bool(rospy.get_param("~precision_mode", False))

        if self.mode == "ackermann":
            self.pub = rospy.Publisher(self.topic, AckermannDriveStamped, queue_size=10)
        elif self.mode == "carmaker_control":
            self.pub = rospy.Publisher(self.topic, Control_Signal, queue_size=10)
        else:
            self.mode = "twist"
            self.pub = rospy.Publisher(self.topic, Twist, queue_size=10)

        self.settings = termios.tcgetattr(sys.stdin)
        rospy.loginfo("carmaker_teleop mode=%s topic=%s", self.mode, self.topic)
        rospy.loginfo(HELP)

    def _get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1) if rlist else ""

    @staticmethod
    def _clamp(value, vmin, vmax):
        return max(vmin, min(vmax, value))

    def _publish(self):
        if self.mode == "ackermann":
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.drive.speed = self.speed
            msg.drive.steering_angle = self.steer
            self.pub.publish(msg)
        elif self.mode == "carmaker_control":
            msg = Control_Signal()
            msg.header.stamp = rospy.Time.now()
            msg.steerangle = self.steer
            msg.gear = self.gear
            msg.gas = self._clamp(self.gas, 0.0, self.max_pedal)
            msg.brake = self._clamp(self.brake, 0.0, 1.0)
            direction = -1.0 if msg.gear == self.reverse_gear else 1.0
            msg.accel = self._clamp(
                direction * (msg.gas - msg.brake) * self.max_accel,
                -self.max_accel,
                self.max_accel,
            )

            self.pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = self.speed
            msg.angular.z = self.steer
            self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            # Keep terminal in raw/no-echo mode while teleop is running.
            raw = termios.tcgetattr(sys.stdin)
            raw[3] = raw[3] & ~(termios.ECHO | termios.ICANON)
            raw[6][termios.VMIN] = 0
            raw[6][termios.VTIME] = 0
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, raw)

            while not rospy.is_shutdown():
                key = self._get_key()

                if self.mode == "carmaker_control":
                    if key == "":
                        gas_decay = self.gas_decay_step
                        brake_decay = self.brake_decay_step
                        if self.precision_mode:
                            gas_decay *= 0.8
                            brake_decay *= 0.8
                        self.gas = self._clamp(self.gas - gas_decay, 0.0, self.max_pedal)
                        self.brake = self._clamp(self.brake - brake_decay, 0.0, 1.0)

                    # Composite keys (q/e/z/c) should apply throttle and steering together.
                    if key in ("w", "q", "e"):
                        if self.gear != self.drive_gear:
                            self.gear = self.drive_gear
                            rospy.loginfo("auto gear=%d (drive)", self.gear)
                        self.brake = 0.0
                        gas_step = self.gas_step
                        if self.precision_mode:
                            gas_step = self.gas_step * self.precision_gas_scale
                        gas_cap = (
                            min(self.max_pedal, self.precision_max_pedal)
                            if self.precision_mode
                            else self.max_pedal
                        )
                        self.gas = self._clamp(self.gas + gas_step, 0.0, gas_cap)
                    if key in ("s", "z", "c"):
                        if self.gear != self.reverse_gear:
                            self.gear = self.reverse_gear
                            rospy.loginfo("auto gear=%d (rear)", self.gear)
                        self.brake = 0.0
                        gas_step = self.gas_step
                        if self.precision_mode:
                            gas_step = self.gas_step * self.precision_gas_scale
                        gas_cap = (
                            min(self.max_pedal, self.precision_max_pedal)
                            if self.precision_mode
                            else self.max_pedal
                        )
                        self.gas = self._clamp(self.gas + gas_step, 0.0, gas_cap)
                    if key == "x":
                        self.gas = 0.0
                        self.brake = 0.0
                    if key in ("a", "q", "z"):
                        steer_step = self.steer_step
                        if self.precision_mode:
                            steer_step = self.steer_step * self.precision_steer_scale
                        self.steer = self._clamp(
                            self.steer + steer_step,
                            -self.max_steer,
                            self.max_steer,
                        )
                    if key in ("d", "e", "c"):
                        steer_step = self.steer_step
                        if self.precision_mode:
                            steer_step = self.steer_step * self.precision_steer_scale
                        self.steer = self._clamp(
                            self.steer - steer_step,
                            -self.max_steer,
                            self.max_steer,
                        )
                    if key == " ":
                        self.gas = 0.0
                        self.brake = self._clamp(self.brake + self.brake_step, 0.0, 1.0)
                        self.steer = 0.0
                    if key == "r":
                        self.gear = self.drive_gear
                        rospy.loginfo("gear=%d (drive)", self.gear)
                    if key == "n":
                        self.gear = self.neutral_gear
                        rospy.loginfo("gear=%d (neutral)", self.gear)
                    if key == "f":
                        self.gear = self.reverse_gear
                        rospy.loginfo("gear=%d (rear)", self.gear)
                    if key == "p":
                        self.gear = self.park_gear
                        rospy.loginfo("gear=%d (parking)", self.gear)
                    if key == "m":
                        self.precision_mode = not self.precision_mode
                        rospy.loginfo(
                            "precision_mode=%s (max_pedal_cap=%.3f)",
                            "ON" if self.precision_mode else "OFF",
                            self.precision_max_pedal,
                        )
                    if key == "\x03":
                        break

                    # Natural steering return when A/D is not pressed.
                    if key not in ("a", "d", "q", "e", "z", "c"):
                        steer_return = self.steer_return_rate
                        if self.precision_mode:
                            steer_return *= self.precision_steer_return_scale
                        self.steer *= 1.0 - self._clamp(steer_return, 0.0, 1.0)
                else:
                    if key == "w":
                        self.speed = self._clamp(
                            self.speed + self.speed_step,
                            -self.max_speed,
                            self.max_speed,
                        )
                    elif key == "x":
                        self.speed = self._clamp(
                            self.speed - self.speed_step,
                            -self.max_speed,
                            self.max_speed,
                        )
                    elif key == "a":
                        self.steer = self._clamp(
                            self.steer + self.steer_step,
                            -self.max_steer,
                            self.max_steer,
                        )
                    elif key == "d":
                        self.steer = self._clamp(
                            self.steer - self.steer_step,
                            -self.max_steer,
                            self.max_steer,
                        )
                    elif key in ("s", " "):
                        self.speed = 0.0
                        self.steer = 0.0
                    elif key == "\x03":
                        break

                self._publish()
                rate.sleep()
        finally:
            self.speed = 0.0
            self.steer = 0.0
            self.gas = 0.0
            self.brake = 0.0
            self._publish()
            sys.stdout.write("\n")
            sys.stdout.flush()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
    rospy.init_node("carmaker_keyboard_teleop")
    KeyboardTeleop().run()
