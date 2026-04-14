#!/usr/bin/env python3
import select
import sys
import termios

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from carmaker_msgs.msg import Control_Signal


HELP = """
Keyboard Teleop
---------------------------
Game-like WASD (carmaker_control mode):
  w : forward (auto drive gear + gas)
  s : reverse (auto rear gear + gas)
  a : steer left
  d : steer right
  x : coast (release gas/brake)
  space : emergency brake

Gear (carmaker_control mode):
  r : drive
  n : neutral
  f : rear
  p : parking

Scale:
  q/z : speed up/down max pedal
  e/c : speed up/down max steering angle

CTRL-C to quit
"""


class KeyboardTeleop(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "carmaker_control").strip().lower()
        self.topic = rospy.get_param("~topic", "/carmaker/control_signal")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))

        # Common limits
        self.speed_step = float(rospy.get_param("~speed_step", 0.2))
        self.steer_step = float(rospy.get_param("~steer_step", 0.05))
        self.max_speed = float(rospy.get_param("~max_speed", 3.0))
        self.max_steer = float(rospy.get_param("~max_steer", 0.5))
        self.max_pedal = float(rospy.get_param("~max_pedal", 0.7))
        self.max_accel = float(rospy.get_param("~max_accel", 3.0))
        self.default_gear = int(rospy.get_param("~default_gear", 2))

        # CarMaker12 gear mapping can vary by project; keep it configurable.
        self.drive_gear = int(rospy.get_param("~drive_gear", 2))
        self.neutral_gear = int(rospy.get_param("~neutral_gear", 1))
        self.reverse_gear = int(rospy.get_param("~reverse_gear", 0))
        self.park_gear = int(rospy.get_param("~park_gear", -9))

        # Game-like control response
        self.gas_step = float(rospy.get_param("~gas_step", 0.08))
        self.brake_step = float(rospy.get_param("~brake_step", 0.10))
        self.steer_return_rate = float(rospy.get_param("~steer_return_rate", 0.15))

        self.speed = 0.0
        self.steer = 0.0
        self.gas = 0.0
        self.brake = 0.0
        self.gear = self.default_gear

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
            msg.accel = self._clamp((msg.gas - msg.brake) * self.max_accel, -self.max_accel, self.max_accel)

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
                    if key == "w":
                        if self.gear != self.drive_gear:
                            self.gear = self.drive_gear
                            rospy.loginfo("auto gear=%d (drive)", self.gear)
                        self.gas = self._clamp(self.gas + self.gas_step, 0.0, self.max_pedal)
                    elif key == "s":
                        if self.gear != self.reverse_gear:
                            self.gear = self.reverse_gear
                            rospy.loginfo("auto gear=%d (rear)", self.gear)
                        self.gas = self._clamp(self.gas + self.gas_step, 0.0, self.max_pedal)
                    elif key == "x":
                        self.gas = 0.0
                        self.brake = 0.0
                    elif key == "a":
                        self.steer = self._clamp(self.steer + self.steer_step, -self.max_steer, self.max_steer)
                    elif key == "d":
                        self.steer = self._clamp(self.steer - self.steer_step, -self.max_steer, self.max_steer)
                    elif key == " ":
                        self.gas = 0.0
                        self.brake = self._clamp(self.brake + self.brake_step, 0.0, 1.0)
                        self.steer = 0.0
                    elif key == "r":
                        self.gear = self.drive_gear
                        rospy.loginfo("gear=%d (drive)", self.gear)
                    elif key == "n":
                        self.gear = self.neutral_gear
                        rospy.loginfo("gear=%d (neutral)", self.gear)
                    elif key == "f":
                        self.gear = self.reverse_gear
                        rospy.loginfo("gear=%d (rear)", self.gear)
                    elif key == "p":
                        self.gear = self.park_gear
                        rospy.loginfo("gear=%d (parking)", self.gear)
                    elif key == "q":
                        self.max_pedal = max(0.1, self.max_pedal * 1.1)
                        rospy.loginfo("max_pedal=%.3f", self.max_pedal)
                    elif key == "z":
                        self.max_pedal = max(0.1, self.max_pedal * 0.9)
                        self.gas = self._clamp(self.gas, 0.0, self.max_pedal)
                        rospy.loginfo("max_pedal=%.3f", self.max_pedal)
                    elif key == "e":
                        self.max_steer = max(0.01, self.max_steer * 1.1)
                        rospy.loginfo("max_steer=%.3f", self.max_steer)
                    elif key == "c":
                        self.max_steer = max(0.01, self.max_steer * 0.9)
                        self.steer = self._clamp(self.steer, -self.max_steer, self.max_steer)
                        rospy.loginfo("max_steer=%.3f", self.max_steer)
                    elif key == "\x03":
                        break

                    # Natural steering return when A/D is not pressed.
                    if key not in ("a", "d"):
                        self.steer *= (1.0 - self._clamp(self.steer_return_rate, 0.0, 1.0))
                else:
                    if key == "w":
                        self.speed = self._clamp(self.speed + self.speed_step, -self.max_speed, self.max_speed)
                    elif key == "x":
                        self.speed = self._clamp(self.speed - self.speed_step, -self.max_speed, self.max_speed)
                    elif key == "a":
                        self.steer = self._clamp(self.steer + self.steer_step, -self.max_steer, self.max_steer)
                    elif key == "d":
                        self.steer = self._clamp(self.steer - self.steer_step, -self.max_steer, self.max_steer)
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
