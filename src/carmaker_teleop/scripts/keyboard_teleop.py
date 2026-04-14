#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from carmaker_msgs.msg import Control_Signal


HELP = """
Keyboard Teleop
---------------------------
Moving:
  w : increase speed target (+x)
  x : decrease speed target (-x)
  a : steer left
  d : steer right
  s : stop (speed=0, steer=0)
  space : emergency brake (speed=0, steer=0)

Gear (carmaker_control mode):
  r : gear drive (1)
  n : gear neutral (0)
  f : gear rear (-1)
  p : gear parking (-9)

Scale:
  q/z : speed up/down max speed
  e/c : speed up/down max steering angle

CTRL-C to quit
"""


class KeyboardTeleop(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "carmaker_control").strip().lower()
        self.topic = rospy.get_param("~topic", "/carmaker/control_signal")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))

        self.speed_step = float(rospy.get_param("~speed_step", 0.2))
        self.steer_step = float(rospy.get_param("~steer_step", 0.05))
        self.max_speed = float(rospy.get_param("~max_speed", 3.0))
        self.max_steer = float(rospy.get_param("~max_steer", 0.5))
        self.max_pedal = float(rospy.get_param("~max_pedal", 0.7))
        self.max_accel = float(rospy.get_param("~max_accel", 3.0))
        self.default_gear = int(rospy.get_param("~default_gear", 1))

        self.speed = 0.0
        self.steer = 0.0
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

            if self.speed >= 0.0:
                msg.gas = self._clamp(abs(self.speed) / max(0.01, self.max_speed), 0.0, self.max_pedal)
                msg.brake = 0.0
            else:
                msg.gas = 0.0
                msg.brake = self._clamp(abs(self.speed) / max(0.01, self.max_speed), 0.0, self.max_pedal)
            msg.accel = self._clamp(self.speed, -self.max_accel, self.max_accel)

            self.pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = self.speed
            msg.angular.z = self.steer
            self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            # Keep terminal in raw mode while teleop is running so keystrokes are not echoed.
            tty.setraw(sys.stdin.fileno())
            while not rospy.is_shutdown():
                key = self._get_key()

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
                elif key == "r":
                    self.gear = 1
                    rospy.loginfo("gear=1 (drive)")
                elif key == "n":
                    self.gear = 0
                    rospy.loginfo("gear=0 (neutral)")
                elif key == "f":
                    self.gear = -1
                    rospy.loginfo("gear=-1 (rear)")
                elif key == "p":
                    self.gear = -9
                    rospy.loginfo("gear=-9 (parking)")
                elif key == "q":
                    self.max_speed = max(0.1, self.max_speed * 1.1)
                    rospy.loginfo("max_speed=%.3f", self.max_speed)
                elif key == "z":
                    self.max_speed = max(0.1, self.max_speed * 0.9)
                    self.speed = self._clamp(self.speed, -self.max_speed, self.max_speed)
                    rospy.loginfo("max_speed=%.3f", self.max_speed)
                elif key == "e":
                    self.max_steer = max(0.01, self.max_steer * 1.1)
                    rospy.loginfo("max_steer=%.3f", self.max_steer)
                elif key == "c":
                    self.max_steer = max(0.01, self.max_steer * 0.9)
                    self.steer = self._clamp(self.steer, -self.max_steer, self.max_steer)
                    rospy.loginfo("max_steer=%.3f", self.max_steer)
                elif key == "\x03":
                    break

                self._publish()
                rate.sleep()
        finally:
            self.speed = 0.0
            self.steer = 0.0
            self._publish()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
    rospy.init_node("carmaker_keyboard_teleop")
    KeyboardTeleop().run()
