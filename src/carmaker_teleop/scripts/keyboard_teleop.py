#!/usr/bin/python3
import select
import sys
import termios
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from carmaker_msgs.msg import Control_Signal

HELP = """
===========================================================
 CarMaker ROS Keyboard Teleop (Perfect Max Speed Lock)
===========================================================
 [WASD 변형 게임 스타일 조작]
  w : 전진 (자동 Drive 기어 + 가속)
  s : 후진 (자동 Reverse 기어 + 가속)
  a / d : 좌 / 우 조향
  q / e : 전진 좌회전 / 전진 우회전
  z / c : 후진 좌회전 / 후진 우회전
  x : 코스팅 (가속/브레이크 페달 모두 해제)
  space : 긴급 제동 및 조향 정렬
  m : 정밀 주차 모드 토글 (속도 및 가속 제한)

 [수동 기어 조작]
  r : Drive(전진)  |  n : Neutral(중립)
  f : Reverse(후진)|  p : Parking(주차)

 CTRL-C를 누르면 안전하게 종료됩니다.
===========================================================
"""

class KeyboardTeleop(object):
    def __init__(self):
        # 파라미터 로드
        self.mode = rospy.get_param("~mode", "carmaker_control").strip().lower()
        self.topic = rospy.get_param("~topic", "/carmaker/control_signal")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))

        # 제한 제원
        self.speed_step = float(rospy.get_param("~speed_step", 0.2))
        self.steer_step = float(rospy.get_param("~steer_step", 0.08))
        self.max_speed = float(rospy.get_param("~max_speed", 3.0))
        self.max_steer = float(rospy.get_param("~max_steer", 0.65))
        self.max_pedal = float(rospy.get_param("~max_pedal", 0.7))
        self.max_accel = float(rospy.get_param("~max_accel", 3.0))
        
        # 기어 정의
        self.drive_gear = int(rospy.get_param("~drive_gear", 1))
        self.neutral_gear = int(rospy.get_param("~neutral_gear", 0))
        self.reverse_gear = int(rospy.get_param("~reverse_gear", -1))
        self.park_gear = int(rospy.get_param("~park_gear", -9))

        # 페달 스텝 및 감쇄
        self.gas_step = float(rospy.get_param("~gas_step", 0.08))
        self.brake_step = float(rospy.get_param("~brake_step", 0.10))
        self.gas_decay_step = float(rospy.get_param("~gas_decay_step", 0.06))
        self.brake_decay_step = float(rospy.get_param("~brake_decay_step", 0.08))
        self.steer_return_rate = float(rospy.get_param("~steer_return_rate", 0.10))
        
        # 정밀 모드 제원
        self.precision_mode = bool(rospy.get_param("~precision_mode", False))
        self.precision_gas_scale = float(rospy.get_param("~precision_gas_scale", 0.10))
        self.precision_steer_scale = float(rospy.get_param("~precision_steer_scale", 1.60))
        self.precision_steer_return_scale = float(rospy.get_param("~precision_steer_return_scale", 0.50))
        self.precision_max_pedal = float(rospy.get_param("~precision_max_pedal", 0.10))

        # 상태 변수
        self.speed = 0.0
        self.steer = 0.0
        self.gas = 0.0
        self.brake = 0.0
        self.gear = int(rospy.get_param("~default_gear", 1))
        
        # 최고 속도 락을 위한 제어 변수들
        self.virtual_speed = 0.0
        self.calculated_accel = 0.0  # 시뮬레이터로 보낼 최종 억제된 가속도

        # Publisher
        if self.mode == "ackermann":
            self.pub = rospy.Publisher(self.topic, AckermannDriveStamped, queue_size=10)
        elif self.mode == "carmaker_control":
            self.pub = rospy.Publisher(self.topic, Control_Signal, queue_size=10)
        else:
            self.mode = "twist"
            self.pub = rospy.Publisher(self.topic, Twist, queue_size=10)

        self.settings = termios.tcgetattr(sys.stdin)
        rospy.loginfo("carmaker_teleop 가동 | Mode: %s | Topic: %s", self.mode, self.topic)
        print(HELP)

    def _get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1) if rlist else ""

    @staticmethod
    def _clamp(value, vmin, vmax):
        return max(vmin, min(vmax, value))

    def _publish(self):
        """현재 제어 상태를 퍼블리시 (최고 속도 도달 시 가속도 제한 반영)"""
        if self.mode == "ackermann":
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.drive.speed = self.speed
            msg.drive.steering_angle = self.steer
        elif self.mode == "carmaker_control":
            msg = Control_Signal()
            msg.header.stamp = rospy.Time.now()
            msg.steerangle = self.steer
            msg.gear = self.gear
            msg.gas = self._clamp(self.gas, 0.0, self.max_pedal)
            msg.brake = self._clamp(self.brake, 0.0, 1.0)
            
            # 무조건적인 페달 계산이 아닌, 물리 한계가 반영된 가속도(calculated_accel)를 송신
            msg.accel = self.calculated_accel
        else:
            msg = Twist()
            msg.linear.x = self.speed
            msg.angular.z = self.steer

        self.pub.publish(msg)

    def _update_carmaker_physics(self, key):
        """사용자가 키를 누르고 있어도 꿀렁임 없이 최고 속도를 부드럽게 유지하는 로직"""
        dt = 1.0 / self.rate_hz
        direction = -1.0 if self.gear == self.reverse_gear else 1.0
        
        # 1. 페달 입력 기반 기본 가속도 계산
        accel = direction * (self.gas - self.brake) * self.max_accel
        if key == "":
            accel -= 0.6 * self.virtual_speed  # 마찰 저항
            
        # 2. 미래 예측 속도 계산
        predicted_speed = self.virtual_speed + accel * dt
        
        # 3. 최고 속도 제한 조건 검사 (가속 방향일 때만 차단)
        if direction > 0 and predicted_speed >= self.max_speed:
            # 전진 중 최고 속도 도달 -> 속도는 max_speed 고정, 유지 가속도는 0으로 락
            self.virtual_speed = self.max_speed
            self.calculated_accel = max(0.0, -self.brake * self.max_accel) # 브레이크 밟을 때만 감속 허용
        elif direction < 0 and predicted_speed <= -self.max_speed:
            # 후진 중 최고 속도 도달 -> 속도는 -max_speed 고정, 유지 가속도는 0으로 락
            self.virtual_speed = -self.max_speed
            self.calculated_accel = min(0.0, self.brake * self.max_accel)
        else:
            # 한계에 도달하지 않았다면 정상적으로 물리 법칙 적용
            self.virtual_speed = predicted_speed
            self.calculated_accel = self._clamp(accel, -self.max_accel, self.max_accel)
            
        # 중립/주차 기어 및 비정상 방향 흐름 방지 안전 클램핑
        if self.gear in (self.neutral_gear, self.park_gear):
            self.virtual_speed *= 0.7
            self.calculated_accel = -0.5 * self.virtual_speed
        elif self.gear == self.drive_gear and self.virtual_speed < 0:
            self.virtual_speed = 0.0
        elif self.gear == self.reverse_gear and self.virtual_speed > 0:
            self.virtual_speed = 0.0

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            raw = termios.tcgetattr(sys.stdin)
            raw[3] = raw[3] & ~(termios.ECHO | termios.ICANON)
            raw[6][termios.VMIN], raw[6][termios.VTIME] = 0, 0
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, raw)

            while not rospy.is_shutdown():
                key = self._get_key()
                if key == "\x03":
                    break

                if self.mode == "carmaker_control":
                    g_scale = self.precision_gas_scale if self.precision_mode else 1.0
                    s_scale = self.precision_steer_scale if self.precision_mode else 1.0
                    ret_scale = self.precision_steer_return_scale if self.precision_mode else 1.0
                    gas_cap = min(self.max_pedal, self.precision_max_pedal) if self.precision_mode else self.max_pedal

                    # 페달 자연 감쇄
                    if key == "":
                        decay_multiplier = 0.8 if self.precision_mode else 1.0
                        self.gas = self._clamp(self.gas - self.gas_decay_step * decay_multiplier, 0.0, self.max_pedal)
                        self.brake = self._clamp(self.brake - self.brake_decay_step * decay_multiplier, 0.0, 1.0)

                    # 키 입력 매핑 (w를 꾹 누르고 있어도 self.gas는 차단당하지 않고 최댓값 유지)
                    if key in ("w", "q", "e"):
                        if self.gear != self.drive_gear:
                            self.gear = self.drive_gear
                            rospy.loginfo("자동 기어 전환 -> DRIVE (%d)", self.gear)
                        self.brake = 0.0
                        self.gas = self._clamp(self.gas + self.gas_step * g_scale, 0.0, gas_cap)

                    elif key in ("s", "z", "c"):
                        if self.gear != self.reverse_gear:
                            self.gear = self.reverse_gear
                            rospy.loginfo("자동 기어 전환 -> REAR (%d)", self.gear)
                        self.brake = 0.0
                        self.gas = self._clamp(self.gas + self.gas_step * g_scale, 0.0, gas_cap)

                    if key == "x":
                        self.gas, self.brake = 0.0, 0.0

                    if key in ("a", "q", "z"):
                        self.steer = self._clamp(self.steer + self.steer_step * s_scale, -self.max_steer, self.max_steer)
                    elif key in ("d", "e", "c"):
                        self.steer = self._clamp(self.steer - self.steer_step * s_scale, -self.max_steer, self.max_steer)

                    if key == " ":
                        self.gas, self.steer = 0.0, 0.0
                        self.brake = self._clamp(self.brake + self.brake_step, 0.0, 1.0)

                    elif key == "r": self.gear = self.drive_gear; rospy.loginfo("기어 -> DRIVE (%d)", self.gear)
                    elif key == "n": self.gear = self.neutral_gear; rospy.loginfo("기어 -> NEUTRAL (%d)", self.gear)
                    elif key == "f": self.gear = self.reverse_gear; rospy.loginfo("기어 -> REAR (%d)", self.gear)
                    elif key == "p": self.gear = self.park_gear; rospy.loginfo("기어 -> PARKING (%d)", self.gear)
                    elif key == "m":
                        self.precision_mode = not self.precision_mode
                        rospy.loginfo("정밀 주차 모드: %s", "ON" if self.precision_mode else "OFF")

                    if key not in ("a", "d", "q", "e", "z", "c"):
                        self.steer *= (1.0 - self._clamp(self.steer_return_rate * ret_scale, 0.0, 1.0))

                    # 물리 시뮬레이션 및 속도 제한부 실행
                    self._update_carmaker_physics(key)

                else:
                    if key == "w": self.speed = self._clamp(self.speed + self.speed_step, -self.max_speed, self.max_speed)
                    elif key == "x": self.speed = self._clamp(self.speed - self.speed_step, -self.max_speed, self.max_speed)
                    elif key == "a": self.steer = self._clamp(self.steer + self.steer_step, -self.max_steer, self.max_steer)
                    elif key == "d": self.steer = self._clamp(self.steer - self.steer_step, -self.max_steer, self.max_steer)
                    elif key in ("s", " "): self.speed, self.steer = 0.0, 0.0

                self._publish()
                rate.sleep()

        finally:
            self.speed, self.steer, self.gas, self.brake, self.virtual_speed, self.calculated_accel = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            self._publish()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == "__main__":
    rospy.init_node("carmaker_keyboard_teleop")
    try:
        KeyboardTeleop().run()
    except rospy.ROSInterruptException:
        pass