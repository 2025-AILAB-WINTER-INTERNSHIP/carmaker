#!/usr/bin/python3
import select
import sys
import termios
import rospy
import math

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from carmaker_msgs.msg import Control_Signal
from carmaker_msgs.msg import DynamicsInfo

HELP = """
===========================================================
 CarMaker ROS Keyboard Teleop (Closed-Loop Feedback)
===========================================================
  [주행] w: 전진 | s: 후진 | a/d: 좌우 조향 | q/e/z/c: 대각선
  [제어] space: 브레이크 | x: 코스팅(페달 해제) | m: 정밀 모드
  [기어] r: Drive | n: Neutral | f: Reverse | p: Parking
  (CTRL-C to quit)
===========================================================
"""

class KeyboardTeleop:
    def __init__(self):
        # 1. 기본 설정 및 제원 로드
        self.mode = rospy.get_param("~mode", "carmaker_control").strip().lower()
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.pub_topic = rospy.get_param("~control_topic", "/carmaker/control_signal")
        self.sub_topic = rospy.get_param("~dynamics_topic", "/carmaker/dynamics_info")

        self.limits = {
            "speed": float(rospy.get_param("~max_speed", 2.0)), # m/s (원하는 대로 정확히 제어됨)
            "steer": float(rospy.get_param("~max_steer", 0.65)),
            "pedal": float(rospy.get_param("~max_pedal", 0.7)),
            "accel": float(rospy.get_param("~max_accel", 3.0))
        }
        
        self.steps = {
            "gas": float(rospy.get_param("~gas_step", 0.08)),
            "brake": float(rospy.get_param("~brake_step", 0.10)),
            "steer": float(rospy.get_param("~steer_step", 0.08)),
            "gas_decay": float(rospy.get_param("~gas_decay_step", 0.06)),
            "brake_decay": float(rospy.get_param("~brake_decay_step", 0.08)),
            "steer_return": float(rospy.get_param("~steer_return_rate", 0.10))
        }

        self.gears = {
            "drive": int(rospy.get_param("~drive_gear", 1)),
            "neutral": int(rospy.get_param("~neutral_gear", 0)),
            "reverse": int(rospy.get_param("~reverse_gear", -1)),
            "park": int(rospy.get_param("~park_gear", -9))
        }

        self.precision_mode = bool(rospy.get_param("~precision_mode", False))

        # 2. 상태 변수 (State Machine)
        self.state = {"steer": 0.0, "gas": 0.0, "brake": 0.0, "gear": self.gears["drive"]}
        
        # 시뮬레이터에서 피드백 받는 실제 차량의 종방향 속도 (m/s)
        self.actual_vx = 0.0 
        self.final_accel = 0.0

        # 3. ROS Publisher & Subscriber 설정
        msg_types = {"ackermann": AckermannDriveStamped, "carmaker_control": Control_Signal, "twist": Twist}
        msg_class = msg_types.get(self.mode, Twist)
        self.pub = rospy.Publisher(self.pub_topic, msg_class, queue_size=10)
        
        # 실제 차량 상태 피드백 구독
        rospy.Subscriber(self.sub_topic, DynamicsInfo, self._dynamics_cb)

        self.settings = termios.tcgetattr(sys.stdin)
        rospy.loginfo(f"가동 완료 | Mode: {self.mode} | Dynamics Sub: {self.sub_topic}")
        print(HELP)

    def _get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1) if rlist else ""

    @staticmethod
    def _clamp(val, v_min, v_max):
        return max(v_min, min(v_max, val))

    def _dynamics_cb(self, msg):
        """CarMaker에서 올라오는 실제 속도(Car_vx) 피드백 갱신"""
        absolute_speed = math.hypot(msg.Car_vx, msg.Car_vy)
        direction = 1.0 if msg.Car_vx >= 0 else -1.0
        self.actual_vx = absolute_speed * direction

    def _calculate_commands(self, is_gas, is_brake):
        """실제 차량 속도(Car_vx)를 피드백 받아 가속도를 물리적으로 제한하는 클램프 로직"""
        d_mult = 1.0 if self.state["gear"] == self.gears["drive"] else (-1.0 if self.state["gear"] == self.gears["reverse"] else 0.0)
        
        # 1. 사용자의 페달 입력에 따른 목표 가속도
        raw_accel = d_mult * (self.state["gas"] - self.state["brake"]) * self.limits["accel"]

        # 2. 실제 속도 기반 최고 속도 제한 로직 (Feedback Limit)
        if d_mult > 0 and self.actual_vx >= self.limits["speed"]:
            # 전진 중 최고 속도(2.0m/s) 이상이면, 가속(양수)을 0으로 컷아웃 (브레이크인 음수만 허용)
            self.final_accel = min(0.0, raw_accel)
        elif d_mult < 0 and self.actual_vx <= -self.limits["speed"]:
            # 후진 중 최고 속도 이상이면, 후진 가속(음수)을 0으로 컷아웃 (브레이크인 양수만 허용)
            self.final_accel = max(0.0, raw_accel)
        else:
            # 한계 미만이면 사용자의 가속 명령을 그대로 인가
            self.final_accel = self._clamp(raw_accel, -self.limits["accel"], self.limits["accel"])

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            raw = termios.tcgetattr(sys.stdin)
            raw[3] = raw[3] & ~(termios.ECHO | termios.ICANON)
            raw[6][termios.VMIN], raw[6][termios.VTIME] = 0, 0
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, raw)

            while not rospy.is_shutdown():
                key = self._get_key()
                if key == "\x03": break

                # [STEP 1] 사용자 의도 파악
                intent = {
                    "gas_fwd": key in ("w", "q", "e"),
                    "gas_rev": key in ("s", "z", "c"),
                    "steer_l": key in ("a", "q", "z"),
                    "steer_r": key in ("d", "e", "c"),
                    "brake": key == " ",
                    "coast": key == "x"
                }
                is_gas = intent["gas_fwd"] or intent["gas_rev"]

                if intent["gas_fwd"] or key == 'r': self.state["gear"] = self.gears["drive"]
                elif intent["gas_rev"] or key == 'f': self.state["gear"] = self.gears["reverse"]
                elif key == 'n': self.state["gear"] = self.gears["neutral"]
                elif key == 'p': self.state["gear"] = self.gears["park"]
                elif key == 'm': self.precision_mode = not self.precision_mode

                g_cap = 0.1 if self.precision_mode else self.limits["pedal"]
                s_scale = 1.6 if self.precision_mode else 1.0

                # [STEP 2] 상태 갱신
                if self.mode == "carmaker_control":
                    # 종방향 페달 제어
                    if is_gas:
                        self.state["brake"] = 0.0
                        self.state["gas"] = self._clamp(self.state["gas"] + self.steps["gas"], 0.0, g_cap)
                    elif intent["brake"]:
                        self.state["gas"] = 0.0
                        self.state["brake"] = self._clamp(self.state["brake"] + self.steps["brake"], 0.0, 1.0)
                    elif intent["coast"]:
                        self.state["gas"], self.state["brake"] = 0.0, 0.0
                    else: # 자연 감쇄
                        self.state["gas"] = self._clamp(self.state["gas"] - self.steps["gas_decay"], 0.0, g_cap)
                        self.state["brake"] = self._clamp(self.state["brake"] - self.steps["brake_decay"], 0.0, 1.0)

                    # 횡방향 조향 제어
                    if intent["steer_l"]:
                        self.state["steer"] = self._clamp(self.state["steer"] + self.steps["steer"] * s_scale, -self.limits["steer"], self.limits["steer"])
                    elif intent["steer_r"]:
                        self.state["steer"] = self._clamp(self.state["steer"] - self.steps["steer"] * s_scale, -self.limits["steer"], self.limits["steer"])
                    else:
                        self.state["steer"] *= (1.0 - self._clamp(self.steps["steer_return"], 0.0, 1.0))

                    # [STEP 3] 피드백 기반 가속도 계산
                    self._calculate_commands(is_gas, intent["brake"])

                # [STEP 4] 퍼블리시
                msg = Control_Signal() if self.mode == "carmaker_control" else (AckermannDriveStamped() if self.mode == "ackermann" else Twist())
                
                if self.mode == "carmaker_control":
                    msg.header.stamp = rospy.Time.now()
                    msg.steerangle, msg.gear = self.state["steer"], self.state["gear"]
                    msg.gas, msg.brake, msg.accel = self.state["gas"], self.state["brake"], self.final_accel
                elif self.mode == "ackermann":
                    msg.header.stamp = rospy.Time.now()
                    # Ackermann 모드일 경우 가짜 speed 대신 실제 피드백 제한을 사용할 수 있으나 호환성을 위해 유지
                    msg.drive.steering_angle = self.state["steer"]
                else:
                    msg.angular.z = self.state["steer"]

                self.pub.publish(msg)
                rate.sleep()

        finally:
            self.state = {k: 0.0 for k in self.state} 
            self.pub.publish(Twist()) 
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == "__main__":
    rospy.init_node("carmaker_keyboard_teleop")
    try: KeyboardTeleop().run()
    except rospy.ROSInterruptException: pass