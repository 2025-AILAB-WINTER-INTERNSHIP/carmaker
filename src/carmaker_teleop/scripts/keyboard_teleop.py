#!/usr/bin/python3
import select
import sys
import termios
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from carmaker_msgs.msg import Control_Signal

# CarMaker 차량 동역학 피드백
from carmaker_msgs.msg import DynamicsInfo

HELP = """
===========================================================
 CarMaker ROS Keyboard Teleop (Logging & Safe Fuel Cut)
===========================================================
  [주행] w: 전진 | s: 후진 | a/d: 좌우 조향 | q/e/z/c: 대각선
  [제어] space: 브레이크 | x: 코스팅(페달 해제) | m: 정밀 모드
  [기어] r: Drive | n: Neutral | f: Reverse | p: Parking
  (CTRL-C to quit)
===========================================================
"""

class KeyboardTeleop:
    def __init__(self):
        self.mode = rospy.get_param("~mode", "carmaker_control").strip().lower()
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.pub_topic = rospy.get_param("~control_topic", "/carmaker/control_signal")
        self.sub_topic = rospy.get_param("~dynamics_topic", "/carmaker/dynamics_info")

        self.limits = {
            "speed": float(rospy.get_param("~max_speed", 2.0)), 
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

        # 내부 상태 머신
        self.state = {"steer": 0.0, "gas": 0.0, "brake": 0.0, "gear": self.gears["drive"]}
        
        # 최종 전송 변수 및 시스템 감시 플래그
        self.actual_vx = 0.0 
        self.final_gas = 0.0
        self.final_brake = 0.0
        self.final_accel = 0.0
        
        self.feedback_received = False
        self.fuel_cut_active = False

        self.pub = rospy.Publisher(self.pub_topic, Control_Signal, queue_size=10)
        rospy.Subscriber(self.sub_topic, DynamicsInfo, self._dynamics_cb)

        self.settings = termios.tcgetattr(sys.stdin)
        rospy.loginfo(f"가동 완료 | 목표 최고 속도: {self.limits['speed']} m/s")
        print(HELP)

    def _get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1) if rlist else ""

    @staticmethod
    def _clamp(val, v_min, v_max):
        return max(v_min, min(v_max, val))

    def _dynamics_cb(self, msg):
        """CarMaker에서 올라오는 실제 속도 피드백 갱신"""
        if not self.feedback_received:
            rospy.loginfo(f"✅ 차량 피드백(DynamicsInfo) 수신 연결 완료! (정상 작동 중)")
            self.feedback_received = True
        self.actual_vx = msg.Car_vx

    def _calculate_commands(self):
        """실제 차량 속도를 피드백 받아 가스 페달과 가속도를 물리적으로 차단"""
        d_mult = 1.0 if self.state["gear"] == self.gears["drive"] else (-1.0 if self.state["gear"] == self.gears["reverse"] else 0.0)
        raw_accel = d_mult * (self.state["gas"] - self.state["brake"]) * self.limits["accel"]

        # 1. 기본 상태를 최종 송신 변수에 복사
        self.final_gas = self.state["gas"]
        self.final_brake = self.state["brake"]
        self.final_accel = self._clamp(raw_accel, -self.limits["accel"], self.limits["accel"])

        # 2. 퓨얼 컷 개입 (엔진 차단 로직)
        if d_mult > 0 and self.actual_vx >= self.limits["speed"]:
            self.final_gas = 0.0  # 엑셀 강제 해제
            self.final_accel = min(0.0, raw_accel)
            
            if not self.fuel_cut_active:
                rospy.logwarn(f"🚨 [전진] 최고 속도({self.limits['speed']}m/s) 도달 -> 퓨얼 컷 개입!")
                self.fuel_cut_active = True

        elif d_mult < 0 and self.actual_vx <= -self.limits["speed"]:
            self.final_gas = 0.0  # 엑셀 강제 해제
            self.final_accel = max(0.0, raw_accel)
            
            if not self.fuel_cut_active:
                rospy.logwarn(f"🚨 [후진] 최고 속도({self.limits['speed']}m/s) 도달 -> 퓨얼 컷 개입!")
                self.fuel_cut_active = True

        else:
            if self.fuel_cut_active:
                rospy.loginfo("🟢 한계 속도 미만 -> 퓨얼 컷 해제 (정상 가속)")
                self.fuel_cut_active = False

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            raw = termios.tcgetattr(sys.stdin)
            raw[3] = raw[3] & ~(termios.ECHO | termios.ICANON)
            raw[6][termios.VMIN], raw[6][termios.VTIME] = 0, 0
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, raw)

            while not rospy.is_shutdown():
                # 상태 변화 감지를 위한 이전 상태 저장
                prev_gear = self.state["gear"]
                prev_mode = self.precision_mode

                key = self._get_key()
                if key == "\x03": break

                # 통신 불량 감지 알림 (3초 주기로 경고)
                if not self.feedback_received:
                    rospy.logwarn_throttle(3.0, f"⚠️ 토픽({self.sub_topic}) 수신 대기 중... 연결을 확인하세요! (퓨얼 컷 미작동 상태)")

                intent = {
                    "gas_fwd": key in ("w", "q", "e"),
                    "gas_rev": key in ("s", "z", "c"),
                    "steer_l": key in ("a", "q", "z"),
                    "steer_r": key in ("d", "e", "c"),
                    "brake": key == " ",
                    "coast": key == "x"
                }
                is_gas = intent["gas_fwd"] or intent["gas_rev"]

                # 기어 및 모드 변경
                if intent["gas_fwd"] or key == 'r': self.state["gear"] = self.gears["drive"]
                elif intent["gas_rev"] or key == 'f': self.state["gear"] = self.gears["reverse"]
                elif key == 'n': self.state["gear"] = self.gears["neutral"]
                elif key == 'p': self.state["gear"] = self.gears["park"]
                elif key == 'm': self.precision_mode = not self.precision_mode

                # [복구된 로깅 기능] 기어 또는 모드가 변경되었을 때만 출력
                if self.state["gear"] != prev_gear:
                    gear_name = {v: k.upper() for k, v in self.gears.items()}.get(self.state["gear"], "UNKNOWN")
                    rospy.loginfo(f"⚙️ 기어 변경됨: {gear_name} ({self.state['gear']})")
                
                if self.precision_mode != prev_mode:
                    rospy.loginfo(f"🎯 정밀 주차 모드: {'활성화 (ON)' if self.precision_mode else '비활성화 (OFF)'}")

                g_cap = 0.1 if self.precision_mode else self.limits["pedal"]
                s_scale = 1.6 if self.precision_mode else 1.0

                # 페달 및 조향 상태 갱신
                if is_gas:
                    self.state["brake"] = 0.0
                    self.state["gas"] = self._clamp(self.state["gas"] + self.steps["gas"], 0.0, g_cap)
                elif intent["brake"]:
                    self.state["gas"] = 0.0
                    self.state["brake"] = self._clamp(self.state["brake"] + self.steps["brake"], 0.0, 1.0)
                elif intent["coast"]:
                    self.state["gas"], self.state["brake"] = 0.0, 0.0
                else: 
                    self.state["gas"] = self._clamp(self.state["gas"] - self.steps["gas_decay"], 0.0, g_cap)
                    self.state["brake"] = self._clamp(self.state["brake"] - self.steps["brake_decay"], 0.0, 1.0)

                if intent["steer_l"]:
                    self.state["steer"] = self._clamp(self.state["steer"] + self.steps["steer"] * s_scale, -self.limits["steer"], self.limits["steer"])
                elif intent["steer_r"]:
                    self.state["steer"] = self._clamp(self.state["steer"] - self.steps["steer"] * s_scale, -self.limits["steer"], self.limits["steer"])
                else:
                    self.state["steer"] *= (1.0 - self._clamp(self.steps["steer_return"], 0.0, 1.0))

                # 퓨얼 컷 및 최종 계산 실행
                self._calculate_commands()

                # 퍼블리시
                msg = Control_Signal()
                msg.header.stamp = rospy.Time.now()
                msg.steerangle, msg.gear = self.state["steer"], self.state["gear"]
                
                # 반드시 state["gas"]가 아닌, 차단이 적용된 final_gas를 전송!
                msg.gas, msg.brake, msg.accel = self.final_gas, self.final_brake, self.final_accel
                
                self.pub.publish(msg)
                rate.sleep()

        finally:
            self.state = {k: 0.0 for k in self.state} 
            self.pub.publish(Control_Signal()) 
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == "__main__":
    rospy.init_node("carmaker_keyboard_teleop")
    try: KeyboardTeleop().run()
    except rospy.ROSInterruptException: pass