#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from carmaker_msgs.msg import TrajectoryPath

# 차량 사양 한계값 (기본값 설정, ROS 파라미터 서버로부터 로드 시도)
MAX_VEL = 10.0          # m/s
MAX_ACCEL = 3.0        # m/s^2
MAX_DECEL = 3.0        # m/s^2
MIN_TURNING_RADIUS = 5.5  # m
MAX_KAPPA = 1.0 / MIN_TURNING_RADIUS
YAW_TOLERANCE_DEG = 5.0  # deg (yaw alignment 허용 편차)
TIME_TOLERANCE_MS = 1.0   # ms (시간 일관성 허용 편차)

# 고차 미분값 제약 설정 (Jerk 및 조향 각속도)
MAX_JERK = 2.0          # m/s^3
MAX_STEER_VEL = 0.6108  # rad/s (타이어 조향각 기준 35.0 deg/s)
WHEELBASE = 2.97        # m

def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def load_params():
    global MAX_VEL, MAX_ACCEL, MAX_DECEL, MIN_TURNING_RADIUS, MAX_KAPPA, MAX_JERK, MAX_STEER_VEL, WHEELBASE, TIME_TOLERANCE_MS, YAW_TOLERANCE_DEG
    try:
        # GlobalPlannerNodelet 의 private namespace 또는 글로벌 namespace로부터 파라미터 로딩 시도 (Fallback Chain)
        MIN_TURNING_RADIUS = rospy.get_param('/GlobalPlannerNodelet/vehicle/min_turning_radius',
                             rospy.get_param('/vehicle/min_turning_radius', MIN_TURNING_RADIUS))
        MAX_VEL = rospy.get_param('/GlobalPlannerNodelet/vehicle/limits/max_vel',
                  rospy.get_param('/vehicle/limits/max_vel', MAX_VEL))
        MAX_ACCEL = rospy.get_param('/GlobalPlannerNodelet/vehicle/limits/max_accel',
                    rospy.get_param('/vehicle/limits/max_accel', MAX_ACCEL))
        MAX_DECEL = rospy.get_param('/GlobalPlannerNodelet/vehicle/limits/max_decel',
                    rospy.get_param('/vehicle/limits/max_decel', MAX_DECEL))
        MAX_JERK = rospy.get_param('/GlobalPlannerNodelet/vehicle/limits/max_jerk',
                   rospy.get_param('/vehicle/limits/max_jerk', MAX_JERK))
        MAX_STEER_VEL = rospy.get_param('/GlobalPlannerNodelet/vehicle/limits/max_steer_vel',
                        rospy.get_param('/vehicle/limits/max_steer_vel', MAX_STEER_VEL))
        WHEELBASE = rospy.get_param('/GlobalPlannerNodelet/vehicle/wheelbase',
                    rospy.get_param('/vehicle/wheelbase', WHEELBASE))
        TIME_TOLERANCE_MS = rospy.get_param('/GlobalPlannerNodelet/global_post_processing/validator/time_tolerance_ms',
                            rospy.get_param('/global_post_processing/validator/time_tolerance_ms', TIME_TOLERANCE_MS))
        YAW_TOLERANCE_DEG = rospy.get_param('/GlobalPlannerNodelet/global_post_processing/validator/yaw_tolerance_deg',
                            rospy.get_param('/global_post_processing/validator/yaw_tolerance_deg', YAW_TOLERANCE_DEG))
        MAX_KAPPA = 1.0 / MIN_TURNING_RADIUS
        rospy.loginfo("[LoadParams] Dynamic specs synchronized: R=%.2fm, wheelbase=%.2fm, max_jerk=%.2fm/s^3, max_steer_vel=%.4frad/s, max_vel=%.2fm/s, time_tol=%.2fms, yaw_tol=%.2fdeg", 
                      MIN_TURNING_RADIUS, WHEELBASE, MAX_JERK, MAX_STEER_VEL, MAX_VEL, TIME_TOLERANCE_MS, YAW_TOLERANCE_DEG)
    except Exception as e:
        rospy.logwarn("[LoadParams] Failed to load parameters from server, using default limits: %s", str(e))

def callback(msg):
    load_params()
    rospy.loginfo("Received trajectory with %d points.", len(msg.points))
    if len(msg.points) < 2:
        rospy.logwarn("Trajectory has less than 2 points. Cannot validate.")
        return

    n = len(msg.points)

    # 데이터 추출
    x = np.array([pt.pose.position.x for pt in msg.points])
    y = np.array([pt.pose.position.y for pt in msg.points])
    v = np.array([pt.longitudinal_velocity for pt in msg.points])
    a = np.array([pt.longitudinal_acceleration for pt in msg.points])
    kappa = np.array([pt.curvature for pt in msg.points])
    t = np.array([pt.time_from_start.to_sec() for pt in msg.points])

    # orientation에서 yaw 추출
    yaws = []
    for pt in msg.points:
        q = pt.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaws.append(yaw)
    yaws = np.array(yaws)

    # 방향 판별 (1: 전진, -1: 후진)
    direction = np.ones(n)
    for i in range(n):
        if v[i] > 1e-5:
            direction[i] = 1
        elif v[i] < -1e-5:
            direction[i] = -1
        else:
            if i > 0:
                direction[i] = direction[i-1]
            elif i < n - 1:
                next_nonzero = i
                while next_nonzero < n and abs(v[next_nonzero]) < 1e-5:
                    next_nonzero += 1
                if next_nonzero < n:
                    direction[i] = 1 if v[next_nonzero] > 0 else -1

    # 1. 속도 검증
    vel_violations = np.sum(np.abs(v) > MAX_VEL + 0.01)
    max_v = np.max(np.abs(v))

    # 2. 가속도 검증 (전진/후진 관계없이 휠 로컬 기준 가/감속 한계 검증)
    dynamic_accel = a * direction
    acc_violations = np.sum(dynamic_accel > MAX_ACCEL + 0.01) + np.sum(dynamic_accel < -MAX_DECEL - 0.01)
    max_a = np.max(dynamic_accel)
    min_a = np.min(dynamic_accel)

    # 3. 곡률 검증
    kappa_violations = np.sum(np.abs(kappa) > MAX_KAPPA + 1e-3)
    max_k = np.max(np.abs(kappa))

    # 4. 방향각(Yaw) 정합성 검증 (Cusp 포인트 및 근처 영역 스킵 처리 추가)
    # Cusp (방향 전환점) 인덱스 검출
    cusp_indices = []
    for i in range(1, n):
        if direction[i] != direction[i-1]:
            cusp_indices.append(i)

    yaw_violations_count = 0
    checked_yaw_count = 0
    max_yaw_err_deg = 0.0
    for i in range(1, n - 1):
        # Cusp 기준 물리적 거리 0.2m 이내 영역은 스킵
        near_cusp = False
        for cusp_idx in cusp_indices:
            if abs(i - cusp_idx) <= 5: # 해상도 0.1m 고려하여 인덱스 거리 5 이하일 때만 실거리 검사
                dist_to_cusp = math.hypot(x[i] - x[cusp_idx], y[i] - y[cusp_idx])
                if dist_to_cusp < 0.2:
                    near_cusp = True
                    break
        if near_cusp:
            continue

        dx = x[i+1] - x[i-1]
        dy = y[i+1] - y[i-1]
        ds = math.hypot(dx, dy)
        if ds < 1e-3:
            continue
        expected_yaw = math.atan2(dy, dx)
        # 역진(Reverse) 상태인 경우 180도 회전 보정
        if direction[i] == -1:
            expected_yaw = wrap_to_pi(expected_yaw + math.pi)

        yaw_err = abs(wrap_to_pi(yaws[i] - expected_yaw))
        yaw_err_deg = math.degrees(yaw_err)
        checked_yaw_count += 1
        if yaw_err_deg > YAW_TOLERANCE_DEG:
            yaw_violations_count += 1
        max_yaw_err_deg = max(max_yaw_err_deg, yaw_err_deg)

    # 5. 시간 일관성(Timestamp Consistency) 검증
    time_violations_count = 0
    max_time_err_sec = 0.0
    for i in range(1, n):
        dt = t[i] - t[i-1]
        if dt <= 1e-6:
            time_violations_count += 1
            continue
        ds = math.hypot(x[i] - x[i-1], y[i] - y[i-1])
        v_avg = (abs(v[i]) + abs(v[i-1])) / 2.0
        # 최소 속도 분모 필터 적용
        expected_dt = ds / max(0.02, v_avg)

        # 기어 변속(Cusp) 구간은 시간 갭이 발생하므로 예외 처리 (속도가 0인 전환점)
        if abs(v[i]) < 1e-3 or abs(v[i-1]) < 1e-3:
            continue

        time_err = abs(dt - expected_dt)
        if time_err > TIME_TOLERANCE_MS / 1000.0:
            time_violations_count += 1
        max_time_err_sec = max(max_time_err_sec, time_err)

    # 6. 저크(Jerk) 검증 및 7. 조향 각속도(Steering Velocity) 검증
    jerk_violations_count = 0
    max_jerk_val = 0.0
    steer_vel_violations_count = 0
    max_steer_vel_val = 0.0

    # 조향각 계산 (phi = arctan(L * kappa))
    phis = np.arctan(WHEELBASE * kappa)

    for i in range(1, n):
        dt = t[i] - t[i-1]
        if dt <= 1e-6:
            continue

        # 시작/정지/Cusp 근처 기어 변속 직후처럼 속도가 극도로 느려지는 순간은 급가속/급회전 수치가 튀기 때문에 필터링
        if abs(v[i]) < 0.1 or abs(v[i-1]) < 0.1:
            continue

        # 수치 연산 시 dt가 너무 작아 저크/조향 각속도가 비정상적으로 튀는 것을 방지하는 분모 가드 (C++ PostProcessor 스펙과 일치)
        acc_dt = max(0.01, dt)

        # Jerk 계산 (da/dt)
        jerk_val = abs(a[i] - a[i-1]) / acc_dt
        if jerk_val > MAX_JERK + 0.01:
            jerk_violations_count += 1
            if jerk_violations_count <= 5:
                rospy.logwarn(f"[Debug Jerk] i={i}, dt={dt:.6f}s, ds={math.hypot(x[i]-x[i-1], y[i]-y[i-1]):.6f}m, v={v[i]:.4f}m/s, a_diff={abs(a[i]-a[i-1]):.4f}m/s^2, jerk={jerk_val:.4f}")
        max_jerk_val = max(max_jerk_val, jerk_val)

        # Steering Velocity 계산 (dphi/dt)
        steer_vel = abs(phis[i] - phis[i-1]) / acc_dt
        if steer_vel > MAX_STEER_VEL + 0.01:
            steer_vel_violations_count += 1
            if steer_vel_violations_count <= 5:
                rospy.logwarn(f"[Debug Steer] i={i}, dt={dt:.6f}s, ds={math.hypot(x[i]-x[i-1], y[i]-y[i-1]):.6f}m, phi_diff={abs(phis[i]-phis[i-1]):.4f}rad, steer_vel={steer_vel:.4f}")
        max_steer_vel_val = max(max_steer_vel_val, steer_vel)

    # 총 경로 길이 계산 (x, y 유클리드 거리 누적)
    total_length = 0.0
    for i in range(1, n):
        total_length += math.hypot(x[i] - x[i-1], y[i] - y[i-1])

    # 보고서 출력
    print("=" * 60)
    print("             TRAJECTORY VALIDATION REPORT")
    print("=" * 60)
    print(f"Total Trajectory Points : {n}")
    print(f"Total Path Length       : {total_length:.2f} m (Back.s: {msg.points[-1].time_from_start.to_sec():.2f}s)")

    # 속도 출력
    print(f"\n[1] Velocity (Limit: {MAX_VEL} m/s)")
    print(f"  - Max Velocity        : {max_v:.4f} m/s")
    print(f"  - Violations Count    : {vel_violations} / {n}")
    print("  - Status              : " + ("OK" if vel_violations == 0 else "FAIL ❌"))

    # 가속도 출력
    print(f"\n[2] Acceleration (Limit: -{MAX_DECEL} ~ +{MAX_ACCEL} m/s^2)")
    print(f"  - Max Acceleration    : {max_a:.4f} m/s^2")
    print(f"  - Min Acceleration    : {min_a:.4f} m/s^2")
    print(f"  - Violations Count    : {acc_violations} / {n}")
    print("  - Status              : " + ("OK" if acc_violations == 0 else "FAIL ❌"))

    # 곡률 출력
    print(f"\n[3] Curvature (Limit: {MAX_KAPPA:.4f} rad/m, Turn Radius: {MIN_TURNING_RADIUS} m)")
    print(f"  - Max Curvature       : {max_k:.4f} rad/m (Equiv. Min Radius: {(1.0/max_k if max_k > 1e-3 else float('inf')):.2f} m)")
    print(f"  - Violations Count    : {kappa_violations} / {n}")
    print("  - Status              : " + ("OK" if kappa_violations == 0 else "FAIL ❌"))

    # 방향각 출력
    print(f"\n[4] Yaw Alignment (Tolerance: {YAW_TOLERANCE_DEG} deg)")
    print(f"  - Max Yaw Error       : {max_yaw_err_deg:.4f} deg")
    print(f"  - Violations Count    : {yaw_violations_count} / {checked_yaw_count}")
    print("  - Status              : " + ("OK" if yaw_violations_count == 0 else "FAIL ❌"))

    # 시간 일관성 출력
    print(f"\n[5] Timestamp Consistency (Tolerance: {TIME_TOLERANCE_MS:.2f} ms)")
    print(f"  - Max Time Mismatch   : {max_time_err_sec * 1000.0:.2f} ms")
    print(f"  - Violations Count    : {time_violations_count} / {n-1}")
    print("  - Status              : " + ("OK" if time_violations_count == 0 else "FAIL ❌"))

    # 저크 출력 (신규)
    print(f"\n[6] Jerk (Limit: {MAX_JERK} m/s^3)")
    print(f"  - Max Jerk            : {max_jerk_val:.4f} m/s^3")
    print(f"  - Violations Count    : {jerk_violations_count} / {n-1}")
    print("  - Status              : " + ("OK" if jerk_violations_count == 0 else "FAIL ❌"))

    # 조향 각속도 출력 (신규)
    print(f"\n[7] Steering Velocity (Limit: {math.degrees(MAX_STEER_VEL):.2f} deg/s, {MAX_STEER_VEL:.4f} rad/s)")
    print(f"  - Max Steering Vel    : {math.degrees(max_steer_vel_val):.2f} deg/s ({max_steer_vel_val:.4f} rad/s)")
    print(f"  - Violations Count    : {steer_vel_violations_count} / {n-1}")
    print("  - Status              : " + ("OK" if steer_vel_violations_count == 0 else "FAIL ❌"))
    print("=" * 60)
def listener():
    rospy.init_node('trajectory_validator', anonymous=True)
    rospy.Subscriber("/planning/trajectory", TrajectoryPath, callback)
    rospy.loginfo("Trajectory Validator Node started. Subscribed to /planning/trajectory.")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
