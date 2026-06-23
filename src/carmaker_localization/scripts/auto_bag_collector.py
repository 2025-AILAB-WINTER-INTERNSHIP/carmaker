#!/usr/bin/env python3
import socket
import time
import subprocess
import os
import math

class CarMakerTclClient:
    """CarMaker Tcl TCP API (Port 16607) Control Wrapper"""
    def __init__(self, host="127.0.0.1", port=16607):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            # 무한 대기 프리징 방지를 위해 소켓 타임아웃 설정
            self.sock.settimeout(3.0)
            # CarMaker Tcl 서버는 연결 시 웰컴 메시지를 보내지 않는 경우가 많으므로 blocking recv 제거
            # self.sock.recv(4096)
            print("[+] Connected to CarMaker TCP API")
        except Exception as e:
            print(f"[-] Connection failed: {e}")
            raise

    def send_cmd(self, cmd):
        self.sock.sendall(f"{cmd}\n".encode('utf-8'))
        res = self.sock.recv(4096).decode('utf-8')
        return res.strip()

    def close(self):
        if self.sock:
            self.sock.close()


def generate_grid_points():
    """
    static_evaluator와 완전히 동일하게 그리드를 생성하기 위해 
    기존에 계산 완료된 grid_registration_results.csv의 주행 가능 좌표들을 파싱하여 로드합니다.
    이후 충전패드 기준 [-15, 15] 범위 필터 및 ROI (중심부 1m, 외곽부 2m step)를 적용합니다.
    """
    center_x, center_y = 0.0, -4.33
    
    # 도커 컨테이너 내부 경로 fallback 설정
    csv_paths = [
        "/workspace/src/carmaker_localization/grid_registration_results.csv",
        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "grid_registration_results.csv")
    ]
    
    csv_path = None
    for path in csv_paths:
        if os.path.exists(path):
            csv_path = path
            break
            
    if not csv_path:
        # 파일이 없을 시 기존의 수학적 백업 생성 로직 작동 (Fallback)
        print("[-] Warning: grid_registration_results.csv not found. Falling back to mathematical grid.")
        points = []
        for dx in range(-10, 11):
            for dy in range(-10, 11):
                points.append((center_x + float(dx), center_y + float(dy)))
        return points

    import csv
    points = []
    print(f"[+] Loading baseline grid from: {csv_path}")
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                grid_i = int(row['grid_i'])
                grid_j = int(row['grid_j'])
                cell_x = float(row['cell_x'])
                cell_y = float(row['cell_y'])
            except (KeyError, ValueError):
                continue
                
            dx = cell_x - center_x
            dy = cell_y - center_y
            
            # 1. [-10, 10]m 외부 구역 필터링
            if abs(dx) > 10.0 or abs(dy) > 10.0:
                continue
                
            # 모든 1m 주행 가능 그리드 수집
            points.append((cell_x, cell_y))
                    
    return points


def main():
    # 1. TCP Client 연결
    cm = CarMakerTclClient()
    try:
        cm.connect()
    except Exception:
        print("[-] Please ensure CarMaker GUI is open and Tcl port is active.")
        return

    # 2. 그리드 포인트 생성
    grid_points = generate_grid_points()
    yaws = [0.0, 90.0, 180.0, 270.0] # [deg]

    # 충전패드 기준 좌표
    center_x, center_y = 0.0, -4.33

    # 총 수집 횟수 및 시간 예측
    total_runs = len(grid_points) * len(yaws)
    run_duration = 3.0
    total_duration = 0.0
    for gx, gy in grid_points:
        total_duration += (run_duration + 2.0) * len(yaws)

    print(f"[+] Total grid points: {len(grid_points)}")
    print(f"[+] Total simulation runs: {total_runs}")
    print(f"[+] Expected time: ~{(total_duration / 3600.0):.2f} hours")

    output_dir = "/workspace/src/carmaker_localization/data/bags"
    os.makedirs(output_dir, exist_ok=True)

    # 테스트런 로드
    testrun_name = "test_raw"
    cm.send_cmd(f"LoadTestRun {testrun_name}")
    time.sleep(2.0) # 테스트런 초기 파일 로드를 위해 2초 대기

    run_idx = 0
    for i, (gx, gy) in enumerate(grid_points):
        for yaw in yaws:
            run_idx += 1
            yaw_rad = yaw * math.pi / 180.0

            # 충전패드 기준 오프셋 (dx, dy) 계산
            dx = gx - center_x
            dy = gy - center_y
            duration = 3.0

            # 파일명에 정수 오프셋 dx, dy를 직접 사용하여 중복 방지
            bag_name = f"grid_dx_{int(round(dx))}_dy_{int(round(dy))}_yaw_{int(yaw)}deg.bag"
            bag_path = os.path.join(output_dir, bag_name)

            # 이미 수집된 정상적인 크기(10KB 이상)의 bag 파일이 존재할 경우 스킵 (이어받기)
            if os.path.exists(bag_path) and os.path.getsize(bag_path) > 10240:
                print(f"[{run_idx}/{total_runs}] Skip: {bag_name} (Already collected)")
                continue

            print(f"\n[{run_idx}/{total_runs}] Teleporting to: ({gx:.1f}, {gy:.1f}) | Yaw: {yaw} deg | Duration: {duration}s")

            # A. CarMaker 차량 초기 위치 설정 (실제 후륜축이 gx, gy에 오도록 뒷범퍼 스폰 위치 sx, sy 역보상)
            rear_axle_offset = 0.82
            sx = gx - rear_axle_offset * math.cos(yaw_rad)
            sy = gy - rear_axle_offset * math.sin(yaw_rad)

            r1 = cm.send_cmd(f'IFileModify TestRun "Vehicle.StartPos" "{sx} {sy} 0"')
            r2 = cm.send_cmd(f'IFileModify TestRun "Vehicle.StartPos.Orientation" "{yaw}"') # degree
            r3 = cm.send_cmd("IFileFlush")
            time.sleep(0.5)  # 디스크 쓰기 시간 보장
            r4 = cm.send_cmd(f"LoadTestRun {testrun_name}")
            print(f"  └─ Tcl Modify Responses: StartPos={r1}, Orientation={r2}, Flush={r3}, Reload={r4}")
            
            # B. 백그라운드로 rosbag 녹화 먼저 실행 (시뮬레이션 시작 순간의 최초 토픽 및 EKF 초기화 로그 누락 방지)
            rosbag_cmd = [
                "rosbag", "record",
                "--lz4",
                "--buffsize", "1024",
                "-O", bag_path,
                "-e", "^/(control|planning|localization|carmaker|diagnostics|tf|parking).*"
            ]
            print(f"  └─ Recording to {bag_name} for {duration}s (sim time)...")
            rosbag_proc = subprocess.Popen(rosbag_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # rosbag record 노드가 완전히 활성화되고 구독을 시작할 수 있도록 1.0초 대기
            time.sleep(1.0)
            
            # C. 시뮬레이션 시작 (Tcl 표준 기동 명령어인 StartSim 사용)
            res = cm.send_cmd("StartSim")
            print(f"  └─ Start Response: {res}")
            
            # CarMaker가 'Running' 상태(1~3)가 될 때까지 폴링(Polling) 대기
            for _ in range(10):
                sim_status = cm.send_cmd('expr {$SimStatus}')
                if sim_status.strip().startswith('E') or not sim_status.strip():
                    sim_status = cm.send_cmd('DDictGet "SimCore.State"')
                # 수치형 혹은 문자열 상태 값에 대응
                if sim_status.strip() in ['1', '2', '3', 'SCState_Simulate', 'SCState_Preflight']:
                    break
                time.sleep(0.5)
            
            # 첫 번째 실행에서만 물리 엔진 및 센서 로딩 오버헤드가 발생하므로 5초를 대기하고,
            # 두 번째 실행부터는 1.0초를 대기합니다.
            if run_idx == 1:
                print("  └─ First run sensor initialization... waiting 5.0s")
                time.sleep(5.0)
            else:
                time.sleep(1.0)

            # 실제 시뮬레이션 시간 기준(DDictGet "Time") 대기
            try:
                start_time_str = cm.send_cmd('DDictGet "Time"')
                if start_time_str.strip().startswith('E') or not start_time_str.strip():
                    start_time_str = cm.send_cmd('expr {$Qu(Time)}')
                start_sim_time = float(start_time_str.strip())
            except (ValueError, Exception):
                start_sim_time = 0.0
                
            elapsed_sim_time = 0.0
            timeout_limit = duration * 3.0  # 무한 대기 방지용 현실 시간 아웃마진
            t_start = time.time()
            
            while elapsed_sim_time < duration:
                # 현실 시간 기준 세이프가드 타임아웃
                if time.time() - t_start > timeout_limit:
                    print("  └─ [Warning] Simulation time polling timeout. Proceeding...")
                    break
                
                time.sleep(0.05)  # 과도한 소켓 통신 오버헤드 방지
                
                try:
                    curr_time_str = cm.send_cmd('DDictGet "Time"')
                    if curr_time_str.strip().startswith('E') or not curr_time_str.strip():
                        curr_time_str = cm.send_cmd('expr {$Qu(Time)}')
                    curr_sim_time = float(curr_time_str.strip())
                    elapsed_sim_time = curr_sim_time - start_sim_time
                except (ValueError, Exception):
                    # 오류 발생 시 현실 시간 경과로 대체 처리
                    elapsed_sim_time = time.time() - t_start
            
            # 프로세스 안전 종료 및 자원 회수
            rosbag_proc.terminate()
            rosbag_proc.wait()
            
            # D. 시뮬레이션 정지 및 리셋 대기
            stop_res = cm.send_cmd("StopSim") # Tcl 표준 종료 명령어인 StopSim 사용
            print(f"  └─ Stop Response: {stop_res}")
            
            # 시뮬레이터가 완전히 'Idle(0)' 상태로 돌아올 때까지 대기
            for _ in range(10):
                sim_status = cm.send_cmd('expr {$SimStatus}')
                if sim_status.strip().startswith('E') or not sim_status.strip():
                    sim_status = cm.send_cmd('DDictGet "SimCore.State"')
                if sim_status.strip() in ['0', 'SCState_Idle']:
                    break
                time.sleep(0.5)
            print("  └─ Simulator is Idle and ready for next run.")

    cm.close()
    print("[+] Automation finished successfully!")


if __name__ == "__main__":
    main()
