import os
import multiprocessing
import concurrent.futures
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_file_loader import EventFileLoader
from tensorboard.util import tensor_util

# ---------------------------------------------------------
# 1. 단일 파일에서 '모든 지표' 추출 함수
# ---------------------------------------------------------
def extract_all_tags_from_file(args):
    file_path, log_dir = args
    run_name = os.path.relpath(os.path.dirname(file_path), log_dir)
    if run_name == '.': run_name = 'Default Run'

    # tag별로 데이터를 담을 딕셔너리: { 'loss': {'steps':[], 'values':[]}, ... }
    extracted_data = {}
    file_size_mb = os.path.getsize(file_path) / (1024 * 1024)
    print(f"[시작] ⏳ {run_name} 스캔 중... (용량: {file_size_mb:.1f} MB)")

    try:
        for event in EventFileLoader(file_path).Load():
            if event.HasField('summary'):
                for value in event.summary.value:
                    tag = value.tag
                    val = None

                    # 스칼라(숫자) 값만 추출 (이미지, 오디오 등 무거운 데이터는 패스)
                    if value.HasField('simple_value'):
                        val = value.simple_value
                    elif value.HasField('tensor'):
                        try:
                            arr = tensor_util.make_ndarray(value.tensor)
                            if arr.size == 1:  # 스칼라인 경우만
                                val = arr.item()
                        except:
                            pass

                    if val is not None:
                        if tag not in extracted_data:
                            extracted_data[tag] = {'steps': [], 'values': []}
                        extracted_data[tag]['steps'].append(event.step)
                        extracted_data[tag]['values'].append(val)

        print(f"[완료] ✅ {run_name} (발견된 지표 수: {len(extracted_data)}개)")
        return run_name, extracted_data
    except Exception as e:
        print(f"[오류] ❌ 파일 읽기 실패 ({file_path}): {e}")
        return run_name, {}

# ---------------------------------------------------------
# 2. 메인 시각화 및 자동 분류 함수
# ---------------------------------------------------------
def plot_all_tensorboard_tags(log_dir, output_dir):
    event_files = []
    for root, dirs, files in os.walk(log_dir):
        for file in files:
            if 'events.out.tfevents' in file:
                event_files.append((os.path.join(root, file), log_dir))

    if not event_files:
        print(f"경고: '{log_dir}' 경로에서 텐서보드 파일을 찾을 수 없습니다.")
        return

    num_cores = multiprocessing.cpu_count()
    print(f"🚀 총 {len(event_files)}개의 파일을 {num_cores}개의 CPU 코어로 동시 분석합니다...\n")

    # 모든 데이터를 하나로 모을 거대한 딕셔너리
    # 구조: { 'tag_name': { 'run_name': {'steps': [], 'values': []} } }
    all_tags_data = {}

    with concurrent.futures.ProcessPoolExecutor(max_workers=num_cores) as executor:
        results = executor.map(extract_all_tags_from_file, event_files)

        for run_name, extracted_data in results:
            for tag, data in extracted_data.items():
                if tag not in all_tags_data:
                    all_tags_data[tag] = {}
                if run_name not in all_tags_data[tag]:
                    all_tags_data[tag][run_name] = {'steps': [], 'values': []}

                all_tags_data[tag][run_name]['steps'].extend(data['steps'])
                all_tags_data[tag][run_name]['values'].extend(data['values'])

    if not all_tags_data:
        print("\n경고: 추출할 수 있는 숫자형(Scalar) 지표가 없습니다.")
        return

    print(f"\n📊 총 {len(all_tags_data)}개의 고유 지표를 발견했습니다. 그래프 생성을 시작합니다...")

    # 저장 경로 설정
    clean_output_dir = output_dir.strip()
    if clean_output_dir:
        os.makedirs(clean_output_dir, exist_ok=True)

    saved_files = []

    # 3. 발견된 '모든 태그'에 대해 각각 그래프 그리기
    for tag_name, runs in all_tags_data.items():
        plt.figure(figsize=(10, 6))

        has_valid_data = False
        for run_name, data in runs.items():
            if data['steps'] and data['values']:
                # Step 기준으로 정렬
                sorted_pairs = sorted(zip(data['steps'], data['values']))
                sorted_steps = [p[0] for p in sorted_pairs]
                sorted_values = [p[1] for p in sorted_pairs]

                plt.plot(sorted_steps, sorted_values, label=run_name, linewidth=1.5)
                has_valid_data = True

        if not has_valid_data:
            plt.close()
            continue

        plt.title(f'Metric: {tag_name}', fontsize=16, fontweight='bold')
        plt.xlabel('Steps', fontsize=12)
        plt.ylabel('Value', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.6)

        # 범례가 많을 경우 그래프 밖으로 빼기 (옵션)
        # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=10)
        plt.legend(loc='best', fontsize=10)

        # 파일명 안전하게 변환 (슬래시 포함 방지)
        safe_tag_name = tag_name.replace('/', '_').replace('\\', '_')
        file_name = f'plot_{safe_tag_name}.png'
        output_filepath = os.path.join(clean_output_dir, file_name) if clean_output_dir else file_name

        plt.savefig(output_filepath, dpi=300, bbox_inches='tight')
        plt.close()
        saved_files.append(output_filepath)
        print(f"  - 저장 완료: {output_filepath}")

    print(f"\n🎉 모든 작업이 끝났습니다! 총 {len(saved_files)}개의 그래프가 저장되었습니다.")

# ---------------------------------------------------------
# 실행 설정
# ---------------------------------------------------------
if __name__ == "__main__":
    LOG_DIRECTORY = "../result_csv"
    OUTPUT_DIRECTORY = "../result_csv"

    plot_all_tensorboard_tags(LOG_DIRECTORY, OUTPUT_DIRECTORY)