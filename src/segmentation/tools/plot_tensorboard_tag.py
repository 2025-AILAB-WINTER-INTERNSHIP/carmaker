import concurrent.futures
import multiprocessing
import os

import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_file_loader import EventFileLoader
from tensorboard.util import tensor_util


def extract_data_from_file(args):
    """
    개별 프로세스(코어)가 할당받아 실행할 작업 함수입니다.
    하나의 이벤트 파일을 처음부터 끝까지 스캔하여 필요한 태그의 데이터만 추출합니다.
    """
    file_path, log_dir, target_tag = args

    # 해당 파일이 속한 폴더명(실험명) 추출
    run_name = os.path.relpath(os.path.dirname(file_path), log_dir)
    if run_name == ".":
        run_name = "Default Run"

    steps = []
    values = []
    file_size_mb = os.path.getsize(file_path) / (1024 * 1024)

    print(f"[시작] ⏳ {run_name} 읽는 중... (용량: {file_size_mb:.1f} MB)")

    try:
        # 파일 스트리밍 읽기
        for event in EventFileLoader(file_path).Load():
            if event.HasField("summary"):
                for value in event.summary.value:
                    if value.tag == target_tag:
                        if value.HasField("simple_value"):
                            steps.append(event.step)
                            values.append(value.simple_value)
                        elif value.HasField("tensor"):
                            val = tensor_util.make_ndarray(value.tensor).item()
                            steps.append(event.step)
                            values.append(val)

        print(f"[완료] ✅ {run_name} (추출된 데이터: {len(steps)}개)")
        return run_name, steps, values

    except Exception as e:
        print(f"[오류] ❌ 파일 읽기 실패 ({file_path}): {e}")
        return run_name, [], []


def plot_tensorboard_multicore(
    log_dir, target_tag, output_filename="multicore_plot.png"
):
    # 1. 텐서보드 파일 목록 수집
    event_files = []
    for root, dirs, files in os.walk(log_dir):
        for file in files:
            if "events.out.tfevents" in file:
                event_files.append((os.path.join(root, file), log_dir, target_tag))

    if not event_files:
        print(f"경고: '{log_dir}' 경로에서 텐서보드 파일을 찾을 수 없습니다.")
        return

    # 2. 멀티프로세싱으로 여러 파일 동시 분석
    num_cores = multiprocessing.cpu_count()
    print(
        f"🚀 총 {len(event_files)}개의 파일을 {num_cores}개의 CPU 코어로 동시 분석합니다...\n"
    )

    # 결과를 담을 딕셔너리 (동일한 Run에 여러 파일이 있을 수 있으므로 합치기 위함)
    run_data = {}

    with concurrent.futures.ProcessPoolExecutor(max_workers=num_cores) as executor:
        # 각 코어에 작업(추출 함수) 분배 및 결과 수집
        results = executor.map(extract_data_from_file, event_files)

        for run_name, steps, values in results:
            if steps and values:
                if run_name not in run_data:
                    run_data[run_name] = {"steps": [], "values": []}
                run_data[run_name]["steps"].extend(steps)
                run_data[run_name]["values"].extend(values)

    if not run_data:
        print(f"\n경고: '{target_tag}' 태그를 가진 데이터가 없습니다.")
        return

    # 3. 수집된 데이터를 바탕으로 그래프 그리기
    plt.figure(figsize=(10, 6))

    for run_name, data in run_data.items():
        # 데이터가 순서대로 섞여 있을 수 있으므로 Step 기준으로 정렬
        sorted_pairs = sorted(zip(data["steps"], data["values"]))
        sorted_steps = [pair[0] for pair in sorted_pairs]
        sorted_values = [pair[1] for pair in sorted_pairs]

        plt.plot(sorted_steps, sorted_values, label=run_name, linewidth=1.5)

    plt.title(
        f"Comparison of {target_tag} (Multi-core)", fontsize=16, fontweight="bold"
    )
    plt.xlabel("Steps", fontsize=12)
    plt.ylabel(target_tag, fontsize=12)
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="center left", bbox_to_anchor=(1, 0.5), fontsize=10)

    plt.savefig(output_filename, dpi=300, bbox_inches="tight")
    plt.close()
    print(
        f"\n🎉 모든 분석이 끝났습니다! '{output_filename}'에 그래프가 저장되었습니다."
    )


# ==========================================
# 실행 설정 (Windows/Mac/Linux 공통 필수 조건)
# ==========================================
if __name__ == "__main__":
    LOG_DIRECTORY = "../runs"
    TARGET_TAG = "test/all/dice"
    OUTPUT_FILE = "test_dice.png"

    plot_tensorboard_multicore(LOG_DIRECTORY, TARGET_TAG, OUTPUT_FILE)
