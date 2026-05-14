import glob
import os
import re

import matplotlib.pyplot as plt
import pandas as pd


def plot_all_csvs(csv_dir, output_dir):
    # 1. 지정한 폴더 및 하위 폴더에서 조건에 맞는 CSV 파일 모두 찾기
    search_pattern = os.path.join(csv_dir, "**", "run-*.csv")
    csv_files = glob.glob(search_pattern, recursive=True)

    if not csv_files:
        # 하위 폴더가 아닌 바로 해당 폴더도 검색
        csv_files = glob.glob(os.path.join(csv_dir, "run-*.csv"))

    if not csv_files:
        print(f"경고: '{csv_dir}' 경로에서 'run-*.csv' 형식의 파일을 찾을 수 없습니다.")
        return

    print(f"📂 총 {len(csv_files)}개의 CSV 파일을 찾았습니다. 분석을 시작합니다...\n")

    tags_data = {}

    # 2. 파일명 분석 및 데이터 로드 (Tag 자동 추출)
    for file in csv_files:
        filename = os.path.basename(file)

        # 파일명에서 '실험명'과 '지표명(Tag)' 추출
        match = re.search(r"run-(.*)-tag-(.*)\.csv", filename)

        if match:
            run_name = match.group(1)
            tag_name = match.group(2)

            try:
                df = pd.read_csv(file)
                if tag_name not in tags_data:
                    tags_data[tag_name] = {}
                tags_data[tag_name][run_name] = df
            except Exception as e:
                print(f"❌ 파일 읽기 실패 ({file}): {e}")

    if not tags_data:
        print("데이터를 추출할 수 없습니다. 파일명 형식을 확인해주세요.")
        return

    print(
        f"📊 총 {len(tags_data)}개의 고유 지표(Tag)를 발견했습니다. 그래프 생성을 시작합니다..."
    )

    # 3. 저장 폴더 설정
    clean_output_dir = output_dir.strip()
    if clean_output_dir:
        os.makedirs(clean_output_dir, exist_ok=True)

    saved_files = []

    # 4. 각 지표(Tag)별로 그래프 그리기
    for tag_name, runs in tags_data.items():
        plt.figure(figsize=(10, 6))

        has_valid_data = False
        for run_name, df in runs.items():
            if "Step" in df.columns and "Value" in df.columns:
                df = df.sort_values(by="Step")
                plt.plot(
                    df["Step"],
                    df["Value"],
                    label=run_name,
                    linewidth=2,
                    marker="o",
                    markersize=4,
                )
                has_valid_data = True

        if not has_valid_data:
            plt.close()
            continue

        display_title = tag_name.replace("_", "/") if "_" in tag_name else tag_name

        plt.title(f"{display_title}", fontsize=16)
        plt.xlabel("Step", fontsize=12)
        plt.ylabel("Value", fontsize=12)
        plt.grid(True, linestyle="--", alpha=0.6)
        plt.legend(loc="best", fontsize=10)

        file_name = f"plot_{tag_name}.png"
        output_filepath = (
            os.path.join(clean_output_dir, file_name) if clean_output_dir else file_name
        )

        plt.savefig(output_filepath, dpi=300, bbox_inches="tight")
        plt.close()

        saved_files.append(output_filepath)
        print(f"  - 저장 완료: {output_filepath} (포함된 실험 수: {len(runs)}개)")

    print(
        f"\n🎉 모든 작업이 끝났습니다! 총 {len(saved_files)}개의 그래프가 저장되었습니다."
    )


# ==========================================
# 실행 설정
# ==========================================
if __name__ == "__main__":
    # 방금 터미널에 입력하셨던 경로로 맞춤 설정해두었습니다.
    CSV_DIRECTORY = "../result_csv"

    # 결과를 저장할 폴더 (공백 " "으로 두면 이 스크립트를 실행한 현재 폴더에 이미지들이 저장됩니다)
    OUTPUT_DIRECTORY = "../result_csv"

    plot_all_csvs(CSV_DIRECTORY, OUTPUT_DIRECTORY)
