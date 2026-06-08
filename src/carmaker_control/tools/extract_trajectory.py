import rosbag
import csv
import argparse
import os

def extract_trajectory(bag_file_path, topic_name, output_csv):
    # 입력받은 파일이 실제로 존재하는지 확인
    if not os.path.isfile(bag_file_path):
        print(f"Error: The file '{bag_file_path}' does not exist.")
        return

    print(f"Reading from: {bag_file_path}")
    print(f"Target topic: {topic_name}")
    
    # 데이터 추출 및 CSV 저장
    try:
        with rosbag.Bag(bag_file_path, 'r') as bag, open(output_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            # CSV 헤더 작성
            writer.writerow(['timestamp', 'x', 'y'])

            count = 0
            # bag 파일에서 토픽 읽기
            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                try:
                    # 메시지 구조에 맞게 변수명 수정 (Car_x, Car_y)
                    x = msg.Car_x  
                    y = msg.Car_y  
                    timestamp = t.to_sec()
                    writer.writerow([timestamp, x, y])
                    count += 1
                except AttributeError:
                    print(f"Error: Topic '{topic_name}' does not contain 'Car_x' or 'Car_y'.")
                    print("Please check the message structure using 'rosmsg show'.")
                    return

        print(f"Successfully extracted {count} points to '{output_csv}'")

    except Exception as e:
        print(f"An error occurred while processing the bag file: {e}")


if __name__ == "__main__":
    # CLI 인자 파서 설정
    parser = argparse.ArgumentParser(description="Extract X, Y trajectory data from a rosbag file.")
    
    # 필수 인자: rosbag 파일 경로
    parser.add_argument("bag_file", type=str, help="Path to the input .bag file")
    
    # 선택 인자: 토픽 이름 (기본값 설정)
    parser.add_argument("-t", "--topic", type=str, default="/carmaker/dynamic_info",
                        help="Topic name containing trajectory data (default: /carmaker/dynamic_info)")
    
    # 선택 인자: 출력 CSV 파일 이름 (기본값 설정)
    parser.add_argument("-o", "--output", type=str, default="trajectory_data.csv",
                        help="Output CSV file name (default: trajectory_data.csv)")

    args = parser.parse_args()

    # 함수 실행
    extract_trajectory(args.bag_file, args.topic, args.output)