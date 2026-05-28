#include "carmaker_localization/feature_extractor.h"
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace carmaker_localization {

FeatureExtractor::FeatureExtractor(const std::string& camera_name)
    : r_max_(15.0), cov_k_(1.0), max_fov_(180.0), is_initialized_(false), lut_initialized_(false),
        has_optimal_point_(false), has_vehicle_footprint_(false),
        veh_x_min_(0.0), veh_x_max_(0.0), veh_y_half_(0.0),
        phys_x_min_(0.0), phys_x_max_(0.0), phys_y_half_(0.0),
        veh_height_(1.605), cam_origin_x_(0.0), cam_origin_y_(0.0),
        camera_name_(camera_name) {
}

void FeatureExtractor::initialize(const std::vector<double>& x_range, const std::vector<double>& y_range,
                                    const std::string& image_type, double resolution) {
    image_type_ = image_type;

    if (x_range.size() >= 2) {
        bev_cfg_.x_min = x_range[0];
        bev_cfg_.x_max = x_range[1];
    } else {
        bev_cfg_.x_min = -10.0; bev_cfg_.x_max = 10.0;
    }

    if (y_range.size() >= 2) {
        bev_cfg_.y_min = y_range[0];
        bev_cfg_.y_max = y_range[1];
    } else {
        bev_cfg_.y_min = -5.0; bev_cfg_.y_max = 15.0;
    }

    bev_cfg_.resolution = resolution;
    bev_cfg_.width = std::ceil((bev_cfg_.x_max - bev_cfg_.x_min) / bev_cfg_.resolution);
    bev_cfg_.height = std::ceil((bev_cfg_.y_max - bev_cfg_.y_min) / bev_cfg_.resolution);

    is_initialized_ = true;
}

void FeatureExtractor::setExtractionParameters(double r_max, double cov_k, double max_fov) {
    r_max_ = r_max;
    cov_k_ = cov_k;
    max_fov_ = max_fov;
}

void FeatureExtractor::setVehicleFootprint(double x_min, double x_max, double y_half,
                                            double height, double margin) {
    // Fr1A 좌표계 기준 차량 점유 영역 설정 + margin 적용
    // GT 이미지에서 차체 외곽선(검은 테두리)은 bbox 경계에 그려지므로,
    // 테두리 픽셀이 BEV 해상도 기준으로 footprint 경계 바깥에 1~2픽셀씩 걸칠 수 있음.
    // → margin(기본 0.15m)으로 마스크를 확장하여 앞/뒤/좌/우 외곽선 오검출 방지.
    veh_x_min_ = x_min - margin;
    veh_x_max_ = x_max + margin;
    veh_y_half_ = y_half + margin;
    veh_height_ = height;

    // 3D Ray-Casting용 물리적 uninflated 규격 저장
    phys_x_min_ = x_min;
    phys_x_max_ = x_max;
    phys_y_half_ = y_half;

    has_vehicle_footprint_ = true;
}

void FeatureExtractor::updateLUT(const std::vector<double>& K_vec, const std::vector<double>& D_vec,
                                    const std::string& distortion_model, const cv::Mat& R_base_cam, const cv::Mat& t_base_cam) {
    if (!is_initialized_) return;

    std::cout << "\n\033[1;33m========== [ DEBUG: " << camera_name_ << " ] ==========\033[0m\n";

    // 1. Intrinsic (초점거리) 검증
    std::cout << "[1. Intrinsic Check]\n";
    std::cout << "  - fx: " << K_vec[0] << " (기대값: 약 236.09)\n";
    std::cout << "  - fy: " << K_vec[4] << " (기대값: 약 236.09)\n";

    // 역행렬 사전 계산 (Base 기준 카메라 위치 및 방향 도출용)
    cv::Mat R_cam_base_test = R_base_cam.t();
    cv::Mat t_cam_base_test = -R_cam_base_test * t_base_cam;

    // 2. Extrinsic Translation (Z축 높이) 검증
    std::cout << "[2. Translation Check (카메라의 Base 기준 물리적 위치)]\n";
    std::cout << "  - X: " << t_cam_base_test.at<double>(0, 0) << " m\n";
    std::cout << "  - Y: " << t_cam_base_test.at<double>(1, 0) << " m\n";
    std::cout << "  - Z: " << t_cam_base_test.at<double>(2, 0) << " m\n";

    // 3. Optical Frame 및 Rotation 검증 (광학 축 방향 테스트)
    cv::Mat z_axis_optical = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
    cv::Mat z_axis_in_base = R_cam_base_test * z_axis_optical;
    std::cout << "[3. Optical Axis Check (카메라 렌즈가 바라보는 방향 벡터)]\n";
    std::cout << "  - Base X방향 (전/후): " << z_axis_in_base.at<double>(0, 0) << "\n";
    std::cout << "  - Base Y방향 (좌/우): " << z_axis_in_base.at<double>(1, 0) << "\n";
    std::cout << "  - Base Z방향 (상/하): " << z_axis_in_base.at<double>(2, 0) << "\n";

    // 4. Raycasting Unit Test (특정 지면 포인트를 카메라로 투영)
    cv::Mat test_point_base = (cv::Mat_<double>(3, 1) << 0.0, -3.0, 0.0); // 우측 3m 바닥
    cv::Mat test_point_cam = R_base_cam * test_point_base + t_base_cam;
    std::cout << "[4. Raycasting Unit Test (우측 3m 지면 좌표 -> 카메라 좌표)]\n";
    std::cout << "  - Camera Opt X (Right): " << test_point_cam.at<double>(0, 0) << "\n";
    std::cout << "  - Camera Opt Y (Down) : " << test_point_cam.at<double>(1, 0) << "\n";
    std::cout << "  - Camera Opt Z (Depth): " << test_point_cam.at<double>(2, 0) << "\n";

    std::cout << "========================================================\n\n";

    std::vector<cv::Point3f> object_points;
    cartesian_lut_x_ = cv::Mat(bev_cfg_.height, bev_cfg_.width, CV_32F);
    cartesian_lut_y_ = cv::Mat(bev_cfg_.height, bev_cfg_.width, CV_32F);

    // [Step 1: 3D Grid Generation] Base 좌표계(차량 중점) 기준 Z=0인 바닥 평면 격자점 생성
    for (int v = 0; v < bev_cfg_.height; ++v) {
        for (int u = 0; u < bev_cfg_.width; ++u) {
            double x = bev_cfg_.x_min + (u + 0.5) * bev_cfg_.resolution;
            double y = bev_cfg_.y_max - (v + 0.5) * bev_cfg_.resolution;
            object_points.emplace_back(x, y, 0.0);

            // 향후 특징점 추출(Covariance 등)에 사용할 물리적 거리(m) 룩업 테이블 임시 저장
            cartesian_lut_x_.at<float>(v, u) = static_cast<float>(x);
            cartesian_lut_y_.at<float>(v, u) = static_cast<float>(y);
        }
    }

    std::vector<cv::Point3f> cam_points;
    cam_points.reserve(object_points.size());
    for (const auto& pt : object_points) {
        // [Step 2: Extrinsic Transform] Base 좌표계 -> Camera 광학 좌표계로 3D 회전 및 이동
        double cam_x = R_base_cam.at<double>(0,0)*pt.x + R_base_cam.at<double>(0,1)*pt.y + t_base_cam.at<double>(0,0);
        double cam_y = R_base_cam.at<double>(1,0)*pt.x + R_base_cam.at<double>(1,1)*pt.y + t_base_cam.at<double>(1,0);
        double cam_z = R_base_cam.at<double>(2,0)*pt.x + R_base_cam.at<double>(2,1)*pt.y + t_base_cam.at<double>(2,0);
        cam_points.emplace_back(cam_x, cam_y, cam_z);
    }

    // Camera intrinsic matrix 및 왜곡 계수(Distortion coefficients) 세팅
    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) K.at<double>(i / 3, i % 3) = K_vec[i];

    // [왜곡 계수 처리]
    // CarMaker 시뮬레이터: 순수 등거리 투영(r = f·θ) 모델을 사용하지만,
    // ROS topic에는 자체 센서 설정값인 D=[1.0,0,0,0]을 publish함.
    // OpenCV cv::fisheye 모델에서 k₁=1.0은 극단적인 핀쿠션 왜곡을 의미하여
    // BEV 역투영이 완전히 틀어지므로, 순수 equidistant(D=[0,0,0,0])로 오버라이드.
    // 실제 카메라 활용 시: 캘리브레이션된 실제 D 값을 그대로 사용.
    cv::Mat D;
    bool is_carmaker_equidistant =
        (distortion_model == "equidistant" || distortion_model == "fisheye") &&
        D_vec.size() >= 1 && D_vec[0] == 1.0 &&
        std::all_of(D_vec.begin() + 1, D_vec.end(), [](double v){ return v == 0.0; });

    if (is_carmaker_equidistant) {
        // CarMaker placeholder → 순수 equidistant (왜곡 없음)으로 오버라이드
        D = cv::Mat::zeros(4, 1, CV_64F);
    } else if (!D_vec.empty()) {
        D = cv::Mat(D_vec.size(), 1, CV_64F);
        for (size_t i = 0; i < D_vec.size(); ++i) D.at<double>(i, 0) = D_vec[i];
    } else {
        D = cv::Mat::zeros(4, 1, CV_64F);
    }

    // rvec과 tvec이 0인 이유:
    // 위 Step 2에서 tf2::doTransform을 통해 이미 cam_points가 '카메라 기준 좌표계'로 변환되었기 때문에,
    // OpenCV 투영 함수 내부에서 추가적인 Extrinsic 변환(회전/이동)을 할 필요가 없음.
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    std::vector<cv::Point2f> image_points;

    // [Step 3: Forward Projection] 렌즈 왜곡을 반영하여 3D Camera 좌표 -> 2D Image 픽셀 좌표로 투영 (수학적 정방향 연산)
    // projectPoints:
    //  1. 정규화 및 theta 도출: cam_points의 X_c, Y_c, Z_c 값을 이용해 입사각을 계산
    //  2. 왜곡 다항식 대입: 파라미터로 넘겨준 D 행렬(왜곡 계수)을 다항식에 단순 대입하여 왜곡을 적용
    //  3. Intrinsic 매핑: 파라미터로 넘겨받은 카메라 내부 파라미터 행렬 K(f_x, f_y, c_x, c_y)를 곱해 최종 픽셀 위치를 계산
    if (distortion_model == "equidistant" || distortion_model == "fisheye") {
        cv::fisheye::projectPoints(cam_points, image_points, rvec, tvec, K, D);
    } else {
        cv::projectPoints(cam_points, rvec, tvec, K, D, image_points);
    }

    // 카메라 원점 및 광선 방향의 Base 프레임 기준 좌표 사전 계산
    cv::Mat R_cam_base = R_base_cam.t();
    cv::Mat t_cam_base = -R_cam_base * t_base_cam;

    double origin_in_base_x = t_cam_base.at<double>(0, 0);
    double origin_in_base_y = t_cam_base.at<double>(1, 0);
    double origin_in_base_z = t_cam_base.at<double>(2, 0);

    cam_origin_x_ = origin_in_base_x;
    cam_origin_y_ = origin_in_base_y;

    double ray_dir_in_base_x = R_cam_base.at<double>(0, 2);
    double ray_dir_in_base_y = R_cam_base.at<double>(1, 2);
    double ray_dir_in_base_z = R_cam_base.at<double>(2, 2);

    // [Step 4: Backward Mapping LUT Construction]
    // cv::remap을 위한 룩업 테이블 생성: 빈 3D 캔버스(u, v)에 원본 이미지의 어느 픽셀(x, y)을 가져올지 기록
    map1_ = cv::Mat(bev_cfg_.height, bev_cfg_.width, CV_32FC1);
    map2_ = cv::Mat(bev_cfg_.height, bev_cfg_.width, CV_32FC1);
    for (int v = 0; v < bev_cfg_.height; ++v) {
        for (int u = 0; u < bev_cfg_.width; ++u) {
            int idx = v * bev_cfg_.width + u;

            // 물리적 지면 격자 좌표 (Base 프레임 기준)
            double gx = cartesian_lut_x_.at<float>(v, u);
            double gy = cartesian_lut_y_.at<float>(v, u);

            // [Ray-casting Ego Occlusion Check]
            // 지면 격자점과 카메라 원점을 잇는 광선이 자차 3D 바디 박스를 통과(오클루전)하는지 검사
            bool is_blocked = false;
            if (has_vehicle_footprint_) {
                for (int step = 1; step <= 10; ++step) {
                    double t = 0.1 + 0.8 * (step / 10.0); // t in [0.18, 0.90]
                    double rx = origin_in_base_x + t * (gx - origin_in_base_x);
                    double ry = origin_in_base_y + t * (gy - origin_in_base_y);
                    double rz = origin_in_base_z * (1.0 - t);
                    // 자차의 3D 바디 범위(높이 Z <= veh_height_)를 가로막으면 시야가 차단된 것
                    // 3D Ray-Casting 시에는 카메라 마운트가 차체 외곽선 경계에 걸쳐 상시 차단되는 오작동을 피하기 위해
                    // 인플레이션 마진이 없는 순수 물리적 차체 크기(phys_x_min_, phys_x_max_, phys_y_half_)를 기준으로 비교해야 합니다.
                    if (rx >= phys_x_min_ && rx <= phys_x_max_ && std::abs(ry) <= phys_y_half_ && rz <= veh_height_) {
                        is_blocked = true;
                        break;
                    }
                }
            }

            // Exclude points behind the camera optical plane or blocked by vehicle body to prevent mathematical fold-over.
            // 어안렌즈의 물리적 최대 화각 한계선(max_fov_)을 일반 삼각법 공식으로 엄밀하게 필터링합니다.
            double r = cv::norm(cam_points[idx]);
            double cos_theta = cam_points[idx].z / (r + 1e-6);   // zero division 방지
            double cos_fov_max = std::cos((max_fov_ / 2.0) * M_PI / 180.0); // 설정된 최대 화각 한계선 적용

            if (is_blocked || cos_theta < cos_fov_max) {
                // 시야각을 벗어나거나 차체에 가린 곳은 -1로 맵핑하여 렌더링에서 제외
                map1_.at<float>(v, u) = -1.0f;
                map2_.at<float>(v, u) = -1.0f;
            } else {
                // 정상적인 곳은 역투영된 원본 픽셀 좌표(x, y)를 LUT에 기록
                map1_.at<float>(v, u) = image_points[idx].x;
                map2_.at<float>(v, u) = image_points[idx].y;
            }
        }
    }

    // [Step 5: Calculate Optimal Ground Point (Sweet Spot)]
    // 카메라 광학 축(Z축)이 바닥 평면(Z=0)과 만나는 교점 계산 (물리적 오차/Covariance 모델링용)
    // Z=0 평면과의 교차점: origin.z + lambda * ray_dir.z = 0
    if (std::abs(ray_dir_in_base_z) > 1e-3) {
        double lambda = -origin_in_base_z / ray_dir_in_base_z;
        if (lambda > 0) {
            optimal_point_.x = origin_in_base_x + lambda * ray_dir_in_base_x;
            optimal_point_.y = origin_in_base_y + lambda * ray_dir_in_base_y;
            has_optimal_point_ = true;
        } else {
            has_optimal_point_ = false;
        }
    } else {
        has_optimal_point_ = false;
    }

    // [Step 6: Build Footprint Mask]
    // 룩업테이블 X, Y 좌표가 완성된 후, 고정된 차량 범위(Footprint)를 바이너리 마스크로 pre-compute해 둡니다.
    footprint_mask_ = cv::Mat::zeros(bev_cfg_.height, bev_cfg_.width, CV_8UC1);
    if (has_vehicle_footprint_) {
        for (int v = 0; v < bev_cfg_.height; ++v) {
            for (int u = 0; u < bev_cfg_.width; ++u) {
                float x = cartesian_lut_x_.at<float>(v, u);
                float y = cartesian_lut_y_.at<float>(v, u);
                if (x >= veh_x_min_ && x <= veh_x_max_ &&
                    std::abs(y) <= veh_y_half_) {
                    footprint_mask_.at<uint8_t>(v, u) = 255;
                }
            }
        }
    }

    lut_initialized_ = true;
}

std::vector<LocalFeature> FeatureExtractor::process(
    const cv::Mat& seg_img,
    cv::Mat& out_bev_image) {

    std::vector<LocalFeature> features;

    // [Step 1: 예외 처리] LUT가 아직 생성되지 않았다면 처리를 건너뜀
    if (!is_initialized_ || !lut_initialized_) {
        return features;
    }

    // 시각화(디버깅)를 위한 3채널 컬러 BEV 이미지 초기화
    out_bev_image = cv::Mat::zeros(bev_cfg_.height, bev_cfg_.width, CV_8UC3);

    // 특징점 검출을 위한 마스크 및 클래스 맵 생성
    cv::Mat mask = cv::Mat::zeros(bev_cfg_.height, bev_cfg_.width, CV_8UC1);
    cv::Mat class_map = cv::Mat::zeros(bev_cfg_.height, bev_cfg_.width, CV_8UC1);

    // 1. GT 이미지 모드 (image_type_ == "gt")
    if (image_type_ == "gt") {
        if (seg_img.channels() == 3) {
            // [Step 2: Backward Mapping 적용 (2D -> 3D 변환)]
            // 배경은 흰색(255,255,255)으로 채움 → 이후 시각화에서 검은색 배경으로 덮어쓰게 됨
            cv::remap(seg_img, out_bev_image, map1_, map2_, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

            // 채널 분리 (R=0, G=1, B=2)
            std::vector<cv::Mat> img_channels;
            cv::split(out_bev_image, img_channels);

            // 검은색 픽셀(차선) 및 노란색 픽셀(랜드마크) 마스크 생성
            cv::Mat is_black_mask = (img_channels[0] == 0) & (img_channels[1] == 0) & (img_channels[2] == 0);
            cv::Mat is_yellow_mask = (img_channels[0] == 255) & (img_channels[1] == 150) & (img_channels[2] == 0);

            // 유효하지 않은 영역(FOV 밖 또는 자차 풋프린트 범위) 마스크 생성
            cv::Mat invalid_mask = (map1_ < 0.0f) | (map1_ >= (float)seg_img.cols) |
                                   (map2_ < 0.0f) | (map2_ >= (float)seg_img.rows);
            if (has_vehicle_footprint_) {
                invalid_mask |= footprint_mask_;
            }

            // 유효하지 않은 영역은 검출 마스크에서 소거
            is_black_mask.setTo(0, invalid_mask);
            is_yellow_mask.setTo(0, invalid_mask);

            // 특징점 추출용 마스크 및 클래스 맵 설정 (루프 없이 고속 연산)
            mask = is_black_mask | is_yellow_mask;
            class_map.setTo(1, is_black_mask);
            class_map.setTo(2, is_yellow_mask);

            // 시각화 컬러 렌더링
            out_bev_image = cv::Mat::zeros(bev_cfg_.height, bev_cfg_.width, CV_8UC3);
            out_bev_image.setTo(cv::Scalar(255, 255, 255), is_black_mask); // 차선: 흰색
            out_bev_image.setTo(cv::Scalar(255, 150, 0), is_yellow_mask);  // 랜드마크: 노란색
        }
    }

    // 2. Raw BGR/Mono 카메라 이미지 모드 (image_type_ == "raw")
    else if (image_type_ == "raw") {
        if (seg_img.channels() == 3) {
            // [Step 2: Backward Mapping 적용 (2D -> 3D 변환)]
            cv::remap(seg_img, out_bev_image, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        } else if (seg_img.channels() == 1) {
            cv::Mat bev_class;
            cv::remap(seg_img, bev_class, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
            cv::cvtColor(bev_class, out_bev_image, cv::COLOR_GRAY2RGB);
        }

        // [자차(Ego) 풋프린트 마스킹]
        // Raw 이미지에서도 차량 보닛이나 차체가 카메라에 찍혀 왜곡 투영되는 영역을 검은색으로 소거합니다.
        if (has_vehicle_footprint_) {
            out_bev_image.setTo(cv::Scalar(0, 0, 0), footprint_mask_);
        }
    }
    // 3. 일반 세그멘테이션 모드 (image_type_ == "segmentation")
    else {
        cv::Mat bev_class;
        if (seg_img.channels() == 1) {
            // [Step 2: Backward Mapping 적용 (2D -> 3D 변환)]
            cv::remap(seg_img, bev_class, map1_, map2_, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
            mask = bev_class > 0;
            class_map = bev_class;

            // 유효하지 않은 영역(FOV 밖 또는 자차 풋프린트 범위) 마스크 생성
            cv::Mat invalid_mask = (map1_ < 0.0f) | (map1_ >= (float)seg_img.cols) |
                                   (map2_ < 0.0f) | (map2_ >= (float)seg_img.rows);
            if (has_vehicle_footprint_) {
                invalid_mask |= footprint_mask_;
            }

            // 유효하지 않은 영역은 검출 마스크에서 소거
            mask.setTo(0, invalid_mask);
            class_map.setTo(0, invalid_mask);

            // 각 클래스별 바이너리 마스크 생성 (필터링된 class_map 기준)
            cv::Mat val_1_mask = (class_map == 1);
            cv::Mat val_2_mask = (class_map == 2);
            cv::Mat val_other_mask = (class_map > 0) & (class_map != 1) & (class_map != 2);

            // 세그멘테이션 클래스 색상 매핑 (루프 없이 고속 렌더링)
            out_bev_image = cv::Mat::zeros(bev_cfg_.height, bev_cfg_.width, CV_8UC3);
            out_bev_image.setTo(cv::Scalar(255, 255, 255), val_1_mask);   // 차선: 흰색
            out_bev_image.setTo(cv::Scalar(255, 76, 76), val_2_mask);     // 랜드마크: 코랄색
            out_bev_image.setTo(cv::Scalar(100, 255, 100), val_other_mask); // 기타 세그먼트: 연초록
        }
    }

    if (cv::countNonZero(mask) == 0) return features;

    std::vector<cv::Point> non_zero_points;
    cv::findNonZero(mask, non_zero_points);
    features.reserve(non_zero_points.size());

    // [Step 3: 특징점 추출 및 불확실성(Covariance) 모델링]
    for (const auto& pt : non_zero_points) {
        int u = pt.x;
        int v = pt.y;
        uint8_t class_id = class_map.at<uint8_t>(v, u);

        // updateLUT에서 미리 캐싱해둔 물리적 3D 좌표 (Base 프레임 기준의 x, y 미터 값)
        float x = cartesian_lut_x_.at<float>(v, u);
        float y = cartesian_lut_y_.at<float>(v, u);
        float r = std::sqrt(x*x + y*y);  // 차량(Base) 중심으로부터의 방사형 거리

        // 유효 거리 필터링
        if (r > r_max_) continue;
        if (r < 0.01) continue;

        // [차량 풋프린트 필터링]
        // GT 이미지의 차량 외곽선(검은 테두리)이 차선으로 잘못 검출되는 것을 방지
        // Fr1A 좌표계 기준 차량 점유 영역 내 픽셀은 특징점에서 제외
        if (has_vehicle_footprint_ &&
            footprint_mask_.at<uint8_t>(v, u) == 255) {
            continue;
        }

        // [3-1. 방향 벡터 계산]
        // 카메라 광학 원점(cam_origin_x, cam_origin_y)에서 특징점(x,y)을 향하는 실제 방사형(Radial) 단위 벡터
        float dx_cam = x - cam_origin_x_;
        float dy_cam = y - cam_origin_y_;
        float r_cam = std::sqrt(dx_cam*dx_cam + dy_cam*dy_cam);

        float e_r_x = dx_cam / (r_cam + 1e-6f);
        float e_r_y = dy_cam / (r_cam + 1e-6f);
        // 방사형 벡터에 수직인 접선형(Tangential) 단위 벡터 (90도 회전)
        float e_t_x = -e_r_y;
        float e_t_y = e_r_x;

        // [3-2. 불확실성(Sigma) 계산]
        // 최적점(Optimal Point: 렌즈 축이 바닥에 닿는 가장 선명한 곳)으로부터의 거리 제곱 계산
        float dx_opt = (has_optimal_point_) ? (x - optimal_point_.x) : 0.0f;
        float dy_opt = (has_optimal_point_) ? (y - optimal_point_.y) : 0.0f;
        float dist_opt_sq = dx_opt*dx_opt + dy_opt*dy_opt;

        float base_sigma_m_per_px = bev_cfg_.resolution;

        // 방사형 오차(sigma_r, 깊이 방향): 카메라 원근법에 의해 거리의 제곱(Z^2)에 비례하여 오차가 급증함
        float sigma_r = base_sigma_m_per_px * (1.0f + cov_k_ * dist_opt_sq);
        float sigma_r_sq = sigma_r * sigma_r;

        // 접선형 오차(sigma_t, 좌우 각도 방향): 픽셀의 각도(FOV)에 의해 거리에 선형적(Z)으로 비례하여 오차가 증가함
        float dist_opt = std::sqrt(dist_opt_sq);
        float sigma_t = base_sigma_m_per_px * (1.0f + cov_k_ * dist_opt);
        float sigma_t_sq = sigma_t * sigma_t;

        // [3-3. 공분산(Covariance) 행렬 생성]
        // 극좌표계(Radial, Tangential) 기준의 오차를 데카르트 좌표계(X, Y)로 투영(Projection)
        // C_{xy} = R * \Sigma * R^T (회전변환) 연산
        LocalFeature f_msg;
        f_msg.x = x;
        f_msg.y = y;
        f_msg.cov_xx = sigma_r_sq * e_r_x * e_r_x + sigma_t_sq * e_t_x * e_t_x;
        f_msg.cov_xy = sigma_r_sq * e_r_x * e_r_y + sigma_t_sq * e_t_x * e_t_y;
        f_msg.cov_yy = sigma_r_sq * e_r_y * e_r_y + sigma_t_sq * e_t_y * e_t_y;
        f_msg.class_id = class_id;

        // 완성된 특징점을 메시지 배열에 추가
        features.push_back(f_msg);
    }

    return features;
}

} // namespace carmaker_localization
