#include "carmaker_localization/feature_extractor.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <sensor_msgs/image_encodings.h>

namespace carmaker_localization {

FeatureExtractor::FeatureExtractor(const ros::NodeHandle& nh, const std::string& camera_name,
                                    const std::vector<double>& x_range, const std::vector<double>& y_range)
    : nh_(nh), lut_initialized_(false), has_optimal_point_(false), camera_name_(camera_name) {
    ros::NodeHandle pnh("~");

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

    bev_cfg_.resolution = pnh.param("feature_extractor/bev/resolution", 0.05);

    bev_cfg_.width = std::ceil((bev_cfg_.x_max - bev_cfg_.x_min) / bev_cfg_.resolution);
    bev_cfg_.height = std::ceil((bev_cfg_.y_max - bev_cfg_.y_min) / bev_cfg_.resolution);
}

void FeatureExtractor::updateLUT(const sensor_msgs::CameraInfoConstPtr& info,
                                    const geometry_msgs::TransformStamped& tf) {
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
        geometry_msgs::Point pt_base;
        pt_base.x = pt.x; pt_base.y = pt.y; pt_base.z = pt.z;
        geometry_msgs::Point pt_cam;

        // [Step 2: Extrinsic Transform] Base 좌표계(Fr1A) -> Camera 광학 좌표계로 3D 회전 및 이동
        tf2::doTransform(pt_base, pt_cam, tf);
        cam_points.emplace_back(pt_cam.x, pt_cam.y, pt_cam.z);
    }

    // Camera intrinsic matrix 및 왜곡 계수(Distortion coefficients) 세팅
    cv::Mat K(3, 3, CV_64F, 0.0);
    for (int i = 0; i < 9; ++i) K.at<double>(i / 3, i % 3) = info->K[i];
    cv::Mat D;
    if (!info->D.empty()) {
        D = cv::Mat(info->D.size(), 1, CV_64F);
        for (size_t i = 0; i < info->D.size(); ++i) D.at<double>(i, 0) = info->D[i];
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
    if (info->distortion_model == "equidistant" || info->distortion_model == "fisheye") {
        cv::fisheye::projectPoints(cam_points, image_points, rvec, tvec, K, D);
    } else {
        cv::projectPoints(cam_points, rvec, tvec, K, D, image_points);
    }

    // [Step 4: Backward Mapping LUT Construction]
    // cv::remap을 위한 룩업 테이블 생성: 빈 3D 캔버스(u, v)에 원본 이미지의 어느 픽셀(x, y)을 가져올지 기록
    map1_ = cv::Mat(bev_cfg_.height, bev_cfg_.width, CV_32FC1);
    map2_ = cv::Mat(bev_cfg_.height, bev_cfg_.width, CV_32FC1);
    for (int v = 0; v < bev_cfg_.height; ++v) {
        for (int u = 0; u < bev_cfg_.width; ++u) {
            int idx = v * bev_cfg_.width + u;
            map1_.at<float>(v, u) = image_points[idx].x;
            map2_.at<float>(v, u) = image_points[idx].y;
        }
    }

    // [Step 5: Calculate Optimal Ground Point (Sweet Spot)]
    // 카메라 광학 축(Z축)이 바닥 평면(Z=0)과 만나는 교점 계산 (물리적 오차/Covariance 모델링용)
    tf2::Transform tf_base2cam;
    tf2::fromMsg(tf.transform, tf_base2cam);
    tf2::Transform tf_cam2base = tf_base2cam.inverse();

    tf2::Vector3 origin_in_base = tf_cam2base * tf2::Vector3(0,0,0);
    tf2::Vector3 ray_dir_in_base = tf_cam2base.getBasis() * tf2::Vector3(0,0,1);

    // Z=0 평면과의 교차점: origin.z + lambda * ray_dir.z = 0
    if (std::abs(ray_dir_in_base.z()) > 1e-3) {
        double lambda = -origin_in_base.z() / ray_dir_in_base.z();
        if (lambda > 0) {
            optimal_point_.x = origin_in_base.x() + lambda * ray_dir_in_base.x();
            optimal_point_.y = origin_in_base.y() + lambda * ray_dir_in_base.y();
            has_optimal_point_ = true;
            NODELET_INFO("[%s] Optimal ground focus point calculated at: (%.2f, %.2f)",
                            camera_name_.c_str(), optimal_point_.x, optimal_point_.y);
        } else {
            has_optimal_point_ = false;
        }
    } else {
        has_optimal_point_ = false;
    }

    lut_initialized_ = true;
}

carmaker_msgs::LocalFeatures FeatureExtractor::process(
    const sensor_msgs::ImageConstPtr& seg_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg,
    cv::Mat& out_bev_image) {

    carmaker_msgs::LocalFeatures features;
    features.header = seg_msg->header;
    features.camera_name = camera_name_;

    // [Step 1: 예외 처리] LUT가 아직 생성되지 않았다면 처리를 건너뜀
    if (!lut_initialized_) {
        ROS_WARN_THROTTLE(1.0, "LUT not initialized yet.");
        return features;
    }

    // ROS 이미지 메시지를 OpenCV Mat 형식으로 변환 (흑백/클래스 ID 이미지 가정)
    cv_bridge::CvImagePtr cv_seg;
    try {
        cv_seg = cv_bridge::toCvCopy(seg_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return features;
    }

    // [Step 2: Backward Mapping 적용 (2D -> 3D 변환)]
    // updateLUT에서 만든 map1_, map2_를 사용하여 원본 이미지(cv_seg->image)의 픽셀들을 빈 3D BEV 캔버스(bev_class)로 병합
    cv::Mat bev_class;
    cv::remap(cv_seg->image, bev_class, map1_, map2_, cv::INTER_NEAREST);

    // 시각화(디버깅)를 위한 3채널 컬러 BEV 이미지 초기화
    out_bev_image = cv::Mat::zeros(bev_cfg_.height, bev_cfg_.width, CV_8UC3);

    // 배경(0)이 아닌, 의미 있는 특징점(클래스 1, 2 등) 픽셀만 추출하기 위한 마스크 생성
    cv::Mat mask = bev_class > 0;
    if (cv::countNonZero(mask) == 0) return features;

    // 파라미터 로드: 최대 탐지 거리(r_max) 및 오차 증가 계수(cov_k)
    ros::NodeHandle pnh("~");
    double r_max = pnh.param("feature_extractor/extraction/r_max", 15.0);
    double cov_k = pnh.param("feature_extractor/extraction/covariance_k", 1.0);

    std::vector<cv::Point> non_zero_points;
    cv::findNonZero(mask, non_zero_points);
    features.features.reserve(non_zero_points.size());

    // [Step 3: 특징점 추출 및 불확실성(Covariance) 모델링]
    for (const auto& pt : non_zero_points) {
        int u = pt.x;
        int v = pt.y;
        uint8_t class_id = bev_class.at<uint8_t>(v, u);

        // 시각화 이미지 색상 칠하기 (1: 차선(흰색), 2: 연석 등(코랄색))
        if (class_id == 1) out_bev_image.at<cv::Vec3b>(v, u) = cv::Vec3b(255, 255, 255); // White
        else if (class_id == 2) out_bev_image.at<cv::Vec3b>(v, u) = cv::Vec3b(76, 76, 255); // Coral

        // updateLUT에서 미리 캐싱해둔 물리적 3D 좌표 (Base 프레임 기준의 x, y 미터 값)
        float x = cartesian_lut_x_.at<float>(v, u);
        float y = cartesian_lut_y_.at<float>(v, u);
        float r = std::sqrt(x*x + y*y);  // 차량(Base) 중심으로부터의 방사형 거리

        // 유효 거리 필터링 (너무 멀거나, 중심점 0에 너무 가까운 노이즈 제거)
        if (r > r_max) continue;
        if (r < 0.01) continue;

        // [3-1. 방향 벡터 계산]
        // Base 기준점(0,0)에서 특징점(x,y)을 향하는 방사형(Radial) 단위 벡터
        float e_r_x = x / r;
        float e_r_y = y / r;
        // 방사형 벡터에 수직인 접선형(Tangential) 단위 벡터 (90도 회전)
        float e_t_x = -y / r;
        float e_t_y = x / r;

        // [3-2. 불확실성(Sigma) 계산]
        // 최적점(Optimal Point: 렌즈 축이 바닥에 닿는 가장 선명한 곳)으로부터의 거리 제곱 계산
        float dx_opt = (has_optimal_point_) ? (x - optimal_point_.x) : 0.0f;
        float dy_opt = (has_optimal_point_) ? (y - optimal_point_.y) : 0.0f;
        float dist_opt_sq = dx_opt*dx_opt + dy_opt*dy_opt;

        float base_sigma_m_per_px = bev_cfg_.resolution;

        // 방사형 오차(sigma_r): 최적점에서 멀어질수록 픽셀이 덮는 물리적 거리가 넓어짐(늘어남)을 모델링
        float sigma_r = base_sigma_m_per_px * (1.0f + cov_k * dist_opt_sq);
        float sigma_r_sq = sigma_r * sigma_r;

        // 접선형 오차(sigma_t): 좌우 해상도는 그리드 해상도에 크게 의존하므로 상수로 가정
        float sigma_t_sq = bev_cfg_.resolution * bev_cfg_.resolution;

        // [3-3. 공분산(Covariance) 행렬 생성]
        // 극좌표계(Radial, Tangential) 기준의 오차를 데카르트 좌표계(X, Y)로 투영(Projection)
        // C_{xy} = R * \Sigma * R^T (회전변환) 연산
        carmaker_msgs::LocalFeature f_msg;
        f_msg.x = x;
        f_msg.y = y;
        f_msg.cov_xx = sigma_r_sq * e_r_x * e_r_x + sigma_t_sq * e_t_x * e_t_x;
        f_msg.cov_xy = sigma_r_sq * e_r_x * e_r_y + sigma_t_sq * e_t_x * e_t_y;
        f_msg.cov_yy = sigma_r_sq * e_r_y * e_r_y + sigma_t_sq * e_t_y * e_t_y;
        f_msg.class_id = class_id;

        // 완성된 특징점을 메시지 배열에 추가
        features.features.push_back(f_msg);
    }

    return features;
}

} // namespace carmaker_localization
