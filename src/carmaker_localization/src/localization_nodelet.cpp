#include "carmaker_localization/localization_nodelet.h"
#include <ros/console.h>
#include "carmaker_localization/osm_landmark_loader.h"
#include "carmaker_localization/icp_registration.h"
#include <XmlRpcValue.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace carmaker_localization {

LocalizationNodelet::~LocalizationNodelet() {
    run_correction_worker_ = false;
    correction_queue_cv_.notify_all();
    if (correction_worker_thread_.joinable()) {
        correction_worker_thread_.join();
    }
}

void LocalizationNodelet::onInit() {
    // 시뮬레이터 clock 때문에 반복 출력되는 TF2 시간 도약 경고를 억제
    if (ros::console::set_logger_level("ros.tf2", ros::console::levels::Error)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    if (ros::console::set_logger_level("ros.tf2_ros", ros::console::levels::Error)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    NODELET_INFO("Initializing carmaker_localization Nodelet...");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

    ros::NodeHandle& nh = getNodeHandle();
    visualizer_ = std::make_shared<Visualizer>(nh);

    // 단일 책임 원칙에 맞춰 분리한 생명주기 초기화
    if (!loadParameters()) {
        NODELET_ERROR("Failed to load parameters. Aborting initialization to prevent crash.");
        return;
    }
    if (!initEkf()) {
        NODELET_ERROR("Failed to initialize EKF. Aborting initialization to prevent crash.");
        return;
    }
    if (!initSvm()) {
        NODELET_ERROR("Failed to initialize SVM. Aborting initialization to prevent crash.");
        return;
    }
    if (!setupRosIo()) {
        NODELET_ERROR("Failed to setup ROS IO and Feature Loader. Aborting initialization to prevent crash.");
        return;
    }

    // ICP 정합 처리를 비동기로 수행할 백그라운드 워커 스레드 기동
    correction_worker_thread_ = std::thread(&LocalizationNodelet::correctionWorkerLoop, this);
}

bool LocalizationNodelet::loadParameters() {
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    image_type_ = pnh.param("setting/image_type", std::string("gt"));

    global_frame_ = pnh.param("frames/global", std::string("map"));
    prediction_bumper_frame_ = pnh.param("frames/prediction_bumper", std::string("Fr1A_Pred"));
    prediction_rear_axle_frame_ = pnh.param("frames/prediction_rear_axle", std::string("Fr1A_Rear_Axle_Pred"));

    tire_radius_ = pnh.param("vehicle/tire_radius", 0.298);
    track_width_ = pnh.param("vehicle/track_width", 1.634);
    rear_axle_x_ = pnh.param("vehicle/rear_axle_offset_x", 0.82);
    wheelbase_ = pnh.param("vehicle/wheelbase", 2.97);

    wheel_speed_std_ = pnh.param("ekf/wheel/speed_std", 0.05);
    imu_gyro_std_ = pnh.param("ekf/imu/gyro_std", 0.01);

    resolution_ = pnh.param("feature_extractor/bev/resolution", 0.05);
    r_max_ = pnh.param("feature_extractor/extraction/r_max", 15.0);

    max_position_step_ = pnh.param("ekf/rate_limiter/max_position_step", 0.15);
    max_yaw_step_ = pnh.param("ekf/rate_limiter/max_yaw_step", 0.05);
    max_position_dev_ = pnh.param("feature_registration/validation_gate/max_position_dev", 1.0);
    max_yaw_dev_ = pnh.param("feature_registration/validation_gate/max_yaw_dev", 0.25);

    pnh.param<double>("diagnostic_period", diag_period_, 1.0);

    // 수동 초기 상태 설정
    XmlRpc::XmlRpcValue init_state_val;
    if (pnh.getParam("ekf/initial_state", init_state_val) && init_state_val.getType() == XmlRpc::XmlRpcValue::TypeArray && init_state_val.size() == 3) {
        use_manual_initial_state_ = true;
        init_x_ = static_cast<double>(init_state_val[0]);
        init_y_ = static_cast<double>(init_state_val[1]);
        double yaw_deg = static_cast<double>(init_state_val[2]);
        init_yaw_ = yaw_deg * M_PI / 180.0;
        NODELET_INFO("Manual initial state set: [%.2f, %.2f, %.2f deg]", init_x_, init_y_, yaw_deg);
    } else {
        use_manual_initial_state_ = false;
        NODELET_INFO("No valid initial_state found. Will initialize from DynamicsInfo (GT).");
    }

    // 채널 설정 1회 파싱 전략
    XmlRpc::XmlRpcValue channel_list;
    std::string channels_param_key = "topics/subscribe/channels";
    if (!pnh.hasParam(channels_param_key)) {
        channels_param_key = "feature_extractor/channels";
    }

    if (pnh.getParam(channels_param_key, channel_list) && channel_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        if (channel_list.size() != NUM_CAMERAS) {
            NODELET_FATAL("This package is hardcoded for exactly %zu cameras. Found %d channels.", NUM_CAMERAS, channel_list.size());
            return false;
        }

        std::vector<double> vis_x_range = {-20.0, 20.0};
        if (pnh.hasParam("feature_extractor/bev_vis/x_range")) {
            pnh.getParam("feature_extractor/bev_vis/x_range", vis_x_range);
        }
        std::vector<double> vis_y_range = {-20.0, 20.0};
        if (pnh.hasParam("feature_extractor/bev_vis/y_range")) {
            pnh.getParam("feature_extractor/bev_vis/y_range", vis_y_range);
        }

        channels_.reserve(NUM_CAMERAS);
        ros::NodeHandle& nh = getNodeHandle();

        for (int i = 0; i < channel_list.size(); ++i) {
            XmlRpc::XmlRpcValue& ch_cfg = channel_list[i];
            channels_.emplace_back();
            Channel& ch = channels_.back();

            ch.name = static_cast<std::string>(ch_cfg["name"]);
            ch.frame = static_cast<std::string>(ch_cfg["frame"]);

            if (ch_cfg.hasMember("input_topic")) {
                ch.input_topic = static_cast<std::string>(ch_cfg["input_topic"]);
            }
            ch.info_topic = ch_cfg.hasMember("info_topic") ? static_cast<std::string>(ch_cfg["info_topic"]) : "/synced/" + ch.name + "/camera_info";

            std::vector<double> x_range = {-10.0, 10.0};
            if (ch_cfg.hasMember("bev_x_range") && ch_cfg["bev_x_range"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                x_range[0] = static_cast<double>(ch_cfg["bev_x_range"][0]);
                x_range[1] = static_cast<double>(ch_cfg["bev_x_range"][1]);
            }
            std::vector<double> y_range = {-5.0, 15.0};
            if (ch_cfg.hasMember("bev_y_range") && ch_cfg["bev_y_range"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                y_range[0] = static_cast<double>(ch_cfg["bev_y_range"][0]);
                y_range[1] = static_cast<double>(ch_cfg["bev_y_range"][1]);
            }

            double ch_max_fov = 180.0;
            if (ch_cfg.hasMember("max_fov")) {
                if (ch_cfg["max_fov"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    ch_max_fov = static_cast<double>(ch_cfg["max_fov"]);
                } else if (ch_cfg["max_fov"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                    ch_max_fov = static_cast<int>(ch_cfg["max_fov"]);
                }
            }

            double ch_cov_k = 1.0;
            if (ch_cfg.hasMember("covariance_k")) {
                if (ch_cfg["covariance_k"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    ch_cov_k = static_cast<double>(ch_cfg["covariance_k"]);
                } else if (ch_cfg["covariance_k"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                    ch_cov_k = static_cast<int>(ch_cfg["covariance_k"]);
                }
            }

            ch.extractor = std::make_shared<FeatureExtractor>(ch.name);
            ch.extractor->initialize(x_range, y_range, image_type_, resolution_);
            ch.extractor->initializeVisualization(vis_x_range, vis_y_range, resolution_);
            ch.extractor->setExtractionParameters(r_max_, ch_cov_k, ch_max_fov);

            // 차량 풋프린트 필터: GT 이미지에서 차량 외곽선(검은 테두리)이 차선으로 오검출되는 것 방지
            // Fr1A 원점 = 후방 범퍼 중앙, x=[0, length], y=[-width/2, width/2]
            double veh_length       = pnh.param("vehicle/length", 4.635);
            double veh_width        = pnh.param("vehicle/width",  1.9);
            double veh_height       = pnh.param("vehicle/height", 1.605);
            double footprint_margin = pnh.param("svm/footprint_margin", 0.15);
            ch.extractor->setVehicleFootprint(0.0, veh_length, veh_width / 2.0, veh_height, footprint_margin);
            ch.bev_x_range = x_range;
            ch.bev_y_range = y_range;
        }
    } else {
        NODELET_FATAL("Failed to get channels configuration array!");
        return false;
    }

    // 이미 로드한 채널 설정에서 SVM 범위를 계산한다(채널 설정을 다시 파싱하지 않음).
    svm_x_min_ = 1e9; svm_x_max_ = -1e9;
    svm_y_min_ = 1e9; svm_y_max_ = -1e9;

    for (const auto& ch : channels_) {
        svm_x_min_ = std::min(svm_x_min_, ch.bev_x_range[0]);
        svm_x_max_ = std::max(svm_x_max_, ch.bev_x_range[1]);
        svm_y_min_ = std::min(svm_y_min_, ch.bev_y_range[0]);
        svm_y_max_ = std::max(svm_y_max_, ch.bev_y_range[1]);
    }

    // 범위 계산 실패 시 기본값
    if (svm_x_min_ > svm_x_max_) { svm_x_min_ = -3.0; svm_x_max_ = 7.68; }
    if (svm_y_min_ > svm_y_max_) { svm_y_min_ = -3.0; svm_y_max_ = 3.0; }

    return true;
}

bool LocalizationNodelet::initEkf() {
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    int dim = pnh.param("ekf/dimension", 2);
    if (dim != 2) {
        NODELET_WARN("Currently only 2D EKF is supported! Overriding dimension to 2.");
    }

    if (wheelbase_ <= 0.1 || rear_axle_x_ < 0.0) {
        NODELET_ERROR("Invalid vehicle specifications. wheelbase: %.2f (expected > 0.1), rear_axle_offset_x: %.2f (expected >= 0.0)", wheelbase_, rear_axle_x_);
        return false;
    }

    // 참고: ekf/state_buffer_duration은 더 이상 사용하지 않는다.
    // 과거 상태로 되돌린 뒤 재전파하는 보정 방식은 제거했고,
    // 100ms 미만의 비전 지연은 현재 상태에 직접 반영한다.
    ekf_core_ = std::make_shared<EkfCore>();
    if (!ekf_core_) {
        NODELET_ERROR("Failed to allocate EkfCore instance.");
        return false;
    }

    q_pos_std_ = pnh.param("ekf/process_noise/pos_std", 0.05);
    q_yaw_std_ = pnh.param("ekf/process_noise/yaw_std", 0.03);

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
    Q(X, X) = std::pow(q_pos_std_, 2);
    Q(Y, Y) = std::pow(q_pos_std_, 2);
    Q(YAW, YAW) = std::pow(q_yaw_std_, 2);

    ekf_core_->setProcessNoise(Q);

    // ROS 의존성을 EkfCore에서 Nodelet으로 격리: warning 로그 콜백 등록
    ekf_core_->setWarnLogCallback(
        [this](const std::string& s) { NODELET_WARN("%s", s.c_str()); }
    );
    NODELET_INFO("EKF Prediction Model: 3D pose state with motion input [velocity, yaw_rate]");
    NODELET_INFO("EKF Frame: rear axle (rear_axle_offset: %.2fm)", rear_axle_x_);
    return true;
}

bool LocalizationNodelet::initSvm() {
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    svm_res_ = pnh.param("feature_extractor/bev/resolution", 0.05);
    if (svm_res_ <= 1e-4) {
        NODELET_ERROR("Invalid SVM resolution parameter: %.4f (expected > 1e-4)", svm_res_);
        return false;
    }
    fusion_ = pnh.param("feature_extractor/fusion", false);

    int svm_h = std::ceil((svm_x_max_ - svm_x_min_) / svm_res_);
    int svm_w = std::ceil((svm_y_max_ - svm_y_min_) / svm_res_);
    if (svm_h <= 0 || svm_w <= 0) {
        NODELET_ERROR("Invalid SVM grid dimensions computed: H=%d, W=%d", svm_h, svm_w);
        return false;
    }
    try {
        svm_canvas_ = cv::Mat::zeros(svm_h, svm_w, CV_8UC3);
    } catch (const cv::Exception& e) {
        NODELET_ERROR("OpenCV Exception allocating svm_canvas_: %s", e.what());
        return false;
    }

    double v_front = pnh.param("vehicle/length", 4.635);
    double v_rear = 0;
    double front_off = pnh.param("svm/seam_front_offset", -0.5);
    double rear_off = pnh.param("svm/seam_rear_offset", 0.5);
    double slope_front = pnh.param("svm/seam_slope_front", 1.0);
    double slope_rear = pnh.param("svm/seam_slope_rear", 1.0);

    double v_fc_x = v_front + front_off;
    double v_rc_x = v_rear + rear_off;

    auto m_to_pix = [&](double x, double y) -> cv::Point {
        int r = std::round((svm_x_max_ - x) / svm_res_);
        int c = std::round((svm_y_max_ - y) / svm_res_);
        return cv::Point(c, r);
    };

    cv::Point c_tl = m_to_pix(svm_x_max_, svm_y_max_);
    cv::Point c_tr = m_to_pix(svm_x_max_, svm_y_min_);
    cv::Point c_bl = m_to_pix(svm_x_min_, svm_y_max_);
    cv::Point c_br = m_to_pix(svm_x_min_, svm_y_min_);

    cv::Point p_fc = m_to_pix(v_fc_x, 0.0);
    cv::Point p_rc = m_to_pix(v_rc_x, 0.0);

    cv::Point p_l1_fl = m_to_pix(svm_x_max_, slope_front * (svm_x_max_ - v_fc_x));
    cv::Point p_l2_fl = m_to_pix(svm_x_max_, -slope_front * (svm_x_max_ - v_fc_x));
    cv::Point p_l1_rl = m_to_pix(svm_x_min_, slope_rear * (v_rc_x - svm_x_min_));
    cv::Point p_l2_rl = m_to_pix(svm_x_min_, -slope_rear * (v_rc_x - svm_x_min_));

    cv::Mat index_map(svm_h, svm_w, CV_8UC1, cv::Scalar(3)); // Left = 3
    std::vector<cv::Point> pts_right = {p_fc, p_l2_fl, c_tr, c_br, p_l2_rl, p_rc};
    cv::fillPoly(index_map, std::vector<std::vector<cv::Point>>{pts_right}, cv::Scalar(4)); // Right = 4
    std::vector<cv::Point> pts_front = {p_fc, p_l1_fl, p_l2_fl};
    cv::fillPoly(index_map, std::vector<std::vector<cv::Point>>{pts_front}, cv::Scalar(1)); // Front = 1
    std::vector<cv::Point> pts_rear = {p_rc, p_l1_rl, p_l2_rl};
    cv::fillPoly(index_map, std::vector<std::vector<cv::Point>>{pts_rear}, cv::Scalar(2)); // Rear = 2

    svm_masks_["front"] = (index_map == 1);
    svm_masks_["rear"] = (index_map == 2);
    svm_masks_["left"] = (index_map == 3);
    svm_masks_["right"] = (index_map == 4);

    seam_line_points_ = {p_fc, p_l1_fl, p_l2_fl, p_rc, p_l1_rl, p_l2_rl};
    return true;
}

bool LocalizationNodelet::setupRosIo() {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    // 랜드마크 로더 설정
    std::string landmark_format = pnh.param("feature_registration/landmark_format", std::string(""));
    if (landmark_format.empty()) {
        landmark_format = pnh.param("feature_registration/map_format", std::string("osm"));
    }
    std::string landmark_file = pnh.param("feature_registration/landmark_file", std::string(""));
    if (landmark_file.empty()) {
        landmark_file = pnh.param("feature_registration/map_file", std::string(""));
    }

    if (landmark_format == "osm") {
        landmark_loader_ = std::make_shared<OsmLandmarkLoader>(resolution_);
        if (!landmark_loader_) {
            NODELET_ERROR("Failed to allocate OsmLandmarkLoader instance.");
            return false;
        }
        if (!landmark_file.empty()) {
            if (landmark_loader_->load(landmark_file)) {
                NODELET_INFO("Landmarks loaded successfully from: %s", landmark_file.c_str());
                if (visualizer_) {
                    visualizer_->publishLandmarkFeatures(landmark_loader_->queryNear(0.0, 0.0, -1.0));
                }
            } else {
                NODELET_ERROR("Failed to load landmarks: %s", landmark_file.c_str());
                return false;
            }
        } else {
            NODELET_WARN("No landmark_file specified for feature_registration. Landmarks will be empty.");
        }
    }

    // 정합 엔진 설정
    std::string registration_type = pnh.param("feature_registration/type", std::string("icp"));
    fitness_threshold_ = pnh.param("feature_registration/fitness_threshold", 0.5);
    publish_metrics_always_ = pnh.param("feature_registration/publish_metrics_always", true);
    int max_iterations = pnh.param("feature_registration/max_iterations", 50);
    search_radius_ = pnh.param("feature_registration/search_radius", 20.0);
    double vision_base_std = pnh.param("ekf/camera/base_std", 0.1);
    double vision_base_yaw_std = pnh.param("ekf/camera/base_yaw_std", 0.02);
    double max_covariance = pnh.param("feature_registration/max_covariance", 10.0);

    double min_search_radius = pnh.param("feature_registration/min_search_radius", 0.5);
    int max_observed_features = pnh.param("feature_registration/max_observed_features", 400);

    if (registration_type == "icp") {
        IcpParams icp_params;
        icp_params.fitness_threshold = fitness_threshold_;
        icp_params.max_iterations = max_iterations;
        icp_params.vision_base_std = vision_base_std;
        icp_params.vision_base_yaw_std = vision_base_yaw_std;
        icp_params.min_search_radius = min_search_radius;
        icp_params.max_covariance = max_covariance;
        icp_params.max_observed_features = max_observed_features;

        registration_engine_ = std::make_shared<IcpRegistration>(icp_params);
        if (!registration_engine_) {
            NODELET_ERROR("Failed to allocate IcpRegistration instance.");
            return false;
        }
    }

    // 정합 활성화 여부 (false 시 EKF 예측만 수행, 디버깅용)
    feature_registration_enabled_ = pnh.param("feature_registration/enable", true);
    NODELET_INFO("Feature registration: %s", feature_registration_enabled_ ? "ENABLED" : "DISABLED (debug mode)");

    // 입출력 발행자 및 구독자
    std::string topic_pose = pnh.param("topics/publish/pose", std::string("/localization/pose"));
    std::string topic_odom = pnh.param("topics/publish/odom", std::string("/localization/odom"));
    std::string topic_dynamics = pnh.param("topics/subscribe/dynamics", std::string("/dynamics_info"));

    dynamics_sub_ = nh.subscribe(topic_dynamics, 100, &LocalizationNodelet::dynamicsCallback, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_odom, 10);
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_pose, 10);

    NODELET_INFO("EKF prediction is driven by DynamicsInfo timestamps.");

    std::string topic_estimation_data = pnh.param("topics/publish/data/estimation_pose", std::string("/localization/data/estimation_pose"));
    std::string topic_correction_data = pnh.param("topics/publish/data/correction_pose", std::string("/localization/data/correction_pose"));
    std::string topic_rmse_pos = pnh.param("topics/publish/data/rmse_position", std::string("/localization/data/rmse_position"));
    std::string topic_yaw_error = pnh.param("topics/publish/data/yaw_error", std::string("/localization/data/yaw_error"));
    std::string topic_yaw_rmse = pnh.param("topics/publish/data/yaw_rmse", std::string("/localization/data/yaw_rmse"));
    std::string topic_longitudinal_error = pnh.param("topics/publish/data/longitudinal_error", std::string("/localization/data/longitudinal_error"));
    std::string topic_lateral_error = pnh.param("topics/publish/data/lateral_error", std::string("/localization/data/lateral_error"));
    std::string topic_longitudinal_rmse = pnh.param("topics/publish/data/longitudinal_rmse", std::string("/localization/data/longitudinal_rmse"));
    std::string topic_lateral_rmse = pnh.param("topics/publish/data/lateral_rmse", std::string("/localization/data/lateral_rmse"));
    std::string topic_nees = pnh.param("topics/publish/data/nees", std::string("/localization/data/nees"));
    estimation_data_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_estimation_data, 10);
    correction_data_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_correction_data, 10);
    rmse_pos_pub_ = nh.advertise<std_msgs::Float64>(topic_rmse_pos, 10);
    yaw_error_pub_ = nh.advertise<std_msgs::Float64>(topic_yaw_error, 10);
    yaw_rmse_pub_ = nh.advertise<std_msgs::Float64>(topic_yaw_rmse, 10);
    longitudinal_error_pub_ = nh.advertise<std_msgs::Float64>(topic_longitudinal_error, 10);
    lateral_error_pub_ = nh.advertise<std_msgs::Float64>(topic_lateral_error, 10);
    longitudinal_rmse_pub_ = nh.advertise<std_msgs::Float64>(topic_longitudinal_rmse, 10);
    lateral_rmse_pub_ = nh.advertise<std_msgs::Float64>(topic_lateral_rmse, 10);
    nees_pub_ = nh.advertise<std_msgs::Float64>(topic_nees, 10);

    std::string topic_r_wheel_vx = pnh.param("topics/publish/debug/r_wheel_vx", std::string("/localization/debug/r_wheel_vx"));
    std::string topic_r_wheel_yaw_rate = pnh.param("topics/publish/debug/r_wheel_yaw_rate", std::string("/localization/debug/r_wheel_yaw_rate"));
    std::string topic_r_imu_yaw_rate = pnh.param("topics/publish/debug/r_imu_yaw_rate", std::string("/localization/debug/r_imu_yaw_rate"));
    std::string topic_icp_metrics = pnh.param("topics/publish/debug/icp_metrics", std::string("/localization/debug/icp_metrics"));

    debug_r_wheel_vx_pub_ = nh.advertise<std_msgs::Float64>(topic_r_wheel_vx, 10);
    debug_r_wheel_yaw_rate_pub_ = nh.advertise<std_msgs::Float64>(topic_r_wheel_yaw_rate, 10);
    debug_r_imu_yaw_rate_pub_ = nh.advertise<std_msgs::Float64>(topic_r_imu_yaw_rate, 10);
    icp_metrics_pub_ = nh.advertise<carmaker_msgs::IcpRegistrationMetrics>(topic_icp_metrics, 10);

    // 진단 업데이터 설정(표준 /diagnostics 토픽으로 직접 발행)
    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(nh, pnh);
    if (!diagnostic_updater_) {
        NODELET_ERROR("Failed to allocate diagnostic_updater.");
        return false;
    }
    diagnostic_updater_->setHardwareID("carmaker_localization");
    diagnostic_updater_->add("Localization Status", this, &LocalizationNodelet::produceDiagnostics);

    // 진단용 WallTimer 설정
    diag_timer_ = nh.createWallTimer(ros::WallDuration(diag_period_), &LocalizationNodelet::diagTimerCallback, this);

    // nodelet 시작 시 모든 특징점 발행자를 미리 초기화한다(스레드 안전).
    std::string prefix = pnh.param("topics/publish/data/features_prefix", std::string("/localization/data/features"));
    for (const auto& ch : channels_) {
        feature_data_pubs_[ch.name] = nh.advertise<carmaker_msgs::LocalFeatures>(prefix + "/" + ch.name, 10);
    }

    // 카메라 구독 설정
    use_bundle_ = pnh.param("topics/subscribe/use_bundle", false);
    std::string topic_bundle = pnh.param("topics/subscribe/bundle", std::string("/segmentation/camera_bundle"));

    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
        const Channel& ch = channels_[i];
        if (!use_bundle_) {
            if (ch.input_topic.empty()) {
                NODELET_FATAL("input_topic is required when use_bundle is false!");
                return false;
            }
            // 비전 메시지가 대기하지 않고 네트워크 레이어에서 즉시 유실 처리
            seg_subs_[i] = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh, ch.input_topic, 1, ros::TransportHints().tcpNoDelay());
            info_subs_[i] = nh.subscribe<sensor_msgs::CameraInfo>(ch.info_topic, 1, boost::bind(&LocalizationNodelet::infoCallback, this, _1, i));
        }

        NODELET_INFO("Configured Localization Channel %zu: [%s]", i, ch.name.c_str());
    }

    if (!use_bundle_) {
        sync_all_ = std::make_unique<message_filters::Synchronizer<SyncPolicy4>>(
            SyncPolicy4(1), // 대기 큐 크기 최소화하여 최신 프레임 활용
            *seg_subs_[0], *seg_subs_[1], *seg_subs_[2], *seg_subs_[3]);

        sync_all_->registerCallback(boost::bind(&LocalizationNodelet::imagesCallback, this, _1, _2, _3, _4));
    } else {
        // 대기 큐 크기를 1로 제한하고 tcpNoDelay()를 적용해 네트워크 버퍼링 지연 제거
        bundle_sub_ = nh.subscribe(topic_bundle, 1, &LocalizationNodelet::bundleCallback, this, ros::TransportHints().tcpNoDelay());
        NODELET_INFO("Subscribed to CameraBundle topic: %s", topic_bundle.c_str());
    }

    // ROS 서비스 서버 설정
    std::string service_get_map = pnh.param("services/get_map", std::string("/localization/get_map"));
    map_srv_ = nh.advertiseService(service_get_map, &LocalizationNodelet::getMapCallback, this);
    NODELET_INFO("Map service advertised: %s", service_get_map.c_str());
    return true;
}

void LocalizationNodelet::infoCallback(const sensor_msgs::CameraInfoConstPtr& msg, size_t idx) {
    // boost::shared_ptr 읽기/쓰기 경합을 막기 위한 스레드 안전 잠금
    std::lock_guard<std::mutex> lock(info_array_mutex_);
    latest_infos_[idx] = msg;
}

void LocalizationNodelet::dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg) {
    carmaker_msgs::DynamicsInfo current_dynamics = *msg;
    {
        std::lock_guard<std::mutex> lock(dyn_mutex_);
        latest_dynamics_ = current_dynamics;
        dynamics_received_ = true;
    }

    double current_time = current_dynamics.header.stamp.toSec();
    if (current_time <= 0.0) {
        current_time = current_dynamics.time.toSec();
    }
    if (current_time <= 0.0) {
        current_time = ros::Time::now().toSec();
    }
    if (current_time <= 0.0) {
        current_time = ros::WallTime::now().toSec();
    }

    updateEstimation(current_time, current_dynamics);
}

void LocalizationNodelet::updateEstimation(double current_time, const carmaker_msgs::DynamicsInfo& dynamics) {
    bool needs_reset = false;
    bool needs_init = false;

    {
        std::lock_guard<std::mutex> lock(estimation_mutex_);

        // 1. 시간 도약 감지(예: bag 반복 재생 또는 시뮬레이션 재시작)
        if (last_prediction_time_ > 0.0) {
            double raw_dt = current_time - last_prediction_time_;
            if (raw_dt < -1.0 || raw_dt > 5.0) {
                NODELET_WARN_THROTTLE(2.0, "EKF Time jump detected (dt: %.3f). Triggering localization reset...", raw_dt);
                needs_reset = true;
            }
            if (last_processed_cycleno_ >= 0 && dynamics.cycleno >= 0 && dynamics.cycleno < last_processed_cycleno_) {
                NODELET_WARN_THROTTLE(2.0, "DynamicsInfo cycle number jumped backwards (%lld -> %lld). Triggering localization reset...",
                                      last_processed_cycleno_, static_cast<long long>(dynamics.cycleno));
                needs_reset = true;
            }
        }

        // 2. 최초 실행 초기화
        if (!needs_reset && last_prediction_time_ <= 0.0) {
            needs_init = true;
        }
    }

    if (needs_reset) {
        resetLocalization();
        return;
    }
    if (needs_init) {
        initLocalization(current_time, dynamics);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(estimation_mutex_);
        if (last_prediction_time_ <= 0.0) {
            return;
        }

        if (dynamics.cycleno > 0 && dynamics.cycleno == last_processed_cycleno_) {
            NODELET_WARN_THROTTLE(2.0, "Skipping duplicate DynamicsInfo cycle number: %lld",
                                  static_cast<long long>(dynamics.cycleno));
            return;
        }

        // 3. dynamics timestamp는 EKF 입력 샘플의 시간 기준이다. 중복/역전 샘플은 재적분하지 않는다.
        if (current_time <= last_prediction_time_) {
            NODELET_WARN_THROTTLE(2.0, "Skipping non-monotonic DynamicsInfo timestamp: %.6f <= %.6f",
                                  current_time, last_prediction_time_);
            return;
        }

        // 4. motion input 계산: 속도는 후륜 휠 평균, yaw rate는 IMU z축 gyro를 사용한다.
        double v_left  = dynamics.Vhcl_RL_rotv * tire_radius_;
        double v_right = dynamics.Vhcl_RR_rotv * tire_radius_;
        double velocity_input = (v_left + v_right) / 2.0;
        double yaw_rate_input = dynamics.Sensor_Inertial_0_Omega_B_z;

        // 5. 차량 동역학 기반 EKF 공정 노이즈와 motion input 노이즈 동적 계산
        const double ax_raw = dynamics.Sensor_Inertial_0_Acc_B_x;

        // 가감속이 심할 때 속도 입력 노이즈 팽창
        double scale_vel = 1.0;
        if (std::abs(ax_raw) > 1.0) {
            scale_vel = std::min(5.0, 1.0 + (std::abs(ax_raw) - 1.0) * 1.5);
        }

        // 급선회 시 yaw 입력 및 yaw 상태 공정 노이즈 팽창
        double scale_yaw = 1.0;
        if (std::abs(yaw_rate_input) > 0.1) { // 0.1 rad/s (약 5.7 deg/s) 이상 선회 시
            scale_yaw = std::min(10.0, 1.0 + (std::abs(yaw_rate_input) - 0.1) * 8.0);
        }

        // 두 요인을 선형 누적으로 결합하여 단일 동적 스케일링 인자 도출 (극단적 팽창 방지를 위해 상한 12.0배 제한)
        double scale_dyn = std::min(12.0, scale_yaw + scale_vel - 1.0);

        Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_dyn = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        Q_dyn(X, X) = std::pow(q_pos_std_, 2) * scale_dyn;
        Q_dyn(Y, Y) = std::pow(q_pos_std_, 2) * scale_dyn;
        Q_dyn(YAW, YAW) = std::pow(q_yaw_std_ * scale_yaw, 2);

        ekf_core_->setProcessNoise(Q_dyn);

        Eigen::Matrix2d R_input = Eigen::Matrix2d::Zero();
        R_input(0, 0) = std::pow(wheel_speed_std_, 2) * scale_vel;
        R_input(1, 1) = std::pow(imu_gyro_std_, 2) * scale_yaw;

        // 6. EKF 예측: 3D pose state를 motion input으로 전파
        ekf_core_->prediction(current_time, velocity_input, yaw_rate_input, R_input);
        latest_motion_velocity_ = velocity_input;
        latest_motion_yaw_rate_ = yaw_rate_input;
        last_processed_cycleno_ = dynamics.cycleno;

        // 디버깅용 입력/참고 분산 발행
        std_msgs::Float64 debug_wheel_vx_msg;
        debug_wheel_vx_msg.data = R_input(0, 0);
        debug_r_wheel_vx_pub_.publish(debug_wheel_vx_msg);

        std_msgs::Float64 debug_wheel_yaw_rate_msg;
        debug_wheel_yaw_rate_msg.data = 2.0 * std::pow(wheel_speed_std_ / track_width_, 2) * scale_yaw;
        debug_r_wheel_yaw_rate_pub_.publish(debug_wheel_yaw_rate_msg);

        std_msgs::Float64 debug_imu_msg;
        debug_imu_msg.data = R_input(1, 1);
        debug_r_imu_yaw_rate_pub_.publish(debug_imu_msg);

        // 7. 누적 RMSE 계산 및 순간 오차 발행(GT 후륜축 vs EKF 후륜축 상태)
        calculateAndPublishErrors(dynamics);

        // 8. Publish EKF 예측 결과 및 디버깅 데이터
        publishEstimation(ros::Time(current_time));
        last_prediction_time_ = current_time;
    }
}

void LocalizationNodelet::imagesCallback(
        const sensor_msgs::ImageConstPtr& img0,
        const sensor_msgs::ImageConstPtr& img1,
        const sensor_msgs::ImageConstPtr& img2,
        const sensor_msgs::ImageConstPtr& img3) {

    std::array<sensor_msgs::ImageConstPtr, 4> imgs = {img0, img1, img2, img3};

    // CameraInfo를 스레드 안전하게 복사
    std::array<sensor_msgs::CameraInfoConstPtr, 4> infos;
    {
        std::lock_guard<std::mutex> lock(info_array_mutex_);
        for (size_t i = 0; i < NUM_CAMERAS; ++i) {
            if (!latest_infos_[i]) {
                ROS_WARN_THROTTLE(2.0, "Waiting for CameraInfo on channel %zu...", i);
                return;
            }
            infos[i] = latest_infos_[i];
        }
    }

    processImages(imgs, infos);
}

void LocalizationNodelet::bundleCallback(const carmaker_msgs::CameraBundleConstPtr& msg) {
    std::array<sensor_msgs::ImageConstPtr, 4> imgs;
    std::array<sensor_msgs::CameraInfoConstPtr, 4> infos;
    if (!unpackBundle(msg, imgs, infos)) return;
    processImages(imgs, infos);
}

bool LocalizationNodelet::unpackBundle(
        const carmaker_msgs::CameraBundleConstPtr& msg,
        std::array<sensor_msgs::ImageConstPtr, 4>& imgs,
        std::array<sensor_msgs::CameraInfoConstPtr, 4>& infos) {

    if (msg->names.size() != 4 || msg->images.size() != 4 || msg->infos.size() != 4) {
        ROS_WARN_THROTTLE(2.0, "CameraBundle size mismatch. Expected 4, got names: %zu, images: %zu, infos: %zu",
                          msg->names.size(), msg->images.size(), msg->infos.size());
        return false;
    }

    // 알맞은 이미지 인덱스를 동적으로 탐색
    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
        const std::string& target_name = channels_[i].name;
        int idx = -1;
        for (size_t j = 0; j < msg->names.size(); ++j) {
            if (msg->names[j] == target_name) {
                idx = static_cast<int>(j);
                break;
            }
        }

        if (idx == -1 || idx >= static_cast<int>(msg->images.size()) || idx >= static_cast<int>(msg->infos.size())) {
            ROS_WARN_THROTTLE(2.0, "Required camera channel '%s' is missing or out of bounds in CameraBundle!", target_name.c_str());
            return false;
        }

        // boost::shared_ptr aliasing 생성자를 사용한 무복사 변환
        imgs[i]  = sensor_msgs::ImageConstPtr(msg, &msg->images[idx]);
        infos[i] = sensor_msgs::CameraInfoConstPtr(msg, &msg->infos[idx]);
    }

    return true;
}

void LocalizationNodelet::diagTimerCallback(const ros::WallTimerEvent& event) {
    ros::Time now = ros::Time::now();
    if (!now.isZero() && !last_timer_time_.isZero()) {
        double diff = (now - last_timer_time_).toSec();
        if (diff < -1.0 || diff > 5.0) {
            NODELET_WARN("[Time Jump Detected in Timer] Diff: %.2f sec. Resetting diagnostics time...", diff);
            diagnostic_updater_->force_update();
        }
    }
    last_timer_time_ = now;
    if (diagnostic_updater_) {
        diagnostic_updater_->update();
    }
}

void LocalizationNodelet::processImages(
        const std::array<sensor_msgs::ImageConstPtr, 4>& imgs,
        const std::array<sensor_msgs::CameraInfoConstPtr, 4>& infos) {

    carmaker_msgs::LocalFeatures combined;
    combined.header = imgs[0]->header;
    combined.camera_name = "combined";

    const ros::Time& img_stamp = imgs[0]->header.stamp;
    constexpr double MAX_CAMERA_INFO_SLOP = 0.1; // Maximum acceptable slop of 100ms

    bool all_lut_ready = true;

    // 1. 입출력 대기를 줄이기 위해 잠금 외부에서 TF를 먼저 조회하고, 초기화 시점 타임스탬프 오차를 진단
    std::array<geometry_msgs::TransformStamped, NUM_CAMERAS> tfs;
    std::array<bool, NUM_CAMERAS> tf_success = {false, false, false, false};

    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
        Channel& ch = channels_[i];

        if (!ch.lut_initialized.load(std::memory_order_acquire)) {
            // 오직 초기화되지 않은 상태에서 최초 LUT 빌드 시에만 시간 동기화 오차 가드 적용
            if (!use_bundle_) {
                double info_slop = std::abs((infos[i]->header.stamp - img_stamp).toSec());
                if (info_slop > MAX_CAMERA_INFO_SLOP) {
                    ROS_WARN_THROTTLE(5.0,
                        "[%s] CameraInfo timestamp slop too high! (Img: %.3f, Info: %.3f, Slop: %.3f s) during initialization. Retrying on next frame...",
                        ch.name.c_str(), img_stamp.toSec(), infos[i]->header.stamp.toSec(), info_slop);
                    all_lut_ready = false;
                    continue; // 슬롭이 너무 크면 기하 정보가 깨진 맵이 캐싱되는 것을 막기 위해 이번 턴의 초기화는 스킵하고 대기합니다.
                }
            }

            try {
                // 잠금 외부에서 TF 조회 수행(base_footprint -> 카메라 광학 좌표계 변환)
                tfs[i] = tf_buffer_->lookupTransform(infos[i]->header.frame_id, "base_footprint", ros::Time(0));
                tf_success[i] = true;
            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(2.0, "TF lookup failed for %s: %s", infos[i]->header.frame_id.c_str(), ex.what());
                all_lut_ready = false;
                continue; // 예외 발생 시 해당 채널 처리를 즉시 스킵하고 다음 채널로 진행
            }
        }
    }

    // 2. 확보된 TF로 잠금 내부에서 LUT를 빠르게 업데이트
    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
        Channel& ch = channels_[i];
        if (!ch.lut_initialized.load(std::memory_order_acquire) && tf_success[i]) {
            std::lock_guard<std::mutex> lock(*ch.lut_mutex);
            if (!ch.lut_initialized.load(std::memory_order_relaxed)) {
                std::vector<double> K(infos[i]->K.begin(), infos[i]->K.end());
                std::vector<double> D(infos[i]->D.begin(), infos[i]->D.end());
                std::string distortion_model = infos[i]->distortion_model;

                tf2::Transform tf_base2cam;
                tf2::fromMsg(tfs[i].transform, tf_base2cam);
                cv::Mat R_base_cam = cv::Mat::eye(3, 3, CV_64F);
                cv::Mat t_base_cam = cv::Mat::zeros(3, 1, CV_64F);
                tf2::Matrix3x3 R_tf = tf_base2cam.getBasis();
                tf2::Vector3 t_tf = tf_base2cam.getOrigin();
                for (int r = 0; r < 3; ++r) {
                    t_base_cam.at<double>(r, 0) = t_tf[r];
                    for (int c = 0; c < 3; ++c) {
                        R_base_cam.at<double>(r, c) = R_tf[r][c];
                    }
                }

                if (ch.extractor->updateLUT(K, D, distortion_model, R_base_cam, t_base_cam)) {
                    ch.lut_initialized.store(true, std::memory_order_release);
                    NODELET_INFO("LUT initialized for %s", ch.name.c_str());
                } else {
                    NODELET_ERROR("Failed to initialize LUT for %s. (OpenCV projection or calibration parameters are invalid)", ch.name.c_str());
                }
            }
        }
    }

    if (!all_lut_ready) return;

    // 3. OpenMP 병렬 비전 처리(다중 스레드 추출로 CPU 사용률 극대화)
    // 4개 채널 처리를 동시에 병렬 수행하여 지연 시간 단축
    std::array<carmaker_msgs::LocalFeatures, NUM_CAMERAS> extracted_features;
    std::array<cv::Mat, NUM_CAMERAS> bev_images;
    std::array<cv::Mat, NUM_CAMERAS> vis_images;

    #pragma omp parallel for schedule(static, 1)
    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
        cv_bridge::CvImagePtr cv_seg;
        try {
            std::string target_encoding = imgs[i]->encoding;
            if (target_encoding == "bgr8") {
                target_encoding = "rgb8";
            } else if (target_encoding == "bgra8") {
                target_encoding = "rgba8";
            }
            cv_seg = cv_bridge::toCvCopy(imgs[i], target_encoding);
        } catch (cv_bridge::Exception& e) {
            NODELET_ERROR("cv_bridge exception: %s", e.what());
            continue;
        }

        std::vector<LocalFeature> local_feats = channels_[i].extractor->process(cv_seg->image, bev_images[i]);
        vis_images[i] = channels_[i].extractor->processVisualization(cv_seg->image);

        carmaker_msgs::LocalFeatures features;
        features.header = imgs[i]->header;
        features.camera_name = channels_[i].name;
        features.features.reserve(local_feats.size());
        for (const auto& lf : local_feats) {
            carmaker_msgs::LocalFeature f_msg;
            f_msg.x = lf.x;
            f_msg.y = lf.y;
            f_msg.cov_xx = lf.cov_xx;
            f_msg.cov_xy = lf.cov_xy;
            f_msg.cov_yy = lf.cov_yy;
            f_msg.class_id = lf.class_id;
            features.features.push_back(f_msg);
        }
        extracted_features[i] = features;
    }

    // 4. 병합 및 후처리(공유 캔버스 업데이트는 mutex로 순차 보호)
    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
        Channel& ch = channels_[i];
        const cv::Mat& bev_image = bev_images[i];
        const auto& features = extracted_features[i];

        if (!bev_image.empty() && svm_masks_.count(ch.name) > 0) {
            std::lock_guard<std::mutex> lock(svm_mutex_);
            if (svm_canvas_.empty()) {
                int h = std::ceil((svm_x_max_ - svm_x_min_) / svm_res_);
                int w = std::ceil((svm_y_max_ - svm_y_min_) / svm_res_);
                svm_canvas_ = cv::Mat::zeros(h, w, bev_image.type());
            }

            double roi_x_max = ch.bev_x_range[1];
            double roi_y_max = ch.bev_y_range[1];
            int r_off = std::round((svm_x_max_ - roi_x_max) / svm_res_);
            int c_off = std::round((svm_y_max_ - roi_y_max) / svm_res_);

            // bev_image와 svm_canvas_ 간의 물리적 좌표 축(X-Y) 정의 엇갈림 정렬:
            // - bev_image: 행=Y축, 열=X축(가로세로 축이 물리계와 반대)
            // - svm_canvas_: 행=X축, 열=Y축(물리계 축 표준 정렬)
            // - 따라서 전치 후 수직 대칭하여 캔버스 축과 맞춘다
            cv::Mat transposed = bev_image.t();
            cv::Mat stitched_image;
            cv::flip(transposed, stitched_image, 0);

            if (visualizer_) {
                visualizer_->publishBevImage(ch.name, stitched_image, "cropped");
            }

            int h = stitched_image.rows;
            int w = stitched_image.cols;

            if (r_off >= 0 && c_off >= 0 && r_off + h <= svm_canvas_.rows && c_off + w <= svm_canvas_.cols) {
                cv::Mat global_mask = svm_masks_[ch.name];
                cv::Mat roi_mask = global_mask(cv::Rect(c_off, r_off, w, h));
                cv::Mat canvas_roi = svm_canvas_(cv::Rect(c_off, r_off, w, h));
                stitched_image.copyTo(canvas_roi, roi_mask);
            } else {
                ROS_WARN_THROTTLE(1.0, "[SVM DEBUG] %s out of bounds: r_off=%d, c_off=%d, w=%d, h=%d, canvas=%dx%d", ch.name.c_str(), r_off, c_off, w, h, svm_canvas_.cols, svm_canvas_.rows);
            }
        }

        const cv::Mat& vis_image = vis_images[i];
        if (!vis_image.empty() && visualizer_) {
            visualizer_->publishBevImage(ch.name, vis_image, "full");
        }

        if (!features.features.empty()) {
            // 데이터 분석용 발행
            feature_data_pubs_[ch.name].publish(features);

            carmaker_msgs::LocalFeatures viz_features = features;
            viz_features.header.frame_id = prediction_bumper_frame_;
            viz_features.header.stamp = features.header.stamp;
            visualizer_->publishObservation(ch.name, viz_features);
            ch.processed_count++;

            if (fusion_) {
                combined.features.insert(combined.features.end(), features.features.begin(), features.features.end());
            } else if (feature_registration_enabled_ && landmark_loader_ && registration_engine_) {
                std::lock_guard<std::mutex> lock(correction_queue_mutex_);
                if (correction_queue_.size() < 1) {
                    correction_queue_.push(features);
                    correction_queue_cv_.notify_one();
                } else {
                    ROS_DEBUG_THROTTLE(2.0, "Correction queue busy. Dropping features from camera: %s", features.camera_name.c_str());
                }
            }
        } else {
            // 특징점이 비어있더라도 publish_metrics_always_가 true라면 빈 특징점 메시지를 큐에 넣어 메트릭 발행을 유도
            if (!fusion_ && feature_registration_enabled_ && landmark_loader_ && registration_engine_ && publish_metrics_always_) {
                std::lock_guard<std::mutex> lock(correction_queue_mutex_);
                if (correction_queue_.size() < 1) {
                    correction_queue_.push(features);
                    correction_queue_cv_.notify_one();
                }
            }
        }
    }

    // 시각 검증을 위해 visualizer가 활성화되어 있으면 심라인 발행
    if (visualizer_) {
        visualizer_->publishSvmImage(svm_canvas_, seam_line_points_);
    }

    if (fusion_ && feature_registration_enabled_ && landmark_loader_ && registration_engine_) {
        if (!combined.features.empty() || publish_metrics_always_) {
            std::lock_guard<std::mutex> lock(correction_queue_mutex_);
            if (correction_queue_.size() < 1) {
                correction_queue_.push(combined);
                correction_queue_cv_.notify_one();
            } else {
                ROS_DEBUG_THROTTLE(2.0, "Correction queue busy. Dropping combined features.");
            }
        }
    }
}

void LocalizationNodelet::performCorrection(const carmaker_msgs::LocalFeatures& features) {
    ros::Time metric_stamp = ros::Time::now();

    if (features.features.empty()) {
        ROS_WARN_THROTTLE(2.0, "performCorrection: Input features are empty!");
        if (publish_metrics_always_) {
            RegistrationResult dummy_result;
            dummy_result.success = false;
            dummy_result.num_observed = 0;
            dummy_result.num_landmarks = landmark_loader_ ? landmark_loader_->queryNear(0.0, 0.0, -1.0).size() : 0;
            dummy_result.fitness_score = std::numeric_limits<double>::quiet_NaN();
            publishIcpMetrics(metric_stamp, dummy_result);
        }
        return;
    }

    Eigen::Isometry2d initial_guess = Eigen::Isometry2d::Identity();
    std::vector<LandmarkFeature> landmarks;
    double query_x = 0.0;
    double query_y = 0.0;

    // 고주기 예측 루프와의 경합을 막기 위해 EKF 상태를 스레드 안전하게 복사
    {
        std::lock_guard<std::mutex> lock(estimation_mutex_);
        auto state_frame = ekf_core_->getState();
        const auto& ekf_pose = state_frame.x;
        // EKF 상태는 후륜축 기준. ICP는 base_footprint(범퍼) 기준이므로 초기추정치 변환
        Eigen::Vector3d pose_rear(ekf_pose(X), ekf_pose(Y), ekf_pose(YAW));
        Eigen::Vector3d pose_bumper = transformPose(pose_rear, -rear_axle_x_);
        initial_guess.translation() = Eigen::Vector2d(pose_bumper(0), pose_bumper(1));
        initial_guess.linear() = Eigen::Rotation2Dd(pose_bumper(2)).toRotationMatrix();
        query_x = ekf_pose(X);
        query_y = ekf_pose(Y);
    }

    landmarks = landmark_loader_->queryNear(query_x, query_y, search_radius_);

    if (landmarks.empty()) {
        if (search_radius_ < 0.0) {
            ROS_WARN_THROTTLE(2.0, "performCorrection: No landmarks found in the entire OSM landmark file!");
        } else {
            ROS_WARN_THROTTLE(2.0, "performCorrection: No landmarks found within search_radius (%.1f m) of pose (%.2f, %.2f)!", search_radius_, query_x, query_y);
        }
        if (publish_metrics_always_) {
            RegistrationResult dummy_result;
            dummy_result.success = false;
            dummy_result.num_observed = features.features.size();
            dummy_result.num_landmarks = 0;
            dummy_result.fitness_score = std::numeric_limits<double>::quiet_NaN();
            publishIcpMetrics(metric_stamp, dummy_result);
        }
        return;
    }

    // 정합 엔진을 ROS와 분리하기 위해 ROS 특징점 메시지를 순수 C++ 특징점으로 변환
    std::vector<LocalFeature> cpp_features;
    cpp_features.reserve(features.features.size());
    for (const auto& f : features.features) {
        LocalFeature cpp_feat;
        cpp_feat.x = f.x;
        cpp_feat.y = f.y;
        cpp_feat.cov_xx = f.cov_xx;
        cpp_feat.cov_yy = f.cov_yy;
        cpp_feat.cov_xy = f.cov_xy;
        cpp_feat.class_id = f.class_id;
        cpp_features.push_back(cpp_feat);
    }

    // ICP 특징점 정합(100Hz EKF 루프가 막히지 않도록 mutex 해제 후 수행)
    bool publish_visual_debug = visualizer_ && visualizer_->hasIcpSubscribers();
    bool publish_metrics = publish_metrics_always_ || icp_metrics_pub_.getNumSubscribers() > 0;
    bool collect_debug = publish_visual_debug || publish_metrics;
    auto registration_result = registration_engine_->align(cpp_features, landmarks, initial_guess, collect_debug, publish_visual_debug);
    metric_stamp = ros::Time::now();

    if (visualizer_ && publish_visual_debug) {
        visualizer_->publishAssociationMarkers(global_frame_, features.header.stamp, registration_result);
    }

    if (registration_result.success) {
        Eigen::Vector3d z;
        z << registration_result.transform.translation().x(),
                registration_result.transform.translation().y(),
                Eigen::Rotation2Dd(registration_result.transform.linear()).angle();

        // 뮤텍스 락 아래 스레드 안전한 EKF 보정 단계 수행
        Eigen::Matrix3d R_reg = registration_result.covariance;
        double current_time = metric_stamp.toSec();
        double img_time = features.header.stamp.toSec();
        double dt = current_time - img_time;

        if (dt < 0.0 || dt >= 1.5) {
            ROS_WARN_THROTTLE(2.0, "performCorrection: Measurement latency too high (%.3f s) or negative. Skipping EKF correction.", dt);
            if (publish_metrics) {
                publishIcpMetrics(metric_stamp, registration_result);
            }
            return;
        }

        Eigen::Matrix3d R_reg_rear = Eigen::Matrix3d::Zero();

        {
            std::lock_guard<std::mutex> lock(estimation_mutex_);

            auto state_frame = ekf_core_->getState();
            current_time = state_frame.timestamp;
            dt = current_time - img_time;

            if (dt < 0.0 || dt >= 1.5) {
                ROS_WARN_THROTTLE(2.0, "performCorrection: Measurement latency too high (%.3f s) or negative. Skipping EKF correction.", dt);
                return;
            }

            const auto& current_state = state_frame.x;
            double v = latest_motion_velocity_;
            double yaw_rate = latest_motion_yaw_rate_;

            // ICP 결과(z)는 base_footprint(범퍼) 기준 → 후륜축으로 변환
            z = transformPose(z, rear_axle_x_);
            const double yaw_z = z(2);

            // 관측 공분산 R_reg 역시 범퍼에서 후륜축으로 전파
            R_reg_rear = propagateCovariance(R_reg, yaw_z, rear_axle_x_);

            // 지연 시간(dt) 동안의 기구학 모델 불확실성을 반영한 공분산 팽창
            // 차체 로컬 오차 공분산을 글로벌 월드 좌표계에 맞게 회전 변환하여 더함
            if (dt > 0.0) {
                double cos_z = std::cos(yaw_z);
                double sin_z = std::sin(yaw_z);
                double var_long = std::pow(0.15, 2) * dt;
                double var_lat  = std::pow(0.10, 2) * dt;
                double var_yaw  = std::pow(0.03, 2) * dt;

                Eigen::Matrix3d Q_latency_global = Eigen::Matrix3d::Zero();
                Q_latency_global(0, 0) = cos_z * cos_z * var_long + sin_z * sin_z * var_lat;
                Q_latency_global(0, 1) = cos_z * sin_z * (var_long - var_lat);
                Q_latency_global(1, 0) = Q_latency_global(0, 1);
                Q_latency_global(1, 1) = sin_z * sin_z * var_long + cos_z * cos_z * var_lat;
                Q_latency_global(2, 2) = var_yaw;

                R_reg_rear += Q_latency_global;
            }

            // 후륜축 기준 기구학 자전거 모델로 전방 지연 시간 선도 보상
            z(0) += v * std::cos(z(2)) * dt;
            z(1) += v * std::sin(z(2)) * dt;
            z(2) += yaw_rate * dt;

            // 요각 정규화
            while (z(2) > M_PI) z(2) -= 2.0 * M_PI;
            while (z(2) < -M_PI) z(2) += 2.0 * M_PI;

            // EKF 상태와 지연 보상된 정합 결과의 편차 검증
            // 급격한 점프 및 오정합으로 인한 필터 오염 방지
            double dx = z(0) - current_state(X);
            double dy = z(1) - current_state(Y);
            double dist = std::hypot(dx, dy);

            double dyaw = z(2) - current_state(YAW);
            while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

            // 최대 허용 편차 검증(설정 파일 기반 검증 게이트)
            // ICP 관측값의 신뢰도(fitness_score가 낮을수록 높음)가 높을수록
            // 검증 게이트(max_position_dev_) 범위를 동적으로 확장하여 누적된 큰 드리프트를 한번에 구제 및 복구
            // EKF 상태의 불확실성(P)에 따른 3-sigma 오차 한계선 계산
            const auto& P_current = state_frame.P;
            double ekf_uncertainty_3sigma = 3.0 * std::sqrt(P_current(X, X) + P_current(Y, Y));
            double ekf_yaw_uncertainty_3sigma = 3.0 * std::sqrt(P_current(YAW, YAW));

            double adaptive_max_position_dev = max_position_dev_;
            if (registration_result.fitness_score < fitness_threshold_ && fitness_threshold_ > 1e-6) {
                double fitness_margin = (fitness_threshold_ - registration_result.fitness_score) / fitness_threshold_;
                adaptive_max_position_dev *= (1.0 + 4.0 * fitness_margin); // 신뢰도가 높을수록 최대 5배까지 완화 (예: 1.0m -> 5.0m)
            }

            // EKF 불확실성에 비례하여 검증 게이트를 탄력적으로 확장하여 재수렴 지연 방지
            adaptive_max_position_dev = std::max(adaptive_max_position_dev, ekf_uncertainty_3sigma);
            double adaptive_max_yaw_dev = std::max(max_yaw_dev_, ekf_yaw_uncertainty_3sigma);

            if (dist > adaptive_max_position_dev || std::abs(dyaw) > adaptive_max_yaw_dev) {
                ROS_WARN_THROTTLE(2.0, "performCorrection: Match rejected by validation gate. Dev: %.3fm, %.1f deg (limits: %.2fm / adaptive: %.2fm, %.1f deg / yaw_limit: %.1f deg). Skipping correction.",
                                  dist, std::abs(dyaw) * 180.0 / M_PI, max_position_dev_, adaptive_max_position_dev, max_yaw_dev_ * 180.0 / M_PI, adaptive_max_yaw_dev * 180.0 / M_PI);
                if (publish_metrics) {
                    publishIcpMetrics(metric_stamp, registration_result);
                }
                return;
            }

            // 후륜축 기준 pose 관측으로 EKF 상태 보정
            ekf_core_->correctPose(z(0), z(1), z(2), R_reg_rear, current_time, max_position_step_, max_yaw_step_);
        }

        // 보정 주파수 추적: 첫 보정 시각을 기록하고 횟수 누적
        {
            std::lock_guard<std::mutex> hz_lock(correction_hz_mutex_);
            if (correction_count_ == 0)
                correction_start_sim_time_ = features.header.stamp.toSec();
            correction_count_++;
        }

        if (publish_metrics) {
            publishIcpMetrics(metric_stamp, registration_result);
        }

        // 최종 로그 및 디버그 시각화(잠금 없이 수행)
        geometry_msgs::PoseWithCovarianceStamped correction_msg;
        correction_msg.header.stamp = ros::Time(current_time);
        correction_msg.header.frame_id = global_frame_;
        // correction_msg: 후륜축 기준 z를 그대로 발행
        correction_msg.pose.pose.position.x = z(0);
        correction_msg.pose.pose.position.y = z(1);
        tf2::Quaternion q;
        q.setRPY(0, 0, z(2));
        correction_msg.pose.pose.orientation = tf2::toMsg(q);

        // 후륜축 기준 공분산 R_reg_rear를 그대로 퍼블리시용 공분산으로 저장
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                int idx = (r == 2 ? 5 : r) * 6 + (c == 2 ? 5 : c);
                correction_msg.pose.covariance[idx] = R_reg_rear(r, c);
            }
        }
        correction_data_pub_.publish(correction_msg);
        visualizer_->publishCorrection(correction_msg);
        ROS_INFO_THROTTLE(2.0, "performCorrection: ICP Registration SUCCESS! Fitness score: %.3f (threshold: %.3f). Observed: %zu, Ref: %zu", registration_result.fitness_score, fitness_threshold_, registration_result.num_observed, registration_result.num_landmarks);
    } else {
        if (publish_metrics) {
            publishIcpMetrics(metric_stamp, registration_result);
        }
        visualizer_->clearCorrection();
        ROS_WARN_THROTTLE(2.0, "performCorrection: ICP Registration FAILED. Fitness score: %.3f (threshold: %.3f). Observed: %zu, Ref: %zu", registration_result.fitness_score, fitness_threshold_, registration_result.num_observed, registration_result.num_landmarks);
    }
}

void LocalizationNodelet::publishIcpMetrics(const ros::Time& stamp, const RegistrationResult& result) {
    carmaker_msgs::IcpRegistrationMetrics msg;
    const double no_data = std::numeric_limits<double>::quiet_NaN();
    msg.header.stamp = stamp;
    msg.header.frame_id = global_frame_;

    msg.grid_i = 0;
    msg.grid_j = 0;
    msg.cell_x = no_data;
    msg.cell_y = no_data;
    msg.observed_count = static_cast<uint32_t>(result.num_observed);
    msg.landmark_count = static_cast<uint32_t>(result.num_landmarks);

    msg.success = result.success;
    msg.attempt_count = 1;
    msg.success_count = result.success ? 1 : 0;
    msg.success_rate = result.success ? 1.0 : 0.0;
    msg.fitness_score = result.fitness_score;
    msg.icp_rear_x = no_data;
    msg.icp_rear_y = no_data;
    msg.icp_rear_yaw = no_data;

    if (result.debug) {
        msg.iterations = static_cast<uint32_t>(std::max(0, result.debug->iterations));
        msg.latency_ms = result.debug->latency_ms;
    }

    msg.longitudinal_rmse = no_data;
    msg.lateral_rmse = no_data;
    msg.yaw_rmse = no_data;

    msg.cov_xx = no_data;
    msg.cov_xy = no_data;
    msg.cov_xyaw = no_data;
    msg.cov_yx = no_data;
    msg.cov_yy = no_data;
    msg.cov_yyaw = no_data;
    msg.cov_yawx = no_data;
    msg.cov_yawy = no_data;
    msg.cov_yawyaw = no_data;
    msg.cov_trace = no_data;
    msg.cov_determinant = no_data;

    if (result.success) {
        const Eigen::Vector3d icp_bumper(
            result.transform.translation().x(),
            result.transform.translation().y(),
            Eigen::Rotation2Dd(result.transform.linear()).angle());
        const Eigen::Vector3d icp_rear = transformPose(icp_bumper, rear_axle_x_);
        msg.icp_rear_x = icp_rear(0);
        msg.icp_rear_y = icp_rear(1);
        msg.icp_rear_yaw = icp_rear(2);
        const Eigen::Matrix3d cov = propagateCovariance(result.covariance, icp_bumper(2), rear_axle_x_);
        msg.cov_xx = cov(0, 0);
        msg.cov_xy = cov(0, 1);
        msg.cov_xyaw = cov(0, 2);
        msg.cov_yx = cov(1, 0);
        msg.cov_yy = cov(1, 1);
        msg.cov_yyaw = cov(1, 2);
        msg.cov_yawx = cov(2, 0);
        msg.cov_yawy = cov(2, 1);
        msg.cov_yawyaw = cov(2, 2);
        msg.cov_trace = cov.trace();
        msg.cov_determinant = cov.determinant();
    }

    icp_metrics_pub_.publish(msg);
}

void LocalizationNodelet::correctionWorkerLoop() {
    while (run_correction_worker_) {
        carmaker_msgs::LocalFeatures features;
        {
            std::unique_lock<std::mutex> lock(correction_queue_mutex_);
            correction_queue_cv_.wait(lock, [this]() {
                return !run_correction_worker_ || !correction_queue_.empty();
            });
            if (!run_correction_worker_) {
                break;
            }
            features = std::move(correction_queue_.front());
            correction_queue_.pop();
        }
        performCorrection(features);
    }
}
void LocalizationNodelet::publishEstimation(const ros::Time& stamp) {
    auto state_frame = ekf_core_->getState();
    const auto& x = state_frame.x;
    const auto& P = state_frame.P;

    const double yaw = x(YAW);

    // EKF 상태(X, Y)는 후륜축 기준 → 위치 출력 명세를 후륜축 기준으로 설정
    double pub_x = x(X);
    double pub_y = x(Y);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = global_frame_;
    pose_msg.pose.pose.position.x = pub_x;
    pose_msg.pose.pose.position.y = pub_y;
    pose_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose_msg.pose.pose.orientation = tf2::toMsg(q);

    // EKF 포즈 공분산(X, Y, YAW)을 후륜축 기준으로 직접 출력
    pose_msg.pose.covariance[0]  = P(X, X);     // XX
    pose_msg.pose.covariance[1]  = P(X, Y);     // XY
    pose_msg.pose.covariance[5]  = P(X, YAW);   // X-YAW
    pose_msg.pose.covariance[6]  = P(X, Y);     // YX
    pose_msg.pose.covariance[7]  = P(Y, Y);     // YY
    pose_msg.pose.covariance[11] = P(Y, YAW);   // Y-YAW
    pose_msg.pose.covariance[30] = P(X, YAW);   // YAW-X
    pose_msg.pose.covariance[31] = P(Y, YAW);   // YAW-Y
    pose_msg.pose.covariance[35] = P(YAW, YAW); // YAW-YAW

    pose_pub_.publish(pose_msg);
    estimation_data_pub_.publish(pose_msg);
    visualizer_->publishEstimation(pose_msg);

    // 속도 출력은 EKF 상태가 아니라 최신 motion input을 사용한다.
    nav_msgs::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = prediction_rear_axle_frame_; // 후륜축 예측 프레임
    odom_msg.pose = pose_msg.pose;
    odom_msg.twist.twist.linear.x = latest_motion_velocity_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.z = latest_motion_yaw_rate_;
    odom_pub_.publish(odom_msg);

    // 1. TF 발행: global -> prediction_rear_axle(후륜축, EKF 추정 상태 기준)
    geometry_msgs::TransformStamped tf_msg1;
    tf_msg1.header.stamp = stamp;
    tf_msg1.header.frame_id = global_frame_;
    tf_msg1.child_frame_id = prediction_rear_axle_frame_;
    tf_msg1.transform.translation.x = x(X);
    tf_msg1.transform.translation.y = x(Y);
    tf_msg1.transform.translation.z = 0.0;
    tf_msg1.transform.rotation = pose_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg1);

    // search_radius가 지역 탐색 모드(양수)이면 현재 EKF 추정 위치 기준으로 랜드마크를 다시 발행
    if (visualizer_ && search_radius_ >= 0.0) {
        double dx = x(X) - last_landmark_pub_x_;
        double dy = x(Y) - last_landmark_pub_y_;
        if (dx*dx + dy*dy > 1.0 * 1.0) {
            auto current_landmarks = landmark_loader_->queryNear(x(X), x(Y), search_radius_);
            visualizer_->publishLandmarkFeatures(current_landmarks);
            last_landmark_pub_x_ = x(X);
            last_landmark_pub_y_ = x(Y);
        }
    }
}

void LocalizationNodelet::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    std::lock_guard<std::mutex> lock(estimation_mutex_);

    if (!ekf_core_->isInitialized()) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "EKF is not initialized yet");
        return;
    }

    auto state_frame = ekf_core_->getState();
    const auto& x = state_frame.x;
    const auto& P = state_frame.P;

    double std_x = std::sqrt(std::max(0.0, P(X, X)));
    double std_y = std::sqrt(std::max(0.0, P(Y, Y)));
    double std_yaw = std::sqrt(std::max(0.0, P(YAW, YAW))) * 180.0 / M_PI; // 도 단위
    double pos_uncertainty = std::sqrt(std_x * std_x + std_y * std_y);

    if (pos_uncertainty < 0.15) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Localization is active and converged");
    } else if (pos_uncertainty < 0.40) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Localization uncertainty is rising");
    } else {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Localization is LOST (high uncertainty)");
    }

    // 중앙 모니터링을 위한 GT 값
    {
        std::lock_guard<std::mutex> dyn_lock(dyn_mutex_);
        if (dynamics_received_ && !std::isnan(latest_dynamics_.RearAxle_x) && !std::isnan(latest_dynamics_.RearAxle_y)) {
            stat.add("True X (m)", latest_dynamics_.RearAxle_x);
            stat.add("True Y (m)", latest_dynamics_.RearAxle_y);
            stat.add("True Yaw (deg)", latest_dynamics_.Car_Yaw * 180.0 / M_PI);
            stat.add("True Vx (m/s)", latest_dynamics_.Car_vx);
            stat.add("True Vy (m/s)", latest_dynamics_.Car_vy);
            stat.add("True Yaw Rate (deg/s)", latest_dynamics_.Car_YawVel * 180.0 / M_PI);
        } else {
            stat.add("True X (m)", "N/A");
            stat.add("True Y (m)", "N/A");
            stat.add("True Yaw (deg)", "N/A");
            stat.add("True Vx (m/s)", "N/A");
            stat.add("True Vy (m/s)", "N/A");
            stat.add("True Yaw Rate (deg/s)", "N/A");
        }
    }

    stat.add("Uncertainty Position X (m)", std_x);
    stat.add("Uncertainty Position Y (m)", std_y);
    stat.add("Uncertainty Yaw (deg)", std_yaw);
    stat.add("Position 2D Uncertainty (m)", pos_uncertainty);

    // 중앙 모니터링을 위한 현재 추정 상태 값
    stat.add("Estimated X (m)", x(X));
    stat.add("Estimated Y (m)", x(Y));
    stat.add("Estimated Yaw (deg)", x(YAW) * 180.0 / M_PI);
    stat.add("Input Velocity (m/s)", latest_motion_velocity_);
    stat.add("Input Yaw Rate (deg/s)", latest_motion_yaw_rate_ * 180.0 / M_PI);

    // 보정 주파수 = 성공 횟수 / (현재 시뮬 시각 - 첫 보정 시뮬 시각)
    {
        std::lock_guard<std::mutex> hz_lock(correction_hz_mutex_);
        double correction_hz = 0.0;
        if (correction_count_ > 0) {
            double elapsed = ros::Time::now().toSec() - correction_start_sim_time_;
            if (elapsed > 1e-6)
                correction_hz = static_cast<double>(correction_count_) / elapsed;
        }
        stat.add("Correction Hz (Hz)", correction_hz);
    }

    double local_inst_pos_err = 0.0;
    double local_inst_yaw_err = 0.0;
    double local_inst_longitudinal_err = 0.0;
    double local_inst_lateral_err = 0.0;
    double local_cumulative_pos_sq_err = 0.0;
    double local_cumulative_yaw_sq_err = 0.0;
    double local_cumulative_longitudinal_sq_err = 0.0;
    double local_cumulative_lateral_sq_err = 0.0;
    double local_nees_latest = 0.0;
    double local_cumulative_nees_sum = 0.0;
    uint64_t local_count = 0;
    {
        local_inst_pos_err = inst_pos_err_;
        local_inst_yaw_err = inst_yaw_err_;
        local_inst_longitudinal_err = inst_longitudinal_err_;
        local_inst_lateral_err = inst_lateral_err_;
        local_cumulative_pos_sq_err = cumulative_pos_sq_err_;
        local_cumulative_yaw_sq_err = cumulative_yaw_sq_err_;
        local_cumulative_longitudinal_sq_err = cumulative_longitudinal_sq_err_;
        local_cumulative_lateral_sq_err = cumulative_lateral_sq_err_;
        local_nees_latest = nees_latest_;
        local_cumulative_nees_sum = cumulative_nees_;
        local_count = err_count_;
    }

    if (local_count > 0) {
        stat.add("Latest Instantaneous Position Error (m)", local_inst_pos_err);
        stat.add("Latest Instantaneous Yaw Error (deg)", local_inst_yaw_err * 180.0 / M_PI);
        stat.add("Cumulative Position RMSE (m)", std::sqrt(local_cumulative_pos_sq_err / local_count));
        stat.add("Cumulative Yaw RMSE (deg)", std::sqrt(local_cumulative_yaw_sq_err / local_count) * 180.0 / M_PI);
        stat.add("Latest Instantaneous Longitudinal Error (m)", local_inst_longitudinal_err);
        stat.add("Latest Instantaneous Lateral Error (m)", local_inst_lateral_err);
        stat.add("Cumulative Longitudinal RMSE (m)", std::sqrt(local_cumulative_longitudinal_sq_err / local_count));
        stat.add("Cumulative Lateral RMSE (m)", std::sqrt(local_cumulative_lateral_sq_err / local_count));
        stat.add("Latest Instantaneous NEES", local_nees_latest);
        stat.add("Cumulative Average NEES", local_cumulative_nees_sum / local_count);
    } else {
        stat.add("Latest Instantaneous Position Error (m)", 0.0);
        stat.add("Latest Instantaneous Yaw Error (deg)", 0.0);
        stat.add("Cumulative Position RMSE (m)", 0.0);
        stat.add("Cumulative Yaw RMSE (deg)", 0.0);
        stat.add("Latest Instantaneous Longitudinal Error (m)", 0.0);
        stat.add("Latest Instantaneous Lateral Error (m)", 0.0);
        stat.add("Cumulative Longitudinal RMSE (m)", 0.0);
        stat.add("Cumulative Lateral RMSE (m)", 0.0);
        stat.add("Latest Instantaneous NEES", 0.0);
        stat.add("Cumulative Average NEES", 0.0);
    }
}

void LocalizationNodelet::resetLocalization() {
    NODELET_INFO("Resetting Localization pipeline ...");

    if (visualizer_) {
        visualizer_->reset();
    }

    for (auto& ch : channels_) {
        ch.lut_initialized.store(false, std::memory_order_release);
    }

    {
        std::lock_guard<std::mutex> lock(dyn_mutex_);
        dynamics_received_ = false;
    }

    {
        std::lock_guard<std::mutex> lock(estimation_mutex_);
        last_prediction_time_ = 0.0;
        last_processed_cycleno_ = -1;
        latest_motion_velocity_ = 0.0;
        latest_motion_yaw_rate_ = 0.0;
        inst_pos_err_ = 0.0;
        inst_yaw_err_ = 0.0;
        inst_longitudinal_err_ = 0.0;
        inst_lateral_err_ = 0.0;
        cumulative_pos_sq_err_ = 0.0;
        cumulative_yaw_sq_err_ = 0.0;
        cumulative_longitudinal_sq_err_ = 0.0;
        cumulative_lateral_sq_err_ = 0.0;
        cumulative_nees_ = 0.0;
        nees_latest_ = 0.0;
        err_count_ = 0;
        last_landmark_pub_x_ = -9999.0;
        last_landmark_pub_y_ = -9999.0;
    }

    // 보정 주파수 추적 초기화(리셋 전 데이터가 주파수 추정에 섞이는 것 방지)
    {
        std::lock_guard<std::mutex> hz_lock(correction_hz_mutex_);
        correction_count_ = 0;
        correction_start_sim_time_ = -1.0;
    }

    if (tf_buffer_) {
        tf_buffer_->clear();
    }

    if (!use_bundle_ && sync_all_) {
        sync_all_.reset(new message_filters::Synchronizer<SyncPolicy4>(
            SyncPolicy4(10),
            *seg_subs_[0], *seg_subs_[1], *seg_subs_[2], *seg_subs_[3]));
        sync_all_->registerCallback(boost::bind(&LocalizationNodelet::imagesCallback, this, _1, _2, _3, _4));
    }
}

void LocalizationNodelet::initLocalization(double current_time, const carmaker_msgs::DynamicsInfo& current_dynamics) {
    std::lock_guard<std::mutex> lock(estimation_mutex_);

    // 다중 스레드 환경에서 경합을 막기 위한 이중 확인
    if (last_prediction_time_ > 0.0) {
        return;
    }

    last_prediction_time_ = current_time;
    double init_yaw = use_manual_initial_state_ ? init_yaw_ : current_dynamics.Car_Yaw;

    // 수동 및 GT 초기 포즈는 모두 후륜축 기준
    double init_x = use_manual_initial_state_ ? init_x_ : current_dynamics.RearAxle_x;
    double init_y = use_manual_initial_state_ ? init_y_ : current_dynamics.RearAxle_y;

    latest_motion_velocity_ = 0.0;
    latest_motion_yaw_rate_ = 0.0;
    last_processed_cycleno_ = current_dynamics.cycleno;

    ekf_core_->initialize(init_x, init_y, init_yaw, current_time);
    NODELET_INFO("EKF Initialized at [%.2f, %.2f, %.2f deg]", init_x, init_y, init_yaw * 180.0 / M_PI);

    // 정적 TF 발행: prediction_rear_axle(후륜축) -> prediction_bumper(범퍼)
    geometry_msgs::TransformStamped static_tf;
    static_tf.header.stamp = ros::Time::now();
    static_tf.header.frame_id = prediction_rear_axle_frame_;
    static_tf.child_frame_id = prediction_bumper_frame_;
    static_tf.transform.translation.x = -rear_axle_x_;
    static_tf.transform.translation.y = 0.0;
    static_tf.transform.translation.z = 0.0;
    static_tf.transform.rotation.x = 0.0;
    static_tf.transform.rotation.y = 0.0;
    static_tf.transform.rotation.z = 0.0;
    static_tf.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(static_tf);

    if (visualizer_) {
        visualizer_->publishLandmarkFeatures(landmark_loader_->queryNear(init_x, init_y, search_radius_));
        last_landmark_pub_x_ = init_x;
        last_landmark_pub_y_ = init_y;
    }
}

bool LocalizationNodelet::getMapCallback(nav_msgs::GetMap::Request&, nav_msgs::GetMap::Response& res) {
    if (!landmark_loader_) {
        NODELET_WARN("Landmark loader is not initialized, cannot provide occupancy grid.");
        return false;
    }
    res.map = landmark_loader_->getOccupancyGrid();
    if (res.map.data.empty()) {
        NODELET_WARN("OccupancyGrid map data is empty.");
        return false;
    }
    return true;
}

Eigen::Vector3d LocalizationNodelet::transformPose(const Eigen::Vector3d& pose_in, double offset_x) const {
    double cos_yaw = std::cos(pose_in(2));
    double sin_yaw = std::sin(pose_in(2));
    Eigen::Vector3d pose_out;
    pose_out(0) = pose_in(0) + offset_x * cos_yaw;
    pose_out(1) = pose_in(1) + offset_x * sin_yaw;
    pose_out(2) = pose_in(2);
    return pose_out;
}

Eigen::Matrix3d LocalizationNodelet::propagateCovariance(const Eigen::Matrix3d& cov_in, double yaw, double offset_x) const {
    Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
    J(0, 2) = -offset_x * std::sin(yaw);
    J(1, 2) =  offset_x * std::cos(yaw);
    return J * cov_in * J.transpose();
}

void LocalizationNodelet::calculateAndPublishErrors(const carmaker_msgs::DynamicsInfo& dynamics) {
    auto state_frame = ekf_core_->getState();
    const auto& x_state = state_frame.x;

    if (std::isnan(dynamics.RearAxle_x) || std::isnan(dynamics.RearAxle_y)) {
        return;
    }

    double dx = dynamics.RearAxle_x - x_state(X);
    double dy = dynamics.RearAxle_y - x_state(Y);
    double pos_err = std::hypot(dx, dy);
    double cos_yaw = std::cos(dynamics.Car_Yaw);
    double sin_yaw = std::sin(dynamics.Car_Yaw);
    double longitudinal_err = cos_yaw * dx + sin_yaw * dy;
    double lateral_err = -sin_yaw * dx + cos_yaw * dy;

    double yaw_err = dynamics.Car_Yaw - x_state(YAW);
    while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
    double yaw_err_abs = std::abs(yaw_err);

    // 3차원 포즈 오차 벡터(X, Y, YAW)
    Eigen::Vector3d e;
    e(0) = dx;
    e(1) = dy;
    e(2) = yaw_err;

    // 3x3 포즈 공분산 부분 행렬
    const auto& P = state_frame.P;
    Eigen::Matrix3d P_pose = Eigen::Matrix3d::Zero();
    P_pose(0, 0) = P(X, X);     P_pose(0, 1) = P(X, Y);     P_pose(0, 2) = P(X, YAW);
    P_pose(1, 0) = P(Y, X);     P_pose(1, 1) = P(Y, Y);     P_pose(1, 2) = P(Y, YAW);
    P_pose(2, 0) = P(YAW, X);   P_pose(2, 1) = P(YAW, Y);   P_pose(2, 2) = P(YAW, YAW);

    // 양의 정부호성을 보장하기 위한 정규화
    P_pose += Eigen::Matrix3d::Identity() * 1e-9;

    double nees = e.transpose() * P_pose.inverse() * e;

    inst_pos_err_ = pos_err;
    inst_yaw_err_ = yaw_err_abs;
    inst_longitudinal_err_ = longitudinal_err;
    inst_lateral_err_ = lateral_err;
    cumulative_pos_sq_err_ += (pos_err * pos_err);
    cumulative_yaw_sq_err_ += (yaw_err * yaw_err);
    cumulative_longitudinal_sq_err_ += (longitudinal_err * longitudinal_err);
    cumulative_lateral_sq_err_ += (lateral_err * lateral_err);
    cumulative_nees_ += nees;
    nees_latest_ = nees;
    err_count_++;

    // 토픽에는 실시간 오차, 누적 방향별 RMSE 및 NEES 발행
    std_msgs::Float64 pos_err_msg;
    pos_err_msg.data = pos_err;
    rmse_pos_pub_.publish(pos_err_msg);

    std_msgs::Float64 yaw_err_msg;
    yaw_err_msg.data = yaw_err_abs * 180.0 / M_PI; // radian -> degree 변환
    yaw_error_pub_.publish(yaw_err_msg);

    const double sample_count = static_cast<double>(err_count_);

    std_msgs::Float64 yaw_rmse_msg;
    yaw_rmse_msg.data = std::sqrt(cumulative_yaw_sq_err_ / sample_count) * 180.0 / M_PI;
    yaw_rmse_pub_.publish(yaw_rmse_msg);

    std_msgs::Float64 longitudinal_error_msg;
    longitudinal_error_msg.data = longitudinal_err;
    longitudinal_error_pub_.publish(longitudinal_error_msg);

    std_msgs::Float64 lateral_error_msg;
    lateral_error_msg.data = lateral_err;
    lateral_error_pub_.publish(lateral_error_msg);

    std_msgs::Float64 longitudinal_rmse_msg;
    longitudinal_rmse_msg.data = std::sqrt(cumulative_longitudinal_sq_err_ / sample_count);
    longitudinal_rmse_pub_.publish(longitudinal_rmse_msg);

    std_msgs::Float64 lateral_rmse_msg;
    lateral_rmse_msg.data = std::sqrt(cumulative_lateral_sq_err_ / sample_count);
    lateral_rmse_pub_.publish(lateral_rmse_msg);

    std_msgs::Float64 nees_msg;
    nees_msg.data = nees;
    nees_pub_.publish(nees_msg);
}



} // namespace carmaker_localization

PLUGINLIB_EXPORT_CLASS(carmaker_localization::LocalizationNodelet, nodelet::Nodelet)
