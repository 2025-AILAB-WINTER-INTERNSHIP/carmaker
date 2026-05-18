#include "carmaker_localization/localization_nodelet.h"
#include "carmaker_localization/osm_map_loader.h"
#include "carmaker_localization/icp_matcher.h"
#include <XmlRpcValue.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>

namespace carmaker_localization {

void LocalizationNodelet::onInit() {
    NODELET_INFO("Initializing carmaker_localization Nodelet...");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    ros::NodeHandle& nh = getNodeHandle();
    visualizer_ = std::make_shared<Visualizer>(nh);

    // SRP-based partitioned lifecycle initialization
    if (!loadParameters()) {
        NODELET_ERROR("Failed to load parameters. Aborting initialization to prevent crash.");
        return;
    }
    initEkf();
    initSvm();
    setupRosIo();
}

bool LocalizationNodelet::loadParameters() {
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    global_frame_ = pnh.param("frames/global", std::string("map"));
    prediction_frame_ = pnh.param("frames/prediction", std::string("Fr1A_pred"));
    tire_radius_ = pnh.param("vehicle/tire_radius", 0.327);

    image_type_ = pnh.param("setting/image_type", std::string("gt"));
    resolution_ = pnh.param("feature_extractor/bev/resolution", 0.05);
    r_max_ = pnh.param("feature_extractor/extraction/r_max", 15.0);
    cov_k_ = pnh.param("feature_extractor/extraction/covariance_k", 1.0);

    // Manual Initial State Configuration
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

    // 1-Pass Channel Parsing Strategy
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

            ch.extractor = std::make_shared<FeatureExtractor>(ch.name);
            ch.extractor->initialize(x_range, y_range, image_type_, resolution_);
            ch.extractor->setExtractionParameters(r_max_, cov_k_);
            ch.bev_x_range = x_range;
            ch.bev_y_range = y_range;
        }
    } else {
        NODELET_FATAL("Failed to get channels configuration array!");
        return false;
    }

    // Determine SVM bounds from the loaded channels vector (No double parsing!)
    svm_x_min_ = 1e9; svm_x_max_ = -1e9;
    svm_y_min_ = 1e9; svm_y_max_ = -1e9;

    for (const auto& ch : channels_) {
        svm_x_min_ = std::min(svm_x_min_, ch.bev_x_range[0]);
        svm_x_max_ = std::max(svm_x_max_, ch.bev_x_range[1]);
        svm_y_min_ = std::min(svm_y_min_, ch.bev_y_range[0]);
        svm_y_max_ = std::max(svm_y_max_, ch.bev_y_range[1]);
    }

    // Bounds Fallback
    if (svm_x_min_ > svm_x_max_) { svm_x_min_ = -3.0; svm_x_max_ = 7.68; }
    if (svm_y_min_ > svm_y_max_) { svm_y_min_ = -3.0; svm_y_max_ = 3.0; }
    return true;
}

void LocalizationNodelet::initEkf() {
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    int dim = pnh.param("ekf/dimension", 2);
    if (dim != 2) {
        NODELET_WARN("Currently only 2D EKF is supported! Overriding dimension to 2.");
    }

    double buffer_duration = pnh.param("ekf/state_buffer_duration", 0.5);
    ekf_core_ = std::make_shared<EkfCore>(buffer_duration);

    double slip_thresh = pnh.param("ekf/wheel_noise/slip_threshold", 0.5);
    double wheel_std = pnh.param("ekf/wheel_noise/speed_std", 0.05);

    imu_id_ = pnh.param("ekf/imu/imu_id", 0);
    double imu_x = pnh.param("vehicle/imu_" + std::to_string(imu_id_) + "/offset_x", 2.29);
    double imu_y = pnh.param("vehicle/imu_" + std::to_string(imu_id_) + "/offset_y", 0.0);
    double imu_z = pnh.param("vehicle/imu_" + std::to_string(imu_id_) + "/offset_z", 0.298);

    double rear_axle_x = pnh.param("vehicle/rear_axle_offset_x", 0.79);
    double wheelbase = pnh.param("vehicle/wheelbase", 3.0);
    double track_width = pnh.param("vehicle/track_width", 1.655);

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() * 1e-4;
    Q(VX, VX) = std::pow(pnh.param("ekf/imu/acc_std", 0.1), 2);
    Q(VY, VY) = Q(VX, VX);
    Q(YAW_RATE, YAW_RATE) = std::pow(pnh.param("ekf/imu/gyro_std", 0.01), 2);

    ekf_core_->setParameters(tire_radius_, wheelbase, track_width, rear_axle_x);
    ekf_core_->setProcessNoise(Q);

    NODELET_INFO("Selected IMU: %d (offset_x: %.2f)", imu_id_, imu_x);
}

void LocalizationNodelet::initSvm() {
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    svm_res_ = pnh.param("feature_extractor/bev/resolution", 0.05);
    fusion_ = pnh.param("feature_extractor/fusion", false);

    int svm_h = std::ceil((svm_x_max_ - svm_x_min_) / svm_res_);
    int svm_w = std::ceil((svm_y_max_ - svm_y_min_) / svm_res_);
    svm_canvas_ = cv::Mat::zeros(svm_h, svm_w, CV_8UC3);

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
}

void LocalizationNodelet::setupRosIo() {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    // Map Loader Setup
    std::string map_type = pnh.param("map_matcher/map_format", std::string("osm"));
    std::string map_file = pnh.param("map_matcher/map_file", std::string(""));

    if (map_type == "osm") {
        map_loader_ = std::make_shared<OsmMapLoader>(resolution_);
        if (!map_file.empty()) {
            if (map_loader_->load(map_file)) {
                NODELET_INFO("Map loaded successfully: %s", map_file.c_str());
            } else {
                NODELET_ERROR("Failed to load map: %s", map_file.c_str());
            }
        } else {
            NODELET_WARN("No map_file specified for map_matcher. Map will be empty.");
        }
    }

    // Matcher Setup
    std::string matcher_type = pnh.param("map_matcher/type", std::string("icp"));
    fitness_threshold_ = pnh.param("map_matcher/fitness_threshold", 0.5);
    int max_iterations = pnh.param("map_matcher/max_iterations", 50);
    search_radius_ = pnh.param("map_matcher/search_radius", 20.0);
    double vision_base_std = pnh.param("ekf/vision_noise/base_std", 0.1);

    if (matcher_type == "icp") {
        matcher_ = std::make_shared<IcpMatcher>(fitness_threshold_, max_iterations, vision_base_std);
    }

    // IO Publishers & Subscribers
    std::string topic_pose = pnh.param("topics/publish/pose", std::string("/localization/pose"));
    std::string topic_odom = pnh.param("topics/publish/odom", std::string("/localization/odom"));
    std::string topic_dynamics = pnh.param("topics/subscribe/dynamics", std::string("/dynamics_info"));

    dynamics_sub_ = nh.subscribe(topic_dynamics, 100, &LocalizationNodelet::dynamicsCallback, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_odom, 10);
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_pose, 10);

    double ekf_freq = pnh.param("ekf/frequency", 100.0);
    prediction_timer_ = nh.createTimer(ros::Duration(1.0 / ekf_freq), &LocalizationNodelet::predictionCallback, this);

    std::string topic_estimation_data = pnh.param("topics/publish/data/estimation_pose", std::string("/localization/data/estimation_pose"));
    std::string topic_correction_data = pnh.param("topics/publish/data/correction_pose", std::string("/localization/data/correction_pose"));
    estimation_data_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_estimation_data, 10);
    correction_data_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_correction_data, 10);

    // Pre-initialize all feature publishers on nodelet startup (Thread-safe)
    std::string prefix = pnh.param("topics/publish/data/features_prefix", std::string("/localization/data/features"));
    for (const auto& ch : channels_) {
        feature_data_pubs_[ch.name] = nh.advertise<carmaker_msgs::LocalFeatures>(prefix + "/" + ch.name, 10);
    }

    // Setup Camera subscriptions
    use_bundle_ = pnh.param("topics/subscribe/use_bundle", false);
    std::string topic_bundle = pnh.param("topics/subscribe/bundle", std::string("/segmentation/camera_bundle"));

    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
        const Channel& ch = channels_[i];
        if (!use_bundle_) {
            if (ch.input_topic.empty()) {
                NODELET_FATAL("input_topic is required when use_bundle is false!");
                return;
            }
            seg_subs_[i] = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh, ch.input_topic, 5);
            info_subs_[i] = nh.subscribe<sensor_msgs::CameraInfo>(ch.info_topic, 1, boost::bind(&LocalizationNodelet::infoCallback, this, _1, i));
        }

        NODELET_INFO("Configured Localization Channel %zu: [%s]", i, ch.name.c_str());
    }

    if (!use_bundle_) {
        sync_all_ = std::make_unique<message_filters::Synchronizer<SyncPolicy4>>(
            SyncPolicy4(10),
            *seg_subs_[0], *seg_subs_[1], *seg_subs_[2], *seg_subs_[3]);

        sync_all_->registerCallback(boost::bind(&LocalizationNodelet::imagesCallback, this, _1, _2, _3, _4));
    } else {
        bundle_sub_ = nh.subscribe(topic_bundle, 10, &LocalizationNodelet::bundleCallback, this);
        NODELET_INFO("Subscribed to CameraBundle topic: %s", topic_bundle.c_str());
    }
}

void LocalizationNodelet::infoCallback(const sensor_msgs::CameraInfoConstPtr& msg, size_t idx) {
    // Thread-safe lock to prevent Data Race on boost::shared_ptr write/read
    std::lock_guard<std::mutex> lock(info_array_mutex_);
    latest_infos_[idx] = msg;
}

void LocalizationNodelet::dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg) {
    std::lock_guard<std::mutex> lock(dyn_mutex_);
    latest_dynamics_ = *msg;
    dynamics_received_ = true;
}

void LocalizationNodelet::predictionCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> dyn_lock(dyn_mutex_);
    if (!dynamics_received_) return;

    std::lock_guard<std::mutex> estimation_lock(estimation_mutex_);

    // Use simulated time directly from ROS master clock if use_sim_time is true
    double current_time = ros::Time::now().toSec();
    if (current_time == 0.0) current_time = event.current_real.toSec();

    if (last_prediction_time_ <= 0.0) {
        last_prediction_time_ = current_time;
        double init_x = use_manual_initial_state_ ? init_x_ : latest_dynamics_.Car_x;
        double init_y = use_manual_initial_state_ ? init_y_ : latest_dynamics_.Car_y;
        double init_yaw = use_manual_initial_state_ ? init_yaw_ : latest_dynamics_.Car_Yaw;
        double init_vx = use_manual_initial_state_ ? 0.0 : latest_dynamics_.Car_vx;
        double init_vy = use_manual_initial_state_ ? 0.0 : latest_dynamics_.Car_vy;

        ekf_core_->initialize(init_x, init_y, init_yaw, current_time, init_vx, init_vy);
        NODELET_INFO("EKF Initialized at [%.2f, %.2f, %.2f deg] with vel [%.2f, %.2f]", init_x, init_y, init_yaw * 180.0 / M_PI, init_vx, init_vy);
        return;
    }

    double dt = current_time - last_prediction_time_;
    if (dt < -0.5 || dt > 1.0) {
        NODELET_WARN("Major time jump detected (dt: %.3f). Resetting EKF...", dt);
        last_prediction_time_ = 0.0;
        return;
    }

    if (dt <= 0.0) {
        NODELET_WARN("Minor time jitter detected (dt: %.3f). Skipping this prediction step...", dt);
        return;
    }

    // 1. EKF Prediction
    ekf_core_->prediction(current_time);

    // 2. IMU Observation Correction
    ros::NodeHandle& pnh = getPrivateNodeHandle();
    Eigen::Matrix3d R_imu = Eigen::Matrix3d::Identity();
    R_imu(0, 0) = std::pow(pnh.param("ekf/imu/acc_std", 0.1), 2);
    R_imu(1, 1) = R_imu(0, 0);
    R_imu(2, 2) = std::pow(pnh.param("ekf/imu/gyro_std", 0.01), 2);

    ekf_core_->correctImu(latest_dynamics_.Sensor_Inertial_0_Acc_B_x,
                          latest_dynamics_.Sensor_Inertial_0_Acc_B_y,
                          latest_dynamics_.Sensor_Inertial_0_Omega_B_z,
                          R_imu, current_time);

    // 3. Wheel Speed Observation Correction
    double v_left = latest_dynamics_.Vhcl_RL_rotv * tire_radius_;
    double v_right = latest_dynamics_.Vhcl_RR_rotv * tire_radius_;
    double vx_wheel = (v_left + v_right) / 2.0;

    Eigen::Matrix2d R_wheel = Eigen::Matrix2d::Identity();
    R_wheel(0, 0) = std::pow(pnh.param("ekf/wheel_noise/speed_std", 0.05), 2);
    R_wheel(1, 1) = 0.01; // Strong non-holonomic constraint (vy ~ 0)

    ekf_core_->correctVelocity(vx_wheel, 0.0, R_wheel, current_time);

    // 4. Publish Final Estimation
    publishEstimation(ros::Time(current_time));
    last_prediction_time_ = current_time;
}

void LocalizationNodelet::publishEstimation(const ros::Time& stamp) {
    auto state_frame = ekf_core_->getState();
    const auto& x = state_frame.x;
    const auto& P = state_frame.P;

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = global_frame_;
    pose_msg.pose.pose.position.x = x(X);
    pose_msg.pose.pose.position.y = x(Y);
    pose_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, x(YAW));
    pose_msg.pose.pose.orientation = tf2::toMsg(q);

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            pose_msg.pose.covariance[i * 6 + j] = P(i, j);
        }
    }
    pose_msg.pose.covariance[35] = P(YAW, YAW);

    pose_pub_.publish(pose_msg);
    estimation_data_pub_.publish(pose_msg);
    visualizer_->publishEstimation(pose_msg);

    nav_msgs::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = prediction_frame_;
    odom_msg.pose = pose_msg.pose;
    odom_msg.twist.twist.linear.x = x(VX);
    odom_msg.twist.twist.linear.y = x(VY);
    odom_msg.twist.twist.angular.z = x(YAW_RATE);
    odom_pub_.publish(odom_msg);

    // Broadcast TF: global -> prediction
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = global_frame_;
    tf_msg.child_frame_id = prediction_frame_;
    tf_msg.transform.translation.x = x(X);
    tf_msg.transform.translation.y = x(Y);
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = pose_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
}

void LocalizationNodelet::imagesCallback(
        const sensor_msgs::ImageConstPtr& img0,
        const sensor_msgs::ImageConstPtr& img1,
        const sensor_msgs::ImageConstPtr& img2,
        const sensor_msgs::ImageConstPtr& img3) {

    std::array<sensor_msgs::ImageConstPtr, 4> imgs = {img0, img1, img2, img3};

    // Thread-safe extraction of CameraInfo to completely prevent concurrency data race
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
    if (msg->names.size() != 4 || msg->images.size() != 4 || msg->infos.size() != 4) {
        ROS_WARN_THROTTLE(2.0, "CameraBundle size mismatch. Expected 4, got names: %zu, images: %zu, infos: %zu",
                            msg->names.size(), msg->images.size(), msg->infos.size());
        return;
    }

    std::array<sensor_msgs::ImageConstPtr, 4> imgs;
    std::array<sensor_msgs::CameraInfoConstPtr, 4> infos;

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
            return;
        }

        // Zero-copy conversion using boost::shared_ptr aliasing constructor
        imgs[i] = sensor_msgs::ImageConstPtr(msg, &msg->images[idx]);
        infos[i] = sensor_msgs::CameraInfoConstPtr(msg, &msg->infos[idx]);
    }

    processImages(imgs, infos);
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

    // 1. TF 조회를 Lock 외부에서 선제적으로 처리 (I/O Block 최소화) 및 초기화 시점 타임스탬프 슬롭 진단
    std::array<geometry_msgs::TransformStamped, NUM_CAMERAS> tfs;
    std::array<bool, NUM_CAMERAS> tf_success = {false, false, false, false};

    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
        Channel& ch = channels_[i];

        if (!ch.lut_initialized.load(std::memory_order_acquire)) {
            // 오직 초기화되지 않은 상태에서 최초 LUT 빌드 시에만 시간 동기화(Slop) 가드 적용!
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
                // Lock 외부에서 TF Lookup 수행 (Base "Fr1A" -> Camera 광학 좌표계 변환 조회)
                tfs[i] = tf_buffer_->lookupTransform(infos[i]->header.frame_id, "Fr1A", ros::Time(0));
                tf_success[i] = true;
            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(2.0, "TF lookup failed for %s: %s", infos[i]->header.frame_id.c_str(), ex.what());
                all_lut_ready = false;
                continue; // 예외 발생 시 해당 채널 처리를 즉시 스킵하고 다음 채널로 진행
            }
        }
    }

    // 2. 확보된 TF로 LUT를 Lock 내부에서 빠르게 업데이트
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

                ch.extractor->updateLUT(K, D, distortion_model, R_base_cam, t_base_cam);
                ch.lut_initialized.store(true, std::memory_order_release);
                NODELET_INFO("LUT initialized for %s", ch.name.c_str());
            }
        }
    }

    if (!all_lut_ready) return;

    // 3. OpenMP 병렬 비전 처리 (Multi-threading 추출로 CPU utilization 극대화)
    // 4개 채널 처리를 동시에 병렬 수행하여 Latency 획기적 단축
    std::array<carmaker_msgs::LocalFeatures, NUM_CAMERAS> extracted_features;
    std::array<cv::Mat, NUM_CAMERAS> bev_images;

    #pragma omp parallel for schedule(static, 1)
    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
        cv_bridge::CvImagePtr cv_seg;
        try {
            cv_seg = cv_bridge::toCvCopy(imgs[i], imgs[i]->encoding);
        } catch (cv_bridge::Exception& e) {
            NODELET_ERROR("cv_bridge exception: %s", e.what());
            continue;
        }

        std::vector<LocalFeature> local_feats = channels_[i].extractor->process(cv_seg->image, bev_images[i]);

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

    // 4. 머지 및 후처리 (공유 데이터 Canvas 업데이트는 Mutex로 순차 보호)
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
            // - bev_image: 행(Rows)=Y축, 열(Cols)=X축 (가로세로 축이 물리계와 반대)
            // - svm_canvas_: 행(Rows)=X축, 열(Cols)=Y축 (물리계 축 표준 정렬)
            // - 따라서 전치(Transpose) 후 수직 대칭(Flip)하여 캔버스 축과 완벽히 매칭
            cv::Mat transposed = bev_image.t();
            cv::Mat stitched_image;
            cv::flip(transposed, stitched_image, 0);

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

        if (!features.features.empty()) {
            // Publish for Data Analysis
            feature_data_pubs_[ch.name].publish(features);

            carmaker_msgs::LocalFeatures viz_features = features;
            viz_features.header.frame_id = prediction_frame_;
            visualizer_->publishObservation(ch.name, viz_features);
            ch.processed_count++;

            if (fusion_) {
                combined.features.insert(combined.features.end(), features.features.begin(), features.features.end());
            } else if (map_loader_ && matcher_) {
                performCorrection(features);
            }
        }
    }

    visualizer_->publishSvmImage(svm_canvas_);

    if (fusion_ && !combined.features.empty() && map_loader_ && matcher_) {
        performCorrection(combined);
    }
}

void LocalizationNodelet::performCorrection(const carmaker_msgs::LocalFeatures& features) {
    if (features.features.empty()) {
        ROS_WARN_THROTTLE(2.0, "performCorrection: Input features are empty!");
        return;
    }

    Eigen::Isometry2d initial_guess = Eigen::Isometry2d::Identity();
    std::vector<MapFeature> ref_features;
    double query_x = 0.0;
    double query_y = 0.0;

    // Thread-safe extraction of EKF state to prevent concurrency conflicts with high-frequency prediction
    {
        std::lock_guard<std::mutex> lock(estimation_mutex_);
        auto state_frame = ekf_core_->getState();
        const auto& ekf_pose = state_frame.x;
        initial_guess.translation() = Eigen::Vector2d(ekf_pose(X), ekf_pose(Y));
        initial_guess.linear() = Eigen::Rotation2Dd(ekf_pose(YAW)).toRotationMatrix();
        query_x = ekf_pose(X);
        query_y = ekf_pose(Y);
    }

    ref_features = map_loader_->queryNear(query_x, query_y, search_radius_);

    if (ref_features.empty()) {
        ROS_WARN_THROTTLE(2.0, "performCorrection: No reference features found in OSM map within search_radius (%.1f m) of pose (%.2f, %.2f)!", search_radius_, query_x, query_y);
        return;
    }

    // Convert ROS features to pure C++ features to achieve absolute ROS-free matching decoupling
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

    // ICP Matching (Mutex released to prevent blocking the 100Hz EKF loop!)
    auto match_result = matcher_->match(cpp_features, ref_features, initial_guess);
    if (match_result.success) {
        Eigen::Vector3d z;
        z << match_result.transform.translation().x(),
                match_result.transform.translation().y(),
                Eigen::Rotation2Dd(match_result.transform.linear()).angle();

        // Thread-safe EKF correction step under mutex lock
        Eigen::Matrix3d R_map = match_result.covariance;
        {
            std::lock_guard<std::mutex> lock(estimation_mutex_);
            ekf_core_->correctPose(z(0), z(1), z(2), R_map, features.header.stamp.toSec());
        }

        // Final Logging and Debug Visualization (Lock free)
        geometry_msgs::PoseWithCovarianceStamped correction_msg;
        correction_msg.header.stamp = features.header.stamp;
        correction_msg.header.frame_id = global_frame_;
        correction_msg.pose.pose.position.x = z(0);
        correction_msg.pose.pose.position.y = z(1);
        tf2::Quaternion q;
        q.setRPY(0, 0, z(2));
        correction_msg.pose.pose.orientation = tf2::toMsg(q);

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                int idx = (r == 2 ? 5 : r) * 6 + (c == 2 ? 5 : c);
                correction_msg.pose.covariance[idx] = match_result.covariance(r, c);
            }
        }
        correction_data_pub_.publish(correction_msg);
        visualizer_->publishCorrection(correction_msg);
        ROS_INFO_THROTTLE(2.0, "performCorrection: ICP Match SUCCESS! Fitness score: %.3f (threshold: %.3f)", match_result.fitness_score, fitness_threshold_);
    } else {
        ROS_WARN_THROTTLE(2.0, "performCorrection: ICP Match FAILED. Fitness score: %.3f (threshold: %.3f). Observed: %zu, Ref: %zu", match_result.fitness_score, fitness_threshold_, features.features.size(), ref_features.size());
    }
}

} // namespace carmaker_localization

PLUGINLIB_EXPORT_CLASS(carmaker_localization::LocalizationNodelet, nodelet::Nodelet)
