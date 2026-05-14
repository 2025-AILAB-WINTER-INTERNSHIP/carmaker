#include "carmaker_localization/localization_nodelet.h"
#include "carmaker_localization/osm_map_loader.h"
#include "carmaker_localization/icp_matcher.h"
#include <XmlRpcValue.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>

namespace carmaker_localization {

void LocalizationNodelet::onInit() {
    NODELET_INFO("Initializing carmaker_localization Nodelet...");
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    visualizer_ = std::make_shared<Visualizer>(nh);

    global_frame_ = pnh.param("frames/global", std::string("map"));
    prediction_frame_ = pnh.param("frames/prediction", std::string("Fr1A_pred"));

    int dim = pnh.param("ekf/dimension", 2);
    if (dim != 2) {
        NODELET_WARN("Currently only 2D EKF is supported! Overriding dimension to 2. Future 3D extensibility will support dimension 3.");
    }

    double buffer_duration = pnh.param("ekf/state_buffer_duration", 0.5);
    ekf_core_ = std::make_shared<EkfCore>(buffer_duration);
    double tire_radius = pnh.param("vehicle/tire_radius", 0.327);
    double slip_thresh = pnh.param("ekf/wheel_noise/slip_threshold", 0.5);
    double wheel_std = pnh.param("ekf/wheel_noise/speed_std", 0.05);

    imu_id_ = pnh.param("ekf/imu/imu_id", 0);
    double imu_x = pnh.param("vehicle/imu_" + std::to_string(imu_id_) + "/offset_x", 2.29); // Usually same as cg_offset_x
    double imu_y = pnh.param("vehicle/imu_" + std::to_string(imu_id_) + "/offset_y", 0.0);
    double imu_z = pnh.param("vehicle/imu_" + std::to_string(imu_id_) + "/offset_z", 0.298);

    double rear_axle_x = pnh.param("vehicle/rear_axle_offset_x", 0.79);
    double wheelbase = pnh.param("vehicle/wheelbase", 3.0);
    double track_width = pnh.param("vehicle/track_width", 1.655);

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() * 1e-4;
    Q(VX, VX) = std::pow(pnh.param("ekf/imu/acc_std", 0.1), 2);
    Q(VY, VY) = Q(VX, VX);
    Q(PSIDOT, PSIDOT) = std::pow(pnh.param("ekf/imu/gyro_std", 0.01), 2);
    ekf_core_->setParameters(tire_radius, slip_thresh, wheel_std, Q, wheelbase, track_width, rear_axle_x, imu_x, imu_y, imu_z);
    ekf_core_->setDivergenceThreshold(pnh.param("ekf/divergence_threshold", 100.0));

    NODELET_INFO("Selected IMU: %d (offset_x: %.2f)", imu_id_, imu_x);

    // Initial State Configuration (Array: [x, y, yaw_deg])
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
        NODELET_INFO("No valid initial_state found (~ or empty). Will initialize from DynamicsInfo (GT).");
    }

    std::string topic_pose = pnh.param("topics/publish/pose", std::string("/localization/pose"));
    std::string topic_odom = pnh.param("topics/publish/odom", std::string("/localization/odom"));
    std::string topic_dynamics = pnh.param("topics/subscribe/dynamics", std::string("/dynamics_info"));

    dynamics_sub_ = nh.subscribe(topic_dynamics, 100, &LocalizationNodelet::dynamicsCallback, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_odom, 10);
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_pose, 10);

    double ekf_freq = pnh.param("ekf/frequency", 100.0);
    prediction_timer_ = nh.createTimer(ros::Duration(1.0 / ekf_freq), &LocalizationNodelet::predictionCallback, this);

    std::string map_type = pnh.param("map_matcher/map_format", std::string("osm"));
    std::string map_file = pnh.param("map_matcher/map_file", std::string(""));

    if (map_type == "osm") {
        map_loader_ = std::make_shared<OsmMapLoader>();
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

    std::string matcher_type = pnh.param("map_matcher/type", std::string("icp"));
    double fitness_threshold = pnh.param("map_matcher/fitness_threshold", 0.5);
    int max_iterations = pnh.param("map_matcher/max_iterations", 50);
    search_radius_ = pnh.param("map_matcher/search_radius", 20.0);
    double vision_base_std = pnh.param("ekf/vision_noise/base_std", 0.1);

    if (matcher_type == "icp") {
        matcher_ = std::make_shared<IcpMatcher>(fitness_threshold, max_iterations, vision_base_std);
    }

    std::string topic_estimation_data = pnh.param("topics/publish/data/estimation_pose", std::string("/localization/data/estimation_pose"));
    std::string topic_correction_data = pnh.param("topics/publish/data/correction_pose", std::string("/localization/data/correction_pose"));
    estimation_data_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_estimation_data, 10);
    correction_data_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_correction_data, 10);

    // SVM Canvas Bounds
    svm_x_min_ = 1e9; svm_x_max_ = -1e9;
    svm_y_min_ = 1e9; svm_y_max_ = -1e9;

    XmlRpc::XmlRpcValue channel_list;
    if (pnh.getParam("feature_extractor/channels", channel_list) && channel_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < channel_list.size(); ++i) {
            XmlRpc::XmlRpcValue& ch_cfg = channel_list[i];
            if (ch_cfg.hasMember("bev_x_range") && ch_cfg["bev_x_range"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                svm_x_min_ = std::min(svm_x_min_, static_cast<double>(ch_cfg["bev_x_range"][0]));
                svm_x_max_ = std::max(svm_x_max_, static_cast<double>(ch_cfg["bev_x_range"][1]));
            }
            if (ch_cfg.hasMember("bev_y_range") && ch_cfg["bev_y_range"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                svm_y_min_ = std::min(svm_y_min_, static_cast<double>(ch_cfg["bev_y_range"][0]));
                svm_y_max_ = std::max(svm_y_max_, static_cast<double>(ch_cfg["bev_y_range"][1]));
            }
        }
    }

    // Fallback if no channels
    if (svm_x_min_ > svm_x_max_) { svm_x_min_ = -3.0; svm_x_max_ = 7.68; }
    if (svm_y_min_ > svm_y_max_) { svm_y_min_ = -3.0; svm_y_max_ = 3.0; }

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

    if (channel_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        if (channel_list.size() != 4) {
            NODELET_ERROR("This package is hardcoded for exactly 4 cameras. Found %d channels.", channel_list.size());
            return;
        }

        channels_.reserve(4);

        for (int i = 0; i < 4; ++i) {
            XmlRpc::XmlRpcValue& ch_cfg = channel_list[i];

            channels_.emplace_back();
            Channel& ch = channels_.back();

            ch.name = static_cast<std::string>(ch_cfg["name"]);
            ch.frame = static_cast<std::string>(ch_cfg["frame"]);
            std::string input_topic = static_cast<std::string>(ch_cfg["input_topic"]);
            std::string info_topic = ch_cfg.hasMember("info_topic") ? static_cast<std::string>(ch_cfg["info_topic"]) : "/synced/" + ch.name + "/camera_info";

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

            ch.extractor = std::make_shared<FeatureExtractor>(nh, ch.name, x_range, y_range);
            ch.bev_x_range = x_range;
            ch.bev_y_range = y_range;

            seg_subs_[i] = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh, input_topic, 5);
            info_subs_[i] = nh.subscribe<sensor_msgs::CameraInfo>(info_topic, 1, boost::bind(&LocalizationNodelet::infoCallback, this, _1, i));

            NODELET_INFO("Configured Localization Channel %d: [%s]", i, ch.name.c_str());
        }

        sync_all_ = std::make_unique<message_filters::Synchronizer<SyncPolicy4>>(
            SyncPolicy4(10),
            *seg_subs_[0], *seg_subs_[1], *seg_subs_[2], *seg_subs_[3]);

        sync_all_->registerCallback(boost::bind(&LocalizationNodelet::imagesCallback, this, _1, _2, _3, _4));
    }
}

void LocalizationNodelet::infoCallback(const sensor_msgs::CameraInfoConstPtr& msg, size_t idx) {
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

    double current_time = event.current_real.toSec();
    if (current_time == 0.0) current_time = ros::Time::now().toSec();

    if (last_prediction_time_ <= 0.0) {
        last_prediction_time_ = current_time;
        Eigen::Matrix<double, STATE_DIM, 1> x0 = Eigen::Matrix<double, STATE_DIM, 1>::Zero();

        if (use_manual_initial_state_) {
            x0(X) = init_x_;
            x0(Y) = init_y_;
            x0(PSI) = init_yaw_;
            x0(VX) = 0.0;
            x0(VY) = 0.0;
        } else {
            x0(X) = latest_dynamics_.Car_x;
            x0(Y) = latest_dynamics_.Car_y;
            x0(PSI) = latest_dynamics_.Car_Yaw;
            x0(VX) = latest_dynamics_.Car_vx;
            x0(VY) = latest_dynamics_.Car_vy;
        }
        x0(SW) = 1.0;

        Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0 = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() * 0.1;
        ekf_core_->init(x0, P0);
        NODELET_INFO("EKF Initialized at [%.2f, %.2f, %.2f]", x0(X), x0(Y), x0(PSI));
        return;
    }

    double dt = current_time - last_prediction_time_;
    if (dt < 0.0 || dt > 1.0) {
        NODELET_WARN("Time jump detected (dt: %.3f). Resetting EKF...", dt);
        last_prediction_time_ = 0.0;
        return;
    }

    if (dt == 0.0) return;

    // 1. EKF Prediction (Constant Acceleration)
    ekf_core_->prediction(timestamp);

    // 2. IMU Observation Correction
    // Correcting [ax, ay, yaw_rate]
    Eigen::Matrix3d R_imu = Eigen::Matrix3d::Identity();
    R_imu(0, 0) = std::pow(pnh.param("ekf/imu/acc_std", 0.1), 2);
    R_imu(1, 1) = R_imu(0, 0);
    R_imu(2, 2) = std::pow(pnh.param("ekf/imu/gyro_std", 0.01), 2);

    ekf_core_->correctImu(latest_dynamics_.Sensor_Inertial_0_Acc_B_x,
                          latest_dynamics_.Sensor_Inertial_0_Acc_B_y,
                          latest_dynamics_.Sensor_Inertial_0_Omega_B_z,
                          R_imu, timestamp);

    // 3. Wheel Speed Observation Correction
    // Simple differential drive model for vx, assuming vy = 0 (non-holonomic)
    double v_left = latest_dynamics_.Vhcl_RL_rotv * tire_radius_;
    double v_right = latest_dynamics_.Vhcl_RR_rotv * tire_radius_;
    double vx_wheel = (v_left + v_right) / 2.0;
    
    Eigen::Matrix2d R_wheel = Eigen::Matrix2d::Identity();
    R_wheel(0, 0) = std::pow(pnh.param("ekf/wheel_noise/speed_std", 0.05), 2);
    R_wheel(1, 1) = 0.01; // Strong non-holonomic constraint (vy ~ 0)

    ekf_core_->correctVelocity(vx_wheel, 0.0, R_wheel, timestamp);

    // 4. Publish Final Estimation
    publishEstimation(ros::Time(current_time));
    last_prediction_time_ = current_time;
}

void LocalizationNodelet::publishEstimation(const ros::Time& stamp) {
    auto x = ekf_core_->getState();
    auto P = ekf_core_->getCovariance();

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = global_frame_;
    pose_msg.pose.pose.position.x = x(X);
    pose_msg.pose.pose.position.y = x(Y);
    pose_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, x(PSI));
    pose_msg.pose.pose.orientation = tf2::toMsg(q);

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            pose_msg.pose.covariance[i * 6 + j] = P(i, j);
        }
    }
    pose_msg.pose.covariance[35] = P(PSI, PSI);

    // Set frame_id for visualization before publishing
    pose_msg.header.frame_id = global_frame_;

    pose_pub_.publish(pose_msg);
    estimation_data_pub_.publish(pose_msg);
    visualizer_->publishEstimation(pose_msg);

    nav_msgs::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = prediction_frame_;
    odom_msg.pose = pose_msg.pose;
    odom_msg.twist.twist.linear.x = x(VX);
    odom_msg.twist.twist.linear.y = x(VY);
    odom_msg.twist.twist.angular.z = x(PSIDOT);
    odom_pub_.publish(odom_msg);

    // 3. Broadcast TF: global -> prediction (To isolate prediction from GT)
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

    const sensor_msgs::ImageConstPtr imgs[4] = {img0, img1, img2, img3};

    // Ensure we have CameraInfo for all channels
    for (int i = 0; i < 4; ++i) {
        if (!latest_infos_[i]) {
            NODELET_WARN_THROTTLE(2.0, "Waiting for CameraInfo on channel %d...", i);
            return;
        }
    }
    const sensor_msgs::CameraInfoConstPtr infos[4] = {latest_infos_[0], latest_infos_[1], latest_infos_[2], latest_infos_[3]};

    carmaker_msgs::LocalFeatures combined;
    combined.header = img0->header;
    combined.camera_name = "combined";

    bool all_lut_ready = true;

    for (size_t i = 0; i < 4; ++i) {
        Channel& ch = channels_[i];
        if (!ch.lut_initialized) {
            geometry_msgs::TransformStamped tf;
            try {
                tf = tf_buffer_->lookupTransform("Fr1A", ch.frame, ros::Time(0));
                ch.extractor->updateLUT(infos[i], tf);
                ch.lut_initialized = true;
                NODELET_INFO("LUT initialized for %s", ch.name.c_str());
            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(2.0, "TF lookup failed for %s: %s", ch.frame.c_str(), ex.what());
                all_lut_ready = false;
                continue;
            }
        }
    }

    if (!all_lut_ready) return;

    for (size_t i = 0; i < 4; ++i) {
        Channel& ch = channels_[i];
        cv::Mat bev_image;
        auto features = ch.extractor->process(imgs[i], infos[i], bev_image);

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

            int h = bev_image.rows;
            int w = bev_image.cols;

            if (r_off >= 0 && c_off >= 0 && r_off + h <= svm_canvas_.rows && c_off + w <= svm_canvas_.cols) {
                cv::Mat global_mask = svm_masks_[ch.name];
                cv::Mat roi_mask = global_mask(cv::Rect(c_off, r_off, w, h));
                cv::Mat canvas_roi = svm_canvas_(cv::Rect(c_off, r_off, w, h));
                bev_image.copyTo(canvas_roi, roi_mask);
            }
        }

        if (!features.features.empty()) {
            // Publish for Data Analysis
            if (feature_data_pubs_.find(ch.name) == feature_data_pubs_.end()) {
                ros::NodeHandle pnh("~");
                std::string prefix = pnh.param("topics/publish/data/features_prefix", std::string("/localization/data/features"));
                feature_data_pubs_[ch.name] = nh.advertise<carmaker_msgs::LocalFeatures>(prefix + "/" + ch.name, 10);
            }
            feature_data_pubs_[ch.name].publish(features);

            features.header.frame_id = prediction_frame_;
            visualizer_->publishObservation(ch.name, features);
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
    auto ekf_pose = ekf_core_->getState();
    Eigen::Isometry2d initial_guess = Eigen::Isometry2d::Identity();
    initial_guess.translation() = Eigen::Vector2d(ekf_pose(X), ekf_pose(Y));
    initial_guess.linear() = Eigen::Rotation2Dd(ekf_pose(PSI)).toRotationMatrix();

    auto ref_features = map_loader_->queryNear(ekf_pose(X), ekf_pose(Y), search_radius_);

    if (!ref_features.empty()) {
        auto match_result = matcher_->match(features.features, ref_features, initial_guess);
        if (match_result.success) {
            Eigen::Vector3d z;
            z << match_result.transform.translation().x(),
                    match_result.transform.translation().y(),
                    Eigen::Rotation2Dd(match_result.transform.linear()).angle();

            // 5. EKF Correction (Vision/Map)
            Eigen::Matrix3d R_map = match_result.covariance;
            ekf_core_->correctPose(z(0), z(1), z(2), R_map, features.header.stamp.toSec());

            // 6. Final Logging and Debug Visualization
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
        }
    }
}

} // namespace carmaker_localization

PLUGINLIB_EXPORT_CLASS(carmaker_localization::LocalizationNodelet, nodelet::Nodelet)
