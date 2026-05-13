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
    visualizer_ = std::make_shared<Visualizer>(nh);

    int dim = pnh.param("ekf/dimension", 2);
    if (dim != 2) {
        NODELET_WARN("Currently only 2D EKF is supported! Overriding dimension to 2. Future 3D extensibility will support dimension 3.");
    }

    double buffer_duration = pnh.param("ekf/state_buffer_duration", 0.5);
    ekf_core_ = std::make_shared<EkfCore>(buffer_duration);
    double tire_radius = pnh.param("vehicle/tire_radius", 0.31);
    double slip_thresh = pnh.param("ekf/wheel_noise/slip_threshold", 0.5);
    double wheel_std = pnh.param("ekf/wheel_noise/speed_std", 0.05);

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() * 1e-4;
    Q(VX, VX) = std::pow(pnh.param("ekf/imu_noise/acc_std", 0.1), 2);
    Q(VY, VY) = Q(VX, VX);
    Q(PSIDOT, PSIDOT) = std::pow(pnh.param("ekf/imu_noise/gyro_std", 0.01), 2);
    ekf_core_->setParameters(tire_radius, slip_thresh, wheel_std, Q);
    ekf_core_->setDivergenceThreshold(pnh.param("ekf/divergence_threshold", 100.0));

    std::string topic_pose = pnh.param("topics/publish/pose", std::string("/localization/pose"));
    std::string topic_odom = pnh.param("topics/publish/odom", std::string("/localization/odom"));
    std::string topic_map_correction = pnh.param("topics/publish/map_correction", std::string("/localization/map_correction"));
    std::string topic_dynamics = pnh.param("topics/subscribe/dynamics", std::string("/dynamics_info"));

    dynamics_sub_ = nh.subscribe(topic_dynamics, 100, &LocalizationNodelet::dynamicsCallback, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_odom, 10);
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_pose, 10);

    double ekf_freq = pnh.param("ekf/frequency", 100.0);
    ekf_timer_ = nh.createTimer(ros::Duration(1.0 / ekf_freq), &LocalizationNodelet::ekfTimerCallback, this);

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

    map_correction_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_map_correction, 10);

    // Dynamic SVM Canvas Bounds
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

    int svm_h = std::ceil((svm_x_max_ - svm_x_min_) / svm_res_);
    int svm_w = std::ceil((svm_y_max_ - svm_y_min_) / svm_res_);
    svm_canvas_ = cv::Mat::zeros(svm_h, svm_w, CV_8UC3);

    double v_front = pnh.param("svm/vehicle/front_edge", 4.68);
    double v_rear = pnh.param("svm/vehicle/rear_edge", 0.0);
    double front_off = pnh.param("svm/vehicle/seam_front_offset", -1.0);
    double rear_off = pnh.param("svm/vehicle/seam_rear_offset", 0.5);
    double slope_front = pnh.param("svm/vehicle/seam_slope_front", 0.8);
    double slope_rear = pnh.param("svm/vehicle/seam_slope_rear", 1.8);

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
        channels_.reserve(channel_list.size());

        for (int i = 0; i < channel_list.size(); ++i) {
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

            ros::NodeHandle pnh("~");
            std::string features_prefix = pnh.param("topics/publish/features_prefix", std::string("/localization/features"));
            ch.feature_pub = nh.advertise<carmaker_msgs::LocalFeatures>(features_prefix + "/" + ch.name + "/local_features", 10);

            ch.seg_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh, input_topic, 5);
            ch.info_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, info_topic, 5);

            ch.sync = std::make_unique<message_filters::Synchronizer<Channel::SyncPolicy>>(Channel::SyncPolicy(10), *ch.seg_sub, *ch.info_sub);
            ch.sync->registerCallback(boost::bind(&LocalizationNodelet::imageCallback, this, _1, _2, i));

            NODELET_INFO("Configured Localization Channel: [%s]", ch.name.c_str());
        }
    }
}

void LocalizationNodelet::dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg) {
    std::lock_guard<std::mutex> lock(dyn_mutex_);
    latest_dynamics_ = *msg;
    dynamics_received_ = true;
}

void LocalizationNodelet::ekfTimerCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> dyn_lock(dyn_mutex_);
    if (!dynamics_received_) return;

    std::lock_guard<std::mutex> ekf_lock(ekf_mutex_);

    double current_time = event.current_real.toSec();
    if (current_time == 0.0) current_time = ros::Time::now().toSec();

    if (last_ekf_time_ <= 0.0) {
        last_ekf_time_ = current_time;
        Eigen::Matrix<double, STATE_DIM, 1> x0 = Eigen::Matrix<double, STATE_DIM, 1>::Zero();
        x0(X) = latest_dynamics_.Car_x;
        x0(Y) = latest_dynamics_.Car_y;
        x0(PSI) = latest_dynamics_.Car_Yaw;
        x0(VX) = latest_dynamics_.Car_vx;
        x0(VY) = latest_dynamics_.Car_vy;
        x0(SW) = 1.0;
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0 = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() * 0.1;
        ekf_core_->init(x0, P0);
        return;
    }

    double dt = current_time - last_ekf_time_;
    if (dt <= 0.0 || dt > 1.0) {
        last_ekf_time_ = current_time;
        return;
    }

    Eigen::Matrix<double, 6, 1> u;
    u << latest_dynamics_.Sensor_Inertial_0_Acc_B_x,
         latest_dynamics_.Sensor_Inertial_0_Acc_B_y,
         latest_dynamics_.Sensor_Inertial_0_Omega_B_z,
         latest_dynamics_.Vhcl_RL_rotv,
         latest_dynamics_.Vhcl_RR_rotv,
         latest_dynamics_.Steer_WhlAng;

    ekf_core_->predict(dt, u);
    last_ekf_time_ = current_time;

    publishEkfState(ros::Time(current_time));
}

void LocalizationNodelet::publishEkfState(const ros::Time& stamp) {
    auto x = ekf_core_->getState();
    auto P = ekf_core_->getCovariance();

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map";
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

    pose_pub_.publish(pose_msg);
    visualizer_->publishEkfState(pose_msg);

    nav_msgs::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose = pose_msg.pose;
    odom_msg.twist.twist.linear.x = x(VX);
    odom_msg.twist.twist.linear.y = x(VY);
    odom_msg.twist.twist.angular.z = x(PSIDOT);
    odom_pub_.publish(odom_msg);

    visualizer_->publishEkfState(pose_msg);
}

void LocalizationNodelet::imageCallback(const sensor_msgs::ImageConstPtr& seg_msg,
                                        const sensor_msgs::CameraInfoConstPtr& info_msg,
                                        size_t ch_idx) {
    Channel& ch = channels_[ch_idx];

    if (!ch.lut_initialized) {
        geometry_msgs::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("base_link", ch.frame, ros::Time(0));
            ch.extractor->updateLUT(info_msg, tf);
            ch.lut_initialized = true;
            NODELET_INFO("LUT initialized for %s", ch.name.c_str());
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(2.0, "TF lookup failed for %s: %s", ch.frame.c_str(), ex.what());
            return;
        }
    }

    cv::Mat bev_image;
    auto features = ch.extractor->process(seg_msg, info_msg, bev_image);

    if (!bev_image.empty() && svm_masks_.count(ch.name) > 0) {
        std::lock_guard<std::mutex> lock(svm_mutex_);
        if (svm_canvas_.empty()) {
            int h = std::ceil((svm_x_max_ - svm_x_min_) / svm_res_);
            int w = std::ceil((svm_y_max_ - svm_y_min_) / svm_res_);
            svm_canvas_ = cv::Mat::zeros(h, w, bev_image.type());
        }

        // Calculate offset in global canvas
        double roi_x_max = ch.bev_x_range[1];
        double roi_y_max = ch.bev_y_range[1];
        int r_off = std::round((svm_x_max_ - roi_x_max) / svm_res_);
        int c_off = std::round((svm_y_max_ - roi_y_max) / svm_res_);

        int h = bev_image.rows;
        int w = bev_image.cols;

        // Ensure within bounds
        if (r_off >= 0 && c_off >= 0 && r_off + h <= svm_canvas_.rows && c_off + w <= svm_canvas_.cols) {
            // Get the corresponding hard-cut mask for this camera
            cv::Mat global_mask = svm_masks_[ch.name];
            cv::Mat roi_mask = global_mask(cv::Rect(c_off, r_off, w, h));

            // Copy to canvas using the hard-cut mask
            cv::Mat canvas_roi = svm_canvas_(cv::Rect(c_off, r_off, w, h));
            bev_image.copyTo(canvas_roi, roi_mask);
        }

        ros::NodeHandle pnh("~");
        visualizer_->publishSvmImage(svm_canvas_);
    }

    if (!features.features.empty()) {
        ch.feature_pub.publish(features);
        visualizer_->publishFeatures(features);
        ch.processed_count++;

        if (map_loader_ && matcher_) {
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

                    double img_timestamp = seg_msg->header.stamp.toSec();
                    {
                        std::lock_guard<std::mutex> lock(ekf_mutex_);
                        ekf_core_->updateVision(img_timestamp, z, match_result.covariance);
                    }

                    geometry_msgs::PoseWithCovarianceStamped correction_msg;
                    correction_msg.header.stamp = seg_msg->header.stamp;
                    correction_msg.header.frame_id = "map";
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
                    map_correction_pub_.publish(correction_msg);
                    visualizer_->publishMapCorrection(correction_msg);
                }
            }
        }
    }
}

} // namespace carmaker_localization

PLUGINLIB_EXPORT_CLASS(carmaker_localization::LocalizationNodelet, nodelet::Nodelet)
