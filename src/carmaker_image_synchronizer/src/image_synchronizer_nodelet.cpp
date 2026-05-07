#include <carmaker_image_synchronizer/image_synchronizer_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace carmaker_image_synchronizer {

void ImageSynchronizerNodelet::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    // 1. Load Parameters
    std::string in_front_img, in_front_info, in_rear_img, in_rear_info;
    std::string in_left_img, in_left_info, in_right_img, in_right_info;
    std::string out_front_img, out_front_info, out_rear_img, out_rear_info;
    std::string out_left_img, out_left_info, out_right_img, out_right_info;
    double slop;
    int queue_size;

    // Input Topics
    pnh_.param<std::string>("input_topics/front/image", in_front_img, "/mono/fisheye/front/image_raw");
    pnh_.param<std::string>("input_topics/front/info",  in_front_info, "/mono/fisheye/front/camera_info");
    pnh_.param<std::string>("input_topics/rear/image",  in_rear_img,  "/mono/fisheye/rear/image_raw");
    pnh_.param<std::string>("input_topics/rear/info",   in_rear_info,  "/mono/fisheye/rear/camera_info");
    pnh_.param<std::string>("input_topics/left/image",  in_left_img,  "/mono/fisheye/left/image_raw");
    pnh_.param<std::string>("input_topics/left/info",   in_left_info,  "/mono/fisheye/left/camera_info");
    pnh_.param<std::string>("input_topics/right/image", in_right_img, "/mono/fisheye/right/image_raw");
    pnh_.param<std::string>("input_topics/right/info",  in_right_info, "/mono/fisheye/right/camera_info");

    // Output Topics
    pnh_.param<std::string>("output_topics/front/image", out_front_img, "/synced/front/image");
    pnh_.param<std::string>("output_topics/front/info",  out_front_info, "/synced/front/camera_info");
    pnh_.param<std::string>("output_topics/rear/image",  out_rear_img,  "/synced/rear/image");
    pnh_.param<std::string>("output_topics/rear/info",   out_rear_info,  "/synced/rear/camera_info");
    pnh_.param<std::string>("output_topics/left/image",  out_left_img,  "/synced/left/image");
    pnh_.param<std::string>("output_topics/left/info",   out_left_info,  "/synced/left/camera_info");
    pnh_.param<std::string>("output_topics/right/image", out_right_img, "/synced/right/image");
    pnh_.param<std::string>("output_topics/right/info",  out_right_info, "/synced/right/camera_info");

    pnh_.param<double>("sync_slop_sec", slop, 0.05);
    pnh_.param<int>("queue_size", queue_size, 10);

    NODELET_INFO("Initializing Multi-Channel Sync for: [%s, %s, %s, %s]", in_front_img.c_str(), in_rear_img.c_str(), in_left_img.c_str(), in_right_img.c_str());

    // 2. Setup Subscribers
    sub_front_ = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh_, in_front_img, queue_size);
    sub_rear_  = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh_, in_rear_img,  queue_size);
    sub_left_  = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh_, in_left_img,  queue_size);
    sub_right_ = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh_, in_right_img, queue_size);

    info_front_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(in_front_info, 1, [this](const auto& msg){ std::lock_guard<std::mutex> lock(info_mutex_); last_info_front_ = *msg; has_info_front_ = true; });
    info_rear_sub_  = nh_.subscribe<sensor_msgs::CameraInfo>(in_rear_info,  1, [this](const auto& msg){ std::lock_guard<std::mutex> lock(info_mutex_); last_info_rear_  = *msg; has_info_rear_  = true; });
    info_left_sub_  = nh_.subscribe<sensor_msgs::CameraInfo>(in_left_info,  1, [this](const auto& msg){ std::lock_guard<std::mutex> lock(info_mutex_); last_info_left_  = *msg; has_info_left_  = true; });
    info_right_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(in_right_info, 1, [this](const auto& msg){ std::lock_guard<std::mutex> lock(info_mutex_); last_info_right_ = *msg; has_info_right_ = true; });

    // 3. Setup Synchronizer
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), *sub_front_, *sub_rear_, *sub_left_, *sub_right_
    );
    sync_->setInterMessageLowerBound(ros::Duration(slop));
    sync_->registerCallback([this](const auto& f, const auto& r, const auto& l, const auto& ri) {
        this->syncCallback(f, r, l, ri);
    });

    // 4. Setup Publishers (8-Channel)
    pub_front_img_  = nh_.advertise<sensor_msgs::Image>(out_front_img, 1);
    pub_rear_img_   = nh_.advertise<sensor_msgs::Image>(out_rear_img,  1);
    pub_left_img_   = nh_.advertise<sensor_msgs::Image>(out_left_img,  1);
    pub_right_img_  = nh_.advertise<sensor_msgs::Image>(out_right_img, 1);

    pub_front_info_ = nh_.advertise<sensor_msgs::CameraInfo>(out_front_info, 1);
    pub_rear_info_  = nh_.advertise<sensor_msgs::CameraInfo>(out_rear_info,  1);
    pub_left_info_  = nh_.advertise<sensor_msgs::CameraInfo>(out_left_info,  1);
    pub_right_info_ = nh_.advertise<sensor_msgs::CameraInfo>(out_right_info, 1);

    NODELET_INFO("Multi-Channel Image Synchronizer Started.");
}

void ImageSynchronizerNodelet::syncCallback(const sensor_msgs::ImageConstPtr& front,
                                            const sensor_msgs::ImageConstPtr& rear,
                                            const sensor_msgs::ImageConstPtr& left,
                                            const sensor_msgs::ImageConstPtr& right) {
    // 1. Unified Master Clock
    ros::Time sync_time = front->header.stamp;

    // 2. Publish Images (Individual topics with unified timestamp)
    auto publish_img = [&](const sensor_msgs::ImageConstPtr& img_ptr, ros::Publisher& pub) {
        // Create a new message to modify the header without changing original data if possible
        // Note: In ROS1, this still involves a copy of the message object.
        sensor_msgs::Image synced_img = *img_ptr;
        synced_img.header.stamp = sync_time;
        pub.publish(synced_img);
    };

    publish_img(front, pub_front_img_);
    publish_img(rear,  pub_rear_img_);
    publish_img(left,  pub_left_img_);
    publish_img(right, pub_right_img_);

    // 3. Publish CameraInfo (Independent per channel for better liveness)
    {
        std::lock_guard<std::mutex> lock(info_mutex_);

        auto safe_pub_info = [&](bool has_info, sensor_msgs::CameraInfo info, ros::Publisher& pub, const char* name) {
            if (has_info) {
                info.header.stamp = sync_time;
                pub.publish(info);
            } else {
                NODELET_WARN_THROTTLE(10.0, "[%s] CameraInfo not ready yet. Skipping this channel's info.", name);
            }
        };

        safe_pub_info(has_info_front_, last_info_front_, pub_front_info_, "Front");
        safe_pub_info(has_info_rear_,  last_info_rear_,  pub_rear_info_,  "Rear");
        safe_pub_info(has_info_left_,  last_info_left_,  pub_left_info_,  "Left");
        safe_pub_info(has_info_right_, last_info_right_, pub_right_info_, "Right");
    }
}

} // namespace carmaker_image_synchronizer

PLUGINLIB_EXPORT_CLASS(carmaker_image_synchronizer::ImageSynchronizerNodelet, nodelet::Nodelet)
