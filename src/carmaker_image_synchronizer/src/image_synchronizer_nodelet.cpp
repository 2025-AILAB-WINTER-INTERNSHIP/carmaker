#include <carmaker_image_synchronizer/image_synchronizer_nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace carmaker_image_synchronizer {

void ImageSynchronizerNodelet::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    // 1. Load Settings
    int expected_channel_num, queue_size;
    double slop;
    std::string master_name;
    
    pnh_.param<int>("settings/channel_num", expected_channel_num, 4);
    pnh_.param<int>("settings/queue_size", queue_size, 10);
    pnh_.param<double>("settings/sync_slop_sec", slop, 0.05);
    pnh_.param<double>("settings/info_timeout_sec", info_timeout_, 2.0);
    pnh_.param<std::string>("settings/master_channel", master_name, "front");

    // 2. Load Channels from YAML
    XmlRpc::XmlRpcValue channel_list;
    if (!pnh_.getParam("channels", channel_list) || channel_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        NODELET_FATAL("Failed to load 'channels' list.");
        return;
    }

    if (channel_list.size() != expected_channel_num) {
        NODELET_FATAL("Current SyncPolicy supports exactly %d channels. Detected: %zu", expected_channel_num, channel_list.size());
        return;
    }

    channels_.resize(channel_list.size());
    for (int i = 0; i < channel_list.size(); ++i) {
        auto& ch = channels_[i];
        XmlRpc::XmlRpcValue& entry = channel_list[i];

        if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            NODELET_ERROR("Channel entry %d is not a struct. Skipping.", i);
            continue;
        }

        ch.name = static_cast<std::string>(entry["name"]);
        std::string in_img  = static_cast<std::string>(entry["in_img"]);
        std::string in_info = static_cast<std::string>(entry["in_info"]);
        std::string out_img = static_cast<std::string>(entry["out_img"]);
        std::string out_info = static_cast<std::string>(entry["out_info"]);

        // Setup message_filters Subscriber
        ch.sub = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh_, in_img, queue_size);

        ch.info_sub = nh_.subscribe<sensor_msgs::CameraInfo>(in_info, 1, 
            [this, i](const sensor_msgs::CameraInfoConstPtr& msg) {
                std::lock_guard<std::mutex> lock(info_mutex_);
                channels_[i].last_info = *msg;
                channels_[i].has_info = true;
                channels_[i].last_info_time = ros::Time::now();
            });

        ch.img_pub  = nh_.advertise<sensor_msgs::Image>(out_img, 1);
        ch.info_pub = nh_.advertise<sensor_msgs::CameraInfo>(out_info, 1);

        if (ch.name == master_name) master_index_ = i;

        NODELET_INFO("Configured Channel [%s]: %s -> %s", ch.name.c_str(), in_img.c_str(), out_img.c_str());
    }

    // 3. Setup Synchronizer (Currently fixed to expected_channel_num inputs for SyncPolicy compatibility)
    if (channels_.size() != expected_channel_num) {
        NODELET_ERROR("Current SyncPolicy only supports %d channels. Detected %zu. Synchronization might fail.", expected_channel_num, channels_.size());
    }

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), 
        *channels_[0].sub, *channels_[1].sub, *channels_[2].sub, *channels_[3].sub
    );
    sync_->setInterMessageLowerBound(ros::Duration(slop));
    sync_->registerCallback(boost::bind(&ImageSynchronizerNodelet::syncCallback, this, _1, _2, _3, _4));

    NODELET_INFO("Image Synchronizer Started. Master: [%s], Slop: %.3fs", channels_[master_index_].name.c_str(), slop);
}

void ImageSynchronizerNodelet::syncCallback(const sensor_msgs::ImageConstPtr& front,
                                            const sensor_msgs::ImageConstPtr& rear,
                                            const sensor_msgs::ImageConstPtr& left,
                                            const sensor_msgs::ImageConstPtr& right) {
    const std::array<sensor_msgs::ImageConstPtr, 4> images = {front, rear, left, right};

    // Use the timestamp from the configured master channel
    const ros::Time& sync_time = images[master_index_]->header.stamp;

    for (size_t i = 0; i < images.size(); ++i) {
        publishWithSync(i, images[i], sync_time);
    }
}

void ImageSynchronizerNodelet::publishWithSync(size_t index, const sensor_msgs::ImageConstPtr& img, const ros::Time& sync_time) {
    auto& ch = channels_[index];

    auto synced_img = boost::make_shared<sensor_msgs::Image>(*img);
    synced_img->header.stamp = sync_time;
    ch.img_pub.publish(synced_img);

    std::lock_guard<std::mutex> lock(info_mutex_);
    if (ch.has_info) {
        if ((ros::Time::now() - ch.last_info_time).toSec() < info_timeout_) {
            auto info = ch.last_info;
            info.header.stamp = sync_time;
            ch.info_pub.publish(info);
        } else {
            NODELET_WARN_THROTTLE(10.0, "[%s] CameraInfo stale (>%.1fs).", ch.name.c_str(), info_timeout_);
        }
    }
}

} // namespace carmaker_image_synchronizer

PLUGINLIB_EXPORT_CLASS(carmaker_image_synchronizer::ImageSynchronizerNodelet, nodelet::Nodelet)
