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

    // 2. Setup Diagnostics
    diagnostic_updater_.setHardwareID("carmaker_image_sync");
    diagnostic_updater_.add("Synchronization Status", this, &ImageSynchronizerNodelet::produceDiagnostics);

    // 3. Load Channels from YAML
    XmlRpc::XmlRpcValue channel_list;
    if (!pnh_.getParam("channels", channel_list) || channel_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        NODELET_FATAL("Failed to load 'channels' list from parameters.");
        return;
    }

    // Validation: Check if YAML list size matches expected_channel_num
    if (channel_list.size() != static_cast<size_t>(expected_channel_num)) {
        NODELET_FATAL("Channel list size (%zu) does not match expected_channel_num (%d).", 
                      channel_list.size(), expected_channel_num);
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

        // Setup message_filters Subscriber & Diagnostic Counter
        ch.sub = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh_, in_img, queue_size);
        ch.sub->registerCallback(boost::bind(&ImageSynchronizerNodelet::imageRawCallback, this, _1, i));

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

    // 4. Setup Synchronizer (Currently fixed to 4 inputs for SyncPolicy compatibility)
    if (channels_.size() != 4) {
        NODELET_ERROR("Current SyncPolicy only supports 4 channels. Detected %zu. Synchronization might fail.", channels_.size());
    }

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), 
        *channels_[0].sub, *channels_[1].sub, *channels_[2].sub, *channels_[3].sub
    );
    sync_->setInterMessageLowerBound(ros::Duration(slop));
    sync_->registerCallback(boost::bind(&ImageSynchronizerNodelet::syncCallback, this, _1, _2, _3, _4));

    NODELET_INFO("Image Synchronizer with Diagnostics Started. Master: [%s]", channels_[master_index_].name.c_str());
}

void ImageSynchronizerNodelet::imageRawCallback(const sensor_msgs::ImageConstPtr& msg, size_t index) {
    channels_[index].received_count++;
}

void ImageSynchronizerNodelet::syncCallback(const sensor_msgs::ImageConstPtr& front,
                                            const sensor_msgs::ImageConstPtr& rear,
                                            const sensor_msgs::ImageConstPtr& left,
                                            const sensor_msgs::ImageConstPtr& right) {
    const std::array<sensor_msgs::ImageConstPtr, 4> images = {front, rear, left, right};
    const ros::Time& sync_time = images[master_index_]->header.stamp;

    total_synced_count_++;

    for (size_t i = 0; i < images.size(); ++i) {
        // Record slop for diagnostics
        channels_[i].last_slop = (images[i]->header.stamp - sync_time).toSec();
        publishWithSync(i, images[i], sync_time);
    }
    
    diagnostic_updater_.update();
}

void ImageSynchronizerNodelet::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Syncing Active");
    stat.add("Total Synced Groups", total_synced_count_);
    stat.add("Master Channel", channels_[master_index_].name);

    for (const auto& ch : channels_) {
        std::string prefix = "Channel [" + ch.name + "]/";
        stat.add(prefix + "Raw Received", ch.received_count);
        stat.add(prefix + "Last Slop (s)", ch.last_slop);
        
        double success_rate = (ch.received_count > 0) ? (double)total_synced_count_ / ch.received_count : 0.0;
        stat.add(prefix + "Approx Sync Rate", success_rate);

        if (!ch.has_info) {
            stat.mergeSummary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Missing CameraInfo on " + ch.name);
        } else if ((ros::Time::now() - ch.last_info_time).toSec() > info_timeout_) {
            stat.mergeSummary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Stale CameraInfo on " + ch.name);
        }
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
