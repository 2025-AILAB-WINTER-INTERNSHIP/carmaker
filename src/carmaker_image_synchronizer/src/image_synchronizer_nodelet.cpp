#include <carmaker_image_synchronizer/image_synchronizer_nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace carmaker_image_synchronizer {

void ImageSynchronizerNodelet::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    // 1. Load Settings
    std::string master_name;

    pnh_.param<bool>("settings/use_bundle", use_bundle_, false);
    pnh_.param<std::string>("settings/master_channel", master_name, "front");
    pnh_.param<int>("settings/queue_size", queue_size_, 10);
    pnh_.param<double>("settings/sync_slop_sec", slop_, 0.05);
    pnh_.param<double>("settings/info_timeout_sec", info_timeout_, 2.0);

    if (use_bundle_) {
        std::string bundle_topic = pnh_.param<std::string>("settings/bundle_topic", "/synced/bundle");
        bundle_pub_ = nh_.advertise<carmaker_msgs::CameraBundle>(bundle_topic, 1);
        NODELET_INFO("Bundle Output Enabled: %s", bundle_topic.c_str());
    }

    // 2. Setup Diagnostics
    diagnostic_updater_.setHardwareID("carmaker_image_sync");
    diagnostic_updater_.add("Synchronization Status", this, &ImageSynchronizerNodelet::produceDiagnostics);

    // 3. Load Channels from YAML
    XmlRpc::XmlRpcValue channel_list;
    if (!pnh_.getParam("channels", channel_list) || channel_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        NODELET_FATAL("Failed to load 'channels' list from parameters.");
        return;
    }

    // Validation: Check if YAML list size is exactly 4
    if (channel_list.size() != 4) {
        NODELET_FATAL("Channel list size must be exactly 4.");
        return;
    }

    for (int i = 0; i < 4; ++i) {
        XmlRpc::XmlRpcValue& entry = channel_list[i];

        if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            NODELET_FATAL("Channel entry %d is not a struct.", i);
            return;
        }

        auto& ch = channels_[i];

        ch.name = static_cast<std::string>(entry["name"]);
        std::string in_img  = static_cast<std::string>(entry["in_img"]);
        std::string in_info = static_cast<std::string>(entry["in_info"]);
        std::string out_img = static_cast<std::string>(entry["out_img"]);
        std::string out_info = static_cast<std::string>(entry["out_info"]);

        // Setup message_filters Subscriber & Diagnostic Counter
        ch.sub = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh_, in_img, queue_size_);
        ch.sub->registerCallback(boost::bind(&ImageSynchronizerNodelet::imageRawCallback, this, _1, i));

        ch.info_sub = nh_.subscribe<sensor_msgs::CameraInfo>(in_info, 1,
            [this, i](const sensor_msgs::CameraInfoConstPtr& msg) {
                std::lock_guard<std::mutex> lock(info_mutex_);
                channels_[i].last_info = *msg;
                channels_[i].has_info = true;
                channels_[i].last_info_time = ros::Time::now();
            });

        if (!use_bundle_) {
            ch.img_pub  = nh_.advertise<sensor_msgs::Image>(out_img, 1);
            ch.info_pub = nh_.advertise<sensor_msgs::CameraInfo>(out_info, 1);
        }

        if (ch.name == master_name) master_index_ = i;
        NODELET_INFO("Configured Channel [%s]: %s -> %s", ch.name.c_str(), in_img.c_str(), out_img.c_str());
    }

    // 4. Setup Synchronizer
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size_),
        *channels_[0].sub, *channels_[1].sub, *channels_[2].sub, *channels_[3].sub
    );
    sync_->setInterMessageLowerBound(ros::Duration(slop_));
    sync_->registerCallback(boost::bind(&ImageSynchronizerNodelet::syncCallback, this, _1, _2, _3, _4));

    // 5. Setup Diagnostics Timer (1Hz) to ensure diagnostics are sent even if sync stalls
    diag_timer_ = nh_.createTimer(ros::Duration(1.0), &ImageSynchronizerNodelet::timerCallback, this);

    NODELET_INFO("Image Synchronizer with Diagnostics Started. Master: [%s]", channels_[master_index_].name.c_str());
}

void ImageSynchronizerNodelet::imageRawCallback(const sensor_msgs::ImageConstPtr& msg, size_t index) {
    checkTimeJump(msg->header.stamp);
    channels_[index].received_count++;
}

void ImageSynchronizerNodelet::syncCallback(const sensor_msgs::ImageConstPtr& front,
                                            const sensor_msgs::ImageConstPtr& rear,
                                            const sensor_msgs::ImageConstPtr& left,
                                            const sensor_msgs::ImageConstPtr& right) {
    const std::array<sensor_msgs::ImageConstPtr, 4> images = {front, rear, left, right};
    processSyncedImages(images);
}

void ImageSynchronizerNodelet::timerCallback(const ros::TimerEvent& event) {
    diagnostic_updater_.update();
}

void ImageSynchronizerNodelet::resetSynchronizer() {
    NODELET_INFO("Resetting message_filters Synchronizer queue...");

    // 1. Re-instantiate message_filters::Synchronizer
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(queue_size_),
        *channels_[0].sub, *channels_[1].sub, *channels_[2].sub, *channels_[3].sub
    ));
    sync_->setInterMessageLowerBound(ros::Duration(slop_));
    sync_->registerCallback(boost::bind(&ImageSynchronizerNodelet::syncCallback, this, _1, _2, _3, _4));

    // 2. CameraInfo cache time re-anchoring to avoid false stale detection
    {
        std::lock_guard<std::mutex> lock(info_mutex_);
        ros::Time now = ros::Time::now();
        for (auto& ch : channels_) {
            if (ch.has_info) {
                ch.last_info_time = now;
            }
        }
    }

    // 3. Clear stats/counts
    total_synced_count_ = 0;
    for (auto& ch : channels_) {
        ch.received_count = 0;
    }

    // 4. Force diagnostic update immediately
    diagnostic_updater_.force_update();
}

void ImageSynchronizerNodelet::checkTimeJump(const ros::Time& current_time) {
    std::lock_guard<std::mutex> lock(time_mutex_);

    if (last_image_time_.isZero()) {
        last_image_time_ = current_time;
        return;
    }

    double diff = (current_time - last_image_time_).toSec();

    // Detect time jump (backward or significant forward jump)
    if (diff < -1.0 || diff > 5.0) {
        NODELET_WARN("[Time Jump Detected] Diff: %.2f sec. Resetting pipeline...", diff);
        resetSynchronizer();
    }

    last_image_time_ = current_time;
}

bool ImageSynchronizerNodelet::getValidCameraInfo(size_t index, const ros::Time& sync_time, sensor_msgs::CameraInfo& out_info) {
    std::lock_guard<std::mutex> lock(info_mutex_);
    auto& ch = channels_[index];

    if (ch.has_info) {
        if ((ros::Time::now() - ch.last_info_time).toSec() < info_timeout_) {
            out_info = ch.last_info;
            out_info.header.stamp = sync_time;
            return true;
        } else {
            NODELET_WARN_THROTTLE(10.0, "[%s] CameraInfo stale (>%.1fs).", ch.name.c_str(), info_timeout_);
        }
    }
    return false;
}

void ImageSynchronizerNodelet::processSyncedImages(const std::array<sensor_msgs::ImageConstPtr, 4>& images) {
    const ros::Time& sync_time = images[master_index_]->header.stamp;

    // Atomic increment
    total_synced_count_++;

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        for (size_t i = 0; i < 4; ++i) {
            channels_[i].last_slop = (images[i]->header.stamp - sync_time).toSec();
        }
    }

    // 4. Output Selection
    if (use_bundle_) {
        carmaker_msgs::CameraBundle bundle;
        bundle.header.stamp = sync_time;
        bundle.header.frame_id = images[master_index_]->header.frame_id;
        bundle.names.reserve(4);
        bundle.images.reserve(4);
        bundle.infos.reserve(4);

        for (size_t i = 0; i < 4; ++i) {
            // Add Name
            bundle.names.push_back(channels_[i].name);

            // Add Image
            sensor_msgs::Image synced_img = *images[i];
            synced_img.header.stamp = sync_time;
            bundle.images.push_back(std::move(synced_img));

            // Add CameraInfo
            sensor_msgs::CameraInfo info;
            if (getValidCameraInfo(i, sync_time, info)) {
                bundle.infos.push_back(std::move(info));
            } else {
                bundle.infos.emplace_back();
            }
        }
        bundle_pub_.publish(bundle);
    } else {
        // Publish individual topics as before
        for (size_t i = 0; i < 4; ++i) {
            publishWithSync(i, images[i], sync_time);
        }
    }
}

void ImageSynchronizerNodelet::publishWithSync(size_t index, const sensor_msgs::ImageConstPtr& img, const ros::Time& sync_time) {
    auto& ch = channels_[index];

    // Deep copy for timestamp modification
    auto synced_img = boost::make_shared<sensor_msgs::Image>(*img);
    synced_img->header.stamp = sync_time;
    ch.img_pub.publish(synced_img);

    sensor_msgs::CameraInfo info;
    if (getValidCameraInfo(index, sync_time, info)) {
        ch.info_pub.publish(info);
    }
}

void ImageSynchronizerNodelet::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Syncing Active");

    // total_synced_count_ is atomic, no lock needed
    stat.add("Total Synced Groups", total_synced_count_.load());

    // master_index_ is effectively read-only after onInit, but we can protect it if needed.
    // For simplicity, we assume channels_ structure is static after onInit.
    stat.add("Master Channel", channels_[master_index_].name);

    for (const auto& ch : channels_) {
        std::string prefix = "Channel [" + ch.name + "]/";

        // received_count is atomic
        uint64_t raw_received = ch.received_count.load();
        stat.add(prefix + "Raw Received", raw_received);

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            stat.add(prefix + "Last Slop (s)", ch.last_slop);
        }

        double success_rate = (raw_received > 0) ? (double)total_synced_count_.load() / raw_received : 0.0;
        stat.add(prefix + "Approx Sync Rate", success_rate);

        {
            std::lock_guard<std::mutex> lock(info_mutex_);
            if (!ch.has_info) {
                stat.mergeSummary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Missing CameraInfo on " + ch.name);
            } else if ((ros::Time::now() - ch.last_info_time).toSec() > info_timeout_) {
                stat.mergeSummary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Stale CameraInfo on " + ch.name);
            }
        }
    }
}

} // namespace carmaker_image_synchronizer

PLUGINLIB_EXPORT_CLASS(carmaker_image_synchronizer::ImageSynchronizerNodelet, nodelet::Nodelet)
