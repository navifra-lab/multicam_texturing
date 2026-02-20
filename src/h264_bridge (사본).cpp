#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

std::vector<double> lidar_timestamps;

const double SYNC_TOLERANCE = 0.015; 

std::string get_pipeline_string() {
    return "appsrc name=mysource format=time is-live=false do-timestamp=true ! "
           "h264parse ! avdec_h264 ! videoconvert ! "
           "video/x-raw, format=BGR ! appsink name=mysink emit-signals=false sync=false max-buffers=1 drop=false";
}

class H264SmartDecoder {
public:
    H264SmartDecoder(ros::NodeHandle& nh, int id) 
        : id_(id), pub_count_(0), last_pub_time_(0.0) {
        
        std::string input_topic = "/cam" + std::to_string(id) + "/image_raw/h264";
        std::string output_topic = "/cam" + std::to_string(id) + "/image_raw";

        sub_ = nh.subscribe<sensor_msgs::CompressedImage>(
            input_topic, 100, &H264SmartDecoder::msgCallback, this);

        pub_ = nh.advertise<sensor_msgs::Image>(output_topic, 10);

        initGStreamer();
        ROS_INFO("Smart Decoder initialized for Cam %d", id);
    }

    ~H264SmartDecoder() {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

private:
    void initGStreamer() {
        GError* err = nullptr;
        std::string pipe_str = get_pipeline_string();
        pipeline_ = gst_parse_launch(pipe_str.c_str(), &err);
        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysource");
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysink");
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    bool getSyncedLidarDiff(double img_time, double& out_diff) {
        auto it = std::lower_bound(lidar_timestamps.begin(), lidar_timestamps.end(), img_time);
        
        if (it != lidar_timestamps.end()) {
            double diff = *it - img_time;
            if (std::abs(diff) <= SYNC_TOLERANCE) {
                out_diff = diff;
                return true;
            }
        }
        if (it != lidar_timestamps.begin()) {
            double diff = *(it - 1) - img_time;
            if (std::abs(diff) <= SYNC_TOLERANCE) {
                out_diff = diff;
                return true;
            }
        }
        return false;
    }

    void msgCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        auto start = std::chrono::high_resolution_clock::now();

        GstBuffer* buffer = gst_buffer_new_allocate(NULL, msg->data.size(), NULL);
        gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
        GST_BUFFER_PTS(buffer) = msg->header.stamp.toNSec(); 

        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);

        GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 100 * GST_MSECOND);
        
        if (sample) {
            double current_img_time = msg->header.stamp.toSec();
            double sync_diff_sec = 0.0;

            if (getSyncedLidarDiff(current_img_time, sync_diff_sec)) {
                GstBuffer* out_buf = gst_sample_get_buffer(sample);
                GstMapInfo map;
                gst_buffer_map(out_buf, &map, GST_MAP_READ);

                sensor_msgs::Image img_msg;
                img_msg.header = msg->header;
                img_msg.height = 1200;
                img_msg.width = 1920;
                img_msg.encoding = "bgr8";
                img_msg.step = 1920 * 3;
                img_msg.data.resize(map.size);
                memcpy(img_msg.data.data(), map.data, map.size);

                pub_.publish(img_msg);

                gst_buffer_unmap(out_buf, &map);

                auto end = std::chrono::high_resolution_clock::now();
                double proc_ms = std::chrono::duration<double, std::milli>(end - start).count();
                
                double now_sec = ros::WallTime::now().toSec();
                double interval_ms = 0.0;
                if (last_pub_time_ != 0.0) {
                    interval_ms = (now_sec - last_pub_time_) * 1000.0;
                }
                last_pub_time_ = now_sec;
                pub_count_++;

                std::string color = "\033[1;32m"; // Green
                double sync_diff_ms = sync_diff_sec * 1000.0;

                if (pub_count_ > 1 && (interval_ms < 85.0 || interval_ms > 115.0)) {
                    color = "\033[1;31m"; // Red (Interval Jitter)
                }
                
                ROS_INFO("%s[Cam %d] Seq: %d | Proc: %.2f ms | Offset: %.2f ms\033[0m", 
                         color.c_str(), id_, pub_count_, proc_ms, sync_diff_ms);
            }

            gst_sample_unref(sample);
        }
    }

    int id_;
    int pub_count_;
    double last_pub_time_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    GstElement *pipeline_ = nullptr;
    GstElement *appsrc_ = nullptr;
    GstElement *appsink_ = nullptr;
};

void loadLidarTimestamps(const std::string& bag_path) {
    ROS_INFO("Loading LiDAR timestamps from: %s", bag_path.c_str());
    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to open bag file: %s", e.what());
        return;
    }

    std::vector<std::string> topics;
    topics.push_back("/hrz/points"); 

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    lidar_timestamps.reserve(view.size());
    for (const rosbag::MessageInstance& m : view) {
        lidar_timestamps.push_back(m.getTime().toSec());
    }
    bag.close();

    std::sort(lidar_timestamps.begin(), lidar_timestamps.end());
    ROS_INFO("Loaded %lu LiDAR frames.", lidar_timestamps.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "h264_bridge_smart_node");
    ros::NodeHandle nh("~");
    gst_init(&argc, &argv);

    if (argc < 2) {
        ROS_ERROR("Usage: rosrun <pkg> h264_bridge_smart <path_to_bag_file>");
        return 1;
    }
    std::string bag_path = argv[1];

    loadLidarTimestamps(bag_path);

    std::vector<int> target_cams = {2, 3, 4, 5};
    std::vector<std::shared_ptr<H264SmartDecoder>> decoders;

    ROS_INFO("Launching Smart Bridge (Target 10Hz Sync)...");

    for(int cam_id : target_cams) {
        decoders.push_back(std::make_shared<H264SmartDecoder>(nh, cam_id));
    }

    ros::AsyncSpinner spinner(4); 
    spinner.start();
    
    ros::waitForShutdown();

    return 0;
}