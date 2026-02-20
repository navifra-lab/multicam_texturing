#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <vector>
#include <algorithm>
#include <chrono>
#include <memory>

// ==========================================
// 디버깅 설정
// ==========================================
bool DEBUG_BYPASS_SYNC = false; 
double SYNC_TOLERANCE = 0.050; 

std::vector<double> lidar_timestamps;

class H264SmartDecoder {
public:
    H264SmartDecoder(ros::NodeHandle& nh, int id) 
        : id_(id), msg_recv_count_(0), pub_count_(0) {
        
        std::string input_topic = "/cam_" + std::to_string(id) + "/image_raw/compressed";
        std::string output_topic = "/cam_" + std::to_string(id) + "/image_raw";

        sub_ = nh.subscribe<sensor_msgs::CompressedImage>(
            input_topic, 20, &H264SmartDecoder::msgCallback, this);
        pub_ = nh.advertise<sensor_msgs::Image>(output_topic, 10);

        initGStreamer();
        ROS_INFO("\033[1;34m[Cam %d] Decoder Ready.\033[0m", id);
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
        // Annex B 포맷(byte-stream)을 위한 파이프라인 설정
        std::string pipe_str = 
            "appsrc name=mysrc_" + std::to_string(id_) + 
            " caps=\"video/x-h264, stream-format=(string)byte-stream, alignment=(string)nal\""
            " format=time is-live=false ! "
            "h264parse ! avdec_h264 ! videoconvert ! "
            "video/x-raw, format=BGR ! appsink name=mysink_" + std::to_string(id_) + 
            " emit-signals=false sync=false max-buffers=2 drop=false";
        
        pipeline_ = gst_parse_launch(pipe_str.c_str(), &err);
        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), ("mysrc_" + std::to_string(id_)).c_str());
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), ("mysink_" + std::to_string(id_)).c_str());
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    bool checkSync(double img_time, double& diff) {
        if (DEBUG_BYPASS_SYNC) return true;
        if (lidar_timestamps.empty()) return false;
        auto it = std::lower_bound(lidar_timestamps.begin(), lidar_timestamps.end(), img_time);
        double d1 = 999.0, d2 = 999.0;
        if (it != lidar_timestamps.end()) d1 = std::abs(*it - img_time);
        if (it != lidar_timestamps.begin()) d2 = std::abs(*(it - 1) - img_time);
        diff = std::min(d1, d2);
        return (diff <= SYNC_TOLERANCE);
    }

    void msgCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        msg_recv_count_++;

        // 1. 버퍼 주입 (Annex B 형식은 PTS를 주지 않아도 parse에서 처리 가능)
        GstBuffer* buffer = gst_buffer_new_allocate(NULL, msg->data.size(), NULL);
        gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
        GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;

        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);

        // 2. 샘플 추출 (동기 모드이므로 10ms 정도만 대기)
        GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 10 * GST_MSECOND);
        
        if (sample) {
            double img_t = msg->header.stamp.toSec();
            double offset = 0;

            if (checkSync(img_t, offset)) {
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
                pub_count_++;

                if (pub_count_ % 30 == 1) {
                    ROS_INFO("\033[1;32m[Cam %d] Successfully Published (Offset: %.1fms)\033[0m", id_, offset*1000.0);
                }
                gst_buffer_unmap(out_buf, &map);
            }
            gst_sample_unref(sample);
        } else {
            if (msg_recv_count_ % 60 == 1) {
                ROS_WARN("[Cam %d] Waiting for first Keyframe in byte-stream...", id_);
            }
        }
    }

    int id_, msg_recv_count_, pub_count_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    GstElement *pipeline_, *appsrc_, *appsink_;
};

void loadLidarTimestamps(const std::string& bag_path) {
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("/hrz/points"));
    for (const rosbag::MessageInstance& m : view) lidar_timestamps.push_back(m.getTime().toSec());
    bag.close();
    std::sort(lidar_timestamps.begin(), lidar_timestamps.end());
    ROS_INFO("LiDAR Frames: %lu", lidar_timestamps.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "h264_bridge_debug_node");
    ros::NodeHandle nh("~");
    gst_init(&argc, &argv);
    if (argc < 2) return 1;
    loadLidarTimestamps(argv[1]);
    std::vector<std::shared_ptr<H264SmartDecoder>> decoders;
    for(int i = 0; i < 6; ++i) decoders.push_back(std::make_shared<H264SmartDecoder>(nh, i));
    ros::AsyncSpinner spinner(8);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}