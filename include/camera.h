//
// Converted to ROS1 (Noetic) from ROS2 version
//

#pragma once

#include <utility>
#include <cstdint>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_eigen/tf2_eigen.h> 

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <deque>
#include <mutex>


namespace color_point_cloud {

class CameraType {
public:
    CameraType(std::string image_topic, std::string camera_info_topic)
        : image_topic_(std::move(image_topic)),
          camera_info_topic_(std::move(camera_info_topic)),
          is_info_initialized_(false),
          is_transform_initialized_(false),
          is_map_initialized_(false),
          image_width_(0),
          image_height_(0)
    {}

    void set_cv_image(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, get_image_msg() ? get_image_msg()->encoding : std::string());
        } catch (const cv_bridge::Exception& e) {
            ROS_WARN_STREAM("cv_bridge exception in set_cv_image: " << e.what());
            return;
        }

        try {
            cv::fisheye::undistortImage(cv_ptr->image, cv_image_, get_camera_matrix_cv(), get_distortion_matrix_cv());
        } catch (const std::exception& e) {
            ROS_WARN_STREAM("Undistort exception: " << e.what());
        }
    }

    void set_cv_image_from_compressed(const sensor_msgs::CompressedImageConstPtr& msg) {
        if (!msg) {
            ROS_WARN("Compressed image message is null");
            return;
        }
        try {
            cv::Mat compressed(1, static_cast<int>(msg->data.size()), CV_8UC1,
                               const_cast<unsigned char*>(msg->data.data()));
            cv_image_ = cv::imdecode(compressed, cv::IMREAD_COLOR);
            cv::imwrite("/dataset/ros1/1/0.jpg",cv_image_);

            if (cv_image_.empty()) {
                ROS_WARN("Failed to decode compressed image");
                return;
            }
            if (!is_map_initialized_) {
                ROS_WARN("Undistortion map is not initialized yet");
                return;
            }

            // cv::Mat undistorted;
            // cv::remap(cv_image_, undistorted, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            // cv_image_ = undistorted;

            // cv::fisheye::undistortImage(cv_image_, cv_image_, get_camera_matrix_cv(), get_distortion_matrix_cv());
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM("Exception during decoding or undistortion: " << e.what());
        }
    }

    void set_image_msg(const sensor_msgs::ImageConstPtr& msg) { image_msg_ = msg; }
    void set_compressed_image_msg(const sensor_msgs::CompressedImageConstPtr& msg) { compressed_image_msg_ = msg; }
    void set_camera_info(const sensor_msgs::CameraInfoConstPtr& msg) { camera_info_ = msg; }

    cv::Mat get_cv_image() { return cv_image_; }

    sensor_msgs::ImageConstPtr get_image_msg() { return image_msg_; }
    sensor_msgs::CompressedImageConstPtr get_compressed_image_msg() { return compressed_image_msg_; }
    sensor_msgs::CameraInfoConstPtr get_camera_info() { return camera_info_; }

    bool is_info_initialized() const { return is_info_initialized_; }
    bool is_transform_initialized() const { return is_transform_initialized_; }

    void set_camera_utils(const sensor_msgs::CameraInfoConstPtr& msg) {
        if (!msg) {
            ROS_WARN("CameraInfo is null in set_camera_utils");
            return;
        }

        camera_frame_id_  = msg->header.frame_id;
        distortion_model_ = msg->distortion_model;

        image_width_  = msg->width;
        image_height_ = msg->height;

        camera_matrix_.setZero();
        camera_matrix_(0,0) = msg->K[0];
        camera_matrix_(0,2) = msg->K[2];
        camera_matrix_(1,1) = msg->K[4];
        camera_matrix_(1,2) = msg->K[5];
        camera_matrix_(2,2) = 1.0;

        camera_matrix_cv_ = (cv::Mat_<double>(3,3)
                             << msg->K[0], 0.0,      msg->K[2],
                                0.0,      msg->K[4], msg->K[5],
                                0.0,      0.0,       1.0);

        rectification_matrix_ << msg->R[0], msg->R[1], msg->R[2],
                                 msg->R[3], msg->R[4], msg->R[5],
                                 msg->R[6], msg->R[7], msg->R[8];

        projection_matrix_.setZero();
        projection_matrix_(0,0) = msg->P[0];
        projection_matrix_(0,2) = msg->P[2];
        projection_matrix_(1,1) = msg->P[5];
        projection_matrix_(1,2) = msg->P[6];
        projection_matrix_(2,2) = 1.0;

        if (msg->D.size() >= 4) {
            distortion_matrix_(0,0) = msg->D[0];
            distortion_matrix_(0,1) = msg->D[1];
            distortion_matrix_(0,2) = msg->D[2];
            distortion_matrix_(0,3) = msg->D[3];

            distortion_matrix_cv_ = (cv::Mat_<double>(1,4) << msg->D[0], msg->D[1], msg->D[2], msg->D[3]);
        } else {
            distortion_matrix_.setZero();
            distortion_matrix_cv_ = cv::Mat::zeros(1,4,CV_64F);
            ROS_WARN("CameraInfo.D has fewer than 4 elements; padded with zeros");
        }

        cv::Size image_size(static_cast<int>(image_width_), static_cast<int>(image_height_));
        cv::Mat R = cv::Mat::eye(3,3,CV_64F);

        // cv::fisheye::initUndistortRectifyMap(
        //     camera_matrix_cv_,     
        //     distortion_matrix_cv_, 
        //     R,                  
        //     camera_matrix_cv_,  
        //     image_size,
        //     CV_16SC2,
        //     map1, map2
        // );

        is_map_initialized_  = true;
        is_info_initialized_ = true;
    }

    void set_lidar_to_camera_matrix(geometry_msgs::TransformStamped& msg) {
        Eigen::Affine3d affine = tf2::transformToEigen(msg);
        lidar_to_camera_matrix_ = affine.matrix();
        is_transform_initialized_ = true;
    }

    void set_lidar_to_camera_matrix_xyzrpy_rad(double x, double y, double z,
                                               double roll, double pitch, double yaw)
    {
        const Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());

        const Eigen::Matrix3d R = (Rz * Ry * Rx).toRotationMatrix();

        lidar_to_camera_matrix_.setIdentity();
        lidar_to_camera_matrix_.block<3, 3>(0, 0) = R;
        lidar_to_camera_matrix_.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

        is_transform_initialized_ = true;
    }

    void set_lidar_to_camera_projection_matrix() {
        Eigen::Matrix4d T_lc_inv = lidar_to_camera_matrix_;
        lidar_to_camera_projection_matrix_ = projection_matrix_ * T_lc_inv.block<4,4>(0,0);
    }

    std::string get_image_topic()       { return image_topic_; }
    std::string get_camera_info_topic() { return camera_info_topic_; }
    std::string get_camera_frame_id()   { return camera_frame_id_; }
    std::string get_distortion_model()  { return distortion_model_; }

    double get_image_width()  const { return image_width_; }
    double get_image_height() const { return image_height_; }

    Eigen::Matrix<double,3,3> get_camera_matrix()               { return camera_matrix_; }
    cv::Mat                   get_camera_matrix_cv()             { return camera_matrix_cv_; }
    Eigen::Matrix<double,3,3> get_rectification_matrix()         { return rectification_matrix_; }
    Eigen::Matrix<double,3,4> get_projection_matrix()            { return projection_matrix_; }
    Eigen::Matrix<double,1,4> get_distortion_matrix()            { return distortion_matrix_; }
    cv::Mat                   get_distortion_matrix_cv()         { return distortion_matrix_cv_; }
    Eigen::Matrix4d           get_lidar_to_camera_matrix()       { return lidar_to_camera_matrix_; }
    Eigen::Matrix<double,3,4> get_lidar_to_camera_projection_matrix() { return lidar_to_camera_projection_matrix_; }

    // --- Image buffer APIs (thread-safe) ---
    void push_keep_all(const sensor_msgs::CompressedImagePtr &img)
    {
        compressed_buffer_.push_back(img); 
    }
    size_t buf_size() const
    {
        std::lock_guard<std::mutex> lk(buf_mtx_);
        return compressed_buffer_.size();
    }
    sensor_msgs::CompressedImageConstPtr buf_front() const
    {
        std::lock_guard<std::mutex> lk(buf_mtx_);
        if (compressed_buffer_.empty())
            return nullptr;
        return compressed_buffer_.front();
    }
    sensor_msgs::CompressedImageConstPtr buf_back() const
    {
        std::lock_guard<std::mutex> lk(buf_mtx_);
        if (compressed_buffer_.empty())
            return nullptr;
        return compressed_buffer_.back();
    }
    double buf_front_time() const
    {
        auto m = buf_front();
        return m ? m->header.stamp.toSec() : std::numeric_limits<double>::quiet_NaN();
    }
    double buf_back_time() const
    {
        auto m = buf_back();
        return m ? m->header.stamp.toSec() : std::numeric_limits<double>::quiet_NaN();
    }
    sensor_msgs::CompressedImageConstPtr buf_at(size_t i) const
    {
        std::lock_guard<std::mutex> lk(buf_mtx_);
        if (i >= compressed_buffer_.size())
            return nullptr;
        return compressed_buffer_[i];
    }
    void erase_front_n(size_t n)
    {
        std::lock_guard<std::mutex> lk(buf_mtx_);
        if (n >= compressed_buffer_.size())
        {
            compressed_buffer_.clear();
            return;
        }
        compressed_buffer_.erase(compressed_buffer_.begin(), compressed_buffer_.begin() + static_cast<long>(n));
    }
    sensor_msgs::CompressedImageConstPtr pop_front_one()
    {
        std::lock_guard<std::mutex> lk(buf_mtx_);
        if (compressed_buffer_.empty())
            return nullptr;
        auto out = compressed_buffer_.front();
        compressed_buffer_.pop_front();
        return out;
    }

private:
    std::string image_topic_;
    std::string camera_info_topic_;

    cv::Mat cv_image_;

    sensor_msgs::ImageConstPtr          image_msg_;
    sensor_msgs::CompressedImageConstPtr compressed_image_msg_;
    sensor_msgs::CameraInfoConstPtr     camera_info_;

    bool is_info_initialized_;
    bool is_transform_initialized_;
    bool is_map_initialized_;

    std::string camera_frame_id_;
    std::string distortion_model_;

    double image_width_;
    double image_height_;

    Eigen::Matrix<double,3,3> camera_matrix_;
    Eigen::Matrix<double,3,3> rectification_matrix_;
    Eigen::Matrix<double,3,4> projection_matrix_;
    Eigen::Matrix<double,1,4> distortion_matrix_;

    cv::Mat camera_matrix_cv_;
    cv::Mat distortion_matrix_cv_;

    Eigen::Matrix4d lidar_to_camera_matrix_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,3,4> lidar_to_camera_projection_matrix_;

    cv::Mat map1, map2;

    mutable std::mutex buf_mtx_;
    std::deque<sensor_msgs::CompressedImageConstPtr> compressed_buffer_;

};

typedef std::shared_ptr<CameraType>       CameraTypePtr;
typedef std::shared_ptr<const CameraType> CameraTypeConstPtr;

} // namespace color_point_cloud