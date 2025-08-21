#include "utility.h"
#include "pointcloudType.h"
#include "camera.h"
#include "lio_sam/cloud_info.h"
#include "lio_sam/save_map.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include "pcl/registration/gicp.h"

using namespace gtsam;
// int imgidx =0;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

class mapOptimization : public ParamServer
{

public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge;

    ros::Publisher pubSLAMInfo;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subLoop;

    ros::ServiceServer srvSaveMap;

    std::deque<nav_msgs::Odometry> gpsQueue;
    lio_sam::cloud_info cloudInfo;

    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> surfCloudKeyFrames;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fullcornerCloudKeyFrames;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fullsurfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudCornerLast;       // corner feature set from odoOptimization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudSurfLast;         // surf feature set from odoOptimization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudCornerLastDS;     // downsampled corner feature set from odoOptimization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudSurfLastDS;       // downsampled surf feature set from odoOptimization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fulllaserCloudCornerLastDS; // downsampled corner feature set from odoOptimization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fulllaserCloudSurfLastDS;   // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterCornerRGB;
    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterSurfRGB;
    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterCornerRGB2;
    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterSurfRGB2;

    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer; // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    deque<std_msgs::Float64MultiArray> loopInfoVec;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    std::map<std::string, color_point_cloud::CameraTypePtr> camera_type_stdmap_;
    std::vector<ros::Subscriber> image_subscribers_;
    ros::Timer camera_timer_;
    std::string saveNodePCDDirectory;

    struct CamSyncState
    {
        bool synced = false;     
        double phase_offset = 0.0; // image_time - lidar_tim
        double last_img_time = std::numeric_limits<double>::quiet_NaN();
    };

    std::map<std::string, CamSyncState> cam_states_;
    std::map<std::string, std::mutex> cam_mtx_;

    bool all_cams_synced_ = false;

    const double kPeriodSec = 0.1;
    const double kInitMatchThreshSec = 0.03; // 초기 1회 동기화 허용 오차
    const double kUseThreshSec = 0.035;      // "사용" 허용 오차 (각 스캔마다 체크)
    const double kResyncThreshSec = 0.06;    // 드리프트 감지 시 재동기화 임계
    const size_t kMaxBuf = 200;

    bool strict_all_cameras_ = true;      // true면 3대 모두 통과해야 사용
    bool drop_lidar_if_mismatch_ = false; // true면 이미지 불일치 시 스캔 자체를 건너뜀

    //    void camera_callback(const ros::TimerEvent & /*event*/)
    //    {
    //        std::for_each(
    //            camera_type_stdmap_.begin(), camera_type_stdmap_.end(),
    //            [this](const std::pair<std::string, color_point_cloud::CameraTypePtr> &pair)
    //            {
    //                const auto &cam = pair.second;

    //                if (!cam->get_compressed_image_msg() || !cam->get_camera_info())
    //                    return;

    //                // if (!cam->is_info_initialized())
    //                // {
    //                //     ROS_INFO("Camera info is setting: %s", pair.first.c_str());
    //                //     cam->set_camera_utils(cam->get_camera_info());
    //                // }

    //                // if (!cam->is_transform_initialized() && cam->is_info_initialized())
    //                // {
    //                //     std::optional<geometry_msgs::TransformStamped> transform =
    //                //         (*transform_provider_ptr_)(cam->get_camera_frame_id(), lidarFrame);

    //                //     if (!transform.has_value())
    //                //         return;

    //                //     // cam->set_lidar_to_camera_matrix(transform.value());
    //                //     // cam->set_lidar_to_camera_projection_matrix();
    //                // }
    //            });
    //    }

    sensor_msgs::CompressedImageConstPtr initialSyncAndPop(
        const std::string &cam_key, double lidar_time)
    {
        std::lock_guard<std::mutex> lk(cam_mtx_[cam_key]);
        auto &st = cam_states_[cam_key];
        auto cam = camera_type_stdmap_[cam_key];
        if (!cam || cam->buf_size() == 0)
            return nullptr;

        const double t_front = cam->buf_front_time();
        const double t_back = cam->buf_back_time();
        if (std::isfinite(t_front) && lidar_time < t_front - kInitMatchThreshSec)
            return nullptr;
        if (std::isfinite(t_back) && lidar_time > t_back + kInitMatchThreshSec)
            return nullptr;

        size_t n = cam->buf_size();
        size_t idx_ge = n; // first index with t >= lidar_time

        for (size_t i = 0; i < n; ++i)
        {
            auto mi = cam->buf_at(i);
            if (!mi)
                break;
            double t = mi->header.stamp.toSec();
            if (t >= lidar_time)
            {
                idx_ge = i;
                break;
            }
        }

        size_t cand0 = (idx_ge > 0) ? (idx_ge - 1) : n;
        size_t cand1 = (idx_ge < n) ? idx_ge : n;      

        size_t best_idx = n;
        double best_abs = std::numeric_limits<double>::infinity();

        auto consider = [&](size_t i)
        {
            if (i >= n)
                return;
            auto mi = cam->buf_at(i);
            if (!mi)
                return;
            double d = std::fabs(mi->header.stamp.toSec() - lidar_time);
            double ti = mi->header.stamp.toSec();
            if (d < best_abs || (d == best_abs && ti <= lidar_time))
            {
                best_abs = d;
                best_idx = i;
            }
        };

        consider(cand0);
        consider(cand1);

        if (!std::isfinite(best_abs) || best_abs > kInitMatchThreshSec)
            return nullptr;

        auto out = cam->buf_at(best_idx);
        cam->erase_front_n(best_idx + 1);

        st.synced = true;
        st.phase_offset = out->header.stamp.toSec() - lidar_time;
        st.last_img_time = out->header.stamp.toSec();
        return out;
    }

    // sensor_msgs::CompressedImageConstPtr popNextFIFO(
    //     const std::string &cam_key, double lidar_time)
    // {
    //     std::lock_guard<std::mutex> lk(cam_mtx_[cam_key]);
    //     auto &st = cam_states_[cam_key];
    //     auto cam = camera_type_stdmap_[cam_key];
    //     if (!cam)
    //         return nullptr;

    //     auto front = cam->buf_front();
    //     if (!front)
    //         return nullptr;

    //     const double target = lidar_time + st.phase_offset;
    //     const double t0 = front->header.stamp.toSec();

    //     if (std::fabs(t0 - target) > kResyncThreshSec)
    //     {
    //         st.synced = false;
    //         return nullptr;
    //     }

    //     auto out = cam->pop_front_one();
    //     st.last_img_time = out ? out->header.stamp.toSec() : st.last_img_time;
    //     return out;
    // }

    // sensor_msgs::CompressedImageConstPtr popNextFIFO(const std::string &cam_key, double lidar_time)
    // {
    //     std::lock_guard<std::mutex> lk(cam_mtx_[cam_key]);
    //     auto &st = cam_states_[cam_key];
    //     auto cam = camera_type_stdmap_[cam_key];
    //     if (!cam)
    //         return nullptr;

    //     const double target = lidar_time + st.phase_offset;

    //     while (cam->buf_size() >= 2)
    //     {
    //         auto f0 = cam->buf_at(0);
    //         auto f1 = cam->buf_at(1);
    //         if (!f0 || !f1)
    //             break;
    //         double d0 = std::fabs(f0->header.stamp.toSec() - target);
    //         double d1 = std::fabs(f1->header.stamp.toSec() - target);
    //         if (d1 <= d0)
    //             cam->pop_front_one(); 
    //         else
    //             break;
    //     }

    //     auto out = cam->pop_front_one();
    //     if (out)
    //         st.last_img_time = out->header.stamp.toSec();
    //     return out;
    // }

    sensor_msgs::CompressedImageConstPtr popNextFIFO(const std::string &cam_key, double lidar_time)
    {
        std::lock_guard<std::mutex> lk(cam_mtx_[cam_key]);
        auto &st = cam_states_[cam_key];
        auto cam = camera_type_stdmap_[cam_key];
        if (!cam)
            return nullptr;

        const double target = lidar_time + st.phase_offset;

        // Fast path: 맨 앞 프레임이 충분히 가까우면 바로 소비
        if (cam->buf_size() >= 1)
        {
            auto f0 = cam->buf_at(0);
            if (f0)
            {
                double e0 = std::fabs(f0->header.stamp.toSec() - target);
                if (e0 <= kUseThreshSec)
                {
                    auto out = cam->pop_front_one();
                    st.last_img_time = out->header.stamp.toSec();
                    return out;
                }
            }
        }

        // Fallback: 앞 두 장만 비교해 더 가까워지면 한 장 버림
        while (cam->buf_size() >= 2)
        {
            auto f0 = cam->buf_at(0), f1 = cam->buf_at(1);
            if (!f0 || !f1)
                break;
            double d0 = std::fabs(f0->header.stamp.toSec() - target);
            double d1 = std::fabs(f1->header.stamp.toSec() - target);
            if (d1 <= d0)
                cam->pop_front_one();
            else
                break;
        }

        auto out = cam->pop_front_one();
        if (out)
            st.last_img_time = out->header.stamp.toSec();
        return out;
    }

    sensor_msgs::CompressedImageConstPtr getImageForLidar(
        const std::string &cam_key, double lidar_time)
    {
        auto &st = cam_states_[cam_key];
        if (!st.synced)
            return initialSyncAndPop(cam_key, lidar_time);
        return popNextFIFO(cam_key, lidar_time);
    }

    mapOptimization()
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 1);
        pubPath = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);

        subCloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS = nh.subscribe<nav_msgs::Odometry>(gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subLoop = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);
        pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

        pubSLAMInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/mapping/slam_info", 1);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        downSizeFilterCornerRGB.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurfRGB.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

        downSizeFilterCornerRGB2.setLeafSize(saveLeafSize, saveLeafSize, saveLeafSize);
        downSizeFilterSurfRGB2.setLeafSize(saveLeafSize, saveLeafSize, saveLeafSize);

        for (const auto &camera_topic : cameraTopics)
        {
            const std::string image_topic = camera_topic + imageTopicLastName;
            const std::string camera_info_topic = camera_topic + cameraInfoTopicLastName;

            color_point_cloud::CameraTypePtr camera_type_ptr = std::make_shared<color_point_cloud::CameraType>(image_topic, camera_info_topic);
            camera_type_stdmap_[camera_topic] = camera_type_ptr;

            image_subscribers_.push_back(
                nh.subscribe<sensor_msgs::CompressedImage>(
                    image_topic, 100,
                    [this, camera_topic](const sensor_msgs::CompressedImageConstPtr &msg)
                    {
                        std::lock_guard<std::mutex> lk(cam_mtx_[camera_topic]); 
                        auto it = camera_type_stdmap_.find(camera_topic);
                        if (it != camera_type_stdmap_.end())
                            it->second->push_compressed(msg, kMaxBuf);
                    }));

            sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo);

            std::string frame_id;
            int width = 0, height = 0;
            std::vector<double> K_vec, D_vec, P_vec;

            if (!nh.getParam(camera_topic + "/frame_id", frame_id))
            {
                ROS_ERROR("[%s] Missing param: frame_id", camera_topic.c_str());
                continue;
            }
            nh.param(camera_topic + "/width", width, 0);
            nh.param(camera_topic + "/height", height, 0);
            if (!nh.getParam(camera_topic + "/K", K_vec))
            {
                ROS_ERROR("[%s] Missing param: K (size 9)", camera_topic.c_str());
                continue;
            }
            if (!nh.getParam(camera_topic + "/D", D_vec))
            {
                ROS_ERROR("[%s] Missing param: D (fisheye: size 4, radtan: size 5 등)", camera_topic.c_str());
                continue;
            }
            if (!nh.getParam(camera_topic + "/P", P_vec))
            {
                P_vec.resize(12, 0.0);
                if (K_vec.size() == 9)
                {
                    P_vec[0] = K_vec[0]; // fx
                    P_vec[2] = K_vec[2]; // cx
                    P_vec[5] = K_vec[4]; // fy
                    P_vec[6] = K_vec[5]; // cy
                    P_vec[10] = 1.0;
                }
            }

            if (K_vec.size() != 9)
            {
                ROS_ERROR("[%s] Camera matrix K must have 9 elements!", camera_topic.c_str());
                continue;
            }
            if (P_vec.size() != 12)
            {
                ROS_ERROR("[%s] Projection matrix P must have 12 elements!", camera_topic.c_str());
                continue;
            }

            cam_info->header.frame_id = frame_id;
            cam_info->width = static_cast<uint32_t>(width);
            cam_info->height = static_cast<uint32_t>(height);

            std::string distortion_model = "equidistant";
            nh.param(camera_topic + "/distortion_model", distortion_model, distortion_model);
            cam_info->distortion_model = distortion_model;

            std::copy(K_vec.begin(), K_vec.end(), cam_info->K.begin());
            cam_info->D = D_vec;

            cam_info->R = {1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0};

            for (size_t i = 0; i < 12; ++i)
                cam_info->P[i] = P_vec[i];

            double x = 0.0, y = 0.0, z = 0.0;
            double roll = 0.0, pitch = 0.0, yaw = 0.0;
            nh.param<double>(camera_topic + "/x", x, 0.0);
            nh.param<double>(camera_topic + "/y", y, 0.0);
            nh.param<double>(camera_topic + "/z", z, 0.0);
            nh.param<double>(camera_topic + "/roll", roll, 0.0);
            nh.param<double>(camera_topic + "/pitch", pitch, 0.0);
            nh.param<double>(camera_topic + "/yaw", yaw, 0.0);

            camera_type_stdmap_[camera_topic]->set_camera_info(cam_info);
            camera_type_stdmap_[camera_topic]->set_camera_utils(cam_info);
            camera_type_stdmap_[camera_topic]->set_lidar_to_camera_matrix_xyzrpy_rad(x, y, z, roll, pitch, yaw);
            camera_type_stdmap_[camera_topic]->set_lidar_to_camera_projection_matrix();
        }

        // camera_timer_ = nh.createTimer(ros::Duration(0.01), &mapOptimization::camera_callback, this);

        allocateMemory();

        saveNodePCDDirectory = savePCDDirectory + "Scans/";
    }

    ~mapOptimization()
    {
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCornerCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalSurfCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMapCloud2(new pcl::PointCloud<pcl::PointXYZRGB>());

        size_t saveThreshold = 5000000;
        size_t fileIndex = 0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr chunkCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++)
        {
            // *globalCornerCloud += *transformPointCloud(fullcornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            // *globalSurfCloud += *transformPointCloud(fullsurfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);

            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";

            *globalMapCloud2 = *globalCornerCloud + *globalSurfCloud;

            for (const auto &pt : globalMapCloud2->points)
            {
                if (!(pt.r == 0 && pt.g == 0 && pt.b == 0))
                    chunkCloud->points.push_back(pt);
            }

            if (chunkCloud->size() >= saveThreshold)
            {
                std::string filename = savePCDDirectory + "cloudGlobal_part" + std::to_string(fileIndex++) + ".pcd";
                pcl::io::savePCDFileBinary(filename, *chunkCloud);
                cout << "\nSaved " << filename << " with " << chunkCloud->size() << " points.\n";
                chunkCloud->clear();
            }

            globalCornerCloud->clear();
            globalSurfCloud->clear();
            globalMapCloud2->clear();
        }

        if (!chunkCloud->empty())
        {
            std::string filename = savePCDDirectory + "cloudGlobal_part" + std::to_string(fileIndex++) + ".pcd";
            pcl::io::savePCDFileBinary(filename, *chunkCloud);
            cout << "\nSaved " << filename << " with " << chunkCloud->size() << " points.\n";
        }

        pcl::io::savePCDFileBinary(savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileBinary(savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D);
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<pcl::PointXYZRGB>());       // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<pcl::PointXYZRGB>());         // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<pcl::PointXYZRGB>());     // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<pcl::PointXYZRGB>());       // downsampled surf featuer set from odoOptimization
        fulllaserCloudCornerLastDS.reset(new pcl::PointCloud<pcl::PointXYZRGB>()); // downsampled corner featuer set from odoOptimization
        fulllaserCloudSurfLastDS.reset(new pcl::PointCloud<pcl::PointXYZRGB>());   // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i)
        {
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr &msgIn)
    {
        // timeLaserInfoStamp/timeLaserInfoCur
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();
        const double lidar_time = msgIn->cloud_corner.header.stamp.toSec();

        // extract info and feature cloud
        cloudInfo = *msgIn;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_accum(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr surf_accum(new pcl::PointCloud<pcl::PointXYZRGB>());

        std::for_each(camera_type_stdmap_.begin(), camera_type_stdmap_.end(),
                      [this, msgIn, lidar_time, &corner_accum, &surf_accum](const auto &pair)
                      {
                          const std::string cam_key = pair.first;
                          auto &cam = pair.second;

                          if (!cam->get_camera_info() || !cam->is_info_initialized() || !cam->is_transform_initialized())
                              return;

                          auto img_msg = getImageForLidar(cam_key, lidar_time);
                          if (!img_msg)
                              return; 

                          cam->set_cv_image_from_compressed(img_msg);
                          const cv::Mat &image = cam->get_cv_image();
                          if (image.empty() || image.type() != CV_8UC3)
                              return;

                          const int W = cam->get_image_width();
                          const int H = cam->get_image_height();
                          const auto &P = cam->get_lidar_to_camera_projection_matrix();

                          // ------- corner -------
                          {
                              color_point_cloud::PointCloudConst cloud_corner{msgIn->cloud_corner};
                              std::vector<color_point_cloud::Point> pts;
                              pts.reserve(cloud_corner.getPointCount());
                              for (size_t i = 0; i < cloud_corner.getPointCount(); ++i)
                              {
                                  pts.emplace_back(cloud_corner.getCurrentPoint());
                                  cloud_corner.nextPoint();
                              }

                              std::vector<pcl::PointXYZRGB> out;
                              out.reserve(pts.size());

#pragma omp parallel
                              {
                                  std::vector<pcl::PointXYZRGB> local;
                                  local.reserve(1024);

#pragma omp for nowait
                                  for (int i = 0; i < static_cast<int>(pts.size()); ++i)
                                  {
                                      const auto &p = pts[i];
                                      Eigen::Vector4d p4(p.x, p.y, p.z, 1.0);
                                      Eigen::Vector3d pc = P * p4;
                                      const double z = pc[2];
                                      if (z <= 1e-6)
                                          continue;

                                      const int xi = static_cast<int>(std::round(pc[0] / z));
                                      const int yi = static_cast<int>(std::round(pc[1] / z));
                                      if (xi < 0 || yi < 0 || xi >= W || yi >= H)
                                          continue;

                                      const cv::Vec3b c = image.at<cv::Vec3b>(yi, xi);
                                      pcl::PointXYZRGB q;
                                      q.x = p.x;
                                      q.y = p.y;
                                      q.z = p.z;
                                      q.r = c[2];
                                      q.g = c[1];
                                      q.b = c[0];
                                      local.push_back(q);
                                  }

#pragma omp critical
                                  out.insert(out.end(), local.begin(), local.end());
                              }
                              corner_accum->points.insert(corner_accum->points.end(), out.begin(), out.end());
                          }

                          // ------- surf -------
                          {
                              color_point_cloud::PointCloudConst cloud_surface{msgIn->cloud_surface};
                              std::vector<color_point_cloud::Point> pts;
                              pts.reserve(cloud_surface.getPointCount());
                              for (size_t i = 0; i < cloud_surface.getPointCount(); ++i)
                              {
                                  pts.emplace_back(cloud_surface.getCurrentPoint());
                                  cloud_surface.nextPoint();
                              }

                              std::vector<pcl::PointXYZRGB> out;
                              out.reserve(pts.size());

#pragma omp parallel
                              {
                                  std::vector<pcl::PointXYZRGB> local;
                                  local.reserve(1024);

#pragma omp for nowait
                                  for (int i = 0; i < static_cast<int>(pts.size()); ++i)
                                  {
                                      const auto &p = pts[i];
                                      Eigen::Vector4d p4(p.x, p.y, p.z, 1.0);
                                      Eigen::Vector3d pc = P * p4;
                                      const double z = pc[2];
                                      if (z <= 1e-6)
                                          continue;

                                      const int xi = static_cast<int>(std::round(pc[0] / z));
                                      const int yi = static_cast<int>(std::round(pc[1] / z));
                                      if (xi < 0 || yi < 0 || xi >= W || yi >= H)
                                          continue;

                                      const cv::Vec3b c = image.at<cv::Vec3b>(yi, xi);
                                      pcl::PointXYZRGB q;
                                      q.x = p.x;
                                      q.y = p.y;
                                      q.z = p.z;
                                      q.r = c[2];
                                      q.g = c[1];
                                      q.b = c[0];
                                      local.push_back(q);
                                  }

#pragma omp critical
                                  out.insert(out.end(), local.begin(), local.end());
                              }
                              surf_accum->points.insert(surf_accum->points.end(), out.begin(), out.end());
                          }
                      });

        if (corner_accum->points.empty() || surf_accum->points.empty())
            return;

        {
            std::lock_guard<std::mutex> lock(mtx);

            laserCloudCornerLast->points.swap(corner_accum->points);
            laserCloudCornerLast->width = laserCloudCornerLast->points.size();
            laserCloudCornerLast->height = 1;
            laserCloudCornerLast->is_dense = false;

            laserCloudSurfLast->points.swap(surf_accum->points);
            laserCloudSurfLast->width = laserCloudSurfLast->points.size();
            laserCloudSurfLast->height = 1;
            laserCloudSurfLast->is_dense = false;

            // std::string filename = "/dataset/ros1/ply/corner/" + std::to_string(imgidx) + ".pcd";
            // pcl::io::savePCDFileBinary(filename, *laserCloudCornerLast);

            // std::string filename2 = "/dataset/ros1/ply/surf/" + std::to_string(imgidx) + ".pcd";
            // pcl::io::savePCDFileBinary(filename2, *laserCloudSurfLast);

            static double timeLastProcessing = -1;
            if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
            {
                timeLastProcessing = timeLaserInfoCur;

                updateInitialGuess();
                extractSurroundingKeyFrames();
                downsampleCurrentScan();
                scan2MapOptimization();
                saveKeyFramesAndFactor();
                correctPoses();
                publishOdometry();
                publishFrames();
            }
        }
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
    }

    void pointAssociateToMap(PointType const *const pi, PointType *const po)
    {
        po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
        po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
        po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, PointTypePose *transformIn)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].r = pointFrom.r;
            cloudOut->points[i].g = pointFrom.g;
            cloudOut->points[i].b = pointFrom.b;
            cloudOut->points[i].rgb = pointFrom.rgb;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                            gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                            gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw = transformIn[2];
        return thisPose6D;
    }

    void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
    {
        using namespace gtsam;

        // ref from gtsam's original code "dataset.cpp"
        std::fstream stream(_filename.c_str(), fstream::out);

        for (const auto &key_value : _estimates)
        {
            auto p = dynamic_cast<const GenericValue<Pose3> *>(&key_value.value);
            if (!p)
                continue;

            const Pose3 &pose = p->value();

            Point3 t = pose.translation();
            Rot3 R = pose.rotation();
            auto col1 = R.column(1); // Point3
            auto col2 = R.column(2); // Point3
            auto col3 = R.column(3); // Point3

            stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
                   << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
                   << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
        }
    }

    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok())
        {
            rate.sleep();
            publishGlobalMap();
        }

        if (savePCD == false)
            return;

        // const std::string kitti_format_pg_filename {savePCDDirectory + "optimized_poses.txt"};
        // saveOptimizedVerticesKITTIformat(isamCurrentEstimate, kitti_format_pg_filename);
    }

    void publishGlobalMap()
    {
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMapKeyFrames(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<pcl::PointXYZRGB>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;                                                                                            // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for (auto &pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
        {
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterGlobalMapKeyFrames;                                                                            // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }

    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            performLoopClosure();
            visualizeLoopClosure();
        }
    }

    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr &loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2)
            return;

        loopInfoVec.push_back(*loopMsg);

        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    void performLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        // TODO: NOISE SCORE
        // auto sq = [](double v)
        // { return v * v; };
        // auto deg2rad = [](double d)
        // { return d * M_PI / 180.0; };

        // // 1) score는 m^2 (MSE). 가능하면 ICP에서 쓴 max_corr와 동일 값으로 평가
        // double score = icp.getFitnessScore(icp.getMaxCorrespondenceDistance());
        // // score가 0에 너무 가까우면 수치불안하니 최소치 부여
        // score = std::max(score, 1e-6);

        // // 2) 이동/회전 스케일과 특성 반경 (주차장 초기값)
        // double alpha_t = 1.0; // translation scale
        // double beta_r = 1.0;  // rotation scale
        // double Rchar = 1.5;   // m, 특징 퍼짐 반경(1.0~2.0 사이 튜닝)

        // // 3) 원시 분산
        // double var_t_raw = alpha_t * score;                  // m^2
        // double var_r_raw = beta_r * score / (Rchar * Rchar); // rad^2

        // // 4) 바닥/천장(초기 튜닝값)
        // // 이동: 3 cm ~ 30 cm
        // double var_t = std::clamp(var_t_raw, sq(0.03), sq(0.30));
        // // 회전: 0.5° ~ 5°
        // double var_r = std::clamp(var_r_raw, sq(deg2rad(0.5)), sq(deg2rad(5.0)));

        // // 5) yaw, z를 조금 더 느슨하게
        // double var_rp = var_r;        // roll, pitch
        // double var_yaw = 2.0 * var_r; // yaw는 2배 느슨
        // double var_xy = var_t;        // x, y
        // double var_z = 2.0 * var_t;   // z는 2배 느슨

        // gtsam::Vector Vector6(6);
        // Vector6 << var_rp, var_rp, var_yaw, var_xy, var_xy, var_z;
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize)
                continue;

            pcl::PointCloud<PointType>::Ptr corner_temp(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr surf_temp(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*cornerCloudKeyFrames[keyNear], *corner_temp);
            pcl::copyPointCloud(*surfCloudKeyFrames[keyNear], *surf_temp);

            *nearKeyframes += *transformPointCloud(corner_temp, &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surf_temp, &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty())
            return;

        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3;
        markerNode.scale.y = 0.3;
        markerNode.scale.z = 0.3;
        markerNode.color.r = 0;
        markerNode.color.g = 0.8;
        markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9;
        markerEdge.color.g = 0.9;
        markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }

    void updateInitialGuess()
    {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // initialization
        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }

        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX, cloudInfo.initialGuessY, cloudInfo.initialGuessZ,
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            }
            else
            {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                  transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    }

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for (auto &pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end())
            {
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
            }
            else
            {
                pcl::PointCloud<PointType>::Ptr corner_temp(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr surf_temp(new pcl::PointCloud<PointType>());
                pcl::copyPointCloud(*cornerCloudKeyFrames[thisKeyInd], *corner_temp);
                pcl::copyPointCloud(*surfCloudKeyFrames[thisKeyInd], *surf_temp);

                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(corner_temp, &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surf_temp, &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCornerRGB.setInputCloud(laserCloudCornerLast);
        downSizeFilterCornerRGB.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurfRGB.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurfRGB.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();

        fulllaserCloudCornerLastDS->clear();
        downSizeFilterCornerRGB2.setInputCloud(laserCloudCornerLast);
        downSizeFilterCornerRGB2.filter(*fulllaserCloudCornerLastDS);

        fulllaserCloudSurfLastDS->clear();
        downSizeFilterSurfRGB2.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurfRGB2.filter(*fulllaserCloudSurfLastDS);
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri.x = laserCloudCornerLastDS->points[i].x;
            pointOri.y = laserCloudCornerLastDS->points[i].y;
            pointOri.z = laserCloudCornerLastDS->points[i].z;
            pointOri.intensity = laserCloudCornerLastDS->points[i].rgb;
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            if (pointSearchSqDis[4] < 1.0)
            {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++)
                {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5;
                cy /= 5;
                cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++)
                {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a13 += ax * az;
                    a22 += ay * ay;
                    a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5;
                a13 /= 5;
                a22 /= 5;
                a23 /= 5;
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
                {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                    float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1)
                    {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri.x = laserCloudSurfLastDS->points[i].x;
            pointOri.y = laserCloudSurfLastDS->points[i].y;
            pointOri.z = laserCloudSurfLastDS->points[i].z;
            pointOri.intensity = laserCloudSurfLastDS->points[i].rgb;
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0)
            {
                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid)
                {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1)
                    {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i)
        {
            if (laserCloudOriCornerFlag[i] == true)
            {
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i)
        {
            if (laserCloudOriSurfFlag[i] == true)
            {
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50)
        {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++)
        {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

            float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

            float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0)
        {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--)
            {
                if (matE.at<float>(0, i) < eignThre[i])
                {
                    for (int j = 0; j < 6; j++)
                    {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                }
                else
                {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
            pow(matX.at<float>(3, 0) * 100, 2) +
            pow(matX.at<float>(4, 0) * 100, 2) +
            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05)
        {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;
            }

            transformUpdate();
        }
        else
        {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
    {
        if (cloudInfo.imuAvailable == true)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        if (sensor == SensorType::LIVOX)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
                return true;
        }

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
            abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }
        else
        {
            // TODO: noise
            //  double noiseScale = 3e-4;;
            //  double noiseRotScale = 5e-3;
            //  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << noiseScale,noiseScale,noiseScale, noiseRotScale,noiseRotScale,noiseRotScale).finished());
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        if (poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisCornerKeyFrame(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisSurfKeyFrame(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisCornerKeyFrame2(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisSurfKeyFrame2(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*fulllaserCloudCornerLastDS, *thisCornerKeyFrame2);
        pcl::copyPointCloud(*fulllaserCloudSurfLastDS, *thisSurfKeyFrame2);
        fullcornerCloudKeyFrames.push_back(thisCornerKeyFrame2);
        fullsurfCloudKeyFrames.push_back(thisSurfKeyFrame2);

        // std::string curr_scd_node_idx = padZeros(cloudKeyPoses6D->size() - 1);

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisKeyFrameCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        // *thisKeyFrameCloud += *thisCornerKeyFrame;
        // *thisKeyFrameCloud += *thisSurfKeyFrame;
        // pcl::io::savePCDFileBinary(saveNodePCDDirectory + curr_scd_node_idx + ".pcd", *thisKeyFrameCloud);

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose &pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);

        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine;         // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        }
        else
        {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuAvailable == true)
            {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut, &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo.getNumSubscribers() != 0)
        {
            if (lastSLAMInfoPubSize != cloudKeyPoses6D->size())
            {
                lio_sam::cloud_info slamInfo;
                slamInfo.header.stamp = timeLaserInfoStamp;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
                *cloudOut += *laserCloudCornerLastDS;
                *cloudOut += *laserCloudSurfLastDS;
                slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
                slamInfo.key_frame_poses = publishCloud(ros::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
                pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
                *localMapOut += *laserCloudCornerFromMapDS;
                *localMapOut += *laserCloudSurfFromMapDS;
                slamInfo.key_frame_map = publishCloud(ros::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
                pubSLAMInfo.publish(slamInfo);
                lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_sam");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");

    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    ros::spin();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}
