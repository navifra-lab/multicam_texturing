    #include "utility.h"
    #include "lio_sam/cloud_info.h"
    // 추가 include
    #include <message_filters/subscriber.h>
    #include <message_filters/synchronizer.h>
    #include <message_filters/sync_policies/approximate_time.h>

    #include <tf2_ros/transform_listener.h>
    #include <tf2_ros/buffer.h>

    #include <pcl_conversions/pcl_conversions.h>
    #include <pcl/PCLPointCloud2.h>
    #include <pcl/common/io.h>  // pcl::concatenatePointCloud
    #include <pcl/common/transforms.h>
    #include <Eigen/Geometry>
    #include <pcl/common/io.h>          // getFieldIndex
    #include <pcl/point_cloud.h>        // 구조체들
    #include <cstring>


    struct VelodynePointXYZIRT
    {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY;
        uint16_t ring;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
    POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
        (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
        (uint16_t, ring, ring) (float, time, time)
    )

    struct OusterPointXYZIRT {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint16_t ring;
        uint16_t noise;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
    POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
        (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
        (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
        (uint16_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
    )

    // Use the Velodyne point format as a common representation
    using PointXYZIRT = VelodynePointXYZIRT;

    const int queueLength = 2000;

    class ImageProjection : public ParamServer
    {
    private:

        std::mutex imuLock;
        std::mutex odoLock;

        ros::Subscriber subLaserCloud;
        ros::Publisher  pubLaserCloud;
        
        ros::Publisher pubExtractedCloud;
        ros::Publisher pubLaserCloudInfo;

        ros::Subscriber subImu;
        std::deque<sensor_msgs::Imu> imuQueue;

        ros::Subscriber subOdom;
        std::deque<nav_msgs::Odometry> odomQueue;

        std::deque<sensor_msgs::PointCloud2> cloudQueue;
        sensor_msgs::PointCloud2 currentCloudMsg;

        double *imuTime = new double[queueLength];
        double *imuRotX = new double[queueLength];
        double *imuRotY = new double[queueLength];
        double *imuRotZ = new double[queueLength];

        int imuPointerCur;
        bool firstPointFlag;
        Eigen::Affine3f transStartInverse;

        pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
        pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
        pcl::PointCloud<PointType>::Ptr   fullCloud;
        pcl::PointCloud<PointType>::Ptr   extractedCloud;

        int deskewFlag;
        cv::Mat rangeMat;

        bool odomDeskewFlag;
        float odomIncreX;
        float odomIncreY;
        float odomIncreZ;

        lio_sam::cloud_info cloudInfo;
        double timeScanCur;
        double timeScanEnd;
        std_msgs::Header cloudHeader;

        vector<int> columnIdnCountVec;

        bool dualLidar{false};
        std::string lidar2Topic;
        std::string mergeTargetFrame; // 합칠 목표 프레임 (예: lidarFrame 또는 cloud frame)
        double approxSlop{0.03};
        int ringOffsetSecond{0};

        // TF
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener{tfBuffer};

        // message_filters (듀얼일 때만 사용)
        using PC2 = sensor_msgs::PointCloud2;
        using ApproxPolicy = message_filters::sync_policies::ApproximateTime<PC2, PC2>;
        std::unique_ptr<message_filters::Subscriber<PC2>> subCloud1, subCloud2;
        std::unique_ptr<message_filters::Synchronizer<ApproxPolicy>> sync;

    public:
        ImageProjection() : deskewFlag(0)
        {
            subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
            subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
            // subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

            if (!multilidar)
            {
                subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
                ROS_INFO("[ImageProjection] single lidar mode");
            }
            else
            {
                mergeTargetFrame = lidarFrame;
                subCloud1.reset(new message_filters::Subscriber<PC2>(nh, pointCloudTopic, 5));
                subCloud2.reset(new message_filters::Subscriber<PC2>(nh, pointCloudTopic2, 5));
                sync.reset(new message_filters::Synchronizer<ApproxPolicy>(ApproxPolicy(20), *subCloud1, *subCloud2));
                sync->setMaxIntervalDuration(ros::Duration(approxSlop));
                sync->registerCallback(boost::bind(&ImageProjection::multiCloudHandler, this, _1, _2));
                ROS_INFO("[ImageProjection] dual lidar mode: %s + %s, slop=%.3f",
                         pointCloudTopic.c_str(), pointCloudTopic2.c_str(), approxSlop);
            }
            pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
            pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1);

            allocateMemory();
            resetParameters();

            pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
        }

        Eigen::Matrix4f rpyToMat44(float x, float y, float z, float roll, float pitch, float yaw)
        {
            Eigen::AngleAxisf Rx(roll, Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf Ry(pitch, Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf Rz(yaw, Eigen::Vector3f::UnitZ());
            Eigen::Quaternionf q = Rz * Ry * Rx; // ZYX
            Eigen::Matrix3f R = q.toRotationMatrix();

            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block<3, 3>(0, 0) = R;
            T(0, 3) = x;
            T(1, 3) = y;
            T(2, 3) = z;
            return T;
        }

        static bool transformPCL2_XYZ(const pcl::PCLPointCloud2 &in, pcl::PCLPointCloud2 &out, const Eigen::Matrix4f &T)
        {
            out = in; // 메타/필드/데이터 전부 복사
            int idx_x = pcl::getFieldIndex(out, "x");
            int idx_y = pcl::getFieldIndex(out, "y");
            int idx_z = pcl::getFieldIndex(out, "z");
            if (idx_x < 0 || idx_y < 0 || idx_z < 0)
            {
                ROS_WARN_THROTTLE(1.0, "[merge] x/y/z field missing in PCLPointCloud2");
                return false;
            }
            const auto &fx = out.fields[idx_x], &fy = out.fields[idx_y], &fz = out.fields[idx_z];
            if (fx.datatype != pcl::PCLPointField::FLOAT32 ||
                fy.datatype != pcl::PCLPointField::FLOAT32 ||
                fz.datatype != pcl::PCLPointField::FLOAT32)
            {
                ROS_WARN_THROTTLE(1.0, "[merge] x/y/z not FLOAT32");
                return false;
            }

            const size_t n = static_cast<size_t>(out.width) * out.height;
            const size_t step = out.point_step;

            const float r00 = T(0, 0), r01 = T(0, 1), r02 = T(0, 2), tx = T(0, 3);
            const float r10 = T(1, 0), r11 = T(1, 1), r12 = T(1, 2), ty = T(1, 3);
            const float r20 = T(2, 0), r21 = T(2, 1), r22 = T(2, 2), tz = T(2, 3);

            for (size_t i = 0; i < n; ++i)
            {
                uint8_t *base = &out.data[i * step];
                float *px = reinterpret_cast<float *>(base + fx.offset);
                float *py = reinterpret_cast<float *>(base + fy.offset);
                float *pz = reinterpret_cast<float *>(base + fz.offset);
                const float x = *px, y = *py, z = *pz;

                *px = r00 * x + r01 * y + r02 * z + tx;
                *py = r10 * x + r11 * y + r12 * z + ty;
                *pz = r20 * x + r21 * y + r22 * z + tz;
            }
            return true;
        }

        // (옵션) ring 오프셋도 PCL2에서 직접 처리
        static void addRingOffsetPCL2(pcl::PCLPointCloud2 &cloud, int offset)
        {
            if (offset == 0)
                return;
            int idx = pcl::getFieldIndex(cloud, "ring");
            if (idx < 0)
                return;
            auto &f = cloud.fields[idx];
            if (f.datatype != pcl::PCLPointField::UINT16)
            {
                ROS_WARN_ONCE("[merge] ring is not UINT16; skip offset");
                return;
            }
            const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
            const size_t step = cloud.point_step;
            for (size_t i = 0; i < n; ++i)
            {
                uint8_t *base = &cloud.data[i * step];
                uint16_t *pr = reinterpret_cast<uint16_t *>(base + f.offset);
                *pr = static_cast<uint16_t>(*pr + offset);
            }
        }

        // uint16 ring 전제(일반적). 다른 타입이면 스킵.
        // void applyRingOffsetIfAny(pcl::PCLPointCloud2 &cloud, int offset)
        // {
        //     if (offset == 0)
        //         return;
        //     for (auto &f : cloud.fields)
        //     {
        //         if (f.name == "ring" && f.datatype == pcl::PCLPointField::UINT16)
        //         {
        //             const size_t n = cloud.width * cloud.height;
        //             const size_t step = cloud.point_step, off = f.offset;
        //             for (size_t i = 0; i < n; ++i)
        //             {
        //                 uint16_t *p = reinterpret_cast<uint16_t *>(&cloud.data[i * step + off]);
        //                 *p = static_cast<uint16_t>(*p + offset);
        //             }
        //             return;
        //         }
        //     }
        // }

        void multiCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &c1, const sensor_msgs::PointCloud2::ConstPtr &c2)
        {
            // 0) ROS → PCL2
            pcl::PCLPointCloud2 p1_raw, p2_raw;
            pcl_conversions::toPCL(*c1, p1_raw);
            pcl_conversions::toPCL(*c2, p2_raw);

            // 1) lidar2 → lidar1 (상대 포즈)
            if (relativePose.size() != 6)
                relativePose.assign(6, 0.0);
            Eigen::Matrix4f T_l1_l2 = rpyToMat44(
                static_cast<float>(relativePose[0]),
                static_cast<float>(relativePose[1]),
                static_cast<float>(relativePose[2]),
                static_cast<float>(relativePose[3]),
                static_cast<float>(relativePose[4]),
                static_cast<float>(relativePose[5]) // degrees 기본 false
            );

            pcl::PCLPointCloud2 p2_in_l1;
            if (!transformPCL2_XYZ(p2_raw, p2_in_l1, T_l1_l2))
                return;
            pcl::PCLPointCloud2 p1_in_l1 = p1_raw;

            // 2) (옵션) lidar1 → target 포즈
            pcl::PCLPointCloud2 p1_tgt, p2_tgt;
            p1_tgt = p1_in_l1;
            p2_tgt = p2_in_l1;

            // 3) (옵션) 2번 라이다 ring 오프셋
            if (ringOffsetSecond != 0)
                addRingOffsetPCL2(p2_tgt, ringOffsetSecond);

            // 4) concat (PCL2)
            pcl::PCLPointCloud2 pcat;
            // NOTE: 1.10에서는 concatenatePointCloud 사용 (경고만 뜸)
            if (!pcl::concatenatePointCloud(p1_tgt, p2_tgt, pcat))
            {
                ROS_WARN_THROTTLE(1.0, "[merge] concatenatePointCloud failed (schema mismatch?)");
                return;
            }

            // 5) PCL2 → ROS
            sensor_msgs::PointCloud2::Ptr merged(new sensor_msgs::PointCloud2);
            pcl_conversions::fromPCL(pcat, *merged);
            merged->header.frame_id = mergeTargetFrame;
            merged->header.stamp = (c1->header.stamp > c2->header.stamp) ? c1->header.stamp : c2->header.stamp;

            // 6) 기존 파이프라인 재사용
            if (!cachePointCloud(merged))
                return;
            if (!deskewInfo())
                return;
            projectPointCloud();
            cloudExtraction();
            publishClouds();
            resetParameters();
        }

        // void multiCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &c1, const sensor_msgs::PointCloud2::ConstPtr &c2)
        // {
        //     // 0) ROS→PCL
        //     pcl::PCLPointCloud2 p1, p2;
        //     pcl_conversions::toPCL(*c1, p1);
        //     pcl_conversions::toPCL(*c2, p2);

        //     // 1) 2번을 1번 프레임으로:  T_l1_l2 (lidar2 → lidar1)
        //     //    lidar2InLidar1 = [x y z r p y]
        //     if (relativePose.size() != 6)
        //         relativePose.assign(6, 0.0);
        //     Eigen::Matrix4f T_l1_l2 = rpyToMat44(
        //         static_cast<float>(relativePose[0]),
        //         static_cast<float>(relativePose[1]),
        //         static_cast<float>(relativePose[2]),
        //         static_cast<float>(relativePose[3]),
        //         static_cast<float>(relativePose[4]),
        //         static_cast<float>(relativePose[5]));

        //     pcl::PCLPointCloud2 p2_in_l1;
        //     pcl::transformPointCloud(p2, p2_in_l1, T_l1_l2); // **여기가 PCL 행렬 변환 핵심**

        //     pcl::PCLPointCloud2 p1_in_target, p2_in_target;
        //     p1_in_target = p1;
        //     p2_in_target = p2_in_l1;

        //     // (옵션) 2번 라이다 ring 오프셋
        //     if (ringOffsetSecond != 0)
        //     {
        //         applyRingOffsetIfAny(p2_in_target, ringOffsetSecond); // 앞서 제시한 그대로 재사용
        //     }

        //     // 3) concat (스키마 동일 가정)
        //     pcl::PCLPointCloud2 pcat;
        //     try
        //     {
        //         pcl::concatenatePointCloud(p1_in_target, p2_in_target, pcat);
        //     }
        //     catch (const std::exception &e)
        //     {
        //         ROS_WARN_STREAM_THROTTLE(1.0, "[merge] concatenate failed: " << e.what());
        //         return;
        //     }

        //     // 4) PCL→ROS, 타임스탬프/프레임 설정
        //     sensor_msgs::PointCloud2::Ptr merged(new sensor_msgs::PointCloud2);
        //     pcl_conversions::fromPCL(pcat, *merged);
        //     merged->header.frame_id = mergeTargetFrame;
        //     merged->header.stamp = (c1->header.stamp > c2->header.stamp) ? c1->header.stamp : c2->header.stamp;

        //     // 5) 기존 파이프라인 그대로
        //     if (!cachePointCloud(merged))
        //         return;
        //     if (!deskewInfo())
        //         return;
        //     projectPointCloud();
        //     cloudExtraction();
        //     publishClouds();
        //     resetParameters();
        // }

        void allocateMemory()
        {
            laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
            tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
            fullCloud.reset(new pcl::PointCloud<PointType>());
            extractedCloud.reset(new pcl::PointCloud<PointType>());

            fullCloud->points.resize(N_SCAN*Horizon_SCAN);

            cloudInfo.startRingIndex.assign(N_SCAN, 0);
            cloudInfo.endRingIndex.assign(N_SCAN, 0);

            cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
            cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

            resetParameters();
        }

        void resetParameters()
        {
            laserCloudIn->clear();
            extractedCloud->clear();
            // reset range matrix for range image projection
            rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

            imuPointerCur = 0;
            firstPointFlag = true;
            odomDeskewFlag = false;

            for (int i = 0; i < queueLength; ++i)
            {
                imuTime[i] = 0;
                imuRotX[i] = 0;
                imuRotY[i] = 0;
                imuRotZ[i] = 0;
            }

            columnIdnCountVec.assign(N_SCAN, 0);
        }

        ~ImageProjection(){}

        void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
        {
            sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

            std::lock_guard<std::mutex> lock1(imuLock);
            imuQueue.push_back(thisImu);

            // debug IMU data
            // cout << std::setprecision(6);
            // cout << "IMU acc: " << endl;
            // cout << "x: " << thisImu.linear_acceleration.x << 
            //       ", y: " << thisImu.linear_acceleration.y << 
            //       ", z: " << thisImu.linear_acceleration.z << endl;
            // cout << "IMU gyro: " << endl;
            // cout << "x: " << thisImu.angular_velocity.x << 
            //       ", y: " << thisImu.angular_velocity.y << 
            //       ", z: " << thisImu.angular_velocity.z << endl;
            // double imuRoll, imuPitch, imuYaw;
            // tf::Quaternion orientation;
            // tf::quaternionMsgToTF(thisImu.orientation, orientation);
            // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
            // cout << "IMU roll pitch yaw: " << endl;
            // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
        }

        void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
        {
            std::lock_guard<std::mutex> lock2(odoLock);
            odomQueue.push_back(*odometryMsg);
        }

        void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
        {
            if (!cachePointCloud(laserCloudMsg))
                return;

            if (!deskewInfo())
                return;

            projectPointCloud();

            cloudExtraction();

            publishClouds();

            resetParameters();
        }

        bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
        {
            // cache point cloud
            cloudQueue.push_back(*laserCloudMsg);
            if (cloudQueue.size() <= 2)
                return false;

            // convert cloud
            currentCloudMsg = std::move(cloudQueue.front());
            cloudQueue.pop_front();
            if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
            {
                pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
            }
            else if (sensor == SensorType::OUSTER)
            {
                // Convert to Velodyne format
                pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
                laserCloudIn->points.resize(tmpOusterCloudIn->size());
                laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
                for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
                {
                    auto &src = tmpOusterCloudIn->points[i];
                    auto &dst = laserCloudIn->points[i];
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.intensity = src.intensity;
                    dst.ring = src.ring;
                    dst.time = src.t * 1e-9f;
                }
            }
            else
            {
                ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
                ros::shutdown();
            }

            // get timestamp
            cloudHeader = currentCloudMsg.header;
            timeScanCur = cloudHeader.stamp.toSec();
            timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

            vector<int> indices;
            pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

            // check dense flag
            if (laserCloudIn->is_dense == false)
            {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }

            // check ring channel
            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == "ring")
                    {
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1)
                {
                    ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    ros::shutdown();
                }
            }

            // check point time
            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (auto &field : currentCloudMsg.fields)
                {
                    if (field.name == "time" || field.name == "t")
                    {
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1)
                    ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }

            return true;
        }

        bool deskewInfo()
        {
            std::lock_guard<std::mutex> lock1(imuLock);
            std::lock_guard<std::mutex> lock2(odoLock);

            // make sure IMU data available for the scan
            if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
            {
                ROS_DEBUG("Waiting for IMU data ...");
                return false;
            }

            imuDeskewInfo();

            odomDeskewInfo();

            return true;
        }

        void imuDeskewInfo()
        {
            cloudInfo.imuAvailable = false;

            while (!imuQueue.empty())
            {
                if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                    imuQueue.pop_front();
                else
                    break;
            }

            if (imuQueue.empty())
                return;

            imuPointerCur = 0;

            for (int i = 0; i < (int)imuQueue.size(); ++i)
            {
                sensor_msgs::Imu thisImuMsg = imuQueue[i];
                double currentImuTime = thisImuMsg.header.stamp.toSec();

                // get roll, pitch, and yaw estimation for this scan
                if (currentImuTime <= timeScanCur)
                    imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

                if (currentImuTime > timeScanEnd + 0.01)
                    break;

                if (imuPointerCur == 0){
                    imuRotX[0] = 0;
                    imuRotY[0] = 0;
                    imuRotZ[0] = 0;
                    imuTime[0] = currentImuTime;
                    ++imuPointerCur;
                    continue;
                }

                // get angular velocity
                double angular_x, angular_y, angular_z;
                imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

                // integrate rotation
                double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
                imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
                imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
                imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
                imuTime[imuPointerCur] = currentImuTime;
                ++imuPointerCur;
            }

            --imuPointerCur;

            if (imuPointerCur <= 0)
                return;

            cloudInfo.imuAvailable = true;
        }

        void odomDeskewInfo()
        {
            cloudInfo.odomAvailable = false;

            while (!odomQueue.empty())
            {
                if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                    odomQueue.pop_front();
                else
                    break;
            }

            if (odomQueue.empty())
                return;

            if (odomQueue.front().header.stamp.toSec() > timeScanCur)
                return;

            // get start odometry at the beinning of the scan
            nav_msgs::Odometry startOdomMsg;

            for (int i = 0; i < (int)odomQueue.size(); ++i)
            {
                startOdomMsg = odomQueue[i];

                if (ROS_TIME(&startOdomMsg) < timeScanCur)
                    continue;
                else
                    break;
            }

            tf::Quaternion orientation;
            tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

            double roll, pitch, yaw;
            tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

            // Initial guess used in mapOptimization
            cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
            cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
            cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
            cloudInfo.initialGuessRoll  = roll;
            cloudInfo.initialGuessPitch = pitch;
            cloudInfo.initialGuessYaw   = yaw;

            cloudInfo.odomAvailable = true;

            // get end odometry at the end of the scan
            odomDeskewFlag = false;

            if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
                return;

            nav_msgs::Odometry endOdomMsg;

            for (int i = 0; i < (int)odomQueue.size(); ++i)
            {
                endOdomMsg = odomQueue[i];

                if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                    continue;
                else
                    break;
            }

            if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
                return;

            Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

            tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
            tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
            Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

            Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

            float rollIncre, pitchIncre, yawIncre;
            pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

            odomDeskewFlag = true;
        }

        void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
        {
            *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

            int imuPointerFront = 0;
            while (imuPointerFront < imuPointerCur)
            {
                if (pointTime < imuTime[imuPointerFront])
                    break;
                ++imuPointerFront;
            }

            if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
            {
                *rotXCur = imuRotX[imuPointerFront];
                *rotYCur = imuRotY[imuPointerFront];
                *rotZCur = imuRotZ[imuPointerFront];
            } else {
                int imuPointerBack = imuPointerFront - 1;
                double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
                *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
                *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
            }
        }

        void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
        {
            *posXCur = 0; *posYCur = 0; *posZCur = 0;

            // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

            // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
            //     return;

            // float ratio = relTime / (timeScanEnd - timeScanCur);

            // *posXCur = ratio * odomIncreX;
            // *posYCur = ratio * odomIncreY;
            // *posZCur = ratio * odomIncreZ;
        }

        PointType deskewPoint(PointType *point, double relTime)
        {
            if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
                return *point;

            double pointTime = timeScanCur + relTime;

            float rotXCur, rotYCur, rotZCur;
            findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

            float posXCur, posYCur, posZCur;
            findPosition(relTime, &posXCur, &posYCur, &posZCur);

            if (firstPointFlag == true)
            {
                transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
                firstPointFlag = false;
            }

            // transform points to start
            Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
            Eigen::Affine3f transBt = transStartInverse * transFinal;

            PointType newPoint;
            newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
            newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
            newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
            newPoint.intensity = point->intensity;

            return newPoint;
        }

        void projectPointCloud()
        {
            int cloudSize = laserCloudIn->points.size();
            // range image projection
            for (int i = 0; i < cloudSize; ++i)
            {
                PointType thisPoint;
                thisPoint.x = laserCloudIn->points[i].x;
                thisPoint.y = laserCloudIn->points[i].y;
                thisPoint.z = laserCloudIn->points[i].z;
                thisPoint.intensity = laserCloudIn->points[i].intensity;

                float range = pointDistance(thisPoint);
                if (range < lidarMinRange || range > lidarMaxRange)
                    continue;

                int rowIdn = laserCloudIn->points[i].ring;
                if (rowIdn < 0 || rowIdn >= N_SCAN)
                    continue;

                if (rowIdn % downsampleRate != 0)
                    continue;

                int columnIdn = -1;
                if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
                {
                    float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                    static float ang_res_x = 360.0/float(Horizon_SCAN);
                    columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                    if (columnIdn >= Horizon_SCAN)
                        columnIdn -= Horizon_SCAN;
                }
                else if (sensor == SensorType::LIVOX)
                {
                    columnIdn = columnIdnCountVec[rowIdn];
                    columnIdnCountVec[rowIdn] += 1;
                }
                
                if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                    continue;

                if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                    continue;

                thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

                rangeMat.at<float>(rowIdn, columnIdn) = range;

                int index = columnIdn + rowIdn * Horizon_SCAN;
                fullCloud->points[index] = thisPoint;
            }
        }

        void cloudExtraction()
        {
            int count = 0;
            // extract segmented cloud for lidar odometry
            for (int i = 0; i < N_SCAN; ++i)
            {
                cloudInfo.startRingIndex[i] = count - 1 + 5;

                for (int j = 0; j < Horizon_SCAN; ++j)
                {
                    if (rangeMat.at<float>(i,j) != FLT_MAX)
                    {
                        // mark the points' column index for marking occlusion later
                        cloudInfo.pointColInd[count] = j;
                        // save range info
                        cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                        // save extracted cloud
                        extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        // size of extracted cloud
                        ++count;
                    }
                }
                cloudInfo.endRingIndex[i] = count -1 - 5;
            }
        }
        
        void publishClouds()
        {
            cloudInfo.header = cloudHeader;
            cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
            pubLaserCloudInfo.publish(cloudInfo);
        }
    };

    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "lio_sam");

        ImageProjection IP;
        
        ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

        ros::MultiThreadedSpinner spinner(3);
        spinner.spin();
        
        return 0;
    }
