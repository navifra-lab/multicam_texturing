#include "utility.h"
#include "lio_sam/cloud_info.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{

public:

    ros::Subscriber subLaserCloudInfo;

    using InfoMsg = lio_sam::cloud_info;
    using InfoConstPtr = lio_sam::cloud_infoConstPtr;
    using ApproxPolicy = message_filters::sync_policies::ApproximateTime<InfoMsg, InfoMsg>;

    std::unique_ptr<message_filters::Subscriber<InfoMsg>> subInfo1, subInfo2;
    std::unique_ptr<message_filters::Synchronizer<ApproxPolicy>> sync;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;
    ros::Publisher pubGoodPoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;
    pcl::PointCloud<PointType>::Ptr goodCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    lio_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    // 병합 시 변환 (lidar2 -> lidar1)
    Eigen::Matrix4f T_21 = Eigen::Matrix4f::Identity();

    // 파라미터
    std::string infoTopic1{"/lio_sam/deskew/cloud_info_1"};
    std::string infoTopic2{"/lio_sam/deskew/cloud_info_2"};

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

    FeatureExtraction()
    {
        if(multilidar)
        {
            // nh.param<std::string>("cloud_info_topic_1", infoTopic1, infoTopic1);
            // nh.param<std::string>("cloud_info_topic_2", infoTopic2, infoTopic2);
            // nh.param("approx_slop", approxSlop, approxSlop);

            // std::vector<double> rpyxyz(6, 0.0);
            // nh.param("relative_pose_rpyxyz", rpyxyz, rpyxyz); // [roll,pitch,yaw(rad), x,y,z(m)]
            // {
            //     Eigen::AngleAxisf Rx(rpyxyz[0], Eigen::Vector3f::UnitX());
            //     Eigen::AngleAxisf Ry(rpyxyz[1], Eigen::Vector3f::UnitY());
            //     Eigen::AngleAxisf Rz(rpyxyz[2], Eigen::Vector3f::UnitZ());
            //     Eigen::Matrix3f R = (Rz * Ry * Rx).toRotationMatrix();
            //     T_21.setIdentity();
            //     T_21.block<3, 3>(0, 0) = R;
            //     T_21(0, 3) = static_cast<float>(rpyxyz[3]);
            //     T_21(1, 3) = static_cast<float>(rpyxyz[4]);
            //     T_21(2, 3) = static_cast<float>(rpyxyz[5]);
            // }

            pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1);
            pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
            pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
            pubGoodPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_good", 1);

            subInfo1.reset(new message_filters::Subscriber<InfoMsg>(nh, "lio_sam/deskew/cloud_info_1", 5));
            subInfo2.reset(new message_filters::Subscriber<InfoMsg>(nh, "lio_sam/deskew/cloud_info_2", 5));
            sync.reset(new message_filters::Synchronizer<ApproxPolicy>(ApproxPolicy(20), *subInfo1, *subInfo2));
            sync->setMaxIntervalDuration(ros::Duration(approxSlop));
            sync->registerCallback(boost::bind(&FeatureExtraction::multiInfoHandler, this, _1, _2));

            initializationValue();
        }
        else
        {
            subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

            pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1);
            pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
            pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
            pubGoodPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_good", 1);

            initializationValue();
        }
    }

    struct FEOutputs
    {
        pcl::PointCloud<PointType>::Ptr corner{new pcl::PointCloud<PointType>()};
        pcl::PointCloud<PointType>::Ptr surface{new pcl::PointCloud<PointType>()};
        pcl::PointCloud<PointType>::Ptr good{new pcl::PointCloud<PointType>()};
        std_msgs::Header header;
        lio_sam::cloud_info info;
    };

    bool processOne(const InfoConstPtr &msg, FEOutputs &out)
    {
        cloudInfo = *msg;
        cloudHeader = msg->header;
        pcl::fromROSMsg(msg->cloud_deskewed, *extractedCloud);

        calculateSmoothness();

        markOccludedPoints();

        buildGoodPointsCloud();

        extractFeatures();

        *out.corner = *cornerCloud;
        *out.surface = *surfaceCloud;
        *out.good = *goodCloud;
        out.header = cloudHeader;
        out.info = cloudInfo;

        cornerCloud->clear();
        surfaceCloud->clear();
        goodCloud->clear();
        extractedCloud->clear();
        return true;
    }

    static inline void sanitizeCloud(pcl::PointCloud<PointType>::Ptr &cloud, float max_abs_coord = 1e4f)
    {
        if (!cloud)
            return;

        std::vector<int> idx;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);

        auto finite_ok = [max_abs_coord](const PointType &p)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                return false;

            if (std::fabs(p.x) > max_abs_coord || std::fabs(p.y) > max_abs_coord || std::fabs(p.z) > max_abs_coord)
                return false;

            return true;
        };
        
        cloud->erase(std::remove_if(cloud->begin(), cloud->end(),
                                    [finite_ok](const PointType &p)
                                    { return !finite_ok(p); }),
                     cloud->end());

        cloud->is_dense = true;
    }

    void multiInfoHandler(const InfoConstPtr &m1, const InfoConstPtr &m2)
    {
        FEOutputs o1, o2;

        if (!processOne(m1, o1))
        {
            return;
        }

        if (!processOne(m2, o2))
        {
            return;
        }

        pcl::PointCloud<PointType>::Ptr c2_in1(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr s2_in1(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr g2_in1(new pcl::PointCloud<PointType>());

        Eigen::Matrix4f T_l1_l2 = rpyToMat44(
            static_cast<float>(relativePose[0]),
            static_cast<float>(relativePose[1]),
            static_cast<float>(relativePose[2]),
            static_cast<float>(relativePose[3]),
            static_cast<float>(relativePose[4]),
            static_cast<float>(relativePose[5])
        );
        pcl::transformPointCloud(*o2.corner, *c2_in1, T_l1_l2);
        pcl::transformPointCloud(*o2.surface, *s2_in1, T_l1_l2);
        pcl::transformPointCloud(*o2.good, *g2_in1, T_l1_l2);

        pcl::PointCloud<PointType>::Ptr c_merged(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr s_merged(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr g_merged(new pcl::PointCloud<PointType>());
        *c_merged = *o1.corner;
        *c_merged += *c2_in1;
        *s_merged = *o1.surface;
        *s_merged += *s2_in1;
        *g_merged = *o1.good;
        *g_merged += *g2_in1;
        
        sanitizeCloud(c_merged);
        sanitizeCloud(s_merged);
        sanitizeCloud(g_merged);

        std_msgs::Header outHeader = o1.header;
        if (o2.header.stamp > outHeader.stamp)
            outHeader.stamp = o2.header.stamp;

        lio_sam::cloud_info outInfo = o1.info;
        outInfo.header = outHeader;
        outInfo.cloud_corner = publishCloud(pubCornerPoints, c_merged, outHeader.stamp, lidarFrame);
        outInfo.cloud_surface = publishCloud(pubSurfacePoints, s_merged, outHeader.stamp, lidarFrame);
        outInfo.cloud_good = publishCloud(pubGoodPoints, g_merged, outHeader.stamp, lidarFrame);

        // pcl::io::savePCDFileBinary("/dataset/0825/pcd/corner1.pcd", *o1.corner);
        // pcl::io::savePCDFileBinary("/dataset/0825/pcd/surf1.pcd", *o1.surface);
        // pcl::io::savePCDFileBinary("/dataset/0825/pcd/good1.pcd", *o1.good);
        // pcl::io::savePCDFileBinary("/dataset/0825/pcd/corner2.pcd", *c2_in1);
        // pcl::io::savePCDFileBinary("/dataset/0825/pcd/surf2.pcd", *s2_in1);
        // pcl::io::savePCDFileBinary("/dataset/0825/pcd/good2.pcd", *g2_in1);
        // pcl::io::savePCDFileBinary("/dataset/0825/pcd/corner3.pcd", *c_merged);
        // pcl::io::savePCDFileBinary("/dataset/0825/pcd/surf3.pcd", *s_merged);
        // pcl::io::savePCDFileBinary("/dataset/0825/pcd/good3.pcd", *g_merged);

        freeCloudInfoMemory();

        pubLaserCloudInfo.publish(outInfo);
    }

    void initializationValue()
    {
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());
        goodCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
    }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        cloudInfo = *msgIn; // new cloud info
        cloudHeader = msgIn->header; // new cloud header
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

        calculateSmoothness();

        markOccludedPoints();

        buildGoodPointsCloud();

        extractFeatures();

        publishFeatureCloud();
    }

    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
            // cloudSmoothness for sorting
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // occluded points
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

            if (columnDiff < 10){
                // 10 pixel diff in range image
                if (depth1 - depth2 > 0.3){
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            // parallel beam
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    void buildGoodPointsCloud()
    {
        goodCloud->clear();
        const int cloudSize = extractedCloud->points.size();

        for (int i = 5; i < cloudSize - 6; ++i)
        {
            if (cloudNeighborPicked[i] == 0)
            {
                const auto &p = extractedCloud->points[i];
                if (pcl::isFinite(p))
                    goodCloud->push_back(p);
            }
        }
    }

    void extractFeatures()
    {
        cornerCloud->clear();
        surfaceCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();

            for (int j = 0; j < 6; j++)
            {

                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20){
                            cloudLabel[ind] = 1;
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {

                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0){
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;
        }
    }

    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }

    void publishFeatureCloud()
    {
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        cloudInfo.cloud_corner  = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_good = publishCloud(pubGoodPoints, goodCloud, cloudHeader.stamp, lidarFrame);

        // publish to mapOptimization
        pubLaserCloudInfo.publish(cloudInfo);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
   
    ros::spin();

    return 0;
}