#include "utility.h"
#include "lvi_sam/cloud_info.h"

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

#include <proj.h>

#include <tf/transform_datatypes.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geoid.hpp>

using namespace gtsam;

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
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

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
    ros::Publisher pubOdomAftMappedROS;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubOdomRecvGPS;
    ros::Publisher pubOdomUseGPS;
    ros::Publisher pubPath;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubCloudAndPoseRegistered;
    ros::Publisher pubLoopConstraintEdge;

    ros::Subscriber subLaserCloudInfo;
    ros::Subscriber subGPS;
    ros::Subscriber subLoopInfo;

    std::deque<nav_msgs::Odometry> gpsQueue;
    lvi_sam::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    vector<double> cloudKeyPosesTime;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr cloudKeyGPSPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;   // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudTotalLast;    // total point set
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;     // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;   // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;
    std::vector<gtsam::GPSFactor> keyframeGPSfactor;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::PointCloud<PointType>::Ptr latestKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr nearHistoryKeyFrameCloud;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxGpsInfo;
    std::mutex mtxGraph;

    bool systemInitialized = true;
    bool isDegenerate = false;
    bool gpsTransfromInit = false;

    cv::Mat matP;

    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    int imuPreintegrationResetId = 0;

    nav_msgs::Path globalPath;

    GeographicLib::LocalCartesian geo_converter;
    Eigen::Vector3d originLLA;

    map<int, int> loopIndexContainer; // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;

    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f transPointAssociateToMap;

    mapOptimization()
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/trajectory", 1);
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/map_global", 1);
        pubOdomAftMappedROS = nh.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/lidar/mapping/odometry", 1);
        pubPath = nh.advertise<nav_msgs::Path>(PROJECT_NAME + "/lidar/mapping/path", 1);

        subLaserCloudInfo = nh.subscribe<lvi_sam::cloud_info>(PROJECT_NAME + "/lidar/feature/cloud_info", 5, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        if (gps_mode == GPSMODE::SENSOR_NAV)
        {
            subGPS = nh.subscribe<sensor_msgs::NavSatFix>(gpsTopic, 50, &mapOptimization::gpsSensorHandler, this, ros::TransportHints().tcpNoDelay());
        }
        else
        {
            subGPS = nh.subscribe<nav_msgs::Odometry>(gpsTopic, 50, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
            pubOdomRecvGPS=nh.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/lidar/mapping/recv_gps", 1);
            pubOdomUseGPS=nh.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/lidar/mapping/use_gps", 1);
        }

        subLoopInfo = nh.subscribe<std_msgs::Float64MultiArray>(PROJECT_NAME + "/vins/loop/match_frame", 5, &mapOptimization::loopHandler, this, ros::TransportHints().tcpNoDelay());

        pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/loop_closure_history_cloud", 1);
        pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/lidar/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/map_local", 1);
        pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/cloud_registered_raw", 1);
        pubCloudAndPoseRegistered = nh.advertise<lvi_sam::cloud_info>(PROJECT_NAME + "/lidar/mapping/cloud_pose_registered", 1);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        allocateMemory();
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        cloudKeyGPSPoses3D.reset(new pcl::PointCloud<PointType>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());   // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());     // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());   // downsampled surf featuer set from odoOptimization

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

        latestKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        nearHistoryKeyFrameCloud.reset(new pcl::PointCloud<PointType>());

        for (int i = 0; i < 6; ++i)
        {
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void laserCloudInfoHandler(const lvi_sam::cloud_infoConstPtr &msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        // extract info ana feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner, *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
        // pcl::fromROSMsg(msgIn->cloud_deskewed,*laserCloudTotalLast);

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {

            timeLastProcessing = timeLaserInfoCur;

            updateInitialGuess();
            if (useGPS==false||systemInitialized==true)
            {
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
        mtxGpsInfo.lock();
        gpsQueue.push_back(*gpsMsg);
        mtxGpsInfo.unlock();
    }

    // WGS84 to UTM conversion function
    bool WGS84ToUTM(double latitude, double longitude, double &utmX, double &utmY)
    {
        int zone;
        // Determine UTM zone from longitude
        zone = static_cast<int>(std::floor((longitude + 180) / 6)) + 1;

        // Set the UTM zone with appropriate hemisphere
        char utmZone[20];
        snprintf(utmZone, sizeof(utmZone), "+proj=utm +zone=%d +datum=WGS84 +units=m +ellps=WGS84", zone);

        // Create Proj contexts
        PJ_CONTEXT *context = proj_context_create();
        PJ *projection = proj_create_crs_to_crs(context, "+proj=longlat +datum=WGS84 +no_defs", utmZone, nullptr);
        if (!projection)
        {
            std::cerr << "Failed to create projection" << std::endl;
            proj_context_destroy(context);
            return false;
        }

        // Prepare coordinates (longitude, latitude) -> (easting, northing)
        PJ_COORD coord = proj_coord(longitude, latitude, 0, 0);
        PJ_COORD result = proj_trans(projection, PJ_FWD, coord);

        // Extract UTM coordinates
        utmX = result.xy.x;
        utmY = result.xy.y;

        // Cleanup
        proj_destroy(projection);
        proj_context_destroy(context);

        return true;
    }
    void gpsSensorHandler(const sensor_msgs::NavSatFix::ConstPtr &gpsMsg)
    {
        static bool init_ENU = false;
        if (gpsMsg->status.status == -1)
        {
            ROS_WARN("Lost Gps!!!");
            return;
        }
        std::cout << "gps status: " << gpsMsg->status.status << std::endl;
        if (std::isnan(gpsMsg->latitude + gpsMsg->longitude + gpsMsg->altitude))
        {
            return;
        }
        Eigen::Vector3d lla(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
        if (!init_ENU)
        {
            init_ENU = true;
            ROS_INFO("Init Orgin GPS LLA  %f, %f, %f", gpsMsg->latitude, gpsMsg->longitude,
                     gpsMsg->altitude);
            // geo_converter.Reset(lla[0], lla[1], lla[2]);
            nav_msgs::Odometry init_msg;
            init_msg.header.stamp = gpsMsg->header.stamp;
            init_msg.header.frame_id = "odom";
            init_msg.child_frame_id = "gps";
            init_msg.pose.pose.position.x = lla[0];
            init_msg.pose.pose.position.y = lla[1];
            init_msg.pose.pose.position.z = lla[2];
            init_msg.pose.covariance[0] = gpsMsg->position_covariance[0];
            init_msg.pose.covariance[7] = gpsMsg->position_covariance[4];
            init_msg.pose.covariance[14] = gpsMsg->position_covariance[8];
            // init_msg.pose.pose.orientation = yaw_quat_left;
            // init_origin_pub.publish(init_msg);
            return;
        }

        std::lock_guard<std::mutex> lock(mtx);
        // gpsQueue.push_back(*gpsMsg);
        double x, y;
        WGS84ToUTM(gpsMsg->latitude, gpsMsg->longitude, x, y);
        static std::unique_ptr<Eigen::Vector3d> gps_origin_;
        static Eigen::Vector3d prev_pos_ = Eigen::Vector3d::Zero();
        static double prev_time_ = 0;

        Eigen::Vector3d pose(x, y, 0.0);

        if (!gps_origin_)
        {
            gps_origin_ = std::make_unique<Eigen::Vector3d>(pose);
            prev_time_ = gpsMsg->header.stamp.toSec();
            std::cout << "Initialized gps_origin_: ("
                      << gps_origin_->x() << ", "
                      << gps_origin_->y() << ", "
                      << gps_origin_->z() << ")" << std::endl;
            pose -= *gps_origin_;

            return;
        }
        else
        {
            pose -= *gps_origin_;
            std::cout << "now gps_pose_: ("
                      << pose.x() << ", "
                      << pose.y() << ", "
                      << pose.z() << ")" << std::endl;
        }

        double distance = sqrt(pow(pose(1) - prev_pos_(1), 2) +
                               pow(pose(0) - prev_pos_(0), 2));
        geometry_msgs::Quaternion q;
        if (distance > 0.2)
        {
            double yaw = atan2(pose(1) - prev_pos_(1),
                               pose(0) - prev_pos_(0)); // 返回值是此点与远点连线与x轴正方向的夹角
            q = tf::createQuaternionMsgFromYaw(yaw);
        }
        else
        {
            q = tf::createQuaternionMsgFromYaw(NAN);
        }

        const int diff_gps_data = 0;
        if (diff_gps_data)
        {
            const int KNUMBEROFTWOGPSFRAME = 10;
            for (int i = KNUMBEROFTWOGPSFRAME; i <= KNUMBEROFTWOGPSFRAME; i++)
            {
                nav_msgs::Odometry odometry_msg;
                const Eigen::Vector3d diff_pos = prev_pos_ + (pose - prev_pos_) * (i * 1.00 / KNUMBEROFTWOGPSFRAME);
                odometry_msg.header = gpsMsg->header;

                const double diff_time = prev_time_ + (gpsMsg->header.stamp.toSec() - prev_time_) * (i * 1.00 / KNUMBEROFTWOGPSFRAME);
                odometry_msg.header.stamp = ros::Time().fromSec(diff_time);

                odometry_msg.pose.pose.position.x = diff_pos(0);
                odometry_msg.pose.pose.position.y = diff_pos(1);
                odometry_msg.pose.pose.position.z = diff_pos(2);

                odometry_msg.pose.pose.orientation.x = q.x;
                odometry_msg.pose.pose.orientation.y = q.y;
                odometry_msg.pose.pose.orientation.z = q.z;
                odometry_msg.pose.pose.orientation.w = q.w;

                odometry_msg.pose.covariance[0] = 0.1;
                odometry_msg.pose.covariance[7] = 0.1;
                odometry_msg.pose.covariance[14] = 0.1;
                gpsQueue.push_back(odometry_msg);
            }
        }
        else
        {
            nav_msgs::Odometry odometry_msg;
            odometry_msg.header = gpsMsg->header;

            odometry_msg.pose.pose.position.x = pose(0);
            odometry_msg.pose.pose.position.y = pose(1);
            odometry_msg.pose.pose.position.z = pose(2);

            odometry_msg.pose.pose.orientation.x = q.x;
            odometry_msg.pose.pose.orientation.y = q.y;
            odometry_msg.pose.pose.orientation.z = q.z;
            odometry_msg.pose.pose.orientation.w = q.w;

            odometry_msg.pose.covariance[0] = 0.1;
            odometry_msg.pose.covariance[7] = 0.1;
            odometry_msg.pose.covariance[14] = 0.1;
            gpsQueue.push_back(odometry_msg);
        }

        prev_pos_ = pose;
        prev_time_ = gpsMsg->header.stamp.toSec();
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

        PointType *pointFrom;

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

        for (int i = 0; i < cloudSize; ++i)
        {
            pointFrom = &cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom->x + transCur(0, 1) * pointFrom->y + transCur(0, 2) * pointFrom->z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom->x + transCur(1, 1) * pointFrom->y + transCur(1, 2) * pointFrom->z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom->x + transCur(2, 1) * pointFrom->y + transCur(2, 2) * pointFrom->z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom->intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 affine3fTogtsamPose3(const Eigen::Affine3f &thisPose)
    {
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(thisPose, x, y, z, roll, pitch, yaw);
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(roll), double(pitch), double(yaw)),
                            gtsam::Point3(double(x), double(y), double(z)));
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

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;
        // create directory and remove old files;
        savePCDDirectory = std::getenv("HOME") + savePCDDirectory;
        int unused = system((std::string("exec rm -r ") + savePCDDirectory).c_str());
        unused = system((std::string("mkdir ") + savePCDDirectory).c_str());
        ++unused;
        // save key frame transformations
        pcl::io::savePCDFileASCII(savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileBinary(savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D);
        // extract global point cloud map
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++)
        {
            // clip cloud
            // pcl::PointCloud<PointType>::Ptr cornerTemp(new pcl::PointCloud<PointType>());
            // pcl::PointCloud<PointType>::Ptr cornerTemp2(new pcl::PointCloud<PointType>());
            // *cornerTemp = *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
            // for (int j = 0; j < (int)cornerTemp->size(); ++j)
            // {
            //     if (cornerTemp->points[j].z > cloudKeyPoses6D->points[i].z && cornerTemp->points[j].z < cloudKeyPoses6D->points[i].z + 5)
            //         cornerTemp2->push_back(cornerTemp->points[j]);
            // }
            // pcl::PointCloud<PointType>::Ptr surfTemp(new pcl::PointCloud<PointType>());
            // pcl::PointCloud<PointType>::Ptr surfTemp2(new pcl::PointCloud<PointType>());
            // *surfTemp = *transformPointCloud(surfCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
            // for (int j = 0; j < (int)surfTemp->size(); ++j)
            // {
            //     if (surfTemp->points[j].z > cloudKeyPoses6D->points[i].z && surfTemp->points[j].z < cloudKeyPoses6D->points[i].z + 5)
            //         surfTemp2->push_back(surfTemp->points[j]);
            // }
            // *globalCornerCloud += *cornerTemp2;
            // *globalSurfCloud   += *surfTemp2;

            // origin cloud
            *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        }
        // down-sample and save corner cloud
        downSizeFilterCorner.setInputCloud(globalCornerCloud);
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudCorner.pcd", *globalCornerCloud);
        // down-sample and save surf cloud
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudSurf.pcd", *globalSurfCloud);
        // down-sample and save global point cloud map
        *globalMapCloud += *globalCornerCloud;
        *globalMapCloud += *globalSurfCloud;
        downSizeFilterSurf.setInputCloud(globalMapCloud);
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed" << endl;
    }

    void publishGlobalMap()
    {
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
        ;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

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
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;                                                                                   // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(&pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, "odom");
    }

    void loopHandler(const std_msgs::Float64MultiArray::ConstPtr &loopMsg)
    {
        // control loop closure frequency
        static double last_loop_closure_time = -1;
        {
            // std::lock_guard<std::mutex> lock(mtx);
            if (timeLaserInfoCur - last_loop_closure_time < 5.0)
                return;
            else
                last_loop_closure_time = timeLaserInfoCur;
        }

        performLoopClosure(*loopMsg);
    }

    void performLoopClosure(const std_msgs::Float64MultiArray &loopMsg)
    {
        pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());
        {
            std::lock_guard<std::mutex> lock(mtx);
            *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        }

        // get lidar keyframe id
        int key_cur = -1; // latest lidar keyframe id
        int key_pre = -1; // previous lidar keyframe id
        {
            loopFindKey(loopMsg, copy_cloudKeyPoses6D, key_cur, key_pre);
            if (key_cur == -1 || key_pre == -1 || key_cur == key_pre) // || abs(key_cur - key_pre) < 25)
                return;
        }

        // check if loop added before
        {
            // if image loop closure comes at high frequency, many image loop may point to the same key_cur
            auto it = loopIndexContainer.find(key_cur);
            if (it != loopIndexContainer.end())
                return;
        }

        // get lidar keyframe cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(copy_cloudKeyPoses6D, cureKeyframeCloud, key_cur, 0);
            loopFindNearKeyframes(copy_cloudKeyPoses6D, prevKeyframeCloud, key_pre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, "odom");
        }

        // get keyframe pose
        Eigen::Affine3f pose_cur;
        Eigen::Affine3f pose_pre;
        Eigen::Affine3f pose_diff_t; // serves as initial guess
        {
            pose_cur = pclPointToAffine3f(copy_cloudKeyPoses6D->points[key_cur]);
            pose_pre = pclPointToAffine3f(copy_cloudKeyPoses6D->points[key_pre]);

            Eigen::Vector3f t_diff;
            t_diff.x() = -(pose_cur.translation().x() - pose_pre.translation().x());
            t_diff.y() = -(pose_cur.translation().y() - pose_pre.translation().y());
            t_diff.z() = -(pose_cur.translation().z() - pose_pre.translation().z());
            if (t_diff.norm() < historyKeyframeSearchRadius)
                t_diff.setZero();
            pose_diff_t = pcl::getTransformation(t_diff.x(), t_diff.y(), t_diff.z(), 0, 0, 0);
        }

        // transform and rotate cloud for matching
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        // pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
        icp.setMaximumIterations(100);
        icp.setRANSACIterations(0);
        icp.setTransformationEpsilon(1e-3);
        icp.setEuclideanFitnessEpsilon(1e-3);

        // initial guess cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud_new(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *cureKeyframeCloud_new, pose_diff_t);

        // match using icp
        icp.setInputSource(cureKeyframeCloud_new);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud_new, *closed_cloud, icp.getFinalTransformation());
            publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, "odom");
        }

        // add graph factor
        if (icp.getFitnessScore() < historyKeyframeFitnessScore && icp.hasConverged() == true)
        {
            // get gtsam pose
            gtsam::Pose3 poseFrom = affine3fTogtsamPose3(Eigen::Affine3f(icp.getFinalTransformation()) * pose_diff_t * pose_cur);
            gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[key_pre]);
            // get noise
            float noise = icp.getFitnessScore();
            gtsam::Vector Vector6(6);
            Vector6 << noise, noise, noise, noise, noise, noise;
            noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);
            // save pose constraint
            mtx.lock();
            loopIndexQueue.push_back(make_pair(key_cur, key_pre));
            loopPoseQueue.push_back(poseFrom.between(poseTo));
            loopNoiseQueue.push_back(constraintNoise);
            mtx.unlock();
            // add loop pair to container
            loopIndexContainer[key_cur] = key_pre;
        }

        // visualize loop constraints
        if (!loopIndexContainer.empty())
        {
            visualization_msgs::MarkerArray markerArray;
            // loop nodes
            visualization_msgs::Marker markerNode;
            markerNode.header.frame_id = "odom";
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
            markerEdge.header.frame_id = "odom";
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
    }

    void loopFindNearKeyframes(const pcl::PointCloud<PointTypePose>::Ptr &copy_cloudKeyPoses6D,
                               pcl::PointCloud<PointType>::Ptr &nearKeyframes,
                               const int &key, const int &searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int key_near = key + i;
            if (key_near < 0 || key_near >= cloudSize)
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[key_near], &copy_cloudKeyPoses6D->points[key_near]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[key_near], &copy_cloudKeyPoses6D->points[key_near]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void loopFindKey(const std_msgs::Float64MultiArray &loopMsg,
                     const pcl::PointCloud<PointTypePose>::Ptr &copy_cloudKeyPoses6D,
                     int &key_cur, int &key_pre)
    {
        if (loopMsg.data.size() != 2)
            return;

        double loop_time_cur = loopMsg.data[0];
        double loop_time_pre = loopMsg.data[1];

        if (abs(loop_time_cur - loop_time_pre) < historyKeyframeSearchTimeDiff)
            return;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return;

        // latest key
        key_cur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time > loop_time_cur)
                key_cur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        key_pre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time < loop_time_pre)
                key_pre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }
    }

    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(0.5);
        while (ros::ok())
        {
            rate.sleep();
            performLoopClosureDetection();
        }
    }

    void performLoopClosureDetection()
    {
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;

        int key_cur = -1;
        int key_pre = -1;

        double loop_time_cur = -1;
        double loop_time_pre = -1;

        // find latest key and time
        {
            std::lock_guard<std::mutex> lock(mtx);

            if (cloudKeyPoses3D->empty())
                return;

            kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
            kdtreeHistoryKeyPoses->radiusSearch(cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

            key_cur = cloudKeyPoses3D->size() - 1;
            loop_time_cur = cloudKeyPoses6D->points[key_cur].time;
        }

        // find previous key and time
        {
            for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
            {
                int id = pointSearchIndLoop[i];
                if (abs(cloudKeyPoses6D->points[id].time - loop_time_cur) > historyKeyframeSearchTimeDiff)
                {
                    key_pre = id;
                    loop_time_pre = cloudKeyPoses6D->points[key_pre].time;
                    break;
                }
            }
        }

        if (key_cur == -1 || key_pre == -1 || key_pre == key_cur ||
            loop_time_cur < 0 || loop_time_pre < 0)
            return;

        std_msgs::Float64MultiArray match_msg;
        match_msg.data.push_back(loop_time_cur);
        match_msg.data.push_back(loop_time_pre);
        performLoopClosure(match_msg);
    }

    bool syncGPS(std::deque<nav_msgs::Odometry> &gpsBuf,
                 nav_msgs::Odometry &aligedGps, double timestamp,
                 double eps_cam)
    {
        bool hasGPS = false;
        // ROS_INFO("sync gps data\n");
        // return false;
        while (!gpsQueue.empty())
        {

            mtxGpsInfo.lock();
            ROS_INFO("gpsQueue %lf %lf %lf", gpsQueue.front().header.stamp.toSec(), timestamp, gpsQueue.back().header.stamp.toSec());
            if (gpsQueue.front().header.stamp.toSec() < timestamp - eps_cam)
            {
                // message too old
                gpsQueue.pop_front();
                mtxGpsInfo.unlock();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timestamp + eps_cam)
            {
                // message too new
                mtxGpsInfo.unlock();
                break;
            }
            else
            {
                hasGPS = true;
                aligedGps = gpsQueue.front();
                gpsQueue.pop_front();
                mtxGpsInfo.unlock();
            }
        }

        if (hasGPS)
            return true;
        else
            return false;
    }

    void updateInitialGuess()
    {
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);
        static Eigen::Affine3f lastImuTransformation;
        // system initialization
        if (cloudKeyPoses3D->points.empty())
        {
            systemInitialized = false;
            if (useGPS)
            {
                ROS_WARN("GPS use to init pose");
                /** when you align gnss and lidar timestamp, make sure (1.0/gpsFrequence) is small encougn
                 *  no need to care about the real gnss frquency. time alignment fail will cause
                 *  "[ERROR] [1689196991.604771416]: sysyem need to be initialized"
                 * */
                nav_msgs::Odometry alignedGPS;
                const int sync_gps_flag = syncGPS(gpsQueue, alignedGPS, timeLaserInfoCur, 1.0 / 10);
                ROS_WARN("updateInitialGuess sync gps data flag %d\n", sync_gps_flag);
                if (sync_gps_flag)
                {
                    ROS_WARN("updateInitialGuess sync gps data finish %lf %lf %lf",
                             alignedGPS.pose.pose.position.x, alignedGPS.pose.pose.position.y, alignedGPS.pose.pose.position.z);
                    /** we store the origin wgs84 coordinate points in covariance[1]-[3] */
                    originLLA.setIdentity();
                    originLLA = Eigen::Vector3d(alignedGPS.pose.covariance[1],
                                                alignedGPS.pose.covariance[2],
                                                alignedGPS.pose.covariance[3]);
                    /** set your map origin points */
                    geo_converter.Reset(originLLA[0], originLLA[1], originLLA[2]);
                    // WGS84->ENU, must be (0,0,0)
                    Eigen::Vector3d enu;
                    geo_converter.Forward(originLLA[0], originLLA[1], originLLA[2], enu[0], enu[1], enu[2]);

                    if (1)
                    {
                        double roll, pitch, yaw;
                        tf::Matrix3x3(tf::Quaternion(alignedGPS.pose.pose.orientation.x,
                                                     alignedGPS.pose.pose.orientation.y,
                                                     alignedGPS.pose.pose.orientation.z,
                                                     alignedGPS.pose.pose.orientation.w))
                            .getRPY(roll, pitch, yaw);
                        std::cout << "initial gps yaw: " << yaw << std::endl;
                        std::cout << "GPS Position: " << enu.transpose() << std::endl;
                        std::cout << "GPS LLA: " << originLLA.transpose() << std::endl;
                    }

                    /** add the first factor, we need this origin GPS point for prior map based localization,
                     * but we need to optimize its value by pose graph if the origin gps RTK status is not fixed.*/
                    PointType gnssPoint;
                    gnssPoint.x = enu[0],
                    gnssPoint.y = enu[1],
                    gnssPoint.z = enu[2];
                    float noise_x = alignedGPS.pose.covariance[0];
                    float noise_y = alignedGPS.pose.covariance[7];
                    float noise_z = alignedGPS.pose.covariance[14];

                    /** if we get reliable origin point, we adjust the weight of this gps factor to fix the map origin */
                    // if (!updateOrigin) {
                    noise_x *= 1e-4;
                    noise_y *= 1e-4;
                    noise_z *= 1e-4;
                    // }
                    gtsam::Vector Vector3(3);
                    Vector3 << noise_x, noise_y, noise_z;
                    noiseModel::Diagonal::shared_ptr gps_noise =
                        noiseModel::Diagonal::Variances(Vector3);
                    gtsam::GPSFactor gps_factor(0, gtsam::Point3(gnssPoint.x, gnssPoint.y, gnssPoint.z),
                                                gps_noise);
                    keyframeGPSfactor.push_back(gps_factor);
                    cloudKeyGPSPoses3D->points.push_back(gnssPoint);

                    transformTobeMapped[0] = cloudInfo.imuRollInit;
                    transformTobeMapped[1] = cloudInfo.imuPitchInit;
                    transformTobeMapped[2] = cloudInfo.imuYawInit;
                    if (!useImuHeadingInitialization)
                        transformTobeMapped[2] = 0;
                    lastImuTransformation = pcl::getTransformation(
                        0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit,
                        cloudInfo.imuYawInit);
                    systemInitialized = true;
                    ROS_WARN("GPS init success");
                }
                else
                {
                    // transformTobeMapped[0] = cloudInfo.imuRollInit;
                    // transformTobeMapped[1] = cloudInfo.imuPitchInit;
                    // transformTobeMapped[2] = cloudInfo.imuYawInit;

                    // if (!useImuHeadingInitialization)
                    //     transformTobeMapped[2] = 0;

                    // lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                    // return;
                }
            }
            else
            {
                transformTobeMapped[0] = cloudInfo.imuRollInit;
                transformTobeMapped[1] = cloudInfo.imuPitchInit;
                transformTobeMapped[2] = cloudInfo.imuYawInit;

                if (!useImuHeadingInitialization)
                    transformTobeMapped[2] = 0;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                return;
            }
        }

        if (useGPS==true&&!systemInitialized)
        {
            ROS_ERROR("sysyem need to be initialized");
            return;
        }
        // use VINS odometry estimation for pose guess
        static int odomResetId = 0;
        static bool lastVinsTransAvailable = false;
        static Eigen::Affine3f lastVinsTransformation;
        if (cloudInfo.odomAvailable == true && cloudInfo.odomResetId == odomResetId)
        {
            // ROS_INFO("Using VINS initial guess");
            if (lastVinsTransAvailable == false)
            {
                // ROS_INFO("Initializing VINS initial guess");
                //! 重要：使用vins发来的T_odom_lidar位姿进行初始的位姿估计
                lastVinsTransformation = pcl::getTransformation(cloudInfo.odomX, cloudInfo.odomY, cloudInfo.odomZ,
                                                                cloudInfo.odomRoll, cloudInfo.odomPitch, cloudInfo.odomYaw);
                lastVinsTransAvailable = true;
            }
            else
            {
                // ROS_INFO("Obtaining VINS incremental guess");
                Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.odomX, cloudInfo.odomY, cloudInfo.odomZ,
                                                                   cloudInfo.odomRoll, cloudInfo.odomPitch, cloudInfo.odomYaw);
                Eigen::Affine3f transIncre = lastVinsTransformation.inverse() * transBack;

                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                  transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastVinsTransformation = pcl::getTransformation(cloudInfo.odomX, cloudInfo.odomY, cloudInfo.odomZ,
                                                                cloudInfo.odomRoll, cloudInfo.odomPitch, cloudInfo.odomYaw);

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                return;
            }
        }
        else
        {
            // ROS_WARN("VINS failure detected.");
            lastVinsTransAvailable = false;
            odomResetId = cloudInfo.odomResetId;
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuAvailable == true)
        {
            // ROS_INFO("Using IMU initial guess");
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
        std::vector<pcl::PointCloud<PointType>> laserCloudCornerSurroundingVec;
        std::vector<pcl::PointCloud<PointType>> laserCloudSurfSurroundingVec;

        laserCloudCornerSurroundingVec.resize(cloudToExtract->size());
        laserCloudSurfSurroundingVec.resize(cloudToExtract->size());

// extract surrounding map
#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (pointDistance(cloudKeyPoses3D->points[thisKeyInd], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;
            laserCloudCornerSurroundingVec[i] = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            laserCloudSurfSurroundingVec[i] = *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }

        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            *laserCloudCornerFromMap += laserCloudCornerSurroundingVec[i];
            *laserCloudSurfFromMap += laserCloudSurfSurroundingVec[i];
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        extractNearby();
    }

    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
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

            pointOri = laserCloudCornerLastDS->points[i];
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

            pointOri = laserCloudSurfLastDS->points[i];
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

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

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
        // This optimization is from the original loam_velodyne, need to cope with coordinate transformation
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
            // lidar -> camera
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
                double imuWeight = 0.01;
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

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
        
        //float frame_base=;
        if(gpsTransfromInit==false)
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
            ROS_INFO("cloudKeyPoses3D size == 0\n");
            std::cout << trans2gtsamPose(transformTobeMapped) << std::endl;
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }
        else
        {
            ROS_INFO("cloudKeyPoses3D size == %d\n", cloudKeyPoses3D->size());
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
            // if (isDegenerate)
            // {
            // adding VINS constraints is deleted as benefits are not obvious, disable for now
            // gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), vinsPoseFrom.between(vinsPoseTo), odometryNoise));
            // }
        }
    }
    int LowerFind(double time)
    {
        int left = 0, right = cloudKeyPosesTime.size() - 1, index = -1;
        while (left <= right)
        {
            int mid = (left + right) >> 1;
            if (cloudKeyPosesTime[mid] >= time)
            {
                right = mid - 1;
                index = mid;
            }
            else
            {
                left = mid + 1;
            }
        }
        return index;
    }
    void addGPSFactor()
    {
        static int add_gps_factor_func = 0;
        add_gps_factor_func += 1;

        if (gpsQueue.empty())
            return;
        // ROS_INFO("add_gps_factor_func=%d size=%d %lf %lf %lf", add_gps_factor_func,
        //          gpsQueue.size(), gpsQueue.front().header.stamp.toSec(),
        //          gpsQueue.back().header.stamp.toSec(), timeLaserInfoCur);
        // ROS_INFO("cloudKeyPoses3D size=%d ", cloudKeyPoses3D->points.size());
        //  wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty() || cloudKeyPoses3D->points.size() == 1)
            return;

        // else if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
        //     return;

        // pose covariance small, no need to correct
        // if (poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold)
        //    return;

        // last gps position
        static PointType lastGPSPoint;
        lastGPSPoint.x = 0;
        lastGPSPoint.y = 0;
        lastGPSPoint.z = 0;
        static int gps_factor_count = 0;
        static int gps_factor_find = 0;
        static double delta_distance = 0;

        nav_msgs::Odometry thisGPS;
        while (gpsQueue.empty() == false)
        {
            thisGPS = gpsQueue.front();
            const double gps_time = thisGPS.header.stamp.toSec();
            const float noise_x = max(thisGPS.pose.covariance[0],2.0);
            const float noise_y = max(thisGPS.pose.covariance[7],2.0);
            float noise_z = max(thisGPS.pose.covariance[14],2.0);
            if (abs(noise_x) > gpsCovThreshold || abs(noise_y) > gpsCovThreshold)
            {
                ROS_INFO("too large gps noise %lf %lf\n",noise_x,noise_y);
                gpsQueue.pop_front();
                continue;
            }
            double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0;
            Eigen::Vector3d LLA(thisGPS.pose.covariance[1], thisGPS.pose.covariance[2], thisGPS.pose.covariance[3]);
            geo_converter.Forward(LLA[0], LLA[1], LLA[2], gps_x, gps_y, gps_z);
            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
            {
                ROS_INFO("too near origin point\n");
                gpsQueue.pop_front();
                continue;
            }

            nav_msgs::Odometry recv_gps_data;
            recv_gps_data.header=thisGPS.header;
            recv_gps_data.pose.pose.position.x=gps_x;
            recv_gps_data.pose.pose.position.y=gps_y;
            recv_gps_data.pose.pose.position.z=gps_z;
            pubOdomRecvGPS.publish(recv_gps_data);

            PointType curGPSPoint;
            curGPSPoint.x = gps_x;
            curGPSPoint.y = gps_y;
            curGPSPoint.z = gps_z;
            // if (pointDistance(curGPSPoint, lastGPSPoint) < GPSDISTANCE)
            // {
            //     ROS_INFO("too distance\n");
            //     gpsQueue.pop_front();
            //     continue;
            // }
            // else
            {
                lastGPSPoint = curGPSPoint;
            }

            int pose3d_index = LowerFind(gps_time);
            if (pose3d_index == -1)
            {
                ROS_INFO("not find time bigger than this gps time\n");
                break;
            }
            else
            {
                int time_check_flag=0;

                if (abs(cloudKeyPosesTime[pose3d_index] - gps_time) > 0.15)
                {
                    if(pose3d_index>0&&abs(cloudKeyPosesTime[pose3d_index-1] - gps_time) < 0.15){
                        pose3d_index-=1;
                        time_check_flag=1;
                    }
                    else{
                        ROS_INFO("too big time diff\n");
                        gpsQueue.pop_front();
                        time_check_flag=0;
                        continue;
                    }
                    
                }
                else{
                    time_check_flag=1;
                }
                if(time_check_flag)
                {
                    gpsQueue.pop_front();
                    if (!useGpsElevation)
                    {
                        gps_z = cloudKeyPoses3D->points[pose3d_index].z;
                        noise_z = 0.01;
                    }
                    gtsam::Vector Vector3(3);
                    Vector3 << noise_x, noise_y, noise_z;
                    // Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                    noiseModel::Diagonal::shared_ptr gps_noise =
                        noiseModel::Diagonal::Variances(Vector3);
                    gtsam::GPSFactor gps_factor(pose3d_index+1,
                                                gtsam::Point3(gps_x, gps_y, gps_z),
                                                gps_noise);

                    keyframeGPSfactor.push_back(gps_factor);
                    cloudKeyGPSPoses3D->points.push_back(curGPSPoint);
                    nav_msgs::Odometry use_gps_data;
                    use_gps_data.header=thisGPS.header;
                    use_gps_data.pose.pose.position.x=gps_x;
                    use_gps_data.pose.pose.position.y=gps_y;
                    use_gps_data.pose.pose.position.z=gps_z;
                    pubOdomUseGPS.publish(use_gps_data);
                    if (keyframeGPSfactor.size() < 15)
                    {
                        ROS_INFO("Accumulated gps factor: %d", keyframeGPSfactor.size());
                        continue;
                    }

                    if (!gpsTransfromInit)
                    {
                        ROS_INFO("Initialize GNSS transform!");
                        mtxGraph.lock();
                        for (int i = 0; i < keyframeGPSfactor.size(); ++i)
                        {
                            const gtsam::GPSFactor gpsFactor = keyframeGPSfactor.at(i);
                            gtSAMgraph.add(gpsFactor);
                            // gpsIndexContainer[gpsFactor.key()] = i;
                        }
                        gpsTransfromInit = true;
                        mtxGraph.unlock();
                    }
                    else
                    {
                        mtxGraph.lock();
                        gtSAMgraph.add(gps_factor);
                        mtxGraph.unlock();
                    }
                    aLoopIsClosed = true;
                }
            }
        }ROS_INFO("Accumulated gps factor: %d", keyframeGPSfactor.size());
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;
        static int loop_factor_number=0;
        for (size_t i = 0; i < loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
            loop_factor_number+=1;
        }
        ROS_INFO("loop factor number %d",loop_factor_number);
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
        if (useGPS)
        {
            addGPSFactor();
        }

        // loop factor
        addLoopFactor();

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

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
        cloudKeyPosesTime.push_back(timeLaserInfoCur);

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
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
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
            // ID for reseting IMU pre-integration
            ++imuPreintegrationResetId;
        }
    }

    void publishOdometry()
    {
        // Publish odometry for ROS
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = "odom";
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        laserOdometryROS.pose.covariance[0] = double(imuPreintegrationResetId);
        if(useGPS==false||gpsTransfromInit==true)
        {
            pubOdomAftMappedROS.publish(laserOdometryROS);
        }
        
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, "odom", "lidar_link");
        br.sendTransform(trans_odom_to_lidar);
    }

    void updatePath(const PointTypePose &pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = "odom";
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

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(&pubKeyPoses, cloudKeyPoses6D, timeLaserInfoStamp, "odom");
        // Publish surrounding key frames
        //pcl::PointCloud<PointType>::Ptr mergedCloud(new pcl::PointCloud<PointType>());
        //*mergedCloud+=*laserCloudSurfFromMapDS;
        //*mergedCloud+=*laserCloudCornerFromMapDS;
        //pcl::concatenatePointCloud(*laserCloudSurfFromMapDS, *laserCloudCornerFromMapDS, *mergedCloud);
        publishCloud(&pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, "odom");
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
            publishCloud(&pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, "odom");
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut, &thisPose6D);
            publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, "odom");
        }
        if (pubCloudAndPoseRegistered.getNumSubscribers() != 0)
        {
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);

            pcl::PointCloud<PointType>::Ptr cloud_input(new pcl::PointCloud<PointType>()), cloud_output(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloud_input);
            *cloud_output += *transformPointCloud(cloud_input, &thisPose6D);

            lvi_sam::cloud_info msg;
            msg.header = cloudInfo.header;
            msg.header.frame_id = "odom";
            msg.odomX = transformTobeMapped[3];
            msg.odomY = transformTobeMapped[4];
            msg.odomZ = transformTobeMapped[5];
            pcl::toROSMsg(*cloud_output, msg.cloud_deskewed);
            pubCloudAndPoseRegistered.publish(msg);
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = "odom";
            pubPath.publish(globalPath);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Lidar Map Optimization Started.\033[0m");

    std::thread loopDetectionthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    ros::spin();

    loopDetectionthread.join();
    visualizeMapThread.join();

    return 0;
}
