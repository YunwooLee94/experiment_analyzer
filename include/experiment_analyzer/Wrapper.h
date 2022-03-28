//
// Created by larr-proj on 22. 2. 24..
//

#ifndef EXPERIMENT_ANALYZER_WRAPPER_H
#define EXPERIMENT_ANALYZER_WRAPPER_H

#include <ros/ros.h>
#include <string>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <chrono>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <experiment_analyzer/Utils.h>

using namespace std;





class RosWrapper{
    struct State{
        pcl::PointCloud<pcl::PointXYZRGB> pcl_Extracted;
    };
    struct zedBox3d{
        vector<analyzer_client::Point> corners;
        vector<analyzer_client::Point> skeletons;
    };
    struct reachCircle{
        vector<double> x_array;
        vector<double> y_array;
        vector<double> r_array;
    };
    struct posInfo{
        double x;
        double y;
        double z;
    };
private:
    ros::NodeHandle nh;
    string global_frame_id;
    string optical_frame_id;
    string base_frame_id;
    string left_frame_id;
    string drone_pose_topic_name;
    zed_interfaces::ObjectsStamped * transformedObjects;
    vector<double> height_object;
    vector<posInfo> pos_object;
    reachCircle targetReach;
    reachCircle dummyReach;

    visualization_msgs::MarkerArray targetReach_modified;
    visualization_msgs::MarkerArray dummyReach_modified;

    visualization_msgs::MarkerArray optimization_modified;

    nav_msgs::Path targetPosHistory;
    nav_msgs::Path dummyPosHistory;
    nav_msgs::Path dronePosHistory;

    nav_msgs::Path targetPosHistory_rev;
    nav_msgs::Path dummyPosHistory_rev;


    tf::TransformBroadcaster* tfBroadcasterPtr;
    tf::TransformListener* tfListenerPtr;
    State state;
//    vector<zedBox3d> transformedObjCorners;

//    ros::Subscriber subZedPose;
    ros::Subscriber subPointCloud;
    ros::Subscriber subZedPose;
    ros::Subscriber subObjPose;
    ros::Subscriber subTargetReach;
    ros::Subscriber subDummyReach;
    ros::Subscriber subTargetPosHistory;
    ros::Subscriber subDronePosHistory;
    ros::Subscriber subDummyPosHistory;
    ros::Subscriber subOptimizationResult;

    void cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr & robotPoseInfo);
    void cbPointCloud(const sensor_msgs::PointCloud2::ConstPtr & pointCloudInfo);
    void cbObject(const zed_interfaces::ObjectsStamped::ConstPtr & objectInfo);
    void cbTargetReach(const visualization_msgs::MarkerArray::ConstPtr & targetReachInfo);
    void cbDummyReach(const visualization_msgs::MarkerArray::ConstPtr & dummyReachInfo);
    void cbDronePosHistory(const visualization_msgs::MarkerArray::ConstPtr & droneHistoryInfo);
    void cbTargetPosHistory(const visualization_msgs::MarkerArray::ConstPtr & targetHistoryInfo);
    void cbDummyPosHistory(const visualization_msgs::MarkerArray::ConstPtr & dummyHistoryInfo);
    void cbOptimizationResult(const visualization_msgs::MarkerArray::ConstPtr & optimizationInfo);
    bool isDronePoseReceived = false;

    ros::Publisher pubZed2Pose;
    ros::Publisher pubPointCloud;
    ros::Publisher pubObjectsDetection;
    ros::Publisher pubTargetReach;
    ros::Publisher pubDummyReach;
    ros::Publisher pubDronePosHistory;
    ros::Publisher pubTargetPosHistory;
    ros::Publisher pubDummyPosHistory;

    ros::Publisher pubTargetPosHistoryRev;
    ros::Publisher pubDummyPosHistoryRev;

    ros::Publisher pubOptimizationResult;

    sensor_msgs::PointCloud2 pclReceived;

    mutex mSet[9]; // 0: Pose, 1: Pointcloud 2: Object Detection 3: TargetReach 4: DummyReach
    //5:Drone Pos History 6: Target Pos History 7: Dummy Pos History 8: OptimizationResult
    geometry_msgs::PoseStamped drone_pose;


    void prepareROSmsgs();
    void publishROSmsgs();




public:
    RosWrapper();
    void run();
};


#endif //EXPERIMENT_ANALYZER_WRAPPER_H
