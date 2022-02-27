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
private:
    ros::NodeHandle nh;
    string global_frame_id;
    string optical_frame_id;
    string base_frame_id;
    string left_frame_id;
    string drone_pose_topic_name;
    zed_interfaces::ObjectsStamped * transformedObjects;

    tf::TransformBroadcaster* tfBroadcasterPtr;
    tf::TransformListener* tfListenerPtr;
    State state;
//    vector<zedBox3d> transformedObjCorners;

//    ros::Subscriber subZedPose;
    ros::Subscriber subPointCloud;
    ros::Subscriber subZedPose;
    ros::Subscriber subObjPose;
    void cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr & robotPoseInfo);
    void cbPointCloud(const sensor_msgs::PointCloud2::ConstPtr & pointCloudInfo);
    void cbObject(const zed_interfaces::ObjectsStamped::ConstPtr & objectInfo);
    bool isDronePoseReceived = false;

    ros::Publisher pubZed2Pose;
    ros::Publisher pubPointCloud;
    ros::Publisher pubObjectsDetection;

    sensor_msgs::PointCloud2 pclReceived;

    mutex mSet[3]; // 0: Pose, 1: Pointcloud 2: Object Detection

    geometry_msgs::PoseStamped drone_pose;


    void prepareROSmsgs();
    void publishROSmsgs();




public:
    RosWrapper();
    void run();
};


#endif //EXPERIMENT_ANALYZER_WRAPPER_H
