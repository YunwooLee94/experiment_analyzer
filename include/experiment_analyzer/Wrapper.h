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
using namespace std;


class RosWrapper{
private:
    ros::NodeHandle nh;
    string global_frame_id;
    string optical_frame_id;
    string base_frame_id;
    string drone_pose_topic_name;

//    ros::Subscriber subZedPose;
    ros::Subscriber subPointCloud;

    void cbCurrentPose(const geometry_msgs::PoseStamped & robotPoseInfo);
    bool isDronePoseReceived = false;

    ros::Publisher pubZed2Pose;



    geometry_msgs::PoseStamped drone_pose;


    void prepareROSmsgs();
    void publishROSmsgs();

public:
    RosWrapper();
    void run();
};


#endif //EXPERIMENT_ANALYZER_WRAPPER_H
