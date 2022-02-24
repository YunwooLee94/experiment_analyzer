//
// Created by larr-proj on 22. 2. 24..
//

#include <experiment_analyzer/Wrapper.h>

void RosWrapper::cbCurrentPose(const geometry_msgs::PoseStamped &robotPoseInfo) {
    isDronePoseReceived = true;
    drone_pose.header.frame_id = robotPoseInfo.header.frame_id;
    drone_pose.pose.position = robotPoseInfo.pose.position;
    drone_pose.pose.orientation.x = robotPoseInfo.pose.orientation.y;
    drone_pose.pose.orientation.y = -robotPoseInfo.pose.orientation.x;
    drone_pose.pose.orientation.z = robotPoseInfo.pose.orientation.w;
    drone_pose.pose.orientation.w = -robotPoseInfo.pose.orientation.z;
//    cout<<"Hello"<<endl;
}

RosWrapper::RosWrapper() {
    nh = ros::NodeHandle("~");
    nh.param<string>("global_frame_id", global_frame_id, "map");
    nh.param<string>("optical_frame_id", optical_frame_id,"zed2_left_camera_optical_frame");
    nh.param<string>("base_frame_id", base_frame_id, "base_link");

    ros::Subscriber subZedPose = nh.subscribe("/zed_add/zed_node/pose", 2, &RosWrapper::cbCurrentPose, this);
    pubZed2Pose = nh.advertise<geometry_msgs::PoseStamped>("zed2_pose",1);

//    cout<<"CONSTRUCTED"<<endl;
}


void RosWrapper::prepareROSmsgs() {

}

void RosWrapper::publishROSmsgs() {
    {   //Drone Pose Publish
        if(isDronePoseReceived)
        {
            cout<<"Received the pose"<<endl;
            pubZed2Pose.publish(drone_pose);
        }

    }

}


void RosWrapper::run(){
    ros::Rate loop_rate_wrapper(5);
    while(ros::ok())
    {
//        cout<<"Hello"<<endl;
        cout<<"isDronePoseReceived"<<isDronePoseReceived<<endl;
        prepareROSmsgs();
        publishROSmsgs();
        ros::spinOnce();
        loop_rate_wrapper.sleep();
    }
}