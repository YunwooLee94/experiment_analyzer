//
// Created by larr-proj on 22. 3. 30..
//
#include <ros/ros.h>
#include <std_msgs/Bool.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "flag_record_on");
    ros::NodeHandle nh("~");
    ros::Publisher pubFlagOn;
    pubFlagOn = nh.advertise<std_msgs::Bool>("record_on",1);

    ros::Rate loop_rate_on(10.0);
    std_msgs::Bool flag_on;
    flag_on.data = true;
    while(ros::ok())
    {
        pubFlagOn.publish(flag_on);
        ros::spinOnce();
        loop_rate_on.sleep();
    }
    return 0;
}