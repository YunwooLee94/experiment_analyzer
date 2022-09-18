//
// Created by larr-proj on 22. 3. 30..
//

#include <ros/ros.h>
#include <std_msgs/Bool.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "flag_record_off");
    ros::NodeHandle nh("~");
    ros::Publisher pubFlagOff;
    pubFlagOff = nh.advertise<std_msgs::Bool>("record_off",1);

    ros::Rate loop_rate_off(10.0);
    std_msgs::Bool flag_off;
    flag_off.data = true;
    while(ros::ok())
    {
        pubFlagOff.publish(flag_off);
        ros::spinOnce();
        loop_rate_off.sleep();
    }
}