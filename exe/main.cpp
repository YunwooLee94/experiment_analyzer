//
// Created by larr-proj on 22. 2. 24..
//
#include <experiment_analyzer/Wrapper.h>
int main(int argc, char** argv){
    ros::init(argc, argv, "analyzer");
    RosWrapper analyzer_wrapper;
    analyzer_wrapper.run();
    return 0;
}