//
// Created by larr-proj on 22. 2. 24..
//

#include <experiment_analyzer/Wrapper.h>

void RosWrapper::cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &robotPoseInfo) {
    mSet[0].lock();
    isDronePoseReceived = true;
    drone_pose.header.frame_id = robotPoseInfo->header.frame_id;
    drone_pose.pose.position = robotPoseInfo->pose.position;
    drone_pose.pose.orientation.x = robotPoseInfo->pose.orientation.y;
    drone_pose.pose.orientation.y = -robotPoseInfo->pose.orientation.x;
    drone_pose.pose.orientation.z = robotPoseInfo->pose.orientation.w;
    drone_pose.pose.orientation.w = -robotPoseInfo->pose.orientation.z;
    mSet[0].unlock();
    //    cout<<"Hello"<<endl;
}

void RosWrapper::cbObject(const zed_interfaces::ObjectsStamped::ConstPtr &objectInfo) {
    if(not objectInfo->objects.empty() and isDronePoseReceived)
    {
        tf::Transform transform_Twb;
        {  //Twb

            transform_Twb.setOrigin(tf::Vector3( drone_pose.pose.position.x, drone_pose.pose.position.y,drone_pose.pose.position.z));
            tf::Quaternion q;
            q.setX(drone_pose.pose.orientation.x);
            q.setY(drone_pose.pose.orientation.y);
            q.setZ(drone_pose.pose.orientation.z);
            q.setW(drone_pose.pose.orientation.w);
            transform_Twb.setRotation(q);
        }
        tf::Transform transform_Tbc;
        {   //Tbc

            transform_Tbc.setOrigin(tf::Vector3(0.0,-0.06,0.0));
            tf::Quaternion q;
//            q.setRPY(-M_PI_2, 0.0, -M_PI_2);
            q.setRPY(0.0, 0.0, 0.0);
            transform_Tbc.setRotation(q);
        }
        tf::Transform transform_Twc = transform_Twb*transform_Tbc;
        Eigen::Transform<float,3,Eigen::Affine> poseMat;
        poseMat.setIdentity();
        Eigen::Vector3f loc(transform_Twc.getOrigin().x(),transform_Twc.getOrigin().y(),transform_Twc.getOrigin().z());
        poseMat.translate(loc);
        Eigen::Quaternionf quaternionf;
        quaternionf.setIdentity();
        quaternionf.w() = transform_Twc.getRotation().w();
        quaternionf.x() = transform_Twc.getRotation().x();
        quaternionf.y() = transform_Twc.getRotation().y();
        quaternionf.z() = transform_Twc.getRotation().z();
        poseMat.rotate(quaternionf);

        tfBroadcasterPtr->sendTransform(tf::StampedTransform(transform_Twc, objectInfo->header.stamp,"map", left_frame_id));

//        {
//            mSet[2].lock();
//            transformedObjects->objects.clear();
//            transformedObjects->header = objectInfo->header;
//            mSet[2].unlock();
////            transformedObjCorners.clear();
//
//            for(const auto &detected_obj : objectInfo->objects){
//                zedBox3d tempBox;
//                for(int i = 0 ; i<8;i++) // bounding box
//                {
//                    analyzer_client::Point tempVec;
//                    tempVec.x =detected_obj.bounding_box_3d.corners.elems[i].kp.elems[0];
//                    tempVec.y =detected_obj.bounding_box_3d.corners.elems[i].kp.elems[1];
//                    tempVec.z =detected_obj.bounding_box_3d.corners.elems[i].kp.elems[2];
//
//                    analyzer_client::Point tempTFVec;
//                    tempTFVec = poseMat*analyzer_client::Point (tempVec.x, tempVec.y,tempVec.z).toEigen();
//                    tempBox.corners.push_back(tempTFVec);
//                }
//
//                for(int i = 0 ; i<18;i++)
//                {
//                    analyzer_client::Point tempVec;
//                    tempVec.x = detected_obj.skeleton_3d.keypoints[i].kp.elems[0];
//                    tempVec.y = detected_obj.skeleton_3d.keypoints[i].kp.elems[1];
//                    tempVec.z = detected_obj.skeleton_3d.keypoints[i].kp.elems[2];
//
//                    analyzer_client::Point tempTFVec;
//                    tempTFVec = poseMat*analyzer_client::Point (tempVec.x, tempVec.y,tempVec.z).toEigen();
//                    tempBox.corners.push_back(tempTFVec);
//                    tempBox.skeletons.push_back(tempVec);
//                }
//                zed_interfaces::Object tempObject;
//                tempObject.skeleton_available = true;
//                for(int i = 0; i<8;i++)
//                {
//                    tempObject.bounding_box_3d.corners.elems[i].kp.elems[0] = tempBox.corners[i].x;
//                    tempObject.bounding_box_3d.corners.elems[i].kp.elems[1] = tempBox.corners[i].y;
//                    tempObject.bounding_box_3d.corners.elems[i].kp.elems[2] = tempBox.corners[i].z;
//                }
//                for(int i = 0;i<18;i++)
//                {
//                    tempObject.skeleton_3d.keypoints[i].kp.elems[0] = tempBox.skeletons[i].x;
//                    tempObject.skeleton_3d.keypoints[i].kp.elems[1] = tempBox.skeletons[i].y;
//                    tempObject.skeleton_3d.keypoints[i].kp.elems[2] = tempBox.skeletons[i].z;
//                }
//                mSet[2].lock();
//                transformedObjects->objects.push_back(tempObject);
//                mSet[2].unlock();
//            }
//        }
    }

}

void RosWrapper::cbPointCloud(const sensor_msgs::PointCloud2::ConstPtr &pointCloudInfo) {
    if(not pointCloudInfo->fields.empty() and isDronePoseReceived)
    {
        tf::Transform transform_Twb;
        {  //Twb

            transform_Twb.setOrigin(tf::Vector3( drone_pose.pose.position.x, drone_pose.pose.position.y,drone_pose.pose.position.z));
            tf::Quaternion q;
            q.setX(drone_pose.pose.orientation.x);
            q.setY(drone_pose.pose.orientation.y);
            q.setZ(drone_pose.pose.orientation.z);
            q.setW(drone_pose.pose.orientation.w);
            transform_Twb.setRotation(q);
        }
        tf::Transform transform_Tbc;
        {   //Tbc


            transform_Tbc.setOrigin(tf::Vector3(0.0,-0.06,0.0));
            tf::Quaternion q;
            q.setRPY(-M_PI_2, 0.0, -M_PI_2);
            transform_Tbc.setRotation(q);
        }
        tf::Transform transform_Twc = transform_Twb*transform_Tbc;

        tfBroadcasterPtr->sendTransform(tf::StampedTransform(transform_Twc, pointCloudInfo->header.stamp,"map", optical_frame_id));
    }
}

RosWrapper::RosWrapper() {
    nh = ros::NodeHandle("~");
    nh.param<string>("global_frame_id", global_frame_id, "map");
    nh.param<string>("optical_frame_id", optical_frame_id,"zed2_left_camera_optical_frame");
    nh.param<string>("base_frame_id", base_frame_id, "base_link");
    nh.param<string>("left_frame_id", left_frame_id, "zed2_left_camera_frame");
    subZedPose = nh.subscribe("/zed_add/zed_node/pose", 2, &RosWrapper::cbCurrentPose,this);
    subPointCloud = nh.subscribe("/zed_client_node/points_removed",2, &RosWrapper::cbPointCloud, this);
    subObjPose = nh.subscribe("/zed2/zed_node/obj_det/objects",2, &RosWrapper::cbObject,this);
    pubZed2Pose = nh.advertise<geometry_msgs::PoseStamped>("zed2_pose",1);
    pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>("point_cloud",1);
    pubObjectsDetection = nh.advertise<zed_interfaces::ObjectsStamped>("detected_objects",1);
    transformedObjects = new zed_interfaces::ObjectsStamped;
    tfBroadcasterPtr = new tf::TransformBroadcaster;
//    cout<<"CONSTRUCTED"<<endl;
}


void RosWrapper::prepareROSmsgs() {

}

void RosWrapper::publishROSmsgs() {
    {   //Drone Pose Publish
        mSet[0].lock();
        if(isDronePoseReceived)
        {
            pubZed2Pose.publish(drone_pose);
        }
        mSet[0].unlock();
    }

    {
        mSet[1].lock();
        if(not pclReceived.data.empty())
        {
            pubPointCloud.publish(pclReceived);
        }
        mSet[1].unlock();
    }
    {
        mSet[2].lock();
        if(not transformedObjects->objects.empty())
        {
            pubObjectsDetection.publish(*transformedObjects);
        }
        mSet[2].unlock();
    }
}


void RosWrapper::run(){
    ros::Rate loop_rate_wrapper(20.0);
    while(ros::ok())
    {
//        cout<<"Hello"<<endl;
//        cout<<"isDronePoseReceived"<<isDronePoseReceived<<endl;
        prepareROSmsgs();
        publishROSmsgs();
        ros::spinOnce();
        loop_rate_wrapper.sleep();
    }
}