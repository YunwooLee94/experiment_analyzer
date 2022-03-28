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

void RosWrapper::cbTargetReach(const visualization_msgs::MarkerArray::ConstPtr &targetReachInfo) {
    mSet[3].lock();
    targetReach.x_array.clear();
    targetReach.y_array.clear();
    targetReach.r_array.clear();
    mSet[3].unlock();

    mSet[3].lock();
    for(const auto &target_circle: targetReachInfo->markers)
    {
        targetReach.x_array.push_back(target_circle.pose.position.x);
        targetReach.y_array.push_back(target_circle.pose.position.y);
        targetReach.r_array.push_back(0.5*target_circle.scale.x);
    }
    mSet[3].unlock();
}

void RosWrapper::cbDronePosHistory(const visualization_msgs::MarkerArray::ConstPtr &droneHistoryInfo) {
    mSet[5].lock();
    dronePosHistory.poses.clear();
    mSet[5].unlock();
    geometry_msgs::PoseStamped tempDronePos;
    dronePosHistory.header.frame_id = global_frame_id;
    mSet[5].lock();
    for(int i =0;i<droneHistoryInfo->markers.size();i++)
    {
        tempDronePos.pose.position.x = droneHistoryInfo->markers[i].pose.position.x;
        tempDronePos.pose.position.y = droneHistoryInfo->markers[i].pose.position.y;
        tempDronePos.pose.position.z = droneHistoryInfo->markers[i].pose.position.z;
        if(tempDronePos.pose.position.z>1.6)
            tempDronePos.pose.position.z=1.6;
        tempDronePos.pose.orientation.w = 1.0;
        tempDronePos.pose.orientation.x = 0.0;
        tempDronePos.pose.orientation.y = 0.0;
        tempDronePos.pose.orientation.z = 0.0;
        dronePosHistory.poses.push_back(tempDronePos);
    }
    mSet[5].unlock();
}

void RosWrapper::cbTargetPosHistory(const visualization_msgs::MarkerArray::ConstPtr &targetHistoryInfo) {
//    mSet[6].lock();
//    targetPosHistory.poses.clear();
//    mSet[6].unlock();
//    geometry_msgs::PoseStamped tempTargetPos;
//    targetPosHistory.header.frame_id = global_frame_id;
//    mSet[6].lock();
//    for(int i =0;i<targetHistoryInfo->markers.size();i++)
//    {
//        tempTargetPos.pose.position.x = targetHistoryInfo->markers[i].pose.position.x;
//        tempTargetPos.pose.position.y = targetHistoryInfo->markers[i].pose.position.y;
//        tempTargetPos.pose.position.z = targetHistoryInfo->markers[i].pose.position.z;
//        if(tempTargetPos.pose.position.z>1.6)
//            tempTargetPos.pose.position.z=1.6;
//        tempTargetPos.pose.orientation.w = 1.0;
//        tempTargetPos.pose.orientation.x = 0.0;
//        tempTargetPos.pose.orientation.y = 0.0;
//        tempTargetPos.pose.orientation.z = 0.0;
//        targetPosHistory.poses.push_back(tempTargetPos);
//    }
//    mSet[6].unlock();

    static int cnt_target_pos=0;
    static int cnt_target_hz=0;
//    cout<<"cnt_target_hz"<<cnt_target_hz<<endl;
    if(cnt_target_pos<2)
    {
        geometry_msgs::PoseStamped tempTargetPos;
        mSet[6].lock();
        targetPosHistory.header.frame_id = global_frame_id;
        mSet[6].unlock();
        int num_pos = (int) targetHistoryInfo->markers.size();

        tempTargetPos.pose.position.x = targetHistoryInfo->markers[num_pos-1].pose.position.x;
        tempTargetPos.pose.position.y = targetHistoryInfo->markers[num_pos-1].pose.position.y;
        tempTargetPos.pose.position.z = targetHistoryInfo->markers[num_pos-1].pose.position.z;
        if(tempTargetPos.pose.position.z>1.6)
            tempTargetPos.pose.position.z=1.6;
        tempTargetPos.pose.orientation.w = 1.0;
        tempTargetPos.pose.orientation.x = 0.0;
        tempTargetPos.pose.orientation.y = 0.0;
        tempTargetPos.pose.orientation.z = 0.0;
        mSet[6].lock();
        targetPosHistory.poses.push_back(tempTargetPos);
        mSet[6].unlock();
        cnt_target_pos++;
        cnt_target_hz++;
    }
    else
    {
        if(cnt_target_hz==0)
        {
            geometry_msgs::PoseStamped tempTargetPos1;
            geometry_msgs::PoseStamped tempTargetPos2;
            geometry_msgs::PoseStamped tempTargetPos3;
            geometry_msgs::PoseStamped tempTargetPos0;

            int topic_last_idx = (int) targetHistoryInfo->markers.size()-1;
            int history_last_idx = (int) targetPosHistory.poses.size()-1;

            tempTargetPos0.pose.position.x = 0.0*targetPosHistory.poses[history_last_idx-1].pose.position.x +
                                             0.8*targetPosHistory.poses[history_last_idx].pose.position.x +
                                             0.2*targetHistoryInfo->markers[topic_last_idx].pose.position.x;
            tempTargetPos0.pose.position.y = 0.0*targetPosHistory.poses[history_last_idx-1].pose.position.y +
                                             0.8*targetPosHistory.poses[history_last_idx].pose.position.y +
                                             0.2*targetHistoryInfo->markers[topic_last_idx].pose.position.y;
            tempTargetPos0.pose.position.z = 0.0*targetPosHistory.poses[history_last_idx-1].pose.position.z +
                                             0.8*targetPosHistory.poses[history_last_idx].pose.position.z +
                                             0.2*targetHistoryInfo->markers[topic_last_idx].pose.position.z;
            if(tempTargetPos0.pose.position.z>1.6)
                tempTargetPos0.pose.position.z=1.6;
            tempTargetPos0.pose.orientation.w = 1.0;
            tempTargetPos0.pose.orientation.x = 0.0;
            tempTargetPos0.pose.orientation.y = 0.0;
            tempTargetPos0.pose.orientation.z = 0.0;
            mSet[6].lock();
            targetPosHistory.poses.push_back(tempTargetPos0);
            mSet[6].unlock();


            tempTargetPos1.pose.position.x = 0.1*targetPosHistory.poses[history_last_idx-1].pose.position.x +
                                             0.4*targetPosHistory.poses[history_last_idx].pose.position.x +
                                             0.5*targetHistoryInfo->markers[topic_last_idx].pose.position.x;
            tempTargetPos1.pose.position.y = 0.1*targetPosHistory.poses[history_last_idx-1].pose.position.y +
                                             0.4*targetPosHistory.poses[history_last_idx].pose.position.y +
                                             0.5*targetHistoryInfo->markers[topic_last_idx].pose.position.y;
            tempTargetPos1.pose.position.z = 0.1*targetPosHistory.poses[history_last_idx-1].pose.position.z +
                                             0.4*targetPosHistory.poses[history_last_idx].pose.position.z +
                                             0.5*targetHistoryInfo->markers[topic_last_idx].pose.position.z;
            if(tempTargetPos1.pose.position.z>1.6)
                tempTargetPos1.pose.position.z=1.6;
            tempTargetPos1.pose.orientation.w = 1.0;
            tempTargetPos1.pose.orientation.x = 0.0;
            tempTargetPos1.pose.orientation.y = 0.0;
            tempTargetPos1.pose.orientation.z = 0.0;
            mSet[6].lock();
            targetPosHistory.poses.push_back(tempTargetPos1);
            mSet[6].unlock();

            tempTargetPos2.pose.position.x = 0.1*targetPosHistory.poses[history_last_idx-1].pose.position.x +
                                             0.2*targetPosHistory.poses[history_last_idx].pose.position.x +
                                             0.7*targetHistoryInfo->markers[topic_last_idx].pose.position.x;
            tempTargetPos2.pose.position.y = 0.1*targetPosHistory.poses[history_last_idx-1].pose.position.y +
                                             0.2*targetPosHistory.poses[history_last_idx].pose.position.y +
                                             0.7*targetHistoryInfo->markers[topic_last_idx].pose.position.y;
            tempTargetPos2.pose.position.z = 0.1*targetPosHistory.poses[history_last_idx-1].pose.position.z +
                                             0.2*targetPosHistory.poses[history_last_idx].pose.position.z +
                                             0.7*targetHistoryInfo->markers[topic_last_idx].pose.position.z;
            if(tempTargetPos2.pose.position.z>1.6)
                tempTargetPos2.pose.position.z=1.6;
            tempTargetPos2.pose.orientation.w = 1.0;
            tempTargetPos2.pose.orientation.x = 0.0;
            tempTargetPos2.pose.orientation.y = 0.0;
            tempTargetPos2.pose.orientation.z = 0.0;
            mSet[6].lock();
            targetPosHistory.poses.push_back(tempTargetPos2);
            mSet[6].unlock();

            tempTargetPos3.pose.position.x = 0.0*targetPosHistory.poses[history_last_idx-1].pose.position.x +
                                             0.0*targetPosHistory.poses[history_last_idx].pose.position.x +
                                             1.0*targetHistoryInfo->markers[topic_last_idx].pose.position.x;
            tempTargetPos3.pose.position.y = 0.0*targetPosHistory.poses[history_last_idx-1].pose.position.y +
                                             0.0*targetPosHistory.poses[history_last_idx].pose.position.y +
                                             1.0*targetHistoryInfo->markers[topic_last_idx].pose.position.y;
            tempTargetPos3.pose.position.z = 0.0*targetPosHistory.poses[history_last_idx-1].pose.position.z +
                                             0.0*targetPosHistory.poses[history_last_idx].pose.position.z +
                                             1.0*targetHistoryInfo->markers[topic_last_idx].pose.position.z;
            if(tempTargetPos3.pose.position.z>1.6)
                tempTargetPos3.pose.position.z=1.6;
            tempTargetPos3.pose.orientation.w = 1.0;
            tempTargetPos3.pose.orientation.x = 0.0;
            tempTargetPos3.pose.orientation.y = 0.0;
            tempTargetPos3.pose.orientation.z = 0.0;
            mSet[6].lock();
            targetPosHistory.poses.push_back(tempTargetPos3);
            mSet[6].unlock();



//            cout<<"HIIIIIIIIIIIIIIIIIIIIIII"<<endl;
            cnt_target_hz++;
        }
        else
        {
            cnt_target_hz++;
        }

        if(cnt_target_hz>10)
            cnt_target_hz=0;
    }

}

void RosWrapper::cbDummyPosHistory(const visualization_msgs::MarkerArray::ConstPtr &dummyHistoryInfo) {
//    mSet[7].lock();
//    dummyPosHistory.poses.clear();
//    mSet[7].unlock();
//    geometry_msgs::PoseStamped tempDummyPos;
//    dummyPosHistory.header.frame_id = global_frame_id;
//    mSet[7].lock();
//    for(int i =0;i<dummyHistoryInfo->markers.size();i++)
//    {
//        tempDummyPos.pose.position.x = dummyHistoryInfo->markers[i].pose.position.x;
//        tempDummyPos.pose.position.y = dummyHistoryInfo->markers[i].pose.position.y;
//        tempDummyPos.pose.position.z = dummyHistoryInfo->markers[i].pose.position.z;
//        if(tempDummyPos.pose.position.z>1.6)
//            tempDummyPos.pose.position.z=1.6;
//        tempDummyPos.pose.orientation.w = 1.0;
//        tempDummyPos.pose.orientation.x = 0.0;
//        tempDummyPos.pose.orientation.y = 0.0;
//        tempDummyPos.pose.orientation.z = 0.0;
//        dummyPosHistory.poses.push_back(tempDummyPos);
//    }
//    mSet[7].unlock();

    static int cnt_dummy_pos = 0;
    static int cnt_dummy_hz = 0;


}

void RosWrapper::cbOptimizationResult(const visualization_msgs::MarkerArray::ConstPtr &optimizationInfo) {
//    cout<<"HJHello"<<endl;
    mSet[8].lock();
    optimization_modified.markers.clear();
    mSet[8].unlock();
//    cout<<"TTTTTTTTTT"<<endl;
    visualization_msgs::Marker tempOptMarker;
    double drone_z;
    mSet[0].lock();
    if(isDronePoseReceived)
    {
        drone_z = drone_pose.pose.position.z;
    }
    else
    {
        drone_z = 0.0;
    }
    mSet[0].unlock();
//    cout<<"FFFFFFFFFFFFF"<<endl;
    mSet[8].lock();
//    cout<<"Size:  "<<optimizationInfo->markers.size()<<endl;
    for(int i = 0; i<(int)optimizationInfo->markers.size();i++)
    {
        tempOptMarker.pose.position = optimizationInfo->markers[i].pose.position;
        tempOptMarker.pose.position.z = drone_z;
        tempOptMarker.pose.orientation = optimizationInfo->markers[i].pose.orientation;
        tempOptMarker.id = i;
        tempOptMarker.ns = std::to_string(i);
        tempOptMarker.type = visualization_msgs::Marker::CUBE;
        tempOptMarker.header.frame_id = global_frame_id;
        tempOptMarker.scale.x = 0.2;
        tempOptMarker.scale.y = 0.2;
        tempOptMarker.scale.z = 0.2;
        tempOptMarker.color.a = 0.5;
        tempOptMarker.color.r = 0.0;
        tempOptMarker.color.g = 0.0;
        tempOptMarker.color.b = 1.0;
//        cout<<"TTTTTTTTTTTTT"<<endl;
        optimization_modified.markers.push_back(tempOptMarker);
    }
//    cout<<"TTTTAASDDSCSDC"<<endl;
    mSet[8].unlock();
//    cout<<"GGGGG"<<endl;
}

void RosWrapper::cbDummyReach(const visualization_msgs::MarkerArray::ConstPtr &dummyReachInfo) {
    mSet[4].lock();
    dummyReach.x_array.clear();
    dummyReach.y_array.clear();
    dummyReach.r_array.clear();
    mSet[4].unlock();
    mSet[4].lock();
    for(const auto &dummy_circle: dummyReachInfo->markers)
    {
        dummyReach.x_array.push_back(dummy_circle.pose.position.x);
        dummyReach.y_array.push_back(dummy_circle.pose.position.y);
        dummyReach.r_array.push_back(0.5*dummy_circle.scale.x);
    }
    mSet[4].unlock();

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

        {
            mSet[2].lock();
            height_object.clear();
            pos_object.clear();
            mSet[2].unlock();
            int cnt_object = 0;
            for(const auto &detected_obj:objectInfo->objects){
                double min_z_comp = +99999;
                double max_z_comp = -99999;
                for(int i =0;i<8;i++){
                    if(min_z_comp>detected_obj.bounding_box_3d.corners.elems[i].kp.elems[2]){
                        min_z_comp = detected_obj.bounding_box_3d.corners.elems[i].kp.elems[2];
                    }
                    if(max_z_comp<detected_obj.bounding_box_3d.corners.elems[i].kp.elems[2]){
                        max_z_comp = detected_obj.bounding_box_3d.corners.elems[i].kp.elems[2];
                    }
                }
                posInfo tempPos{};
                tempPos.x = detected_obj.head_position.elems[0];
                tempPos.y = detected_obj.head_position.elems[1];
                tempPos.z = 0.5*(max_z_comp-min_z_comp);

                geometry_msgs::PoseStamped tempPose;
                tempPose.header.frame_id = global_frame_id;
                tempPose.pose.position.x = tempPos.x;
                tempPose.pose.position.y = tempPos.y;
                tempPose.pose.position.z = tempPos.z;
                if(cnt_object==0)
                {
                    targetPosHistory_rev.header.frame_id = global_frame_id;
                    mSet[2].lock();
                    targetPosHistory_rev.poses.push_back(tempPose);
                    mSet[2].unlock();
                }
                if(cnt_object==1)
                {
                    dummyPosHistory_rev.header.frame_id = global_frame_id;
                    mSet[2].lock();
                    dummyPosHistory_rev.poses.push_back(tempPose);
                    mSet[2].unlock();
                }
                mSet[2].lock();
                height_object.push_back(max_z_comp-min_z_comp);
                pos_object.push_back(tempPos);
                mSet[2].unlock();
                cnt_object++;
            }
        }

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
    subTargetReach = nh.subscribe("/target_handler_target/target_manager/reach_info",2,&RosWrapper::cbTargetReach, this);
    subDummyReach = nh.subscribe("/target_handler_dummy/target_manager/reach_info",2, &RosWrapper::cbDummyReach,this);
    subDronePosHistory = nh.subscribe("/server/drone_pos_history",2, &RosWrapper::cbDronePosHistory,this);
    subTargetPosHistory = nh.subscribe("/server/target_pos_history",2, &RosWrapper::cbTargetPosHistory,this);
    subDummyPosHistory = nh.subscribe("/server/dummy_pos_history",2, &RosWrapper::cbDummyPosHistory,this);
    subOptimizationResult = nh.subscribe("/server/plan_traj_marker",1, &RosWrapper::cbOptimizationResult,this);

    pubZed2Pose = nh.advertise<geometry_msgs::PoseStamped>("zed2_pose",1);
    pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>("point_cloud",1);
    pubObjectsDetection = nh.advertise<zed_interfaces::ObjectsStamped>("detected_objects",1);
    pubTargetReach = nh.advertise<visualization_msgs::MarkerArray>("target_reach_info",1);
    pubDummyReach = nh.advertise<visualization_msgs::MarkerArray>("dummmy_reach_info",1);
    pubDronePosHistory =nh.advertise<nav_msgs::Path>("drone_pos_history",1);
    pubTargetPosHistory = nh.advertise<nav_msgs::Path>("target_pos_history",1);
    pubDummyPosHistory = nh.advertise<nav_msgs::Path>("dummy_pos_history",1);

    pubDummyPosHistoryRev = nh.advertise<nav_msgs::Path>("dummy_pos_history_rev",1);
    pubTargetPosHistoryRev = nh.advertise<nav_msgs::Path>("target_pos_history_rev",1);

    pubOptimizationResult = nh.advertise<visualization_msgs::MarkerArray>("optimization_result",1);


    transformedObjects = new zed_interfaces::ObjectsStamped;
    tfBroadcasterPtr = new tf::TransformBroadcaster;
//    cout<<"CONSTRUCTED"<<endl;
}


void RosWrapper::prepareROSmsgs() {

    { // Target Reach Info
//        cout<<"Hello"<<endl;
        mSet[3].lock();
        targetReach_modified.markers.clear();
        mSet[3].unlock();
        visualization_msgs::Marker tempMarker;
        mSet[3].lock();
        for(int i = 0 ; i< targetReach.x_array.size();i++)
        {
            tempMarker.pose.position.x = targetReach.x_array[i];
            tempMarker.pose.position.y = targetReach.y_array[i];
//            tempMarker.pose.position.z = 0.5*height_object[0];
            tempMarker.pose.position.z = 0.8;
            tempMarker.pose.orientation.w = 1.0;
            tempMarker.pose.orientation.x = 0.0;
            tempMarker.pose.orientation.y = 0.0;
            tempMarker.pose.orientation.z = 0.0;

//            tempMarker.scale.x = 2*targetReach.r_array[i];
//            tempMarker.scale.y = 2*targetReach.r_array[i];
            tempMarker.scale.x = 2*(0.2+0.025*(i+1));
            tempMarker.scale.y = 2*(0.2+0.025*(i+1));

//            tempMarker.scale.z = height_object[0];
            tempMarker.scale.z = 1.6;
            tempMarker.header.frame_id = global_frame_id;
            tempMarker.id = i;
            tempMarker.ns = std::to_string(i);
            tempMarker.type = visualization_msgs::Marker::SPHERE;
            tempMarker.color.a = 0.2;
            tempMarker.color.r = 1.0;
            tempMarker.color.g = 0.0;
            tempMarker.color.b = 0.0;
            targetReach_modified.markers.emplace_back(tempMarker);
        }
        mSet[3].unlock();
//        cout<<targetReach_modified.markers.size()<<endl;
//        cout<<"HiHiHiHi"<<endl;
    }
    if(height_object.size()>1) //Dummy Reach Info
    { // Dummy Reach Info
        mSet[4].lock();
        dummyReach_modified.markers.clear();
        mSet[4].unlock();
        visualization_msgs::Marker tempMarker;
        mSet[4].lock();
        for(int i = 0 ; i< dummyReach.x_array.size();i++)
        {
            tempMarker.pose.position.x = dummyReach.x_array[i];
            tempMarker.pose.position.y = dummyReach.y_array[i];
//            tempMarker.pose.position.z = 0.5*height_object[1];
            tempMarker.pose.position.z = 0.8;

            tempMarker.pose.orientation.w = 1.0;
            tempMarker.pose.orientation.x = 0.0;
            tempMarker.pose.orientation.y = 0.0;
            tempMarker.pose.orientation.z = 0.0;
//            tempMarker.scale.x = 2*dummyReach.r_array[i];
//            tempMarker.scale.y = 2*dummyReach.r_array[i];
            tempMarker.scale.x = 2*(0.2+0.025*(i+1));
            tempMarker.scale.y = 2*(0.2+0.025*(i+1));

//            tempMarker.scale.z = height_object[1];
            tempMarker.scale.z = 1.6;

            tempMarker.header.frame_id = global_frame_id;
            tempMarker.id = i;
            tempMarker.ns = std::to_string(i);
            tempMarker.type = visualization_msgs::Marker::SPHERE;
            tempMarker.color.a = 0.2;
            tempMarker.color.r = 0.0;
            tempMarker.color.g = 1.0;
            tempMarker.color.b = 0.0;
            dummyReach_modified.markers.emplace_back(tempMarker);
        }
        mSet[4].unlock();
    }
    else
    {
        dummyReach_modified.markers.clear();
    }
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
//        if(not transformedObjects->objects.empty())
//        {
//            pubObjectsDetection.publish(*transformedObjects);
//        }
//        if(not targetPosHistory_rev.poses.empty())
//            pubTargetPosHistoryRev.publish(targetPosHistory_rev);
//        if(not dummyPosHistory_rev.poses.empty())
//            pubDummyPosHistoryRev.publish(dummyPosHistory_rev);

        mSet[2].unlock();
    }
    {
        mSet[3].lock();
        if(not targetReach_modified.markers.empty())
        {
            pubTargetReach.publish(targetReach_modified);
        }
        mSet[3].unlock();
    }
    {
        mSet[4].lock();
        if(not dummyReach_modified.markers.empty())
        {
            pubDummyReach.publish(dummyReach_modified);
        }
        mSet[4].unlock();
    }
    {
        mSet[5].lock();
        if(not dronePosHistory.poses.empty())
            pubDronePosHistory.publish(dronePosHistory);
        mSet[5].unlock();
    }
    {
        mSet[6].lock();
        if(not targetPosHistory.poses.empty())
            pubTargetPosHistory.publish(targetPosHistory);
        mSet[6].unlock();
    }
    {
        mSet[7].lock();
        if(not dummyPosHistory.poses.empty())
            pubDummyPosHistory.publish(dummyPosHistory);
        mSet[7].unlock();
    }
    {
        mSet[8].lock();
        if(not optimization_modified.markers.empty())
            pubOptimizationResult.publish(optimization_modified);
        mSet[8].unlock();
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