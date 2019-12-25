/**
 * @file    CameraClientROSMain.cpp
 *
 * @date    2019-12-25
 *
 * Copyright (c) organization
 *
 */

#include "hps_camera/CameraClientROS.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ros_camera_client");

    hps_camera::CameraClientROS ccr;
    if (!ccr.isInitialized()) {
        ROS_INFO("Failed to initialize the device\n");
        return 0;
    }

    ros::Rate loopRate(10);
    while (ros::ok()) {
        loopRate.sleep();
    }

    return 0;
}
