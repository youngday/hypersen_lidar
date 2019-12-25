/**
 * @file    CameraClientROS.cpp
 *
 * @date    2019-12-23
 *
 * Copyright (c) organization
 *
 */
#include <sensor_msgs/PointCloud2.h>

#include "DeviceHandler.hpp"
#include "hps_camera/CameraClientROS.hpp"

namespace hps_camera {

CameraClientROS::CameraClientROS() : _deviceHandler(nullptr) {
    int transportType;
    this->_nh.param<int>("transport_type", transportType, 0);
    assert(transportType == 0 || transportType == 1);

    this->_deviceHandler = std::unique_ptr<DeviceHandler>(new DeviceHandler(transportType));
    this->onInit();
}

CameraClientROS::~CameraClientROS() = default;

void CameraClientROS::onInit() {
    this->subscribe();

    // device handler's point cloud publisher needs to be assigned before device initialization
    this->_deviceHandler->pointcloudPub = this->_nh.advertise<sensor_msgs::PointCloud2>("pointcloud_output", 1);
    this->_deviceHandler->initDevice(this->_nh);
}

void CameraClientROS::subscribe() {
}

void CameraClientROS::unsubscribe() {
}

bool CameraClientROS::isInitialized() const {
    return this->_deviceHandler->isInitialized();
}

}  // namespace hps_camera
