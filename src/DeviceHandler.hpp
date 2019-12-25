/**
 * @file    DeviceHandler.hpp
 *
 * @date    2019-12-24
 *
 * Copyright (c) organization
 *
 */

#ifndef DEVICEHANDLER_HPP_
#define DEVICEHANDLER_HPP_
#pragma once

#include <ros/console.h>
#include <ros/ros.h>

#include <hps3d/api.h>

namespace hps_camera {
class DeviceHandler {
 public:
    static ros::Publisher pointcloudPub;

    explicit DeviceHandler(const size_t transportType);
    ~DeviceHandler();

    bool initDevice(const ros::NodeHandle& nh);

    const bool isInitialized() const;

 private:
    static void handleSignal(const int sig);

    static void* callbackHandler(HPS3D_HandleTypeDef* handle, AsyncIObserver_t* event);

    void setHDRConf(const ros::NodeHandle& nh);

    void setKalManFilter(const ros::NodeHandle& nh);

    void setSmoothFilter(const ros::NodeHandle& nh);

    void setMountingAngle(const ros::NodeHandle& nh);

    void setEdgeDetection(const ros::NodeHandle& nh);

 private:
    static HPS3D_HandleTypeDef handler;

    static AsyncIObserver_t eventObserver;

    size_t _transportType;

    bool _isInitialized;
};
}  // namespace hps_camera
#endif /* DEVICEHANDLER_HPP_ */
