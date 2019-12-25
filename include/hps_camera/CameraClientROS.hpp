/**
 * @file    CameraClientROS.hpp
 *
 * @date    2019-12-23
 *
 * Copyright (c) organization
 *
 */
#ifndef CAMERACLIENTROS_HPP_
#define CAMERACLIENTROS_HPP_
#pragma once

#include <ros/ros.h>

#include <memory>

namespace hps_camera {

class DeviceHandler;

class CameraClientROS {
 public:
    CameraClientROS();

    ~CameraClientROS();

    bool isInitialized() const;

 protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();

 private:
    ros::NodeHandle _nh;
    std::unique_ptr<DeviceHandler> _deviceHandler;
};

}  // namespace hps_camera
#endif /* CAMERACLIENTROS_HPP_ */
