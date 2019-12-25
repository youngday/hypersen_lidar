/**
 * @file    DeviceHandler.cpp
 *
 * @date    2019-12-24
 *
 * Copyright (c) organization
 *
 */

#include "DeviceHandler.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <csignal>

namespace hps_camera {

HPS3D_HandleTypeDef DeviceHandler::handler = HPS3D_HandleTypeDef();
AsyncIObserver_t DeviceHandler::eventObserver = AsyncIObserver_t();
ros::Publisher DeviceHandler::pointcloudPub;

DeviceHandler::DeviceHandler(const size_t transportType)
    : _transportType(transportType),  //
      _isInitialized(false) {
}

DeviceHandler::~DeviceHandler() {
    HPS3D_RemoveDevice(&DeviceHandler::handler);
}

bool DeviceHandler::initDevice(const ros::NodeHandle& nh) {
    if (signal(SIGINT, handleSignal) == SIG_ERR || signal(SIGTSTP, handleSignal) == SIG_ERR) {
        ROS_ERROR("Signal initialization failed\n ");
        return false;
    }

    bool debugEnable;
    nh.param<bool>("debug_enable", debugEnable, false);
    HPS3D_SetDebugEnable(debugEnable);
    HPS3D_SetDebugFunc([](char* str) { ROS_INFO("%s", str); });

    if (this->_transportType == 1) {
        std::string ethernetServerInfo;
        nh.param<std::string>("ethernet_server_info", ethernetServerInfo, "192.168.0.10");
        HPS3D_SetEthernetServerInfo(&DeviceHandler::handler, const_cast<char*>(ethernetServerInfo.c_str()), 12345);
    } else {
        char fileName[10][20];
        int devCnt = HPS3D_GetDeviceList(const_cast<char*>("/dev/"), const_cast<char*>("ttyACM"), fileName);
        DeviceHandler::handler.DeviceName = fileName[0];
    }

    RET_StatusTypeDef ret = HPS3D_Connect(&DeviceHandler::handler);
    if (ret != RET_OK) {
        ROS_INFO("Device open failed,ret = %d\n", ret);
        return false;
    }

    ret = HPS3D_ConfigInit(&DeviceHandler::handler);
    if (RET_OK != ret) {
        ROS_INFO("Initialization failed:%d\n", ret);
        return false;
    }
    ROS_INFO("Initialization succeed\n");

    // Set convert point cloud data enable
    HPS3D_SetOpticalEnable(&DeviceHandler::handler, true);
    HPS3D_SetPointCloudEn(true);

    this->setHDRConf(nh);
    this->setKalManFilter(nh);
    this->setSmoothFilter(nh);
    this->setMountingAngle(nh);
    this->setEdgeDetection(nh);

    // Add observer one
    eventObserver.AsyncEvent = ISubject_Event_DataRecvd;
    eventObserver.NotifyEnable = true;
    HPS3D_AddObserver(&DeviceHandler::callbackHandler, &DeviceHandler::handler, &DeviceHandler::eventObserver);

    DeviceHandler::handler.RunMode = RUN_CONTINUOUS;
    HPS3D_SetRunMode(&DeviceHandler::handler);

    this->_isInitialized = true;

    return this->_isInitialized;
}

void DeviceHandler::setHDRConf(const ros::NodeHandle& nh) {
    HDRConf hdrConf;

    int hdrConfHdrMode;
    double hdrConfQualityOverexposed;
    double hdrConfQualityOverexposedSerious;
    double hdrConfQualityWeak;
    double hdrConfQualityWeakSerious;
    int hdrConfSimpleHdrMaxIntegration;
    int hdrConfSimpleHdrMinIntegration;
    int hdrConfSuperHdrFrameNumber;
    int hdrConfSuperHdrMaxIntegration;
    int hdrConfHdrDisableIntegrationTime;

    nh.param<int>("hdr_mode", hdrConfHdrMode, 1);
    nh.param<double>("quality_over_exposed", hdrConfQualityOverexposed, 500.);
    nh.param<double>("quality_over_exposed_serious", hdrConfQualityOverexposedSerious, 800.);
    nh.param<double>("quality_weak", hdrConfQualityWeak, 120.);
    nh.param<double>("quality_weak_serious", hdrConfQualityWeakSerious, 80.);
    nh.param<int>("simple_hdr_max_integration", hdrConfSimpleHdrMaxIntegration, 2000);
    nh.param<int>("simple_hdr_min_integration", hdrConfSimpleHdrMinIntegration, 200);
    nh.param<int>("super_hdr_frame_number", hdrConfSuperHdrFrameNumber, 3);
    nh.param<int>("super_hdr_max_integration", hdrConfSuperHdrMaxIntegration, 20000);
    nh.param<int>("hdr_disable_integration_time", hdrConfHdrDisableIntegrationTime, 2000);

    hdrConf.hdr_mode = static_cast<HDRModeTypeDef>(hdrConfHdrMode);
    hdrConf.qualtity_overexposed = hdrConfQualityOverexposed;
    hdrConf.qualtity_overexposed_serious = hdrConfQualityOverexposedSerious;
    hdrConf.qualtity_weak = hdrConfQualityWeak;
    hdrConf.qualtity_weak_serious = hdrConfQualityWeakSerious;
    hdrConf.simple_hdr_max_integration = hdrConfSimpleHdrMaxIntegration;
    hdrConf.simple_hdr_min_integration = hdrConfSimpleHdrMinIntegration;
    hdrConf.super_hdr_frame_number = hdrConfSuperHdrFrameNumber;
    hdrConf.super_hdr_max_integration = hdrConfSuperHdrMaxIntegration;
    hdrConf.hdr_disable_integration_time = hdrConfHdrDisableIntegrationTime;

    HPS3D_SetHDRConfig(&DeviceHandler::handler, hdrConf);
}

void DeviceHandler::setKalManFilter(const ros::NodeHandle& nh) {
    DistanceFilterConfTypeDef distanceFilterConf;

    int distanceFilterConfFilterType;
    double distanceFilterConfKalmanK;
    int distanceFilterConfKalmanThreshold;
    int distanceFilterConfNumCheck;

    nh.param<int>("filter_type", distanceFilterConfFilterType, 0);
    nh.param<double>("kalman_K", distanceFilterConfKalmanK, 0.3);
    nh.param<int>("kalman_threshold", distanceFilterConfKalmanThreshold, 200);
    nh.param<int>("num_check", distanceFilterConfNumCheck, 2);

    distanceFilterConf.filter_type = static_cast<DistanceFilterTypeDef>(distanceFilterConfFilterType);
    distanceFilterConf.kalman_K = distanceFilterConfKalmanK;
    distanceFilterConf.kalman_threshold = distanceFilterConfKalmanThreshold;
    distanceFilterConf.num_check = distanceFilterConfNumCheck;

    HPS3D_SetSimpleKalman(&DeviceHandler::handler, distanceFilterConf);
}

void DeviceHandler::setSmoothFilter(const ros::NodeHandle& nh) {
    SmoothFilterConfTypeDef smoothFilterConf;

    int smoothFilterConfType;
    int smoothFilterConfArg1;
    nh.param<int>("smooth_filter_type", smoothFilterConfType, 0);
    nh.param<int>("smooth_filter_arg1", smoothFilterConfArg1, 0);
    smoothFilterConf.type = static_cast<SmoothFilterTypeDef>(smoothFilterConfType);
    smoothFilterConf.arg1 = smoothFilterConfArg1;

    HPS3D_SetSmoothFilter(&DeviceHandler::handler, smoothFilterConf);
}

void DeviceHandler::setMountingAngle(const ros::NodeHandle& nh) {
    MountingAngleParamTypeDef mountingAngleParamConf;

    bool mountingAngleParamConfEnable;
    int mountingAngleParamConfAngleVertical;
    int mountingAngleParamConfHeight;
    nh.param<bool>("mounting_angle_enable", mountingAngleParamConfEnable, false);
    nh.param<int>("mounting_angle_anglevertical", mountingAngleParamConfAngleVertical, 0);
    nh.param<int>("mounting_angle_height", mountingAngleParamConfHeight, 0);

    mountingAngleParamConf.enable = mountingAngleParamConfEnable;
    mountingAngleParamConf.angle_vertical = mountingAngleParamConfAngleVertical;
    mountingAngleParamConf.height = mountingAngleParamConfHeight;
    HPS3D_SetMountingAngleParamConf(&DeviceHandler::handler, mountingAngleParamConf);
}

void DeviceHandler::setEdgeDetection(const ros::NodeHandle& nh) {
    bool edgeDetectionEnable;
    int edgeDetectionThresholdValue;
    nh.param<bool>("edge_detection_enable", edgeDetectionEnable, false);
    nh.param<int>("edge_detection_threshold_value", edgeDetectionThresholdValue, 0);

    HPS3D_SetEdgeDetectionEnable(edgeDetectionEnable);
    HPS3D_SetEdgeDetectionValue(edgeDetectionThresholdValue);
}

void DeviceHandler::handleSignal(const int sig) {
    if (HPS3D_RemoveDevice(&DeviceHandler::handler) != RET_OK) {
        ROS_ERROR("HPS3D_RemoveDevice failed\n");
    } else {
        ROS_INFO("HPS3D_RemoveDevice succeed\n");
    }
    exit(0);
}

void* DeviceHandler::callbackHandler(HPS3D_HandleTypeDef* handle, AsyncIObserver_t* event) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 outputPointCloud;
    outputPointCloud.header.stamp = ros::Time::now();

    if (event->AsyncEvent != ISubject_Event_DataRecvd) {
        return 0;
    }

    switch (event->RetPacketType) {
        case SIMPLE_ROI_PACKET:
        case SIMPLE_DEPTH_PACKET:
        case OBSTACLE_PACKET:
        case NULL_PACKET:
        case FULL_ROI_PACKET: {
            break;
        }
        case FULL_DEPTH_PACKET: {
            if (ros::ok()) {
                cloud.width = event->MeasureData.point_cloud_data->width;
                cloud.height = event->MeasureData.point_cloud_data->height;
                cloud.points.resize(cloud.width * cloud.height);
                for (size_t i = 0; i < cloud.points.size(); ++i) {
                    const auto& pointData = event->MeasureData.point_cloud_data[0].point_data[i];
                    if (pointData.z < LOW_AMPLITUDE) {
                        cloud.points[i].x = pointData.x / 1000.0;
                        cloud.points[i].y = pointData.y / 1000.0;
                        cloud.points[i].z = pointData.z / 1000.0;
                    } else {
                        cloud.points[i].x = 0;
                        cloud.points[i].y = 0;
                        cloud.points[i].z = 0;
                    }
                }
                pcl::toROSMsg(cloud, outputPointCloud);
                outputPointCloud.header.frame_id = "hps_camera";
                DeviceHandler::pointcloudPub.publish(outputPointCloud);
            }
            break;
        }

        default: {
            ROS_ERROR("system error!\n");
            break;
        }
    }

    return 0;
}

const bool DeviceHandler::isInitialized() const {
    return this->_isInitialized;
}

}  // namespace hps_camera
