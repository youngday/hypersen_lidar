// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <assert.h>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
//for c , find file path 
#include <stdio.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include <pcl_conversions/pcl_conversions.h> //pcl point

#include "YamlParam.hpp" //self yaml file config
#include "api.h"         //for hypersen lib



using namespace std;
using namespace std::chrono_literals;

YamlParam myYaml;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_pub_;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher") {

    char *buffer; // for c
    buffer = getcwd(NULL, 0);
    cout << "find file path " << buffer << endl;
    //if use it 
    // const char *model_path = strcat(buffer,"/models");

    RCLCPP_INFO(this->get_logger(), "Loading yaml.");
    YAML::Node login = YAML::LoadFile("login.yaml");
    YAML::Node config = YAML::LoadFile("config.yaml");
    RCLCPP_INFO(this->get_logger(), "Load yaml OK.");
    if (login["account"]) {
      auto account = login["account"];
      myYaml.userName = account["userName"].as<string>();
      myYaml.passWord = account["passWord"].as<string>();
      myYaml.serialNum = account["serialNum"].as<string>();
    }
    if (config["param"]) {
      auto param = config["param"];

      myYaml.transport_type = param["transport_type"].as<int>();
      myYaml.ethernet_server_info = param["ethernet_server_info"].as<string>();

      myYaml.hdr_mode = param["hdr_mode"].as<int>();
      myYaml.quality_over_exposed = param["quality_over_exposed"].as<int>();
      myYaml.quality_over_exposed_serious =
          param["quality_over_exposed_serious"].as<int>();
      myYaml.quality_weak = param["quality_weak"].as<int>();
      myYaml.quality_weak_serious = param["quality_weak_serious"].as<int>();
      myYaml.simple_hdr_max_integration =
          param["simple_hdr_max_integration"].as<int>();
      myYaml.simple_hdr_min_integration =
          param["simple_hdr_min_integration"].as<int>();
      myYaml.super_hdr_frame_number = param["super_hdr_frame_number"].as<int>();
      myYaml.super_hdr_max_integration =
          param["super_hdr_max_integration"].as<int>();
      myYaml.hdr_disable_integration_time =
          param["hdr_disable_integration_time"].as<int>();
      myYaml.super_hdr_max_integration =
          param["super_hdr_max_integration"].as<int>();
      myYaml.filter_type = param["filter_type"].as<int>();
      myYaml.kalman_K = param["kalman_K"].as<float>();
      myYaml.kalman_threshold = param["kalman_threshold"].as<int>();
      myYaml.num_check = param["num_check"].as<int>();
      myYaml.kalman_threshold = param["kalman_threshold"].as<int>();
      myYaml.smooth_filter_type = param["smooth_filter_type"].as<int>();
      myYaml.smooth_filter_arg1 = param["smooth_filter_arg1"].as<int>();
      myYaml.mounting_angle_enable = param["mounting_angle_enable"].as<bool>();
      myYaml.mounting_angle_vertical =
          param["mounting_angle_vertical"].as<int>();
      myYaml.mounting_angle_height = param["mounting_angle_height"].as<int>();
      myYaml.edge_detection_enable = param["edge_detection_enable"].as<bool>();
      myYaml.edge_detection_threshold_value =
          param["edge_detection_threshold_value"].as<int>();

      myYaml.loop_rate = param["loop_rate"].as<int>();

    } else {
      RCLCPP_ERROR(this->get_logger(), "Load param item of config.yaml ERROR.");
      exit(0);
    }
    if (config["debug"]) {

      auto debug = config["debug"];
      myYaml.debug_enable = debug["debug_enable"].as<bool>();

    } else {
      RCLCPP_ERROR(this->get_logger(), "Load debug item of config.yaml ERROR.");
      exit(0);
    }

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    point_cloud2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud2", 10);
    this->initDevice();

    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(this->count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      this->publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(2s, timer_callback);
  }
  ~MinimalPublisher() { HPS3D_RemoveDevice(&handler); };

  // for hps_camera driver
public:
  bool initDevice() {
    if (signal(SIGINT, handleSignal) == SIG_ERR ||
        signal(SIGTSTP, handleSignal) == SIG_ERR) {
      RCLCPP_ERROR(this->get_logger(), "Signal initialization failed\n ");

      return false;
    }

    HPS3D_SetDebugEnable(myYaml.debug_enable);
    HPS3D_SetDebugFunc([](char *str) { cout << "%s" << str << endl; });

    this->_transportType = myYaml.transport_type;
    if (this->_transportType == 1) {
      HPS3D_SetEthernetServerInfo(
          &handler, const_cast<char *>(myYaml.ethernet_server_info.c_str()),
          12345);
    } else {
      char fileName[10][20];
      int devCnt = HPS3D_GetDeviceList(const_cast<char *>("/dev/"),
                                       const_cast<char *>("ttyACM"), fileName);

      RCLCPP_INFO(this->get_logger(), "Usb serial devCnt:%d\n ", devCnt);

      handler.DeviceName = fileName[0];
    }

    RET_StatusTypeDef ret = HPS3D_Connect(&handler);
    if (ret != RET_OK) {
      RCLCPP_INFO(this->get_logger(), "Device open failed,ret = %d\n", ret);

      return false;
    }

    ret = HPS3D_ConfigInit(&handler);
    if (RET_OK != ret) {
      RCLCPP_INFO(this->get_logger(), "Initialization failed:%d\n", ret);
      return false;
    }

    // Set convert point cloud data enable
    HPS3D_SetOpticalEnable(&handler, true);
    HPS3D_SetPointCloudEn(true);

    this->setHDRConf();
    this->setKalManFilter();
    this->setSmoothFilter();
    this->setMountingAngle();
    this->setEdgeDetection();

    // Add observer one
    eventObserver.AsyncEvent = ISubject_Event_DataRecvd;
    eventObserver.NotifyEnable = true;
    HPS3D_AddObserver(&callbackHandler, &handler, &eventObserver);

    handler.RunMode = RUN_CONTINUOUS;
    HPS3D_SetRunMode(&handler);

    RCLCPP_INFO(this->get_logger(), "Initialization succeed\n");

    return true;
  }

private:
  void setHDRConf(void) {
    HDRConf hdrConf;

    hdrConf.hdr_mode = static_cast<HDRModeTypeDef>(myYaml.hdr_mode);
    hdrConf.qualtity_overexposed = myYaml.quality_over_exposed;
    hdrConf.qualtity_overexposed_serious = myYaml.quality_over_exposed_serious;
    hdrConf.qualtity_weak = myYaml.quality_weak;
    hdrConf.qualtity_weak_serious = myYaml.quality_weak_serious;
    hdrConf.simple_hdr_max_integration = myYaml.simple_hdr_max_integration;
    hdrConf.simple_hdr_min_integration = myYaml.simple_hdr_min_integration;
    hdrConf.super_hdr_frame_number = myYaml.super_hdr_frame_number;
    hdrConf.super_hdr_max_integration = myYaml.super_hdr_max_integration;
    hdrConf.hdr_disable_integration_time = myYaml.hdr_disable_integration_time;

    HPS3D_SetHDRConfig(&handler, hdrConf);
  }

  void setKalManFilter(void) {
    DistanceFilterConfTypeDef distanceFilterConf;

    distanceFilterConf.filter_type =
        static_cast<DistanceFilterTypeDef>(myYaml.filter_type);
    distanceFilterConf.kalman_K = myYaml.kalman_K;
    distanceFilterConf.kalman_threshold = myYaml.kalman_threshold;
    distanceFilterConf.num_check = myYaml.num_check;

    HPS3D_SetSimpleKalman(&handler, distanceFilterConf);
  }

  void setSmoothFilter(void) {
    SmoothFilterConfTypeDef smoothFilterConf;

    smoothFilterConf.type =
        static_cast<SmoothFilterTypeDef>(myYaml.smooth_filter_type);
    smoothFilterConf.arg1 = myYaml.smooth_filter_arg1;

    HPS3D_SetSmoothFilter(&handler, smoothFilterConf);
  }

  void setMountingAngle(void) {
    MountingAngleParamTypeDef mountingAngleParamConf;

    mountingAngleParamConf.enable = myYaml.mounting_angle_enable;
    mountingAngleParamConf.angle_vertical = myYaml.mounting_angle_vertical;
    mountingAngleParamConf.height = myYaml.mounting_angle_height;
    HPS3D_SetMountingAngleParamConf(&handler, mountingAngleParamConf);
  }

  void setEdgeDetection(void) {

    HPS3D_SetEdgeDetectionEnable(myYaml.edge_detection_enable);
    HPS3D_SetEdgeDetectionValue(myYaml.edge_detection_threshold_value);
  }

  void setGPIOOut(bool isON) {

    GPIOOutConfTypeDef gpio_out_config;
    gpio_out_config.gpio = GPOUT_1;
    gpio_out_config.polarity = GPIO_POLARITY_LOW;

    handler.RunMode = RUN_IDLE;
    HPS3D_SetRunMode(&handler);

    if (isON) {

      gpio_out_config.polarity = GPIO_POLARITY_HIGH;
      HPS3D_SetGPIOOut(&handler, gpio_out_config);
    } else {
      gpio_out_config.polarity = GPIO_POLARITY_LOW;
      HPS3D_SetGPIOOut(&handler, gpio_out_config);
    }
    handler.RunMode = RUN_CONTINUOUS;
    HPS3D_SetRunMode(&handler);
  }

  static void handleSignal(const int sig) {
    if (HPS3D_RemoveDevice(&handler) != RET_OK) {
      cout << "HPS3D_RemoveDevice failed.sig:" << sig << endl;
    } else {
      cout << "HPS3D_RemoveDevice succeed.sig:" << sig << endl;
    }
    exit(0);
  }

  static void *callbackHandler(HPS3D_HandleTypeDef *handle,
                               AsyncIObserver_t *event) {
    HPS3D_HandleTypeDef *not_used_handle;
    not_used_handle = handle;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    auto outputPointCloud = sensor_msgs::msg::PointCloud2();

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

      if (rclcpp::ok()) {

        cloud.width = event->MeasureData.point_cloud_data->width;
        cloud.height = event->MeasureData.point_cloud_data->height;
        cloud.points.resize(cloud.width * cloud.height);
        for (size_t i = 0; i < cloud.points.size(); ++i) {
          const auto &pointData =
              event->MeasureData.point_cloud_data[0].point_data[i];
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
        outputPointCloud.header.stamp = rclcpp::Clock().now();
        point_cloud2_pub_->publish(outputPointCloud);
      };

      break;
    }

    default: {
      cout << "system error!" << endl;
      break;
    }
    }

    return 0;
  }

private: // for hypersen
  static HPS3D_HandleTypeDef handler;
  static AsyncIObserver_t eventObserver;
  size_t _transportType{};

private: // for self
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_{};
};

// static member initialize
HPS3D_HandleTypeDef MinimalPublisher::handler{};
AsyncIObserver_t MinimalPublisher::eventObserver{};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
