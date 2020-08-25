/**
 * @file    YamlParam.hpp
 *
 * @date    2020-08-18
 *
 * Copyright (c) organization
 *
 */

#ifndef YAMLPARAM_HPP_
#define YAMLPARAM_HPP_

#include <yaml-cpp/yaml.h>

using namespace std;

class YamlParam {
public:
  string userName;
  string passWord;
  string serialNum;

  int transport_type;
  string ethernet_server_info;
  bool debug_enable;
  // hdr settings
  int hdr_mode;
  int quality_over_exposed;
  int quality_over_exposed_serious;
  int quality_weak;
  int quality_weak_serious;
  int simple_hdr_max_integration;
  int simple_hdr_min_integration;
  int super_hdr_frame_number;
  int super_hdr_max_integration;
  int hdr_disable_integration_time;

  int loop_rate;

  // kalman filter
  int filter_type;
  float kalman_K;
  int kalman_threshold;
  int num_check;

  // smooth filter 0=disable ,1=aver,2=gauss
  int smooth_filter_type;
  int smooth_filter_arg1;

  // mounting angle
  bool mounting_angle_enable;
  int mounting_angle_vertical;
  int mounting_angle_height;

  // edge detection enable
  bool edge_detection_enable;
  int edge_detection_threshold_value;
};

#endif /* YAMLPARAM_HPP_ */
