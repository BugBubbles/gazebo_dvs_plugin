#pragma once
#include <vector>
#include <random>
#include <algorithm>
#include <memory>
#include <limits>
#include <cstring>

#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <gazebo/common/Plugin.hh>

class GAZEBO_VISIBLE EventSimulator
{
public:
  // the minimum time interval between two events for events generation
  static constexpr double_t DYNAMIC_RANGE = 50.0;

protected:
  // Shape
  int width_;
  int height_;
  // Thresholds
  double_t th_pos_, th_neg_, th_noise_;

public:
  EventSimulator() {};
  EventSimulator(int width, int heigh, sdf::ElementPtr _sdf)
  {
    ROS_ERROR("[DVS_plugin] Not Implemented!");
    return;
  };
  virtual ~EventSimulator() {};
  virtual void simulateMain(const cv::Mat *last_iamge, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, const ros::Time &current_time, const ros::Time &last_time)
  {
    ROS_ERROR("[DVS_plugin] Not Implemented!");
    return;
  };
};