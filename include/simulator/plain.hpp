#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <simulator/simulator.hpp>
class GAZEBO_VISIBLE Plain : public EventSimulator
{
public:
  Plain();
  Plain(int width, int height, sdf::ElementPtr _sdf);
  virtual ~Plain();

  virtual void simulateMain(const cv::Mat *last_iamge, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, const ros::Time &current_time, const ros::Time &last_time);

private:
  void fillEvents(const cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events);
};