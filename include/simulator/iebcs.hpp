/*
 * simu.hpp
 *
 *  Created on: 12 Feb 2021
 *      Author: joubertd
 *
 * Modified from
 * https://github.com/neuromorphicsystems/IEBCS
 */
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

#include <simulator/simulator.hpp>

using vdouble = std::vector<double_t>;
using vtime = std::vector<uint64_t>;
class GAZEBO_VISIBLE IEBCSim : public EventSimulator
{
  // public:
  //   // the minimum time interval between two events for events generation
  //   static constexpr float DYNAMIC_RANGE = 50;

private:
  // Shape
  // int width_;
  // int height_;
  // States
  uint64_t time_;
  vdouble last_v_;
  vdouble cur_v_;
  vtime time_px_;
  vtime cur_ref_;
  // Thresholds
  // double_t th_noise_;
  vdouble cur_th_pos_;
  vdouble cur_th_neg_;
  std::default_random_engine generator_cur_th_pos_;
  std::normal_distribution<double_t> distribution_cur_th_pos_;
  std::default_random_engine generator_cur_th_neg_;
  std::normal_distribution<double_t> distribution_cur_th_neg_;
  // Latency
  double_t tau_;
  double_t m_jit_;
  double_t m_lat_;
  double_t ref_;
  vdouble tau_p_;
  std::default_random_engine generator_lat_;
  std::normal_distribution<double_t> distribution_lat_;
  // Noise
  vtime bgn_neg_next_;
  vtime bgn_pos_next_;
  vdouble bgn_hist_pos_;
  vdouble bgn_hist_neg_;
  std::default_random_engine generator_noise_;
  std::uniform_real_distribution<double_t> distribution_noise_;
  // Utils
  vtime PER;

public:
  IEBCSim();
  IEBCSim(int width, int height, sdf::ElementPtr _sdf);
  virtual ~IEBCSim();

  virtual void simulateMain(const cv::Mat *last_iamge, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, const ros::Time &current_time, const ros::Time &last_time);

private:
  void setThreshold(double_t th_pos, double_t th_neg, double_t th_noise);
  void setLatency(double_t lat, double_t jit, double_t ref, double_t tau);
  void setShape(int width, int height);
  void initNextNoise(int &&x, int &&y, int &&p);
  void updateNextNoise(int &&x, int &&y, int &&p);
  void initNoise(const double_t *distrib_pos, const double_t *distrib_neg, uint64_t size);
  void disableNoise();
  void initImg(const double_t *img);
  void masterRst();
  void checkNoise(const uint64_t dt, std::vector<dvs_msgs::Event> *ev_pk);
  void clamp(double_t &val);
  void updateImg(const double_t *img, const uint64_t dt, std::vector<dvs_msgs::Event> *ev_pk);
  dvs_msgs::Event Event(uint64_t x, uint64_t y, bool polarity, uint64_t ts);
};