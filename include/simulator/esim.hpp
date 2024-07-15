/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ---LICENSE-END**/
/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * bla: Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* ESIM is proposed by Henri Rebecq et al, in paper "ESIM: an Open Event Camera Simulator"*/

#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include <random>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <gazebo/common/Plugin.hh>

#include <simulator/simulator.hpp>
class GAZEBO_VISIBLE Esim : public EventSimulator
{
  // public:
  //   // the minimum time interval between two events for events generation
  //   static constexpr double_t DYNAMIC_RANGE = 50;

private:
  ros::Time last_time_;
  cv::Mat mem_last_image_;
  // double_t th_pos_, th_neg_, th_noise_;
  std::default_random_engine generator_cur_th_;
  std::normal_distribution<double_t> distribution_cur_th_;

public:
  Esim();
  Esim(int width, int height, sdf::ElementPtr _sdf);
  virtual ~Esim();

  virtual void simulateMain(const cv::Mat *last_iamge, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, const ros::Time &current_time, const ros::Time &last_time);

private:
  double_t adaptiveSample(const cv::Mat *last_image, const cv::Mat *curr_image);
  void processDelta(cv::Mat *last_image, const cv::Mat *curr_image, ros::Time &ts, std::vector<dvs_msgs::Event> *events);
  void fillEvents(const cv::Mat *mask, int polarity, ros::Time &ts, std::vector<dvs_msgs::Event> *events);
};
