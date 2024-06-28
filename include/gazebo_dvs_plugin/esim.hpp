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
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo/common/Plugin.hh>

class GAZEBO_VISIBLE Esim
{
public:
  // the minimum time interval between two events for events generation
  static constexpr float MIN_TIME_INTERVAL = 1e-4;

private:
  ros::Time last_time;
  cv::Mat mem_last_image;
  int cols, rows;
  float event_threshold;

public:
  Esim();

  Esim(float event_threshold, int width, int height);

  ~Esim();

  void simulateESIM(cv::Mat *last_iamge, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, const geometry_msgs::TwistStamped &imu_msg, const ros::Time &current_time, const ros::Time &last_time);

  void setEventThreshold(const float event_threshold);

private:
  float lightChange(const float last_pixel, const float curr_pixel, const float f_time_interval);

  float adaptiveSample(const cv::Mat *last_image, const cv::Mat *curr_image, const float f_time_interval);

  void processDelta(cv::Mat *last_image, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events);

  void fillEvents(const cv::Mat *mask, const int polarity, std::vector<dvs_msgs::Event> *events);
};
