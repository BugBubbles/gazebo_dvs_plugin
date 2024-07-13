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
#include <simulator/esim.hpp>
////////////////////////////////////////////////////////////////////////////////
// Constructor
Esim::Esim()
{
  last_time_ = ros::Time::now();
  th_pos_ = 0.3 * DYNAMIC_RANGE, th_neg_ = 0.6 * DYNAMIC_RANGE, th_noise_ = 0.035 * DYNAMIC_RANGE;
  mem_last_image_ = cv::Mat::ones(0, 0, CV_64F);
}

Esim::Esim(int width, int height, sdf::ElementPtr _sdf)
{
  last_time_ = ros::Time::now();
  mem_last_image_ = cv::Mat::ones(width, height, CV_64F);
  th_pos_ = 0.3 * DYNAMIC_RANGE, th_neg_ = 0.6 * DYNAMIC_RANGE, th_noise_ = 0.035 * DYNAMIC_RANGE;
  if (_sdf->HasElement("posThreshold"))
  {
    th_pos_ = _sdf->GetElement("posThreshold")->Get<double_t>() * DYNAMIC_RANGE;
  }
  else
  {
    ROS_WARN("[DVS_plugin] No positive threshold was found, default value is 0.3.\n");
  }
  if (_sdf->HasElement("negThreshold"))
  {
    th_neg_ = _sdf->GetElement("negThreshold")->Get<double_t>() * DYNAMIC_RANGE;
  }
  else
  {
    ROS_WARN("[DVS_plugin] No negative threshold was found, default value is 0.6.\n");
  }
  if (_sdf->HasElement("noiseThreshold"))
  {
    th_noise_ = _sdf->GetElement("noiseThreshold")->Get<double_t>() * DYNAMIC_RANGE;
  }
  else
  {
    ROS_WARN("[DVS_plugin] No noise threshold was found, default value is 0.035.\n");
  }
  distribution_cur_th_ = std::normal_distribution<double_t>(0, th_noise_);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Esim::~Esim()
{
}

// last_image : the last time after event was created 32UC1
// curr_image : the current image 32UC1
void Esim::simulateMain(const cv::Mat *last_image, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, const ros::Time &current_time, const ros::Time &last_time)
{
  mem_last_image_ = last_image->clone();
  double_t f_time_interval = static_cast<double_t>(current_time.toSec() - last_time.toSec());

  // calculate the timestamp between two consecutive frames
  double_t sample_step = 0.05 * this->adaptiveSample(last_image, curr_image);
  // ROS_WARN("sample: %f", sample_step);
  sample_step = std::min(sample_step, (double_t)5.0);
  // the sample interval between two events for events generation is 1 us.
  cv::Mat slope = (*curr_image - *last_image) / sample_step;

  // convert to log intensity to store the last time after event was created
  for (int iter_num = 0; iter_num < static_cast<int>(sample_step); iter_num++)
  {
    // convert to log intensity to store the current image during the interpolation
    ros::Time ts = last_time_ + ros::Duration(static_cast<double_t>(iter_num) / sample_step * f_time_interval);
    mem_last_image_ = mem_last_image_ + slope;
    // *last_image = *curr_image;
    this->processDelta(&mem_last_image_, last_image, ts, events);
  }
}

// last_image : the last time after event was created
// curr_image : the current image
// this function will change the last_image in accordance with events activation after the event was created
void Esim::processDelta(cv::Mat *last_image, const cv::Mat *curr_image, ros::Time &ts, std::vector<dvs_msgs::Event> *events)
{
  cv::Mat pos_mask = *curr_image - *last_image;
  cv::Mat neg_mask = *last_image - *curr_image;

  double_t th_noise_;
  cv::threshold(pos_mask, pos_mask, th_pos_, 255, cv::THRESH_BINARY);
  cv::threshold(neg_mask, neg_mask, th_neg_, 255, cv::THRESH_BINARY);
  last_image->forEach<double_t>([&](double_t &pixel, const int *position) -> void
                                {
                                 if (pos_mask.at<double_t>(position[0], position[1]) > 0)
                                 {
                                   th_noise_ = std::max(static_cast<double_t>(0.0), distribution_cur_th_(generator_cur_th_));
                                   pixel += static_cast<double_t>(th_pos_+th_noise_);
                                 }
                                 else if (neg_mask.at<double_t>(position[0], position[1]) > 0)
                                 {
                                   th_noise_ = std::max(static_cast<double_t>(0.0), distribution_cur_th_(generator_cur_th_));
                                   pixel -= static_cast<double_t>(th_neg_+th_noise_);
                                 } });

  pos_mask.convertTo(pos_mask, CV_32S);
  neg_mask.convertTo(neg_mask, CV_32S);

  this->fillEvents(&pos_mask, 1, ts, events);
  this->fillEvents(&neg_mask, 0, ts, events);
}

double_t Esim::adaptiveSample(const cv::Mat *last_image, const cv::Mat *curr_image)
{
  double_t max_val = -10000.0;
  // calculate the light change between the two frames
  last_image->forEach<double_t>([&](const double_t &p, const int *position) -> void
                                {
    double_t l1 = last_image->at<double_t>(position[0], position[1]);                       
    double_t l2 = curr_image->at<double_t>(position[0], position[1]);
    double_t temp = std::fabs(l1 - l2);
    if (temp > max_val)
    {
      max_val = temp;
    } });
  return max_val;
}

// void Esim::fillEvents(cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events)
void Esim::fillEvents(const cv::Mat *mask, int polarity, ros::Time &ts, std::vector<dvs_msgs::Event> *events)
{
  // findNonZero fails when there are no zeros
  // TODO is there a better workaround then iterating the binary image twice?
  if (cv::countNonZero(*mask) != 0)
  {
    std::vector<cv::Point> locs;
    cv::findNonZero(*mask, locs);

    for (int i = 0; i < locs.size(); i++)
    {
      dvs_msgs::Event event;
      event.x = locs[i].x;
      event.y = locs[i].y;
      event.ts = ts;
      event.polarity = polarity;
      events->push_back(event);
    }
  }
}
