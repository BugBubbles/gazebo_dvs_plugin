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
#include <gazebo_dvs_plugin/esim.hpp>
////////////////////////////////////////////////////////////////////////////////
// Constructor
Esim::Esim()
{
  last_time = ros::Time::now();
  event_threshold = 10.0;
  mem_last_image = cv::Mat::ones(0, 0, CV_32F);
}

Esim::Esim(float event_threshold, int width, int height)
{
  this->event_threshold = event_threshold;
  last_time = ros::Time::now();
  mem_last_image = cv::Mat::ones(width, height, CV_32F);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Esim::~Esim()
{
}

void Esim::setEventThreshold(const float event_threshold)
{
  this->event_threshold = event_threshold;
}

// last_image : the last time after event was created 32UC1
// curr_image : the current image 32UC1
void Esim::simulateESIM(cv::Mat *last_iamge, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, const geometry_msgs::TwistStamped &imu_msg, const ros::Time &current_time, const ros::Time &last_time)
{
  cv::Mat last_image_ = this->mem_last_image.clone();
  cv::Mat curr_image_ = curr_image->clone();

  float f_time_interval = current_time.toSec() - last_time.toSec();

  curr_image_.convertTo(curr_image_, CV_32F);

  curr_image_ += 1e-4;
  last_image_ += 1e-4;

  // calculate the timestamp between two consecutive frames
  float max_t_l = this->adaptiveSample(&last_image_, &curr_image_, f_time_interval);
  // the sample interval between two events for events generation is 1 us.
  float t_sample_interval = 0.5 * max_t_l;
  // caculate the slope matrix between the two frames
  cv::Mat slope = (curr_image_ - this->mem_last_image) / f_time_interval;

  // convert to log intensity to store the last time after event was created
  if (t_sample_interval >= f_time_interval)
  {
    this->processDelta(&this->mem_last_image, &curr_image_, events);
  }
  else
  {
    for (int iter_num = 0; iter_num < static_cast<int>(f_time_interval / t_sample_interval); iter_num++)
    {
      // convert to log intensity to store the current image during the interpolation
      last_image_ += slope * t_sample_interval;
      this->processDelta(&this->mem_last_image, &last_image_, events);
    }
  }
}

// last_image : the last time after event was created
// curr_image : the current image
// this function will change the last_image in accordance with events activation after the event was created
void Esim::processDelta(cv::Mat *last_image, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events)
{
  if (curr_image->size() == last_image->size())
  {
    cv::Mat pos_diff = *curr_image - *last_image;
    cv::Mat neg_diff = *last_image - *curr_image;

    cv::Mat pos_mask;
    cv::Mat neg_mask;

    cv::threshold(pos_diff, pos_mask, this->event_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(neg_diff, neg_mask, this->event_threshold, 255, cv::THRESH_BINARY);

    last_image->forEach<float>([&](float &pixel, const int *position) -> void
                               {
                                 if (pos_mask.at<float>(position[0], position[1]) >0)
                                 {
                                   pixel = curr_image->at<float>(position[0], position[1]);
                                 }
                                 if (neg_mask.at<float>(position[0], position[1]) >0)
                                 {
                                   pixel = curr_image->at<float>(position[0], position[1]);
                                 } });

    pos_mask.convertTo(pos_mask, CV_32S);
    neg_mask.convertTo(neg_mask, CV_32S);

    this->fillEvents(&pos_mask, 0, events);
    this->fillEvents(&neg_mask, 1, events);
  }
  else
  {
    gzwarn << "Unexpected change in image size (" << last_image->size() << " -> " << curr_image->size() << "). Publishing no events for this frame change." << std::endl;
  }
}

float Esim::lightChange(const float last_pixel, const float curr_pixel, const float f_time_interval)
{
  // l1 and l2 are the logirithmic light intensities of the two frames
  return std::fabs((last_pixel - curr_pixel) / f_time_interval);
}

float Esim::adaptiveSample(const cv::Mat *last_image, const cv::Mat *curr_image, const float f_time_interval)
{
  cv::Mat last_image_(last_image->size(), CV_32F);
  // calculate the velocity and angular velocity for the camera ego movement
  // calculate the light change between the two frames
  last_image_.forEach<float>([&](float &p, const int *position) -> void
                             {
       float l1 = last_image->at<float>(position[0], position[1]);                       
    float l2 = curr_image->at<float>(position[0], position[1]);
    p = this->lightChange(l1, l2, f_time_interval); });
  double temp_max_t_l, min_;
  cv::Point min_loc, max_loc;
  // get the minimum value of the two
  cv::minMaxLoc(last_image_, &min_, &temp_max_t_l, &min_loc, &max_loc);

  // max_t_b is the change of the light intensity between the two frames
  return static_cast<float>(0.1 / temp_max_t_l);
}

// void Esim::fillEvents(cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events)
void Esim::fillEvents(const cv::Mat *mask, const int polarity, std::vector<dvs_msgs::Event> *events)
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
      event.ts = ros::Time::now();
      event.polarity = polarity;
      events->push_back(event);
    }
  }
}
