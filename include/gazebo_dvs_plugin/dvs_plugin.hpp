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
 *
 */

#pragma once
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <rosgraph_msgs/Clock.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <sensor_msgs/CameraInfo.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <simulator/simulator.hpp>
#include <simulator/esim.hpp>
#include <simulator/iebcs.hpp>
#include <simulator/plain.hpp>

namespace gazebo
{
  class GAZEBO_VISIBLE DvsPlugin : public SensorPlugin
  {
  public:
    DvsPlugin();
    ~DvsPlugin();
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  private:
    virtual void mainCallback(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              const std::string &_format);
    // void ClockCallback(const rosgraph_msgs::Clock::ConstPtr &msg);
    void publishEvents(std::vector<dvs_msgs::Event> *events);
    void publishCameraInfo(const std::string &camera_info);

  private:
    unsigned int width_, height_;
    std::string format_;

    sensors::CameraSensorPtr parentCameraSensor;
    rendering::CameraPtr camera_;
    // interpolate start time and end time between frames
    ros::Time last_time_, current_time_, clk_;

    event::ConnectionPtr newFrameConnection;

    ros::NodeHandle node_handle_;
    ros::Publisher event_pub_, info_pub_;
    // ros::Subscriber clock_sub_;
    std::string namespace_;

  private:
    cv::Mat last_image_;
    bool has_last_image_;
    EventSimulator *simu_;
  };
}
