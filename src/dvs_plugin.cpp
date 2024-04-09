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
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * IMPLEMENTATION INSPIRED BY
 * https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_opticalFlow_plugin.cpp
 */

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <tf/tf.h>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <gazebo_dvs_plugin/dvs_plugin.hpp>
#include <gazebo_dvs_plugin/esim.hpp>
using namespace std;
using namespace cv;

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(DvsPlugin)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  DvsPlugin::DvsPlugin()
      : SensorPlugin(), width(0), height(0), depth(0), has_last_image(false)
  {
    // initialize the message tunnel.
    this->imu_msgs_ = {};

    velocity_x = velocity_y = velocity_z = 0.0;
    angular_velocity_x = angular_velocity_y = angular_velocity_z = 0;

    // store the t1 and t2 for two immediate frames.
    this->current_time_ = ros::Time::now();
    this->last_time_ = ros::Time::now();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  DvsPlugin::~DvsPlugin()
  {
    this->parentSensor.reset();
    this->camera.reset();
  }

  // load funtion provide the api for `roslaunch` to execute. It just need a subscriber to accquire the exists sensors' data.
  void DvsPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    if (!_sensor)
      gzerr << "Invalid sensor pointer." << endl;

#if GAZEBO_MAJOR_VERSION >= 7
    this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
    this->camera = this->parentSensor->Camera();
#else
    this->parentSensor = boost::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
    this->camera = this->parentSensor->GetCamera();
#endif

    if (!this->parentSensor)
    {
      gzerr << "DvsPlugin not attached to a camera sensor." << endl;
      return;
    }

#if GAZEBO_MAJOR_VERSION >= 7
    this->width = this->camera->ImageWidth();
    this->height = this->camera->ImageHeight();
    this->depth = this->camera->ImageDepth();
    this->format = this->camera->ImageFormat();
#else
    this->width = this->camera->GetImageWidth();
    this->height = this->camera->GetImageHeight();
    this->depth = this->camera->GetImageDepth();
    this->format = this->camera->GetImageFormat();
#endif

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzwarn << "[gazebo_ros_dvs_camera] Please specify a robotNamespace." << endl;

    string sensorName = "";
    if (_sdf->HasElement("sensorName"))
      sensorName = _sdf->GetElement("sensorName")->Get<std::string>() + "/";
    else
      gzwarn << "[gazebo_ros_dvs_camera] Please specify a sensorName." << endl;

    string eventName = "events";
    if (_sdf->HasElement("eventsTopicName"))
      eventName = _sdf->GetElement("eventsTopicName")->Get<std::string>();

    const string eventTopic = sensorName + eventName;

    string imuName = "imu";
    if (_sdf->HasElement("imuTopicName"))
      imuName = _sdf->GetElement("imuTopicName")->Get<std::string>();

    const string imuTopic = sensorName + imuName;

    if (_sdf->HasElement("eventThreshold"))
      this->event_threshold = _sdf->GetElement("eventThreshold")->Get<float>();
    else
      gzwarn << "[gazebo_ros_dvs_camera] Please specify a DVS threshold." << endl;

    event_pub_ = node_handle_.advertise<dvs_msgs::EventArray>(eventTopic, 10, true);

    this->newFrameConnection = this->camera->ConnectNewImageFrame(
        boost::bind(&DvsPlugin::mainCallback, this, _1, this->width, this->height, this->depth, this->format));

    this->parentSensor->SetActive(true);

    this->imu_sub_ = this->node_handle_.subscribe("/iris/imu", 1000, &DvsPlugin::imuCallback, this);

    // Initialize the publisher that publishes the IMU data
    this->imu_pub_ = this->node_handle_.advertise<sensor_msgs::Imu>(imuTopic, 1000);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  // actually this funtion is the main part of dvs_plugin
  void DvsPlugin::mainCallback(const unsigned char *_image,
                               unsigned int _width, unsigned int _height, unsigned int _depth,
                               const std::string &_format)
  {
#if GAZEBO_MAJOR_VERSION >= 7
    _image = this->camera->ImageData(0);
#else
    _image = this->camera->GetImageData(0);
#endif
    // add DepthCameraSensor to get the depth image
    this->parentDepthCamera->Update(true);
    this->curr_dep_img_ = this->parentDepthCamera->DepthData();
    this->current_time_ = ros::Time::now();

    /*
#if GAZEBO_MAJOR_VERSION >= 7
float rate = this->camera->RenderRate();
#else
float rate = this->camera->GetRenderRate();
#endif
if (!isfinite(rate))
rate =  30.0;
float dt = 1.0 / rate;
     */

    // convert given frame to opencv image
    cv::Mat input_image(_height, _width, CV_8UC3);
    input_image.data = (uchar *)_image;

    // color to grayscale
    cv::Mat curr_image_rgb(_height, _width, CV_8UC3);
    cvtColor(input_image, curr_image_rgb, CV_RGB2BGR);
    cvtColor(curr_image_rgb, input_image, CV_BGR2GRAY);

    input_image.convertTo(input_image, CV_32F);

    // convert to log intensity
    input_image.forEach<float>([](float &p, const int *position)
                                { p = std::log(1e-4 + static_cast<float>(p)); });

    cv::Mat curr_image = input_image;

    /* TODO any encoding configuration should be supported
        try {
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*_image, sensor_msgs::image_encodings::BGR8);
          std::cout << "Image: " << std::endl << " " << cv_ptr->image << std::endl << std::endl;
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception %s", e.what());
          std::cout << "ERROR";
        }
    */

    assert(_height == height && _width == width);
    if (this->has_last_image)
    {
      std::vector<dvs_msgs::Event> events;
      // this->processDelta(&this->last_image, &curr_image, &this->imu_msgs_, &events);

      Esim::simulateESIM(&this->last_image, &curr_image, this->imu_msgs_, &events,
                        this->curr_dep_img_, this->current_time_, this->last_time_);

      this->publishEvents(&events);
      this->imu_msgs_.clear();
    }
    else if (curr_image.size().area() > 0)
    {
      this->last_image = curr_image;

      // this->last_dep_img_ = this->curr_dep_img_;
      this->has_last_image = true;
      // clear all the items in tunnel for a next step storing.
      this->imu_msgs_.clear();
      this->last_time_ = this->current_time_;
    }
    else
    {
      gzwarn << "Ignoring empty image." << endl;
    }
  }

  void DvsPlugin::processDelta(cv::Mat *last_image, cv::Mat *curr_image, std::vector<sensor_msgs::Imu> *imu_msgs, std::vector<dvs_msgs::Event> *events)
  {
    if (curr_image->size() == last_image->size())
    {
      cv::Mat pos_diff = *curr_image - *last_image;
      cv::Mat neg_diff = *last_image - *curr_image;

      cv::Mat pos_mask;
      cv::Mat neg_mask;

      cv::threshold(pos_diff, pos_mask, event_threshold, 255, cv::THRESH_BINARY);
      cv::threshold(neg_diff, neg_mask, event_threshold, 255, cv::THRESH_BINARY);

      *last_image += pos_mask & pos_diff;
      *last_image -= neg_mask & neg_diff;

      std::vector<dvs_msgs::Event> events;

      this->fillEvents(&pos_mask, 0, &events);
      this->fillEvents(&neg_mask, 1, &events);

      this->publishEvents(&events);
    }
    else
    {
      gzwarn << "Unexpected change in image size (" << last_image->size() << " -> " << curr_image->size() << "). Publishing no events for this frame change." << endl;
    }
  }

  void DvsPlugin::fillEvents(cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events)
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
    Esim::fillEvents(mask, polarity, events);
  }

  void DvsPlugin::publishEvents(std::vector<dvs_msgs::Event> *events)
  {
    if (events->size() > 0)
    {
      dvs_msgs::EventArray msg;
      msg.events.clear();
      msg.events.insert(msg.events.end(), events->begin(), events->end());
      msg.width = width;
      msg.height = height;

      // TODO what frame_id is adequate?
      msg.header.frame_id = namespace_;
      msg.header.stamp = ros::Time::now();

      event_pub_.publish(msg);
    }
  }

  // Callback function for the IMU subscriber
  void DvsPlugin::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
  {
    // Store the latest IMU data
    // this->latest_imu_msg_ = *msg;
    // Publish the latest IMU data
    // push the imu messages in the tunnel.
    this->imu_msgs_.push_back(*msg);
    // this->imu_pub_.publish(this->latest_imu_msg_);
  }
}
