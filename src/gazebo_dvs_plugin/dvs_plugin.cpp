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

#include <gazebo_dvs_plugin/dvs_plugin.hpp>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(DvsPlugin)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  DvsPlugin::DvsPlugin()
  {
    // store the t1 and t2 for two immediate frames.
    has_last_image_ = false;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  DvsPlugin::~DvsPlugin()
  {
    this->parentCameraSensor.reset();
    this->camera_.reset();
    delete[] simu_;
  }

  // load funtion provide the api for `roslaunch` to execute. It just need a subscriber to accquire the exists sensors' data.
  void DvsPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    if (!_sensor)
    {
      ROS_ERROR("Invalid sensor pointer.");
      return;
    }

#if GAZEBO_MAJOR_VERSION >= 7
    this->parentCameraSensor = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(_sensor);
    this->camera_ = this->parentCameraSensor->Camera();
#else
    this->parentCameraSensor = boost::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
    this->camera_ = this->parentCameraSensor->GetCamera();
#endif

    if (!this->parentCameraSensor)
    {
      ROS_ERROR("DvsPlugin not attached to a camera sensor.");
      return;
    }

#if GAZEBO_MAJOR_VERSION >= 7
    this->width_ = this->camera_->ImageWidth();
    this->height_ = this->camera_->ImageHeight();
    this->format_ = this->camera_->ImageFormat();
#else
    this->width_ = this->camera_->GetImageWidth();
    this->height_ = this->camera_->GetImageHeight();
    this->format_ = this->camera_->GetImageFormat();
#endif

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      ROS_WARN("[DVS_plugin] Please specify a robotNamespace.");

    std::string sensorName = "";
    if (_sdf->HasElement("sensorName"))
      sensorName = _sdf->GetElement("sensorName")->Get<std::string>() + "/";
    else
      ROS_WARN("[DVS_plugin] Please specify a sensorName.");

    std::string eventName = "events";
    if (_sdf->HasElement("eventsTopicName"))
      eventName = _sdf->GetElement("eventsTopicName")->Get<std::string>();

    const std::string eventTopic = sensorName + eventName;

    if (_sdf->HasElement("model"))
    {
      std::string model = _sdf->GetElement("model")->Get<std::string>();
      if (model == "IEBCS")
      {
        simu_ = new IEBCSim(width_, height_, _sdf);
      }
      else if (model == "ESIM")
      {
        simu_ = new Esim(width_, height_, _sdf);
      }
      else if (model == "PLAIN")
      {
        simu_ = new Plain(width_, height_, _sdf);
      }
      else
      {
        ROS_ERROR("[DVS_plugin] Invalid event simulator model!");
        return;
      }
    }
    else
    {
      ROS_WARN("[DVS_plugin] No model was explicit, default is plain Event Camera.");
      simu_ = new Plain(width_, height_, _sdf);
    }

    this->event_pub_ = this->node_handle_.advertise<dvs_msgs::EventArray>(eventTopic, 10000, true);

    this->newFrameConnection = this->camera_->ConnectNewImageFrame(
        boost::bind(&DvsPlugin::mainCallback, this, _1, this->width_, this->height_, this->format_));

    // Make sure the parent sensors are active
    this->parentCameraSensor->SetActive(true);

    // this->clock_sub_ = this->node_handle_.subscribe("/clock", 100, &DvsPlugin::ClockCallback, this);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  // actually this funtion is the main part of dvs_plugin
  void DvsPlugin::mainCallback(const unsigned char *_image,
                               unsigned int _width, unsigned int _height,
                               const std::string &_format)
  {
#if GAZEBO_MAJOR_VERSION >= 7
    _image = this->camera_->ImageData(0);
#else
    _image = this->camera_->GetImageData(0);
#endif
    // add DepthCameraSensor to get the depth_ image
    current_time_ = ros::Time::now();

    /*
#if GAZEBO_MAJOR_VERSION >= 7
float rate = this->camera_->RenderRate();
#else
float rate = this->camera_->GetRenderRate();
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

    input_image.convertTo(input_image, CV_64F);

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
    // accuquire the img message for calibration

    assert(_height == height_ && _width == width_);
    if (has_last_image_)
    {
      std::vector<dvs_msgs::Event> events;
      // this->processDelta(&last_image_, &curr_image, &events);
      simu_->simulateMain(&last_image_, &input_image, &events, current_time_, last_time_);
      last_image_ = input_image;
      last_time_ = current_time_;
      this->publishEvents(&events);
    }
    else if (input_image.size().area() > 0)
    {
      last_image_ = input_image;
      has_last_image_ = true;
      last_time_ = current_time_;
    }
    else
    {
      ROS_WARN("Ignoring empty image.");
    }
  }

  void DvsPlugin::publishEvents(std::vector<dvs_msgs::Event> *events)
  {
    if (events->size() > 0)
    {
      dvs_msgs::EventArray msg;
      msg.events.clear();
      msg.events.insert(msg.events.end(), events->begin(), events->end());
      msg.width = width_;
      msg.height = height_;

      msg.header.frame_id = namespace_;
      msg.header.stamp = ros::Time::now();

      event_pub_.publish(msg);
    }
  }

  // Callback function for the Clock subscriber
  // void DvsPlugin::ClockCallback(const rosgraph_msgs::Clock::ConstPtr &clk)
  // {
  //   clk_ = clk->clock;
  // }
}
