# DVS Gazebo Plugin

This package provides a DVS simulation implemented as Gazebo plugin, forked from https://github.com/HBPNeurorobotics/gazebo_dvs_plugin.

## Install

First, make sure the DVS datatypes are available in your installation.
For this, clone the [RPG DVS ROS](https://github.com/uzh-rpg/rpg_dvs_ros) package into your catkin workspace.

Then, clone this package into your workspace and rebuild.

## Usage

This plugin can be used as a drop-in replacement for normal Gazebo camera plugins.
Both, the DVS plugin and the [CameraPlugin](https://bitbucket.org/osrf/gazebo/src/666bf30ad9a3c042955b55f79cf1a5416a70d83d/plugins/CameraPlugin.cc)
use the Gazebo [CameraSensor](https://bitbucket.org/osrf/gazebo/src/666bf30ad9a3c042955b55f79cf1a5416a70d83d/gazebo/sensors/CameraSensor.cc) internally.

The following SDF snippet shows an example usage:
```xml
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='event_camera'>
  <pose>0 0 0 0 0 0</pose>
   <link name="link">
        <inertial>
          <mass>0.015</mass>
          <inertia>
            <ixx>4.15e-6</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>2.407e-6</iyy>
          <iyz>0</iyz>
          <izz>2.407e-6</izz>
          </inertia>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.01 0.01 0.01</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.01 0.01 0.01</size>
            </box>
          </geometry>
        </visual>
    <sensor name='camera' type='camera'>
        <camera name='__default__'>
            <horizontal_fov>1.8</horizontal_fov>
            <image>
                <width>128</width>
                <height>128</height>
            </image>
            <clip>
                <near>0.1</near>
                <far>100</far>
            </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>400</update_rate>
        <visualize>1</visualize>
        <plugin name='camera_controller' filename='libgazebo_dvs_plugin.so'>
            <robotNamespace>/</robotNamespace>
            <sensorName>dvs</sensorName>
            <eventsTopicName>events</eventsTopicName>
            <model>IEBCS</model>  <!-- PLAIN, ESIM, IEBCS -->
            <posThreshold>0.5</posThreshold>
            <negThreshold>0.5</negThreshold>
            <noiseThreshold>0.035</noiseThreshold>
            <!-- Valid for IEBCS event camera -->
            <latency>100</latency>
            <jitter>10</jitter>
            <refractory>100</refractory>
            <tau>300</tau>
            <luminance>0.1</luminance>  <!-- 0.1, 161, 3k -->
            <noiseCache>/home/docker/.gazebo/models/event_camera/materials/noise</noiseCache>
        </plugin>
    </sensor>
    <self_collide>0</self_collide>
   <kinematic>0</kinematic>
  </link>
  </model>
</sdf>
```
## Parameter Explaination
| Parameter | Description |
| :---: | --- |
| `robotNamespace` | The root space of ROS, if you don't know how to use it, keep it as default `/`. |
| `sensorName` | The name of the sensor, it will be used as the topic name of the sensor. We suggest to set it as `dvs` |
| `eventsTopicName` | The name of the topic that the sensor will publish events to. We suggest to set it as `events` |
| `model` | The model of the event camera, it can be `PLAIN`, `ESIM`, `IEBCS`, represented for plainly difference between two intermediate frames, [ESIM](http://ieeexplore.ieee.org/document/7862386/) based and [IEBCS](https://github.com/neuromorphicsystems/IEBCS) based. |
| `posThreshold` | The positive threshold of the event camera, it is used to determine the positive event. |
| `negThreshold` | The negative threshold of the event camera, it is used to determine the negative event. |
| `noiseThreshold` | The noise threshold of the event camera, it is used to determine the noise event. We suggest it should no more than 0.1 |
| `latency` | The latency of arbieter with its unit being us. **Only valid for IEBCS model**. |
| `jitter` | The jitter of clock with its unit being us. **Only valid for IEBCS model**. |
| `refractory` | The refractory period of the event camera with its unit being us. **Only valid for IEBCS model**. |
| `tau` | The time constant of 2nd damping process with its unit being us. **Only valid for IEBCS model**. |
| `luminance` | The luminance of the IEBCS based noise parameter, it can be `0.1`, `161`, `3k`. **Only valid for IEBCS model**. |
| `noiseCache` | The path of the noise cache file, it is used to store the noise of the event camera. **Only valid for IEBCS model**. |

The sensor parameter `update_rate` has only limited effect in Gazebo.
The real rate is determined by the rendering pipeline and can be way lower than the specified rate.
Still, this implementation yields a higher event frequency than similar Python-based implementations as a standalone node.
## Recommanded Parameters List
| `model` | `posThreshold` | `negThreshold` | `noiseThreshold` | `latency` | `jitter` | `refractory` | `tau` | `luminance` | `noiseCache` |
|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| `PLAIN` | 0.3 | 0.6 | - | - | - | - | - | - | - |
| `ESIM` | 0.3 | 0.6 | 0.035 | - | - | - | - | - | - |
| `IEBCS` | 0.5 | 0.5 | 0.035 | 100 | 10 | 100 | 300 | 0.1 | `/path/to/your/materials/noise` |

# Acknowledgement

We thank paper [Towards a framework for end-to-end control of a simulated vehicle with spiking neural networks](http://ieeexplore.ieee.org/document/7862386/) and [Event camera simulator improvements via characterized parameters](https://github.com/neuromorphicsystems/IEBCS) for their open source, please cite their papers if you use this code.

```
@INPROCEEDINGS{7862386,
author={J. Kaiser and J. C. V. Tieck and C. Hubschneider and P. Wolf and M. Weber and M. Hoff and A. Friedrich and K. Wojtasik and A. Roennau and R. Kohlhaas and R. Dillmann and J. M. Zöllner},
booktitle={2016 IEEE International Conference on Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR)},
title={Towards a framework for end-to-end control of a simulated vehicle with spiking neural networks},
year={2016},
pages={127-134},
keywords={automobiles;cameras;complex networks;feedforward neural nets;learning (artificial intelligence);mobile robots;DVS;camera images;complex networks;deep learning architectures;end-to-end simulated vehicle control;hand-crafted feature detectors;neural self-driving vehicle applications;neurorobotics applications;rate-based neural networks;silicon retina;spiking neural networks;steering wheel decoder;vehicle end-to-end for lane following behavior;Biological neural networks;Brain modeling;Cameras;Robot sensing systems;Voltage control},
doi={10.1109/SIMPAR.2016.7862386},
month={Dec},}
@article{joubertEventCameraSimulator2021,
  title = {Event Camera Simulator Improvements via Characterized Parameters},
  author = {Joubert, Damien and Marcireau, Alexandre and Ralph, Nic and Jolley, Andrew and family=Schaik, given=André, prefix=van, useprefix=true and Cohen, Gregory},
  date = {2021-07-27},
  journaltitle = {Frontiers in Neuroscience},
  shortjournal = {Front. Neurosci.},
  volume = {15},
  pages = {702765},
  issn = {1662-453X},
  doi = {10/grr5d8},
  url = {https://www.frontiersin.org/articles/10.3389/fnins.2021.702765/full},
  urldate = {2023-02-13},
  langid = {english},
  annotation = {16 citations (Crossref/title) [2024-07-09]},
}

```
