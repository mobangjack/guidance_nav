/**
 * Copyright (c) 2017, Jack Mo (mobangjack@foxmail.com).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#pragma once

#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include<tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

#include <string>
#include <map>

#include <opencv2/opencv.hpp>

#include "guidance_nav/GuidanceNavAction.h"

#include "pid.h"

class GuidanceNav
{
public:
    GuidanceNav(); // Constructor
    ~GuidanceNav(); // Destructor

protected:
    ros::NodeHandle nh;
    
    // subscribers
    ros::Subscriber odom_sub;

    // publishers
    ros::Publisher twist_pub;

    // actions
    typedef actionlib::SimpleActionServer<guidance_nav::GuidanceNavAction> GuidanceNavActionServer;

    GuidanceNavActionServer* guidance_nav_action_server;
    guidance_nav::GuidanceNavFeedback guidance_nav_feedback;
    guidance_nav::GuidanceNavResult guidance_nav_result;

protected:
    nav_msgs::Odometry odom;  // Guidance position (Odometry) feedback

    // other member variables
    PID_t pid[4]; // x, y, z, yaw

    float xy_err_tolerence;
    float z_err_tolerence;
    float yaw_err_tolerence;
    
    std::string pid_param_file;

    geometry_msgs::Twist twist;
/*
    void fill_pid_param(int i, std::map<std::string, double> pid_param);
    bool load_pid_params();
*/
    void fill_pid_param(PID_t* pid, const cv::FileNodeIterator& it);
    bool load_pid_param(std::string filepath, PID_t* pid);
    
protected:
    // subscriber callbacks
    void odom_callback(const nav_msgs::Odometry& g_odom);

    void publish_twist();
    
    // pid control
    bool pid_control(float x, float y, float z, float yaw);

    // action callbacks
    bool guidance_nav_action_callback(const guidance_nav::GuidanceNavGoalConstPtr& goal);

};
