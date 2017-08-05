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
 
#include "guidance_nav.h"

GuidanceNav::GuidanceNav() : nh("~")
{
    ros::NodeHandle np("~");
    nh.param<float>("xy_err_tolerence", xy_err_tolerence, 0.1f);
    nh.param<float>("z_err_tolerence", z_err_tolerence, 0.1f);
    nh.param<float>("yaw_err_tolerence", yaw_err_tolerence, 0.1f);
    nh.param<std::string>("pid_param_file", pid_param_file, "");

    // load default pid parameters
    load_pid_param(pid_param_file, pid);

    // initialize subscribers
    odom_sub = nh.subscribe("odom", 1, &GuidanceNav::odom_callback, this);

    // initialize publishers
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // initialize actions
    guidance_nav_action_server = new GuidanceNavActionServer(nh,
            "guidance_nav_action",
            boost::bind(&GuidanceNav::guidance_nav_action_callback, this, _1), false);
    guidance_nav_action_server->start();

    std::cout << "                                                                         \n       111                      111                                      \n       111                      111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111  1111111  111111111  11111111   1111111  111111111   11111111 \n       111 1111 1111   1111     1111 111  1111 111  11111 111  111 111   \n       111 111   111   1111     111  1111 11    111 1111  111  111  111  \n 111   111 111111111   1111     111   111    111111 111   111  111 1111  \n 111   111 111         1111     111   111 111111111 111   111  1111111   \n 111  1111 111         1111     111   111 111   111 111   111  111111    \n 111  111  1111  111    111     111  1111 111  1111 111   111  111       \n  1111111   11111111    111111  11111111  111111111 111   111  11111111  \n   11111     11111       11111  1111111    11111111 111   111  111  1111 \n                                                              111    111 \n                                                               11111111  " << std::endl;

    std::cout << "\n----------------------------Copyright 2017. Jetbang----------------------------" << std::endl;
}

GuidanceNav::~GuidanceNav()
{
    if (guidance_nav_action_server != NULL)
    {   
        delete guidance_nav_action_server;
        guidance_nav_action_server = NULL;
    }
}

void GuidanceNav::fill_pid_param(PID_t* pid, const cv::FileNodeIterator& it)
{
    pid->kp = (float)(*it)["kp"];
    pid->ki = (float)(*it)["ki"];
    pid->kd = (float)(*it)["kd"];
    pid->db = (float)(*it)["db"];
    pid->it = (float)(*it)["it"];
    pid->Emax = (float)(*it)["Emax"];
    pid->Pmax = (float)(*it)["Pmax"];
    pid->Imax = (float)(*it)["Imax"];
    pid->Dmax = (float)(*it)["Dmax"];
    pid->Omax = (float)(*it)["Omax"];

    PID_Reset(pid);

    std::cout << "pid param:" << std::endl;
    std::cout << pid->kp << "  ";
    std::cout << pid->ki << "  ";
    std::cout << pid->kd << "  ";
    std::cout << pid->db << "  ";
    std::cout << pid->it << "  ";
    std::cout << pid->Emax << "  ";
    std::cout << pid->Pmax << "  ";
    std::cout << pid->Imax << "  ";
    std::cout << pid->Dmax << "  ";
    std::cout << pid->Omax << "  ";
    std::cout << std::endl;
}

bool GuidanceNav::load_pid_param(std::string filepath, PID_t* pid)
{
    std::cout << "Loading pid configuration file: " << filepath << std::endl;
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if( !fs.isOpened()) // if we have file with parameters, read them
    {
        std::cout<<"ERROR, cannot open pid configuration file: "<< filepath << std::endl;
    }
    cv::FileNode list_n = fs["pid"];
    cv::FileNodeIterator it = list_n.begin(), it_end = list_n.end();
    fill_pid_param(&pid[0], it); // x
    fill_pid_param(&pid[1], it); // y
    for (int i = 2; i < 4 && it != it_end; ++it)
    {
        fill_pid_param(&pid[i], it);
    }
    fs.release();
    return true;
}

/*    
void GuidanceNav::fill_pid_param(int i, std::map<std::string, double> pid_param)
{
    pid[i].kp = pid_param["kp"];
    pid[i].ki = pid_param["ki"];
    pid[i].kd = pid_param["kd"];
    pid[i].db = pid_param["db"];
    pid[i].it = pid_param["it"];
    pid[i].Emax = pid_param["Emax"];
    pid[i].Pmax = pid_param["Pmax"];
    pid[i].Imax = pid_param["Imax"];
    pid[i].Dmax = pid_param["Dmax"];
    pid[i].Omax = pid_param["Omax"];
}

bool GuidanceNav::load_pid_params()
{
    std::vector<std::map<std::string, double> > pid_params;
    std::map<std::string, double> d_pid_params;
    nh.getParam("pid", d_pid_params);
    //nh.getParam("pid", pid_params);
    if (pid_params.size() < 3)
    {
        return false;
    }
    fill_pid_param(0, pid_params[0]); // x
    fill_pid_param(1, pid_params[0]); // y
    for (int i = 1; i < pid_params.size(); i++)
    {
        fill_pid_param(i + 1, pid_params[i]);
    }
    return true;
}
*/

void GuidanceNav::odom_callback(const nav_msgs::Odometry& g_odom)
{
    odom = g_odom;
}

bool GuidanceNav::pid_control(float x, float y, float z, float yaw)
{
    float ref = 0;
    float fdb = 0;

    ref = x;
    fdb = odom.pose.pose.position.x;
    PID_Calc(&pid[0], ref, fdb);

    ref = y;
    fdb = odom.pose.pose.position.y;
    PID_Calc(&pid[1], ref, fdb);

    ref = z;
    fdb = odom.pose.pose.position.z;
    PID_Calc(&pid[2], ref, fdb);
    
    ref = yaw;
    fdb = tf::getYaw(odom.pose.pose.orientation);
    PID_Calc(&pid[3], ref, fdb);

    twist.linear.x = pid[0].out;
    twist.linear.y = pid[1].out;
    twist.linear.z = pid[2].out;
    twist.angular.z = pid[3].out;
}

void GuidanceNav::publish_twist()
{
    twist_pub.publish(twist);
}

bool GuidanceNav::guidance_nav_action_callback(const guidance_nav::GuidanceNavGoalConstPtr& goal)
{
    float dst_x = goal->x;
    float dst_y = goal->y;
    float dst_z = goal->z;
    float dst_yaw = goal->yaw;

    float org_x = odom.pose.pose.position.x;
    float org_y = odom.pose.pose.position.y;
    float org_z = odom.pose.pose.position.z;
    float org_yaw = tf::getYaw(odom.pose.pose.orientation);

    float dis_x = dst_x - org_x;
    float dis_y = dst_y - org_y;
    float dis_z = dst_z - org_z; 
    float dis_yaw = dst_yaw - org_yaw;

    float det_x, det_y, det_z, det_yaw;

    int x_progress = 0; 
    int y_progress = 0; 
    int z_progress = 0; 
    int yaw_progress = 0;

    while (x_progress < 100 || y_progress < 100 || z_progress < 100 || yaw_progress < 100) {

        float yaw = tf::getYaw(odom.pose.pose.orientation);

        det_x = (100 * (dst_x - odom.pose.pose.position.x)) / dis_x;
        det_y = (100 * (dst_y - odom.pose.pose.position.y)) / dis_y;
        det_z = (100 * (dst_z - odom.pose.pose.position.z)) / dis_z;
        det_yaw = (100 * (dst_yaw - yaw)) / dis_yaw;

        x_progress = 100 - (int)det_x;
        y_progress = 100 - (int)det_y;
        z_progress = 100 - (int)det_z;
        yaw_progress = 100 - (int)det_yaw;

        //lazy evaluation
        if (std::abs(dst_x - odom.pose.pose.position.x) < xy_err_tolerence) x_progress = 100;
        if (std::abs(dst_y - odom.pose.pose.position.y) < xy_err_tolerence) y_progress = 100;
        if (std::abs(dst_z - odom.pose.pose.position.z) < z_err_tolerence) z_progress = 100;
        if (std::abs(dst_yaw - yaw) < yaw_err_tolerence) yaw_progress = 100;

        guidance_nav_feedback.x_progress = x_progress;
        guidance_nav_feedback.y_progress = y_progress;
        guidance_nav_feedback.z_progress = z_progress;
        guidance_nav_feedback.yaw_progress = yaw_progress;

        guidance_nav_action_server->publishFeedback(guidance_nav_feedback);

        pid_control(dst_x, dst_y, dst_z, dst_yaw);
        publish_twist();

        usleep(20000);
    }

    guidance_nav_result.result = true;
    guidance_nav_action_server->setSucceeded(guidance_nav_result);

    return true;
}

