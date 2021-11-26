/**
 * MIT License
 *
 * Copyright (c) 2021 Anubhav Paras
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file obstacle_avoidance.hpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Declaration of the ObstacleAvoidance class
 * @version 0.1
 * @date 2021-11-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_ROS_WALKER_BOT_OBSTACLE_AVOIDANCE_HPP_
#define INCLUDE_ROS_WALKER_BOT_OBSTACLE_AVOIDANCE_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

class ObstacleAvoidance {
 public:
  /**
   * @brief Construct a new Obstacle Avoidance object
   * 
   * @param ros_node_h 
   */
  explicit ObstacleAvoidance(ros::NodeHandle ros_node_h);

  /**
   * @brief Destroy the Obstacle Avoidance object
   * 
   */
  ~ObstacleAvoidance();

 private:
  ros::NodeHandle ros_node_h;

  /**
   * @brief publisher object for the /cmd_vel topic
   * 
   */
  ros::Publisher velocity_pub;

  /**
   * @brief subsriber to the /sensor_msgs/LaserScan topic
   * 
   */
  ros::Subscriber laser_scan_sub;

  /**
   * @brief minimum distance of the bot from the obstacle
   * 
   */
  float threshold_dist = 0.8;  // in metres
  std::string cmdvel_topic = "/cmd_vel";
  std::string laserscan_topic = "/scan";

  int turn_count = 0;
  int MAX_ONE_SIDE_TURN_COUNT = 100;


  /**
   * @brief Callback method for the laserscan subscriber
   * 
   * @param laserscan_data laserscan topic message
   */
  void laserscan_call_back(
            const sensor_msgs::LaserScan::ConstPtr& laserscan_msg);

  /**
   * @brief to check if there
   * 
   * @param laserscan_data_range distances of the obstacles at every angle from 0-360
   * @param angle_range arc angle to get the area that should be obstacle free in either side
   *                    if angle_range=10:
   *                    obstacles will be checked in range: 10 deg left and 10 deg right of the bot 
   * @return true if the path is clear and there are no obstacles with a certain range 
   * @return false if obstacle/s is/are there within a certain range
   */
  bool is_path_clear(const std::vector<float>& laserscan_data_range,
                     int angle_range);
};

#endif  // INCLUDE_ROS_WALKER_BOT_OBSTACLE_AVOIDANCE_HPP_
