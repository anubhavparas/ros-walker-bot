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
 * @file walkerbot_pubsub.hpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Declaration of the WalkerBotPubSub class
 * @version 0.1
 * @date 2021-11-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_ROS_WALKER_BOT_WALKERBOT_PUBSUB_HPP_
#define INCLUDE_ROS_WALKER_BOT_WALKERBOT_PUBSUB_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ros-walker-bot/obstacle_avoidance.hpp>

class WalkerBotPubSub {
 public:
  /**
   * @brief Construct a new WalkerBotPubSub object
   * 
   * @param ros_node_h 
   */
  explicit WalkerBotPubSub(ros::NodeHandle ros_node_h);

  /**
   * @brief Destroy the WalkerBotPubSub object
   * 
   */
  ~WalkerBotPubSub();

 private:
  ros::NodeHandle ros_node_h;

  /**
   * @brief object responsible for checking if the path is clear for the bot 
   *        and giving the appropriate velocities
   * 
   */
  std::shared_ptr<ObstacleAvoidance> obstacle_avoider;

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

  std::string cmdvel_topic = "/cmd_vel";
  std::string laserscan_topic = "/scan";


  /**
   * @brief Callback method for the laserscan subscriber
   * 
   * @param laserscan_data laserscan topic message
   */
  void laserscan_call_back(
            const sensor_msgs::LaserScan::ConstPtr& laserscan_msg);
};

#endif  // INCLUDE_ROS_WALKER_BOT_WALKERBOT_PUBSUB_HPP_
