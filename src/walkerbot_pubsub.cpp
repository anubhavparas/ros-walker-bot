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
 * @file walkerbot_pubsub.cpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Definitions of the WalkerBotPubSub class
 * @version 0.1
 * @date 2021-11-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros-walker-bot/walkerbot_pubsub.hpp>

WalkerBotPubSub::WalkerBotPubSub(ros::NodeHandle ros_node_h)
            : obstacle_avoider(std::make_shared<ObstacleAvoidance>()) {
  this->ros_node_h = ros_node_h;
  this->velocity_pub = this->ros_node_h.advertise<geometry_msgs::Twist>(
      this->cmdvel_topic, 10);

  this->laser_scan_sub = this->ros_node_h.subscribe(
                                    this->laserscan_topic,
                                    10,
                                    &WalkerBotPubSub::laserscan_call_back,
                                    this);
  ros::spinOnce();
}

WalkerBotPubSub::~WalkerBotPubSub() {
}

void WalkerBotPubSub::laserscan_call_back(
    const sensor_msgs::LaserScan::ConstPtr& laserscan_msg) {
  int angle_range = 10;
  int front_ind = 0;
  int left_ind = 0 + angle_range;
  int right_ind = 360 - angle_range;
  geometry_msgs::Twist velocity_cmd;

  ROS_INFO_STREAM("Laser data::"
                  << " Left: " << laserscan_msg->ranges[left_ind]
                  << " Front: " << laserscan_msg->ranges[front_ind]
                  << " Right: " << laserscan_msg->ranges[right_ind]);

  velocity_cmd = this->obstacle_avoider->get_bot_velocity(
      laserscan_msg->ranges,
      angle_range);

  this->velocity_pub.publish(velocity_cmd);
}
