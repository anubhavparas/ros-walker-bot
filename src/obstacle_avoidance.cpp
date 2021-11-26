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
 * @file obstacle_avoidance.cpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Definitions of the ObstacleAvoidance class
 * @version 0.1
 * @date 2021-11-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros-walker-bot/obstacle_avoidance.hpp>

ObstacleAvoidance::ObstacleAvoidance(ros::NodeHandle ros_node_h) {
  this->ros_node_h = ros_node_h;
  this->velocity_pub = this->ros_node_h.advertise<geometry_msgs::Twist>(
      this->cmdvel_topic, 10);

  this->laser_scan_sub = this->ros_node_h.subscribe(this->laserscan_topic,
                                        10,
                                        &ObstacleAvoidance::laserscan_call_back,
                                        this);
  ros::spinOnce();
}

ObstacleAvoidance::~ObstacleAvoidance() {
}

bool ObstacleAvoidance::is_path_clear(
                     const std::vector<float>& laserscan_data_range,
                     int angle_range) {
  bool path_clear = true;
  int left_ind = 0 + angle_range;
  int right_ind = 360 - angle_range;

  if (angle_range <= 0 || left_ind >= 360 || right_ind < 0) {
    ROS_ERROR_STREAM("Range cannot be more than 360 or less than 0");
    return false;
  }

  // check for left side range
  for (int i=0; i <= left_ind; i++) {
    if (laserscan_data_range[i] <= this->threshold_dist) {
      return false;
    }
  }

  // check for right side range
  for (int i=359; i >= right_ind; i--) {
    if (laserscan_data_range[i] <= this->threshold_dist) {
      return false;
    }
  }
  return path_clear;
}

void ObstacleAvoidance::laserscan_call_back(
    const sensor_msgs::LaserScan::ConstPtr& laserscan_msg) {

  int angle_range = 10;
  int front_ind = 0;
  int left_ind = 0 + angle_range;
  int right_ind = 360 - angle_range;
  this->threshold_dist = 0.8;
  ROS_INFO_STREAM("Laser data::"
                  << " Left: " << laserscan_msg->ranges[left_ind]
                  << " Front: " << laserscan_msg->ranges[front_ind]
                  << " Right: " << laserscan_msg->ranges[right_ind]);

  geometry_msgs::Twist velocity_cmd;

  // check for obstacles:
  if (this->is_path_clear(laserscan_msg->ranges, angle_range)) {
    // move forward
    velocity_cmd.linear.x = 0.5;
    velocity_cmd.angular.z = 0.0;
  } else {
    ROS_WARN_STREAM("Obstacle ahead. Turning...");

    // stop moving forward
    velocity_cmd.linear.x = 0.0;

    // turn left MAX_ONE_SIDE_TURN_COUNT times: ang.z_vel = 1.0
    // and then right MAX_ONE_SIDE_TURN_COUNT times: ang.z_vel = -1.0
    // and repeat
    velocity_cmd.angular.z =
                (this->turn_count < MAX_ONE_SIDE_TURN_COUNT) ? 1.0 : -1.0;
    this->turn_count = (this->turn_count + 1) % (MAX_ONE_SIDE_TURN_COUNT * 2);
  }

  this->velocity_pub.publish(velocity_cmd);
}
