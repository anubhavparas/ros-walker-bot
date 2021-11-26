# Walker Bot
Simple ROS-based obstacle avoidance robot.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
 ---
## Overview
Walker bot is a turtlebot3-based robot capable of moving in an environment while avoiding obstacles. If the bot encounters an obstacle within a pre-defined distance range it stops moving forward and turns until there is no obstacle in its path.

The `ros_walker_bot_node` in this package gets the distances of the obstacles by subscribing to the `\scan` topic that contains the laser scan data of the Turtlebot and publishes linear and angular velocity commands to `\cmd_vel` topic of the Turtlebot.

## Dependencies
- Ubuntu 18.04 (LTS)
- ROS Melodic
- Turtlebot3 ROS Package

## Instructions to build and run the code
 - Make sure you have ROS Melodic installed in your computer. If not refer to [site](http://wiki.ros.org/melodic/Installation/Ubuntu).
 
 - Create a workspace:
    ```
    mkdir -p ~/walkerbot_ws/src
    cd ~/walkerbot_ws/src
    ```
 - Clone the repository into the workspace:
    ```
    git clone https://github.com/anubhavparas/ros-walker-bot.git
    ```
 - Build the workspace:
    ```
    cd ~/walkerbot_ws
    catkin_make or catkin build (preferred)
    source devel/setup.bash
    ```
### Turtlebot3 installation:
 - This `ros_walker_bot` package also needs Turtlebot3 ROS package to be there. Follow the instructions to install turtlebot3:
    ```
    cd ~/walkerbot_ws/src
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
    cd ~/walkerbot_ws/
    catkin_make or catkin build (preferred)
    source devel/setup.bash
    ```

  - Set the environment variable for the turtlebots model:
    ```
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    ```


### Running with the launch file
- Run using launch file: This will spawn:
    - a ros_walker_bot node: `ros_walker_bot_node`
    - gazebo world with turtlebot3
    ```
    roslaunch ros_walker_bot ros_walker_bot.launch 
    
    # arguments are:
    # is_record_bag - default=false
    # launch_gazebo - default=true
    # bagfile - default=recorded_topics.bag
    ```

- To enable `rosbag` recording (by default it is disabled): 
    - This will store the rosbag recording in a file `home/<username>/.ros/recorded_topics.bag`
        ```
        roslaunch ros_walker_bot ros_walker_bot.launch is_record_bag:=true
        ```
    - __Note__: rosbag record will not be recording any messages in any of the `*/camera*` topics.


### Running without the launch file

- To launch gazebo and turtlebot3 run the following in a new terminal. This will start `rosmaster` node too:
    ```
    cd ~/walkerbot_ws/
    source devel/setup.bash
    roslaunch turtlebot3_gazebo turtlebot3_house.launch
    ```

- To run the ros_walker_bot_node run the following in a new terminal:
    ```
    cd ~/walkerbot_ws/
    source devel/setup.bash
    rosrun ros_walker_bot ros_walker_bot_node
    ```

### Setting the logger level
- To set the logger level of `ros_walker_bot` to `info`/`debug`:
  ```
  rosservice call /ros_walker_bot/set_logger_level "{logger: 'rosout', level: 'debug'}"
  ```

### Rosbag record and play
- To run rosbag recording run the following in a new terminal (assuming the above nodes are running):
    ```
    cd ~/walkerbot_ws/
    source devel/setup.bash
    rosbag record -O record_topics.bag -a -x '(.*)/camera(.*)' --duration 30

    # -O record_topics.bag: name of the output bag file
    # -a: to record all topics
    # -x: to exclude certain topics. here, /camera topics will not be recorded
    # --duration 30: record for 30 sec 
    ```

- To verify the recording:
    - Examining the bag file:
        ```
        rosbag info <your bagfile>
        ```
    - Stop/close the gazebo world and only start the ros_walker_bot_node:
        - in a new terminal start:  `$ roscore`
        ```
        # in another new terminal run:
        cd ~/walkerbot_ws/
        source devel/setup.bash
        rosrun ros_walker_bot ros_walker_bot_node
        ```
    - Replay the bag file in a new terminal: 
        ```
        rosbag play record_topics.bag
        ```
    - You can verify that the ros_walker_bot_node is now subscribing to the recorded messages (`/scan` topic).

- Run a sample recorded rosbag [file](results/bag).
    - In a new terminal start the ros_walker_bot_node to subscribe the `/scan` topic:
        ```
        cd ~/walkerbot_ws/
        source devel/setup.bash
        roslaunch ros_walker_bot ros_walker_bot.launch launch_gazebo:=false
        ```
    - In a new terminal replay the bag file. This is a 30 sec recording of many topics including `/scan` topic.
        ```
        cd ~/walkerbot_ws/
        source devel/setup.bash
        roscd ros_walker_bot/results/bag
        ```
    - Examining the bag file: [Sample output](results/rosbag_info.png).
        ```
        rosbag info record_topics.bag
        ```
    - Play the bag file: [Sample output](results/rosbag_replay_demo.png).
        ```
        rosbag play record_topics.bag
   
        ```
## Run cppcheck and cpplint
Run cppcheck: Results are stored in `./results/cppcheck_process.txt`, `./results/cppcheck_result.txt` 
```
cd ~/walkerbot_ws/src/ros-walker-bot
```
```
sh run_cppcheck.sh
```

Run cpplint: Results are stored in `./results/cpplint_result.txt`
```
sh run_cpplint.sh
```
