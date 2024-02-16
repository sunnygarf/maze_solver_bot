# Maze Solver Bot

https://github.com/sunnygarf/maze_solver_bot/assets/155351380/c641af27-9844-412e-bf82-0d61c9c80a1e

This repository contains the implementation of a maze solver bot utilising the wall-following algorithm, simulation environment, and trajectory visualiser. This project is written in C++ with the ROS (Robot Operating System) framework (roscpp), and offers a solution for autonomously navigating through mazes.

## Getting Started

This guide outlines the steps to run the maze solver bot on Ubuntu 20.04 with ROS Noetic. For other platforms, please refer to additional resources.

### Setup

1. Desktop-Full Install ROS Noetic following the [Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu).
2. Add the following lines to the end of the configuration file `~/.bash_profile`. Here we source the installed ROS packages and the workspace that we will be working in.

   ```
   source /opt/ros/noetic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   export TURTLEBOT3_MODEL=burger
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/maze_solver_bot/models
   ```
3. Add the following lines to the end of the configuration file `~/.bashrc`. This sources our previous configuration for terminals.

   ```
   if [ -f ~/.bash_profile ]; then
       . ~/.bash_profile
   fi
   ```
4. Execute the following command to install xterm: `sudo apt install xterm`. xterm is used to execute our modules in the script file `run.sh`.
5. Execute the following command to install `slam_gmapping`: `sudo apt install ros-noetic-slam-gmapping`. This is a package that does not come with the base installation.
6. Create a workspace `~/catkin_ws/src/` and place this repository here.
7. Extract the dependencies from `packages.zip` and place them in `~/catkin_ws/src/` as well. The depedencies are listed with basic descriptions below:

   - turtlebot3 configuration and simulation -  `turtlebot3_bringup`, `turtlebot3_description`, `turtlebot3_slam` - [turtlebot3 Repository](https://github.com/ROBOTIS-GIT/turtlebot3)
   - Dependency for turtlebot3 packages - `turtlebot3_msgs` - [Repository](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
   - Model visualiser - `gazebo2rviz` - [Repository](https://github.com/andreasBihlmaier/gazebo2rviz)
   - Dependency for the above - `pysdf` - [Repository](https://github.com/andreasBihlmaier/pysdf)
   - Trajectory visualiser - `hector_map_tools`, `hector_nav_msgs`, `hector_trajectory_server` - [hector_slam Repository](https://github.com/tu-darmstadt-ros-pkg/hector_slam)
8. Place the model files  `model.config` and `model.sdf` in `~/.gazebo/models/`. Premade files can be found in this repository under `/models`.
9. Execute the command `catkin_make` in `~/catkin_ws/` to build the workspace.

### Execution

0. Fast Execution (executes all of the steps below with no timing control; make sure to place and run the file `run.sh` in `~/catkin_ws/` or any of its parent directories)

   ```
   ./run.sh
   ```
1. ROS (redundant step since `roslaunch` automatically boots up ROS, but is a good ritual)

   ```
   roscore
   ```
2. Simulation Environment (Gazebo with model and bot)

   ```
   roslaunch maze_solver_bot maze_world.launch
   ```
3. Visualisation Renderer (Gazebo model to RViz)

   ```
   roslaunch gazebo2rviz gazebo2rviz.launch
   ```
4. Environment Visualiser (RViz with configured settings)

   ```
   rosrun rviz rviz -d `rospack find maze_solver_bot`/rviz/visualiser.rviz
   ```
5. SLAM (also launches another RViz window with SLAM visualisation; do not close this as it shuts down SLAM as well)

   ```
   roslaunch turtlebot3_slam turtlebot3_slam.launch
   ```
6. Trajectory Visualiser

   ```
   roslaunch maze_solver_bot trajectory.launch
   ```
7. Maze Solver (bot control)

   ```
   roslaunch maze_solver_bot maze_solver_bot.launch
   ```

### Configuration

- Parameters such as default velocities, controller values, operation thresholds, scan angles and more can be tuned in `/config/param.yml`.
- Custom models can be placed in `/models`, or can set your own custom path by changing the environment variable `GAZEBO_MODEL_PATH` in `~/.bash_profile`. Choose the model to load by replacing the following lines (22) with your desired model (file) name in `/worlds/maze.world`:

  ```
      <!-- maze -->
      <include>
        <uri>model://enclosed</uri>
      </include>
  ```
- Starting position for the bot can be specified in `/launch/maze_world.launch`. Note that the bot can be placed in its desired starting position (and can be moved any time) using the Gazebo GUI as well.
