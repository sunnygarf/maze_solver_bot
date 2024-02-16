#!/bin/bash

DUR=5

cd ~/catkin_ws &
catkin_make clean
if catkin_make
then
  echo "build success uwu c:"
else
  echo "build error owo :c"
  exit -1
fi

xterm -title "roscore" -hold -e "roscore" &
sleep $DUR

xterm -title "world" -hold -e "roslaunch maze_solver_bot maze_world.launch" &
sleep $DUR

xterm -title "renderer" -hold -e "roslaunch gazebo2rviz gazebo2rviz.launch" &
sleep $DUR

xterm -title "visualiser" -hole -e "rosrun rviz rviz -d `rospack find maze_solver_bot`/rviz/visualiser.rviz" &
sleep $DUR

xterm -title "slam" -hold -e "roslaunch turtlebot3_slam turtlebot3_slam.launch" &
sleep $DUR

xterm -title "trajectory" -hold -e "roslaunch maze_solver_bot trajectory.launch" &
sleep $DUR

xterm -title "solver" -hold -e "roslaunch maze_solver_bot maze_solver_bot.launch" &
sleep $DUR