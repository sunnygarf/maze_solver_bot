#include <ros/ros.h>
#include "driver.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "maze_solver_bot");
  ros::NodeHandle nh;

  Driver driver(&nh);

  ros::Rate loop_rate(200);
  while (ros::ok()) {
    driver.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}