#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

#include "controller.hpp"

class Driver
{
  public:
    Driver(ros::NodeHandle*);
    ~Driver();

    void parse_data(void);
    void loop(void);

  private:
    void updateVelocity(void);
    void updateVelocity(double linear_velocity, double angular_velocity);

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);

    static int const FRONT = 0;
    static int const LEFT = 1;
    static int const RIGHT = 2;

    static int const GET_DIRECTION = 0;
    static int const MOVE_FORWARD = 1;
    static int const TURN_LEFT = 2;
    static int const TURN_RIGHT = 3;
    int _state;

    const double _DEG2RAD = M_PI / 180;
    const double _FINISHED_TURNING_THRESHOLD = _DEG2RAD * 15;

    const uint16_t _scan_angles[3] = {0, 30, 330};
    double _scan_data[3];

    double _THRESHOLD_DISTANCE_TO_CHECK_FRONT;
    double _THRESHOLD_DISTANCE_TO_CHECK_SIDES;

    // scan range to check existence of left wall
    double _MIN_SCAN_ANGLE;
    double _MAX_SCAN_ANGLE;

    double _THRESHOLD_DISTANCE_TO_LEFT_WALL;

    // wall following
    double _distance_to_left_wall;
    bool _exists_left_wall;

    // corner detection
    double _distance_to_corner;
    bool _exists_corner;

    // position tracking
    double _pose;
    double _prev_pose;

    Controller _controller;

    ros::Publisher _vel_pub;
    ros::Subscriber _scan_sub;
    ros::Subscriber _odom_sub;

    ros::NodeHandle* _nh;
    ros::NodeHandle _pn;
};

#endif