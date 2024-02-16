#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <ros/ros.h>

class Controller
{
  public:
    Controller(ros::NodeHandle*);
    ~Controller();

    void parse_data(void);
    void calculateGain(void);

    /* set linear and angular velocities for desired motion. enables the bot to
    pivot while moving, always perpendicular to the left wall */
    void seekLeftWall(void);
    void moveForward(void);
    void turnLeft(void);
    void turnRight(void);

    double getLinearVelocity(void);
    double getAngularVelocity(void);

    void setDeltaDistance(double distance_to_left_wall);
    void setGain(double distance_to_corner);
    void setAngleError(double angle_error);

    double _TARGET_DISTANCE_TO_LEFT_WALL;

    // bot can loop around if it seeks for corner too early
    double _THRESHOLD_DISTANCE_TO_SEEK_CORNER;

  private:
    double _DEFAULT_LINEAR_VELOCITY;
    double _DEFAULT_ANGULAR_VELOCITY;

    // controller parameters
    double _PROPORTIONAL;
    double _DERIVATIVE;

    /* buffer to tune corner turning radius.
    bot can get stuck in corner without buffer */
    double _CORNER_BUFFER;

    double _linear_velocity;
    double _angular_velocity;

    /* difference between current and target distance to left wall.
    pos -> left turn, neg -> right turn */
    double _delta_distance;
    double _gain;         // proportional input + derivative feedback
    double _angle_error;  // angle offset from normal to left wall (90 deg)

    ros::NodeHandle* _nh;
};

#endif