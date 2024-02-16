#include <yaml-cpp/yaml.h>
#include "controller.hpp"

Controller::Controller(ros::NodeHandle *nh)
{
  _nh = nh;
  parse_data();
}

Controller::~Controller() {}

void Controller::parse_data()
{
  _nh->getParam("DEFAULT_LINEAR_VELOCITY", _DEFAULT_LINEAR_VELOCITY);
  _nh->getParam("DEFAULT_ANGULAR_VELOCITY", _DEFAULT_ANGULAR_VELOCITY);
  _nh->getParam("PROPORTIONAL", _PROPORTIONAL);
  _nh->getParam("DERIVATIVE", _DERIVATIVE);
  _nh->getParam("CORNER_BUFFER", _CORNER_BUFFER);
  _nh->getParam("TARGET_DISTANCE_TO_LEFT_WALL", _TARGET_DISTANCE_TO_LEFT_WALL);
  _nh->getParam("THRESHOLD_DISTANCE_TO_SEEK_CORNER", _THRESHOLD_DISTANCE_TO_SEEK_CORNER);
}

void Controller::calculateGain()
{
  _gain = _PROPORTIONAL * _delta_distance + _DERIVATIVE * _angle_error;
}

void Controller::seekLeftWall()
{
  _linear_velocity = _DEFAULT_LINEAR_VELOCITY;
  _angular_velocity = 0;
}

void Controller::moveForward()
{
  _linear_velocity = _DEFAULT_LINEAR_VELOCITY * 0.2;
  _angular_velocity = _gain;
}

void Controller::turnLeft()
{
  _linear_velocity = 0;
  _angular_velocity = _DEFAULT_ANGULAR_VELOCITY;
}

void Controller::turnRight()
{
  _linear_velocity = 0;
  _angular_velocity = _DEFAULT_ANGULAR_VELOCITY * -1;
}

double Controller::getLinearVelocity()
{
  return _linear_velocity;
}

double Controller::getAngularVelocity()
{
  return _angular_velocity;
}

void Controller::setDeltaDistance(double distance_to_left_wall)
{
  _delta_distance = distance_to_left_wall - _TARGET_DISTANCE_TO_LEFT_WALL;
}

void Controller::setGain(double distance_to_corner)
{
  _gain = (_THRESHOLD_DISTANCE_TO_SEEK_CORNER - distance_to_corner) * _CORNER_BUFFER;
}

void Controller::setAngleError(double angle_error)
{
  _angle_error = angle_error;
}