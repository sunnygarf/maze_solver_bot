#include <cmath>
#include "driver.hpp"

Driver::Driver(ros::NodeHandle *nh): _controller(Controller(nh)), _pn("~")
{
  _nh = nh;
  parse_data();

  // wall following parameters
  _distance_to_left_wall = _controller._TARGET_DISTANCE_TO_LEFT_WALL;
  _exists_left_wall = false;

  // corner detection parameters
  _distance_to_corner = _controller._THRESHOLD_DISTANCE_TO_SEEK_CORNER;
  _exists_corner = false;

  _pose = 0;
  _prev_pose = 0;

  // start state - get direction first
  _state = GET_DIRECTION;

  std::string pub_topic = _nh->param<std::string>("pub_topic", "");
  _vel_pub = _nh->advertise<geometry_msgs::Twist>(pub_topic, 10);

  _scan_sub = _nh->subscribe("scan", 10, &Driver::laserScanCallback, this);
  _odom_sub = _nh->subscribe("odom", 10, &Driver::odometryCallback, this);
}

Driver::~Driver()
{
  updateVelocity(0, 0);
  ros::shutdown();
}

void Driver::parse_data()
{
  _nh->getParam("THRESHOLD_DISTANCE_TO_CHECK_FRONT", _THRESHOLD_DISTANCE_TO_CHECK_FRONT);
  _nh->getParam("THRESHOLD_DISTANCE_TO_CHECK_SIDES", _THRESHOLD_DISTANCE_TO_CHECK_SIDES);
  _nh->getParam("MIN_SCAN_ANGLE", _MIN_SCAN_ANGLE);
  _nh->getParam("MAX_SCAN_ANGLE", _MAX_SCAN_ANGLE);
  _nh->getParam("THRESHOLD_DISTANCE_TO_LEFT_WALL", _THRESHOLD_DISTANCE_TO_LEFT_WALL);
}

void Driver::loop()
{
  switch(_state) {
    case GET_DIRECTION:
      /* no front wall within threshold:
      1. about to bump into left wall -> turn right
      2. about to bump into right wall -> turn left
      3. safe from bumping into any wall -> move forward */
      if (_scan_data[FRONT] > _THRESHOLD_DISTANCE_TO_CHECK_FRONT) {
        if (_scan_data[LEFT] < _THRESHOLD_DISTANCE_TO_CHECK_SIDES) {
          _prev_pose = _pose;
          _state = TURN_RIGHT;
        } else if (_scan_data[RIGHT] < _THRESHOLD_DISTANCE_TO_CHECK_SIDES) {
          _prev_pose = _pose;
          _state = TURN_LEFT;
        } else
          _state = MOVE_FORWARD;
      }

      // default to turning right upon encountering front wall
      if (_scan_data[FRONT] < _THRESHOLD_DISTANCE_TO_CHECK_FRONT) {
        _prev_pose = _pose;
        _state = TURN_RIGHT;
      }
      break;

    case MOVE_FORWARD:
      /* left wall exists -> turn while moving forward to align bot
      perpendicular to left wall.
      pos -> left turn, neg -> right turn */
      if (_exists_left_wall) {
        _controller.setDeltaDistance(_distance_to_left_wall);
        _controller.calculateGain();

        if (_exists_corner)
          _controller.setGain(_distance_to_corner);

        _controller.moveForward();
        updateVelocity();
        // no left wall within threshold -> move forward and seek left wall
      } else {
        _controller.seekLeftWall();
        updateVelocity();
      }
      _state = GET_DIRECTION;
      break;

    case TURN_LEFT:
      if (fabs(_prev_pose-_pose) >= _FINISHED_TURNING_THRESHOLD)
        _state = GET_DIRECTION;
      else {
        _controller.turnLeft();
        updateVelocity();
      }
      break;

    case TURN_RIGHT:
      if (fabs(_prev_pose-_pose) >= _FINISHED_TURNING_THRESHOLD)
        _state = GET_DIRECTION;
      else {
        _controller.turnRight();
        updateVelocity();
      }
      break;

    default:
      _state = GET_DIRECTION;
      break;
  }
}

void Driver::updateVelocity()
{
  geometry_msgs::Twist velocity;
  velocity.linear.x = _controller.getLinearVelocity();
  velocity.angular.z = _controller.getAngularVelocity();

  _vel_pub.publish(velocity);
}

void Driver::updateVelocity(double linear_velocity, double angular_velocity)
{
  geometry_msgs::Twist velocity;
  velocity.linear.x = linear_velocity;
  velocity.angular.z = angular_velocity;

  _vel_pub.publish(velocity);
}

void Driver::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  // parse data
  for (int i = 0; i < 3; i++) {
    if (std::isinf(scan->ranges.at(_scan_angles[i])))
      _scan_data[i] = scan->range_max;
    else
      _scan_data[i] = scan->ranges.at(_scan_angles[i]);
  }

  double min_distance_to_left_wall = scan->range_max;
  double distance_to_left_wall;

  // find shortest distance scanned
  for (int i = _MIN_SCAN_ANGLE; i < _MAX_SCAN_ANGLE; i += 5) {
    distance_to_left_wall = scan->ranges.at(i);

    if ((!std::isinf(distance_to_left_wall)) && (distance_to_left_wall < _THRESHOLD_DISTANCE_TO_LEFT_WALL) &&
      (distance_to_left_wall < min_distance_to_left_wall)) {
        min_distance_to_left_wall = distance_to_left_wall;
        _controller.setAngleError((90 - i) * _DEG2RAD);
    }
  }

  // check if left wall within threshold
  if (min_distance_to_left_wall == scan->range_max) {
    _exists_left_wall = false;
    _distance_to_left_wall = _controller._TARGET_DISTANCE_TO_LEFT_WALL;
  } else {
    _exists_left_wall = true;
    _distance_to_left_wall = min_distance_to_left_wall;
  }

  // check if there is a corner; i.e. front wall within threshold
  if (_scan_data[FRONT] < _controller._THRESHOLD_DISTANCE_TO_SEEK_CORNER) {
    _exists_corner = true;
    _distance_to_corner = _scan_data[FRONT];
  } else
    _exists_corner = false;
}

void Driver::odometryCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  double x = odom->pose.pose.orientation.x;
  double y = odom->pose.pose.orientation.y;
  double z = odom->pose.pose.orientation.z;
  double w = odom->pose.pose.orientation.w;

  double sin_y = 2 * (x * y + z * w);
  double cos_y = 1 - 2 * (pow(y, 2) + pow(z, 2));
  _pose = atan2(sin_y, cos_y);
}