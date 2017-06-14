#ifndef JACO_TRAJECTORY_CONTROLLER_H_
#define JACO_TRAJECTORY_CONTROLLER_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/foreach.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ecl/geometry.hpp>
#include <kinova_msgs/JointVelocity.h>
#include <sensor_msgs/JointState.h>

#define NUM_JACO_JOINTS 7

#define LARGE_ACTUATOR_VELOCITY 0.8378 //maximum velocity of large actuator (joints 1-3) (rad/s)
#define SMALL_ACTUATOR_VELOCITY 1.0472 //maximum velocity of small actuator (joints 4-6) (rad/s)
#define TIME_SCALING_FACTOR 1.75 //keep the trajectory at a followable speed

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

//gains for trajectory follower
#define KP 225.0
#define KV 10.0
#define ERROR_THRESHOLD .03 //threshold in radians for combined joint error to consider motion a success

class JacoTrajectoryController
{
public:
  JacoTrajectoryController();

  void jointStateCallback(const sensor_msgs::JointState &msg);

  void executeSmoothTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

private:
  ros::NodeHandle n;
  ros::NodeHandle pnh;

  //Actions
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smoothTrajectoryServer;

  // Messages
  ros::Publisher angularCmdPublisher;
  ros::Subscriber jointStatesSubscriber;

  boost::recursive_mutex executionMutex;

  // Parameters
  double maxCurvature;

  sensor_msgs::JointState jointStates;

  std::vector<std::string> jointNames;
};

#endif

