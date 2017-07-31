#include <kinova_driver/jaco_trajectory_controller.h>

using namespace std;

JacoTrajectoryController::JacoTrajectoryController() : pnh("~"),
  smoothTrajectoryServer(pnh, "trajectory", boost::bind(&JacoTrajectoryController::executeSmoothTrajectory, this, _1), false)
{
  pnh.param("max_curvature", maxCurvature, 100.0);
  pnh.param("sim", sim_flag_, false);

  jointNames.clear();
  jointNames.push_back("j2s7s300_joint_1");
  jointNames.push_back("j2s7s300_joint_2");
  jointNames.push_back("j2s7s300_joint_3");
  jointNames.push_back("j2s7s300_joint_4");
  jointNames.push_back("j2s7s300_joint_5");
  jointNames.push_back("j2s7s300_joint_6");
  jointNames.push_back("j2s7s300_joint_7");

  // Setting up simulation vs. real robot
  if(!sim_flag_)
  {
    ROS_INFO("Using real robot arm.");

    // Connect to the low-level angular driver from kinova-ros
    angularCmdPublisher = n.advertise<kinova_msgs::JointVelocity>("j2s7s300_driver/in/joint_velocity", 1);
  }
  else
  {
    ROS_INFO("Using simulation robot arm.");

    // Setup a fake gravity comp service
    start_gravity_comp_ = n.advertiseService(
                "j2s7s300/in/start_gravity_comp", &JacoTrajectoryController::startGravityCompService, this);
    stop_gravity_comp_ = n.advertiseService(
                "j2s7s300/in/stop_gravity_comp", &JacoTrajectoryController::stopGravityCompService, this);

    // Connect to the gazebo low-level ros controller
    angCmdSimPublisher = n.advertise<trajectory_msgs::JointTrajectory>("/j2s7s300/command", 1);
  }

  // Subscribes to the joint states of the robot
  jointStatesSubscriber = n.subscribe("joint_states", 1, &JacoTrajectoryController::jointStateCallback, this);


  // Start the trajectory server
  smoothTrajectoryServer.start();
}

/** Fake Gravity Comp Services for Simulation **/
bool JacoTrajectoryController::startGravityCompService(kinova_msgs::Start::Request &req,
                                             kinova_msgs::Start::Response &res)
{
    ROS_INFO("Simulation 'enabled' grav comp. Note: nothing actually happens");
    res.start_result = "Start gravity compensation requested.";
    return true;
}

/** Fake Gravity Comp Services for Simulation **/
bool JacoTrajectoryController::stopGravityCompService(kinova_msgs::Stop::Request &req,
                                             kinova_msgs::Stop::Response &res)
{
    ROS_INFO("Simulation 'disabled' grav comp. Note: nothing actually happens");
    res.stop_result = "Stop gravity compensation requested.";
    return true;
}

void JacoTrajectoryController::jointStateCallback(const sensor_msgs::JointState &msg)
{
  jointStates = msg;
}

/** Adjust angle to equivalent angle on [-pi, pi]
 *  @param angle the angle to be simplified (-inf, inf)
 *  @return the simplified angle on [-pi, pi]
 */
static inline double simplify_angle(double angle)
{
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double current_rev;
  if (fabs(angle - previous_rev) < fabs(angle - next_rev))
    return angle - previous_rev;
  return angle - next_rev;
}

/** Calculates nearest desired angle to the current angle
 *  @param desired desired joint angle [-pi, pi]
 *  @param current current angle (-inf, inf)
 *  @return the closest equivalent angle (-inf, inf)
 */
static inline double nearest_equivalent(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
    return lowVal;
  if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
    return medVal;
  return highVal;
}

void JacoTrajectoryController::executeSmoothTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  float trajectoryPoints[NUM_JACO_JOINTS][goal->trajectory.points.size()];
  int numPoints = goal->trajectory.points.size();

  //get trajectory data
  for (unsigned int i = 0; i < numPoints; i++)
  {
    for (int trajectoryIndex = 0; trajectoryIndex < goal->trajectory.joint_names.size(); trajectoryIndex ++)
    {
      string jointName = goal->trajectory.joint_names[trajectoryIndex];
      int jointIndex = distance(jointNames.begin(), find(jointNames.begin(), jointNames.end(), jointName));
      if (jointIndex >= 0 && jointIndex < NUM_JACO_JOINTS)
      {
        trajectoryPoints[jointIndex][i] = goal->trajectory.points.at(i).positions.at(trajectoryIndex);
      }
    }
  }

  //initialize arrays needed to fit a smooth trajectory to the given points
  ecl::Array<double> timePoints(numPoints);
  timePoints[0] = 0.0;
  vector<ecl::Array<double> > jointPoints;
  jointPoints.resize(NUM_JACO_JOINTS);
  float prevPoint[NUM_JACO_JOINTS];
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
  {
    jointPoints[i].resize(numPoints);
    jointPoints[i][0] = trajectoryPoints[i][0];
    prevPoint[i] = trajectoryPoints[i][0];
  }

  //determine time component of trajectories for each joint
  for (unsigned int i = 1; i < numPoints; i++)
  {
    float maxTime = 0.0;
    for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
    {
      //calculate approximate time required to move to the next position
      float time = fabs(trajectoryPoints[j][i] - prevPoint[j]);
      if (j <= 3)
        time /= LARGE_ACTUATOR_VELOCITY;
      else
        time /= SMALL_ACTUATOR_VELOCITY;

      if (time > maxTime)
        maxTime = time;

      jointPoints[j][i] = trajectoryPoints[j][i];
      prevPoint[j] = trajectoryPoints[j][i];
    }

    timePoints[i] = timePoints[i - 1] + maxTime * TIME_SCALING_FACTOR;
  }

  vector<ecl::SmoothLinearSpline> splines;
  splines.resize(NUM_JACO_JOINTS);
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
  {
    ecl::SmoothLinearSpline tempSpline(timePoints, jointPoints[i], maxCurvature);
    splines.at(i) = tempSpline;
  }

  //control loop
  bool trajectoryComplete = false;
  double startTime = ros::Time::now().toSec();
  double t = 0;
  float error[NUM_JACO_JOINTS];
  float totalError;
  float prevError[NUM_JACO_JOINTS] = {0};
  float currentPoint;
  double current_joint_pos[NUM_JACO_JOINTS];
  kinova_msgs::JointVelocity trajectoryPoint;
  ros::Rate rate(100);
  bool reachedFinalPoint;
  ros::Time finalPointTime;

  // Check if we send the trajectory to simulation vs. real robot
  // if sim just send position trajectory and not run the PID loop
  if (sim_flag_)
  {
    ROS_INFO("WARNING: Simulation velocity trajectory is executed as a position trajectory");

    // Setup msg JointTrajectory for std gazebo ros controller
    // Populate JointTrajectory with the points in the current goal
    trajectory_msgs::JointTrajectory jtm;
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtm.joint_names = goal->trajectory.joint_names;
    jtm.points = goal->trajectory.points;

    // Publish out trajaectory points listed in the goal
    angCmdSimPublisher.publish(jtm);

    // Wait a second for the arm to move a bit
    ros::Duration(1.0).sleep();

    // Check the total error to determine if the trajectory finished
    totalError = 1.0;
    float prevError = 0.0;
    while (abs(totalError -prevError) > 0.001 && totalError > 0.03){
        prevError = totalError;

        // Copy from joint_state publisher to current joints
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {   
          current_joint_pos[i] = jointStates.position[i];
        } 

        // Compute total error of all joints away from last position
        totalError = 0;
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          currentPoint = simplify_angle(current_joint_pos[i]);
          error[i] = nearest_equivalent(simplify_angle(goal->trajectory.points[numPoints-1].positions[i]),currentPoint) - currentPoint;
          totalError += fabs(error[i]);
        }

    }

    // Tell the server we finished the trajectory
    ROS_INFO("Trajectory Control Complete.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smoothTrajectoryServer.setSucceeded(result);

  }
  else
  {

    // Sending to the real robot
    while (!trajectoryComplete)
    {
      //check for preempt requests from clients
      if (smoothTrajectoryServer.isPreemptRequested())
      {
        //stop gripper control
        trajectoryPoint.joint1 = 0.0;
        trajectoryPoint.joint2 = 0.0;
        trajectoryPoint.joint3 = 0.0;
        trajectoryPoint.joint4 = 0.0;
        trajectoryPoint.joint5 = 0.0;
        trajectoryPoint.joint6 = 0.0;
        trajectoryPoint.joint7 = 0.0;

        angularCmdPublisher.publish(trajectoryPoint);

        //preempt action server
        smoothTrajectoryServer.setPreempted();
        ROS_INFO("Smooth trajectory server preempted by client");

        return;
      }

      //get time for trajectory
      t = ros::Time::now().toSec() - startTime;
      if (t > timePoints.at(timePoints.size() - 1))
      {
        //use final trajectory point as the goal to calculate error until the error
        //is small enough to be considered successful
        /*
        {
          boost::recursive_mutex::scoped_lock lock(api_mutex);
          GetAngularPosition(position_data);
        }
        */

        if (!reachedFinalPoint)
        {
          reachedFinalPoint = true;
          finalPointTime = ros::Time::now();
        }

        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          current_joint_pos[i] = jointStates.position[i];
        }

        totalError = 0;
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          currentPoint = simplify_angle(current_joint_pos[i]);
          error[i] = nearest_equivalent(simplify_angle((splines.at(i))(timePoints.at(timePoints.size() - 1))),
                                        currentPoint) - currentPoint;
          totalError += fabs(error[i]);
        }

        if (totalError < .035 || ros::Time::now() - finalPointTime >= ros::Duration(3.0))
        {
          //stop arm
          trajectoryPoint.joint1 = 0.0;
          trajectoryPoint.joint2 = 0.0;
          trajectoryPoint.joint3 = 0.0;
          trajectoryPoint.joint4 = 0.0;
          trajectoryPoint.joint5 = 0.0;
          trajectoryPoint.joint6 = 0.0;
          trajectoryPoint.joint7 = 0.0;
          angularCmdPublisher.publish(trajectoryPoint);
          trajectoryComplete = true;
          ROS_INFO("Trajectory complete!");
          break;
        }
      }
      else
      {
        //calculate error
        /*
        {
          boost::recursive_mutex::scoped_lock lock(api_mutex);
          GetAngularPosition(position_data);
        }
        */
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          current_joint_pos[i] = jointStates.position[i];
        }

        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          currentPoint = simplify_angle(current_joint_pos[i]);
          error[i] = nearest_equivalent(simplify_angle((splines.at(i))(t)), currentPoint) - currentPoint;
        }
      }

      //calculate control input
      //populate the velocity command
      trajectoryPoint.joint1 = (KP * error[0] + KV * (error[0] - prevError[0]) * RAD_TO_DEG);
      trajectoryPoint.joint2 = (KP * error[1] + KV * (error[1] - prevError[1]) * RAD_TO_DEG);
      trajectoryPoint.joint3 = (KP * error[2] + KV * (error[2] - prevError[2]) * RAD_TO_DEG);
      trajectoryPoint.joint4 = (KP * error[3] + KV * (error[3] - prevError[3]) * RAD_TO_DEG);
      trajectoryPoint.joint5 = (KP * error[4] + KV * (error[4] - prevError[4]) * RAD_TO_DEG);
      trajectoryPoint.joint6 = (KP * error[5] + KV * (error[5] - prevError[5]) * RAD_TO_DEG);
      trajectoryPoint.joint7 = (KP * error[6] + KV * (error[6] - prevError[6]) * RAD_TO_DEG);

      //for debugging:
      // cout << "Errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << endl;

      //send the velocity command
      angularCmdPublisher.publish(trajectoryPoint);

      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        prevError[i] = error[i];
      }

      rate.sleep();
      ros::spinOnce();
    }

    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smoothTrajectoryServer.setSucceeded(result);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_trajectory_controller");

  JacoTrajectoryController jtc;
  ros::spin();
}

