#include <kinova_driver/hlpr_trajectory_controller.h>

// using namespace std;

JacoTrajectoryController::JacoTrajectoryController() : pnh("~"),
  smoothTrajectoryServer(pnh, "trajectory", boost::bind(&JacoTrajectoryController::executeSmoothTrajectory, this, _1), false),
  jointNames()
{
  pnh.param("max_curvature", maxCurvature, 100.0);
  pnh.param("sim", sim_flag_, false);

  // jointNames.clear();
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
    ROS_INFO("Using real Kinova arm.");
    ROS_WARN("This is Adam's version. This is a test.");
    // Connect to the low-level angular driver from kinova-ros
    angularCmdPublisher = n.advertise<kinova_msgs::JointVelocity>("j2s7s300_driver/in/joint_velocity", 1);
  }
  else
  {
    ROS_INFO("Using simulated Kinova arm.");

    // Setup a fake gravity comp service (torque control)
    start_gravity_comp_ = n.advertiseService(
                "j2s7s300_driver/in/start_gravity_comp", &JacoTrajectoryController::startGravityCompService, this);
    stop_gravity_comp_ = n.advertiseService(
                "j2s7s300_driver/in/stop_gravity_comp", &JacoTrajectoryController::stopGravityCompService, this);

    // Setup a fake admittance service (Force control)
    start_force_control_service_ = n.advertiseService("j2s7s300_driver/in/start_force_control", &JacoTrajectoryController::startForceControlCallback, this);
    stop_force_control_service_ = n.advertiseService("j2s7s300_driver/in/stop_force_control", &JacoTrajectoryController::stopForceControlCallback, this);

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

bool JacoTrajectoryController::startForceControlCallback(kinova_msgs::Start::Request &req, kinova_msgs::Start::Response &res)
{
    ROS_INFO("Simulation 'enabled' admittance mode. Note: nothing actually happens");
    res.start_result = "Start force control requested.";
    return true;
}

bool JacoTrajectoryController::stopForceControlCallback(kinova_msgs::Stop::Request &req, kinova_msgs::Stop::Response &res)
{
    ROS_INFO("Simulation 'disabled' admittance mode. Note: nothing actually happens");
    res.stop_result = "Stop force control requested.";
    return true;
}

void JacoTrajectoryController::jointStateCallback(const sensor_msgs::JointState &msg)
{
  // if(msg.name.size() != NUM_JACO_JOINTS) {
  //   ROS_WARN_STREAM("Joint state had " << msg.name.size() << " joints, but the arm has " << NUM_JACO_JOINTS << " joints.");
  // }

  // Get the names of all the joints
  // std::vector<std::string> pub_joint_names = msg.name;

  // Create a new message
  sensor_msgs::JointState arm_msg;
  // std::vector<double> position, velocity, effort;
  // std::vector<std::string> names;
  arm_msg.position.resize(NUM_JACO_JOINTS);
  arm_msg.velocity.resize(NUM_JACO_JOINTS);
  arm_msg.effort.resize(NUM_JACO_JOINTS);
  arm_msg.name.resize(NUM_JACO_JOINTS);

  // Cycle through the number of JACO joints
  for (int joint_id = 0; joint_id < NUM_JACO_JOINTS; ++joint_id){
    // Find the location of the joint
    std::string joint_name = jointNames[joint_id];
    int msg_loc = std::distance(msg.name.begin(), std::find(msg.name.begin(), msg.name.end(), joint_name));

    // The warning printed if the following check fails is quite verbose.
    // Effort information takes a while to get assigned to the messages (maybe a calibration
    // or filtering waiting period?, so we use _DELAYED, but only for the error related to effort,
    // to avoid a huge number of printouts as the program starts.
    // 
    // First check name, position, and velocity, which always seem to work ("shouldn't" fail)
    if(msg_loc >= msg.name.size() || joint_id >= arm_msg.name.size() ||
       msg_loc >= msg.position.size() || joint_id >= arm_msg.position.size() ||
       msg_loc >= msg.velocity.size() || joint_id >= arm_msg.velocity.size()) {
      ROS_WARN_STREAM("At least one of the Kinova joint indices was out of bounds when attempting " <<
                      "to assign joint states. Error encountered at joint " << joint_name << "(#" <<
                      joint_id << "), but there may be additional errors. For now, aborting the " <<
                      "jointStateCallback.");
      break;
    }
    else {
      arm_msg.name[joint_id] = msg.name[msg_loc];
      arm_msg.position[joint_id] = msg.position[msg_loc];
      arm_msg.velocity[joint_id] = msg.velocity[msg_loc];
    }
    // Now check the efforts separately
    if(msg_loc >= msg.effort.size() || joint_id >= arm_msg.effort.size()) {
      // Do a delayed printout, so we only show this if the error is persistent
      ROS_WARN_STREAM_DELAYED_THROTTLE(5, "Effort information is missing for at least one Kinova joint.");
    }
    else {
      arm_msg.effort[joint_id] = msg.effort[msg_loc];
    }
  }

  jointStates = arm_msg;
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

void JacoTrajectoryController::stopMotion() {
  kinova_msgs::JointVelocity cmdVel;
  cmdVel.joint1 = 0.0;
  cmdVel.joint2 = 0.0;
  cmdVel.joint3 = 0.0;
  cmdVel.joint4 = 0.0;
  cmdVel.joint5 = 0.0;
  cmdVel.joint6 = 0.0;
  cmdVel.joint7 = 0.0;

  angularCmdPublisher.publish(cmdVel);
}

void JacoTrajectoryController::executeSmoothTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  int numPoints = goal->trajectory.points.size();
  int numJoints = goal->trajectory.joint_names.size();

  assert(numPoints > 0);
  assert(numJoints == NUM_JACO_JOINTS);

  //initialize arrays needed to fit a smooth trajectory to the given points
  std::vector<double> timePoints(numPoints);
  float velocityPoints[NUM_JACO_JOINTS][numPoints];
  float positionPoints[NUM_JACO_JOINTS][numPoints];

  //get trajectory data
  for (unsigned int i = 0; i < numPoints; i++)
  {
    timePoints[i] = goal->trajectory.points[i].time_from_start.toSec();
    for (int unsortedJointIndex = 0; unsortedJointIndex < numJoints; ++unsortedJointIndex)
    {
      std::string jointName = goal->trajectory.joint_names[unsortedJointIndex];
      int jointIndex = std::distance(jointNames.begin(), std::find(jointNames.begin(), jointNames.end(), jointName));
      if (jointIndex >= 0 && jointIndex < NUM_JACO_JOINTS)
      {
        velocityPoints[jointIndex][i] = goal->trajectory.points.at(i).velocities.at(unsortedJointIndex);
        positionPoints[jointIndex][i] = goal->trajectory.points.at(i).positions.at(unsortedJointIndex);
      }
    }
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
    ros::Duration(0.1).sleep();

    // Check the total error to determine if the trajectory finished
    totalError = 1.0;
    float prevError = 0.0;
    bool jointError = true;
    while (abs(totalError - prevError) > 0.001 && jointError)
    {
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
          jointError = jointError || error[i] > ERROR_THRESHOLD;
        }

    // Rate to check if error has changed
    ros::Duration(0.1).sleep();
    }

    // Tell the server we finished the trajectory
    ROS_INFO("Trajectory Control Complete.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smoothTrajectoryServer.setSucceeded(result);

  }
  else
  {
    ROS_INFO_STREAM("there are " << timePoints.size() << " points in the trajectory.");

    kinova_msgs::JointVelocity cmdVel;
    // Sending to the real robot
    while (!trajectoryComplete)
    {
      //check for preempt requests from clients
      if (smoothTrajectoryServer.isPreemptRequested())
      {
        stopMotion();
        smoothTrajectoryServer.setPreempted();
        ROS_INFO("Smooth trajectory server preempted by client");

        return;
      }

      //get time for trajectory
      t = ros::Time::now().toSec() - startTime;
      if (t > timePoints.at(timePoints.size() - 1))
      {
        if (!reachedFinalPoint)
        {
          reachedFinalPoint = true;
          finalPointTime = ros::Time::now();
        }
        ROS_INFO_STREAM("Past the end: " << t << " vs. " << timePoints.at(timePoints.size() - 1));
        //use final trajectory point as the goal to calculate error until the error
        //is small enough to be considered successful
        /*
        {
          boost::recursive_mutex::scoped_lock lock(api_mutex);
          GetAngularPosition(position_data);
        }
        */

        bool jointError = false;
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          current_joint_pos[i] = jointStates.position[i];
          currentPoint = simplify_angle(current_joint_pos[i]);

          size_t last_idx = timePoints.size() - 1;
          double setpoint = 0;

          kinova_msgs::JointVelocity cmdVel;
          setpoint = simplify_angle(positionPoints[i][last_idx]);
          setpoint = nearest_equivalent(setpoint, currentPoint);
          error[i] = setpoint - currentPoint;
          ROS_INFO_STREAM("setpoint: " << setpoint << ", current point: " << currentPoint << ", error: " << error[i]);

          jointError = jointError || fabs(error[i]) > ERROR_THRESHOLD;
        }
        cmdVel.joint1 = error[0] * RAD_TO_DEG;
        cmdVel.joint2 = error[1] * RAD_TO_DEG;
        cmdVel.joint3 = error[2] * RAD_TO_DEG;
        cmdVel.joint4 = error[3] * RAD_TO_DEG;
        cmdVel.joint5 = error[4] * RAD_TO_DEG;
        cmdVel.joint6 = error[5] * RAD_TO_DEG;
        cmdVel.joint7 = error[6] * RAD_TO_DEG;
        ROS_INFO_STREAM("joint position errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5]);

        if (!jointError || ros::Time::now() - finalPointTime >= ros::Duration(5.0))
        // if (!jointError)
        {
          // ROS_INFO_STREAM("Final joint position errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5]);
          stopMotion();
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

          double t_point = -1;
          double last_t_point = -1;
          double interpVal = -1;
          for (size_t t_idx = 1; t_idx < timePoints.size(); ++t_idx)
          {
            t_point = timePoints[t_idx];
            last_t_point = timePoints[t_idx-1];
            if (t > last_t_point && t < t_point)  // found our interp values
            {
              interpVal = velocityPoints[i][t_idx-1] + (t-last_t_point)/(t_point-last_t_point) * ((velocityPoints[i][t_idx] - velocityPoints[i][t_idx-1]));
              break;
            }
          }
          if(t > t_point) 
          {
            interpVal = velocityPoints[i][timePoints.size() - 1];
          }
          else if(interpVal == -1)
          {
            ROS_INFO_STREAM("Attempting to lookup interp value for t=" << t << ", in range " << timePoints[0] << " to " << timePoints[timePoints.size() - 1]);
          }

          // double interpVal = velocityPoints[i][timePoints.size() - 1];
          error[i] = interpVal;
        }
        cmdVel.joint1 = error[0] * RAD_TO_DEG;
        cmdVel.joint2 = error[1] * RAD_TO_DEG;
        cmdVel.joint3 = error[2] * RAD_TO_DEG;
        cmdVel.joint4 = error[3] * RAD_TO_DEG;
        cmdVel.joint5 = error[4] * RAD_TO_DEG;
        cmdVel.joint6 = error[5] * RAD_TO_DEG;
        cmdVel.joint7 = error[6] * RAD_TO_DEG;
      }
      ROS_INFO_STREAM("current velocity: " << error);

      //calculate control input
      //populate the velocity command

      //send the velocity command
      angularCmdPublisher.publish(cmdVel);

      //for debugging:
      // ROS_INFO_STREAM("Final joint position errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5]);

      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        prevError[i] = error[i];
      }

      rate.sleep();
    }

    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smoothTrajectoryServer.setSucceeded(result);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_trajectory_controller");

  ros::AsyncSpinner spinner(3);
  spinner.start();
  JacoTrajectoryController jtc;
  ros::waitForShutdown();
}
