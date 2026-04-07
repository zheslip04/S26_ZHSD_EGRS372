#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int32.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalID.h>
#include <string>
#include <signal.h>
#include <cmath>

#include  "egrs_lab7/turtlebot_status.h"
#include  "egrs_lab7/go_home.h"
#include  "egrs_lab7/return_to_work.h"
#include  "egrs_lab7/update_count.h"

// =====================================================
// Globals
// =====================================================

ros::Publisher goal_pub;
ros::Publisher led_pub;
ros::Publisher cmd_pub;
ros::Publisher cancel_pub;
ros::Publisher status_pub = n.advertise<lab7::turtlebot_status>("TurtleBot_Status", 1);

bool bumper_pressed = false;
bool last_bumper_pressed = false;
bool battery_low = false;
bool goal_reached = false;
bool goal_sent = false;
bool update_count(lab7::update_count::Request &req, lab7::update_count::Response &res);

int place_count = 0;

// Keep battery part same as lab 7
const double LOW_BATTERY_VOLTAGE  = 10.0;
const double HIGH_BATTERY_VOLTAGE = 11.0;

// =====================================================
// Robot states
// =====================================================

enum RobotState
{
  GO_HOME_START,
  GO_PICK,
  WAIT_PICK,
  GO_PLACE,
  WAIT_PLACE,
  GO_HOME_CHARGE,
  CHARGING_WAIT
};

RobotState state = GO_HOME_START;
RobotState saved_state = GO_PICK;

// =====================================================
// Goal storage
// =====================================================

geometry_msgs::PoseStamped home_goal;
geometry_msgs::PoseStamped pick_goal;
geometry_msgs::PoseStamped place_goal;

// =====================================================
// Helper functions
// =====================================================

std::string stateToString(RobotState s)
{
  switch (s)
  {
    case GO_HOME_START:  return "GO_HOME_START";
    case GO_PICK:        return "GO_PICK";
    case WAIT_PICK:      return "WAIT_PICK";
    case GO_PLACE:       return "GO_PLACE";
    case WAIT_PLACE:     return "WAIT_PLACE";
    case GO_HOME_CHARGE: return "GO_HOME_CHARGE";
    case CHARGING_WAIT:  return "CHARGING_WAIT";
    default:             return "UNKNOWN";
  }
}

geometry_msgs::PoseStamped makeGoal(double x, double y, double z, double w)
{
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = z;
  goal.pose.orientation.w = w;
  return goal;
}

void sendGoal(const geometry_msgs::PoseStamped& goal, const std::string& name)
{
  geometry_msgs::PoseStamped msg = goal;
  msg.header.stamp = ros::Time::now();
  goal_pub.publish(msg);

  goal_sent = true;
  goal_reached = false;

  ROS_INFO("Sent goal: %s -> x=%.3f y=%.3f",
           name.c_str(),
           goal.pose.position.x,
           goal.pose.position.y);
}

void publishLEDCount()
{
  std_msgs::Int32 msg;
  msg.data = place_count;
  led_pub.publish(msg);
  ROS_INFO("Place count = %d", place_count);
}

void stopRobot()
{
  geometry_msgs::Twist stop_msg;
  stop_msg.linear.x = 0.0;
  stop_msg.linear.y = 0.0;
  stop_msg.linear.z = 0.0;
  stop_msg.angular.x = 0.0;
  stop_msg.angular.y = 0.0;
  stop_msg.angular.z = 0.0;

  for (int i = 0; i < 5; i++)
  {
    cmd_pub.publish(stop_msg);
    ros::Duration(0.05).sleep();
  }
}

void cancelMoveBaseGoal()
{
  actionlib_msgs::GoalID cancel_msg;
  cancel_pub.publish(cancel_msg);   // empty GoalID cancels all goals
  ros::Duration(0.1).sleep();
}

void sigintHandler(int sig)
{
  ROS_WARN("Shutting down lab7 node...");
  cancelMoveBaseGoal();
  stopRobot();
  ros::shutdown();
}

RobotState normalizeResumeState(RobotState s)
{
  if (s == WAIT_PICK)
    return GO_PICK;
  if (s == WAIT_PLACE)
    return GO_PLACE;

  return s;
}

// =====================================================
// NEW: Update goals from parameter server while running
// =====================================================

bool updateGoalsFromParams()
{
  static double last_home_x = 0.0,  last_home_y = 0.0,  last_home_theta = 0.0;
  static double last_pick_x = 0.0,  last_pick_y = 0.0,  last_pick_theta = 0.0;
  static double last_place_x = 0.0, last_place_y = 0.0, last_place_theta = 0.0;
  static bool first_time = true;

  double home_x, home_y, home_theta;
  double pick_x, pick_y, pick_theta;
  double place_x, place_y, place_theta;

  ros::param::get("/home_location/x", home_x);
  ros::param::get("/home_location/y", home_y);
  ros::param::get("/home_location/theta", home_theta);

  ros::param::get("/pick_location/x", pick_x);
  ros::param::get("/pick_location/y", pick_y);
  ros::param::get("/pick_location/theta", pick_theta);

  ros::param::get("/place_location/x", place_x);
  ros::param::get("/place_location/y", place_y);
  ros::param::get("/place_location/theta", place_theta);

  bool changed =
      first_time ||
      home_x != last_home_x || home_y != last_home_y || home_theta != last_home_theta ||
      pick_x != last_pick_x || pick_y != last_pick_y || pick_theta != last_pick_theta ||
      place_x != last_place_x || place_y != last_place_y || place_theta != last_place_theta;

  if (changed)
  {
    home_goal  = makeGoal(home_x,  home_y,  sin(home_theta / 2.0),  cos(home_theta / 2.0));
    pick_goal  = makeGoal(pick_x,  pick_y,  sin(pick_theta / 2.0),  cos(pick_theta / 2.0));
    place_goal = makeGoal(place_x, place_y, sin(place_theta / 2.0), cos(place_theta / 2.0));

    last_home_x = home_x;
    last_home_y = home_y;
    last_home_theta = home_theta;

    last_pick_x = pick_x;
    last_pick_y = pick_y;
    last_pick_theta = pick_theta;

    last_place_x = place_x;
    last_place_y = place_y;
    last_place_theta = place_theta;

    first_time = false;

    ROS_INFO("Updated params:");
    ROS_INFO("  HOME  -> x=%.3f y=%.3f theta=%.3f", home_x, home_y, home_theta);
    ROS_INFO("  PICK  -> x=%.3f y=%.3f theta=%.3f", pick_x, pick_y, pick_theta);
    ROS_INFO("  PLACE -> x=%.3f y=%.3f theta=%.3f", place_x, place_y, place_theta);
  }

  return changed;
}

// =====================================================
// Callbacks
// =====================================================

void bumperCallback(const std_msgs::Byte::ConstPtr& msg)
{
  bumper_pressed = (msg->data != 0);
}

void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  double voltage = msg->voltage;

  if (voltage < LOW_BATTERY_VOLTAGE)
  {
    if (!battery_low)
      ROS_WARN("Battery LOW: %.2f V", voltage);

    battery_low = true;
  }
  else if (voltage > HIGH_BATTERY_VOLTAGE)
  {
    if (battery_low)
      ROS_INFO("Battery recovered: %.2f V", voltage);

    battery_low = false;
  }
}

void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  if (msg->status.status == 3)
  {
    goal_reached = true;
    ROS_INFO("Goal reached successfully.");
  }
}

//function for the update count service
//the service requests a new count for the LED
//the service responds with the old LED count
bool update_count(lab7::update_count::Request  &req, lab7::update_count::Response &res)
{
  //creates a new Node handler to publish the data inside this service
  ros::NodeHandle m;

  //creates the publisher for the LED data
  ros::Publisher LED_pub = m.advertise<std_msgs::Int32>("LED_data", 1);

  //creates the message variable
  std_msgs::Int32 LED_update;

  //sets the response to the old number of places
  res.old_count = places;

  //updates the places with the request
  places = req.new_count;

  //updates the message variable with the new number
  LED_update.data=places;

  //publishes the new LED data
  LED_pub.publish(LED_update);

  return true;
}

// =====================================================
// Main
// =====================================================

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lab7");
  ros::NodeHandle nh;

  signal(SIGINT, sigintHandler);

  // Publishers
  goal_pub   = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  led_pub    = nh.advertise<std_msgs::Int32>("led_data", 10);
  cmd_pub    = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  cancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

  // Subscribers
  ros::Subscriber bumper_sub  = nh.subscribe("bumper", 10, bumperCallback);
  ros::Subscriber battery_sub = nh.subscribe("battery_state", 10, batteryCallback);
  ros::Subscriber result_sub  = nh.subscribe("/move_base/result", 10, resultCallback);
  
  // Services
  ros::ServiceServer count_service = n.advertiseService("update_count", update_count); 
  

  ros::Duration(3.0).sleep();

  // Read params once at startup
  updateGoalsFromParams();

  publishLEDCount();

  ros::Rate loop_rate(10);

  ROS_INFO("lab7 node started.");
  ROS_INFO("Initial state -> %s", stateToString(state).c_str());

  while (ros::ok())
  {
    ros::spinOnce();

    bool new_bump = (bumper_pressed && !last_bumper_pressed);

    // Re-read params while running
    bool goals_changed = updateGoalsFromParams();

    // If currently moving and a parameter changed, resend the active goal
    if (goals_changed)
    {
      if (state == GO_HOME_START || state == GO_PICK || state == GO_PLACE || state == GO_HOME_CHARGE)
      {
        cancelMoveBaseGoal();
        goal_sent = false;
        goal_reached = false;
        ROS_INFO("Active goal canceled so updated parameter goal can be resent.");
      }
    }

    // Battery interrupt
    if (battery_low && state != GO_HOME_CHARGE && state != CHARGING_WAIT)
    {
      ROS_WARN("Battery low -> switching to GO_HOME_CHARGE");

      saved_state = state;
      state = GO_HOME_CHARGE;
      goal_sent = false;
      goal_reached = false;
    }

    switch (state)
    {
      case GO_HOME_START:
      {
        if (!goal_sent)
          sendGoal(home_goal, "HOME");

        if (goal_reached)
        {
          goal_reached = false;
          goal_sent = false;
          state = GO_PICK;
          ROS_INFO("State -> %s", stateToString(state).c_str());
        }
        break;
      }

      case GO_PICK:
      {
        if (!goal_sent)
          sendGoal(pick_goal, "PICK");

        if (goal_reached)
        {
          goal_reached = false;
          goal_sent = false;
          state = WAIT_PICK;
          ROS_INFO("State -> %s", stateToString(state).c_str());
        }
        break;
      }

      case WAIT_PICK:
      {
        if (new_bump)
        {
          ROS_INFO("Pick confirmed by button.");
          state = GO_PLACE;
          goal_sent = false;
          goal_reached = false;
          ROS_INFO("State -> %s", stateToString(state).c_str());
        }
        break;
      }

      case GO_PLACE:
      {
        if (!goal_sent)
          sendGoal(place_goal, "PLACE");

        if (goal_reached)
        {
          goal_reached = false;
          goal_sent = false;
          state = WAIT_PLACE;
          ROS_INFO("State -> %s", stateToString(state).c_str());
        }
        break;
      }

      case WAIT_PLACE:
      {
        if (new_bump)
        {
          ROS_INFO("Place confirmed by button.");
		  
		  bool update_count();
          //place_count++;
          publishLEDCount();

          state = GO_PICK;
          goal_sent = false;
          goal_reached = false;
          ROS_INFO("State -> %s", stateToString(state).c_str());
        }
        break;
      }

      case GO_HOME_CHARGE:
      {
        if (!goal_sent)
          sendGoal(home_goal, "HOME (CHARGING)");

        if (goal_reached)
        {
          goal_reached = false;
          goal_sent = false;
          state = CHARGING_WAIT;
          ROS_INFO("Reached HOME for charging. State -> %s", stateToString(state).c_str());
        }
        break;
      }

      case CHARGING_WAIT:
      {
        if (!battery_low)
        {
          RobotState resume_state = normalizeResumeState(saved_state);

          ROS_INFO("Battery OK -> resuming previous task.");
          state = resume_state;
          goal_sent = false;
          goal_reached = false;
          ROS_INFO("State -> %s", stateToString(state).c_str());
        }
        break;
      }

      default:
        break;
    }

    last_bumper_pressed = bumper_pressed;
    loop_rate.sleep();
  }

  cancelMoveBaseGoal();
  stopRobot();
  return 0;
}
