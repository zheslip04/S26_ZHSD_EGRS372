#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <cmath>
#include <csignal>

#define PI 3.14159265359

// =====================================================
// Global variables
// =====================================================
ros::Publisher cmd_pub;
ros::Publisher led_pub;

bool bumper_pressed = false;
bool last_bumper_pressed = false;

double current_yaw = 0.0;
int hit_count = 0;

enum RobotState
{
  MOVE_FORWARD,
  BACK_UP,
  TURN_90
};

RobotState state = MOVE_FORWARD;
ros::Time state_start_time;

// Turn control
double turn_start_yaw = 0.0;
double target_yaw = 0.0;
double target_turn = PI / 2.0;

// Motion parameters
double forward_speed = 0.08;
double backup_speed = -0.08;
double backup_duration = 2.5;

// Turn tuning
double max_turn_speed = 0.7;      // fastest allowed turning speed
double min_turn_speed = 0.12;     // prevents stall
double turn_kp = 1.4;             // proportional gain
double turn_tolerance = 2.0 * PI / 180.0;   // 2 degrees tolerance

// =====================================================
// Helper functions
// =====================================================
double normalizeAngle(double angle)
{
  while (angle > PI)
    angle -= 2.0 * PI;
  while (angle < -PI)
    angle += 2.0 * PI;
  return angle;
}

void publishVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd;
  cmd.linear.x = linear;
  cmd.angular.z = angular;
  cmd_pub.publish(cmd);
}

void publishLEDCount()
{
  std_msgs::Int32 msg;
  msg.data = hit_count;
  led_pub.publish(msg);
}

void stopRobot()
{
  for (int i = 0; i < 5; i++)
  {
    publishVelocity(0.0, 0.0);
    ros::Duration(0.02).sleep();
  }
}

// Optional: handle Ctrl+C more cleanly
void sigintHandler(int)
{
  ROS_WARN("Ctrl+C detected. Stopping robot before shutdown...");
  stopRobot();
  ros::shutdown();
}

// =====================================================
// Callback functions
// =====================================================
void bumperCallback(const std_msgs::Byte::ConstPtr& msg)
{
  bumper_pressed = ((msg->data & 0x01) != 0);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_yaw = tf::getYaw(msg->pose.pose.orientation);
}

// =====================================================
// Main
// =====================================================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_bumper_hex");
  ros::NodeHandle nh;

  signal(SIGINT, sigintHandler);

  cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  led_pub = nh.advertise<std_msgs::Int32>("/led_data", 10);

  ros::Subscriber bumper_sub = nh.subscribe("/bumper", 10, bumperCallback);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

  ros::Rate loop_rate(50);   // faster loop helps reduce overshoot
  ros::Duration(0.5).sleep();

  hit_count = 0;
  publishLEDCount();

  state = MOVE_FORWARD;
  state_start_time = ros::Time::now();

  ROS_INFO("wall_bumper_hex node started.");

  while (ros::ok())
  {
    ros::spinOnce();

    bool new_bump = (bumper_pressed && !last_bumper_pressed);

    if (state == MOVE_FORWARD)
    {
      publishVelocity(forward_speed, 0.0);

      if (new_bump)
      {
        hit_count++;
        publishLEDCount();
        ROS_INFO("Wall hit count = %d", hit_count);

        stopRobot();
        state = BACK_UP;
        state_start_time = ros::Time::now();
      }
    }
    else if (state == BACK_UP)
    {
      double elapsed = (ros::Time::now() - state_start_time).toSec();

      if (elapsed < backup_duration)
      {
        publishVelocity(backup_speed, 0.0);
      }
      else
      {
        stopRobot();

        turn_start_yaw = current_yaw;
        target_yaw = normalizeAngle(turn_start_yaw + target_turn);

        state = TURN_90;
        state_start_time = ros::Time::now();
      }
    }
    else if (state == TURN_90)
    {
      // Error between where we want to be and where we are now
      double error = normalizeAngle(target_yaw - current_yaw);

      if (std::fabs(error) > turn_tolerance)
      {
        // Proportional turning: slow down as we approach target
        double cmd_w = turn_kp * error;

        // Limit max speed
        if (cmd_w > max_turn_speed) cmd_w = max_turn_speed;
        if (cmd_w < -max_turn_speed) cmd_w = -max_turn_speed;

        // Enforce minimum useful speed so robot does not stall
        if (std::fabs(cmd_w) < min_turn_speed)
        {
          cmd_w = (cmd_w >= 0.0) ? min_turn_speed : -min_turn_speed;
        }

        publishVelocity(0.0, cmd_w);
      }
      else
      {
        stopRobot();
        ROS_INFO("90 degree turn complete.");

        state = MOVE_FORWARD;
        state_start_time = ros::Time::now();
      }
    }

    last_bumper_pressed = bumper_pressed;
    loop_rate.sleep();
  }

  ROS_WARN("ROS shutdown detected. Sending stop commands...");
  stopRobot();

  return 0;
}