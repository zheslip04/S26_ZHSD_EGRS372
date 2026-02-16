// FILENAME:
// draw_poly.cpp
#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "tf/tfMessage.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#define INITIALIZE_VALUE -1
#define PI 3.14159265359

// ---------------- GLOBAL VARIABLES ----------------
bool flag;
double current_angle = INITIALIZE_VALUE;
double target_speed = 0.0;
double target_forward = 0.5;
double moved = 0;
double initialx = INITIALIZE_VALUE;
double initialy = INITIALIZE_VALUE;

// ODOMETRY DATA (circle mode)
double current_x = 0.0;
double current_y = 0.0;
double current_yaw = 0.0;

// POLYGON CONTROLS
int g_sides = 4;
double g_side_length = 0.5;
double g_step_angle = PI / 2;

// ---------------- ODOM CALLBACK ----------------
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw = yaw;
}

// ---------------- FORWARD CALLBACK ----------------
void forwardprog(const tf::tfMessage& cvalue)
{
    double dx, dy;

    if (initialx == INITIALIZE_VALUE || initialy == INITIALIZE_VALUE)
    {
        initialx = cvalue.transforms[0].transform.translation.x;
        initialy = cvalue.transforms[0].transform.translation.y;
    }

    dx = cvalue.transforms[0].transform.translation.x - initialx;
    dy = cvalue.transforms[0].transform.translation.y - initialy;

    moved = sqrt(dx * dx + dy * dy);

    if (moved >= target_forward)
        flag = false;

    target_speed = fabs(target_forward - moved) / 4 + 0.1;
}

// ---------------- TURN CALLBACK ----------------
void Turnprog(const tf::tfMessage& cvalue)
{
    double z = cvalue.transforms[0].transform.rotation.z;
    double w = cvalue.transforms[0].transform.rotation.w;

    current_angle = 2 * atan2(z, w);
    if (current_angle < 0) current_angle += 2 * PI;
    if (current_angle >= 2 * PI) current_angle -= 2 * PI;
}

// ---------------- ANGLE WRAP ----------------
double wrapToPi(double angle)
{
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_poly");
    ros::NodeHandle n;

    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber tf;
    ros::Subscriber odom_sub = n.subscribe("/odom", 10, odomCallback);

    ros::Rate loop_rate(1000);

    // ---------------- USER INPUT ----------------
    char input = 0;
    std::cout << "Type 'y' and enter to begin: ";
    while (input != 'y' && ros::ok()) std::cin >> input;

    while (true)
    {
        std::cout << "Enter number of sides (integer > 1): ";
        std::cin >> g_sides;
        if (std::cin.fail() || g_sides <= 1)
        {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input.\n";
        }
        else break;
    }

    while (true)
    {
        std::cout << "Enter side length in meters (0.1 to 1.0): ";
        std::cin >> g_side_length;
        if (std::cin.fail() || g_side_length < 0.1 || g_side_length > 1.0)
        {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input.\n";
        }
        else break;
    }

    target_forward = g_side_length;
    g_step_angle = (2.0 * PI) / g_sides;

    geometry_msgs::Twist c;
    c.linear.x = 0;
    c.angular.z = 0;

    // ================= CIRCLE MODE (FIXED) =================
    if (g_sides >= 10)
    {
        std::cout << "Circle mode activated\n";

        double v = 0.2;
        double r = g_side_length;
        double w = v / r;

        c.linear.x = v;
        c.angular.z = -w;

        ros::spinOnce();
        double start_x = current_x;
        double start_y = current_y;
        double start_yaw = current_yaw;
        double prev_yaw = current_yaw;
        double accumulated_yaw = 0.0;

        while (ros::ok())
        {
            vel_pub.publish(c);
            ros::spinOnce();

            double dyaw = wrapToPi(current_yaw - prev_yaw);
            accumulated_yaw += fabs(dyaw);
            prev_yaw = current_yaw;

            double dist_to_start = sqrt(
                pow(current_x - start_x, 2) +
                pow(current_y - start_y, 2)
            );

            if (accumulated_yaw >= (2.0 * PI - 0.05) &&
                dist_to_start < 0.05)
            {
                break;
            }

            loop_rate.sleep();
        }

        c.linear.x = 0;
        c.angular.z = 0;
        vel_pub.publish(c);
        return 0;
    }

    // ================= POLYGON MODE (UNCHANGED) =================
    for (int i = 0; i < g_sides; i++)
    {
        flag = true;
        moved = 0;
        initialx = INITIALIZE_VALUE;
        initialy = INITIALIZE_VALUE;

        tf = n.subscribe("/tf", 1, forwardprog);
        ros::spinOnce();
        loop_rate.sleep();

        while (flag && ros::ok())
        {
            c.linear.x = target_speed;
            c.angular.z = 0;
            vel_pub.publish(c);
            ros::spinOnce();
            loop_rate.sleep();
        }
        tf.shutdown();

        ros::Time go = ros::Time::now();
        while ((ros::Time::now() - go) < ros::Duration(0.1) && ros::ok())
        {
            c.linear.x = 0;
            c.angular.z = 0;
            vel_pub.publish(c);
            ros::spinOnce();
            loop_rate.sleep();
        }

        tf = n.subscribe("/tf", 1, Turnprog);
        current_angle = INITIALIZE_VALUE;

        while (current_angle == INITIALIZE_VALUE && ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }

        double target_angle = current_angle - g_step_angle;
        if (target_angle < 0) target_angle += 2 * PI;

        while (ros::ok())
        {
            double diff = wrapToPi(target_angle - current_angle);
            if (fabs(diff) < 0.05) break;

            c.linear.x = 0;
            c.angular.z = -fabs(diff) / PI - 0.5;
            vel_pub.publish(c);

            ros::spinOnce();
            loop_rate.sleep();
        }

        tf.shutdown();
    }

    c.linear.x = 0;
    c.angular.z = 0;
    vel_pub.publish(c);
    return 0;
}

