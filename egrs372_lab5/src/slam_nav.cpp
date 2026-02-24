#include <ros/ros.h>                    // Core ROS functionality
#include <geometry_msgs/PoseStamped.h> // Message type used to send navigation goals
#include <cmath>                       // Math functions (sin, cos)
#include <iostream>                    // Input/output (cin, cout)
#include <string>                      // String handling

#define PI 3.14159265359               // Define constant for pi (used for degree → rad conversion)

int main(int argc, char **argv)
{
  // Initialize ROS node
  // argc and argv allow ROS to process command line arguments
  ros::init(argc, argv, "slam_nav");

  // Create a NodeHandle object
  // This is the main access point for communication with the ROS system
  ros::NodeHandle nh;

  // Create a publisher that sends PoseStamped messages
  // Topic: /move_base_simple/goal
  // Queue size: 10 messages
  // This is the same topic used by RViz "2D Goal Pose"
  ros::Publisher goal_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  // Set loop rate to 10 Hz (10 times per second)
  // Only important because we are running inside a while loop
  ros::Rate loop_rate(10);

  // Small delay to allow publisher to connect properly
  ros::Duration(0.5).sleep();

  // Main loop runs continuously while ROS is active
  while (ros::ok())
  {
    // Variables to store user input
    int frame = 0;          // Frame selection (base_link or map)
    double xpos = 0.0;      // X position of goal
    double ypos = 0.0;      // Y position of goal
    double zrot_deg = 0.0;  // Rotation angle in degrees

    // ============================
    // GET REFERENCE FRAME INPUT
    // ============================
    while (true)
    {
      // Ask user which reference frame to use
      std::cout << "Reference Frame [1]: Local (base_link) [2]: Global (map): ";
      std::cin >> frame;

      // Check if input failed OR invalid option selected
      if (std::cin.fail() || (frame != 1 && frame != 2))
      {
        // Clear error flags
        std::cin.clear();

        // Remove invalid characters from input buffer
        std::cin.ignore(10000, '\n');

        std::cout << "Invalid input. Enter 1 or 2.\n";
      }
      else
      {
        // Valid input received → exit loop
        break;
      }
    }

    // ============================
    // GET X POSITION
    // ============================
    while (true)
    {
      std::cout << "X Pos: ";
      std::cin >> xpos;

      if (std::cin.fail())
      {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Invalid input.\n";
      }
      else
      {
        break;
      }
    }

    // ============================
    // GET Y POSITION
    // ============================
    while (true)
    {
      std::cout << "Y Pos: ";
      std::cin >> ypos;

      if (std::cin.fail())
      {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Invalid input.\n";
      }
      else
      {
        break;
      }
    }

    // ============================
    // GET ROTATION ANGLE
    // ============================
    while (true)
    {
      std::cout << "Rotation Around Z (degrees): ";
      std::cin >> zrot_deg;

      if (std::cin.fail())
      {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Invalid input.\n";
      }
      else
      {
        break;
      }
    }

    // ============================
    // CONVERT DEGREES TO RADIANS
    // ============================
    // ROS quaternions require radians, not degrees
    double yaw = zrot_deg * PI / 180;

    // ============================
    // CONVERT YAW TO QUATERNION
    // ============================
    // For rotation around Z axis only:
    //
    // qx = 0
    // qy = 0
    // qz = sin(yaw/2)
    // qw = cos(yaw/2)
    //
    // This represents orientation in 3D space
    double q1 = 0.0;                      // x component
    double q2 = 0.0;                      // y component
    double q3 = std::sin(yaw / 2.0);      // z component
    double q0 = std::cos(yaw / 2.0);      // w component

    // ============================
    // SET REFERENCE FRAME STRING
    // ============================
    std::string frame_id;

    if (frame == 1)
      frame_id = "base_link";   // Robot's local coordinate frame
    else
      frame_id = "map";         // Global coordinate frame

    // ============================
    // CREATE GOAL MESSAGE
    // ============================
    geometry_msgs::PoseStamped goal;

    // Timestamp when goal is created
    goal.header.stamp = ros::Time::now();

    // Reference coordinate frame
    goal.header.frame_id = frame_id;

    // Set goal position
    goal.pose.position.x = xpos;
    goal.pose.position.y = ypos;
    goal.pose.position.z = 0.0; // always 0 for ground robot

    // Set goal orientation (quaternion)
    goal.pose.orientation.x = q1;
    goal.pose.orientation.y = q2;
    goal.pose.orientation.z = q3;
    goal.pose.orientation.w = q0;

    // ============================
    // PUBLISH GOAL TO ROS
    // ============================
    goal_pub.publish(goal);

    // Print confirmation to user
    std::cout << "\nGoal published:\n";
    std::cout << "Frame: " << frame_id << "\n";
    std::cout << "X pos: " << xpos << "\n";
    std::cout << "Y pos: " << ypos << "\n";
    std::cout << "Yaw (deg): " << zrot_deg << "\n\n";

    // Process ROS callbacks (not strictly needed here, but good practice)
    ros::spinOnce();

    // Sleep to maintain loop rate
    loop_rate.sleep();
  }

  return 0;
}