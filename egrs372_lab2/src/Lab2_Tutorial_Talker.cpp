//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include "math.h"
#include <iostream>

//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"


int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_Tutorial_Talker");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare variables
  int integer_input;
  unsigned uint_output;
  std_msgs::UInt64 uint_pub;

  //declare publisher "squarenum" is the name of the node
  //1 is the number of values to keep stored until they are overwritten
  ros::Publisher square = n.advertise<std_msgs::UInt64>("squarenum", 1);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
    //clear the input buffer
    std::fflush; //the two “f's” are correct, not a typo

    //prompt the user for an input
    std::cout << "Enter the number to be squared: ";
    //get the input from the user
    std::cin >> integer_input;

    //square the value
    uint_output = (unsigned)pow(integer_input,2);
     //confirm the number is being sent
    std::cout << "Sending the number " << uint_output << std::endl;

    //set the message value
    uint_pub.data=uint_output;
    //publish the data
    square.publish(uint_pub);
    

    //sends out any data necessary then waits based on the loop rate
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}





