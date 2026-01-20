//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <string>

//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/String.h"


int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "lab2_string_input");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare variables
  char string_input[100];
  int string_L;
  //unsigned output;
  std_msgs::String char_pub;

  //declare publisher "squarenum" is the name of the node
  //1 is the number of values to keep stored until they are overwritten
  ros::Publisher square = n.advertise<std_msgs::String>("lab2_string", 100);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
    //clear the input buffer
    std::fflush; //the two “f's” are correct, not a typo

    //prompt the user for an input
    std::cout << "Enter the message: ";
    //get the input from the user
    std::cin.getline(string_input, 100);

    

    //string length
    string_L = strlen(string_input);

     //confirm the number is being sent
    std::cout << "Sending message: " << string_input << std::endl;

    //set the message value
    char_pub.data=string_input;
    //publish the data
    square.publish(char_pub);
    

    //sends out any data necessary then waits based on the loop rate
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}




