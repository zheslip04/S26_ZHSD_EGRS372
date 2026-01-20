//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <string>

//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/String.h"

void uint_function(const std_msgs::String string)
{
  //declare the variables
  //char string_input[100] = string;
  //set the variable to the squared number
  int string_size = strlen(string.data.c_str());

  //output the data
  std::cout << "The message " << string << " was published The message is " << string_size << " charaters long" <<std::endl;
  //std::cout << "The message is" << string_size << " charaters long" <<std::endl;
}

int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "lab2_string_output");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare subsrcriber "squarenum" is the name of the node
  //1 is how many to save in the buffer
  //uint_function is the function called when a value is recieved
  ros::Subscriber square = n.subscribe("lab2_string", 100, uint_function);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {

    //looks for data
    ros::spin();
  }

  return 0;
}



