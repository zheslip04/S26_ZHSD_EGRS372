//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>

//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"

void uint_function(const std_msgs::UInt64 uint)
{
  //declare the variables
  unsigned uint_output;
  //set the variable to the squared number
  uint_output = uint.data;

  //output the data
  std::cout << "The number " << uint_output << " was published" <<std::endl;
}

int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_Tutorial_Listener");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare subsrcriber "squarenum" is the name of the node
  //1 is how many to save in the buffer
  //uint_function is the function called when a value is recieved
  ros::Subscriber square = n.subscribe("squarenum", 1, uint_function);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {

    //looks for data
    ros::spin();
  }

  return 0;
}



