// FILENAME:
// draw_square.cpp
/* **********************************************/
/*                                              */
/*   Author:      Preston Fairchild, Team MARLA */
/*   Date:        11-14-19                      */
/*   Project:     lab #3 for EGRS 372           */
/*   Purpose:     Program mobile robot to       */
/*                travel in a square            */
/*                (TurtleBot3, running ROS)     */
/*                                              */
/* **********************************************/

// INCLUDE/DEFINE NECESSARY CONSTANTS / LIBRARIES
// -----------------------------------------------
#include "ros/ros.h"             // INCLUDE ROS (ROBOT OPERATING SYSTEM) 
#include <sstream>               // LIBRARY THAT PROVIDES STRING CLASSES
#include <iostream> 			 // FOR INPUTS AND OUTPUTS FOR CIN (INPUT)
#include "math.h"   			 // FOR ABSOLUTE VALUE AND TRIG FUNCTIONS
#include "geometry_msgs/Twist.h" // PUBLISHING A GEOETRY_MSGS/...
// 							     // TWIST AS THE SPEED OF THE ROBOT
#include "tf/tfMessage.h"  		 // SUBSCRIBING TO TF/TFMESSAGE ...
// 						   	 	 // AS THE TRANSFORMATION MATRICES OF THE ROBOT
#define INITIALIZE_VALUE -1      // VARIABLES INITIALLY HOLD VALUE OF (-1)
#define PI 3.14159265359         // RATIO OF CIRCLE, CIRCUMFERENCE:DIAMETER

// ...

// DEFINING GLOBAL VARIABLES
// ------------------------- 
bool flag; 							  // 'FLAG' = FALSE WHEN CONDITIONS ARE MET
double current_angle=0; 			  // CURRENT ANGLE OF THE ROBOT
double target_speed=0.0; 			  // ROBOT TRAVELLING SPEED (METERS/SECOND)
double target_angle=INITIALIZE_VALUE; // DESIRED ANGLE OF ROBOT PATH (THETA)
double target_forward=0.5;            // DISTANCE THE ROBOT IS TRYING TO MOVE
double moved=0; 					  // DISTANCE TRAVELLED (FORWARD, IN METERS) 
double initialx=INITIALIZE_VALUE;     // INITIAL X COORDINATE OF A MOVE
double initialy=INITIALIZE_VALUE;     // INITIAL Y COORDINATE OF A MOVE



// THIS FUNCTION HANDLES FORWARD MOVEMENT OF THE ROBOT... ----------------------
// WILL CHECK TO SEE IF THE DISTANCE MOVED IS LESS THAN THE TARGET -------------
// ACCEPTS tf::tfMessage cvalue (CONSTANT) -------------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void forwardprog(const tf::tfMessage cvalue)
{
  double dx, dy; // VARIABLES FOR X,Y COORDINATES

  // SETS THE INITIAL X AND Y COORDINATES
  if(initialx==INITIALIZE_VALUE || initialy==INITIALIZE_VALUE)
  {
    initialx=cvalue.transforms[0].transform.translation.x;
    initialy=cvalue.transforms[0].transform.translation.y;
  }

  // CALCULATES DISTANCE IN X AND Y TRAVELED (ONLY CARES ABOUT FORWARD MOVEMENT)
  dx = std::abs(cvalue.transforms[0].transform.translation.x-initialx);
  dy = std::abs(cvalue.transforms[0].transform.translation.y-initialy);
  
  // CALCULATES TOTAL DISTANCE THE ROBOT HAS TRAVELLED
  moved = sqrt(dx*dx+dy*dy);
  
  // CALCULATES DISTANCE (METERS) THE ROBOT HAS YET TO TRAVEL
  if(moved>target_forward)
  {
    flag=false;
  }

  // SETS A SPEED PROPORTIONAL TO THE DISTANCE YET TO BE TRAVELED
  // PLUS AN OFFSET TO ACCOUNT FOR FRICTION
  // SPEED IS IN M/S
  target_speed = std::abs(target_forward - moved)/4+0.1;
}
 
 // ...


// THIS FUNCTION 
// ACCEPTS tf::tfMessage cvalue (CONSTANT) -------------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void Turnprog(const tf::tfMessage cvalue)
{
  double turnz, turnw, mindist;

  // CALCULATE ORIENTATION OF THE ROBOT
  turnz = cvalue.transforms[0].transform.rotation.z;
  turnw = cvalue.transforms[0].transform.rotation.w;

  // CALCULATE CURRENT ANGLE OF THE ROBOT
  current_angle = 2*atan2(turnz,turnw);

  // CONVERTS THE CURRENT ANGLE TO BE BETWEEN 0 AND 2PI
  if(current_angle < 0)
  {
   current_angle =  current_angle + 2*PI;
  }
  if(current_angle >= 2*PI)
  {
    current_angle =  current_angle - 2*PI;
  }
  
  // SETS THE TARGET ANGLE

  if(target_angle == INITIALIZE_VALUE)

  {

    target_angle =  current_angle + PI/2;

  }
  
  // CONVERTS THE TARGET ANGLE TO BE BETWEEN 0 AND 2PI
  if(target_angle < 0)
  {
    target_angle =  target_angle + 2*PI;
  }
  if(target_angle >= 2*PI)
  {
    target_angle =  target_angle - 2*PI;
  }

  // DETERMINES IF THE ROBOT HAS PASSED THE TARGET ANGLE. 
  // (ONLY WORKS FOR TURNING COUNTER-CLOCKWISE)
  // THE LOGIC AFTER && ACCOUNTS FOR GOING FROM A HIGH CURRENT ANGLE,
  // SUCH AS 7PI/4, TO A LOW VALUE, SUCH AS 0
  if((current_angle>=target_angle)&&(std::abs(current_angle-target_angle)<0.1))
  {
    flag = false;
  }

  // FINDS THE MINIMUM ANGLE BETWEEN THE CURRENT AND TARGET ANGLE
  mindist = std::abs(target_angle-current_angle);
  
  // ACCOUNTS FOR GOING FROM A HIGH CURRENT ANGLE, 
  // SUCH AS 7PI/4, TO A LOW VALUE, SUCH AS 0
  if(std::abs(target_angle-current_angle+2*PI)<mindist)
  {
    mindist=std::abs(target_angle-current_angle+2*PI);
  }

  // SETS THE TARGET SPEED TO BE PROPORTIONAL TO THE NEEDED ANGLE FOR TRAVEL
  // PLUS AN OFFSET TO ACCOUNT FOR FRICTION
  // SPEED IS MEASURED IN RAD/S
  target_speed = mindist/PI+0.5;
}

// ...

 


int main(int argc, char **argv)
{

  // NAMES THE PROGRAM (FOR VISUAL PURPOSES)
  ros::init(argc, argv, "lab3_tutorial");
  
  // NODEHANDLE::ADVERTISE() RETURNS A ROS::PUBLISHER OBJECT, 
  // WHICH SERVES TWO PURPOSES: 1) IT CONTAINS A PUBLISH() METHOD THAT 
  // LETS YOU PUBLISH MESSAGES ONTO THE TOPIC IT WAS CREATED WITH, AND 
  // 2) WHEN IT GOES OUT OF SCOPE, IT WILL AUTOMATICALLY UNADVERTISE.
  ros::NodeHandle n;

  // PUBLISHER DECLARATION FOR THE VELOCITY OF THE ROBOT
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  
  ros::Subscriber tf; // SUBSCRIBER DECLARATION TO GET THE TRANSFORMATION MATRICES OF THE ROBOT
  
  // SETS THE FREQUENCY FOR WHICH THE PROGRAM SLEEPS AT. 1000=1/1000 SECOND
  // A ROS::RATE OBJECT ALLOWS YOU TO SPECIFY A FREQUENCY THAT YOU WOULD LIKE TO 
  // LOOP AT. IT WILL KEEP TRACK OF HOW LONG IT HAS BEEN SINCE THE LAST CALL 
  // TO RATE::SLEEP(), AND SLEEP FOR THE CORRECT AMOUNT OF TIME. Ref:
  // http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
  ros::Rate loop_rate(1000);

  // SETS THE CURRENT TIME TO BE 0
  ros::Time begin = ros::Time::now();

  // GETS AN INPUT FROM THE USER, WAITS UNTIL IT IS 'Y'
  char input=0;
  fflush(stdin);
  std::cout << "Type 'y' and enter to begin: ";
  while(input!='y' && ros::ok())
  {
    std::cin >> input;
  }

  // INITIALIZES THE ROBOT VELOCITY
  geometry_msgs::Twist c;
  c.linear.x=0.0;
  c.linear.y=0.0;
  c.linear.z=0.0;
  c.angular.x=0.0;
  c.angular.y=0.0;
  c.angular.z=0.0;
  
  // CREATES A TIME VARIABLE
  ros::Time go;
  
  // LOOPS 4 TIMES  (FOR EACH SIDE OF A SQUARE)
  for(int i=0;i<4;i++)
  {
	  
    // INITIALIZES THE NEEDED VALUES
    flag = true;
    moved = 0;
    initialx=INITIALIZE_VALUE;
    initialy=INITIALIZE_VALUE;

    // DECLARE SUBSCRIBER TO THE TF TOPIC (MR POSE) WITH THE FUNCTION FORWARDPROG
    tf = n.subscribe("/tf", 1, forwardprog);
    ros::spinOnce();
    loop_rate.sleep();
    
    // GOES FORWARD UNTIL THE NECESSARY DISTANCE IS TRAVELED
    while(flag && ros::ok())
    {
	  //PUBLISHES THE TARGET SPEED, CALCULATED WHEN THE ROBOT POSITION IS SUBSCRIBED (WHEN ros::spinOnce(); IS RUN)
      c.linear.x=target_speed;
      c.angular.z=0;
      vel_pub.publish(c);

      //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
      ros::spinOnce();
      loop_rate.sleep(); 

    }
    // SHUTS DOWN THE SUBSCRIBER TO CHANGE ITS RELATED FUNCTION LATER
    tf.shutdown();

    // STOPS THE ROBOT FOR A LITTLE TO BE SAFE
    go = ros::Time::now();
    while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
    {
	  //PUBLISHES 0 VELOCITY TO STOP THE ROBOT;
      c.linear.x=0.0;
      c.angular.z=0;
      vel_pub.publish(c);
      
	  //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
      ros::spinOnce();
      loop_rate.sleep();
    }

    // INITIALIZES THE FLAG
    flag = true;
    // DECLARE SUBSCRIBER TO THE TF TOPIC (MR POSE) WITH THE FUNCTION TURNPROG
    tf = n.subscribe("/tf", 1, Turnprog);
    ros::spinOnce();
    loop_rate.sleep();

    // SPINS IN PLACE COUNTER CLOCKWISE UNTIL THE DESIRED ANGLE IS REACHED
    while(flag && ros::ok())
    {
	  //PUBLISHES THE TARGET SPEED, CALCULATED WHEN THE ROBOT POSITION IS SUBSCRIBED (WHEN ros::spinOnce(); IS RUN)
      c.linear.x=0.0;
      c.angular.z=target_speed;
      vel_pub.publish(c);
	  
	  //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
      ros::spinOnce();
      loop_rate.sleep();
    }
    
	// SHUTS DOWN THE SUBSCRIBER TO CHANGE ITS RELATED FUNCTION LATER
    tf.shutdown();
    
	// INCREMENTS THE TARGET ANGLE
    target_angle=target_angle+PI/2;

    // STOPS THE ROBOT FOR A LITTLE TO BE SAFE
    go = ros::Time::now();
    while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
    {
	  //PUBLISHES 0 VELOCITY TO STOP THE ROBOT;
      c.linear.x=0.0;
      c.angular.z=0;
      vel_pub.publish(c);

      //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
      ros::spinOnce();
      loop_rate.sleep();
    }

  }


  return 0;
}

