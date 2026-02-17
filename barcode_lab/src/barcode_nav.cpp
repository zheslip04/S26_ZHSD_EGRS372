#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>
#include <string>
#include <vector>

#define PI 3.14159265359

// ===================== ODOM STATE =====================
double current_x = 0.0;
double current_y = 0.0;
double current_yaw = 0.0;
bool have_odom = false;

// ===================== TASK START (RETURN) =====================
double start_x = 0.0;
double start_y = 0.0;
double start_yaw = 0.0;

// ===================== DRIVE GOAL TRACKING =====================
double drive_start_x = 0.0;
double drive_start_y = 0.0;
double goal_distance = 0.0;

// Heading-hold for straight lines (reduces drift on tile)
double drive_target_yaw = 0.0;

// ===================== TURN GOAL TRACKING =====================
double turn_target_yaw = 0.0;
bool turn_target_set = false;

// ===================== SQUARE TRACKING =====================
int square_side = 0;
int square_direction = +1; // +1 CCW, -1 CW

// ===================== ROS =====================
ros::Publisher cmd_pub;

// ===================== TILE-TUNED PARAMETERS =====================
// Linear speed: slow down near goal to reduce overshoot
double V_MAX    = 0.22;   // max forward speed
double V_MIN    = 0.06;   // min forward speed
double DIST_TOL = 0.02;   // stop tolerance for distances (meters)

// Turn control: P-control with clamps + braking
double Kp_turn  = 2.0;    // turn gain
double W_MAX    = 0.85;   // max angular speed
double W_MIN    = 0.10;   // min angular speed
double YAW_TOL  = 0.012;  // stop tolerance (radians) ~0.7 degrees

// Heading-hold during straight segments (keep lines straight)
double Kp_hold  = 1.2;    // heading hold gain
double W_HOLD_MAX = 0.35; // clamp heading correction while driving

// Return-to-start control for Barcode 3
double Kp_home  = 1.6;    // steering gain while homing
double HOME_W_MAX = 0.55;
double RETURN_TOL = 0.08; // meters

// Braking/settle (helps a lot on tile)
double BRAKE_DT = 0.02;   // seconds between brake publishes
int    BRAKE_N  = 6;      // number of brake publishes
double SETTLE_T = 0.10;   // settle time after stopping

// ===================== EXECUTION CONTROL =====================
bool busy = false;

enum Mode
{
  IDLE,
  // Square
  SQ_DRIVE,
  SQ_TURN,
  // Barcode 3 sequence
  B3_TURN1,
  B3_OUT,
  B3_TURN2,
  B3_HOME_TURN,
  B3_HOME_DRIVE
};

Mode mode = IDLE;

// ===================== BARCODE MAPPING =====================
// If you want explicit barcode strings, set these (or via params).
// If empty, it will auto-learn the first 3 unique confirmed barcodes.
std::string barcode1 = "705632441947";
std::string barcode2 = "05111140759";
std::string barcode3 = "123456789012";
std::vector<std::string> learned; // auto-learn storage

// ===================== HELPERS =====================
double wrapToPi(double a)
{
  while (a > PI) a -= 2.0 * PI;
  while (a < -PI) a += 2.0 * PI;
  return a;
}

double dist(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

std::string normalize(const std::string &s)
{
  // Strip newline/tab characters; keep everything else (zbar formats vary)
  std::string out;
  for (char c : s)
  {
    if (c != '\n' && c != '\r' && c != '\t') out.push_back(c);
  }
  return out;
}

void publishCmd(double lin, double ang)
{
  geometry_msgs::Twist t;
  t.linear.x = lin;
  t.angular.z = ang;
  cmd_pub.publish(t);
}

void brakeAndSettle()
{
  // Actively publish stop several times to reduce coasting/overshoot
  for (int i = 0; i < BRAKE_N; i++)
  {
    publishCmd(0.0, 0.0);
    ros::Duration(BRAKE_DT).sleep();
  }
  ros::Duration(SETTLE_T).sleep();
}

double computeDriveSpeed(double remaining_m)
{
  // Proportional slow-down near goal reduces overshoot on tile
  double v = 0.7 * std::fabs(remaining_m);
  if (v > V_MAX) v = V_MAX;
  if (v < V_MIN) v = V_MIN;
  return v;
}

double computeTurnSpeed(double err_rad)
{
  double w = Kp_turn * err_rad;

  if (w >  W_MAX) w =  W_MAX;
  if (w < -W_MAX) w = -W_MAX;

  if (std::fabs(w) < W_MIN)
    w = (w >= 0.0) ? W_MIN : -W_MIN;

  return w;
}

bool matchContains(const std::string& haystack, const std::string& needle)
{
  if (needle.empty()) return false;
  return haystack.find(needle) != std::string::npos;
}

int autoLearnIndex(const std::string& code)
{
  // Return 0/1/2 if learned, -1 otherwise. Learn if there is room.
  for (size_t i = 0; i < learned.size(); i++)
    if (learned[i] == code) return (int)i;

  if (learned.size() < 3)
  {
    learned.push_back(code);
    ROS_INFO("Learned BARCODE %zu = '%s'", learned.size(), code.c_str());
    return (int)learned.size() - 1;
  }
  return -1;
}

// ===================== ODOM CALLBACK =====================
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  current_yaw = tf::getYaw(q);

  have_odom = true;
}

// ===================== BARCODE CALLBACK =====================
void barcodeCallback(const std_msgs::String::ConstPtr& msg)
{
  if (!have_odom)
  {
    ROS_WARN("barcode_nav: waiting for /odom...");
    return;
  }

  if (busy)
  {
    ROS_WARN("barcode_nav: busy executing motion. Ignoring new barcode.");
    return;
  }

  std::string raw = normalize(msg->data);

  // Decide which behavior: explicit mapping OR auto-learn
  int which = -1;

  if (!barcode1.empty() || !barcode2.empty() || !barcode3.empty())
  {
    if (matchContains(raw, barcode1)) which = 0;
    else if (matchContains(raw, barcode2)) which = 1;
    else if (matchContains(raw, barcode3)) which = 2;
    else which = -1;
  }
  else
  {
    which = autoLearnIndex(raw);
  }

  if (which < 0)
  {
    ROS_WARN("invalid barcode: '%s' (waiting)", raw.c_str());
    return;
  }

  // Record start pose for "circle back to its starting point"
  start_x = current_x;
  start_y = current_y;
  start_yaw = current_yaw;

  // Start behavior
  if (which == 0)
  {
    ROS_INFO("Barcode 1 -> square 1m CLOCKWISE");
    busy = true;
    square_side = 0;
    square_direction = -1;          // clockwise
    drive_start_x = current_x;
    drive_start_y = current_y;
    goal_distance = 1.0;
    drive_target_yaw = current_yaw; // heading-hold
    mode = SQ_DRIVE;
  }
  else if (which == 1)
  {
    ROS_INFO("Barcode 2 -> square 1m COUNTERCLOCKWISE");
    busy = true;
    square_side = 0;
    square_direction = +1;          // counterclockwise
    drive_start_x = current_x;
    drive_start_y = current_y;
    goal_distance = 1.0;
    drive_target_yaw = current_yaw; // heading-hold
    mode = SQ_DRIVE;
  }
  else if (which == 2)
  {
    ROS_INFO("Barcode 3 -> 180, forward 2m, 180, then return to start (homing)");
    busy = true;

    // First 180 turn (target yaw)
    turn_target_set = false;
    turn_target_yaw = wrapToPi(current_yaw + PI);
    turn_target_set = true;

    mode = B3_TURN1;
  }
}

// ===================== STATE MACHINE =====================
void update()
{
  if (!have_odom) return;

  switch (mode)
  {
    case IDLE:
      break;

    // =========================================================
    // SQUARE DRIVE 1m (with heading hold)
    // =========================================================
    case SQ_DRIVE:
    {
      double traveled = dist(drive_start_x, drive_start_y, current_x, current_y);
      double remaining = goal_distance - traveled;

      if (remaining <= DIST_TOL)
      {
        publishCmd(0.0, 0.0);
        brakeAndSettle();

        // Set 90-degree turn target yaw
        turn_target_yaw = wrapToPi(current_yaw + square_direction * (PI / 2.0));
        turn_target_set = true;

        mode = SQ_TURN;
      }
      else
      {
        // heading hold to keep the side straight
        double heading_err = wrapToPi(drive_target_yaw - current_yaw);
        double w_hold = Kp_hold * heading_err;
        if (w_hold >  W_HOLD_MAX) w_hold =  W_HOLD_MAX;
        if (w_hold < -W_HOLD_MAX) w_hold = -W_HOLD_MAX;

        double v = computeDriveSpeed(remaining);
        publishCmd(v, w_hold);
      }
      break;
    }

    // =========================================================
    // SQUARE TURN 90 degrees (P-control + braking + settle)
    // =========================================================
    case SQ_TURN:
    {
      if (!turn_target_set)
      {
        // Safety: in case something resets it
        turn_target_yaw = wrapToPi(current_yaw + square_direction * (PI / 2.0));
        turn_target_set = true;
      }

      double err = wrapToPi(turn_target_yaw - current_yaw);

      if (std::fabs(err) <= YAW_TOL)
      {
        publishCmd(0.0, 0.0);
        brakeAndSettle();

        turn_target_set = false;
        square_side++;

        if (square_side >= 4)
        {
          ROS_INFO("Square complete. Stopping.");
          publishCmd(0.0, 0.0);
          mode = IDLE;
          busy = false;
        }
        else
        {
          // Start next side
          drive_start_x = current_x;
          drive_start_y = current_y;
          goal_distance = 1.0;
          drive_target_yaw = current_yaw; // reset heading hold each side
          mode = SQ_DRIVE;
        }
      }
      else
      {
        double w = computeTurnSpeed(err);
        publishCmd(0.0, w);
      }
      break;
    }

    // =========================================================
    // BARCODE 3: TURN 180
    // =========================================================
    case B3_TURN1:
    {
      double err = wrapToPi(turn_target_yaw - current_yaw);

      if (std::fabs(err) <= YAW_TOL)
      {
        publishCmd(0.0, 0.0);
        brakeAndSettle();

        drive_start_x = current_x;
        drive_start_y = current_y;
        goal_distance = 2.0;
        drive_target_yaw = current_yaw; // hold heading outbound
        mode = B3_OUT;
      }
      else
      {
        double w = computeTurnSpeed(err);
        publishCmd(0.0, w);
      }
      break;
    }

    // =========================================================
    // BARCODE 3: DRIVE OUT 2m (with heading hold)
    // =========================================================
    case B3_OUT:
    {
      double traveled = dist(drive_start_x, drive_start_y, current_x, current_y);
      double remaining = goal_distance - traveled;

      if (remaining <= DIST_TOL)
      {
        publishCmd(0.0, 0.0);
        brakeAndSettle();

        // Second 180 turn
        turn_target_yaw = wrapToPi(current_yaw + PI);
        turn_target_set = true;
        mode = B3_TURN2;
      }
      else
      {
        double heading_err = wrapToPi(drive_target_yaw - current_yaw);
        double w_hold = Kp_hold * heading_err;
        if (w_hold >  W_HOLD_MAX) w_hold =  W_HOLD_MAX;
        if (w_hold < -W_HOLD_MAX) w_hold = -W_HOLD_MAX;

        double v = computeDriveSpeed(remaining);
        publishCmd(v, w_hold);
      }
      break;
    }

    // =========================================================
    // BARCODE 3: TURN BACK 180
    // =========================================================
    case B3_TURN2:
    {
      double err = wrapToPi(turn_target_yaw - current_yaw);

      if (std::fabs(err) <= YAW_TOL)
      {
        publishCmd(0.0, 0.0);
        brakeAndSettle();

        mode = B3_HOME_TURN;
      }
      else
      {
        double w = computeTurnSpeed(err);
        publishCmd(0.0, w);
      }
      break;
    }

    // =========================================================
    // BARCODE 3: TURN TOWARD HOME (bearing to start)
    // =========================================================
    case B3_HOME_TURN:
    {
      double dx = start_x - current_x;
      double dy = start_y - current_y;
      double bearing = std::atan2(dy, dx);
      double err = wrapToPi(bearing - current_yaw);

      if (std::fabs(err) <= 0.03) // ~1.7 degrees is good for homing
      {
        publishCmd(0.0, 0.0);
        brakeAndSettle();
        mode = B3_HOME_DRIVE;
      }
      else
      {
        // Use the home steering controller (slightly gentler than turn controller)
        double w = Kp_home * err;
        if (w >  HOME_W_MAX) w =  HOME_W_MAX;
        if (w < -HOME_W_MAX) w = -HOME_W_MAX;

        // ensure minimum turning effort
        if (std::fabs(w) < 0.10) w = (w >= 0.0) ? 0.10 : -0.10;

        publishCmd(0.0, w);
      }
      break;
    }

    // =========================================================
    // BARCODE 3: DRIVE HOME (steer continuously toward start)
    // =========================================================
    case B3_HOME_DRIVE:
    {
      double d_home = dist(start_x, start_y, current_x, current_y);

      if (d_home <= RETURN_TOL)
      {
        publishCmd(0.0, 0.0);
        brakeAndSettle();
        ROS_INFO("Returned to start. Stopping.");
        mode = IDLE;
        busy = false;
      }
      else
      {
        double dx = start_x - current_x;
        double dy = start_y - current_y;
        double bearing = std::atan2(dy, dx);
        double err = wrapToPi(bearing - current_yaw);

        double w = Kp_home * err;
        if (w >  HOME_W_MAX) w =  HOME_W_MAX;
        if (w < -HOME_W_MAX) w = -HOME_W_MAX;

        // Slow down as we approach home
        double v = 0.25 * d_home;
        if (v > 0.18) v = 0.18;
        if (v < 0.06) v = 0.06;

        publishCmd(v, w);
      }
      break;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barcode_nav");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Optional explicit mapping (if you want it):
  // rosparam set /barcode_nav/barcode1 "0705632441947"
  // If these are empty, we auto-learn first 3 unique confirmed barcodes.
  pnh.param<std::string>("barcode1", barcode1, std::string("705632441947"));
  pnh.param<std::string>("barcode2", barcode2, std::string("05111140759"));
  pnh.param<std::string>("barcode3", barcode3, std::string("123456789012"));

  cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
  ros::Subscriber bc_sub   = nh.subscribe("/barcode_confirmed", 10, barcodeCallback);

  if (barcode1.empty() && barcode2.empty() && barcode3.empty())
  {
    ROS_INFO("barcode_nav: Auto-learn mode ON. Show 3 different confirmed barcodes to learn #1/#2/#3.");
  }
  else
  {
    ROS_INFO("barcode_nav: Explicit barcode mapping ON.");
  }

  ros::Rate rate(30);

  while (ros::ok())
  {
    ros::spinOnce();
    update();
    rate.sleep();
  }

  publishCmd(0.0, 0.0);
  return 0;
}
