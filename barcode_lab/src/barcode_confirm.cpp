#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

// This node waits for 5 consecutive identical barcode readings
// before publishing a confirmed barcode

class BarcodeConfirm
{
public:
  BarcodeConfirm()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // number of consecutive identical reads required
    pnh.param("required_count", required_count_, 5);

    sub_ = nh.subscribe("/barcode", 50, &BarcodeConfirm::callback, this);
    pub_ = nh.advertise<std_msgs::String>("/barcode_confirmed", 10);

    ROS_INFO("barcode_confirm: waiting for %d consecutive identical reads...", required_count_);
  }

private:

  void callback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string& current = msg->data;

    // check if same as previous
    if (current == last_)
    {
      streak_++;
    }
    else
    {
      last_ = current;
      streak_ = 1;
      published_ = false;
    }

    // publish only once when threshold reached
    if (!published_ && streak_ >= required_count_)
    {
      std_msgs::String out;
      out.data = last_;
      pub_.publish(out);

      published_ = true;

      ROS_INFO("CONFIRMED BARCODE: %s", last_.c_str());
    }
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;

  std::string last_ = "";
  int streak_ = 0;
  int required_count_ = 5;
  bool published_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barcode_confirm");

  BarcodeConfirm node;

  ros::spin();

  return 0;
}
