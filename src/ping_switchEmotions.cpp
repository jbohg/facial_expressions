#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;

  ros::Publisher exp_pub = nh.advertise<std_msgs::String>("expression", 1000);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    
    std_msgs::String msg;

    std::stringstream ss;
    if(count%2)
      ss << "hap";
    else 
      ss << "sad";
    msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    exp_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
