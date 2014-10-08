#include "ros/ros.h"
#include "std_msgs/String.h"

#include <facial_expression/emotionInterface.h>

emotions::EmotionInterface emo;

void expressionCallback(const std_msgs::String::ConstPtr& msg)
{
  
  emo.setAll(msg->data.c_str());
  
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "switch_expression");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string serial_config, exp_lib;
  // get relevant parameters as which cameras should be used
  nh_priv.param<std::string>("serial_config", serial_config, "config/serialport.yaml");
  nh_priv.param<std::string>("expression_lib", exp_lib, "config/emotions.yaml");
  emo.configure( serial_config.c_str(), exp_lib.c_str());

  ros::Subscriber sub = nh.subscribe("expression", 1000, expressionCallback);

  ros::spin();

  return 0;
}
