#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::vector<float> array = msg->data;
    std::cout << "msg->data[0]=" << msg->data[0] << std::endl;
    std::cout << "msg->data.size=" << msg->data.size() << std::endl;
    std::cout << "msg->data=" << msg->data[0] << ", " << msg->data[1] <<  ", " << msg->data[2] << ", " <<  msg->data[3] << ", " <<  msg->data[4] << ", " <<  msg->data[5] << std::endl;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1, chatterCallback);

  ros::spin();

  return 0;
}