#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>

/*
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

void callback(const sensor_msgs::ImageConstPtr & rgb_image,
              const sensor_msgs::PointCloud2ConstPtr::ConstPtr& point_cloud2)
{
 
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "building_status");
    ros::NodeHandle nh("~");

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/zed/zed_node/rgb/image_rect_color",1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/zed/zed_node/point_cloud/cloud_registered",1);

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), rgb_sub, cloud_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    
    return 0;
}
*/

int main(int argc, char *argv[])
{
}