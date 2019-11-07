/**
 * This tutorial demonstrates how to receive the Left and Right rectified images
 * from the ZED node
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "HalconCpp.h"
#include <string>
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include <sstream>
#include <math.h>
#include "halcon_image.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"

using namespace std;
using namespace HalconCpp;

/**
 * Subscriber callbacks. The argument of the callback is a constant pointer to the received message
 */

void threshold(HObject Image){
    HImage Mandrill = Image;
    Hlong width,height;
    Mandrill.GetImageSize(&width,&height);
    HWindow w(0,0,width,height);  
    Mandrill.DispImage(w); 
    w.Click();   
    w.ClearWindow();
    HRegion Bright = Mandrill >= 128; 
    HRegion Conn = Bright.Connection();
    HRegion Large = Conn.SelectShape("area","and",500,90000);
    HRegion Eyes = Large.SelectShape("anisometry","and",1,1.7);
    Eyes.DispRegion(w);  
    w.Click();    
}

void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
    //初始化halcon对象
    HObject  ho_ImageSub, ho_Image;
    //获取halcon-bridge图像指针
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointer = halcon_bridge::toHalconCopy(msg);
    ho_Image = *halcon_bridge_imagePointer->image;

    threshold(ho_Image);

}




/**
 * Node main function
 */
int main(int argc, char** argv) {
    
    ros::init(argc, argv, "zed_simple_test");

    ros::NodeHandle n;

    ros::Subscriber subLeftRectified  = n.subscribe("/zed/zed_node/left/image_rect_color", 10,
                                        imageLeftRectifiedCallback);

    ros::spin();

    return 0;
}
