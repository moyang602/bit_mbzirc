#include "HalconCpp.h"
#include <ros/ros.h> 
int main (int argc, char** argv) 
{
    ros::init(argc, argv, "brick_locate");
    
    ros::NodeHandle nh;
    
    using namespace HalconCpp;
    
    HImage Mandrill("monkey");                // read image from file "monkey"
    Hlong width,height;
    Mandrill.GetImageSize(&width,&height);

    HWindow w(0,0,width,height);              // window with size equal to image

    Mandrill.DispImage(w);                    // display image in window
    w.Click();                                // wait for mouse click
    w.ClearWindow();

    HRegion Bright = Mandrill >= 128;         // select all bright pixels
    HRegion Conn = Bright.Connection();       // get connected components

    // select regions with a size of at least 500 pixels
    HRegion Large = Conn.SelectShape("area","and",500,90000);

    // select the eyes out of the instance variable Large by using
    // the anisometry as region feature:
    HRegion Eyes = Large.SelectShape("anisometry","and",1,1.7);

    Eyes.DispRegion(w);                       // display result image in window
    w.Click();                                // wait for mouse click
}