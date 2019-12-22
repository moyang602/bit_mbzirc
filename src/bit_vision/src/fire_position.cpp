#ifndef __APPLE__
#  include "HalconCpp.h"
#  include "HDevThread.h"
#  if defined(__linux__) && !defined(__arm__) && !defined(NO_EXPORT_APP_MAIN)
#    include <X11/Xlib.h>
#  endif
#else
#  ifndef HC_LARGE_IMAGES
#    include <HALCONCpp/HalconCpp.h>
#    include <HALCONCpp/HDevThread.h>
#  else
#    include <HALCONCppxl/HalconCpp.h>
#    include <HALCONCppxl/HDevThread.h>
#  endif
#  include <stdio.h>
#  include <HALCON/HpThread.h>
#  include <CoreFoundation/CFRunLoop.h>
#endif

#include <ros/ros.h>
#undef Success    //  解决添加#include <tf/transform_listener.h>时报错的问题
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <bit_vision/FirePosition.h>
#include "halcon_image.h"
#include <tf/transform_listener.h>

# define HandEye        1   
# define StereoEye      2

using namespace std;
using namespace HalconCpp;

//初始化halcon对象
HObject  ho_ImageL, ho_ImageR;
tf::StampedTransform transform_IRLOnBase;

//读入相机标定参数  双目三维定位
void fire_position_StereoEye(HObject ho_ImageL, HObject ho_ImageR, HTuple &fire_X, HTuple &fire_Y, HTuple &fire_Z, bool &data_flag)
{
  /*
  // Local iconic variables
  HObject  ho_Img1, ho_Rectangle, ho_Image1, ho_ConnectedRegion1, ho_SelectedRegions1;
  HObject  ho_Image11, ho_Image12, ho_Image13, ho_ImageResult11, ho_ImageResult12;
  HObject  ho_ImageResult13, ho_Region1, ho_RegionOpening1;
  HObject  ho_RegionClosing1, ho_RegionFillUp1, ho_ImageReduced1;
  HObject  ho_RegionBorder1, ho_Contours1, ho_Rectangle1, ho_Cross1, ho_Cross2;

  // Local control variables
  
  HTuple  hv_Width, hv_Height, hv_WindowHandle;
  HTuple  hv_Number1, hv_Number2, hv_k, hv_Row1, hv_Column1;
  HTuple  hv_Phi1, hv_Length11, hv_Length12, hv_PointOrder1;


  //ReadImage(&ho_Img1, "C:/Users/Admin/Documents/fire-detection/infrared1.jpeg");
  GetImageSize(Image, &hv_Width, &hv_Height);
  GenRectangle1(&ho_Rectangle, 0, 0, 288, 360);
  ReduceDomain(Image, ho_Rectangle, &ho_Image1); 

  Decompose3(ho_Image1, &ho_Image11, &ho_Image12, &ho_Image13);
  
  Threshold(ho_Image12, &ho_Region1, 155, 255);
  OpeningCircle(ho_Region1, &ho_RegionOpening1, 1.5);
  ClosingCircle(ho_RegionOpening1, &ho_RegionClosing1, 1);
  FillUp(ho_Region1, &ho_RegionFillUp1);
  //reduce_domain (ImageResult12, RegionFillUp1, ImageReduced1)
  Connection(ho_Region1, &ho_ConnectedRegion1);
  SelectShape(ho_ConnectedRegion1, &ho_SelectedRegions1, "max_diameter", "and", 20, 999);
  CountObj(ho_SelectedRegions1, &hv_Number1);
  ReduceDomain(ho_Image1, ho_SelectedRegions1, &ho_ImageReduced1);

  if (0 != (HTuple(hv_Number1!=0)))
  {
    hv_k = 1;

    Boundary(ho_SelectedRegions1, &ho_RegionBorder1, "inner");
    GenContourRegionXld(ho_RegionBorder1, &ho_Contours1, "center");
    FitRectangle2ContourXld(ho_Contours1, "regression", -1, 0, 0, 3, 2, &hv_Row1, &hv_Column1, &hv_Phi1, &hv_Length11, &hv_Length12, &hv_PointOrder1);
    GenRectangle2ContourXld(&ho_Rectangle1, hv_Row1, hv_Column1, hv_Phi1, hv_Length11, hv_Length12);

    GenCrossContourXld(&ho_Cross1, hv_Row1, hv_Column1, 6, hv_Phi1);

    xL = hv_Row1;
    yL = hv_Column1;

  }


   HObject  ho_Img1, ho_Rectangle, ho_Image1;
  HObject  ho_Image11, ho_Image12, ho_Image13;
  HObject  ho_ImageResult11, ho_ImageResult12;
  HObject  ho_ImageResult13, ho_Region1, ho_RegionOpening1;
  HObject  ho_RegionClosing1, ho_RegionFillUp1, ho_ImageReduced1;
  HObject  ho_ConnectedRegion1, ho_SelectedRegions1;
  HObject  ho_RegionBorder1, ho_Contours1, ho_Rectangle1, ho_Cross1;

  // Local control variables
  
  HTuple  hv_Width, hv_Height, hv_WindowHandle;
  HTuple  hv_Number1, hv_Number2, hv_k, hv_Row1, hv_Column1;
  HTuple  hv_Phi1, hv_Length11, hv_Length12, hv_PointOrder1;


  //ReadImage(&ho_Img1, "C:/Users/Admin/Documents/fire-detection/infrared1.jpeg");
  GetImageSize(Image, &hv_Width, &hv_Height);
  GenRectangle1(&ho_Rectangle, 0, 0, 288, 360);
  ReduceDomain(Image, ho_Rectangle, &ho_Image1);
  Decompose3(ho_Image1, &ho_Image11, &ho_Image12, &ho_Image13);
  
  //阈值分割 选出面积最大的区域
  Threshold(ho_Image12, &ho_Region1, 155, 255);
  OpeningCircle(ho_Region1, &ho_RegionOpening1, 1.5);
  ClosingCircle(ho_RegionOpening1, &ho_RegionClosing1, 1);
  FillUp(ho_Region1, &ho_RegionFillUp1);
  Connection(ho_Region1, &ho_ConnectedRegion1);
  SelectShape(ho_ConnectedRegion1, &ho_SelectedRegions1, "max_diameter", "and", 20, 999);
  CountObj(ho_SelectedRegions1, &hv_Number1);
  ReduceDomain(ho_Image1, ho_SelectedRegions1, &ho_ImageReduced1);

  if (0 != (HTuple(hv_Number1!=0)))
  {
    hv_k = 1;

    Boundary(ho_SelectedRegions1, &ho_RegionBorder1, "inner");
    GenContourRegionXld(ho_RegionBorder1, &ho_Contours1, "center");
    FitRectangle2ContourXld(ho_Contours1, "regression", -1, 0, 0, 3, 2, &hv_Row1, &hv_Column1, &hv_Phi1, &hv_Length11, &hv_Length12, &hv_PointOrder1);
    GenRectangle2ContourXld(&ho_Rectangle1, hv_Row1, hv_Column1, hv_Phi1, hv_Length11, hv_Length12);

    GenCrossContourXld(&ho_Cross1, hv_Row1, hv_Column1, 6, hv_Phi1);

    xR = hv_Row1;
    yR = hv_Column1;
  }


  HTuple hv_CameraParameters1, hv_CameraParameters2,hv_RealPose;
  HTuple hv_X, hv_Y, hv_Z, hv_Dist;

  try
  {
    ReadCamPar("campar1.dat", &hv_CameraParameters1);
    ReadCamPar("campar2.dat", &hv_CameraParameters2);
    ReadPose("relpose.dat", &hv_RealPose);

    IntersectLinesOfSight(hv_CameraParameters1, hv_CameraParameters2, hv_RealPose, rowL, colomnL, rowR, colomnR, &hv_X, &hv_Y, &hv_Z, &hv_Dist);
    fire_X = hv_X;
    fire_Y = hv_Y;
    fire_Z = hv_Z;

    return 0;
  }
  catch (HException &exception)
  {
    ROS_ERROR("  Error #%u in %s: %s\n", exception.ErrorCode(),
            (const char *)exception.ProcName(),
            (const char *)exception.ErrorMessage());

    return 1;
  }
  
  */
}

//单目定位
void fire_position_HandEye(HObject ho_Image, double &delta_x, double &delta_y, bool &data_flag)
{
   // Local iconic variables
  HObject  ho_GrayImage1, ho_Region1;
  HObject  ho_RegionOpening1, ho_RegionClosing1, ho_RegionFillUp1;
  HObject  ho_ConnectedRegions1, ho_SelectedRegions1, ho_Cross1;

  // Local control variables
  HTuple  hv_Width, hv_Height, hv_WindowHandle1;
  HTuple  hv_WindowHandle2, hv_Number1, hv_Area1, hv_Row1;
  HTuple  hv_Column1;

  try
  {
    Rgb1ToGray(ho_Image, &ho_GrayImage1);

    Threshold(ho_GrayImage1, &ho_Region1, 100, 255);
    OpeningCircle(ho_Region1, &ho_RegionOpening1, 1.5);
    ClosingCircle(ho_RegionOpening1, &ho_RegionClosing1, 1.5);
    FillUp(ho_RegionClosing1, &ho_RegionFillUp1);
    Connection(ho_RegionFillUp1, &ho_ConnectedRegions1);
    SelectShape(ho_ConnectedRegions1, &ho_SelectedRegions1, "area", "and", 5, 2000);
    CountObj(ho_SelectedRegions1, &hv_Number1);

    if (0 != (hv_Number1==1))
    {
      AreaCenter(ho_SelectedRegions1, &hv_Area1, &hv_Row1, &hv_Column1);
      data_flag = true;
      delta_x = 384/2 - hv_Column1.D();
      delta_y = 288/2 - hv_Row1.D();
    }
    else
    {
      data_flag = false;
      delta_x = 0.0;
      delta_y = 0.0;

    }
  }
  catch (HException &exception)
  {
    ROS_ERROR("44  Error #%u in %s: %s\n", exception.ErrorCode(),
            (const char *)exception.ProcName(),
            (const char *)exception.ErrorMessage());
    data_flag = false;
    delta_x = 0.0;
    delta_y = 0.0;
  }
    
  return;
}

//回调函数
void Imagecallback(const sensor_msgs::Image::ConstPtr& LeftImage, const sensor_msgs::Image::ConstPtr& RightImage) 
{
    //获取halcon-bridge图像指针
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointerL = halcon_bridge::toHalconCopy(LeftImage);
    ho_ImageL = *halcon_bridge_imagePointerL->image;
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointerR = halcon_bridge::toHalconCopy(RightImage);
    ho_ImageR = *halcon_bridge_imagePointerR->image;
}

// service 回调函数，输入参数req，输出参数res
bool GetFirePosition(bit_vision::FirePosition::Request&  req,
                     bit_vision::FirePosition::Response& res)
{
  ROS_INFO("VisionAlgorithm:[%d]",req.cameraUsed);

  bool data_flag = false;     // 数据置信度

  // Local control variables
  HTuple  hv_MSecond, hv_Second, hv_Minute, hv_Hour;
  HTuple  hv_Day, hv_YDay, hv_Month, hv_Year;

  try
  {
    GetSystemTime(&hv_MSecond, &hv_Second, &hv_Minute, &hv_Hour, &hv_Day, &hv_YDay, &hv_Month, &hv_Year);
  
    HTuple fire_X(0);
    HTuple fire_Y(0);
    HTuple fire_Z(0);
    double delta_x = 0;
    double delta_y = 0;
    switch (req.cameraUsed)
    {
      case HandEye:
        fire_position_HandEye(ho_ImageL, delta_x, delta_y, data_flag);
        WriteImage(ho_ImageL, "jpeg", 0, ((((("/home/ugvcontrol/bit_mbzirc/src/bit_vision/image/IR/L"+hv_Month)+hv_Day)+hv_Hour)+hv_Minute)+hv_Second)+".jpg");
        break;
      case StereoEye:
        fire_position_StereoEye(ho_ImageL,ho_ImageR, fire_X, fire_Y, fire_Z, data_flag);
        WriteImage(ho_ImageL, "jpeg", 0, ((((("/home/ugvcontrol/bit_mbzirc/src/bit_vision/image/IR/L"+hv_Month)+hv_Day)+hv_Hour)+hv_Minute)+hv_Second)+".jpg");
        WriteImage(ho_ImageR, "jpeg", 0, ((((("/home/ugvcontrol/bit_mbzirc/src/bit_vision/image/IR/R"+hv_Month)+hv_Day)+hv_Hour)+hv_Minute)+hv_Second)+".jpg");
        break;
      default:
        break;
    }
    
    if (data_flag)
    {
      ROS_INFO_STREAM("Vision data:"<<delta_x<<","<<delta_y);
      res.flag = true;
      res.FirePos.header.stamp = ros::Time().now();
      res.FirePos.header.frame_id = "IRL_link";

      res.FirePos.point.x = delta_x;
      res.FirePos.point.y = delta_y;
      res.FirePos.point.z = 0;
    }
    else
    {
      res.flag = false;
      res.FirePos.header.stamp = ros::Time().now();
      res.FirePos.header.frame_id = "IRL_link";

      res.FirePos.point.x = 0.0;
      res.FirePos.point.y = 0.0;
      res.FirePos.point.z = 0.0;
    }
  }
  catch (HException &exception)
  {
    ROS_ERROR("44  Error #%u in %s: %s\n", exception.ErrorCode(),
            (const char *)exception.ProcName(),
            (const char *)exception.ErrorMessage());
    
  }
  // if (data_flag)
  // {
    
  //   ROS_INFO_STREAM("Vision data:"<<fire_X.D()<<","<<fire_Y.D()<<","<<fire_Z.D());

  //   tf::Transform transform_TargetOnIRL;
  //   transform_TargetOnIRL.setOrigin(tf::Vector3(fire_X.D(), fire_Y.D(), fire_Z.D()));
  //   tf::Quaternion q;
  //   q.setRPY(0, 0, 0);
  //   transform_TargetOnIRL.setRotation(q);

  //   tf::Transform transform_TargetOnBase = transform_IRLOnBase*transform_TargetOnIRL;

  //   ROS_INFO_STREAM("Vision data:"<<fire_X.D()<<","<<fire_Y.D()<<","<<fire_Z.D());

  //   res.flag = true;
  //   res.FirePos.header.stamp = ros::Time().now();
  //   res.FirePos.header.frame_id = "base_link";

  //   res.FirePos.point.x = transform_TargetOnBase.getOrigin().x();
  //   res.FirePos.point.y = transform_TargetOnBase.getOrigin().y();
  //   res.FirePos.point.z = transform_TargetOnBase.getOrigin().z();
  // }
  // else
  // {
  //   res.flag = false;
  //   res.FirePos.header.stamp = ros::Time().now();
  //   res.FirePos.header.frame_id = "base_link";

  //   res.FirePos.point.x = 0.0;
  //   res.FirePos.point.y = 0.0;
  //   res.FirePos.point.z = 0.0;
  // }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Task3_Vision_node");

  ros::NodeHandle nh;
  
  message_filters::Subscriber<sensor_msgs::Image> subArm(nh,"/cameraIR_arm/image",1);
  message_filters::Subscriber<sensor_msgs::Image> subCar(nh,"/cameraIR_arm/image",1);

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(subArm, subCar, 5);
  sync.registerCallback(boost::bind(&Imagecallback, _1, _2));

  // 服务-计算砖堆位置
  ros::ServiceServer service = nh.advertiseService("GetFireVisionData",GetFirePosition);

  ROS_INFO_STREAM("Ready to process fire position data");

  //指定循环的频率 
  ros::Rate loop_rate(20); 
  tf::TransformListener listener;
  
  while(ros::ok()) 
  { 
      // 获取 IRL_link 在 base_link下的坐标
      try{
      listener.lookupTransform("base_link", "IRL_link", ros::Time(0), transform_IRLOnBase);
      }
      catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      }
      
      //处理ROS的信息，比如订阅消息,并调用回调函数 
      ros::spinOnce(); 
      loop_rate.sleep(); 
  } 

  return 0;
}



