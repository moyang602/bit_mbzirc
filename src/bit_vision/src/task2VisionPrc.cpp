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
#include "tf/transform_broadcaster.h"
#include "bit_vision/VisionProc.h"
#include <opencv2/core/core.hpp>

#include<iostream>
#undef Success  
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace HalconCpp;
using namespace cv;

# define GetBrickPos        1   
# define GetBrickAngle      2
# define GetPutPos          3
# define GetPutAngle        4
# define GetLPose           5
# define GetBrickPos_only   6
# define NotRun             0
int algorithm = GetBrickPos;     // 当前算法
bool data_flag = false;     // 数据置信度

//取砖和放砖用同一个变量表示角度
HTuple brick_angle(0);
string brick_color = "green";
HTuple Brick_X(0);
HTuple Brick_Y(0);
HTuple Brick_Z(0);
tf::StampedTransform transform_ZedOnBase;

//分别把不同任务加过来
// 1.定位圆形标志的位置(二维)
void circle_location(HObject ho_ImageL,HObject ho_ImageR,HTuple &hv_X, HTuple &hv_Y, HTuple &hv_Z)
{
 // Local iconic variables
  HObject  ho_Image, ho_ClassRegions;
  HObject  ho_ClassRegion, ho_ImageReduced, ho_ImageMean, ho_Region;
  HObject  ho_RegionFillUp, ho_ConnectedRegions, ho_RegionErosion;
  HObject  ho_RegionDilation, ho_Objects, ho_ImageReduced1;
  HObject  ho_GrayImage, ho_ImageGauss, ho_ImageRoberts, ho_Regions;
  HObject  ho_ConnectedRegions2, ho_SelectedRegions, ho_RegionTrans;
  HObject  ho_Cross1, ho_ClassRegionsR, ho_ClassRegionR, ho_ImageReducedR;
  HObject  ho_ImageMeanR, ho_RegionR, ho_RegionFillUpR, ho_ConnectedRegionsR;
  HObject  ho_RegionErosionR, ho_RegionDilationR, ho_ObjectsR;
  HObject  ho_ImageReduced1R, ho_GrayImageR, ho_ImageGaussR;
  HObject  ho_ImageRobertsR, ho_RegionsR, ho_SelectedRegionsR;
  HObject  ho_RegionTransR, ho_Cross1R;

  // Local control variables
  HTuple  hv_RelPose, hv_CamParam1, hv_CamParam2;
  HTuple  hv_AcqHandle, hv_Width, hv_Height, hv_WindowHandle1;
  HTuple  hv_WindowHandle2, hv_pathFile, hv_MLPHandle, hv_color;
  HTuple  hv_index, hv_Number, hv_NumberCircle, hv_Area, hv_Row;
  HTuple  hv_Column, hv_Exception, hv_NumberR, hv_NumberCircleR;
  HTuple  hv_AreaR, hv_RowR, hv_ColumnR;
  HTuple  hv_Dist;

  //WriteImage(ho_ImageL, "png", 0, "/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/testImageL.png");
  //WriteImage(ho_ImageR, "png", 0, "/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/testImageR.png");
  //鲁棒的定位图像中的圆心,并能根据距离排序,选择距离最近的圆心

  //CropRectangle1(ho_Image1, &ho_ImageL, 0, 0, 1242, 2208);
  //CropRectangle1(ho_Image1, &ho_ImageR, 0, 2208, 1242, 4416);

  ReadPose("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/relpose_01.dat", &hv_RelPose);
  ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar1_01.dat", &hv_CamParam1);
  ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar2_01.dat", &hv_CamParam2);

  GetImageSize(ho_ImageL, &hv_Width, &hv_Height);

  //step1:根据颜色提取指定颜色的砖块区域
  hv_pathFile = "/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/box_segment_mlp_retrain.mlp";
  ReadClassMlp(hv_pathFile, &hv_MLPHandle);

  //分类砖块颜色
  ClassifyImageClassMlp(ho_ImageL, &ho_ClassRegions, hv_MLPHandle, 0.9);
  //比如接收指令拾取green砖块

  if (brick_color == "red")
  {
    hv_index = 1;
  }
  else if (brick_color == "green")
  {
    hv_index = 2;
  }
  else if (brick_color == "blue")
  {
    hv_index = 3;
  }

  SelectObj(ho_ClassRegions, &ho_ClassRegion, hv_index);

  //step2: color select
  ClosingCircle(ho_ClassRegion, &ho_ClassRegion, 10);
  ReduceDomain(ho_ImageL, ho_ClassRegion, &ho_ImageReduced);

  MeanImage(ho_ImageReduced, &ho_ImageMean, 20, 20);
  DynThreshold(ho_ImageL, ho_ImageMean, &ho_Region, 0, "light");
  FillUp(ho_Region, &ho_RegionFillUp);
  Connection(ho_RegionFillUp, &ho_ConnectedRegions);
  ErosionCircle(ho_ConnectedRegions, &ho_RegionErosion, 3.5);
  DilationCircle(ho_RegionErosion, &ho_RegionDilation, 3.5);
  SelectShape(ho_RegionDilation, &ho_Objects, ((HTuple("area").Append("convexity")).Append("rectangularity")), 
      "and", ((HTuple(100000).Append(0.8)).Append(0.8)), ((HTuple(6000000).Append(1)).Append(1)));
  CountObj(ho_Objects, &hv_Number);

  //step3: 提取ROI区域用于轮廓提取

  ReduceDomain(ho_ImageL, ho_Objects, &ho_ImageReduced1);
  Rgb1ToGray(ho_ImageReduced1, &ho_GrayImage);
  GaussFilter(ho_GrayImage, &ho_ImageGauss, 7);
  ReduceDomain(ho_ImageGauss, ho_Objects, &ho_ImageReduced);
  Roberts(ho_ImageGauss, &ho_ImageRoberts, "gradient_sum");
  Threshold(ho_ImageRoberts, &ho_Regions, 0, 28);
  Connection(ho_Regions, &ho_ConnectedRegions2);
  SelectShape(ho_ConnectedRegions2, &ho_SelectedRegions, (HTuple("area").Append("circularity")), 
      "and", (HTuple(8000).Append(0.7)), (HTuple(100000).Append(1)));
  CountObj(ho_SelectedRegions, &hv_NumberCircle);
  ShapeTrans(ho_SelectedRegions, &ho_RegionTrans, "outer_circle");
  AreaCenter(ho_RegionTrans, &hv_Area, &hv_Row, &hv_Column);
  GenCrossContourXld(&ho_Cross1, hv_Row, hv_Column, 60, 0.785398);

  //分类砖块颜色
  ClassifyImageClassMlp(ho_ImageR, &ho_ClassRegionsR, hv_MLPHandle, 0.9);
  //比如接收指令拾取green砖块
  SelectObj(ho_ClassRegionsR, &ho_ClassRegionR, hv_index);

  //step2: color select
  ClosingCircle(ho_ClassRegionR, &ho_ClassRegionR, 10);
  ReduceDomain(ho_ImageR, ho_ClassRegionR, &ho_ImageReducedR);

  MeanImage(ho_ImageReducedR, &ho_ImageMeanR, 20, 20);
  DynThreshold(ho_ImageR, ho_ImageMeanR, &ho_RegionR, 0, "light");
  FillUp(ho_RegionR, &ho_RegionFillUpR);
  Connection(ho_RegionFillUpR, &ho_ConnectedRegionsR);
  ErosionCircle(ho_ConnectedRegionsR, &ho_RegionErosionR, 3.5);
  DilationCircle(ho_RegionErosionR, &ho_RegionDilationR, 3.5);
  SelectShape(ho_RegionDilationR, &ho_ObjectsR, ((HTuple("area").Append("convexity")).Append("rectangularity")), 
      "and", ((HTuple(100000).Append(0.8)).Append(0.8)), ((HTuple(6000000).Append(1)).Append(1)));
  CountObj(ho_ObjectsR, &hv_NumberR);

  //step3: 提取ROI区域用于轮廓提取
  ReduceDomain(ho_ImageR, ho_ObjectsR, &ho_ImageReduced1R);
  Rgb1ToGray(ho_ImageReduced1R, &ho_GrayImageR);
  GaussFilter(ho_GrayImageR, &ho_ImageGaussR, 7);
  ReduceDomain(ho_ImageGaussR, ho_ObjectsR, &ho_ImageReducedR);
  Roberts(ho_ImageGaussR, &ho_ImageRobertsR, "gradient_sum");
  Threshold(ho_ImageRobertsR, &ho_RegionsR, 0, 28);
  Connection(ho_RegionsR, &ho_ConnectedRegionsR);
  SelectShape(ho_ConnectedRegionsR, &ho_SelectedRegionsR, (HTuple("area").Append("circularity")), 
      "and", (HTuple(8000).Append(0.7)), (HTuple(100000).Append(1)));
  CountObj(ho_SelectedRegionsR, &hv_NumberCircleR);
  ShapeTrans(ho_SelectedRegionsR, &ho_RegionTransR, "outer_circle");
  AreaCenter(ho_RegionTransR, &hv_AreaR, &hv_RowR, &hv_ColumnR);
  GenCrossContourXld(&ho_Cross1R, hv_RowR, hv_ColumnR, 60, 0.785398);

  if (0 != (HTuple(hv_NumberCircle==1).TupleAnd(hv_NumberCircleR==1)))
  {
    try
    {
      IntersectLinesOfSight(hv_CamParam1, hv_CamParam2, hv_RelPose, hv_Row, hv_Column, 
          hv_RowR, hv_ColumnR, &hv_X, &hv_Y, &hv_Z, &hv_Dist);
      data_flag = true;
    }
    catch (HException &exception)
    {
      ROS_ERROR("44  Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
      data_flag = false;
      return ;
    }
  }
}



//2.拾取砖块角度估计
void pick_brick(HObject ho_Image)
{

  // Local iconic variables
  HObject  ho_ClassRegions, ho_ClassRegion;
  HObject  ho_ImageReduced, ho_grayImage, ho_Bright, ho_fillRegion;
  HObject  ho_RegionOpening, ho_ConnectedRegions2, ho_SelectedRegions;
  HObject  ho_Contours;

  // Local control variables
  HTuple  hv_CamParam, hv_pathFile, hv_MLPHandle;
  HTuple  hv_Width, hv_Height, hv_WindowHandle, hv_color;
  HTuple  hv_index, hv_Number1, hv_Phi, hv_angle, hv_BrickPose;
  HTuple  hv_CovPose, hv_Error;

  try
  {
    //step1 :读入相机标定参数
    ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar1_01.dat", &hv_CamParam);
    //step2:读入训练好的分割模型
    hv_pathFile = "/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/box_segment_mlp_retrain.mlp";
    ReadClassMlp(hv_pathFile, &hv_MLPHandle);
    //step3:读图 并 分割图像
    GetImageSize(ho_Image, &hv_Width, &hv_Height);
    //SetWindowAttr("background_color","black");
    //OpenWindow(0,0,hv_Width*0.5,hv_Height*0.5,0,"visible","",&hv_WindowHandle);
    //HDevWindowStack::Push(hv_WindowHandle);
    //分类砖块颜色
    ClassifyImageClassMlp(ho_Image, &ho_ClassRegions, hv_MLPHandle, 0.9);
    //比如接收指令拾取green砖块
    //接收砖块颜色指令
    if (brick_color=="red")
    {
      hv_index = 1;
    }
    else if (brick_color=="green")
    {
      hv_index = 2;
    }
    else if (brick_color=="blue")
    {
      hv_index = 3;
    }
    else if (brick_color=="orange")
    {
      hv_index = 4;
    }

    SelectObj(ho_ClassRegions, &ho_ClassRegion, hv_index);
    //step4: 基于分割结果分割图像并进行开闭运算预处理
    OpeningCircle(ho_ClassRegion, &ho_ClassRegion, 5.5);
    ReduceDomain(ho_Image, ho_ClassRegion, &ho_ImageReduced);
    //step5:以分割结果作为输入
    Rgb1ToGray(ho_ImageReduced, &ho_grayImage);
    //阈值可能需要具体实验的时候调节一下
    Threshold(ho_grayImage, &ho_Bright, 134, 201);
    FillUp(ho_Bright, &ho_fillRegion);
    OpeningCircle(ho_fillRegion, &ho_RegionOpening, 5.5);
    Connection(ho_RegionOpening, &ho_ConnectedRegions2);
    CountObj(ho_ConnectedRegions2, &hv_Number1);

    SelectShape(ho_ConnectedRegions2, &ho_SelectedRegions, (HTuple("rectangularity").Append("area")), 
        "and", (HTuple(0.9).Append(80000)), (HTuple(1).Append(1000000)));
    CountObj(ho_SelectedRegions, &hv_Number1);

    //step6:计算矩形的角度
    //select_shape_std (SelectedRegions, SelectedRegions, 'rectangle2', 90)
    OrientationRegion(ho_SelectedRegions, &hv_Phi);

    //将角度控制在-pi/2到pi/2之间
    if (hv_Phi<(-0.5*3.14)&&hv_Phi>(-3.14))
    {
      hv_Phi[0] = 3.14+HTuple(hv_Phi[0]);
    }
    else if (hv_Phi>(0.5*3.14)&&hv_Phi<3.14)
    {
      hv_Phi[0] = HTuple(hv_Phi[0])-3.14;
    }

    brick_angle = (hv_Phi[0]*180)/3.14;
    //ROS_INFO("brick_angle = %lf",brick_angle.D());

    data_flag = true;
  }
  catch (HException &exception)
  {
    ROS_ERROR("22  Error #%u in %s: %s\n", exception.ErrorCode(),
            (const char *)exception.ProcName(),
            (const char *)exception.ErrorMessage());
    data_flag = false;

    return;
  }
   
}


// 3.放砖角度估计
void put_brick(HObject ho_Image1)
{

  // Local iconic variables
  HObject  ho_ClassRegions, ho_BrickRegion;
  HObject  ho_RegionOpening, ho_ImageReduced, ho_brick, ho_SelectedContours;
  HObject  ho_SortedContours, ho_Line1, ho_grayImage, ho_Bright_2;
  HObject  ho_fillRegion, ho_ConnectedRegions2, ho_SelectedRegions;
  HObject  ho_Contours;

  // Local control variables
  HTuple  hv_WindowHandle, hv_pathFile, hv_MLPHandle;
  HTuple  hv_CameraParam, hv_Width, hv_Height, hv_brickcolor;
  HTuple  hv_index, hv_Area, hv_Row, hv_Column, hv_PI, hv_Number;
  HTuple  hv_Attrib, hv_RowBegin, hv_ColBegin, hv_RowEnd;
  HTuple  hv_ColEnd, hv_Nr1, hv_Nc1, hv_Dist1, hv_angle_brick;
  HTuple  hv_rotZ_1, hv_Number2, hv_Phi_2, hv_rot_Z_2;

  //计算最上直线的角度
  try
  {
    
    //读入训练好的分割模型
    hv_pathFile = "/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/box_segment_mlp_retrain.mlp";
    ReadClassMlp(hv_pathFile, &hv_MLPHandle);
    ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar1_01.dat", 
        &hv_CameraParam);
    //读入第一张图像 用于识别砖块的轮廓
    GetImageSize(ho_Image1, &hv_Width, &hv_Height);

    ClassifyImageClassMlp(ho_Image1, &ho_ClassRegions, hv_MLPHandle, 0.5);
    //基于先前举起砖块时做的颜色分类结果 先选择砖块对应的区域

    if (0 != (brick_color=="red"))
    {
      hv_index = 1;
    }
    else if (0 != (brick_color=="green"))
    {
      hv_index = 2;
    }
    else if (0 != (brick_color=="blue"))
    {
      hv_index = 3;
    }
    SelectObj(ho_ClassRegions, &ho_BrickRegion, hv_index);
    //
    //开闭运算
    OpeningCircle(ho_BrickRegion, &ho_RegionOpening, 3.5);
    AreaCenter(ho_RegionOpening, &hv_Area, &hv_Row, &hv_Column);
    //
    ReduceDomain(ho_Image1, ho_RegionOpening, &ho_ImageReduced);
    //
    GenContourRegionXld(ho_ImageReduced, &ho_brick, "border");

    //*****************************************************************************
    //提取剩余区域的轮廓
    hv_PI = 3.14;
    SegmentContoursXld(ho_brick, &ho_SelectedContours, "lines", 2, 4, 1);
    //
    CountObj(ho_SelectedContours, &hv_Number);
    ROS_INFO("Line Number = %lf",hv_Number.D());
    //如果检测到的直线个数为0 则处理下一张图像
    //根据长度筛选直线
    SelectContoursXld(ho_SelectedContours, &ho_SelectedContours, "contour_length", 
        hv_Width/8, 2*hv_Width, -0.5, 0.5);
    CountObj(ho_SelectedContours, &hv_Number);
    //根据视野进行线段的筛选
    SortContoursXld(ho_SelectedContours, &ho_SortedContours, "upper_left", "true", 
        "row");
    GenEmptyObj(&ho_Line1);
    SelectObj(ho_SortedContours, &ho_Line1, 1);
    //拟合拾取砖块的上边缘直线方程
    GetContourGlobalAttribXld(ho_Line1, "cont_approx", &hv_Attrib);
    if (0 != (hv_Attrib==-1))
    {
      //得到直线上两个点的参数
      FitLineContourXld(ho_Line1, "tukey", 2, 0, 5, 2, &hv_RowBegin, &hv_ColBegin, 
          &hv_RowEnd, &hv_ColEnd, &hv_Nr1, &hv_Nc1, &hv_Dist1);
    }
    //
    hv_angle_brick = hv_Nc1.TupleAtan2(hv_Nr1);
    //约束角度在-pi/2-pi/2区间内
    if (0 != (hv_angle_brick<(-0.5*3.14)))
    {
      hv_angle_brick[0] = 3.14+HTuple(hv_angle_brick[0]);
    }
    else if (0 != (hv_angle_brick>(0.5*3.14)))
    {
      hv_angle_brick[0] = HTuple(hv_angle_brick[0])-3.14;
    }

    //角度需要与三维空间中的位姿建立映射关系
    //此时以弧度为单位
    hv_rotZ_1 = (hv_angle_brick[0]*180)/3.14;
    //*****************************************************************************************************
    //方法二 : 与拾取砖块的过程类似
    //step5:以砖块颜色分割结果作为输入
    Rgb1ToGray(ho_ImageReduced, &ho_grayImage);
    //阈值可能需要具体实验的时候调节一下
    Threshold(ho_grayImage, &ho_Bright_2, 134, 201);
    FillUp(ho_Bright_2, &ho_fillRegion);
    OpeningCircle(ho_fillRegion, &ho_RegionOpening, 5.5);
    Connection(ho_RegionOpening, &ho_ConnectedRegions2);
    CountObj(ho_ConnectedRegions2, &hv_Number2);
    SelectShape(ho_ConnectedRegions2, &ho_SelectedRegions, (HTuple("rectangularity").Append("area")), 
        "and", (HTuple(0.9).Append(80000)), (HTuple(1).Append(1000000)));
    CountObj(ho_SelectedRegions, &hv_Number2);
    //计算矩形的角度
    OrientationRegion(ho_SelectedRegions, &hv_Phi_2);
    //限制角度在-pi/2到pi/2区间内
    if (0 != (hv_Phi_2<(-0.5*3.14)))
    {
      hv_Phi_2[0] = 3.14+HTuple(hv_Phi_2[0]);
    }
    else if (0 != (hv_Phi_2>(0.5*3.14)))
    {
      hv_Phi_2[0] = HTuple(hv_Phi_2[0])-3.14;
    }
    hv_rot_Z_2 = (hv_Phi_2*180)/3.14;
    //提取区域轮廓 计算矩形在相机坐标系下的pose
    GenContourRegionXld(ho_SelectedRegions, &ho_Contours, "border");

    //需要增加判断是否为空(无检测结果)输出单位是角度
    brick_angle = hv_rot_Z_2;
    ROS_INFO("brick_angle = %lf",brick_angle.D());
    //brick_angle = hv_Phi_2;
    Brick_X = 0.0;
    Brick_Y = 0.0;
    Brick_Z = 0.0;
    data_flag = true;
  }
  catch (HException &exception)
  {
    ROS_ERROR("33  Error #%u in %s: %s\n", exception.ErrorCode(),
            (const char *)exception.ProcName(),
            (const char *)exception.ErrorMessage());
    data_flag = false;

    return;
  }

}

// 4.定位圆形标志的位置(二维)
void circle_location_only(HObject ho_ImageL,HObject ho_ImageR,HTuple &hv_X, HTuple &hv_Y, HTuple &hv_Z)
{
 // Local iconic variables
  HObject  ho_Image, ho_ClassRegions;
  HObject  ho_ClassRegion, ho_ImageReduced, ho_ImageMean, ho_Region;
  HObject  ho_RegionFillUp, ho_ConnectedRegions, ho_RegionErosion;
  HObject  ho_RegionDilation, ho_Objects, ho_ImageReduced1;
  HObject  ho_GrayImage, ho_ImageGauss, ho_ImageRoberts, ho_Regions;
  HObject  ho_ConnectedRegions2, ho_SelectedRegions, ho_RegionTrans;
  HObject  ho_Cross1, ho_ClassRegionsR, ho_ClassRegionR, ho_ImageReducedR;
  HObject  ho_ImageMeanR, ho_RegionR, ho_RegionFillUpR, ho_ConnectedRegionsR;
  HObject  ho_RegionErosionR, ho_RegionDilationR, ho_ObjectsR;
  HObject  ho_ImageReduced1R, ho_GrayImageR, ho_ImageGaussR;
  HObject  ho_ImageRobertsR, ho_RegionsR, ho_SelectedRegionsR;
  HObject  ho_RegionTransR, ho_Cross1R;

  // Local control variables
  HTuple  hv_RelPose, hv_CamParam1, hv_CamParam2;
  HTuple  hv_AcqHandle, hv_Width, hv_Height, hv_WindowHandle1;
  HTuple  hv_WindowHandle2, hv_pathFile, hv_MLPHandle, hv_color;
  HTuple  hv_index, hv_Number, hv_NumberCircle, hv_Area, hv_Row;
  HTuple  hv_Column, hv_Exception, hv_NumberR, hv_NumberCircleR;
  HTuple  hv_AreaR, hv_RowR, hv_ColumnR;
  HTuple  hv_Dist;

  ReadPose("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/relpose_01.dat", &hv_RelPose);
  ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar1_01.dat", &hv_CamParam1);
  ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar2_01.dat", &hv_CamParam2);

  GetImageSize(ho_ImageL, &hv_Width, &hv_Height);


  //step3: 提取ROI区域用于轮廓提取
  Rgb1ToGray(ho_ImageL, &ho_GrayImage);
  GaussFilter(ho_GrayImage, &ho_ImageGauss, 7);
  Roberts(ho_ImageGauss, &ho_ImageRoberts, "gradient_sum");
  Threshold(ho_ImageRoberts, &ho_Regions, 0, 20);
  Connection(ho_Regions, &ho_ConnectedRegions2);
  SelectShape(ho_ConnectedRegions2, &ho_SelectedRegions, (HTuple("area").Append("circularity")), 
      "and", (HTuple(50000).Append(0.7)), (HTuple(1000000).Append(1)));
  CountObj(ho_SelectedRegions, &hv_NumberCircle);
  ShapeTrans(ho_SelectedRegions, &ho_RegionTrans, "outer_circle");
  AreaCenter(ho_RegionTrans, &hv_Area, &hv_Row, &hv_Column);
  GenCrossContourXld(&ho_Cross1, hv_Row, hv_Column, 60, 0.785398);

  //step3: 提取ROI区域用于轮廓提取
  Rgb1ToGray(ho_ImageR, &ho_GrayImageR);
  GaussFilter(ho_GrayImageR, &ho_ImageGaussR, 7);
  Roberts(ho_ImageGaussR, &ho_ImageRobertsR, "gradient_sum");
  Threshold(ho_ImageRobertsR, &ho_RegionsR, 0, 20);
  Connection(ho_RegionsR, &ho_ConnectedRegionsR);
  SelectShape(ho_ConnectedRegionsR, &ho_SelectedRegionsR, (HTuple("area").Append("circularity")), 
      "and", (HTuple(50000).Append(0.7)), (HTuple(1000000).Append(1)));
  CountObj(ho_SelectedRegionsR, &hv_NumberCircleR);
  ShapeTrans(ho_SelectedRegionsR, &ho_RegionTransR, "outer_circle");
  AreaCenter(ho_RegionTransR, &hv_AreaR, &hv_RowR, &hv_ColumnR);
  GenCrossContourXld(&ho_Cross1R, hv_RowR, hv_ColumnR, 60, 0.785398);

  if (0 != (HTuple(hv_NumberCircle==1).TupleAnd(hv_NumberCircleR==1)))
  {
    try
    {
      IntersectLinesOfSight(hv_CamParam1, hv_CamParam2, hv_RelPose, hv_Row, hv_Column, 
          hv_RowR, hv_ColumnR, &hv_X, &hv_Y, &hv_Z, &hv_Dist);
      data_flag = true;
    }
    catch (HException &exception)
    {
      ROS_ERROR("44  Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
      data_flag = false;
      return ;
    }
  }
}


//初始化halcon对象
HObject  ho_ImageL, ho_ImageR;

//回调函数
void callback(const sensor_msgs::Image::ConstPtr& LeftImage, const sensor_msgs::Image::ConstPtr& RightImage) 
{
    
    //获取halcon-bridge图像指针
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointerL = halcon_bridge::toHalconCopy(LeftImage);
    ho_ImageL = *halcon_bridge_imagePointerL->image;
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointerR = halcon_bridge::toHalconCopy(RightImage);
    ho_ImageR = *halcon_bridge_imagePointerR->image;
    
    
    /*****************************************************
    *                   开始图像处理程序
    *****************************************************/
    
    
}

bool GetVisionData(bit_vision::VisionProc::Request&  req,
                   bit_vision::VisionProc::Response& res)
{
    ROS_INFO("BrickType:[%s], VisionAlgorithm:[%d]",req.BrickType.c_str(),req.ProcAlgorithm);
    // 设置视觉处理颜色与算法
    brick_color = req.BrickType;
    algorithm = req.ProcAlgorithm;
    data_flag = false;

    switch (algorithm)
    {
        case GetBrickPos:
            circle_location(ho_ImageL,ho_ImageR,Brick_X,Brick_Y,Brick_Z);
            break;
        case GetBrickAngle:
            circle_location(ho_ImageL,ho_ImageR,Brick_X,Brick_Y,Brick_Z);
            pick_brick(ho_ImageL); 
            break;
        case GetPutPos:
            //尚未加入L型架检测//目前是定位下面砖的圆形标志
            circle_location(ho_ImageL,ho_ImageR,Brick_X, Brick_Y,Brick_Z);
            break;
        case GetPutAngle:
            put_brick(ho_ImageL);
            break;
        case GetLPose:
            put_brick(ho_ImageL);
            break;
        case GetBrickPos_only:
            circle_location_only(ho_ImageL,ho_ImageR,Brick_X,Brick_Y,Brick_Z);
            break;
        default:
            break;
    }

    if (data_flag)
    {
        tf::Transform transform_TargetOnZed;
        transform_TargetOnZed.setOrigin(tf::Vector3(Brick_X.D(), Brick_Y.D(), Brick_Z.D()));
        tf::Quaternion q;
        q.setRPY(0, 0, -brick_angle.D()/180.0*3.14159);
        transform_TargetOnZed.setRotation(q);
 
        tf::Transform transform3 = transform_ZedOnBase*transform_TargetOnZed;

        ROS_INFO_STREAM("Vision data:"<<Brick_X.D()<<","<<Brick_Y.D()<<","<<Brick_Z.D());
        ROS_INFO_STREAM("Vision angle:"<<brick_angle.D());
        // 返回目标在末端电磁铁坐标系下的位姿
        res.VisionData.header.stamp = ros::Time().now();
        res.VisionData.header.frame_id = "base_link";

        res.VisionData.Flag = true;
        res.VisionData.Pose.position.x = transform3.getOrigin().x();
        if (algorithm == GetBrickPos||algorithm == GetBrickAngle)
        {
          res.VisionData.Pose.position.y = transform3.getOrigin().y()+0.035;
        }
        else if(algorithm == GetBrickPos_only)
        {
          res.VisionData.Pose.position.y = transform3.getOrigin().y()+0.015;
        }
        else
        {
          res.VisionData.Pose.position.y = transform3.getOrigin().y();
        }
        res.VisionData.Pose.position.z = transform3.getOrigin().z();
        res.VisionData.Pose.orientation.x = 0.0;
        res.VisionData.Pose.orientation.y = 0.0;
        res.VisionData.Pose.orientation.z = brick_angle.D()/180.0*3.14159;//transform3.getRotation().getZ();

    }
    else    // 如果没有识别结果
    {
        res.VisionData.header.stamp = ros::Time().now();
        res.VisionData.header.frame_id = "zed_link";

        res.VisionData.Flag = false;
        res.VisionData.Pose.position.x = 0.0;
        res.VisionData.Pose.position.y = 0.0;
        res.VisionData.Pose.position.z = 0.0;
        res.VisionData.Pose.orientation.x = 0.0;
        res.VisionData.Pose.orientation.y = 0.0;
        res.VisionData.Pose.orientation.z = 0.0;
    }

    algorithm = NotRun;
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Task2_Vision_node");

  ros::NodeHandle nh; 

  message_filters::Subscriber<sensor_msgs::Image> subleft(nh,"/zed/zed_node/left/image_rect_color",1);
  message_filters::Subscriber<sensor_msgs::Image> subRight(nh,"/zed/zed_node/right/image_rect_color",1);

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(subleft, subRight, 5);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::ServiceServer service = nh.advertiseService("GetVisionData",GetVisionData);

  ROS_INFO_STREAM("Ready to process vision data");

  //指定循环的频率 
  ros::Rate loop_rate(20); 
  tf::TransformListener listener;
  
  while(ros::ok()) 
  { 
      // 获取 zed_link 在 base_link下的坐标
      try{
      listener.lookupTransform("base_link", "zed_link", ros::Time(0), transform_ZedOnBase);
      }
      catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      }
      
      //处理ROS的信息，比如订阅消息,并调用回调函数 
      ros::spinOnce(); 
      loop_rate.sleep(); 
  } 

  //ros::MultiThreadedSpinner spinner(3); // Use 4 threads
  //spinner.spin();
  
  return 0;
}