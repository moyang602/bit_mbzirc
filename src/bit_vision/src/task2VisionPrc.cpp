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
HTuple Brick_RX(0);
HTuple Brick_RY(0);
HTuple Brick_RZ(0);
tf::StampedTransform transform_ZedOnBase;

//分别把不同任务加过来
// 1.定位圆形标志的位置(二维)
void rectangle_pose(HObject ho_Image,HTuple &hv_X, HTuple &hv_Y, HTuple &hv_Z,HTuple &hv_RX, HTuple &hv_RY, HTuple &hv_RZ)
{

    // Local iconic variables
    HObject  ho_Image1, ho_Image2, ho_Image3;
    HObject  ho_ImageH, ho_ImageS, ho_ImageV, ho_Region, ho_Regions;
    HObject  ho_ConnectedRegions3, ho_SelectedRegions4, ho_ObjectSelectedL;
    HObject  ho_RegionClosing, ho_ContourL, ho_ContoursSplit;
    HObject  ho_SelectedEdges, ho_UnionContours2, ho_UnionContours;

    // Local control variables
    HTuple  hv_CamParam1, hv_RectWidth, hv_RectHeight;
    HTuple  hv_Second_0, hv_AbsoluteHisto, hv_RelativeHisto;
    HTuple  hv_MinThresh, hv_MaxThresh, hv_Second_1, hv_time1;
    HTuple  hv_REC, hv_index, hv_indexes_REC, hv_NumberContourSplit;
    HTuple  hv_PoseL, hv_CovPoseL, hv_ErrorL;
    HTuple  hv_Exception, hv_Seconds_3;
    HTuple  hv_Seconds_4;

    //鲁棒的定位图像中的白色矩形标志,并选择最右下角的矩形轮廓进行位姿估计
    ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar1_01.dat", &hv_CamParam1);

    hv_RectWidth = 0.3;
    hv_RectHeight = 0.17;

    Decompose3(ho_Image, &ho_Image1, &ho_Image2, &ho_Image3);
    TransFromRgb(ho_Image1, ho_Image2, ho_Image3, &ho_ImageH, &ho_ImageS, &ho_ImageV, 
    "hsv");
    //***********************************************************************************
    //选择灰度直方图两个波峰之间的波谷
    CountSeconds(&hv_Second_0);

    Threshold(ho_ImageS, &ho_Regions, 0, 50);
    CountSeconds(&hv_Second_1);
    hv_time1 = hv_Second_1-hv_Second_0;
    //************************************************************************************

    Connection(ho_Regions, &ho_ConnectedRegions3);
    SelectShape(ho_ConnectedRegions3, &ho_SelectedRegions4, (HTuple("rectangularity").Append("area")), 
    "and", (HTuple(0.85).Append(70000)), (HTuple(1).Append(100000000)));

    try
    {
      //对区域根据retangularity进行排序
      RegionFeatures(ho_SelectedRegions4, "rectangularity", &hv_REC);
      TupleSortIndex(hv_REC, &hv_index);
      TupleInverse(hv_index, &hv_indexes_REC);
      hv_index = HTuple(hv_indexes_REC[0])+1;
      SelectObj(ho_SelectedRegions4, &ho_ObjectSelectedL, hv_index);
      //闭运算
      ClosingCircle(ho_ObjectSelectedL, &ho_RegionClosing, 10);
      GenContourRegionXld(ho_RegionClosing, &ho_ContourL, "border");

      SegmentContoursXld(ho_ContourL, &ho_ContoursSplit, "lines", 7, 5, 3);
      SelectContoursXld(ho_ContoursSplit, &ho_SelectedEdges, "contour_length", 200, 
          2000, -0.5, 0.5);
      CountObj(ho_SelectedEdges, &hv_NumberContourSplit);
      GetRectanglePose(ho_ContourL, hv_CamParam1, 0.3, 0.17, "huber", 1, &hv_PoseL, 
          &hv_CovPoseL, &hv_ErrorL);
      hv_X = ((const HTuple&)hv_PoseL)[0];
      hv_Y = ((const HTuple&)hv_PoseL)[1];
      hv_Z = ((const HTuple&)hv_PoseL)[2];

      hv_RX = ((const HTuple&)hv_PoseL)[3];
      hv_RY = ((const HTuple&)hv_PoseL)[4];
      hv_RZ = ((const HTuple&)hv_PoseL)[5];
      data_flag = true;

    }

    catch (HException &HDevExpDefaultException)
    {
      HDevExpDefaultException.ToHTuple(&hv_Exception);
      try
      {
        //将共线的轮廓连接起来
        CountSeconds(&hv_Seconds_3);
        UnionCollinearContoursExtXld(ho_SelectedEdges, &ho_UnionContours2, 500, 10, 
            50, 0.7, 0, -1, 1, 1, 1, 1, 1, 0, "attr_keep");
        UnionAdjacentContoursXld(ho_UnionContours2, &ho_UnionContours, 10, 1, "attr_keep");
        GetRectanglePose(ho_UnionContours, hv_CamParam1, 0.3, 0.17, "huber", 1, &hv_PoseL, 
            &hv_CovPoseL, &hv_ErrorL);
        CountSeconds(&hv_Seconds_4);
        hv_X = ((const HTuple&)hv_PoseL)[0];
        hv_Y = ((const HTuple&)hv_PoseL)[1];
        hv_Z = ((const HTuple&)hv_PoseL)[2];

        hv_RX = ((const HTuple&)hv_PoseL)[3];
        hv_RY = ((const HTuple&)hv_PoseL)[4];
        hv_RZ = ((const HTuple&)hv_PoseL)[5];
        
        data_flag = true;
      }
        catch (HException &exception)
        {
        ROS_ERROR("11  Error #%u in %s: %s\n", exception.ErrorCode(),
                (const char *)exception.ProcName(),
                (const char *)exception.ErrorMessage());
        data_flag = false;
        }
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


// 4.根据颜色分类定位砖块位置
void brick_location(HObject ho_ImageL,HObject ho_ImageR,HTuple &hv_X, HTuple &hv_Y, HTuple &hv_Z)
{
  // Local iconic variables
  HObject  ho_ClassRegions;
  HObject  ho_ClassRegions2, ho_ClassRed, ho_ClassGreen, ho_ClassBLue;
  HObject  ho_ConnectedRegions1, ho_ConnectedRegions2, ho_ConnectedRegions3;
  HObject  ho_ObjectSelectedRed, ho_ObjectSelectedGreen, ho_ObjectSelectedBlue;
  HObject  ho_ClassRed2, ho_ClassGreen2, ho_ClassBLue2, ho_ConnectedRegions1_2;
  HObject  ho_ConnectedRegions2_2, ho_ConnectedRegions3_2;
  HObject  ho_ObjectSelectedRed_2, ho_ObjectSelectedGreen_2;
  HObject  ho_ObjectSelectedBlue_2;
  // Local control variables
  HTuple  hv_pathFile, hv_MLPHandle, hv_Area1, hv_Row1;
  HTuple  hv_Column1, hv_Indices, hv_num, hv_Area_1, hv_Row_1;
  HTuple  hv_Column_1, hv_Area2, hv_Row2, hv_Column2, hv_Area_2;
  HTuple  hv_Row_2, hv_Column_2, hv_Area3, hv_Row3, hv_Column3;
  HTuple  hv_Area_3, hv_Row_3, hv_Column_3, hv_areas, hv_rows;
  HTuple  hv_columns, hv_index, hv_class, hv_row_L, hv_column_L;
  HTuple  hv_Area1_2, hv_Row1_2, hv_Column1_2, hv_Indices_2;
  HTuple  hv_num2, hv_Area_1_2, hv_Row_1_2, hv_Column_1_2;
  HTuple  hv_Area_2_2, hv_Row_2_2, hv_Column_2_2, hv_Area3_2;
  HTuple  hv_Row3_2, hv_Column3_2, hv_Area_3_2, hv_Row_3_2;
  HTuple  hv_Column_3_2, hv_areas_2, hv_rows_2, hv_columns_2;
  HTuple  hv_index_2, hv_row_R, hv_column_R;
  HTuple  hv_Dist;
  HTuple hv_RelPose,hv_CamParam1, hv_CamParam2;

  try
    {
    ReadPose("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/relpose_01.dat", &hv_RelPose);
    ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar1_01.dat", &hv_CamParam1);
    ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar2_01.dat", &hv_CamParam2);

    hv_pathFile = "/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/box_segment_mlp_retrain.mlp";
    ReadClassMlp(hv_pathFile, &hv_MLPHandle);

    ClassifyImageClassMlp(ho_ImageL, &ho_ClassRegions, hv_MLPHandle, 0.9);

    SelectObj(ho_ClassRegions, &ho_ClassRed, 1);
    SelectObj(ho_ClassRegions, &ho_ClassGreen, 2);
    SelectObj(ho_ClassRegions, &ho_ClassBLue, 3);

    Connection(ho_ClassRed, &ho_ConnectedRegions1);
    Connection(ho_ClassGreen, &ho_ConnectedRegions2);
    Connection(ho_ClassBLue, &ho_ConnectedRegions3);

    AreaCenter(ho_ConnectedRegions1, &hv_Area1, &hv_Row1, &hv_Column1);
    TupleSortIndex(hv_Area1, &hv_Indices);
    hv_num = hv_Indices.TupleLength();
    SelectObj(ho_ConnectedRegions1, &ho_ObjectSelectedRed, HTuple(hv_Indices[hv_num-1])+1);
    AreaCenter(ho_ObjectSelectedRed, &hv_Area_1, &hv_Row_1, &hv_Column_1);

    AreaCenter(ho_ConnectedRegions2, &hv_Area2, &hv_Row2, &hv_Column2);
    TupleSortIndex(hv_Area2, &hv_Indices);
    hv_num = hv_Indices.TupleLength();
    SelectObj(ho_ConnectedRegions2, &ho_ObjectSelectedGreen, HTuple(hv_Indices[hv_num-1])+1);
    AreaCenter(ho_ObjectSelectedGreen, &hv_Area_2, &hv_Row_2, &hv_Column_2);

    AreaCenter(ho_ConnectedRegions3, &hv_Area3, &hv_Row3, &hv_Column3);
    TupleSortIndex(hv_Area3, &hv_Indices);
    hv_num = hv_Indices.TupleLength();
    SelectObj(ho_ConnectedRegions3, &ho_ObjectSelectedBlue, HTuple(hv_Indices[hv_num-1])+1);
    AreaCenter(ho_ObjectSelectedBlue, &hv_Area_3, &hv_Row_3, &hv_Column_3);

  //比较3种region的面积 面积最大的作为分类结果
    hv_areas.Clear();
    hv_areas.Append(hv_Area_1);
    hv_areas.Append(hv_Area_2);
    hv_areas.Append(hv_Area_3);
    //提取面积最大的区域对应的坐标
    hv_rows.Clear();
    hv_rows.Append(hv_Row_1);
    hv_rows.Append(hv_Row_2);
    hv_rows.Append(hv_Row_3);
    hv_columns.Clear();
    hv_columns.Append(hv_Column_1);
    hv_columns.Append(hv_Column_2);
    hv_columns.Append(hv_Column_3);

    TupleSortIndex(hv_areas, &hv_Indices);
    hv_num = hv_Indices.TupleLength();
    hv_index = HTuple(hv_Indices[hv_num-1]);

    if (0 != (hv_index==0))
    {
      hv_class = "red";
    }
    else if (0 != (hv_index==1))
    {
      hv_class = "green";
    }
    else if (0 != (hv_index==2))
    {
      hv_class = "blue";
    }

    //获取左图中的位置
    hv_row_L = HTuple(hv_rows[hv_index]);
    hv_column_L = HTuple(hv_columns[hv_index]);

    ROS_INFO_STREAM("Vision data:"<<hv_row_L.D()<<","<<hv_column_L.D());


    //分割右图
    ClassifyImageClassMlp(ho_ImageR, &ho_ClassRegions, hv_MLPHandle, 0.9);
    
    SelectObj(ho_ClassRegions, &ho_ClassRed, 1);
    SelectObj(ho_ClassRegions, &ho_ClassGreen, 2);
    SelectObj(ho_ClassRegions, &ho_ClassBLue, 3);

    Connection(ho_ClassRed, &ho_ConnectedRegions1);
    Connection(ho_ClassGreen, &ho_ConnectedRegions2);
    Connection(ho_ClassBLue, &ho_ConnectedRegions3);

    AreaCenter(ho_ConnectedRegions1, &hv_Area1, &hv_Row1, &hv_Column1);
    TupleSortIndex(hv_Area1, &hv_Indices);
    hv_num = hv_Indices.TupleLength();
    SelectObj(ho_ConnectedRegions1, &ho_ObjectSelectedRed, HTuple(hv_Indices[hv_num-1])+1);
    AreaCenter(ho_ObjectSelectedRed, &hv_Area_1, &hv_Row_1, &hv_Column_1);

    AreaCenter(ho_ConnectedRegions2, &hv_Area2, &hv_Row2, &hv_Column2);
    TupleSortIndex(hv_Area2, &hv_Indices);
    hv_num = hv_Indices.TupleLength();
    SelectObj(ho_ConnectedRegions2, &ho_ObjectSelectedGreen, HTuple(hv_Indices[hv_num-1])+1);
    AreaCenter(ho_ObjectSelectedGreen, &hv_Area_2, &hv_Row_2, &hv_Column_2);

    AreaCenter(ho_ConnectedRegions3, &hv_Area3, &hv_Row3, &hv_Column3);
    TupleSortIndex(hv_Area3, &hv_Indices);
    hv_num = hv_Indices.TupleLength();
    SelectObj(ho_ConnectedRegions3, &ho_ObjectSelectedBlue, HTuple(hv_Indices[hv_num-1])+1);
    AreaCenter(ho_ObjectSelectedBlue, &hv_Area_3, &hv_Row_3, &hv_Column_3);

    //比较3种region的面积 面积最大的作为分类结果
    hv_areas.Clear();
    hv_areas.Append(hv_Area_1);
    hv_areas.Append(hv_Area_2);
    hv_areas.Append(hv_Area_3);
    //提取面积最大的区域对应的坐标
    hv_rows.Clear();
    hv_rows.Append(hv_Row_1);
    hv_rows.Append(hv_Row_2);
    hv_rows.Append(hv_Row_3);
    hv_columns.Clear();
    hv_columns.Append(hv_Column_1);
    hv_columns.Append(hv_Column_2);
    hv_columns.Append(hv_Column_3);

    TupleSortIndex(hv_areas, &hv_Indices);
    hv_num = hv_Indices.TupleLength();
    hv_index = HTuple(hv_Indices[hv_num-1]);

    hv_row_R = HTuple(hv_rows[hv_index]);
    hv_column_R = HTuple(hv_columns[hv_index]);

    ROS_INFO_STREAM("Vision data:"<<hv_row_R.D()<<","<<hv_column_R.D());
  
    IntersectLinesOfSight(hv_CamParam1, hv_CamParam2, hv_RelPose, hv_row_L, hv_column_L, 
        hv_row_R, hv_column_R, &hv_X, &hv_Y, &hv_Z, &hv_Dist);
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

    // Local control variables
    HTuple  hv_MSecond, hv_Second, hv_Minute, hv_Hour;
    HTuple  hv_Day, hv_YDay, hv_Month, hv_Year;

    GetSystemTime(&hv_MSecond, &hv_Second, &hv_Minute, &hv_Hour, &hv_Day, &hv_YDay, &hv_Month, &hv_Year);

    WriteImage(ho_ImageL, "jpeg", 0, ((((("/home/ugvcontrol/bit_mbzirc/src/bit_vision/image/L"+hv_Month)+hv_Day)+hv_Hour)+hv_Minute)+hv_Second)+".jpg");
    WriteImage(ho_ImageR, "jpeg", 0, ((((("/home/ugvcontrol/bit_mbzirc/src/bit_vision/image/R"+hv_Month)+hv_Day)+hv_Hour)+hv_Minute)+hv_Second)+".jpg");

    switch (algorithm)
    {
        case GetBrickPos:
            rectangle_pose(ho_ImageL,Brick_X,Brick_Y,Brick_Z,Brick_RX,Brick_RY,Brick_RZ);
            break;
        case GetBrickAngle:
            rectangle_pose(ho_ImageL,Brick_X,Brick_Y,Brick_Z,Brick_RX,Brick_RY,Brick_RZ);
            break;
        case GetPutPos:
            //尚未加入L型架检测//目前是定位下面砖的圆形标志
            rectangle_pose(ho_ImageL,Brick_X,Brick_Y,Brick_Z,Brick_RX,Brick_RY,Brick_RZ);
            break;
        case GetPutAngle:
            put_brick(ho_ImageL);
            break;
        case GetLPose:
            put_brick(ho_ImageL);
            break;
        case GetBrickPos_only:
            brick_location(ho_ImageL,ho_ImageR,Brick_X,Brick_Y,Brick_Z);
            break;
        default:
            break;
    }

    if (data_flag)
    {
        tf::Transform transform_TargetOnZed;
        transform_TargetOnZed.setOrigin(tf::Vector3(Brick_X.D(), Brick_Y.D(), Brick_Z.D()));
        tf::Quaternion q;
        q.setRPY(0, 0, -Brick_RZ.D()/180.0*3.14159);
        transform_TargetOnZed.setRotation(q);
 
        tf::Transform transform3 = transform_ZedOnBase*transform_TargetOnZed;

        ROS_INFO_STREAM("Vision data:"<<Brick_X.D()<<","<<Brick_Y.D()<<","<<Brick_Z.D());
        ROS_INFO_STREAM("Vision angle:"<<Brick_RX.D()<<","<<Brick_RY.D()<<","<<Brick_RZ.D());
        // 返回目标在末端电磁铁坐标系下的位姿
        res.VisionData.header.stamp = ros::Time().now();
        res.VisionData.header.frame_id = "base_link";

        res.VisionData.Flag = true;
        res.VisionData.Pose.position.x = transform3.getOrigin().x()-0.017;
        if (algorithm == GetBrickPos||algorithm == GetBrickAngle)
        {
          res.VisionData.Pose.position.y = transform3.getOrigin().y()+0.040;
        }
        else if(algorithm == GetBrickPos_only)
        {
          res.VisionData.Pose.position.y = transform3.getOrigin().y();
        }
        else
        {
          res.VisionData.Pose.position.y = transform3.getOrigin().y();
        }
        res.VisionData.Pose.position.z = transform3.getOrigin().z();
        res.VisionData.Pose.orientation.x = 0.0;
        res.VisionData.Pose.orientation.y = 0.0;
        res.VisionData.Pose.orientation.z = Brick_RZ.D()/180.0*3.14159;//transform3.getRotation().getZ();

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