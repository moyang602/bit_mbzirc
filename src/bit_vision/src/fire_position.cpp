///////////////////////////////////////////////////////////////////////////////
// File generated by HDevelop for HALCON/C++ Version 18.11.1.1
// Non-ASCII strings in this file are encoded in local-8-bit encoding (cp936).
// Ensure that the interface encoding is set to locale encoding by calling
// SetHcppInterfaceStringEncodingIsUtf8(false) at the beginning of the program.
// 
// Please note that non-ASCII characters in string constants are exported
// as octal codes in order to guarantee that the strings are correctly
// created on all systems, independent on any compiler settings.
// 
// Source files with different encoding should not be mixed in one project.
///////////////////////////////////////////////////////////////////////////////



#ifndef __APPLE__
#  include "HalconCpp.h"
#  include "HDevThread.h"
#  if defined(__linux__) && (defined(__i386__) || defined(__x86_64__)) \
                         && !defined(NO_EXPORT_APP_MAIN)
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
#include "bit_vision/LocateInfo.h"

using namespace std;
using namespace HalconCpp;

ros::Publisher pub;
bit_vision::LocateInfo firelocateInfo;

using namespace HalconCpp;

// Procedure declarations 
// External procedures 
// Chapter: Develop
// Short Description: Open a new graphics window that preserves the aspect ratio of the given image. 
void dev_open_window_fit_image (HObject ho_Image, HTuple hv_Row, HTuple hv_Column, 
    HTuple hv_WidthLimit, HTuple hv_HeightLimit, HTuple *hv_WindowHandle);
// Chapter: Develop
// Short Description: Switch dev_update_pc, dev_update_var and dev_update_window to 'off'. 
void dev_update_off ();
// Chapter: Graphics / Text
// Short Description: This procedure writes a text message. 
void disp_message (HTuple hv_WindowHandle, HTuple hv_String, HTuple hv_CoordSystem, 
    HTuple hv_Row, HTuple hv_Column, HTuple hv_Color, HTuple hv_Box);
// Chapter: Graphics / Text
// Short Description: Set font independent of OS 
void set_display_font (HTuple hv_WindowHandle, HTuple hv_Size, HTuple hv_Font, HTuple hv_Bold, 
    HTuple hv_Slant);

// Procedures 
// External procedures 
// Chapter: Develop
// Short Description: Open a new graphics window that preserves the aspect ratio of the given image. 
void dev_open_window_fit_image (HObject ho_Image, HTuple hv_Row, HTuple hv_Column, 
    HTuple hv_WidthLimit, HTuple hv_HeightLimit, HTuple *hv_WindowHandle)
{

  // Local iconic variables

  // Local control variables
  HTuple  hv_MinWidth, hv_MaxWidth, hv_MinHeight;
  HTuple  hv_MaxHeight, hv_ResizeFactor, hv_ImageWidth, hv_ImageHeight;
  HTuple  hv_TempWidth, hv_TempHeight, hv_WindowWidth, hv_WindowHeight;

  //This procedure opens a new graphics window and adjusts the size
  //such that it fits into the limits specified by WidthLimit
  //and HeightLimit, but also maintains the correct image aspect ratio.
  //
  //If it is impossible to match the minimum and maximum extent requirements
  //at the same time (f.e. if the image is very long but narrow),
  //the maximum value gets a higher priority,
  //
  //Parse input tuple WidthLimit
  if (0 != (HTuple((hv_WidthLimit.TupleLength())==0).TupleOr(hv_WidthLimit<0)))
  {
    hv_MinWidth = 500;
    hv_MaxWidth = 800;
  }
  else if (0 != ((hv_WidthLimit.TupleLength())==1))
  {
    hv_MinWidth = 0;
    hv_MaxWidth = hv_WidthLimit;
  }
  else
  {
    hv_MinWidth = ((const HTuple&)hv_WidthLimit)[0];
    hv_MaxWidth = ((const HTuple&)hv_WidthLimit)[1];
  }
  //Parse input tuple HeightLimit
  if (0 != (HTuple((hv_HeightLimit.TupleLength())==0).TupleOr(hv_HeightLimit<0)))
  {
    hv_MinHeight = 400;
    hv_MaxHeight = 600;
  }
  else if (0 != ((hv_HeightLimit.TupleLength())==1))
  {
    hv_MinHeight = 0;
    hv_MaxHeight = hv_HeightLimit;
  }
  else
  {
    hv_MinHeight = ((const HTuple&)hv_HeightLimit)[0];
    hv_MaxHeight = ((const HTuple&)hv_HeightLimit)[1];
  }
  //
  //Test, if window size has to be changed.
  hv_ResizeFactor = 1;
  GetImageSize(ho_Image, &hv_ImageWidth, &hv_ImageHeight);
  //First, expand window to the minimum extents (if necessary).
  if (0 != (HTuple(hv_MinWidth>hv_ImageWidth).TupleOr(hv_MinHeight>hv_ImageHeight)))
  {
    hv_ResizeFactor = (((hv_MinWidth.TupleReal())/hv_ImageWidth).TupleConcat((hv_MinHeight.TupleReal())/hv_ImageHeight)).TupleMax();
  }
  hv_TempWidth = hv_ImageWidth*hv_ResizeFactor;
  hv_TempHeight = hv_ImageHeight*hv_ResizeFactor;
  //Then, shrink window to maximum extents (if necessary).
  if (0 != (HTuple(hv_MaxWidth<hv_TempWidth).TupleOr(hv_MaxHeight<hv_TempHeight)))
  {
    hv_ResizeFactor = hv_ResizeFactor*((((hv_MaxWidth.TupleReal())/hv_TempWidth).TupleConcat((hv_MaxHeight.TupleReal())/hv_TempHeight)).TupleMin());
  }
  hv_WindowWidth = hv_ImageWidth*hv_ResizeFactor;
  hv_WindowHeight = hv_ImageHeight*hv_ResizeFactor;
  //Resize window
  SetWindowAttr("background_color","black");
  OpenWindow(hv_Row,hv_Column,hv_WindowWidth,hv_WindowHeight,0,"visible","",&(*hv_WindowHandle));
  HDevWindowStack::Push((*hv_WindowHandle));
  if (HDevWindowStack::IsOpen())
    SetPart(HDevWindowStack::GetActive(),0, 0, hv_ImageHeight-1, hv_ImageWidth-1);
  return;
}

// Chapter: Develop
// Short Description: Switch dev_update_pc, dev_update_var and dev_update_window to 'off'. 
void dev_update_off ()
{

  //This procedure sets different update settings to 'off'.
  //This is useful to get the best performance and reduce overhead.
  //
  // dev_update_pc(...); only in hdevelop
  // dev_update_var(...); only in hdevelop
  // dev_update_window(...); only in hdevelop
  return;
}

// Chapter: Graphics / Text
// Short Description: This procedure writes a text message. 
void disp_message (HTuple hv_WindowHandle, HTuple hv_String, HTuple hv_CoordSystem, 
    HTuple hv_Row, HTuple hv_Column, HTuple hv_Color, HTuple hv_Box)
{

  // Local iconic variables

  // Local control variables
  HTuple  hv_GenParamName, hv_GenParamValue;

  //This procedure displays text in a graphics window.
  //
  //Input parameters:
  //WindowHandle: The WindowHandle of the graphics window, where
  //   the message should be displayed
  //String: A tuple of strings containing the text message to be displayed
  //CoordSystem: If set to 'window', the text position is given
  //   with respect to the window coordinate system.
  //   If set to 'image', image coordinates are used.
  //   (This may be useful in zoomed images.)
  //Row: The row coordinate of the desired text position
  //   A tuple of values is allowed to display text at different
  //   positions.
  //Column: The column coordinate of the desired text position
  //   A tuple of values is allowed to display text at different
  //   positions.
  //Color: defines the color of the text as string.
  //   If set to [], '' or 'auto' the currently set color is used.
  //   If a tuple of strings is passed, the colors are used cyclically...
  //   - if |Row| == |Column| == 1: for each new textline
  //   = else for each text position.
  //Box: If Box[0] is set to 'true', the text is written within an orange box.
  //     If set to' false', no box is displayed.
  //     If set to a color string (e.g. 'white', '#FF00CC', etc.),
  //       the text is written in a box of that color.
  //     An optional second value for Box (Box[1]) controls if a shadow is displayed:
  //       'true' -> display a shadow in a default color
  //       'false' -> display no shadow
  //       otherwise -> use given string as color string for the shadow color
  //
  //It is possible to display multiple text strings in a single call.
  //In this case, some restrictions apply:
  //- Multiple text positions can be defined by specifying a tuple
  //  with multiple Row and/or Column coordinates, i.e.:
  //  - |Row| == n, |Column| == n
  //  - |Row| == n, |Column| == 1
  //  - |Row| == 1, |Column| == n
  //- If |Row| == |Column| == 1,
  //  each element of String is display in a new textline.
  //- If multiple positions or specified, the number of Strings
  //  must match the number of positions, i.e.:
  //  - Either |String| == n (each string is displayed at the
  //                          corresponding position),
  //  - or     |String| == 1 (The string is displayed n times).
  //
  //
  //Convert the parameters for disp_text.
  if (0 != (HTuple(hv_Row==HTuple()).TupleOr(hv_Column==HTuple())))
  {
    return;
  }
  if (0 != (hv_Row==-1))
  {
    hv_Row = 12;
  }
  if (0 != (hv_Column==-1))
  {
    hv_Column = 12;
  }
  //
  //Convert the parameter Box to generic parameters.
  hv_GenParamName = HTuple();
  hv_GenParamValue = HTuple();
  if (0 != ((hv_Box.TupleLength())>0))
  {
    if (0 != (HTuple(hv_Box[0])==HTuple("false")))
    {
      //Display no box
      hv_GenParamName = hv_GenParamName.TupleConcat("box");
      hv_GenParamValue = hv_GenParamValue.TupleConcat("false");
    }
    else if (0 != (HTuple(hv_Box[0])!=HTuple("true")))
    {
      //Set a color other than the default.
      hv_GenParamName = hv_GenParamName.TupleConcat("box_color");
      hv_GenParamValue = hv_GenParamValue.TupleConcat(HTuple(hv_Box[0]));
    }
  }
  if (0 != ((hv_Box.TupleLength())>1))
  {
    if (0 != (HTuple(hv_Box[1])==HTuple("false")))
    {
      //Display no shadow.
      hv_GenParamName = hv_GenParamName.TupleConcat("shadow");
      hv_GenParamValue = hv_GenParamValue.TupleConcat("false");
    }
    else if (0 != (HTuple(hv_Box[1])!=HTuple("true")))
    {
      //Set a shadow color other than the default.
      hv_GenParamName = hv_GenParamName.TupleConcat("shadow_color");
      hv_GenParamValue = hv_GenParamValue.TupleConcat(HTuple(hv_Box[1]));
    }
  }
  //Restore default CoordSystem behavior.
  if (0 != (hv_CoordSystem!=HTuple("window")))
  {
    hv_CoordSystem = "image";
  }
  //
  if (0 != (hv_Color==HTuple("")))
  {
    //disp_text does not accept an empty string for Color.
    hv_Color = HTuple();
  }
  //
  DispText(hv_WindowHandle, hv_String, hv_CoordSystem, hv_Row, hv_Column, hv_Color, 
      hv_GenParamName, hv_GenParamValue);
  return;
}

// Chapter: Graphics / Text
// Short Description: Set font independent of OS 
void set_display_font (HTuple hv_WindowHandle, HTuple hv_Size, HTuple hv_Font, HTuple hv_Bold, 
    HTuple hv_Slant)
{

  // Local iconic variables

  // Local control variables
  HTuple  hv_OS, hv_Fonts, hv_Style, hv_Exception;
  HTuple  hv_AvailableFonts, hv_Fdx, hv_Indices;

  //This procedure sets the text font of the current window with
  //the specified attributes.
  //
  //Input parameters:
  //WindowHandle: The graphics window for which the font will be set
  //Size: The font size. If Size=-1, the default of 16 is used.
  //Bold: If set to 'true', a bold font is used
  //Slant: If set to 'true', a slanted font is used
  //
  GetSystem("operating_system", &hv_OS);
  if (0 != (HTuple(hv_Size==HTuple()).TupleOr(hv_Size==-1)))
  {
    hv_Size = 16;
  }
  if (0 != ((hv_OS.TupleSubstr(0,2))==HTuple("Win")))
  {
    //Restore previous behaviour
    hv_Size = (1.13677*hv_Size).TupleInt();
  }
  else
  {
    hv_Size = hv_Size.TupleInt();
  }
  if (0 != (hv_Font==HTuple("Courier")))
  {
    hv_Fonts.Clear();
    hv_Fonts[0] = "Courier";
    hv_Fonts[1] = "Courier 10 Pitch";
    hv_Fonts[2] = "Courier New";
    hv_Fonts[3] = "CourierNew";
    hv_Fonts[4] = "Liberation Mono";
  }
  else if (0 != (hv_Font==HTuple("mono")))
  {
    hv_Fonts.Clear();
    hv_Fonts[0] = "Consolas";
    hv_Fonts[1] = "Menlo";
    hv_Fonts[2] = "Courier";
    hv_Fonts[3] = "Courier 10 Pitch";
    hv_Fonts[4] = "FreeMono";
    hv_Fonts[5] = "Liberation Mono";
  }
  else if (0 != (hv_Font==HTuple("sans")))
  {
    hv_Fonts.Clear();
    hv_Fonts[0] = "Luxi Sans";
    hv_Fonts[1] = "DejaVu Sans";
    hv_Fonts[2] = "FreeSans";
    hv_Fonts[3] = "Arial";
    hv_Fonts[4] = "Liberation Sans";
  }
  else if (0 != (hv_Font==HTuple("serif")))
  {
    hv_Fonts.Clear();
    hv_Fonts[0] = "Times New Roman";
    hv_Fonts[1] = "Luxi Serif";
    hv_Fonts[2] = "DejaVu Serif";
    hv_Fonts[3] = "FreeSerif";
    hv_Fonts[4] = "Utopia";
    hv_Fonts[5] = "Liberation Serif";
  }
  else
  {
    hv_Fonts = hv_Font;
  }
  hv_Style = "";
  if (0 != (hv_Bold==HTuple("true")))
  {
    hv_Style += HTuple("Bold");
  }
  else if (0 != (hv_Bold!=HTuple("false")))
  {
    hv_Exception = "Wrong value of control parameter Bold";
    throw HException(hv_Exception);
  }
  if (0 != (hv_Slant==HTuple("true")))
  {
    hv_Style += HTuple("Italic");
  }
  else if (0 != (hv_Slant!=HTuple("false")))
  {
    hv_Exception = "Wrong value of control parameter Slant";
    throw HException(hv_Exception);
  }
  if (0 != (hv_Style==HTuple("")))
  {
    hv_Style = "Normal";
  }
  QueryFont(hv_WindowHandle, &hv_AvailableFonts);
  hv_Font = "";
  {
  HTuple end_val48 = (hv_Fonts.TupleLength())-1;
  HTuple step_val48 = 1;
  for (hv_Fdx=0; hv_Fdx.Continue(end_val48, step_val48); hv_Fdx += step_val48)
  {
    hv_Indices = hv_AvailableFonts.TupleFind(HTuple(hv_Fonts[hv_Fdx]));
    if (0 != ((hv_Indices.TupleLength())>0))
    {
      if (0 != (HTuple(hv_Indices[0])>=0))
      {
        hv_Font = HTuple(hv_Fonts[hv_Fdx]);
        break;
      }
    }
  }
  }
  if (0 != (hv_Font==HTuple("")))
  {
    throw HException("Wrong value of control parameter Font");
  }
  hv_Font = (((hv_Font+"-")+hv_Style)+"-")+hv_Size;
  SetFont(hv_WindowHandle, hv_Font);
  return;
}

//定义全局变量
static HTuple xL,yL,xR,yR;


// Main procedure 
void actionL(HObject Image)
{

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
  dev_open_window_fit_image(Image, 0, 0, hv_Width, hv_Height, &hv_WindowHandle);
  if (HDevWindowStack::IsOpen())
    DispObj(Image, HDevWindowStack::GetActive());
  set_display_font(hv_WindowHandle, 15, "mono", "true", "false");
  GenRectangle1(&ho_Rectangle, 0, 0, 288, 360);
  ReduceDomain(Image, ho_Rectangle, &ho_Image1); 
  //WriteImage(Image, "jpeg", 0, "C:/Users/Admin/Documents/fire-detection/infrared11.jpeg");
  if (HDevWindowStack::IsOpen())
    DispObj(Image, HDevWindowStack::GetActive());

  Decompose3(ho_Image1, &ho_Image11, &ho_Image12, &ho_Image13);
  
  Threshold(ho_Image12, &ho_Region1, 155, 255);
  OpeningCircle(ho_Region1, &ho_RegionOpening1, 1.5);
  ClosingCircle(ho_RegionOpening1, &ho_RegionClosing1, 1);
  FillUp(ho_Region1, &ho_RegionFillUp1);
  //reduce_domain (ImageResult12, RegionFillUp1, ImageReduced1)
  Connection(ho_Region1, &ho_ConnectedRegion1);
  SelectShape(ho_ConnectedRegion1, &ho_SelectedRegions1, "max_diameter", "and", 20, 
      999);
  CountObj(ho_SelectedRegions1, &hv_Number1);
  ReduceDomain(ho_Image1, ho_SelectedRegions1, &ho_ImageReduced1);

  if (0 != (HTuple(hv_Number1!=0)))
  {
    hv_k = 1;
    if (HDevWindowStack::IsOpen())
      SetLineWidth(HDevWindowStack::GetActive(),1);
    Boundary(ho_SelectedRegions1, &ho_RegionBorder1, "inner");
    GenContourRegionXld(ho_RegionBorder1, &ho_Contours1, "center");
    FitRectangle2ContourXld(ho_Contours1, "regression", -1, 0, 0, 3, 2, &hv_Row1, 
        &hv_Column1, &hv_Phi1, &hv_Length11, &hv_Length12, &hv_PointOrder1);
    GenRectangle2ContourXld(&ho_Rectangle1, hv_Row1, hv_Column1, hv_Phi1, hv_Length11, 
        hv_Length12);

    GenCrossContourXld(&ho_Cross1, hv_Row1, hv_Column1, 6, hv_Phi1);

    xL = hv_Row1;
    yL = hv_Column1;

  }
  else
  {
    set_display_font(hv_WindowHandle, 20, "mono", "true", "false");
    disp_message(hv_WindowHandle, "No fire detected", "image", 0, 0, "red", "true");
  }
}

void actionR(HObject Image)
{

  // Local iconic variables
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


  dev_update_off();
  if (HDevWindowStack::IsOpen())
    CloseWindow(HDevWindowStack::Pop());

  //ReadImage(&ho_Img1, "C:/Users/Admin/Documents/fire-detection/infrared1.jpeg");
  GetImageSize(Image, &hv_Width, &hv_Height);
  dev_open_window_fit_image(Image, 0, 0, hv_Width, hv_Height, &hv_WindowHandle);
  if (HDevWindowStack::IsOpen())
    DispObj(Image, HDevWindowStack::GetActive());
  set_display_font(hv_WindowHandle, 15, "mono", "true", "false");
  GenRectangle1(&ho_Rectangle, 0, 0, 288, 360);
  ReduceDomain(Image, ho_Rectangle, &ho_Image1);
  if (HDevWindowStack::IsOpen())
    DispObj(Image, HDevWindowStack::GetActive());

  Decompose3(ho_Image1, &ho_Image11, &ho_Image12, &ho_Image13);
  
  //阈值分割 选出面积最大的区域
  Threshold(ho_Image12, &ho_Region1, 155, 255);
  OpeningCircle(ho_Region1, &ho_RegionOpening1, 1.5);
  ClosingCircle(ho_RegionOpening1, &ho_RegionClosing1, 1);
  FillUp(ho_Region1, &ho_RegionFillUp1);
  Connection(ho_Region1, &ho_ConnectedRegion1);
  SelectShape(ho_ConnectedRegion1, &ho_SelectedRegions1, "max_diameter", "and", 20, 
      999);
  CountObj(ho_SelectedRegions1, &hv_Number1);
  ReduceDomain(ho_Image1, ho_SelectedRegions1, &ho_ImageReduced1);

  if (0 != (HTuple(hv_Number1!=0)))
  {
    hv_k = 1;
    if (HDevWindowStack::IsOpen())
      SetLineWidth(HDevWindowStack::GetActive(),1);
    Boundary(ho_SelectedRegions1, &ho_RegionBorder1, "inner");
    GenContourRegionXld(ho_RegionBorder1, &ho_Contours1, "center");
    FitRectangle2ContourXld(ho_Contours1, "regression", -1, 0, 0, 3, 2, &hv_Row1, 
        &hv_Column1, &hv_Phi1, &hv_Length11, &hv_Length12, &hv_PointOrder1);
    GenRectangle2ContourXld(&ho_Rectangle1, hv_Row1, hv_Column1, hv_Phi1, hv_Length11, 
        hv_Length12);

    GenCrossContourXld(&ho_Cross1, hv_Row1, hv_Column1, 6, hv_Phi1);

    xR = hv_Row1;
    yR = hv_Column1;

  }
  else
  {
    set_display_font(hv_WindowHandle, 20, "mono", "true", "false");
    disp_message(hv_WindowHandle, "No fire detected", "image", 0, 0, "red", "true");
  }
}

static HTuple fire_X,fire_Y,fire_Z;

//读入相机标定参数  三维定位
void fire_position(HTuple rowL,HTuple colomnL,HTuple rowR,HTuple colomnR)
{
  HTuple hv_CameraParameters1, hv_CameraParameters2,hv_RealPose;
  HTuple hv_X, hv_Y, hv_Z, hv_Dist;

  ReadCamPar("campar1.dat", &hv_CameraParameters1);
  ReadCamPar("campar2.dat", &hv_CameraParameters2);
  ReadPose("relpose.dat", &hv_RealPose);

  IntersectLinesOfSight(hv_CameraParameters1, hv_CameraParameters2, hv_RealPose, 
        rowL, colomnL, rowR, colomnR, &hv_X, &hv_Y, &hv_Z, &hv_Dist);
  fire_X = hv_X;
  fire_Y = hv_Y;
  fire_Z = hv_Z;

  //info需要改名
  firelocateInfo.header.stamp = ros::Time().now();
  firelocateInfo.header.frame_id = "camerair_link";

  firelocateInfo.flag = true;
  firelocateInfo.BrickType = "fire";
  firelocateInfo.position.x = fire_X.D();
  firelocateInfo.position.y = fire_Y.D();
  firelocateInfo.position.z = fire_Z.D();
}

void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) 
{
    
    HObject  ho_Image;
    //获取halcon-bridge图像指针
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointer = halcon_bridge::toHalconCopy(msg);
    ho_Image = *halcon_bridge_imagePointer->image;
    
    actionL(ho_Image);

}

void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) 
{
   
    HObject  ho_Image;
    //获取halcon-bridge图像指针
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointer = halcon_bridge::toHalconCopy(msg);
    ho_Image = *halcon_bridge_imagePointer->image;
    
    actionR(ho_Image);

}

int main(int argc, char *argv[])
{
  int ret = 0;

  //给左右视图中定位位置初始化赋值
  xL = 0;
  yL = 0;
  xR = 0;
  yR = 0;

  ros::init(argc, argv, "fire_position");

  ros::NodeHandle nh; 

  try
  {
    //此处需要换成红外左右相机发布消息
    ros::Subscriber subLeft  = nh.subscribe("/cameraIR_arm/imagearm", 1,
                                        imageLeftRectifiedCallback);
    ros::Subscriber subRight = nh.subscribe("/cameraIR_car/imagecar", 1,
                                        imageRightRectifiedCallback);

    ros::spin();
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        fire_position(xL,yL,xR,yR);

        pub.publish(firelocateInfo);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

  }
  catch (HException &exception)
  {
    ROS_ERROR("  Error #%u in %s: %s\n", exception.ErrorCode(),
            (const char *)exception.ProcName(),
            (const char *)exception.ErrorMessage());
    ret = 1;
  }
  return ret;
}




