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
#include <string>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <math.h>
#include "halcon_image.h"
#include "std_msgs/Empty.h"
#include "tf/transform_broadcaster.h"
#include <bit_vision_msgs/VisionProc.h>
#include <opencv2/core/core.hpp>

#undef Success  
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace HalconCpp;
using namespace cv;

# define GetBrickPoseMERO   1
# define GetBrickPoseMERC   2  
# define GetBrickPoseZED    3  
# define GetBrickPoseZEDNew 4  
# define GetBrickLoc        5
# define GetPutPos          6
# define GetPutAngle        7
# define GetLPose           8
# define NotRun             0

int algorithm = GetBrickPoseMERO;     // 当前算法

# define Deg2Rad  0.017453292519943         
# define Rad2Deg  57.29577951308232

//取砖和放砖用同一个变量表示角度
HTuple brick_angle(0);
string brick_color = "G";
HTuple L_shelf_angle(0);
tf::StampedTransform transform_MEROnBase;
tf::StampedTransform transform_ZEDLOnBase;
tf::StampedTransform transform_ZEDROnBase;

void get_rectangle2_points (HTuple hv_CenterY, HTuple hv_CenterX, HTuple hv_Phi, 
    HTuple hv_Len1, HTuple hv_Len2, HTuple *hv_VertexesY, HTuple *hv_VertexesX);

void get_cam_par_data (HTuple hv_CameraParam, HTuple hv_ParamName, HTuple *hv_ParamValue);
// Chapter: Calibration / Camera Parameters
// Short Description: Get the names of the parameters in a camera parameter tuple. 
void get_cam_par_names (HTuple hv_CameraParam, HTuple *hv_CameraType, HTuple *hv_ParamNames);
// Chapter: Calibration / Camera Parameters
// Short Description: Set the value of a specified camera parameter in the camera parameter tuple. 
void set_cam_par_data (HTuple hv_CameraParamIn, HTuple hv_ParamName, HTuple hv_ParamValue, 
    HTuple *hv_CameraParamOut);


void get_rectangle2_points (HTuple hv_CenterY, HTuple hv_CenterX, HTuple hv_Phi, 
    HTuple hv_Len1, HTuple hv_Len2, HTuple *hv_VertexesY, HTuple *hv_VertexesX)
{

  // Local iconic variables

  // Local control variables
  HTuple  hv_RowTem, hv_ColTem, hv_Cos, hv_Sin;

  //Initialize the variable for coordinate of vertexes of rectangle2

  (*hv_VertexesY) = HTuple();

  (*hv_VertexesX) = HTuple();



  //Initialize the temperary variables

  hv_RowTem = 0;

  hv_ColTem = 0;

  //Judge the rectangle if it is available

  if (0 != (HTuple(hv_Len1<0).TupleOr(hv_Len2<0)))
  {

    return;

  }

  //Compute the sine and cosine of tuple Phi

  TupleCos(hv_Phi, &hv_Cos);

  TupleSin(hv_Phi, &hv_Sin);

  //Compute the coordinate of the upper-right vertex of rectangle

  hv_RowTem = (hv_CenterY-(hv_Len1*hv_Sin))-(hv_Len2*hv_Cos);

  hv_ColTem = (hv_CenterX+(hv_Len1*hv_Cos))-(hv_Len2*hv_Sin);

  (*hv_VertexesY) = (*hv_VertexesY).TupleConcat(hv_RowTem);

  (*hv_VertexesX) = (*hv_VertexesX).TupleConcat(hv_ColTem);



  //Compute the coordinate of the upper-left vertex of rectangle

  hv_RowTem = (hv_CenterY+(hv_Len1*hv_Sin))-(hv_Len2*hv_Cos);

  hv_ColTem = (hv_CenterX-(hv_Len1*hv_Cos))-(hv_Len2*hv_Sin);

  (*hv_VertexesY) = (*hv_VertexesY).TupleConcat(hv_RowTem);

  (*hv_VertexesX) = (*hv_VertexesX).TupleConcat(hv_ColTem);



  //Compute the coordinate of the bottom-left vertex of rectangle

  hv_RowTem = (hv_CenterY+(hv_Len1*hv_Sin))+(hv_Len2*hv_Cos);

  hv_ColTem = (hv_CenterX-(hv_Len1*hv_Cos))+(hv_Len2*hv_Sin);

  (*hv_VertexesY) = (*hv_VertexesY).TupleConcat(hv_RowTem);

  (*hv_VertexesX) = (*hv_VertexesX).TupleConcat(hv_ColTem);



  //Compute the coordinate of the bottom-right vertex of rectangle

  hv_RowTem = (hv_CenterY-(hv_Len1*hv_Sin))+(hv_Len2*hv_Cos);

  hv_ColTem = (hv_CenterX+(hv_Len1*hv_Cos))+(hv_Len2*hv_Sin);

  (*hv_VertexesY) = (*hv_VertexesY).TupleConcat(hv_RowTem);

  (*hv_VertexesX) = (*hv_VertexesX).TupleConcat(hv_ColTem);

  return;
}


// Procedures 
// External procedures 
// Chapter: Calibration / Camera Parameters
// Short Description: Get the value of a specified camera parameter from the camera parameter tuple. 
void get_cam_par_data (HTuple hv_CameraParam, HTuple hv_ParamName, HTuple *hv_ParamValue)
{
  // Local iconic variables

  // Local control variables
  HTuple  hv_CameraType, hv_CameraParamNames, hv_Index;
  HTuple  hv_ParamNameInd, hv_I;

  //get_cam_par_data returns in ParamValue the value of the
  //parameter that is given in ParamName from the tuple of
  //camera parameters that is given in CameraParam.
  //
  //Get the parameter names that correspond to the
  //elements in the input camera parameter tuple.
  get_cam_par_names(hv_CameraParam, &hv_CameraType, &hv_CameraParamNames);
  //
  //Find the index of the requested camera data and return
  //the corresponding value.
  (*hv_ParamValue) = HTuple();
  {
  HTuple end_val11 = (hv_ParamName.TupleLength())-1;
  HTuple step_val11 = 1;
  for (hv_Index=0; hv_Index.Continue(end_val11, step_val11); hv_Index += step_val11)
  {
    hv_ParamNameInd = HTuple(hv_ParamName[hv_Index]);
    if (0 != (hv_ParamNameInd==HTuple("camera_type")))
    {
      (*hv_ParamValue) = (*hv_ParamValue).TupleConcat(hv_CameraType);
      continue;
    }
    hv_I = hv_CameraParamNames.TupleFind(hv_ParamNameInd);
    if (0 != (hv_I!=-1))
    {
      (*hv_ParamValue) = (*hv_ParamValue).TupleConcat(HTuple(hv_CameraParam[hv_I]));
    }
    else
    {
      throw HException("Unknown camera parameter "+hv_ParamNameInd);
    }
  }
  }
  return;
}

// Chapter: Calibration / Camera Parameters
// Short Description: Get the names of the parameters in a camera parameter tuple. 
void get_cam_par_names (HTuple hv_CameraParam, HTuple *hv_CameraType, HTuple *hv_ParamNames)
{

  // Local iconic variables

  // Local control variables
  HTuple  hv_CameraParamAreaScanDivision, hv_CameraParamAreaScanPolynomial;
  HTuple  hv_CameraParamAreaScanTelecentricDivision, hv_CameraParamAreaScanTelecentricPolynomial;
  HTuple  hv_CameraParamAreaScanTiltDivision, hv_CameraParamAreaScanTiltPolynomial;
  HTuple  hv_CameraParamAreaScanImageSideTelecentricTiltDivision;
  HTuple  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial;
  HTuple  hv_CameraParamAreaScanBilateralTelecentricTiltDivision;
  HTuple  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial;
  HTuple  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision;
  HTuple  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial;
  HTuple  hv_CameraParamAreaScanHypercentricDivision, hv_CameraParamAreaScanHypercentricPolynomial;
  HTuple  hv_CameraParamLinesScan, hv_CameraParamAreaScanTiltDivisionLegacy;
  HTuple  hv_CameraParamAreaScanTiltPolynomialLegacy, hv_CameraParamAreaScanTelecentricDivisionLegacy;
  HTuple  hv_CameraParamAreaScanTelecentricPolynomialLegacy;
  HTuple  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy;
  HTuple  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy;

  //get_cam_par_names returns for each element in the camera
  //parameter tuple that is passed in CameraParam the name
  //of the respective camera parameter. The parameter names
  //are returned in ParamNames. Additionally, the camera
  //type is returned in CameraType. Alternatively, instead of
  //the camera parameters, the camera type can be passed in
  //CameraParam in form of one of the following strings:
  //  - 'area_scan_division'
  //  - 'area_scan_polynomial'
  //  - 'area_scan_tilt_division'
  //  - 'area_scan_tilt_polynomial'
  //  - 'area_scan_telecentric_division'
  //  - 'area_scan_telecentric_polynomial'
  //  - 'area_scan_tilt_bilateral_telecentric_division'
  //  - 'area_scan_tilt_bilateral_telecentric_polynomial'
  //  - 'area_scan_tilt_object_side_telecentric_division'
  //  - 'area_scan_tilt_object_side_telecentric_polynomial'
  //  - 'area_scan_hypercentric_division'
  //  - 'area_scan_hypercentric_polynomial'
  //  - 'line_scan'
  //
  hv_CameraParamAreaScanDivision.Clear();
  hv_CameraParamAreaScanDivision[0] = "focus";
  hv_CameraParamAreaScanDivision[1] = "kappa";
  hv_CameraParamAreaScanDivision[2] = "sx";
  hv_CameraParamAreaScanDivision[3] = "sy";
  hv_CameraParamAreaScanDivision[4] = "cx";
  hv_CameraParamAreaScanDivision[5] = "cy";
  hv_CameraParamAreaScanDivision[6] = "image_width";
  hv_CameraParamAreaScanDivision[7] = "image_height";
  hv_CameraParamAreaScanPolynomial.Clear();
  hv_CameraParamAreaScanPolynomial[0] = "focus";
  hv_CameraParamAreaScanPolynomial[1] = "k1";
  hv_CameraParamAreaScanPolynomial[2] = "k2";
  hv_CameraParamAreaScanPolynomial[3] = "k3";
  hv_CameraParamAreaScanPolynomial[4] = "p1";
  hv_CameraParamAreaScanPolynomial[5] = "p2";
  hv_CameraParamAreaScanPolynomial[6] = "sx";
  hv_CameraParamAreaScanPolynomial[7] = "sy";
  hv_CameraParamAreaScanPolynomial[8] = "cx";
  hv_CameraParamAreaScanPolynomial[9] = "cy";
  hv_CameraParamAreaScanPolynomial[10] = "image_width";
  hv_CameraParamAreaScanPolynomial[11] = "image_height";
  hv_CameraParamAreaScanTelecentricDivision.Clear();
  hv_CameraParamAreaScanTelecentricDivision[0] = "magnification";
  hv_CameraParamAreaScanTelecentricDivision[1] = "kappa";
  hv_CameraParamAreaScanTelecentricDivision[2] = "sx";
  hv_CameraParamAreaScanTelecentricDivision[3] = "sy";
  hv_CameraParamAreaScanTelecentricDivision[4] = "cx";
  hv_CameraParamAreaScanTelecentricDivision[5] = "cy";
  hv_CameraParamAreaScanTelecentricDivision[6] = "image_width";
  hv_CameraParamAreaScanTelecentricDivision[7] = "image_height";
  hv_CameraParamAreaScanTelecentricPolynomial.Clear();
  hv_CameraParamAreaScanTelecentricPolynomial[0] = "magnification";
  hv_CameraParamAreaScanTelecentricPolynomial[1] = "k1";
  hv_CameraParamAreaScanTelecentricPolynomial[2] = "k2";
  hv_CameraParamAreaScanTelecentricPolynomial[3] = "k3";
  hv_CameraParamAreaScanTelecentricPolynomial[4] = "p1";
  hv_CameraParamAreaScanTelecentricPolynomial[5] = "p2";
  hv_CameraParamAreaScanTelecentricPolynomial[6] = "sx";
  hv_CameraParamAreaScanTelecentricPolynomial[7] = "sy";
  hv_CameraParamAreaScanTelecentricPolynomial[8] = "cx";
  hv_CameraParamAreaScanTelecentricPolynomial[9] = "cy";
  hv_CameraParamAreaScanTelecentricPolynomial[10] = "image_width";
  hv_CameraParamAreaScanTelecentricPolynomial[11] = "image_height";
  hv_CameraParamAreaScanTiltDivision.Clear();
  hv_CameraParamAreaScanTiltDivision[0] = "focus";
  hv_CameraParamAreaScanTiltDivision[1] = "kappa";
  hv_CameraParamAreaScanTiltDivision[2] = "image_plane_dist";
  hv_CameraParamAreaScanTiltDivision[3] = "tilt";
  hv_CameraParamAreaScanTiltDivision[4] = "rot";
  hv_CameraParamAreaScanTiltDivision[5] = "sx";
  hv_CameraParamAreaScanTiltDivision[6] = "sy";
  hv_CameraParamAreaScanTiltDivision[7] = "cx";
  hv_CameraParamAreaScanTiltDivision[8] = "cy";
  hv_CameraParamAreaScanTiltDivision[9] = "image_width";
  hv_CameraParamAreaScanTiltDivision[10] = "image_height";
  hv_CameraParamAreaScanTiltPolynomial.Clear();
  hv_CameraParamAreaScanTiltPolynomial[0] = "focus";
  hv_CameraParamAreaScanTiltPolynomial[1] = "k1";
  hv_CameraParamAreaScanTiltPolynomial[2] = "k2";
  hv_CameraParamAreaScanTiltPolynomial[3] = "k3";
  hv_CameraParamAreaScanTiltPolynomial[4] = "p1";
  hv_CameraParamAreaScanTiltPolynomial[5] = "p2";
  hv_CameraParamAreaScanTiltPolynomial[6] = "image_plane_dist";
  hv_CameraParamAreaScanTiltPolynomial[7] = "tilt";
  hv_CameraParamAreaScanTiltPolynomial[8] = "rot";
  hv_CameraParamAreaScanTiltPolynomial[9] = "sx";
  hv_CameraParamAreaScanTiltPolynomial[10] = "sy";
  hv_CameraParamAreaScanTiltPolynomial[11] = "cx";
  hv_CameraParamAreaScanTiltPolynomial[12] = "cy";
  hv_CameraParamAreaScanTiltPolynomial[13] = "image_width";
  hv_CameraParamAreaScanTiltPolynomial[14] = "image_height";
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision.Clear();
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision[0] = "focus";
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision[1] = "kappa";
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision[2] = "tilt";
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision[3] = "rot";
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision[4] = "sx";
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision[5] = "sy";
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision[6] = "cx";
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision[7] = "cy";
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision[8] = "image_width";
  hv_CameraParamAreaScanImageSideTelecentricTiltDivision[9] = "image_height";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial.Clear();
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[0] = "focus";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[1] = "k1";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[2] = "k2";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[3] = "k3";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[4] = "p1";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[5] = "p2";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[6] = "tilt";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[7] = "rot";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[8] = "sx";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[9] = "sy";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[10] = "cx";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[11] = "cy";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[12] = "image_width";
  hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial[13] = "image_height";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision.Clear();
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision[0] = "magnification";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision[1] = "kappa";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision[2] = "tilt";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision[3] = "rot";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision[4] = "sx";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision[5] = "sy";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision[6] = "cx";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision[7] = "cy";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision[8] = "image_width";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivision[9] = "image_height";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial.Clear();
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[0] = "magnification";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[1] = "k1";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[2] = "k2";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[3] = "k3";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[4] = "p1";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[5] = "p2";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[6] = "tilt";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[7] = "rot";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[8] = "sx";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[9] = "sy";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[10] = "cx";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[11] = "cy";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[12] = "image_width";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial[13] = "image_height";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision.Clear();
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[0] = "magnification";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[1] = "kappa";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[2] = "image_plane_dist";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[3] = "tilt";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[4] = "rot";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[5] = "sx";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[6] = "sy";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[7] = "cx";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[8] = "cy";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[9] = "image_width";
  hv_CameraParamAreaScanObjectSideTelecentricTiltDivision[10] = "image_height";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial.Clear();
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[0] = "magnification";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[1] = "k1";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[2] = "k2";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[3] = "k3";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[4] = "p1";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[5] = "p2";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[6] = "image_plane_dist";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[7] = "tilt";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[8] = "rot";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[9] = "sx";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[10] = "sy";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[11] = "cx";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[12] = "cy";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[13] = "image_width";
  hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial[14] = "image_height";
  hv_CameraParamAreaScanHypercentricDivision.Clear();
  hv_CameraParamAreaScanHypercentricDivision[0] = "focus";
  hv_CameraParamAreaScanHypercentricDivision[1] = "kappa";
  hv_CameraParamAreaScanHypercentricDivision[2] = "sx";
  hv_CameraParamAreaScanHypercentricDivision[3] = "sy";
  hv_CameraParamAreaScanHypercentricDivision[4] = "cx";
  hv_CameraParamAreaScanHypercentricDivision[5] = "cy";
  hv_CameraParamAreaScanHypercentricDivision[6] = "image_width";
  hv_CameraParamAreaScanHypercentricDivision[7] = "image_height";
  hv_CameraParamAreaScanHypercentricPolynomial.Clear();
  hv_CameraParamAreaScanHypercentricPolynomial[0] = "focus";
  hv_CameraParamAreaScanHypercentricPolynomial[1] = "k1";
  hv_CameraParamAreaScanHypercentricPolynomial[2] = "k2";
  hv_CameraParamAreaScanHypercentricPolynomial[3] = "k3";
  hv_CameraParamAreaScanHypercentricPolynomial[4] = "p1";
  hv_CameraParamAreaScanHypercentricPolynomial[5] = "p2";
  hv_CameraParamAreaScanHypercentricPolynomial[6] = "sx";
  hv_CameraParamAreaScanHypercentricPolynomial[7] = "sy";
  hv_CameraParamAreaScanHypercentricPolynomial[8] = "cx";
  hv_CameraParamAreaScanHypercentricPolynomial[9] = "cy";
  hv_CameraParamAreaScanHypercentricPolynomial[10] = "image_width";
  hv_CameraParamAreaScanHypercentricPolynomial[11] = "image_height";
  hv_CameraParamLinesScan.Clear();
  hv_CameraParamLinesScan[0] = "focus";
  hv_CameraParamLinesScan[1] = "kappa";
  hv_CameraParamLinesScan[2] = "sx";
  hv_CameraParamLinesScan[3] = "sy";
  hv_CameraParamLinesScan[4] = "cx";
  hv_CameraParamLinesScan[5] = "cy";
  hv_CameraParamLinesScan[6] = "image_width";
  hv_CameraParamLinesScan[7] = "image_height";
  hv_CameraParamLinesScan[8] = "vx";
  hv_CameraParamLinesScan[9] = "vy";
  hv_CameraParamLinesScan[10] = "vz";
  //Legacy parameter names
  hv_CameraParamAreaScanTiltDivisionLegacy.Clear();
  hv_CameraParamAreaScanTiltDivisionLegacy[0] = "focus";
  hv_CameraParamAreaScanTiltDivisionLegacy[1] = "kappa";
  hv_CameraParamAreaScanTiltDivisionLegacy[2] = "tilt";
  hv_CameraParamAreaScanTiltDivisionLegacy[3] = "rot";
  hv_CameraParamAreaScanTiltDivisionLegacy[4] = "sx";
  hv_CameraParamAreaScanTiltDivisionLegacy[5] = "sy";
  hv_CameraParamAreaScanTiltDivisionLegacy[6] = "cx";
  hv_CameraParamAreaScanTiltDivisionLegacy[7] = "cy";
  hv_CameraParamAreaScanTiltDivisionLegacy[8] = "image_width";
  hv_CameraParamAreaScanTiltDivisionLegacy[9] = "image_height";
  hv_CameraParamAreaScanTiltPolynomialLegacy.Clear();
  hv_CameraParamAreaScanTiltPolynomialLegacy[0] = "focus";
  hv_CameraParamAreaScanTiltPolynomialLegacy[1] = "k1";
  hv_CameraParamAreaScanTiltPolynomialLegacy[2] = "k2";
  hv_CameraParamAreaScanTiltPolynomialLegacy[3] = "k3";
  hv_CameraParamAreaScanTiltPolynomialLegacy[4] = "p1";
  hv_CameraParamAreaScanTiltPolynomialLegacy[5] = "p2";
  hv_CameraParamAreaScanTiltPolynomialLegacy[6] = "tilt";
  hv_CameraParamAreaScanTiltPolynomialLegacy[7] = "rot";
  hv_CameraParamAreaScanTiltPolynomialLegacy[8] = "sx";
  hv_CameraParamAreaScanTiltPolynomialLegacy[9] = "sy";
  hv_CameraParamAreaScanTiltPolynomialLegacy[10] = "cx";
  hv_CameraParamAreaScanTiltPolynomialLegacy[11] = "cy";
  hv_CameraParamAreaScanTiltPolynomialLegacy[12] = "image_width";
  hv_CameraParamAreaScanTiltPolynomialLegacy[13] = "image_height";
  hv_CameraParamAreaScanTelecentricDivisionLegacy.Clear();
  hv_CameraParamAreaScanTelecentricDivisionLegacy[0] = "focus";
  hv_CameraParamAreaScanTelecentricDivisionLegacy[1] = "kappa";
  hv_CameraParamAreaScanTelecentricDivisionLegacy[2] = "sx";
  hv_CameraParamAreaScanTelecentricDivisionLegacy[3] = "sy";
  hv_CameraParamAreaScanTelecentricDivisionLegacy[4] = "cx";
  hv_CameraParamAreaScanTelecentricDivisionLegacy[5] = "cy";
  hv_CameraParamAreaScanTelecentricDivisionLegacy[6] = "image_width";
  hv_CameraParamAreaScanTelecentricDivisionLegacy[7] = "image_height";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy.Clear();
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[0] = "focus";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[1] = "k1";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[2] = "k2";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[3] = "k3";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[4] = "p1";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[5] = "p2";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[6] = "sx";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[7] = "sy";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[8] = "cx";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[9] = "cy";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[10] = "image_width";
  hv_CameraParamAreaScanTelecentricPolynomialLegacy[11] = "image_height";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy.Clear();
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy[0] = "focus";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy[1] = "kappa";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy[2] = "tilt";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy[3] = "rot";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy[4] = "sx";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy[5] = "sy";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy[6] = "cx";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy[7] = "cy";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy[8] = "image_width";
  hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy[9] = "image_height";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy.Clear();
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[0] = "focus";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[1] = "k1";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[2] = "k2";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[3] = "k3";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[4] = "p1";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[5] = "p2";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[6] = "tilt";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[7] = "rot";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[8] = "sx";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[9] = "sy";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[10] = "cx";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[11] = "cy";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[12] = "image_width";
  hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy[13] = "image_height";
  //
  //If the camera type is passed in CameraParam
  if (0 != (HTuple((hv_CameraParam.TupleLength())==1).TupleAnd(HTuple(hv_CameraParam[0]).TupleIsString())))
  {
    (*hv_CameraType) = ((const HTuple&)hv_CameraParam)[0];
    if (0 != ((*hv_CameraType)==HTuple("area_scan_division")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_polynomial")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_telecentric_division")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanTelecentricDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_telecentric_polynomial")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanTelecentricPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_division")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanTiltDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_polynomial")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanTiltPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_image_side_telecentric_division")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanImageSideTelecentricTiltDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_image_side_telecentric_polynomial")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_bilateral_telecentric_division")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanBilateralTelecentricTiltDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_bilateral_telecentric_polynomial")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_object_side_telecentric_division")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanObjectSideTelecentricTiltDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_object_side_telecentric_polynomial")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_hypercentric_division")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanHypercentricDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_hypercentric_polynomial")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanHypercentricPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("line_scan")))
    {
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamLinesScan);
    }
    else
    {
      throw HException(("Unknown camera type '"+(*hv_CameraType))+"' passed in CameraParam.");
    }
    return;
  }
  //
  //If the camera parameters are passed in CameraParam
  if (0 != ((HTuple(hv_CameraParam[0]).TupleIsString()).TupleNot()))
  {
    //Format of camera parameters for HALCON 12 and earlier
    switch ((hv_CameraParam.TupleLength()).I())
    {
      //
      //Area Scan
    case 8:
      //CameraType: 'area_scan_division' or 'area_scan_telecentric_division'
      if (0 != (HTuple(hv_CameraParam[0])!=0.0))
      {
        (*hv_ParamNames) = hv_CameraParamAreaScanDivision;
        (*hv_CameraType) = "area_scan_division";
      }
      else
      {
        (*hv_ParamNames) = hv_CameraParamAreaScanTelecentricDivisionLegacy;
        (*hv_CameraType) = "area_scan_telecentric_division";
      }
      break;
    case 10:
      //CameraType: 'area_scan_tilt_division' or 'area_scan_telecentric_tilt_division'
      if (0 != (HTuple(hv_CameraParam[0])!=0.0))
      {
        (*hv_ParamNames) = hv_CameraParamAreaScanTiltDivisionLegacy;
        (*hv_CameraType) = "area_scan_tilt_division";
      }
      else
      {
        (*hv_ParamNames) = hv_CameraParamAreaScanBilateralTelecentricTiltDivisionLegacy;
        (*hv_CameraType) = "area_scan_tilt_bilateral_telecentric_division";
      }
      break;
    case 12:
      //CameraType: 'area_scan_polynomial' or 'area_scan_telecentric_polynomial'
      if (0 != (HTuple(hv_CameraParam[0])!=0.0))
      {
        (*hv_ParamNames) = hv_CameraParamAreaScanPolynomial;
        (*hv_CameraType) = "area_scan_polynomial";
      }
      else
      {
        (*hv_ParamNames) = hv_CameraParamAreaScanTelecentricPolynomialLegacy;
        (*hv_CameraType) = "area_scan_telecentric_polynomial";
      }
      break;
    case 14:
      //CameraType: 'area_scan_tilt_polynomial' or 'area_scan_telecentric_tilt_polynomial'
      if (0 != (HTuple(hv_CameraParam[0])!=0.0))
      {
        (*hv_ParamNames) = hv_CameraParamAreaScanTiltPolynomialLegacy;
        (*hv_CameraType) = "area_scan_tilt_polynomial";
      }
      else
      {
        (*hv_ParamNames) = hv_CameraParamAreaScanBilateralTelecentricTiltPolynomialLegacy;
        (*hv_CameraType) = "area_scan_tilt_bilateral_telecentric_polynomial";
      }
      break;
      //
      //Line Scan
    case 11:
      //CameraType: 'line_scan'
      (*hv_ParamNames) = hv_CameraParamLinesScan;
      (*hv_CameraType) = "line_scan";
      break;
    default:
      throw HException("Wrong number of values in CameraParam.");
    }
  }
  else
  {
    //Format of camera parameters since HALCON 13
    (*hv_CameraType) = ((const HTuple&)hv_CameraParam)[0];
    if (0 != ((*hv_CameraType)==HTuple("area_scan_division")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=9))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_polynomial")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=13))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_telecentric_division")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=9))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanTelecentricDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_telecentric_polynomial")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=13))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanTelecentricPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_division")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=12))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanTiltDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_polynomial")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=16))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanTiltPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_image_side_telecentric_division")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=11))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanImageSideTelecentricTiltDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_image_side_telecentric_polynomial")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=15))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanImageSideTelecentricTiltPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_bilateral_telecentric_division")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=11))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanBilateralTelecentricTiltDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_bilateral_telecentric_polynomial")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=15))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanBilateralTelecentricTiltPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_object_side_telecentric_division")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=12))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanObjectSideTelecentricTiltDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_tilt_object_side_telecentric_polynomial")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=16))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanObjectSideTelecentricTiltPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_hypercentric_division")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=9))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanHypercentricDivision);
    }
    else if (0 != ((*hv_CameraType)==HTuple("area_scan_hypercentric_polynomial")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=13))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamAreaScanHypercentricPolynomial);
    }
    else if (0 != ((*hv_CameraType)==HTuple("line_scan")))
    {
      if (0 != ((hv_CameraParam.TupleLength())!=12))
      {
        throw HException("Wrong number of values in CameraParam.");
      }
      (*hv_ParamNames).Clear();
      (*hv_ParamNames)[0] = "camera_type";
      (*hv_ParamNames).Append(hv_CameraParamLinesScan);
    }
    else
    {
      throw HException("Unknown camera type in CameraParam.");
    }
  }
  return;
}

// Chapter: Calibration / Camera Parameters
// Short Description: Set the value of a specified camera parameter in the camera parameter tuple. 
void set_cam_par_data (HTuple hv_CameraParamIn, HTuple hv_ParamName, HTuple hv_ParamValue, 
    HTuple *hv_CameraParamOut)
{

  // Local iconic variables

  // Local control variables
  HTuple  hv_Index, hv_ParamNameInd, hv_CameraParamNames;
  HTuple  hv_I, hv_CameraType, hv_IsTelecentric;

  //set_cam_par_data sets the value of the parameter that
  //is given in ParamName in the tuple of camera parameters
  //given in CameraParamIn. The modified camera parameters
  //are returned in CameraParamOut.
  //
  //Check for consistent length of input parameters
  if (0 != ((hv_ParamName.TupleLength())!=(hv_ParamValue.TupleLength())))
  {
    throw HException("Different number of values in ParamName and ParamValue");
  }
  //First, get the parameter names that correspond to the
  //elements in the input camera parameter tuple.
  get_cam_par_names(hv_CameraParamIn, &hv_CameraType, &hv_CameraParamNames);
  //
  //Find the index of the requested camera data and return
  //the corresponding value.
  (*hv_CameraParamOut) = hv_CameraParamIn;
  {
  HTuple end_val16 = (hv_ParamName.TupleLength())-1;
  HTuple step_val16 = 1;
  for (hv_Index=0; hv_Index.Continue(end_val16, step_val16); hv_Index += step_val16)
  {
    hv_ParamNameInd = HTuple(hv_ParamName[hv_Index]);
    hv_I = hv_CameraParamNames.TupleFind(hv_ParamNameInd);
    if (0 != (hv_I!=-1))
    {
      (*hv_CameraParamOut)[hv_I] = HTuple(hv_ParamValue[hv_Index]);
    }
    else
    {
      throw HException("Wrong ParamName "+hv_ParamNameInd);
    }
    //Check the consistency of focus and telecentricity
    if (0 != (hv_ParamNameInd==HTuple("focus")))
    {
      hv_IsTelecentric = HTuple((hv_CameraType.TupleStrstr("telecentric"))!=-1).TupleAnd((hv_CameraType.TupleStrstr("image_side_telecentric"))==-1);
      if (0 != hv_IsTelecentric)
      {
        throw HException(HTuple("Focus for telecentric lenses is always 0, and hence, cannot be changed."));
      }
      if (0 != (HTuple(hv_IsTelecentric.TupleNot()).TupleAnd(HTuple(hv_ParamValue[hv_Index])==0.0)))
      {
        throw HException("Focus for non-telecentric lenses must not be 0.");
      }
    }
  }
  }
  return;
}


//分别把不同任务加过来
// 1.1 工业相机原始图像定位矩形标志
void rectangle_pose_MERO(HObject ho_Image, double Pose[6], bool &Flag)
{
  // Local iconic variables
  HObject  ho_Image1, ho_Image2, ho_Image3;
  HObject  ho_ImageH, ho_ImageS, ho_ImageV, ho_Regions, ho_RegionOpening;
  HObject  ho_ConnectedRegions, ho_SelectedRegions, ho_ObjectSelected;
  HObject  ho_RegionClosing, ho_Contour, ho_ContoursSplit;
  HObject  ho_SelectedEdges, ho_UnionContours, ho_UnionContours2;

  // Local control variables
  HTuple  hv_RectWidth, hv_RectHeight, hv_CamParOriginal;
  HTuple  hv_REC, hv_index, hv_indexes_REC, hv_Pose, hv_CovPose;
  HTuple  hv_Error, hv_Exception;

  try
  {
      //step1:根据颜色提取指定颜色的砖块区域
      if (brick_color=="O")
      {
        hv_RectWidth = 0.19;
        hv_RectHeight = 0.15;
      }
      else if (brick_color=="B")
      {
        hv_RectWidth = 0.3;
        hv_RectHeight = 0.15;
      }
      else if (brick_color=="G")
      {
        hv_RectWidth = 0.3;
        hv_RectHeight = 0.15;
      }
      else if (brick_color=="R")
      {
        hv_RectWidth = 0.2;
        hv_RectHeight = 0.15;
      }

      ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/camparMER.dat", &hv_CamParOriginal);

      Decompose3(ho_Image, &ho_Image1, &ho_Image2, &ho_Image3);
      TransFromRgb(ho_Image1, ho_Image2, ho_Image3, &ho_ImageH, &ho_ImageS, &ho_ImageV, 
          "hsv");
      Threshold(ho_ImageS, &ho_Regions, 0, 100);
      OpeningCircle(ho_Regions, &ho_RegionOpening, 5);

      Connection(ho_RegionOpening, &ho_ConnectedRegions);

      SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, (HTuple("rectangularity").Append("area")), 
          "and", (HTuple(0.85).Append(10000)), (HTuple(1).Append(100000000)));
      //对区域根据retangularity进行排序
      RegionFeatures(ho_SelectedRegions, "rectangularity", &hv_REC);
      TupleSortIndex(hv_REC, &hv_index);
      TupleInverse(hv_index, &hv_indexes_REC);
      hv_index = HTuple(hv_indexes_REC[0])+1;
      SelectObj(ho_SelectedRegions, &ho_ObjectSelected, hv_index);
      ClosingCircle(ho_ObjectSelected, &ho_RegionClosing, 10);
      GenContourRegionXld(ho_RegionClosing, &ho_Contour, "border");
      SegmentContoursXld(ho_Contour, &ho_ContoursSplit, "lines", 7, 5, 3);

      try
      {
        SelectContoursXld(ho_ContoursSplit, &ho_SelectedEdges, "contour_length", 100, 
            2000, -1.5, 1.5);
        UnionAdjacentContoursXld(ho_SelectedEdges, &ho_UnionContours, 10, 1, "attr_keep");
        GetRectanglePose(ho_UnionContours, hv_CamParOriginal, hv_RectWidth, hv_RectHeight, 
            "huber", 1, &hv_Pose, &hv_CovPose, &hv_Error);
        
        for(int i=0;i<6;i++)
        {
          Pose[i] = hv_Pose[i].D();
        }
        Flag = true;

      }
      catch (HException &HDevExpDefaultException)
      {
        HDevExpDefaultException.ToHTuple(&hv_Exception);
        try
        {
          SelectContoursXld(ho_ContoursSplit, &ho_SelectedEdges, "contour_length", 
              100, 2000, -0.5, 0.5);
          //将共线的轮廓连接起来
          UnionCollinearContoursExtXld(ho_SelectedEdges, &ho_UnionContours2, 500, 10, 
              50, 0.7, 0, -1, 1, 1, 1, 1, 1, 0, "attr_keep");
          UnionAdjacentContoursXld(ho_UnionContours2, &ho_UnionContours, 10, 1, "attr_keep");
          GetRectanglePose(ho_UnionContours, hv_CamParOriginal, hv_RectWidth, hv_RectHeight, 
              "huber", 1, &hv_Pose, &hv_CovPose, &hv_Error);

          for(int i=0;i<6;i++)
          {
            Pose[i] = hv_Pose[i].D();
          }
          Flag = true;
        }
        catch (HException &exception)
        {
          Flag = false;
          for(int i=0;i<6;i++)
          {
            Pose[i] = 0.0;
          }
          ROS_ERROR("Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
        }
      }
  }
  catch (HException &exception)
  {
      Flag = false;
      for(int i=0;i<6;i++)
      {
        Pose[i] = 0.0;
      }
      ROS_ERROR("Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
  }

}

// 1.2 工业相机校正后图像定位矩形标志
void rectangle_pose_MERC(HObject ho_Image, double Pose[6], bool &Flag)
{
  // Local iconic variables
  HObject  ho_ConnectedRegions, ho_SelectedRegions;
  HObject  ho_ObjectSelected, ho_RegionClosing, ho_Contour;
  HObject  ho_ContoursSplit, ho_SelectedEdges, ho_UnionContours;
  HObject  ho_UnionContours2, ho_GrayImage, ho_Edges, ho_EdgesRectifiedAdaptive;
  HObject  ho_ImageRectifiedAdaptive, ho_Region;

  // Local control variables
  HTuple  hv_RectWidth, hv_RectHeight, hv_CamParOriginal;
  HTuple  hv_REC, hv_index, hv_indexes_REC, hv_Pose, hv_CovPose;
  HTuple  hv_Error, hv_Exception, hv_CamParOriginal_Corrected;
  HTuple  hv_CamParVirtualAdaptive;

  try
  {
      //step1:根据颜色提取指定颜色的砖块区域
      if (brick_color=="O")
      {
        hv_RectWidth = 0.19;
        hv_RectHeight = 0.15;
      }
      else if (brick_color=="B")
      {
        hv_RectWidth = 0.3;
        hv_RectHeight = 0.15;
      }
      else if (brick_color=="G")
      {
        hv_RectWidth = 0.3;
        hv_RectHeight = 0.15;
      }
      else if (brick_color=="R")
      {
        hv_RectWidth = 0.2;
        hv_RectHeight = 0.15;
      }

      ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/camparMER.dat", &hv_CamParOriginal);
      ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/camparMER_Corrected.dat", &hv_CamParOriginal_Corrected);

      Rgb1ToGray(ho_Image, &ho_GrayImage);
      EdgesSubPix(ho_GrayImage, &ho_Edges, "lanser2", 0.5, 20, 40);
      //Change the radial distortion: mode 'adaptive'
      ChangeRadialDistortionCamPar("adaptive", hv_CamParOriginal, 0, &hv_CamParVirtualAdaptive);
      ChangeRadialDistortionContoursXld(ho_Edges, &ho_EdgesRectifiedAdaptive, hv_CamParOriginal, 
          hv_CamParVirtualAdaptive);

      //get  ImageRectifiedAdaptive
      ChangeRadialDistortionImage(ho_GrayImage, ho_GrayImage, &ho_ImageRectifiedAdaptive, 
          hv_CamParOriginal, hv_CamParVirtualAdaptive);
      Threshold(ho_ImageRectifiedAdaptive, &ho_Region, 131, 255);
      Connection(ho_Region, &ho_ConnectedRegions);
      SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, (HTuple("rectangularity").Append("area")), 
          "and", (HTuple(0.85).Append(10000)), (HTuple(1).Append(100000000)));

      RegionFeatures(ho_SelectedRegions, "rectangularity", &hv_REC);
      TupleSortIndex(hv_REC, &hv_index);
      TupleInverse(hv_index, &hv_indexes_REC);
      hv_index = HTuple(hv_indexes_REC[0])+1;
      SelectObj(ho_SelectedRegions, &ho_ObjectSelected, hv_index);
      //
      ClosingCircle(ho_ObjectSelected, &ho_RegionClosing, 10);
      GenContourRegionXld(ho_RegionClosing, &ho_Contour, "border");
      SegmentContoursXld(ho_Contour, &ho_ContoursSplit, "lines", 7, 5, 3);

      try
      {
        SelectContoursXld(ho_ContoursSplit, &ho_SelectedEdges, "contour_length", 100, 
            2000, -1.5, 1.5);
        UnionAdjacentContoursXld(ho_SelectedEdges, &ho_UnionContours, 10, 1, "attr_keep");
        GetRectanglePose(ho_UnionContours, hv_CamParOriginal_Corrected, hv_RectWidth, 
            hv_RectHeight, "huber", 1, &hv_Pose, &hv_CovPose, &hv_Error);
        
        for(int i=0;i<6;i++)
        {
          Pose[i] = hv_Pose[i].D();
        }
        Flag = true;

      }
      catch (HException &HDevExpDefaultException)
      {
        HDevExpDefaultException.ToHTuple(&hv_Exception);
        try
        {
          SelectContoursXld(ho_ContoursSplit, &ho_SelectedEdges, "contour_length", 
              100, 2000, -0.5, 0.5);
          //将共线的轮廓连接起来
          UnionCollinearContoursExtXld(ho_SelectedEdges, &ho_UnionContours2, 500, 10, 
              50, 0.7, 0, -1, 1, 1, 1, 1, 1, 0, "attr_keep");
          UnionAdjacentContoursXld(ho_UnionContours2, &ho_UnionContours, 10, 1, "attr_keep");
          GetRectanglePose(ho_UnionContours, hv_CamParOriginal_Corrected, hv_RectWidth, 
              hv_RectHeight, "huber", 1, &hv_Pose, &hv_CovPose, &hv_Error);

          for(int i=0;i<6;i++)
          {
            Pose[i] = hv_Pose[i].D();
          }
          Flag = true;
        }
        catch (HException &exception)
        {
          Flag = false;
          for(int i=0;i<6;i++)
          {
            Pose[i] = 0.0;
          }
          ROS_ERROR("Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
        }
      }
  }
  catch (HException &exception)
  {
      Flag = false;
      for(int i=0;i<6;i++)
      {
        Pose[i] = 0.0;
      }
      ROS_ERROR("Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
  }

}

// 1.3 ZED相机校正后图像定位矩形标志
void rectangle_pose_ZED(HObject ho_Image, double Pose[6], bool &Flag)
{
   // Local iconic variables
  HObject  ho_Image1, ho_Image2, ho_Image3;
  HObject  ho_ImageH, ho_ImageS, ho_ImageV, ho_Regions, ho_RegionOpening;
  HObject  ho_ConnectedRegions, ho_SelectedRegions, ho_ObjectSelected;
  HObject  ho_RegionClosing, ho_Contour, ho_ContoursSplit;
  HObject  ho_SelectedEdges, ho_UnionContours, ho_UnionContours2;

  // Local control variables
  HTuple  hv_RectWidth, hv_RectHeight, hv_CamParOriginal;
  HTuple  hv_REC, hv_index, hv_indexes_REC, hv_Pose, hv_CovPose;
  HTuple  hv_Error, hv_Exception;

  try
  {
      //step1:根据颜色提取指定颜色的砖块区域
      if (brick_color=="O")
      {
        hv_RectWidth = 0.19;
        hv_RectHeight = 0.15;
      }
      else if (brick_color=="B")
      {
        hv_RectWidth = 0.3;
        hv_RectHeight = 0.15;
      }
      else if (brick_color=="G")
      {
        hv_RectWidth = 0.3;
        hv_RectHeight = 0.15;
      }
      else if (brick_color=="R")
      {
        hv_RectWidth = 0.2;
        hv_RectHeight = 0.15;
      }

      ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar2_01.dat", &hv_CamParOriginal);

      Decompose3(ho_Image, &ho_Image1, &ho_Image2, &ho_Image3);
      TransFromRgb(ho_Image1, ho_Image2, ho_Image3, &ho_ImageH, &ho_ImageS, &ho_ImageV, 
          "hsv");
      Threshold(ho_ImageS, &ho_Regions, 0, 50);
      OpeningCircle(ho_Regions, &ho_RegionOpening, 5);

      Connection(ho_RegionOpening, &ho_ConnectedRegions);

      SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, (HTuple("rectangularity").Append("area")), 
          "and", (HTuple(0.8).Append(1000)), (HTuple(1).Append(1000000)));
      //对区域根据retangularity进行排序
      RegionFeatures(ho_SelectedRegions, "rectangularity", &hv_REC);
      TupleSortIndex(hv_REC, &hv_index);
      TupleInverse(hv_index, &hv_indexes_REC);
      hv_index = HTuple(hv_indexes_REC[0])+1;
      SelectObj(ho_SelectedRegions, &ho_ObjectSelected, hv_index);
      ClosingCircle(ho_ObjectSelected, &ho_RegionClosing, 10);
      GenContourRegionXld(ho_RegionClosing, &ho_Contour, "border");
      SegmentContoursXld(ho_Contour, &ho_ContoursSplit, "lines", 5, 8, 3);

      try
      {
        SelectContoursXld(ho_ContoursSplit, &ho_SelectedEdges, "contour_length", 100, 
            2000, -1.5, 1.5);
        UnionAdjacentContoursXld(ho_SelectedEdges, &ho_UnionContours, 10, 1, "attr_keep");
        GetRectanglePose(ho_UnionContours, hv_CamParOriginal, hv_RectWidth, hv_RectHeight, 
            "huber", 1, &hv_Pose, &hv_CovPose, &hv_Error);
        
        for(int i=0;i<6;i++)
        {
          Pose[i] = hv_Pose[i].D();
        }
        Flag = true;

      }
      catch (HException &HDevExpDefaultException)
      {
        HDevExpDefaultException.ToHTuple(&hv_Exception);
        try
        {
          SelectContoursXld(ho_ContoursSplit, &ho_SelectedEdges, "contour_length", 
              125, 2000, -0.5, 0.5);
          //将共线的轮廓连接起来
          UnionCollinearContoursExtXld(ho_SelectedEdges, &ho_UnionContours2, 500, 10, 
              50, 0.7, 0, -1, 1, 1, 1, 1, 1, 0, "attr_keep");
          UnionAdjacentContoursXld(ho_UnionContours2, &ho_UnionContours, 1000, 1, "attr_keep");
          GetRectanglePose(ho_UnionContours, hv_CamParOriginal, hv_RectWidth, hv_RectHeight, 
              "huber", 1, &hv_Pose, &hv_CovPose, &hv_Error);

          for(int i=0;i<6;i++)
          {
            Pose[i] = hv_Pose[i].D();
          }
          Flag = true;
        }
        catch (HException &exception)
        {
          Flag = false;
          for(int i=0;i<6;i++)
          {
            Pose[i] = 0.0;
          }
          ROS_ERROR("Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
        }
      }
  }
  catch (HException &exception)
  {
      Flag = false;
      for(int i=0;i<6;i++)
      {
        Pose[i] = 0.0;
      }
      ROS_ERROR("Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
  }

}

// 1.4 ZED相机校正后图像新方法定位矩形标志
void rectangle_pose_ZED_new(HObject ho_Image, double Pose[6], bool &Flag, int &OrangeIndex)
{
  // Local iconic variables
  HObject  ho_ClassRegions, ho_ColorObjectSelected;
  HObject  ho_RectangleSelect, ho_GrayImage, ho_Red, ho_Green;
  HObject  ho_Blue, ho_Hue, ho_Saturation, ho_Intensity, ho_ImageMedian;
  HObject  ho_DynRegion, ho_Contours, ho_FilledRegion, ho_SelectedRegions11;
  HObject  ho_SelectedRegions12, ho_SelectedRegions13, ho_SelectedRegions14;
  HObject  ho_ImageSub, ho_SelectedRegions21, ho_SelectedRegions22;
  HObject  ho_SelectedRegions23, ho_SelectedRegions24, ho_RegionIntersection;
  HObject  ho_RegionDifference1, ho_RegionDifference2, ho_RegionOpening1;
  HObject  ho_RegionOpening2, ho_RegionUnion1, ho_RegionUnion;
  HObject  ho_ConnectedRegions, ho_ObjectSelected, ho_ObjectROI_2;
  HObject  ho_OrangeRegions, ho_OrangeSelected, ho_Rectangle1;
  HObject  ho_Rectangle, ho_RectangleLeft, ho_ImageReducedLeft;
  HObject  ho_LeftRegions, ho_ObjectSelectedLeft, ho_RectangleRight;
  HObject  ho_ImageReducedRight, ho_RightRegions, ho_ObjectSelectedRight;
  HObject  ho_Cross, ho_ResultContour, ho_Contour, ho_ROIContour;
  HObject  ho_ClosedContours, ho_RegionROI, ho_ContoursROI;
  HObject  ho_ContoursSelected, ho_Rectangle2;

  // Local control variables
  HTuple  hv_pathFile, hv_MLPHandle, hv_CamParOriginal;
  HTuple  hv_ImageFiles, hv_Brick_color, hv_coord_label, hv_Index;
  HTuple  hv_pose, hv_Width, hv_Height, hv_RectWidth, hv_RectHeight;
  HTuple  hv_color_index, hv_RowSelect, hv_ColumnSelect, hv_PhiSelect;
  HTuple  hv_Length1Select, hv_Length2Select, hv_Number, hv_REC_row;
  HTuple  hv_REC_column, hv_weight, hv_Index1, hv_index, hv_Row2;
  HTuple  hv_Column2, hv_Phi1, hv_Length11, hv_Length21, hv_NumberOrange;
  HTuple  hv_Row, hv_Column, hv_Phi, hv_Length1, hv_Length2;
  HTuple  hv_AreaAnchor, hv_AreaLeft, hv_Row1, hv_Column1;
  HTuple  hv_AreaRight, hv_CenterY, hv_CenterX, hv_Len1, hv_Len2;
  HTuple  hv_Area, hv_VertexesY, hv_VertexesX, hv_MetrologyHandle;
  HTuple  hv_LineRow1, hv_LineColumn1, hv_LineRow2, hv_LineColumn2;
  HTuple  hv_Tolerance, hv_RowBegin, hv_ColBegin, hv_RowEnd;
  HTuple  hv_ColEnd, hv_Nr, hv_Nc, hv_Dist, hv_J, hv_IsOverlapping;
  HTuple  hv_Rows, hv_Columns, hv_Pose, hv_CovPose, hv_Error;
  HTuple  hv_Exception, hv_Row3, hv_Column3, hv_Phi2, hv_Length12;
  HTuple  hv_Length22;

  try
  {
    hv_pathFile = "/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/brick_color_classify_0111.mlp";
    ReadClassMlp(hv_pathFile, &hv_MLPHandle);
    ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar2_01.dat", &hv_CamParOriginal);
    OrangeIndex = 0;
    hv_pose = HTuple();

    //step1:根据颜色提取指定颜色的砖块区域
    if (brick_color=="O")
    {
      hv_RectWidth = 0.19;
      hv_RectHeight = 0.15;
      hv_color_index = 4;
    }
    else if (brick_color=="B")
    {
      hv_RectWidth = 0.3;
      hv_RectHeight = 0.15;
      hv_color_index = 3;
    }
    else if (brick_color=="G")
    {
      hv_RectWidth = 0.3;
      hv_RectHeight = 0.15;
      hv_color_index = 2;
    }
    else if (brick_color=="R")
    {
      hv_RectWidth = 0.2;
      hv_RectHeight = 0.15;
      hv_color_index = 1;
    }

    GetImageSize(ho_Image, &hv_Width, &hv_Height);
    //颜色分类
    ClassifyImageClassMlp(ho_Image, &ho_ClassRegions, hv_MLPHandle, 0.9);
    SelectObj(ho_ClassRegions, &ho_ColorObjectSelected, hv_color_index);
    SmallestRectangle2(ho_ColorObjectSelected, &hv_RowSelect, &hv_ColumnSelect, &hv_PhiSelect, 
        &hv_Length1Select, &hv_Length2Select);
    GenRectangle2(&ho_RectangleSelect, hv_RowSelect, hv_ColumnSelect, hv_PhiSelect, 
        hv_Length1Select, hv_Length2Select);

    Rgb1ToGray(ho_Image, &ho_GrayImage);
    Decompose3(ho_Image, &ho_Red, &ho_Green, &ho_Blue);
    TransFromRgb(ho_Red, ho_Green, ho_Blue, &ho_Hue, &ho_Saturation, &ho_Intensity, "hsv");

    //方式一
    MedianImage(ho_Saturation, &ho_ImageMedian, "square", 9, "mirrored");
    VarThreshold(ho_ImageMedian, &ho_DynRegion, 35, 35, 0.2, 5, "dark");
    GenContourRegionXld(ho_DynRegion, &ho_Contours, "border");
    GenRegionContourXld(ho_Contours, &ho_FilledRegion, "filled");
    //分开写便于调试参数
    SelectShape(ho_FilledRegion, &ho_SelectedRegions11, "rectangularity", "and", 0.7, 1.0);
    SelectShape(ho_SelectedRegions11, &ho_SelectedRegions12, (HTuple("rect2_len1").Append("rect2_len2")), "and", (HTuple(80).Append(70)), (HTuple(999).Append(999)));
    SelectGray(ho_SelectedRegions12, ho_ImageMedian, &ho_SelectedRegions13, "mean", "and", 0, 70);
    SelectGray(ho_SelectedRegions13, ho_ImageMedian, &ho_SelectedRegions14, "deviation", "and", 0, 50);

    //方式二
    SubImage(ho_GrayImage, ho_Saturation, &ho_ImageSub, 1, 0);
    MedianImage(ho_ImageSub, &ho_ImageMedian, "square", 9, "mirrored");
    VarThreshold(ho_ImageMedian, &ho_DynRegion, 35, 35, 0.2, 5, "light");
    GenContourRegionXld(ho_DynRegion, &ho_Contours, "border");
    GenRegionContourXld(ho_Contours, &ho_FilledRegion, "filled");
    //分开写便于调试参数
    SelectShape(ho_FilledRegion, &ho_SelectedRegions21, "rectangularity", "and", 0.7, 1.0);
    SelectShape(ho_SelectedRegions21, &ho_SelectedRegions22, (HTuple("rect2_len1").Append("rect2_len2")),  "and", (HTuple(80).Append(70)), (HTuple(999).Append(999)));
    SelectGray(ho_SelectedRegions22, ho_ImageMedian, &ho_SelectedRegions23, "mean", "and", 70, 255);
    SelectGray(ho_SelectedRegions23, ho_ImageMedian, &ho_SelectedRegions24, "deviation", "and", 0, 50);

    //方式一与方式二结果合并
    Intersection(ho_SelectedRegions14, ho_SelectedRegions24, &ho_RegionIntersection);
    Difference(ho_SelectedRegions14, ho_SelectedRegions24, &ho_RegionDifference1);
    Difference(ho_SelectedRegions24, ho_SelectedRegions14, &ho_RegionDifference2);

    OpeningCircle(ho_RegionDifference1, &ho_RegionOpening1, 15);
    OpeningCircle(ho_RegionDifference2, &ho_RegionOpening2, 15);
    Union2(ho_RegionIntersection, ho_RegionOpening1, &ho_RegionUnion1);
    Union2(ho_RegionUnion1, ho_RegionOpening2, &ho_RegionUnion);

    Connection(ho_RegionUnion, &ho_ConnectedRegions);

    Intersection(ho_RectangleSelect, ho_ConnectedRegions, &ho_ConnectedRegions);
    Connection(ho_ConnectedRegions, &ho_ConnectedRegions);
    CountObj(ho_ConnectedRegions, &hv_Number);

    //判断合并后的结果是否为空
    if (hv_Number==0)
    {
      Flag = false;
      for(int i=0;i<6;i++)
      {
        Pose[i] = 0.0;
      }
      ROS_WARN("Can't find brick");
      return;
    }
    else
    {
      //如果含有多个,自己定义选择
      RegionFeatures(ho_ConnectedRegions, "row", &hv_REC_row);
      RegionFeatures(ho_ConnectedRegions, "column", &hv_REC_column);
      hv_weight = HTuple();
      {
      HTuple end_val92 = hv_Number-1;
      HTuple step_val92 = 1;
      for (hv_Index1=0; hv_Index1.Continue(end_val92, step_val92); hv_Index1 += step_val92)
      {
        hv_weight[hv_Index1] = ((hv_Height-HTuple(hv_REC_row[hv_Index1]))*(hv_Height-HTuple(hv_REC_row[hv_Index1])))+(((hv_Width/2)-HTuple(hv_REC_column[hv_Index1]))*((hv_Width/2)-HTuple(hv_REC_column[hv_Index1])));
      }
      }
      TupleSortIndex(hv_weight, &hv_index);
      SelectObj(ho_ConnectedRegions, &ho_ObjectSelected, HTuple(hv_index[0])+1);
      //only one region selected
    }

    //judge the color is orange?
    if (brick_color=="O")
    {
      ClassifyImageClassMlp(ho_Image, &ho_OrangeRegions, hv_MLPHandle, 0.9);
      SelectObj(ho_OrangeRegions, &ho_OrangeSelected, 4);
      SmallestRectangle2(ho_OrangeSelected, &hv_Row2, &hv_Column2, &hv_Phi1, &hv_Length11, &hv_Length21);
      GenRectangle2(&ho_Rectangle1, hv_Row2, hv_Column2, hv_Phi1, hv_Length11, hv_Length21);
      Intersection(ho_Rectangle1, ho_ObjectSelected, &ho_RegionIntersection);
      CountObj(ho_RegionIntersection, &hv_NumberOrange);

      if (hv_NumberOrange==0)
      {
        Flag = false;
        for(int i=0;i<6;i++)
        {
          Pose[i] = 0.0;
        }
        ROS_WARN("Can't find orange brick");
        return;
      }

      SmallestRectangle2(ho_RegionIntersection, &hv_Row, &hv_Column, &hv_Phi, &hv_Length1, &hv_Length2);
      GenRectangle2(&ho_Rectangle, hv_Row, hv_Column, hv_Phi, hv_Length1, hv_Length2);
      AreaCenter(ho_Rectangle, &hv_AreaAnchor, &hv_Row, &hv_Column);

      //Left region
      GenRectangle2(&ho_RectangleLeft, hv_Row-((2*hv_Length1)*((-hv_Phi).TupleSin())), 
          hv_Column-((2*hv_Length1)*((-hv_Phi).TupleCos())), hv_Phi, hv_Length1, 
          hv_Length2);
      ReduceDomain(ho_Image, ho_RectangleLeft, &ho_ImageReducedLeft);
      ClassifyImageClassMlp(ho_ImageReducedLeft, &ho_LeftRegions, hv_MLPHandle, 0.9);
      SelectObj(ho_LeftRegions, &ho_ObjectSelectedLeft, 4);
      AreaCenter(ho_ObjectSelectedLeft, &hv_AreaLeft, &hv_Row1, &hv_Column1);

      //Right Region
      GenRectangle2(&ho_RectangleRight, hv_Row+((2*hv_Length1)*((-hv_Phi).TupleSin())), 
          hv_Column+((2*hv_Length1)*((-hv_Phi).TupleCos())), hv_Phi, hv_Length1, 
          hv_Length2);
      ReduceDomain(ho_Image, ho_RectangleRight, &ho_ImageReducedRight);
      ClassifyImageClassMlp(ho_ImageReducedRight, &ho_RightRegions, hv_MLPHandle, 0.9);
      SelectObj(ho_RightRegions, &ho_ObjectSelectedRight, 4);
      AreaCenter(ho_ObjectSelectedRight, &hv_AreaRight, &hv_Row1, &hv_Column1);

      if (hv_AreaLeft>(hv_AreaAnchor*0.4))
      {
        if (hv_AreaRight>(hv_AreaAnchor*0.4))
        {
          OrangeIndex = 0;     // Center
        }
        else
        {
          OrangeIndex = 1;     // Right
        }
      }
      else
      {
        OrangeIndex = -1;      // Left
      }
      ho_ObjectSelected = ho_RegionIntersection;
    }
    
    try
    {
      SmallestRectangle2(ho_ObjectSelected, &hv_CenterY, &hv_CenterX, &hv_Phi, &hv_Len1, &hv_Len2);

      AreaCenter(ho_ObjectSelected, &hv_Area, &hv_Row, &hv_Column);

      GenRectangle2(&ho_Rectangle, hv_CenterY, hv_CenterX, hv_Phi, hv_Len1, hv_Len2);
      get_rectangle2_points(hv_CenterY, hv_CenterX, hv_Phi, hv_Len1, hv_Len2, &hv_VertexesY, &hv_VertexesX);
      GenCrossContourXld(&ho_Cross, hv_VertexesY, hv_VertexesX, 60, hv_Phi);

      CreateMetrologyModel(&hv_MetrologyHandle);
      SetMetrologyModelImageSize(hv_MetrologyHandle, hv_Width, hv_Height);

      hv_LineRow1.Clear();
      hv_LineRow1.Append(HTuple(hv_VertexesY[0]));
      hv_LineRow1.Append(HTuple(hv_VertexesY[1]));
      hv_LineRow1.Append(HTuple(hv_VertexesY[2]));
      hv_LineRow1.Append(HTuple(hv_VertexesY[3]));
      hv_LineColumn1.Clear();
      hv_LineColumn1.Append(HTuple(hv_VertexesX[0]));
      hv_LineColumn1.Append(HTuple(hv_VertexesX[1]));
      hv_LineColumn1.Append(HTuple(hv_VertexesX[2]));
      hv_LineColumn1.Append(HTuple(hv_VertexesX[3]));
      hv_LineRow2.Clear();
      hv_LineRow2.Append(HTuple(hv_VertexesY[1]));
      hv_LineRow2.Append(HTuple(hv_VertexesY[2]));
      hv_LineRow2.Append(HTuple(hv_VertexesY[3]));
      hv_LineRow2.Append(HTuple(hv_VertexesY[0]));
      hv_LineColumn2.Clear();
      hv_LineColumn2.Append(HTuple(hv_VertexesX[1]));
      hv_LineColumn2.Append(HTuple(hv_VertexesX[2]));
      hv_LineColumn2.Append(HTuple(hv_VertexesX[3]));
      hv_LineColumn2.Append(HTuple(hv_VertexesX[0]));

      hv_Tolerance = 50;

      AddMetrologyObjectLineMeasure(hv_MetrologyHandle, hv_LineRow1, hv_LineColumn1,hv_LineRow2, hv_LineColumn2, hv_Tolerance, 50, 1.5, 5, HTuple(), HTuple(), &hv_Index1);
      SetMetrologyObjectParam(hv_MetrologyHandle, hv_Index1, "num_instances", 1);
      SetMetrologyObjectParam(hv_MetrologyHandle, hv_Index1, "measure_select", "first");
      SetMetrologyObjectParam(hv_MetrologyHandle, hv_Index1, "min_score", .7);

      ApplyMetrologyModel(ho_Saturation, hv_MetrologyHandle);

      //Access results
      GetMetrologyObjectResultContour(&ho_ResultContour, hv_MetrologyHandle, "all", "all", 1.5);
      GetMetrologyObjectMeasures(&ho_Contour, hv_MetrologyHandle, "all", "all", &hv_Row1, &hv_Column1);
      GenCrossContourXld(&ho_Cross, hv_Row1, hv_Column1, 16, 0.785398);

      FitLineContourXld(ho_ResultContour, "tukey", -1, 0, 5, 2, &hv_RowBegin, &hv_ColBegin, &hv_RowEnd, &hv_ColEnd, &hv_Nr, &hv_Nc, &hv_Dist);
      //Find intersection points [Rows, Columns]
      for (hv_J=0; hv_J<=3; hv_J+=1)
      {
        IntersectionLines(HTuple(hv_RowBegin[hv_J]), HTuple(hv_ColBegin[hv_J]), HTuple(hv_RowEnd[hv_J]), 
                          HTuple(hv_ColEnd[hv_J]), HTuple(hv_RowBegin[(hv_J+1)%4]), HTuple(hv_ColBegin[(hv_J+1)%4]), 
                          HTuple(hv_RowEnd[(hv_J+1)%4]), HTuple(hv_ColEnd[(hv_J+1)%4]), &hv_Row, &hv_Column, &hv_IsOverlapping);
        GenCrossContourXld(&ho_Cross, hv_Row, hv_Column, 60, 0.785398);
        hv_Rows[hv_J] = hv_Row;
        hv_Columns[hv_J] = hv_Column;
      }

      GenContourPolygonXld(&ho_ROIContour, hv_Rows, hv_Columns);
      CloseContoursXld(ho_ROIContour, &ho_ClosedContours);

      GenRegionContourXld(ho_ClosedContours, &ho_RegionROI, "filled");
      GenContourRegionXld(ho_RegionROI, &ho_ContoursROI, "border");

      GetRectanglePose(ho_ContoursROI, hv_CamParOriginal, hv_RectWidth, hv_RectHeight, "tukey", 1, &hv_Pose, &hv_CovPose, &hv_Error);
      for(int i=0;i<6;i++)
      {
        Pose[i] = hv_Pose[i].D();
      }
      Flag = true;
    }
    catch (HException &exception)
    {
      ROS_WARN("First method can't find, #%u in %s: %s\n", exception.ErrorCode(),
        (const char *)exception.ProcName(),
        (const char *)exception.ErrorMessage());
      SmallestRectangle2(ho_ObjectSelected, &hv_Row3, &hv_Column3, &hv_Phi2, &hv_Length12, 
          &hv_Length22);
      GenRectangle2(&ho_Rectangle2, hv_Row3, hv_Column3, hv_Phi2, hv_Length12, hv_Length22);
      GenContourRegionXld(ho_Rectangle2, &ho_ContoursSelected, "border");
      GetRectanglePose(ho_ContoursSelected, hv_CamParOriginal, hv_RectWidth, hv_RectHeight, 
          "tukey", 1, &hv_Pose, &hv_CovPose, &hv_Error);
      for(int i=0;i<6;i++)
      {
        Pose[i] = hv_Pose[i].D();
      }
      Flag = true;
    }
  }
  catch (HException &exception)
  {
    Flag = false;
    for(int i=0;i<6;i++)
    {
      Pose[i] = 0.0;
    }
    ROS_ERROR("Error #%u in %s: %s\n", exception.ErrorCode(),
        (const char *)exception.ProcName(),
        (const char *)exception.ErrorMessage());
  }

}


// 2.定位砖堆位置 图像来源:zed 双目
void color_bricks_location(HObject ho_ImageL,HObject ho_ImageR, double Pose[6], bool &Flag)
{
  HObject  ho_ClassRegions;
  HObject  ho_ClassRegions2, ho_ClassRegion, ho_ClassRegion2;
  HObject  ho_ConnectedRegions, ho_SelectedRegions, ho_ConnectedRegions2;
  HObject  ho_SelectedRegions2;

  // Local control variables
  HTuple  hv_pathFile, hv_MLPHandle;
  HTuple  hv_index, hv_Area, hv_Row, hv_Column, hv_Area2;
  HTuple  hv_Row2, hv_Column2, hv_Indices, hv_Inverted, hv_R;
  HTuple  hv_C, hv_Indices2, hv_Inverted2, hv_R2, hv_C2, hv_RelPose;
  HTuple  hv_CamParam1, hv_CamParam2;
  HTuple  hv_Dist, hv_Exception;
  HTuple  hv_X, hv_Y, hv_Z;
  try
  {
      ReadPose("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/relpose_01.dat", &hv_RelPose);
      ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar1_01.dat", &hv_CamParam1);
      ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar2_01.dat", &hv_CamParam2);

      hv_pathFile = "/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/brick_color_classify_0111.mlp";
      ReadClassMlp(hv_pathFile, &hv_MLPHandle);
      //识别
      
      try
      {
          ClassifyImageClassMlp(ho_ImageL, &ho_ClassRegions, hv_MLPHandle, 0.9);
          ClassifyImageClassMlp(ho_ImageR, &ho_ClassRegions2, hv_MLPHandle, 0.9);

          ROS_INFO("begin classify");

          if (brick_color=="O")
          {
            SelectObj(ho_ClassRegions, &ho_ClassRegion, 1);
            SelectObj(ho_ClassRegions2, &ho_ClassRegion2, 1);
          }
          else if (brick_color=="B")
          {
            SelectObj(ho_ClassRegions, &ho_ClassRegion, 2);
            SelectObj(ho_ClassRegions2, &ho_ClassRegion2, 2);
          }
          else if (brick_color=="G")
          {
            SelectObj(ho_ClassRegions, &ho_ClassRegion, 3);
            SelectObj(ho_ClassRegions2, &ho_ClassRegion2, 3);
          }
          else if (brick_color=="R")
          {
            SelectObj(ho_ClassRegions, &ho_ClassRegion, 4);
            SelectObj(ho_ClassRegions2, &ho_ClassRegion2, 4);
          }

          //set area threshold
          Connection(ho_ClassRegion, &ho_ConnectedRegions);
          SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, "area", "and", 150, 99999999);
          AreaCenter(ho_SelectedRegions, &hv_Area, &hv_Row, &hv_Column);

          Connection(ho_ClassRegion2, &ho_ConnectedRegions2);
          SelectShape(ho_ConnectedRegions2, &ho_SelectedRegions2, "area", "and", 150, 99999999);
          AreaCenter(ho_SelectedRegions2, &hv_Area2, &hv_Row2, &hv_Column2);

          //sort according to area
          TupleSortIndex(hv_Area, &hv_Indices);
          TupleInverse(hv_Indices, &hv_Inverted);
          hv_R = HTuple(hv_Row[HTuple(hv_Inverted[0])]);
          hv_C = HTuple(hv_Column[HTuple(hv_Inverted[0])]);

          TupleSortIndex(hv_Area2, &hv_Indices2);
          TupleInverse(hv_Indices2, &hv_Inverted2);
          hv_R2 = HTuple(hv_Row2[HTuple(hv_Inverted2[0])]);
          hv_C2 = HTuple(hv_Column2[HTuple(hv_Inverted2[0])]);

          // ROS_INFO_STREAM("Vision data:"<<hv_R.D()<<","<<hv_C.D());
        
          IntersectLinesOfSight(hv_CamParam1, hv_CamParam2, hv_RelPose, hv_R, hv_C, 
              hv_R2, hv_C2, &hv_X, &hv_Y, &hv_Z, &hv_Dist);
          
          Flag = true;
          Pose[0] = hv_X.D();
          Pose[1] = hv_Y.D();
          Pose[2] = hv_Z.D();
    }
    catch (HException &exception)
    {
      ROS_ERROR("Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
      Flag = false;
    }

  }
  catch (HException &exception)
  {
    ROS_ERROR("Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
    Flag = false;
  }

}

// 3.放砖角度估计
void put_brick(HObject ho_Image1, double Pose[6], bool &Flag)
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
    hv_pathFile = "/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/brick_color_classify_0111.mlp";
    ReadClassMlp(hv_pathFile, &hv_MLPHandle);
    ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/campar1_01.dat", 
        &hv_CameraParam);
    //读入第一张图像 用于识别砖块的轮廓
    GetImageSize(ho_Image1, &hv_Width, &hv_Height);

    ClassifyImageClassMlp(ho_Image1, &ho_ClassRegions, hv_MLPHandle, 0.5);
    //基于先前举起砖块时做的颜色分类结果 先选择砖块对应的区域

    if (brick_color=="R")
    {
      hv_index = 1;
    }
    else if (brick_color=="G")
    {
      hv_index = 2;
    }
    else if (brick_color=="B")
    {
      hv_index = 3;
    }
    SelectObj(ho_ClassRegions, &ho_BrickRegion, hv_index);

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
    Flag = true;
  }
  catch (HException &exception)
  {
    ROS_ERROR("Error #%u in %s: %s\n", exception.ErrorCode(),
            (const char *)exception.ProcName(),
            (const char *)exception.ErrorMessage());
    Flag = false;

    return;
  }

}


//5. L架方向(角度) 图像来源: MER
void L_shelf_orientation(HObject ho_Image,HTuple &hv_Phi1)
{

  // Local iconic variables
  HObject  ho_GrayImage, ho_Edges, ho_EdgesRectifiedAdaptive;
  HObject  ho_ROI, ho_ImageRectifiedAdaptive, ho_SameRegions;
  HObject  ho_SelectedRegions;

  // Local control variables
  HTuple  hv_Pointer, hv_Type, hv_Width, hv_Height;
  HTuple  hv_CamParOriginal, hv_CamParVirtualFixed, hv_CamParVirtualAdaptive;
  HTuple  hv_WidthVirtualAdaptive, hv_HeightVirtualAdaptive;
  HTuple  hv_Phi, hv_Ra, hv_Rb;

  GetImagePointer1(ho_Image, &hv_Pointer, &hv_Type, &hv_Width, &hv_Height);

  Rgb1ToGray(ho_Image, &ho_GrayImage);
  ReadCamPar("model/camparMER.dat", 
      &hv_CamParOriginal);

  EdgesSubPix(ho_GrayImage, &ho_Edges, "lanser2", 0.5, 20, 40);

  //Eliminating radial distortions from the extracted edges
  //Change the radial distortion: set kappa to 0
  hv_CamParVirtualFixed = hv_CamParOriginal;
  set_cam_par_data(hv_CamParVirtualFixed, "kappa", 0, &hv_CamParVirtualFixed);

  //Change the radial distortion: mode 'adaptive'
  ChangeRadialDistortionCamPar("adaptive", hv_CamParOriginal, 0, &hv_CamParVirtualAdaptive);
  ChangeRadialDistortionContoursXld(ho_Edges, &ho_EdgesRectifiedAdaptive, hv_CamParOriginal, 
      hv_CamParVirtualAdaptive);

  get_cam_par_data(hv_CamParVirtualAdaptive, "image_width", &hv_WidthVirtualAdaptive);
  get_cam_par_data(hv_CamParVirtualAdaptive, "image_height", &hv_HeightVirtualAdaptive);
  GenRectangle1(&ho_ROI, 0, 0, hv_HeightVirtualAdaptive-1, hv_WidthVirtualAdaptive-1);
  ChangeRadialDistortionImage(ho_GrayImage, ho_ROI, &ho_ImageRectifiedAdaptive, hv_CamParOriginal, 
      hv_CamParVirtualAdaptive);

  //
  //threshold (ImageRectifiedAdaptive, WhiteRegions, 160, 255)
  Regiongrowing(ho_ImageRectifiedAdaptive, &ho_SameRegions, 1, 1, 6, 100);
  //
  SelectShape(ho_SameRegions, &ho_SelectedRegions, (HTuple("rectangularity").Append("area")), 
      "and", (HTuple(0.8).Append(10000)), (HTuple(1).Append(100000000)));
  //
  OrientationRegion(ho_SelectedRegions, &hv_Phi);
  EllipticAxis(ho_SelectedRegions, &hv_Ra, &hv_Rb, &hv_Phi1);
  //hv_Phi1为L架角度,单位是弧度
}


//初始化halcon对象
HObject  ho_ImageL, ho_ImageR, ho_ImageMER;

//回调函数
void callback_MER(const sensor_msgs::Image::ConstPtr& MERImage) 
{
    //获取MER图像
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointerMER = halcon_bridge::toHalconCopy(MERImage);
    ho_ImageMER = *halcon_bridge_imagePointerMER->image;
    
}

//回调函数
void callback(const sensor_msgs::Image::ConstPtr& LeftImage, const sensor_msgs::Image::ConstPtr& RightImage) 
{
    //获取halcon-bridge图像指针
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointerL = halcon_bridge::toHalconCopy(LeftImage);
    ho_ImageL = *halcon_bridge_imagePointerL->image;
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointerR = halcon_bridge::toHalconCopy(RightImage);
    ho_ImageR = *halcon_bridge_imagePointerR->image;
}

bool GetVisionData(bit_vision_msgs::VisionProc::Request&  req,
                   bit_vision_msgs::VisionProc::Response& res)
{
    ROS_INFO("BrickType:[%s], VisionAlgorithm:[%d]",req.BrickType.c_str(),req.ProcAlgorithm);
    // 设置视觉处理颜色与算法  rectangle_pose
    brick_color = req.BrickType;
    algorithm = req.ProcAlgorithm;

    double ZEDPose[6];  // x,y,z,rx,ry,rz(RPY)
    double MERPose[6];
    bool MER_flag = false;     // 数据置信度
    bool ZED_flag = false;     // 数据置信度
    int OrangeIndex = 0;        // 橙色砖块位置检测标志
    // Local control variables
    HTuple  hv_MSecond, hv_Second, hv_Minute, hv_Hour;
    HTuple  hv_Day, hv_YDay, hv_Month, hv_Year;

    GetSystemTime(&hv_MSecond, &hv_Second, &hv_Minute, &hv_Hour, &hv_Day, &hv_YDay, &hv_Month, &hv_Year);

    switch (algorithm)
    {
        case GetBrickPoseMERO:
            //取砖块的位姿 输入为 MER 图像
            WriteImage(ho_ImageMER, "jpeg", 0, "/home/ugvcontrol/image/MER/Before/"+hv_Month+"-"+hv_Day+"-"+hv_Hour+"-"+hv_Minute+"-"+hv_Second+".jpg");
            rectangle_pose_MERO(ho_ImageMER, MERPose, MER_flag);
            ROS_INFO_STREAM("Vision data:"<<MERPose[0]<<","<<MERPose[1]<<","<<MERPose[2]<<","<<MERPose[3]<<","<<MERPose[4]<<","<<MERPose[5]);
            break;
        case GetBrickPoseMERC:
            //取砖块的位姿 输入为 MER 图像
            WriteImage(ho_ImageMER, "jpeg", 0, "/home/ugvcontrol/image/MER/After/"+hv_Month+"-"+hv_Day+"-"+hv_Hour+"-"+hv_Minute+"-"+hv_Second+".jpg");
            rectangle_pose_MERC(ho_ImageMER, MERPose, MER_flag);
            ROS_INFO_STREAM("Vision data:"<<MERPose[0]<<","<<MERPose[1]<<","<<MERPose[2]<<","<<MERPose[3]<<","<<MERPose[4]<<","<<MERPose[5]);
            break;
        case GetBrickPoseZED:
            //取砖块的位姿 输入为 ZED 图像
            WriteImage(ho_ImageR, "jpeg", 0, "/home/ugvcontrol/image/ZED/Pose/R"+hv_Month+"-"+hv_Day+"-"+hv_Hour+"-"+hv_Minute+"-"+hv_Second+".jpg");
            rectangle_pose_ZED(ho_ImageR, ZEDPose, ZED_flag);
            ROS_INFO_STREAM("Vision data:"<<ZEDPose[0]<<","<<ZEDPose[1]<<","<<ZEDPose[2]<<","<<ZEDPose[3]<<","<<ZEDPose[4]<<","<<ZEDPose[5]);
            break;
        case GetBrickPoseZEDNew:
            //取砖块的位姿 输入为 ZED 图像 新方法
            WriteImage(ho_ImageR, "jpeg", 0, "/home/ugvcontrol/image/ZED/Pose/R"+hv_Month+"-"+hv_Day+"-"+hv_Hour+"-"+hv_Minute+"-"+hv_Second+".jpg");
            rectangle_pose_ZED_new(ho_ImageR, ZEDPose, ZED_flag, OrangeIndex);
            ROS_INFO_STREAM("Vision data:"<<ZEDPose[0]<<","<<ZEDPose[1]<<","<<ZEDPose[2]<<","<<ZEDPose[3]<<","<<ZEDPose[4]<<","<<ZEDPose[5]<<", Index:"<<OrangeIndex);
            break;
        case GetBrickLoc:
            //定位砖堆位置 输入为zed双目
            WriteImage(ho_ImageL, "jpeg", 0, "/home/ugvcontrol/image/ZED/Locate/L"+hv_Month+"-"+hv_Day+"-"+hv_Hour+"-"+hv_Minute+"-"+hv_Second+".jpg");
            WriteImage(ho_ImageR, "jpeg", 0, "/home/ugvcontrol/image/ZED/Locate/R"+hv_Month+"-"+hv_Day+"-"+hv_Hour+"-"+hv_Minute+"-"+hv_Second+".jpg");
            color_bricks_location(ho_ImageL,ho_ImageR, ZEDPose, ZED_flag); 
            ROS_INFO_STREAM("Vision data:"<<ZEDPose[0]<<","<<ZEDPose[1]<<","<<ZEDPose[2]<<","<<ZEDPose[3]<<","<<ZEDPose[4]<<","<<ZEDPose[5]);
            break;
        case GetPutPos:
            //放砖位姿(未完善)
            rectangle_pose_ZED(ho_ImageL, ZEDPose, ZED_flag);
            break;
        case GetPutAngle:
            put_brick(ho_ImageR, ZEDPose, ZED_flag);
            break;
        case GetLPose:
            //L架的角度 输入是MER图像
            L_shelf_orientation(ho_ImageMER,L_shelf_angle);
            break;
        default:
            break;
    }

    if (ZED_flag||MER_flag)
    {
        tf::Transform transform_TargetOnBase;

        tf::Transform transform_TargetOnMER;
        tf::Transform transform_TargetOnZED;
        tf::Quaternion q;
        switch (algorithm)
        {
          case GetBrickPoseMERO:
            transform_TargetOnMER.setOrigin(tf::Vector3(MERPose[0]-0.005, MERPose[1]-0.0059, MERPose[2]));
            q.setRPY(MERPose[3]*Deg2Rad, MERPose[4]*Deg2Rad, MERPose[5]*Deg2Rad);
            transform_TargetOnMER.setRotation(q);
            transform_TargetOnBase = transform_MEROnBase*transform_TargetOnMER;
            ROS_INFO_STREAM("Vision data MERPose:"<<MERPose[0]<<","<<MERPose[1]<<","<<MERPose[2]<<","<<MERPose[3]<<","<<MERPose[4]<<","<<MERPose[5]);

            break;
          case GetBrickPoseMERC:
            transform_TargetOnMER.setOrigin(tf::Vector3(MERPose[0]+ 0.000, MERPose[1]-0.000, MERPose[2]-0.000));
            q.setRPY(MERPose[3]*Deg2Rad, MERPose[4]*Deg2Rad, MERPose[5]*Deg2Rad);
            transform_TargetOnMER.setRotation(q);
            transform_TargetOnBase = transform_MEROnBase*transform_TargetOnMER;
            break;
          case GetBrickPoseZED:
            transform_TargetOnZED.setOrigin(tf::Vector3(ZEDPose[0]+0.003, ZEDPose[1] + 0.0035, ZEDPose[2] + 0.01 ));
            q.setRPY(ZEDPose[3]*Deg2Rad, ZEDPose[4]*Deg2Rad, ZEDPose[5]*Deg2Rad);
            transform_TargetOnZED.setRotation(q);
            transform_TargetOnBase = transform_ZEDROnBase*transform_TargetOnZED;
            break;
          case GetBrickPoseZEDNew:
            transform_TargetOnZED.setOrigin(tf::Vector3(ZEDPose[0]+0.000, ZEDPose[1] + 0.000, ZEDPose[2] + 0.00));
            q.setRPY(ZEDPose[3]*Deg2Rad, ZEDPose[4]*Deg2Rad, ZEDPose[5]*Deg2Rad);
            transform_TargetOnZED.setRotation(q);
            transform_TargetOnBase = transform_ZEDROnBase*transform_TargetOnZED;
            break;
          case GetBrickLoc:
            transform_TargetOnZED.setOrigin(tf::Vector3(ZEDPose[0], ZEDPose[1], ZEDPose[2]));
            q.setRPY(0, 0, 0);
            transform_TargetOnZED.setRotation(q);
            transform_TargetOnBase = transform_ZEDLOnBase*transform_TargetOnZED;
            ROS_INFO_STREAM("Vision data ZEDPose:"<<ZEDPose[0]<<","<<ZEDPose[1]<<","<<ZEDPose[2]<<","<<ZEDPose[3]<<","<<ZEDPose[4]<<","<<ZEDPose[5]);

            break;
          case GetPutPos:
            break;
          case GetPutAngle:
            break;
          case GetLPose:
            break;
          default:
            break;
        }

        // 返回目标在末端电磁铁坐标系下的位姿
        res.VisionData.header.stamp = ros::Time().now();
        res.VisionData.header.frame_id = "base_link";

        res.VisionData.Flag = true;
        res.VisionData.Pose.position.x = transform_TargetOnBase.getOrigin().x();
        res.VisionData.Pose.position.y = transform_TargetOnBase.getOrigin().y();
        res.VisionData.Pose.position.z = transform_TargetOnBase.getOrigin().z();
        res.VisionData.Pose.orientation.x = transform_TargetOnBase.getRotation().getX();
        res.VisionData.Pose.orientation.y = transform_TargetOnBase.getRotation().getY();
        res.VisionData.Pose.orientation.z = transform_TargetOnBase.getRotation().getZ();
        res.VisionData.Pose.orientation.w = transform_TargetOnBase.getRotation().getW();
        res.VisionData.OrangeIndex = OrangeIndex;
    }
    else    // 如果没有识别结果
    {
        res.VisionData.header.stamp = ros::Time().now();
        res.VisionData.header.frame_id = "base_link";

        res.VisionData.Flag = false;
        res.VisionData.Pose.position.x = 0.0;
        res.VisionData.Pose.position.y = 0.0;
        res.VisionData.Pose.position.z = 0.0;
        res.VisionData.Pose.orientation.x = 0.0;
        res.VisionData.Pose.orientation.y = 0.0;
        res.VisionData.Pose.orientation.z = 0.0;
        res.VisionData.Pose.orientation.w = 0.0;
        res.VisionData.OrangeIndex = 0;
    }
    algorithm = NotRun;
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Task2_Vision_node");

  // Local control variables
  ros::NodeHandle nh; 
  //可能需要根据不同任务来切换相机,接受不同节点消息
  message_filters::Subscriber<sensor_msgs::Image> subleft(nh,"/zed/zed_node/left/image_rect_color",1); 
  message_filters::Subscriber<sensor_msgs::Image> subRight(nh,"/zed/zed_node/right/image_rect_color",1); 

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(subleft, subRight,5);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::Subscriber subMER  = nh.subscribe("/CameraMER/ContinuousImage", 1, callback_MER);  // /CameraMER/SingleImage /CameraMER/HDRImage /CameraMER/ContinuousImage 

  ros::ServiceServer service = nh.advertiseService("GetVisionData",GetVisionData);

  ROS_INFO_STREAM("Ready to process task2 vision data");

  //指定循环的频率 
  ros::Rate loop_rate(20); 
  tf::TransformListener listener;
  
  while(ros::ok()) 
  { 
      // 获取 MER_link, zed_linkL, zed_linkR 在 base_link下的坐标 
      try{
        listener.lookupTransform("base_link", "MER_link", ros::Time(0), transform_MEROnBase);
        listener.lookupTransform("base_link", "zed_linkL", ros::Time(0), transform_ZEDLOnBase);
        listener.lookupTransform("base_link", "zed_linkR", ros::Time(0), transform_ZEDROnBase);
      }
      catch (tf::TransformException ex){
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
