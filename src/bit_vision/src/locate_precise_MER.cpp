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
#include "bit_vision_msgs/VisionProc.h"
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
# define NotRun             0
int algorithm = GetBrickPos;     // 当前算法
bool data_flag = false;     // 数据置信度

//取砖和放砖用同一个变量表示角度
string brick_color = "G";
HTuple Brick_X(0);
HTuple Brick_Y(0);
HTuple Brick_Z(0);
HTuple Brick_RX(0);
HTuple Brick_RY(0);
HTuple Brick_RZ(0);
HTuple Pile_X(0);
HTuple Pile_Y(0);
HTuple Pile_Z(0);
HTuple L_shelf_angle(0);
tf::StampedTransform transform_MEROnBase;

void get_cam_par_data (HTuple hv_CameraParam, HTuple hv_ParamName, HTuple *hv_ParamValue);
// Chapter: Calibration / Camera Parameters
// Short Description: Get the names of the parameters in a camera parameter tuple. 
void get_cam_par_names (HTuple hv_CameraParam, HTuple *hv_CameraType, HTuple *hv_ParamNames);
// Chapter: Calibration / Camera Parameters
// Short Description: Set the value of a specified camera parameter in the camera parameter tuple. 
void set_cam_par_data (HTuple hv_CameraParamIn, HTuple hv_ParamName, HTuple hv_ParamValue, 
    HTuple *hv_CameraParamOut);
// Local procedures 
void classify_color_regions (HObject ho_Image, HObject *ho_select_region, HTuple hv_MLPHandle, 
    HTuple *hv_class, HTuple *hv_row, HTuple *hv_column);
void select_max_area_region (HObject ho_Region, HObject *ho_ObjectSelected, HTuple *hv_Area, 
    HTuple *hv_Row, HTuple *hv_Column);

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

// Local procedures 
void classify_color_regions (HObject ho_Image, HObject *ho_select_region, HTuple hv_MLPHandle, 
    HTuple *hv_class, HTuple *hv_row, HTuple *hv_column)
{

  // Local iconic variables
  HObject  ho_ClassRegions, ho_ClassRed, ho_ClassGreen;
  HObject  ho_ClassBLue, ho_ConnectedRegions1, ho_ConnectedRegions2;
  HObject  ho_ConnectedRegions3, ho_ObjectSelectedRed, ho_ObjectSelectedGreen;
  HObject  ho_ObjectSelectedBlue;

  // Local control variables
  HTuple  hv_Area_1, hv_Row_1, hv_Column_1, hv_Area_2;
  HTuple  hv_Row_2, hv_Column_2, hv_Area_3, hv_Row_3, hv_Column_3;
  HTuple  hv_areas, hv_rows, hv_columns, hv_Indices, hv_num;
  HTuple  hv_index;

  ClassifyImageClassMlp(ho_Image, &ho_ClassRegions, hv_MLPHandle, 0.9);

  SelectObj(ho_ClassRegions, &ho_ClassRed, 1);
  SelectObj(ho_ClassRegions, &ho_ClassGreen, 2);
  SelectObj(ho_ClassRegions, &ho_ClassBLue, 3);

  Connection(ho_ClassRed, &ho_ConnectedRegions1);
  Connection(ho_ClassGreen, &ho_ConnectedRegions2);
  Connection(ho_ClassBLue, &ho_ConnectedRegions3);

  select_max_area_region(ho_ConnectedRegions1, &ho_ObjectSelectedRed, &hv_Area_1, 
      &hv_Row_1, &hv_Column_1);
  select_max_area_region(ho_ConnectedRegions2, &ho_ObjectSelectedGreen, &hv_Area_2, 
      &hv_Row_2, &hv_Column_2);
  select_max_area_region(ho_ConnectedRegions3, &ho_ObjectSelectedBlue, &hv_Area_3, 
      &hv_Row_3, &hv_Column_3);


  //比较3种region的面积 面积最大的作为分类结果
  hv_areas.Clear();
  hv_areas.Append(hv_Area_1);
  hv_areas.Append(hv_Area_2);
  hv_areas.Append(hv_Area_3);
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
    (*hv_class) = "R";
    (*ho_select_region) = ho_ObjectSelectedRed;
  }
  else if (0 != (hv_index==1))
  {
    (*hv_class) = "G";
    (*ho_select_region) = ho_ObjectSelectedGreen;
  }
  else if (0 != (hv_index==2))
  {
    (*hv_class) = "B";
    (*ho_select_region) = ho_ObjectSelectedBlue;
  }

  (*hv_row) = HTuple(hv_rows[hv_index]);
  (*hv_column) = HTuple(hv_columns[hv_index]);
  return;
  return;
}

void select_max_area_region (HObject ho_Region, HObject *ho_ObjectSelected, HTuple *hv_Area, 
    HTuple *hv_Row, HTuple *hv_Column)
{

  // Local iconic variables

  // Local control variables
  HTuple  hv_Area1, hv_Row1, hv_Column1, hv_Indices;
  HTuple  hv_num;

  AreaCenter(ho_Region, &hv_Area1, &hv_Row1, &hv_Column1);
  TupleSortIndex(hv_Area1, &hv_Indices);
  hv_num = hv_Indices.TupleLength();
  SelectObj(ho_Region, &(*ho_ObjectSelected), HTuple(hv_Indices[hv_num-1])+1);
  AreaCenter((*ho_ObjectSelected), &(*hv_Area), &(*hv_Row), &(*hv_Column));
  return;
}


// 1.定位矩形标志(图像来源: MER)
void rectangle_pose(HObject ho_ImageL,HTuple &hv_X, HTuple &hv_Y, HTuple &hv_Z,HTuple &hv_RX, HTuple &hv_RY, HTuple &hv_RZ)
{

    // Local iconic variables
  HObject  ho_GrayImage, ho_Edges, ho_EdgesRectifiedAdaptive;
  HObject  ho_ROI, ho_ImageRectifiedAdaptive, ho_Regions, ho_ConnectedRegions3;
  HObject  ho_SelectedRegions4, ho_ObjectSelectedL, ho_RegionClosing;
  HObject  ho_ContourL, ho_ContoursSplit, ho_SelectedEdges;
  HObject  ho_UnionContours1, ho_UnionContours2, ho_UnionContours;

  // Local control variables
  HTuple  hv_RectWidth, hv_RectHeight, hv_Pointer;
  HTuple  hv_Type, hv_Width, hv_Height, hv_CamParOriginal;
  HTuple  hv_CamParVirtualFixed, hv_CamParVirtualAdaptive;
  HTuple  hv_WidthVirtualAdaptive, hv_HeightVirtualAdaptive;
  HTuple  hv_REC, hv_index, hv_indexes_REC, hv_PoseL, hv_CovPoseL;
  HTuple  hv_ErrorL,hv_Exception, hv_NumberContourSplit;
  HTuple  hv_Seconds_3, hv_Seconds_4,hv_NumberRegion;

  try{

      //step1:根据颜色提取指定颜色的砖块区域
      hv_RectWidth = 0.3;
      hv_RectHeight = 0.17;

      GetImagePointer1(ho_ImageL, &hv_Pointer, &hv_Type, &hv_Width, &hv_Height);
      Rgb1ToGray(ho_ImageL, &ho_GrayImage);
      ReadCamPar("/home/ugvcontrol/bit_mbzirc/src/bit_vision/model/MER_calib.dat", &hv_CamParOriginal);
      EdgesSubPix(ho_GrayImage, &ho_Edges, "lanser2", 0.5, 20, 40);
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
      //step2: 提取ROI区域用于轮廓提取 此处用特定颜色的分割以及矩形拟合来提取区域
      Threshold(ho_ImageRectifiedAdaptive, &ho_Regions, 65, 91);
      Connection(ho_Regions, &ho_ConnectedRegions3);
      SelectShape(ho_ConnectedRegions3, &ho_SelectedRegions4, (HTuple("rectangularity").Append("area")), 
          "and", (HTuple(0.85).Append(10000)), (HTuple(1).Append(100000000)));
      CountObj(ho_SelectedRegions4, &hv_NumberRegion);
      if (0 != (hv_NumberRegion!=0))
      {
        try
        {
          //对区域根据retangularity进行排序
          RegionFeatures(ho_SelectedRegions4, "rectangularity", &hv_REC);
          TupleSortIndex(hv_REC, &hv_index);
          TupleInverse(hv_index, &hv_indexes_REC);
          hv_index = HTuple(hv_indexes_REC[0])+1;
          ROS_INFO("hv_index = %lf",hv_index.D());
          SelectObj(ho_SelectedRegions4, &ho_ObjectSelectedL, hv_index);
          //
          ClosingCircle(ho_ObjectSelectedL, &ho_RegionClosing, 10);
          GenContourRegionXld(ho_RegionClosing, &ho_ContourL, "border");

          SegmentContoursXld(ho_ContourL, &ho_ContoursSplit, "lines", 7, 5, 3);
          SelectContoursXld(ho_ContoursSplit, &ho_SelectedEdges, "contour_length", 100, 
              2000, -10, 10);
          UnionAdjacentContoursXld(ho_SelectedEdges, &ho_UnionContours1, 10, 1, "attr_keep");
          GetRectanglePose(ho_UnionContours1, hv_CamParVirtualAdaptive, 0.3, 0.17, "huber", 
              1, &hv_PoseL, &hv_CovPoseL, &hv_ErrorL);
          hv_X = ((const HTuple&)hv_PoseL)[0];
          hv_Y = ((const HTuple&)hv_PoseL)[1];
          hv_Z = ((const HTuple&)hv_PoseL)[2];
          ROS_INFO("Brick X = %lf",hv_X.D());
          ROS_INFO("Brick Y = %lf",hv_Y.D());
          ROS_INFO("Brick Z = %lf",hv_Z.D());
          data_flag = true;
        }

      catch (HException &HDevExpDefaultException)
      {
        HDevExpDefaultException.ToHTuple(&hv_Exception);
        try
        {
          //对区域根据retangularity进行排序
          RegionFeatures(ho_SelectedRegions4, "rectangularity", &hv_REC);
          TupleSortIndex(hv_REC, &hv_index);
          TupleInverse(hv_index, &hv_indexes_REC);
          hv_index = HTuple(hv_indexes_REC[0])+1;
          SelectObj(ho_SelectedRegions4, &ho_ObjectSelectedL, hv_index);
          //
          ClosingCircle(ho_ObjectSelectedL, &ho_RegionClosing, 10);
          GenContourRegionXld(ho_RegionClosing, &ho_ContourL, "border");
          //
          SegmentContoursXld(ho_ContourL, &ho_ContoursSplit, "lines", 7, 5, 3);
          SelectContoursXld(ho_ContoursSplit, &ho_SelectedEdges, "contour_length", 100, 
              2000, -0.5, 0.5);
          CountObj(ho_SelectedEdges, &hv_NumberContourSplit);
          //将共线的轮廓连接起来
          CountSeconds(&hv_Seconds_3);
          UnionCollinearContoursExtXld(ho_SelectedEdges, &ho_UnionContours2, 500, 10, 
              50, 0.7, 0, -1, 1, 1, 1, 1, 1, 0, "attr_keep");
          UnionAdjacentContoursXld(ho_UnionContours2, &ho_UnionContours, 10, 1, "attr_keep");
          GetRectanglePose(ho_UnionContours, hv_CamParVirtualAdaptive, 0.3, 0.17, "huber", 
              1, &hv_PoseL, &hv_CovPoseL, &hv_ErrorL);
          CountSeconds(&hv_Seconds_4);
          hv_X = ((const HTuple&)hv_PoseL)[0];
          hv_Y = ((const HTuple&)hv_PoseL)[1];
          hv_Z = ((const HTuple&)hv_PoseL)[2];
          ROS_INFO("Brick X = %lf",hv_X.D());
          ROS_INFO("Brick Y = %lf",hv_Y.D());
          ROS_INFO("Brick Z = %lf",hv_Z.D());
          data_flag = true;
        }

        catch (exception e)
        {
          data_flag = false;
        }

      } 
    }
  }
  catch (HException &exception)
  {
      ROS_ERROR("11  Error #%u in %s: %s\n", exception.ErrorCode(),
              (const char *)exception.ProcName(),
              (const char *)exception.ErrorMessage());
      data_flag = false;
  }

}


//初始化halcon对象
HObject ho_ImageMER;

//回调函数
void callback(const sensor_msgs::Image::ConstPtr& MERImage) 
{
    //获取MER图像
    halcon_bridge::HalconImagePtr halcon_bridge_imagePointerMER = halcon_bridge::toHalconCopy(MERImage);
    ho_ImageMER = *halcon_bridge_imagePointerMER->image;
    
}

bool GetVisionData(bit_vision_msgs::VisionProc::Request&  req,
                   bit_vision_msgs::VisionProc::Response& res)
{
    ROS_INFO("BrickType:[%s], VisionAlgorithm:[%d]",req.BrickType.c_str(),req.ProcAlgorithm);
    // 设置视觉处理颜色与算法
    brick_color = req.BrickType;
    algorithm = req.ProcAlgorithm;
    data_flag = false;

    switch (algorithm)
    {
        case GetBrickPos:
            //取砖块的位姿 输入为MER图像
            rectangle_pose(ho_ImageMER,Brick_X,Brick_Y,Brick_Z,Brick_RX,Brick_RY,Brick_RZ);
            break;
        default:
            break;
    }

    if (data_flag)
    {
        tf::Transform transform_TargetOnMER;
        transform_TargetOnMER.setOrigin(tf::Vector3(Brick_X.D(), Brick_Y.D(), Brick_Z.D()));
        tf::Quaternion q;
        q.setRPY(Brick_RX.D(), Brick_RY.D(), Brick_RZ.D());
        transform_TargetOnMER.setRotation(q);
 
        tf::Transform transform_TargetOnBase = transform_MEROnBase*transform_TargetOnMER;

        ROS_INFO_STREAM("Vision data:"<<Brick_X.D()<<","<<Brick_Y.D()<<","<<Brick_Z.D());
        
        // 返回目标在末端电磁铁坐标系下的位姿
        res.VisionData.header.stamp = ros::Time().now();
        res.VisionData.header.frame_id = "base_link";

        res.VisionData.Flag = true;
        res.VisionData.Pose.position.x = transform_TargetOnBase.getOrigin().x();
        res.VisionData.Pose.position.y = transform_TargetOnBase.getOrigin().y();
        res.VisionData.Pose.position.z = transform_TargetOnBase.getOrigin().z();
        res.VisionData.Pose.orientation.x = 0.0;
        res.VisionData.Pose.orientation.y = 0.0;
        res.VisionData.Pose.orientation.z = transform_TargetOnBase.getRotation().getZ();
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
    }

    algorithm = NotRun;
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Task2_Vision_node");

  ros::NodeHandle nh; 

  ros::Subscriber subLeft  = nh.subscribe("Image_hdr", 1, callback);

  ros::ServiceServer service = nh.advertiseService("GetPreciseVisionData",GetVisionData);

  ROS_INFO_STREAM("Ready to process vision data");

  //指定循环的频率 
  ros::Rate loop_rate(20); 
  tf::TransformListener listener;
  
  while(ros::ok()) 
  { 
      // 获取 MER_link 在 base_link下的坐标 
      try{
        listener.lookupTransform("base_link", "MER_link", ros::Time(0), transform_MEROnBase);
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