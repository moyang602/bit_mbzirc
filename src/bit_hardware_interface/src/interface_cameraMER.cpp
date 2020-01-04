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

#include "halcon_image.h"
#undef Status  

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>

#include "GxIAPI.h"
#include "DxImageProc.h"
#include <bit_hardware_msgs/MER_srv.h>
#include <bit_hardware_msgs/MER_hdr.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace HalconCpp;

// MER相机相关
GX_DEV_HANDLE g_hDevice = NULL;						///< 设备句柄
GX_FRAME_DATA g_frameData;     						///< 采集图像参数
void *g_pRaw8Buffer = NULL;                         ///< 将非8位raw数据转换成8位数据的时候的中转缓冲buffer
void *g_pRGBframeData = NULL;                       ///< RAW数据转换成RGB数据后的存储空间，大小是相机输出数据大小的3倍
int64_t g_nPixelFormat = GX_PIXEL_FORMAT_BAYER_RG8; ///< 当前相机的pixelformat格式
int64_t g_nColorFilter = GX_COLOR_FILTER_BAYER_RG;      ///< bayer插值的参数
void *g_pframeInfoData = NULL;                      ///< 帧信息数据缓冲区
size_t g_nframeInfoDataSize = 0;                    ///< 帧信息数据长度

//----------------------------------------------------------------------------------
/**
\brief  将相机输出的原始数据转换为RGB数据
\param  [in] pImageBuf  指向图像缓冲区的指针
\param  [in] pImageRaw8Buf  将非8位的Raw数据转换成8位的Raw数据的中转缓冲buffer
\param  [in,out] pImageRGBBuf  指向RGB数据缓冲区的指针
\param  [in] nImageWidth 图像宽
\param  [in] nImageHeight 图像高
\param  [in] nPixelFormat 图像的格式
\param  [in] nPixelColorFilter Raw数据的像素排列格式
\return 无返回值
*/
//----------------------------------------------------------------------------------
void ProcessData(void * pImageBuf, void * pImageRaw8Buf, void * pImageRGBBuf, int nImageWidth, int nImageHeight, int nPixelFormat, int nPixelColorFilter)
{
	switch(nPixelFormat)
	{
		//当数据格式为12位时，位数转换为4-11
	case GX_PIXEL_FORMAT_BAYER_GR12:
	case GX_PIXEL_FORMAT_BAYER_RG12:
	case GX_PIXEL_FORMAT_BAYER_GB12:
	case GX_PIXEL_FORMAT_BAYER_BG12:
		//将12位格式的图像转换为8位格式
		DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);	
		//将Raw8图像转换为RGB图像以供显示
		DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight,RAW2RGB_NEIGHBOUR,
			DX_PIXEL_COLOR_FILTER(nPixelColorFilter),false);		        
		break;

		//当数据格式为10位时，位数转换为2-9
	case GX_PIXEL_FORMAT_BAYER_GR10:
	case GX_PIXEL_FORMAT_BAYER_RG10:
	case GX_PIXEL_FORMAT_BAYER_GB10:
	case GX_PIXEL_FORMAT_BAYER_BG10:
		////将10位格式的图像转换为8位格式,有效位数2-9
		DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_2_9);
		//将Raw8图像转换为RGB图像以供显示
		DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight,RAW2RGB_NEIGHBOUR,
			DX_PIXEL_COLOR_FILTER(nPixelColorFilter),false);	
		break;

	case GX_PIXEL_FORMAT_BAYER_GR8:
	case GX_PIXEL_FORMAT_BAYER_RG8:
	case GX_PIXEL_FORMAT_BAYER_GB8:
	case GX_PIXEL_FORMAT_BAYER_BG8:
		//将Raw8图像转换为RGB图像以供显示
		DxRaw8toRGB24(pImageBuf,pImageRGBBuf, nImageWidth, nImageHeight,RAW2RGB_NEIGHBOUR,
			DX_PIXEL_COLOR_FILTER(nPixelColorFilter),false);//RAW2RGB_ADAPTIVE	
		break;

	case GX_PIXEL_FORMAT_MONO12:
		//将12位格式的图像转换为8位格式
		DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);	
		//将Raw8图像转换为RGB图像以供显示
		DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
			DX_PIXEL_COLOR_FILTER(NONE),false);		        
		break;

	case GX_PIXEL_FORMAT_MONO10:
		//将10位格式的图像转换为8位格式
		DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);	
		//将Raw8图像转换为RGB图像以供显示
		DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight,RAW2RGB_NEIGHBOUR,
			DX_PIXEL_COLOR_FILTER(NONE),false);		        
		break;

	case GX_PIXEL_FORMAT_MONO8:
		//将Raw8图像转换为RGB图像以供显示
		DxRaw8toRGB24(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight,RAW2RGB_NEIGHBOUR,
			DX_PIXEL_COLOR_FILTER(NONE),false);		        
		break;

	default:
		break;
	}
}

HObject Mat2HObject(const cv::Mat &image)
{
	HObject Hobj = HObject();
	int hgt = image.rows;
	int wid = image.cols;
	int i;
	//  CV_8UC3  
	if (image.type() == CV_8UC3)
	{
		vector<cv::Mat> imgchannel;
		split(image, imgchannel);
		cv::Mat imgB = imgchannel[0];
		cv::Mat imgG = imgchannel[1];
		cv::Mat imgR = imgchannel[2];
		uchar* dataR = new uchar[hgt*wid];
		uchar* dataG = new uchar[hgt*wid];
		uchar* dataB = new uchar[hgt*wid];
		for (i = 0; i<hgt; i++)
		{
			memcpy(dataR + wid*i, imgR.data + imgR.step*i, wid);
			memcpy(dataG + wid*i, imgG.data + imgG.step*i, wid);
			memcpy(dataB + wid*i, imgB.data + imgB.step*i, wid);
		}
		GenImage3(&Hobj, "byte", wid, hgt, (Hlong)dataR, (Hlong)dataG, (Hlong)dataB);
		delete[]dataR;
		delete[]dataG;
		delete[]dataB;
		dataR = NULL;
		dataG = NULL;
		dataB = NULL;
	}
	//  CV_8UCU1  
	else if (image.type() == CV_8UC1)
	{
		uchar* data = new uchar[hgt*wid];
		for (i = 0; i<hgt; i++)
			memcpy(data + wid*i, image.data + image.step*i, wid);
		GenImage1(&Hobj, "byte", wid, hgt, (Hlong)data);
		delete[] data;
		data = NULL;
	}
	return Hobj;
}

cv::Mat HObject2Mat(HObject Hobj)
{
	HTuple htCh;
	HString cType;
	cv::Mat Image;
	ConvertImageType(Hobj, &Hobj, "byte");
	CountChannels(Hobj, &htCh);
	Hlong wid = 0;
	Hlong hgt = 0;
	if (htCh[0].I() == 1)
	{
		HImage hImg(Hobj);
		void *ptr = hImg.GetImagePointer1(&cType, &wid, &hgt);//GetImagePointer1(Hobj, &ptr, &cType, &wid, &hgt);
		int W = wid;
		int H = hgt;
		Image.create(H, W, CV_8UC1);
		unsigned char *pdata = static_cast<unsigned char *>(ptr);
		memcpy(Image.data, pdata, W*H);
	}
	else if (htCh[0].I() == 3)
	{
		void *Rptr;
		void *Gptr;
		void *Bptr;
		HImage hImg(Hobj);
		hImg.GetImagePointer3(&Rptr, &Gptr, &Bptr, &cType, &wid, &hgt);
		int W = wid;
		int H = hgt;
		Image.create(H, W, CV_8UC3);
		vector<cv::Mat> VecM(3);
		VecM[0].create(H, W, CV_8UC1);
		VecM[1].create(H, W, CV_8UC1);
		VecM[2].create(H, W, CV_8UC1);
		unsigned char *R = (unsigned char *)Rptr;
		unsigned char *G = (unsigned char *)Gptr;
		unsigned char *B = (unsigned char *)Bptr;
		memcpy(VecM[2].data, R, W*H);
		memcpy(VecM[1].data, G, W*H);
		memcpy(VecM[0].data, B, W*H);
		cv::merge(VecM, Image);
	}
	return Image;
}


// service 回调函数，输入参数req，输出参数res
bool GrabImage(bit_hardware_msgs::MER_srv::Request  &req,
               bit_hardware_msgs::MER_srv::Response &res)
{
    GX_STATUS status = GX_STATUS_SUCCESS;

    ROS_INFO("The exposure_time is %lf", req.exposure_time);
    status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, req.exposure_time);
	// 设置红色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, req.BalanceRatioRed);
    // 设置绿色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, req.BalanceRatioGreen);
    // 设置蓝色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, req.BalanceRatioBlue);

    // status = GXSendCommand(g_hDevice, GX_COMMAND_TRIGGER_SOFTWARE);

    status = GXGetImage(g_hDevice, &g_frameData, 100);

    if(status == 0)
    {
        if(g_frameData.nStatus == 0)
        {
            //将Raw数据处理成RGB数据
            ProcessData(g_frameData.pImgBuf, 
                        g_pRaw8Buffer, 
                        g_pRGBframeData, 
                        g_frameData.nWidth, 
                        g_frameData.nHeight,
                        g_nPixelFormat,
                        g_nColorFilter);
            cv::Mat MatImg(g_frameData.nHeight,g_frameData.nWidth,CV_8UC3, (unsigned char*)g_pRGBframeData);

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", MatImg).toImageMsg();

            res.success_flag = true;
            res.MER_image = *msg;
            return true;
        }
        else
        {
            ROS_INFO("Grab error, code: %d\n", g_frameData.nStatus);
            res.success_flag = false;
            return false;
        }
    }
    else
    {
        res.success_flag = false;
        return false;
    }

}

// service 回调函数，输入参数req，输出参数res
bool GrabImage_hdr(bit_hardware_msgs::MER_hdr::Request  &req,
               	   bit_hardware_msgs::MER_hdr::Response &res)
{
    GX_STATUS status = GX_STATUS_SUCCESS;
  
	// 设置红色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, 2.0);
    // 设置绿色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, 1.6);
    // 设置蓝色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, 2.5);

	std::vector<float> exposure_time{1000,20000,10000};
	std::vector<HObject> PrcImage;
	int count = 0;
	for(int i=0;i<3;i++)
	{
		
		count = 0;
		bool flag = false;
		do
		{
			status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_time[i]);
			status = GXGetImage(g_hDevice, &g_frameData, 100);
			count++;
		
			if(status == 0)
			{
				if(g_frameData.nStatus == 0)
				{
					//printf("采集成功: 宽：%d 高：%d\n", g_frameData.nWidth, g_frameData.nHeight);

					//将Raw数据处理成RGB数据
					ProcessData(g_frameData.pImgBuf, 
								g_pRaw8Buffer, 
								g_pRGBframeData, 
								g_frameData.nWidth, 
								g_frameData.nHeight,
								g_nPixelFormat,
								g_nColorFilter);
					cv::Mat MatImg(g_frameData.nHeight,g_frameData.nWidth,CV_8UC3, (unsigned char*)g_pRGBframeData);

					PrcImage.push_back(Mat2HObject(MatImg));
					flag = true;
				}
				else
				{
					ROS_INFO("Grab error, code: %d\n", g_frameData.nStatus);
					res.success_flag = false;
				}
			}
		}
		while(!flag||count>10);

	}
	HObject ho_ImageResult1, ho_ImageResult2;
	AddImage(PrcImage[0], PrcImage[1], &ho_ImageResult1, 0.5, 0);
	AddImage(ho_ImageResult1, PrcImage[2], &ho_ImageResult2, 0.5, 0);

	cv::Mat halconImage = HObject2Mat(ho_ImageResult2);

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", halconImage).toImageMsg();

	res.success_flag = true;
	res.MER_image = *msg;
	return true;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "interface_cameraMER");
    //声明节点句柄 
    ros::NodeHandle nh("~");

    std::string param_SN_;
    int param_ExposureTime;
	int param_SpeedLevel;
	double param_BalanceRatioRed;
	double param_BalanceRatioGreen;
	double param_BalanceRatioBlue;
	std::string param_CameraMode_;

    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<std::string>("SN", param_SN_, "RW0171009017");
    private_node_handle.param<int>("ExposureTime", param_ExposureTime, 10000);
	private_node_handle.param<int>("SpeedLevel", param_SpeedLevel, 3);
	private_node_handle.param<double>("BalanceRatioRed", param_BalanceRatioRed, 2.0);
	private_node_handle.param<double>("BalanceRatioGreen", param_BalanceRatioGreen, 1.6);
	private_node_handle.param<double>("BalanceRatioBlue", param_BalanceRatioBlue, 2.5);
	private_node_handle.param<std::string>("CameraMode", param_CameraMode_, "SINGLE");


    //-------------  MER相机处理  -----------------//
    GX_STATUS status = GX_STATUS_SUCCESS;

    // 初始化，申请资源
    status = GXInitLib();
    
    //枚举设备个数
	uint32_t nDeviceNum = 0;
	status = GXUpdateDeviceList(&nDeviceNum, 1000);

	if(nDeviceNum <=0)
	{
		ROS_ERROR("There is no MER camera!");
		return 0;
	}
	else
	{
        // 通过序列号打开相机
        GX_OPEN_PARAM openParam;

        openParam.accessMode = GX_ACCESS_EXCLUSIVE;     ///< 独占方式
        openParam.openMode = GX_OPEN_SN;                ///< 通过SN打开
        openParam.pszContent = (char*)param_SN_.c_str();

        status = GXOpenDevice(&openParam, &g_hDevice);
        if(status == GX_STATUS_SUCCESS)
		{
			ROS_INFO("Open MER %s success!",openParam.pszContent);
		}
		else
		{
			ROS_ERROR("Open MER %s failed!",openParam.pszContent);
			return 0;			
		}
	}

    //----------- 开采前准备 -------------//  
    int64_t nPayLoadSize = 0;
	status = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
	g_frameData.pImgBuf = malloc(nPayLoadSize);

    //将非8位raw数据转换成8位数据的时候的中转缓冲buffer
	g_pRaw8Buffer = malloc(nPayLoadSize);
    //RGB数据是RAW数据的3倍大小
	g_pRGBframeData = malloc(nPayLoadSize * 3);
    //获取相机输出数据的颜色格式
	status = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_FORMAT, &g_nPixelFormat);
    //获取相机Bayer插值参数
    status = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_nColorFilter);
    //设置采集速度级别  级别越大，帧率越大
	GXSetInt(g_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, param_SpeedLevel);

	//获取帧信息长度并申请帧信息数据空间
	GXGetBufferLength(g_hDevice, GX_BUFFER_FRAME_INFORMATION, &g_nframeInfoDataSize);
	g_pframeInfoData = malloc(g_nframeInfoDataSize);

    //设置AOI
	int64_t nOffsetX = 0;
	int64_t nOffsetY = 0;
	int64_t nWidth   = 1292;
	int64_t nHeight  = 964;
	status = GXSetInt(g_hDevice, GX_INT_OFFSET_X, nOffsetX);
	status = GXSetInt(g_hDevice, GX_INT_OFFSET_Y, nOffsetY);
	status = GXSetInt(g_hDevice, GX_INT_WIDTH, nWidth);
	status = GXSetInt(g_hDevice, GX_INT_HEIGHT, nHeight);

	//----------- 设置相机参数 -------------//  
	// 设置自动白平衡设置为关闭
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);

	// 设置红色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, param_BalanceRatioRed);
    // 设置绿色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, param_BalanceRatioGreen);
    // 设置蓝色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, param_BalanceRatioBlue);

	if(param_CameraMode_ == "SINGLE")
	{
		//设置触发开关为ON
		GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
		//设置采集模式为单帧模式
		GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_SINGLE_FRAME);
		// 设置自动曝光设置为关闭
    	status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
		//设置曝光时间
		status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, param_ExposureTime);
	}
	else if(param_CameraMode_ == "CONTINUOUS")
	{
		//设置触发开关为OFF
    	status = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
		//设置采集模式为连续模式
		GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
		// 设置自动曝光设置为连续自动曝光
    	status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
	}

      
	//！！！！！！！！！！  开启相机服务  ！！！！！！！！！//

	// 开启单帧采图服务
	ros::ServiceServer service_once = nh.advertiseService("GrabMERImage",GrabImage);
	ROS_INFO_STREAM("Server GrabMERImage start!");
	// 开启HDR采图服务
	ros::ServiceServer service_hdr = nh.advertiseService("GrabHDRImage",GrabImage_hdr);
	ROS_INFO_STREAM("Server GrabHDRImage start!");

	// 设置图像消息发布
	ros::Publisher MER_pub = nh.advertise<sensor_msgs::Image>("MER_Continuous_image", 1);
	
	//发送开采命令
    status = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_START);

    ros::Rate loop_rate(30); 
    while(ros::ok()) 
    { 
		if (param_CameraMode_ == "CONTINUOUS")
		{
			status = GXGetImage(g_hDevice, &g_frameData, 100);
			if(status == 0)	// 如果采集到数据，进行处理
			{
				if(g_frameData.nStatus == 0)
				{
					//将Raw数据处理成RGB数据
					ProcessData(g_frameData.pImgBuf, 
								g_pRaw8Buffer, 
								g_pRGBframeData, 
								g_frameData.nWidth, 
								g_frameData.nHeight,
								g_nPixelFormat,
								g_nColorFilter);
					cv::Mat MatImg(g_frameData.nHeight,g_frameData.nWidth,CV_8UC3, (unsigned char*)g_pRGBframeData);

					sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", MatImg).toImageMsg();
					MER_pub.publish(msg);
				}
				else
				{
					ROS_INFO("Grab error, code: %d\n", g_frameData.nStatus);
				}
				
			}
			
		}
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
	
    //发送停采命令
	status = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_STOP);

    //释放buffer
	free(g_frameData.pImgBuf);
	g_frameData.pImgBuf = NULL;
	free(g_pRaw8Buffer);
	g_pRaw8Buffer = NULL;
	free(g_pRGBframeData);
	g_pRGBframeData = NULL;
	free(g_pframeInfoData);
	g_pframeInfoData = NULL;

    //关闭设备
	status = GXCloseDevice(g_hDevice);

    // 结束调用，释放资源
    status = GXCloseLib();

    return 0;
}