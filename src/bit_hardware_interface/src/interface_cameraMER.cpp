#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include "GxIAPI.h"
#include "DxImageProc.h"
#include <bit_hardware_interface/MER_srv.h>

using namespace cv;

bool subscriber_connected_;  // 订阅者连接状态
ros::Publisher pub;	// 图像发布器
std::string param_MsgName;

// MER相机相关
GX_DEV_HANDLE g_hDevice = NULL;///< 设备句柄
GX_FRAME_DATA g_frameData;     ///< 采集图像参数
void *g_pRaw8Buffer = NULL;                         ///< 将非8位raw数据转换成8位数据的时候的中转缓冲buffer
void *g_pRGBframeData = NULL;                       ///< RAW数据转换成RGB数据后的存储空间，大小是相机输出数据大小的3倍
int64_t g_nPixelFormat = GX_PIXEL_FORMAT_BAYER_GR8; ///< 当前相机的pixelformat格式
int64_t g_nColorFilter = GX_COLOR_FILTER_NONE;      ///< bayer插值的参数
void *g_pframeInfoData = NULL;                      ///< 帧信息数据缓冲区
size_t g_nframeInfoDataSize = 0;                    ///< 帧信息数据长度

void subscribeCallback()
{
  if (pub.getNumSubscribers() > 0)
  {
    if (!subscriber_connected_)
    {
      ROS_INFO_STREAM(param_MsgName<<"  Starting stream");
      subscriber_connected_ = true;
    }
  }
  else
  {
    if (subscriber_connected_)
    {
      ROS_INFO_STREAM(param_MsgName<<"  Stopping stream");
      //camera_.StopGrab();
      subscriber_connected_ = false;
    }
  }
}

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

// service 回调函数，输入参数req，输出参数res
bool GrabImage(bit_hardware_interface::MER_srv::Request  &req,
               bit_hardware_interface::MER_srv::Response &res)
{
    ROS_INFO("The exposure_time is %lf", req.exposure_time);
    //status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, req.exposure_time);




}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "interface_cameraMER");
    //声明节点句柄 
    ros::NodeHandle nh("~");

    ros::SubscriberStatusCallback rsscb = boost::bind(&subscribeCallback);

	nh.param<std::string>("MsgName", param_MsgName, "imageMER");

    pub = nh.advertise<sensor_msgs::Image>(param_MsgName, 1, rsscb, rsscb);

    //-------------  MER相机处理  -----------------//

    GX_STATUS status = GX_STATUS_SUCCESS;

    // 初始化，申请资源
    status = GXInitLib();
    
    //枚举设备个数
	uint32_t nDeviceNum = 0;
	status = GXUpdateDeviceList(&nDeviceNum, 1000);

	if(nDeviceNum <=0)
	{
		printf("当前无可用设备!\n");
		return 0;
	}
	else
	{
		// //默认打开第1个设备
		// status = GXOpenDeviceByIndex(1, &g_hDevice);
		// if(status == GX_STATUS_SUCCESS)
		// {
		// 	printf("打开设备成功!\n");
		// }
		// else
		// {
		// 	printf("打开设备失败!\n");
		// 	return 0;			
		// }

        // 通过序列号打开相机
        GX_OPEN_PARAM openParam;

        openParam.accessMode = GX_ACCESS_EXCLUSIVE;     ///< 独占方式
        openParam.openMode = GX_OPEN_SN;                ///< 通过SN打开
        openParam.pszContent = (char*)"RW0172009017";

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
    
    //----------- 相机参数读取与设置 -------------//
    //设置采集模式为单帧采集
	GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_SINGLE_FRAME);

    //设置触发开关为ON
	GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);

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
	GXSetInt(g_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 3);

    //设置AOI
	int64_t nOffsetX = 0;
	int64_t nOffsetY = 0;
	int64_t nWidth   = 1292;
	int64_t nHeight  = 964;
	status = GXSetInt(g_hDevice, GX_INT_OFFSET_X, nOffsetX);
	status = GXSetInt(g_hDevice, GX_INT_OFFSET_Y, nOffsetY);
	status = GXSetInt(g_hDevice, GX_INT_WIDTH, nWidth);
	status = GXSetInt(g_hDevice, GX_INT_HEIGHT, nHeight);

    //设置曝光时间
	status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, 10000);

    // 设置自动白平衡设置为关闭
    int64_t   nValue = 0;       
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, nValue);

    // 设置红色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, 2.1);
    // 设置绿色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, 1.0);
    // 设置蓝色通道
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    status = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, 2.3);

    //发送开采命令
    status = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_START);

    //获取帧信息长度并申请帧信息数据空间
	GXGetBufferLength(g_hDevice, GX_BUFFER_FRAME_INFORMATION, &g_nframeInfoDataSize);
	g_pframeInfoData = malloc(g_nframeInfoDataSize);


    printf("nPayLoadSize = %ld\n",nPayLoadSize);
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        status = GXSendCommand(g_hDevice, GX_COMMAND_TRIGGER_SOFTWARE);
        if(g_frameData.pImgBuf == NULL)
		{
			continue;
		}

        

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