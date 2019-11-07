#include <ros/ros.h> 
#include <std_msgs/String.h> 
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include "HCNetSDK.h"
#include "iniFile.h"

using namespace cv;
image_transport::Publisher pub;

// 取流相关信息，用于线程传递
typedef struct tagREAL_PLAY_INFO
{
	char szIP[16];
	int iUserID;
	int iChannel;
}REAL_PLAY_INFO, *LPREAL_PLAY_INFO;

bool ByteToMat(BYTE* pImg, int nH, int nW, int nFlag, Mat& outImg)//nH,nW为BYTE*类型图像的高和宽,nFlag为通道数
{
    if(pImg == NULL)
    {
        return false;
    }
    int nByte = nH * nW * nFlag / 8;//字节计算
    int nType = nFlag == 8 ? CV_8UC1 : CV_8UC3;
    outImg = Mat::zeros(nH, nW, nType);
    memcpy(outImg.data, pImg, nByte);
    return true;
}

FILE *g_pFile = NULL;
void PsDataCallBack(LONG lRealHandle, DWORD dwDataType,BYTE *pPacketBuffer,DWORD nPacketSize, void* pUser)
{

	if (dwDataType  == NET_DVR_SYSHEAD)
	{	
 /*       //写入头数据
		g_pFile = fopen("./record/ps.dat", "wb");
		
		if (g_pFile == NULL)
		{
			printf("CreateFileHead fail\n");
			return;
		}

		//写入头数据
		fwrite(pPacketBuffer, sizeof(unsigned char), nPacketSize, g_pFile);
	//	printf("write head len=%d\n", nPacketSize);
    */
	}
	else
	{
        // BYTE* --> cv::mat
		ROS_INFO("nPacketSize = %d",nPacketSize);
        cv::Mat image;
        ByteToMat(pPacketBuffer,384,288,8, image);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        pub.publish(msg);

		if(g_pFile != NULL)
		{
	//		fwrite(pPacketBuffer, sizeof(unsigned char), nPacketSize, g_pFile);
	//		printf("write data len=%d\n", nPacketSize);
		}
	}	

}



int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "interface_camera_ir"); 
    //声明节点句柄 
    ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	pub = it.advertise("cameraIR/image", 1);


    NET_DVR_Init();
	// 注册设备
	LONG lUserID;
	//登录参数,包括设备地址、登录用户、密码等
	NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
	struLoginInfo.bUseAsynLogin = 0; //同步登录方式
	strcpy(struLoginInfo.sDeviceAddress, "192.168.50.11"); //设备 IP 地址
	struLoginInfo.wPort = 8000; //设备服务端口
	strcpy(struLoginInfo.sUserName, "admin"); //设备登录用户名
	strcpy(struLoginInfo.sPassword, "space305"); //设备登录密码
	//设备信息, 输出参数
	NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};
	lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);
	if (lUserID < 0)
	{
		ROS_ERROR("Login failed, error code: %d", NET_DVR_GetLastError());
		NET_DVR_Cleanup();
		return 0;
	}

 	NET_DVR_JPEGPARA strPicPara = {0};
    strPicPara.wPicQuality = 2;
    strPicPara.wPicSize = 0;
    int iRet;
	char *sJpegPicBuffer;
	LPDWORD lpSizeReturned;
    //iRet = NET_DVR_CaptureJPEGPicture(lUserID, struDeviceInfoV40.struDeviceV30.byStartChan, &strPicPara, "./ssss.jpeg");


    ros::Rate loop_rate(20);
    while (nh.ok()) 
	{
		
/*		try
		{
			iRet =NET_DVR_CaptureJPEGPicture_NEW(lUserID, struDeviceInfoV40.struDeviceV30.byStartChan, &strPicPara, sJpegPicBuffer, 11059, lpSizeReturned);
		}
		catch (int i)
		{
			DWORD rtn = NET_DVR_GetLastError();
			ROS_INFO("rtn = %d", rtn);
		}
		DWORD rtn = NET_DVR_GetLastError();
		ROS_INFO("rtn = %d", rtn);	
		if (!iRet)
		{
			ROS_INFO("pyd1---NET_DVR_CaptureJPEGPicture error, %d", NET_DVR_GetLastError());
			return 0;
		}
		ROS_INFO("lpSizeReturned = %d",*lpSizeReturned);
		*/
		iRet = NET_DVR_CaptureJPEGPicture(lUserID, struDeviceInfoV40.struDeviceV30.byStartChan, &strPicPara, "/home/ugvcontrol/bit_mbzirc/src/bit_hardware_interface/src/ssss.jpg");
		cv::Mat image =cv::imread("/home/ugvcontrol/bit_mbzirc/src/bit_hardware_interface/src/ssss.jpg");
		
		//ByteToMat((BYTE *)sJpegPicBuffer,384,288,24, image);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

		pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
	NET_DVR_Logout_V30(lUserID);
    NET_DVR_Cleanup();
}