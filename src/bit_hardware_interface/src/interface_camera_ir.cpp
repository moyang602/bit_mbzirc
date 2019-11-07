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
ros::NodeHandle nh;
image_transport::ImageTransport it(nh);
image_transport::Publisher pub = it.advertise("cameraIR/image", 1);

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

        cv::Mat image;
        ByteToMat(pPacketBuffer,384,288,8, image);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", image).toImageMsg();

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
    


    NET_DVR_Init();
    NET_DVR_SetLogToFile(3, "./src/bit_hardware_interface/src/record/");

    IniFile ini("/src/bit_hardware_interface/include/cameraIR/Device.ini");
	unsigned int dwSize = 0;
	char sSection[16] = "DEVICE";

    char *sIP = ini.readstring(sSection, "ip", "error", dwSize);
	int iPort = ini.readinteger(sSection, "port", 0);
	char *sUserName = ini.readstring(sSection, "username", "error", dwSize); 
	char *sPassword = ini.readstring(sSection, "password", "error", dwSize);
	int iChannel = ini.readinteger(sSection, "channel", 0);


    NET_DVR_DEVICEINFO_V30 struDeviceInfo;
	int iUserID = NET_DVR_Login_V30(sIP, iPort, sUserName, sPassword, &struDeviceInfo);
    if(iUserID >= 0)
	{
		NET_DVR_PREVIEWINFO struPreviewInfo = {0};
		struPreviewInfo.lChannel =iChannel;
		struPreviewInfo.dwStreamType = 0;
		struPreviewInfo.dwLinkMode = 0;
		struPreviewInfo.bBlocked = 1;
		struPreviewInfo.bPassbackRecord  = 1;


		int iRealPlayHandle = NET_DVR_RealPlay_V40(iUserID, &struPreviewInfo, PsDataCallBack, NULL);

		if(iRealPlayHandle >= 0)
		{
			ROS_INFO("[GetStream]---RealPlay %s:%d success", sIP, iChannel, NET_DVR_GetLastError());
		}
		else
		{
			ROS_INFO("[GetStream]---RealPlay %s:%d failed, error = %d", sIP, iChannel, NET_DVR_GetLastError());
		}
	}
	else
	{
		ROS_INFO("[GetStream]---Login %s failed, error = %d", sIP, NET_DVR_GetLastError());
	}

//    cv::Mat image = cv::imread("./src/bit_hardware_interface/src/test.png", CV_LOAD_IMAGE_COLOR);
 //   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg();

    ros::Rate loop_rate(5);
    while (nh.ok()) {
     //   pub.publish(msg);
     //   ros::spinOnce();
        loop_rate.sleep();
    }
}