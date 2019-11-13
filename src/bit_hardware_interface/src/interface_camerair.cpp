#include <ros/ros.h> 
#include <std_msgs/String.h> 
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include "HCNetSDK.h"

using namespace cv;

bool subscriber_connected_;  // 订阅者连接状态
ros::Publisher pub;	// 图像发布器
std::string param_MsgName;
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

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "interface_camerair"); 
    //声明节点句柄 
    ros::NodeHandle nh("~");
//	image_transport::ImageTransport it(nh);

	ros::SubscriberStatusCallback rsscb = boost::bind(&subscribeCallback);


	std::string param_DeviceAddress;
	int param_Port;
	std::string param_UserName;
	std::string param_Password;
	std::string param_PictureName;
	
	nh.param<std::string>("DeviceAddress", param_DeviceAddress, "192.168.50.11");
	nh.param<int>("Port", param_Port, 8000);
	nh.param<std::string>("UserName", param_UserName, "admin");
	nh.param<std::string>("Password", param_Password, "space305");
	nh.param<std::string>("PictureName", param_PictureName, "./src/bit_hardware_interface/src/camerair_car.jpg");
	nh.param<std::string>("MsgName", param_MsgName, "imagecar");

//	pub = it.advertise(param_MsgName, 1, rsscb,rsscb);
	pub = nh.advertise<sensor_msgs::Image>(param_MsgName, 1, rsscb, rsscb);

    NET_DVR_Init();
	// 注册设备
	LONG lUserID;
	//登录参数,包括设备地址、登录用户、密码等
	NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
	struLoginInfo.bUseAsynLogin = 0; //同步登录方式
	
	strcpy(struLoginInfo.sDeviceAddress, param_DeviceAddress.c_str()); //设备 IP 地址
	struLoginInfo.wPort = param_Port; //设备服务端口
	strcpy(struLoginInfo.sUserName, param_UserName.c_str()); //设备登录用户名
	strcpy(struLoginInfo.sPassword, param_Password.c_str()); //设备登录密码
	/*
	strcpy(struLoginInfo.sDeviceAddress, "192.168.50.11"); //设备 IP 地址
	struLoginInfo.wPort = 8000; //设备服务端口
	strcpy(struLoginInfo.sUserName, "admin"); //设备登录用户名
	strcpy(struLoginInfo.sPassword, "space305"); //设备登录密码	*/
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

    ros::Rate loop_rate(20);
    while (nh.ok()) 
	{
		if (subscriber_connected_)
		{
			char* storeImgPath = const_cast<char*>(param_PictureName.c_str());
			iRet = NET_DVR_CaptureJPEGPicture(lUserID, struDeviceInfoV40.struDeviceV30.byStartChan, &strPicPara, storeImgPath);
			cv::Mat image =cv::imread(storeImgPath);
			
			//ByteToMat((BYTE *)sJpegPicBuffer,384,288,24, image);
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

			pub.publish(msg);
		}

        ros::spinOnce();
        loop_rate.sleep();
    }
	NET_DVR_Logout_V30(lUserID);
    NET_DVR_Cleanup();
}