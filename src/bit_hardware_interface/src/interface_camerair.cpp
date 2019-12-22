#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include "HCNetSDK.h"
#include "XmlBase.h"
using namespace cv;

bool subscriber_connected_;  // 订阅者连接状态
ros::Publisher pub;	// 图像发布器
std::string param_MsgName;

void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
    char tempbuf[256] = {0};
    switch(dwType) 
    {
    case EXCEPTION_RECONNECT: //预览时重连
        ROS_INFO("----------reconnect--------%ld", time(NULL));
        break;
    default:
        break;
    }
}


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

union byte2float
{
    unsigned char bytebuf[4];
    float floatbuf;
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "interface_camerair");
    //声明节点句柄 
    ros::NodeHandle nh("~");

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
	nh.param<std::string>("MsgName", param_MsgName, "image");

    pub = nh.advertise<sensor_msgs::Image>(param_MsgName, 1, rsscb, rsscb);

    // 初始化SDK
    if(!NET_DVR_Init())
    {
        DWORD rtn = NET_DVR_GetLastError();
        if (rtn == NET_DVR_ALLOC_RESOURCE_ERROR)
        {
            ROS_INFO("SDK resource allocate error, %d", rtn);
        }
        else if (rtn == NET_DVR_GETLOCALIPANDMACFAIL)
        {
            ROS_INFO("Client IP get error, %d",rtn);            
        }
        return rtn;   
    }
    ROS_INFO("SDK init success!");
    // 设置网络连接超时时间和连接尝试次数  uint: ms [300,75000]
    NET_DVR_SetConnectTime(2000, 1);
    // 设置重连时间
    NET_DVR_SetReconnect(10000, true);
 
    // 设置接受异常消息的回调函数
    NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);


    /********************************************/
    // 注册设备
    long lUserID;

    //登录参数，包括设备地址、登录用户、密码等

    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};        // 登录参数
    struLoginInfo.bUseAsynLogin = 0;                    // 同步登录方式
    strcpy(struLoginInfo.sDeviceAddress, param_DeviceAddress.c_str()); //设备 IP 地址
    struLoginInfo.wPort = 8000;                         // 设备端口号
    strcpy(struLoginInfo.sUserName, "admin");           // 登录用户名
    strcpy(struLoginInfo.sPassword, "space305");           // 登录密码

    //设备信息, 输出参数
    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};     // 设备信息（同步登录时有效）

    // 用户注册设备 （支持异步登录）
    lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);
    if (lUserID < 0)
    {
        ROS_INFO("Login failed, error code: %d\n", NET_DVR_GetLastError());
        NET_DVR_Cleanup();
        return -1;
    }
    ROS_INFO("IR camera login success!");
    /********************************************/
   
    // 配置热成像原始数据为“全屏测温数据”
    char StatusBuffer[10240] = { 0 };
	char OutBuffer[10240] = { 0 };
	char InBuffer[10240] = { 0 };
	CXmlBase xmlBase;
	int dwRet;
	
    // NET_DVR_STDXMLConfig参数
    NET_DVR_XML_CONFIG_INPUT lpInputParam = {0};

    lpInputParam.dwSize = sizeof(lpInputParam);

    xmlBase.CreateRoot("ThermalStreamParam");
	xmlBase.SetAttribute("version", "2.0");
	xmlBase.AddNode("videoCodingType", "pixel-to-pixel_thermometry_data");
	xmlBase.OutOfElem();
	xmlBase.WriteToBuf(InBuffer, sizeof(InBuffer), dwRet);

    lpInputParam.lpRequestUrl = (void*)"PUT /ISAPI/Thermal/channels/1/streamParam";   //请求信令，字符串格式
    lpInputParam.dwRequestUrlLen = sizeof("PUT /ISAPI/Thermal/channels/1/streamParam"); //请求信令长度，字符串长度
    lpInputParam.dwRecvTimeOut = 3000;                                  //接收超时时间，单位：ms，填0则使用默认超时5s
    lpInputParam.byForceEncrpt = 0;                                     //是否强制加密，0-否，1-是
    lpInputParam.byNumOfMultiPart = 0;                                  //0-无效，其他值表示报文分段个数，非零时lpInBuffer传入的是NET_DVR_MIME_UNIT结构体数组的指针，该值即代表结构体个数
    lpInputParam.lpInBuffer = InBuffer;                                 //输入参数缓冲区，XML格式
    lpInputParam.dwInBufferSize = sizeof(InBuffer);                     //输入参数缓冲区大小

    NET_DVR_XML_CONFIG_OUTPUT lpOutputParam = {0};
    lpOutputParam.dwSize = sizeof(lpOutputParam);

    lpOutputParam.lpOutBuffer = OutBuffer;
	lpOutputParam.dwOutBufferSize = sizeof(OutBuffer);
	lpOutputParam.lpStatusBuffer = StatusBuffer;
	lpOutputParam.dwStatusSize = sizeof(StatusBuffer);

    if (NET_DVR_STDXMLConfig(lUserID, &lpInputParam, &lpOutputParam))
    {
    }
    else
    {
        ROS_INFO("StatusBuffer:%s", (char*)lpInputParam.lpRequestUrl);
    }

    char pJpegPicBuff[500000] = { 0 };  //Jpeg图片指针
	char pP2PDataBuff[500000] = { 0 };  //全屏测温数据指针      384*288*4 = 442368

    NET_DVR_JPEGPICTURE_WITH_APPENDDATA lpJpegWithAppend;
    lpJpegWithAppend.pJpegPicBuff = pJpegPicBuff;
    lpJpegWithAppend.dwJpegPicLen = sizeof(pJpegPicBuff);
    lpJpegWithAppend.pP2PDataBuff = pP2PDataBuff;
    lpJpegWithAppend.dwP2PDataLen = sizeof(pP2PDataBuff);

    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        NET_DVR_CaptureJPEGPicture_WithAppendData(lUserID, struDeviceInfoV40.struDeviceV30.byStartChan, &lpJpegWithAppend);

        DWORD   Size = lpJpegWithAppend.dwSize;
        DWORD   Channel = lpJpegWithAppend.dwChannel;//通道号
        DWORD   JpegPicLen = lpJpegWithAppend.dwJpegPicLen;//Jpeg图片长度
        DWORD   JpegPicWidth = lpJpegWithAppend.dwJpegPicWidth;  // 图像宽度
        DWORD   JpegPicHeight = lpJpegWithAppend.dwJpegPicHeight;  //图像高度
        DWORD   P2PDataLen = lpJpegWithAppend.dwP2PDataLen;//全屏测温数据长度
        BYTE    IsFreezedata = lpJpegWithAppend.byIsFreezedata;//是否数据冻结 0-否 1-是

        unsigned char array[288][384];
        unsigned char min = 255;
        unsigned char max = 0;
        for (size_t i = 0; i < 288; i++)
        {
            for (size_t j = 0; j < 384; j++)
            {
                union byte2float xbuf;
                xbuf.bytebuf[0] = (unsigned char)pP2PDataBuff[0+(i*384+j)*4];
                xbuf.bytebuf[1] = (unsigned char)pP2PDataBuff[1+(i*384+j)*4];
                xbuf.bytebuf[2] = (unsigned char)pP2PDataBuff[2+(i*384+j)*4];
                xbuf.bytebuf[3] = (unsigned char)pP2PDataBuff[3+(i*384+j)*4];
                array[i][j] = (((unsigned char)(xbuf.floatbuf - 0 )*2)<(255)? ((unsigned char)(xbuf.floatbuf - 0 )*2):(255));
                if (array[i][j]>max)
                {
                    max = array[i][j];
                }
                else if (array[i][j]<min)
                {
                    min = array[i][j];
                }
            }
        }
        //ROS_INFO("max = %d, min = %d",max, min);
        cv::Mat MatImg(288,384,CV_8UC1, (unsigned char*)array);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", MatImg).toImageMsg();

        if (subscriber_connected_)
        {
            pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
   

    //注销设备
    NET_DVR_Logout_V30(lUserID);
    // 释放SDK资源
    NET_DVR_Cleanup();

    return 0;
}