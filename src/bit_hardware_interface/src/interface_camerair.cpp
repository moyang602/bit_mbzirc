#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include "HCNetSDK.h"
#include "LinuxPlayM4.h"
using namespace cv;

LONG nPort=-1;
HWND hWnd=0;

bool subscriber_connected_;  // 订阅者连接状态
ros::Publisher pub;	// 图像发布器
std::string param_MsgName;


//change vedio format from yv12 to YUV
void yv12toYUV(char *outYuv, char *inYv12, int width, int height,int widthStep)  
{  
   int col,row;  
   unsigned int Y,U,V;  
   int tmp;  
   int idx;   

    for (row=0; row<height; row++)  
    {  
        idx=row * widthStep;  
        int rowptr=row*width;  

        for (col=0; col<width; col++)  
        {  
            //int colhalf=col>>1;  
            tmp = (row/2)*(width/2)+(col/2);    
            Y=(unsigned int) inYv12[row*width+col];  
            U=(unsigned int) inYv12[width*height+width*height/4+tmp];  
            V=(unsigned int) inYv12[width*height+tmp];  

            outYuv[idx+col*3]   = Y;  
            outYuv[idx+col*3+1] = U;  
            outYuv[idx+col*3+2] = V;
        }  
    }
} 


//解码回调 视频为YUV数据(YV12)，音频为PCM数据  
void CALLBACK DecCBFun(int nPort,char * pBuf,int nSize,FRAME_INFO * pFrameInfo, void* nReserved1,int nReserved2)  
{  
    long lFrameType = pFrameInfo->nType;  
    
    if(lFrameType ==T_YV12)  //YV12
    {  
        //ROS_INFO("nSize = %d, nWidth = %d, nHeight = %d",nSize, pFrameInfo->nWidth,pFrameInfo->nHeight);
        
        IplImage* pImgYCrCb = cvCreateImage(cvSize(pFrameInfo->nWidth,pFrameInfo->nHeight), IPL_DEPTH_8U, 3);//得到图像的Y分量    
        yv12toYUV(pImgYCrCb->imageData, pBuf, pFrameInfo->nWidth,pFrameInfo->nHeight,pImgYCrCb->widthStep);//得到全部RGB图像  
        
        //申请内存
        IplImage* pImg = cvCreateImage(cvSize(pFrameInfo->nWidth,pFrameInfo->nHeight), IPL_DEPTH_8U, 3);  
        IplImage* motion = cvCreateImage(cvSize(pFrameInfo->nWidth,pFrameInfo->nHeight), IPL_DEPTH_8U, 1);  
        
        cvCvtColor(pImgYCrCb,pImg,CV_YCrCb2RGB);

        Mat MatImg(cvarrToMat(pImg));

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", MatImg).toImageMsg();

        if (subscriber_connected_)
		{
            pub.publish(msg);
        }

        cvReleaseImage(&pImgYCrCb); 
        cvReleaseImage(&pImg);
    }

    if(lFrameType ==T_AUDIO16)
    {
        //PCM    
    }

}

DWORD totalBufSize = 0;
BYTE TotolBuffer[11059200];
void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer,DWORD dwBufSize,void* dwUser)
{
    DWORD dRet; 

    switch (dwDataType)
    {
    case NET_DVR_SYSHEAD: //系统头

        if (!PlayM4_GetPort(&nPort)) //获取播放库未使用的通道号  
        {  
            break;  
        }  
        if(dwBufSize > 0)  
        {   
            if (!PlayM4_OpenStream(nPort,pBuffer,dwBufSize,1024*1024))  
            {  
                dRet=PlayM4_GetLastError(nPort);  
                break;  
            } 
            //设置解码回调函数 只解码不显示  
            if (!PlayM4_SetDecCallBack(nPort,DecCBFun))  
            {  
                dRet=PlayM4_GetLastError(nPort);  
                break;  
            }  
            //打开视频解码  
            if (!PlayM4_Play(nPort,hWnd))  
            {  
                dRet=PlayM4_GetLastError(nPort);  
                break;  
            }  

            //打开音频解码, 需要码流是复合流  
            if (!PlayM4_PlaySoundShare(nPort))  
            {  
                dRet=PlayM4_GetLastError(nPort);  
                break;  
            }         
        }  
        break; 


    case NET_DVR_STREAMDATA: //码流数据
        if (dwBufSize > 0 && nPort != -1)  
        {  
            BOOL inData=PlayM4_InputData(nPort,pBuffer,dwBufSize); 

            while (!inData)  
            {  
                waitKey(0);  
                inData=PlayM4_InputData(nPort,pBuffer,dwBufSize);  
                printf("PlayM4_InputData failed \n");     
            }  
        }
        break;
    default: //其他数据
        break;
    }
}

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
	nh.param<std::string>("MsgName", param_MsgName, "imagecar");

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
    strcpy(struLoginInfo.sDeviceAddress, "192.168.50.11"); //设备 IP 地址
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
   

    //---------------------------------------
    //启动预览并设置回调数据流
    LONG lRealPlayHandle;

    NET_DVR_PREVIEWINFO struPlayInfo = {0};
    struPlayInfo.hPlayWnd = 0; //需要 SDK 解码时句柄设为有效值，仅取流不解码时可设为空
    struPlayInfo.lChannel = 1; //预览通道号
    struPlayInfo.dwStreamType = 0; //0-主码流，1-子码流，2-码流 3，3-码流 4，以此类推
    struPlayInfo.dwLinkMode = 0; //0- TCP 方式，1- UDP 方式，2- 多播方式，3- RTP 方式，4-RTP/RTSP，5-RSTP/HTTP
    struPlayInfo.bBlocked = 1; //0- 非阻塞取流，1- 阻塞取流
    //struPlayInfo.byProtoType = 0; //应用层取流协议：0- 私有协议，1- RTSP 协议

    lRealPlayHandle = NET_DVR_RealPlay_V40(lUserID, &struPlayInfo, g_RealDataCallBack_V30, NULL);
    if (lRealPlayHandle < 0)
    {
        ROS_INFO("NET_DVR_RealPlay_V40 error");
        NET_DVR_Logout(lUserID);
        NET_DVR_Cleanup();
        return -1;
    }
    
    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    //关闭预览
    NET_DVR_StopRealPlay(lRealPlayHandle);
    //---------------------------------------

    if(!NET_DVR_StopRealPlay(lRealPlayHandle))  
    {  
        printf("NET_DVR_StopRealPlay error! Error number: %d\n", NET_DVR_GetLastError());  
        return -1;
    } 

    //注销设备
    NET_DVR_Logout_V30(lUserID);
    // 释放SDK资源
    NET_DVR_Cleanup();

    return 0;
}