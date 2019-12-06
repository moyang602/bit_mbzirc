/*
 * @Descripttion: 
 * @version: 
 * @Author: Ifan Ma
 * @Date: 2019-10-25 17:12:28
 * @LastEditors: Ifan Ma
 * @LastEditTime: 2019-10-25 19:50:17
 */
#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 

#include <sensor_msgs/NavSatFix.h>
#include <SerialComm.h>


serial::Serial ser; //声明串口对象

/* serial */
std::string param_port_path_;
int param_baudrate_;
int param_loop_rate_;

GPRMC gprmc;

int gps_analyse (char * buff , GPRMC *gps_data)
{
    
    char *ptr=NULL;
     if(gps_data==NULL)
      {
         return -1;
      }
      if( std::strlen(buff) < 10)
      {
         return -1;
      }
/* 如果buff字符串中包含字符"$GPRMC"则将$GPRMC的地址赋值给ptr */
      if( ( ptr = std::strstr(buff,"$GNGGA") ) ==NULL )
      {
         return -1;
      }
/* sscanf函数为从字符串输入，意思是将ptr内存单元的值作为输入分别输入到后面的结构体成员 */
      sscanf(ptr,"$GNGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,",&(gps_data->time),&(gps_data->latitude),\
             &(gps_data->la),&(gps_data->longitude),&(gps_data->lo),&(gps_data->mode),\
             &(gps_data->pos_state),&(gps_data->accu),&(gps_data->altitude));
      //ROS_INFO("$GNGGA,%f,%f,%c,%f,%c,%d,%d,  ,%f,",gps_data->time,gps_data->latitude,gps_data->la,gps_data->longitude,gps_data->lo,gps_data->mode,gps_data->pos_state,gps_data->altitude);
      if (gps_data->mode == 0){
          ROS_INFO("No Satellite!");
      }
      else {
          ROS_INFO("SateNum:%d ,Accracy:%f ,%f %c,%f %c, %3.2fm High",gps_data->pos_state,gps_data->accu,gps_data->latitude,gps_data->la,gps_data->longitude,gps_data->lo,gps_data->altitude);
      }

      return 0;
}


int GPS_GetData()
{
    int n = 0;
    int rtn = 0;
    std::string buff;
    buff = ser.read( GPS_LEN );
    if ( buff.length() < 0 ) 
    {
       ROS_ERROR_STREAM("read error\n");
       return -1;
    }

    memset(&gprmc, 0 , sizeof(gprmc));
    char buf[GPS_LEN];
    std::strcpy(buf,buff.c_str());
    gps_analyse( buf , & gprmc );
    return rtn;
}

int GPS_end()
{
    ser.close();
    return 0;
}

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "interface_gps"); 
    //声明节点句柄 
    ros::NodeHandle nh; 

    //发布主题 
    ros::Publisher GPS_pub = nh.advertise<sensor_msgs::NavSatFix>("gps", 1);

	nh.param<std::string>("port", param_port_path_, "/dev/ttyUSB0");
	nh.param<int>("baudrate", param_baudrate_, 9600);
	nh.param<int>("loop_rate", param_loop_rate_, 1);

    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort(param_port_path_); 
        ser.setBaudrate(param_baudrate_); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    //指定循环的频率 
    ros::Rate loop_rate(param_loop_rate_); 

    while(ros::ok()) 
    { 
        while(ser.available() < 10);

        //ROS_INFO("num:%d \n",ser.available());
        sensor_msgs::NavSatFix gps_data;

        gps_data.header.stamp = ros::Time::now();
        gps_data.header.frame_id = "gps_link";

        GPS_GetData();
        //gps_data.status="working";
        gps_data.latitude = gprmc.latitude;
        gps_data.longitude = gprmc.longitude;
        gps_data.altitude = gprmc.altitude;
    
        ser.flushInput(); 
        GPS_pub.publish(gps_data);
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
}