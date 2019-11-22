#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 

#include<geometry_msgs/Twist.h>
#include<bit_control_tool/heightNow.h>
#include<bit_control_tool/EndEffector.h>
#include "tf/transform_broadcaster.h"

serial::Serial ser; //声明串口对象

bit_control_tool::heightNow hn; 
bit_control_tool::EndEffector ee;

bool param_use_debug;
/* serial */
std::string param_port_path_;
int param_baudrate_;

//回调函数 
void write_callback_h(const geometry_msgs::Twist& cmd_vel) 
{
    float send = 0;
    send = hn.x + cmd_vel.linear.z * 1000;

    if (send > 670 ) send = 670;
    if (send < 320 ) send = 320;
    uint8_t cmd[4] = {'\0'};
    cmd[0] = 0xaa;
    cmd[1] = ( int(send *10) >> 8 ) &0xff;
    cmd[2] =   int(send *10 ) & 0xff;
    cmd[3] = 0x55;
    //ROS_INFO("1:%d,2:%d\n",cmd[1],cmd[2]);
    ser.write(cmd,4);
    ROS_INFO("SetHeight: %3.2f mm",send); 
    //ser.write(msg->data);   //发送串口数据 
} 

void write_callback_ee(const bit_control_tool::EndEffector & endeffCmd) 
{
    uint8_t cmd[4] = {'\0'};
    cmd[0] = 0xa0;
    ROS_INFO("SetEndEffector: "); 
    if ( endeffCmd.MagState > 0 ){
        cmd[1] = 0xbb;
        ROS_INFO("Mag: On"); 
    }else if ( endeffCmd.PumpState> 0){
        cmd[1] = 0xcc;
        ROS_INFO("Pump: On"); 
    }else {
        cmd[1] = 0xdd;
        ROS_INFO("Mag: Off"); 
        ROS_INFO("Pump: Off"); 
    }
    cmd[2] = 0x00;
    cmd[3] = 0x55;
    ser.write(cmd,4);
    
    //ser.write(msg->data);   //发送串口数据 
} 

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "bit_control_tool"); 
    //声明节点句柄 
    ros::NodeHandle nh; 

    uint8_t rec[4] = {'\0'}; 

    //订阅主题，并配置回调函数 
    ros::Subscriber height_sub = nh.subscribe("cmd_vel", 1000, write_callback_h); 
    //发布主题 
    ros::Publisher height_pub = nh.advertise<bit_control_tool::heightNow>("heightNow", 1000); 
    
   //订阅主题，并配置回调函数 
    ros::Subscriber endeff_sub = nh.subscribe("endeffCmd", 1000, write_callback_ee); 
    //发布主题 
    ros::Publisher endeff_pub = nh.advertise<bit_control_tool::EndEffector>("endeffState", 1000); 

    //nh.param<bool>("debug_imu", param_use_debug, false);
	nh.param<std::string>("io_control/port", param_port_path_, "/dev/ttyUSB1");
	nh.param<int>("io_control/baudrate", param_baudrate_, 9600);
    nh.param<bool>("io_control/debug_imu",param_use_debug,false);
    ROS_INFO_STREAM("port is " << param_port_path_);
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
        return -1; 
    } 

    //指定循环的频率 
    ros::Rate loop_rate(500); 
    while(ros::ok()) 
    { 

        if(ser.available()){ 
            //ROS_INFO_STREAM("Reading from serial port\n"); 
            
            ser.read(rec,ser.available());
            if (rec[0] == 0xaa)
            {
                if (rec[3] == 0x55){
                    hn.x = double( rec[1] *0x100 + rec[2] )/10.0f;
                    ROS_INFO_STREAM("rec:"<<hn.x);
                }
            }
            //ROS_INFO_STREAM(hn.x<<"mm");  
            if (rec[0] == 0xa0)
            {
                if (rec[3] == 0x55){
                    if ( rec[1] == 0xbb){
                        ee.MagState = 1;
                    }else if( rec[1] == 0xcc){
                        ee.PumpState = 1;
                    }else if ( rec[1] == 0xdd ){
                        ee.MagState = 0;
                        ee.PumpState = 0;
                    }
                }
            }
        } 
        endeff_pub.publish(ee);
        if (hn.x >320 && hn.x <670){
            height_pub.publish(hn);
        }  

        // 发布TF   car_link -> base_link
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.45, 0, hn.x/1000.0));
        tf::Quaternion q;
        q.setRPY(0, 0, 1.57079);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "car_link", "base_link"));

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 

    } 
}