#include <ros/ros.h> 

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "bit_task/UGV_msg.h"

double car_vx = 0;
double car_vy = 0;
double car_vrz = 0;

void Vel_Callback(const geometry_msgs::Twist& cmd_vel)
{
    // 相对于car_link
    car_vx = cmd_vel.linear.x;
    car_vx = cmd_vel.linear.x;
    car_vrz = cmd_vel.angular.z;
}

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "msg_manage"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
 
    //订阅消息
    ros::Subscriber sub = nh.subscribe("cmd_vel", 500, Vel_Callback);
    //发布主题 
    ros::Publisher UGV_pub = nh.advertise<bit_task::UGV_msg>("UGV_data", 1);

    // TF 读取
    tf::TransformListener listener;
    tf::StampedTransform transform_CarOnMap;
    double car_x = 0;
    double car_y = 0;
    double car_z = 0;
    //指定循环的频率 
    ros::Rate loop_rate(20); 
    
    while(ros::ok()) 
    { 
        // 读取小车位置
        try
        {
            listener.lookupTransform("map", "car_link", ros::Time(0), transform_CarOnMap);
            car_x = transform_CarOnMap.getOrigin().x();
            car_y = transform_CarOnMap.getOrigin().y();
            car_z = transform_CarOnMap.getOrigin().z();
        }
        catch (tf::TransformException ex)
        {
            continue;
        }
        
        bit_task::UGV_msg ugv_data;
        
        ugv_data.flag_detect_blueprint = 0;		//识别建筑图案完成标志位
        //ugv_data.blueprint	= {1,2,3};		    //建筑图矩阵
        ugv_data.flag_update_blueprint = 0;		//监督更新标志位
        //ugv_data.blueprint_update = {1,2};	    //实时建筑图矩阵
        ugv_data.flag_finished = 0;			    //任务结束标志位
        ugv_data.x	= car_x;					//当前本机位置x坐标
        ugv_data.y	= car_y;			    	//当前本机位置y坐标
        ugv_data.z  = car_z;					//当前本机位置z坐标
        ugv_data.vx	= car_vx;					    //当前本机速度x方向的大小
        ugv_data.vy = car_vy;					    //当前本机速度y方向的大小
        ugv_data.vrz = car_vrz;				        //当前本机速度绕z方向的大小

        UGV_pub.publish(ugv_data);
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
}