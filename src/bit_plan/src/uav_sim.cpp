#include <ros/ros.h> 
#include <std_msgs/String.h> 
#include <bit_task/UAV_msg.h>

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "uav_sim_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 

    //发布主题 
    ros::Publisher uav_pub = nh.advertise<bit_task::UAV_msg>("UAV", 5);

    //指定循环的频率 
    ros::Rate loop_rate(10); 

    while(ros::ok()) 
    { 
        bit_task::UAV_msg uav_data;

        uav_data.UAV_Num = 1;               // 个体编号
        uav_data.flag_detect_bricks	= 4;	// 发现砖堆标志位（++至4，搜索砖堆任务完成）
        uav_data.x_r = 0.0;			    	// 红色位置x坐标
        uav_data.y_r = 0.0;			    	// 红色位置y坐标
        uav_data.radius_r = 0.0;	        // 红色区域半径
        uav_data.x_g = 0.0;			    	// 绿色位置x坐标
        uav_data.y_g = 0.0;			    	// 绿色位置y坐标
        uav_data.radius_g = 0.0;	        // 绿色区域半径
        uav_data.x_b = 0.0;			    	// 蓝色位置x坐标
        uav_data.y_b = 0.0;				    // 蓝色位置y坐标
        uav_data.radius_b = 0.0;	        // 蓝色区域半径
        uav_data.x_o = 0.0;				    // 橙色位置x坐标
        uav_data.y_o = 0.0;				    // 橙色位置y坐标
        uav_data.radius_o = 0.0;	        // 橙色区域半径
        uav_data.flag_detect_L = 1;		    // 获取建筑位置标志位
        uav_data.x_L_cross = 0.0;	        // L型交点位置x坐标
        uav_data.y_L_cross = 0.0;			// L型交点位置y坐标
        uav_data.x_L_1 = 0.0;				// L型1位置x坐标
        uav_data.y_L_1 = 0.0;				// L型1位置y坐标
        uav_data.x_L_2 = 0.0;				// L型2位置x坐标
        uav_data.y_L_2 = 0.0;				// L型2位置y坐标
        uav_data.state = "Run";		        // 当前任务状态
        uav_data.flag_droped = 0;			// 砖块放置完成标志位
        uav_data.x = 0.0;					// 当前本机位置x坐标
        uav_data.y = 0.0;					// 当前本机位置y坐标
        uav_data.z = 0.0;					// 当前本机位置z坐标
        uav_data.vx = 0.0;					// 当前本机速度x方向的大小
        uav_data.vy = 0.0;				    // 当前本机速度y方向的大小
        uav_data.vz = 0.0;					// 当前本机速度z方向的大小

        uav_pub.publish(uav_data);
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 

}