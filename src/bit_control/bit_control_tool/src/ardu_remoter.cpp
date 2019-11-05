#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 

#include<geometry_msgs/Twist.h>

#define DEADZONE 50
#define MAX_x 1
#define MAX_y 1
#define MAX_z 0.6
serial::Serial ser; //声明串口对象

bool param_use_debug;

/* serial */
std::string param_port_path_;
int param_baudrate_;
int param_loop_rate_;
serial::parity_t param_patity_;

int i = 0;
int j = 0;
uint8_t sum = 0;
int rec_right[4] = {0};


int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "ardu_remoter_pub"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
	geometry_msgs::Twist cmd;

    uint8_t rec[2000] = {'\0'}; 

    //发布主题 
    ros::Publisher remo_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_remo", 1000); 
    
    //nh.param<bool>("debug_imu", param_use_debug, false);
	nh.param<std::string>("port", param_port_path_, "/dev/ttyUSB1");
	nh.param<int>("baudrate", param_baudrate_, 9600);
	nh.param<int>("loop_rate", param_loop_rate_, 20);
    nh.param<bool>("debug_imu",param_use_debug,false);

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
    ros::Rate loop_rate(param_loop_rate_); 
    int middle_cali = 0;
    i=0;
    uint8_t rec1=0;
    uint8_t index = 0;
    while(ros::ok()) 
    { 
        while(ser.available() < 10);

            //ROS_INFO("num:%d \n",ser.available());
            
        ser.read(rec,1);
        if (rec[0] == 0xaa)
        {
            ser.read(rec+1,9);
            for(j = 0; j<4 ; ++j){
                rec_right[j] = ( rec[2*j+1] <<8 ) + rec[2*j+2];
                sum += rec[2*j+1];
                sum += rec[2*j+2];
            }
            sum += 0xaa;
            //ROS_INFO("1:%d 2:%d 3:%d 4:%d \n",rec_right[0],rec_right[1],rec_right[2],rec_right[3]);
            //ROS_INFO("sum=%x;rec[9]=%x\n",sum,rec[9]);
            if (rec[9] == sum ){
                //ROS_INFO("ok\n");
                rec_right[0] -= 1500;
                rec_right[1] -= 1500;
                rec_right[2] -= 1000;
                rec_right[3] -= 1500;

                for (i= 0; i <4 ;i++){
                    if (i == 2){
                        if(rec_right[i] > 1000) rec_right[i] = 1000;
                        if(rec_right[i] < 0) rec_right[i] = 0;
                        if (rec_right[i]<100) rec_right[i] = 0;
                        else rec_right[i] -=100;
                    }
                    else{
                        if(rec_right[i] > 500) rec_right[i] = 500;
                        if(rec_right[i] < -500) rec_right[i] = -500;
                        if (abs(rec_right[i])<DEADZONE) rec_right[i] = 0;
                    }
                }

                cmd.linear.x = float( rec_right[0] ) ;
                cmd.linear.y = float( rec_right[1] ) ;
                cmd.angular.x = float( rec_right[2] ) ;
                cmd.angular.y = float( rec_right[3] ) ;
            }
            sum = 0;
            ser.flushInput(); 
        }
        else{
            continue;
        }        
        remo_pub.publish(cmd);
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } //
}
