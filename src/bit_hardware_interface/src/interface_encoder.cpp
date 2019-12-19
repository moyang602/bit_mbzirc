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

#include <bit_hardware_interface/encoder_srv.h>


#define CNT_NUM 10

serial::Serial ser; //声明串口对象

/* serial */
std::string param_port_path_;
int param_baudrate_;
int param_loop_rate_;

float res_encoder[4] = {0.0f};
bool data_ready = 0;
std::string param_good_init_rotation_;
bool use_absolute_encoder_ = true;

bool clbEncoder(bit_hardware_interface::encoder_srv::Request&  req,
                   bit_hardware_interface::encoder_srv::Response& res)
{
    ros::Time start = ros::Time::now();
    while(1){
        if (!use_absolute_encoder_){
            for (int i = 0;i<4; i++){
                res.init_pos.push_back(0.0f); 
            }
            res.success_flag = 1;
            break;
        }
        if ( (ros::Time::now().toSec() - start.toSec()) > 5.0 ){
            res.success_flag = 0;
            break;
        }
        if (data_ready) {
            res.init_pos.push_back(res_encoder[3]); 
            res.init_pos.push_back(res_encoder[2]); 
            res.init_pos.push_back(res_encoder[1]); 
            res.init_pos.push_back(res_encoder[0]); 
            res.success_flag = 1;
            break;
        }
    }
}


double good_rotation[4] = {0.0};
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "interface_encoder"); 
    //声明节点句柄 
    ros::NodeHandle nh("~"); 

    ros::ServiceServer service = nh.advertiseService("clbEncoder",clbEncoder);

	nh.param<std::string>("port", param_port_path_, "/dev/ttyUSB0");
	nh.param<int>("baudrate", param_baudrate_, 9600);
	nh.param<int>("loop_rate", param_loop_rate_, 1);
    nh.param<std::string>("good_init_rotation", param_good_init_rotation_, "[0.0 0.0 0.0 0.0]");
    nh.param<bool>("use_absolute_encoder", use_absolute_encoder_, true);
  
    
    sscanf(param_good_init_rotation_.c_str(),"[%lf %lf %lf %lf]",&(good_rotation[3]),&(good_rotation[2]),&(good_rotation[1]),&(good_rotation[0]));
    ROS_INFO("m0:%lf,m1:%lf,m2:%lf,m3:%lf",good_rotation[0],good_rotation[1],good_rotation[2],good_rotation[3]);

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

    uint8_t sum = 0;
    double init_encoder[4] = {0.0};
    uint8_t error[4] = {0};
    int encoder_cnt = 0;
    uint8_t rec[30] = {0};
    while(ros::ok()) 
    { 
        try{ 
                while(ser.available() < 10);
                //ROS_INFO("num:%d \n",ser.available());
                ser.read(rec,1);

                if (rec[0] == 0xaa){
                    ser.read(rec+1,13);
                    sum = 0;
                    for (int i = 0; i < 13 ; i++){
                        sum += rec[i];
                    }
                    if (rec[13] == sum){
                        for (int i = 0; i<4 ; i++){
                            init_encoder[i] = double((rec[3*i +1]<<15) | (rec[3*i +2]<<7) | (rec[3*i +3]>>1))/0xfffff * 3.1415926 *2.0;
                            error[i] = rec[3*i + 3] & 0x01;
                            init_encoder[i] = good_rotation[i]-init_encoder[i];
                            if (init_encoder[i] < -3.1415926) init_encoder[i] += 3.1415926*2;
                            if (init_encoder[i] > 3.1415926) init_encoder[i] -= 3.1415926*2;
                        }

                        ROS_INFO("m0:%lf,m1:%lf,m2:%lf,m3:%lf",init_encoder[3],init_encoder[2],init_encoder[1],init_encoder[0]);

                        if(error[0] +error[1] +error[2] +error[3] == 0){ //全都是0，都不报错
                            
                            if (encoder_cnt < CNT_NUM+10){
                                if (10 <=encoder_cnt){
                                    ROS_INFO("ok:%d",encoder_cnt);
                                    res_encoder[0] += init_encoder[0]/CNT_NUM;
                                    res_encoder[1] += init_encoder[1]/CNT_NUM;
                                    res_encoder[2] += init_encoder[2]/CNT_NUM;
                                    res_encoder[3] += init_encoder[3]/CNT_NUM;
                                }
                                encoder_cnt ++;
                            }
                            else{
                                data_ready = 1;
                            }
                        }   
                    }


                }
                

                
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
            ser.close();
        }

        ros::spinOnce(); 
        loop_rate.sleep(); 
        
        //处理ROS的信息，比如订阅消息,并调用回调函数 

    } 
    ser.close();
}
