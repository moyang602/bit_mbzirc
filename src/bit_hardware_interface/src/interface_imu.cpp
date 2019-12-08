#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#define g 9.81
bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

/**
 * #purpose	: 字符串转十六进制字符串
 * #note	: 可用于汉字字符串
 * #param str		: 要转换成十六进制的字符串
 * #param separator	: 十六进制字符串间的分隔符
 * #return	: 接收转换后的字符串
 */
std::string strToHex(std::string str, std::string separator = " 0x")
{
	const std::string hex = "0123456789ABCDEF";
	std::stringstream ss;
 
	for (std::string::size_type i = 0; i < str.size(); ++i)
		ss << separator << hex[(unsigned char)str[i] >> 4] << hex[(unsigned char)str[i] & 0xf];
	
	return ss.str();
}

int main(int argc, char** argv)
{
    // 串口相关
    serial::Serial ser;
    std::string param_port_path_;
    int param_baudrate_;
    int param_loop_rate_;

    // TF相关
    std::string tf_parent_frame_id;
    std::string tf_frame_id;
    std::string frame_id;
    double time_offset_in_seconds;
    bool broadcast_tf;
    double linear_acceleration_stddev;
    double angular_velocity_stddev;
    double orientation_stddev;
    int data_packet_start;

    tf::Quaternion orientation;
    tf::Quaternion zero_orientation;
    tf::Quaternion differential_rotation;

    // ROS节点
    ros::init(argc, argv, "interface_imu");
    // 声明节点私有句柄
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<std::string>("port", param_port_path_, "/dev/ttyUSB0");
    private_node_handle.param<int>("baudrate", param_baudrate_, 9600);
	private_node_handle.param<int>("loop_rate", param_loop_rate_, 20);
    
    private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
    private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
    private_node_handle.param<std::string>("frame_id", frame_id, "imu_link"); 
    private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
    private_node_handle.param<bool>("broadcast_tf", broadcast_tf, false);
    private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 1e-3);
    private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
    private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);
    // 声明节点共有句柄
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 50);
    ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);

    ros::Rate loop_rate(param_loop_rate_); // 默认 50 hz

    // imu消息初始化
    sensor_msgs::Imu imu;

    // imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
    // imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
    // imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

    // imu.angular_velocity_covariance[0] = angular_velocity_stddev;
    // imu.angular_velocity_covariance[4] = angular_velocity_stddev;
    // imu.angular_velocity_covariance[8] = angular_velocity_stddev;

    // imu.orientation_covariance[0] = orientation_stddev;
    // imu.orientation_covariance[4] = orientation_stddev;
    // imu.orientation_covariance[8] = orientation_stddev;

    imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
    imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
    imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

    imu.angular_velocity_covariance[0] = 1e-6;
    imu.angular_velocity_covariance[4] = 1e-6;
    imu.angular_velocity_covariance[8] = 1e-6;

    imu.orientation_covariance[0] =(1e-17)*7.43243; 
    imu.orientation_covariance[4] = (1e-9)*1.86259;
    imu.orientation_covariance[8] = (1e-15)*6.22595;


    // TF初始化
    static tf::TransformBroadcaster tf_br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));

    //
    std::string input;
    std::string read;

    while(ros::ok())
    {
        try
        {
            if (ser.isOpen())
            {
                // read string from serial device
                if(ser.available())
                {
                    read = ser.read(ser.available());
                    //ROS_INFO("read %i new characters, adding to %i characters of old input.", (int)read.size(), (int)input.size());
                    input += read;

                    while (input.length() >= 33) // 包最少有33个字节
                    {
                        //ROS_INFO_STREAM("receive data:"<<strToHex(input));
                        //parse for data packets
                        data_packet_start = input.find("\x55\x51");    // 寻找包头 0x55 0x51
                        if (data_packet_start != std::string::npos) // 如果找到进行处理
                        {
                            input.erase(0, data_packet_start); // 去掉前面无用的数据
                            if ((input.length() >= 33))  //如果存在完整的一帧
                            {
                                for (size_t count = 0; count < 3; count++)
                                {
                                    // 计算校验
                                    char sum = 0;
                                    for (size_t i = 0; i < 10; i++)
                                    {
                                        sum += input[i];
                                    }
                                    if (sum == input[10])   // 如果校验正确
                                    {
                                        uint8_t DataH = 0;
                                        uint8_t DataL = 0;
                                        switch (input[1])
                                        {
                                            case 0x51: // 线加速度
                                            {
                                                // get linear_acceleration values
                                                DataH = input[3];
                                                DataL = input[2];
                                                int16_t ax = ((int16_t)DataH<<8) | DataL;
                                                DataH = input[5];
                                                DataL = input[4];
                                                int16_t ay = ((int16_t)DataH<<8) | DataL;
                                                DataH = input[7];
                                                DataL = input[6];
                                                int16_t az = ((int16_t)DataH<<8) | DataL;
                                                // calculate accelerations in m/s²
                                                imu.linear_acceleration.x = ax * (16.0 / 32768.0)* g;
                                                imu.linear_acceleration.y = ay * (16.0 / 32768.0)* g;
                                                imu.linear_acceleration.z = az * (16.0 / 32768.0)* g;
                                                //ROS_INFO("linear_acceleration is: %8.3f, %8.3f, %8.3f",imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z);
                                            }
                                                break;
                                            case 0x52:  // 角速度
                                            {
                                                // get angular_velocity values
                                                DataH = input[3];
                                                DataL = input[2];
                                                int16_t gx = ((int16_t)DataH<<8) | DataL;
                                                DataH = input[5];
                                                DataL = input[4];
                                                int16_t gy = ((int16_t)DataH<<8) | DataL;
                                                DataH = input[7];
                                                DataL = input[6];
                                                int16_t gz = ((int16_t)DataH<<8) | DataL;
                                                // calculate rotational velocities in rad/s
                                                //TODO: check / test if rotational velocities are correct
                                                imu.angular_velocity.x = gx * (2000.0/32768.0) * (M_PI/180.0);
                                                imu.angular_velocity.y = gy * (2000.0/32768.0) * (M_PI/180.0);
                                                imu.angular_velocity.z = gz * (2000.0/32768.0) * (M_PI/180.0);
                                                //ROS_INFO("angular_velocity is: %8.3f, %8.3f, %8.3f",imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z);
                                            }
                                                break;
                                            case 0x59:  // 四元数
                                            {
                                                // get quaternion values
                                                DataH = input[3];
                                                DataL = input[2];
                                                int16_t w = ((int16_t)DataH<<8) | DataL;
                                                DataH = input[5];
                                                DataL = input[4];
                                                int16_t x = ((int16_t)DataH<<8) | DataL;
                                                DataH = input[7];
                                                DataL = input[6];
                                                int16_t y = ((int16_t)DataH<<8) | DataL;
                                                DataH = input[9];
                                                DataL = input[8];
                                                int16_t z = ((int16_t)DataH<<8) | DataL;

                                                double wf = w/32768.0;
                                                double xf = x/32768.0;
                                                double yf = y/32768.0;
                                                double zf = z/32768.0;

                                                //ROS_INFO("quaternion is: %8.3f, %8.3f, %8.3f %8.3f",wf,xf,yf,zf);

                                                // 初始化四元数
                                                tf::Quaternion orientation(xf, yf, zf, wf);

                                                if (!zero_orientation_set)
                                                {
                                                    zero_orientation = orientation;
                                                    zero_orientation_set = true;
                                                }

                                                //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
                                            
                                                differential_rotation = zero_orientation.inverse() * orientation;

                                                quaternionTFToMsg(differential_rotation, imu.orientation);
                                                
                                                
                                                double roll, pitch, yaw;//定义存储r\p\y的容器
                                                tf::Matrix3x3(differential_rotation).getRPY(roll, pitch, yaw);//进行转换
                                                imu.angular_velocity.x = roll;
                                                imu.angular_velocity.y = pitch;
                                                imu.angular_velocity.z = yaw;
                                            
                                            }
                                                break;
                                            default:
                                                break;
                                        }         
                                        input.erase(0, 11); // delete everything up to and including the processed packet
                                    }
                                    else
                                    {
                                        ROS_INFO("seems to be wrong data package");
                                        input.erase(0, 11); // delete everything up to and including the processed packet
                                    }
                                }
                                // calculate measurement time
                                ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

                                // publish imu message
                                imu.header.stamp = measurement_time;
                                imu.header.frame_id = frame_id;

                                imu_pub.publish(imu);

                                // publish tf transform
                                if (broadcast_tf)
                                {
                                    transform.setRotation(differential_rotation);
                                    tf_br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
                                }
                            }
                        }
                        else       // 如果没有找到清空缓存区
                        {
                            input.clear();
                        }
                    }
                }
            }
            else
            {
                // try and open the serial port
                try
                {
                    ser.setPort(param_port_path_);
                    ser.setBaudrate(param_baudrate_);
                    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                    ser.setTimeout(to);
                    ser.open();
                }
                catch (serial::IOException& e)
                {
                    ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
                    ros::Duration(5).sleep();
                }

                if(ser.isOpen())
                {
                    ROS_INFO_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
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
    }
}
