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
    private_node_handle.param<std::string>("port", param_port_path_, "/dev/ttyUSB1");
    private_node_handle.param<int>("baudrate", param_baudrate_, 115200);
	private_node_handle.param<int>("loop_rate", param_loop_rate_, 50);
    
    private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
    private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
    private_node_handle.param<std::string>("frame_id", frame_id, "imu_link"); 
    private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
    private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
    private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
    private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
    private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);
    // 声明节点共有句柄
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("IMU_data", 50);
    ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);

    ros::Rate loop_rate(param_loop_rate_); // 默认 50 hz

    // imu消息初始化
    sensor_msgs::Imu imu;

    imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
    imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
    imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

    imu.angular_velocity_covariance[0] = angular_velocity_stddev;
    imu.angular_velocity_covariance[4] = angular_velocity_stddev;
    imu.angular_velocity_covariance[8] = angular_velocity_stddev;

    imu.orientation_covariance[0] = orientation_stddev;
    imu.orientation_covariance[4] = orientation_stddev;
    imu.orientation_covariance[8] = orientation_stddev;

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
                    ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
                    input += read;

                    while (input.length() >= 11) // 包最少有11个字节
                    {
                        //parse for data packets
                        data_packet_start = input.find("$\x55");    // 寻找包头 0x55  ？？？ 待确定 可能没有$
                        if (data_packet_start != std::string::npos) // 如果找到进行处理
                        {
                            ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
                            if ((input.length() >= data_packet_start + 10))  //如果存在完整的一帧
                            {
                                // 计算校验
                                int sum = 0;
                                for (size_t i = data_packet_start; i < data_packet_start+10; i++)
                                {
                                    sum += input[i];
                                }
                                if (sum == input[data_packet_start + 10])
                                {
                                    ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                                    switch (input[data_packet_start+1])
                                    {
                                        case 0x51: // 线加速度
                                        {
                                            // get linear_acceleration values
                                            int16_t ax = ((short)input[data_packet_start + 2]<<8) | input[data_packet_start + 3];
                                            int16_t ay = ((short)input[data_packet_start + 4]<<8) | input[data_packet_start + 5];
                                            int16_t az = ((short)input[data_packet_start + 6]<<8) | input[data_packet_start + 7];
                                            // calculate accelerations in m/s²
                                            imu.linear_acceleration.x = ax * (16.0 / 32768.0) * g;
                                            imu.linear_acceleration.y = ay * (16.0 / 32768.0) * g;
                                            imu.linear_acceleration.z = az * (16.0 / 32768.0) * g;
                                        }
                                            break;
                                        case 0x52:  // 角速度
                                        {
                                            // get angular_velocity values
                                            int16_t gx = ((short)input[data_packet_start + 2]<<8) | input[data_packet_start + 3];
                                            int16_t gy = ((short)input[data_packet_start + 4]<<8) | input[data_packet_start + 5];
                                            int16_t gz = ((short)input[data_packet_start + 6]<<8) | input[data_packet_start + 7];
                                            // calculate rotational velocities in rad/s
                                            //TODO: check / test if rotational velocities are correct
                                            imu.angular_velocity.x = gx * (2000.0/32768.0) * (M_PI/180.0);
                                            imu.angular_velocity.y = gy * (2000.0/32768.0) * (M_PI/180.0);
                                            imu.angular_velocity.z = gz * (2000.0/32768.0) * (M_PI/180.0);
                                        }
                                            break;
                                        case 0x59:  // 四元数
                                        {
                                            // get quaternion values
                                            int16_t w = ((short)input[data_packet_start + 2]<<8) | input[data_packet_start + 3];
                                            int16_t x = ((short)input[data_packet_start + 4]<<8) | input[data_packet_start + 5];
                                            int16_t y = ((short)input[data_packet_start + 6]<<8) | input[data_packet_start + 7];
                                            int16_t z = ((short)input[data_packet_start + 8]<<8) | input[data_packet_start + 9];

                                            double wf = w/32768.0;
                                            double xf = x/32768.0;
                                            double yf = y/32768.0;
                                            double zf = z/32768.0;

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
                                        }
                                            break;
                                        default:
                                            break;
                                    }         
                                    input.erase(0, data_packet_start + 11); // delete everything up to and including the processed packet
                                }
                                else
                                {
                                    ROS_DEBUG("seems to be wrong data package");
                                    input.erase(0, data_packet_start + 11); // delete everything up to and including the processed packet
                                }
                         
                            }
                            else
                            {
                                if (input.length() >= data_packet_start + 11)
                                {
                                    input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                                }
                                else
                                {
                                    // do not delete start character, maybe complete package has not arrived yet
                                    input.erase(0, data_packet_start);
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
                ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
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
