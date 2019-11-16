#include "ros/ros.h"
#include "bit_task/FindMapAddress.h"
#include "bit_task/isAddressExist.h"
#include "bit_task/WriteAddress.h"

// 获取指定地点位置
bool FindMapAddress(bit_task::FindMapAddress::Request  &req,
                 bit_task::FindMapAddress::Response &res)
{
    std::string Name = req.AddressToFind;

    // To do 添加匹配找点的程序
    res.AddressPose.header.frame_id = "map";
    res.AddressPose.header.stamp = ros::Time().now();
    res.AddressPose.pose.position.x = 0;
    res.AddressPose.pose.position.y = 0;
    res.AddressPose.pose.position.z = 0;
    res.AddressPose.pose.orientation.x = 0;
    res.AddressPose.pose.orientation.y = 0;
    res.AddressPose.pose.orientation.z = 0;
    res.AddressPose.pose.orientation.w = 0;

    ROS_INFO_STREAM("The pose of ["<<Name<<"] has been send back");
}

// 指定地点位置是否存在
bool isAddressExist(bit_task::isAddressExist::Request  &req,
                 bit_task::isAddressExist::Response &res)
{
    std::string Name = req.AddressToFind;

    // To do 判断是否存在程序
    if (true)
    {
        res.flag = true;
        ROS_INFO_STREAM("The pose of ["<<Name<<"] exist");
    }
    else
    {
        res.flag = false;
        ROS_INFO_STREAM("The pose of ["<<Name<<"] doesn't exist");
    }
    
}

// 指定地点位置是否存在
bool WriteAddress(bit_task::WriteAddress::Request  &req,
                  bit_task::WriteAddress::Response &res)
{
    std::string Name = req.AddressToFind;

    // To do 写入地点程序
    if (true)
    {
        res.state = 0;
        ROS_INFO_STREAM("The pose of ["<<Name<<"] has been written");
    }
    else
    {
        res.state = 1;
        ROS_INFO_STREAM("The pose of ["<<Name<<"] exist");
    }
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_address_manage_node");
    ros::NodeHandle nh("~");

    ros::ServiceServer service_find = nh.advertiseService("FindMapAddress",FindMapAddress);
    ros::ServiceServer service_is = nh.advertiseService("isAddressExist",isAddressExist);
    ros::ServiceServer service_write = nh.advertiseService("WriteAddress",WriteAddress);

    ROS_INFO_STREAM("Map address manage start work");

    ros::spin();

    return 0;
}