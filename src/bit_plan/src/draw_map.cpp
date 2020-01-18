#include "ros/ros.h"
#include "bit_task_msgs/teach_robot.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
static ros::Publisher marker_pub;
static visualization_msgs::Marker marker_waypoints;
static visualization_msgs::Marker text_marker;
static tf::StampedTransform tf_CarOnMap;
static tf::StampedTransform S_CarOnMap;
static tf::Transform tf_CarOnS;
static std::vector<geometry_msgs::Pose> rec;

#define START 0
#define REC   1
#define FINISH2 2
#define FINISH3 3
#define GET2   4
#define GET3   5

using namespace std;
bool first2 = true;
bool first3 = true;

int index_temp = 0;


void Init_Marker()
{
    marker_waypoints.header.frame_id = "map";
    marker_waypoints.ns = "marker_waypoints";
    marker_waypoints.action = visualization_msgs::Marker::ADD;
    marker_waypoints.id = 1;
    marker_waypoints.type = visualization_msgs::Marker::CUBE_LIST;
    marker_waypoints.scale.x = 0.2;
    marker_waypoints.scale.y = 0.2;
    marker_waypoints.scale.z = 0.3;
    marker_waypoints.color.r = 0;
    marker_waypoints.color.g = 0.5;
    marker_waypoints.color.b = 1.0;
    marker_waypoints.color.a = 1.0;

}
void DrawTextMarker(std::string inText, int inID, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "map";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = inID;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    text_marker.text = inText;

    marker_pub.publish(text_marker);
}

void generate_point(int ia, float x, float y){
    geometry_msgs::Point point;
    point.z = 0.15;
    point.x = x;
    point.y = y;
    marker_waypoints.points.push_back(point);

    float ex_x = point.x;
    float ex_y = point.y;

    std::ostringstream stringStream;
    stringStream << "ex_" << ia;
    std::string face_id = stringStream.str();
    DrawTextMarker(face_id,ia,0.2,ex_x,ex_y,marker_waypoints.scale.z+0.2,0,0.5,1.0);
    marker_pub.publish(marker_waypoints);
}

static int i;
static int get_index2;
static int get_index3;
bool Teach_robot(bit_task_msgs::teach_robot::Request  &req,
                bit_task_msgs::teach_robot::Response  &res)
{
    ROS_INFO("in%d",req.kind);
    if (req.kind == START)
    {
        first2 = true;
        first3 = true;
        i = 0;
        S_CarOnMap = tf_CarOnMap;
        generate_point(i, tf_CarOnMap.getOrigin().getX(), tf_CarOnMap.getOrigin().getY());
        get_index2 = 0;
        get_index3 = 0;
    }
    else if (req.kind == REC)
    {
        
        i ++;
        tf_CarOnS = S_CarOnMap.inverse() * tf_CarOnMap.inverse().inverse();
        geometry_msgs::Pose temp;
        temp.position.x = tf_CarOnS.getOrigin().getX();
        temp.position.y = tf_CarOnS.getOrigin().getY();
        temp.position.z = 0.15;
        tf::quaternionTFToMsg(tf_CarOnS.getRotation(), temp.orientation);
        rec.push_back(temp);
        ROS_INFO_STREAM(rec.back());
        generate_point(i, tf_CarOnMap.getOrigin().getX(), tf_CarOnMap.getOrigin().getY());

    }
    else if (req.kind == FINISH3)
    {
        first3 = true;

        std::fstream f("pose_teach3.txt", ios::out);
        if(f.bad())
        {
            std::cout << "打开文件出错" << std::endl;
            return false;
        }
        
        for(int index = 0;index < rec.size(); index ++ )
        {
            f << index <<"\t" << rec[index].position.x << "\t" << rec[index].position.y << "\t" << rec[index].orientation.x << "\t" << rec[index].orientation.y << "\t" << rec[index].orientation.z << "\t" << rec[index].orientation.w << "\t" <<std::endl; 
        } 
        f.close();
        rec.clear();
     
    }
    else if (req.kind == FINISH2)
    {
        first2 = true;

        std::fstream f("pose_teach2.txt", ios::out);
        if(f.bad())
        {
            std::cout << "打开文件出错" << std::endl;
            return false;
        }
        
        for(int index = 0;index < rec.size(); index ++ )
        {
            f << index <<"\t" << rec[index].position.x << "\t" << rec[index].position.y << "\t" << rec[index].orientation.x << "\t" << rec[index].orientation.y << "\t" << rec[index].orientation.z << "\t" << rec[index].orientation.w << "\t" <<std::endl; 
        } 
        f.close();
        rec.clear();
     
    }
    else if (req.kind == GET3)
    {
        
        if (first3){
            double x;
            double y;
            double ox;
            double oy;
            double oz;
            double ow;

            first3 = false;
            fstream fin;
            fin.open("pose_teach3.txt",ios::in);    
        
            if(!fin)
            {
                std::cout << "打开文件出错" << std::endl;
                return false;
            }
            std::cout<<"文件打开成功，按行读入"<<std::endl;
            //如果知道数据格式，可以直接用>>读入

            
            geometry_msgs::Pose temp;
            while(!fin.eof())
            {
                fin >> index_temp >> x >> y >> ox >> oy >> oz >> ow ; 
                temp.position.x = x;
                temp.position.y = y;
                temp.position.z = 0.15;
                temp.orientation.x = ox;
                temp.orientation.y = oy;
                temp.orientation.z = oz;
                temp.orientation.w = ow;
                rec.push_back(temp);
                std::cout << "Read In: " << index_temp << std::endl;
            }
    
            fin.close();
        }
        
        if (get_index3 == index_temp){
            get_index3 = 0;
        }
        res.num = get_index3;
        res.pose = rec[get_index3];
        get_index3 ++;
    }
    else if (req.kind == GET2)
    {
        
        if (first2){
            double x;
            double y;
            double ox;
            double oy;
            double oz;
            double ow;

            first2 = false;
            fstream fin;
            fin.open("pose_teach2.txt",ios::in);    
        
            if(!fin)
            {
                std::cout << "打开文件出错" << std::endl;
                return false;
            }
            std::cout<<"文件打开成功，按行读入"<<std::endl;
            //如果知道数据格式，可以直接用>>读入

            
            geometry_msgs::Pose temp;
            while(!fin.eof())
            {
                fin >> index_temp >> x >> y >> ox >> oy >> oz >> ow ; 
                temp.position.x = x;
                temp.position.y = y;
                temp.position.z = 0.15;
                temp.orientation.x = ox;
                temp.orientation.y = oy;
                temp.orientation.z = oz;
                temp.orientation.w = ow;
                rec.push_back(temp);
                std::cout << "Read In: " << index_temp << std::endl;
            }
    
            fin.close();
        }
        
        if (get_index2 == index_temp){
            get_index2 = 0;
        }
        res.num = get_index2;
        res.pose = rec[get_index2];
        get_index2 ++;
    }

    
    // Init_WayPoints(req.x,req.y,req.r,req.p);
    
    // //tell the action client that we want to spin a thread by default

    // PublishWaypointsMarker();
    return true;
    
 }
 
int main(int argc, char **argv)
 {
    ros::init(argc, argv, "Teach_robot_server");
    ros::NodeHandle n;
    Init_Marker();

    ros::Rate loop_rate(20); 
    tf::TransformListener listener;

    marker_pub = n.advertise<visualization_msgs::Marker>("targetpoints_marker", 100);
    ROS_INFO("Teach_robot points are generated.");
    ros::ServiceServer service = n.advertiseService("Teach_robot",Teach_robot);

    while (ros::ok())
    {
        try{
            listener.lookupTransform("map","car_link", ros::Time(0), tf_CarOnMap);
        }
        catch (tf::TransformException ex){
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }

    for (int j =0; j< rec.size();j++)
    {
        ROS_INFO_STREAM(j<<rec[j]);
    }
   

    return 0;
}

  //文件打开方式选项：
//　ios::in　　　　= 0x01,　//供读，文件不存在则创建(ifstream默认的打开方式)
//　ios::out　　　 = 0x02,　//供写，文件不存在则创建，若文件已存在则清空原内容(ofstream默认的打开方式)
//　ios::ate　　　 = 0x04,　//文件打开时，指针在文件最后。可改变指针的位置，常和in、out联合使用
//　ios::app　　　 = 0x08,　//供写，文件不存在则创建，若文件已存在则在原文件内容后写入新的内容，指针位置总在最后
//　ios::trunc　　 = 0x10,　//在读写前先将文件长度截断为0（默认）
//　ios::nocreate　= 0x20,　//文件不存在时产生错误，常和in或app联合使用
//　ios::noreplace = 0x40,　//文件存在时产生错误，常和out联合使用
//　ios::binary　　= 0x80　 //二进制格式文件