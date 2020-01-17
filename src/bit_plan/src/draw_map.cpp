#include "ros/ros.h"
#include "bit_task_msgs/teach_robot.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
static ros::Publisher marker_pub;
static visualization_msgs::Marker marker_waypoints;
static visualization_msgs::Marker text_marker;
static tf::StampedTransform tf_CarOnMap;
static tf::StampedTransform S_CarOnMap;
static tf::Transform tf_CarOnS;
static std::vector<geometry_msgs::Pose> rec;

#define START 0
#define REC   1
#define GET   2


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
static int get_index;
bool Teach_robot(bit_task_msgs::teach_robot::Request  &req,
                bit_task_msgs::teach_robot::Response  &res)
{
    ROS_INFO("in%d",req.kind);
    if (req.kind == START)
    {
        i = 0;
        S_CarOnMap = tf_CarOnMap;
        generate_point(i, tf_CarOnMap.getOrigin().getX(), tf_CarOnMap.getOrigin().getY());
        get_index = 0;
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
    else if (req.kind == GET)
    {
        if (get_index == i){
            get_index = 0;
        }
        res.num = get_index;
        res.pose = rec[get_index];
        get_index ++;
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