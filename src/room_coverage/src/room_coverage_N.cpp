
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static std::vector <move_base_msgs::MoveBaseGoal> arWayPoint;
static ros::Publisher marker_pub;
static visualization_msgs::Marker marker_waypoints;
static visualization_msgs::Marker text_marker;
static tf::StampedTransform map_current_pose;

void Init_WayPoints()
{
   
    int max=1,left=30,right=0,length=12;

    move_base_msgs::MoveBaseGoal newWayPoint;
    tf::Quaternion q;

    newWayPoint.target_pose.header.frame_id = "map";
    for (int i=0;i<=max;i++){
       for(int j=0;j<=max;j++){
    newWayPoint.target_pose.pose.position.x = (max-j)*length/max+map_current_pose.getOrigin().getX();
    newWayPoint.target_pose.pose.position.y = (i*left-(max-i)*right)/max+map_current_pose.getOrigin().getY();
    q.setRPY( 0, 0,(j == max ? (-1.57): 3.14) );
    newWayPoint.target_pose.pose.orientation.x = q.x();
    newWayPoint.target_pose.pose.orientation.y = q.y();
    newWayPoint.target_pose.pose.orientation.z = q.z();
    newWayPoint.target_pose.pose.orientation.w = q.w();
    arWayPoint.push_back(newWayPoint);
      }

    }
    
}

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

    geometry_msgs::Point point;
    point.z = 0.15;
    int nNumWP = arWayPoint.size();
    for(int i=0; i<nNumWP ; i++ )
    {
        point.x = arWayPoint[i].target_pose.pose.position.x;
        point.y = arWayPoint[i].target_pose.pose.position.y;
        marker_waypoints.points.push_back(point);
    }
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

void PublishWaypointsMarker()
{
    int nNumWP = arWayPoint.size();
    for(int i=0; i<nNumWP ; i++ )
    {
        float wp_x = arWayPoint[i].target_pose.pose.position.x;
        float wp_y = arWayPoint[i].target_pose.pose.position.y;

        std::ostringstream stringStream;
        stringStream << "wp_" << i;
        std::string face_id = stringStream.str();
        DrawTextMarker(face_id,i,0.2,wp_x,wp_y,marker_waypoints.scale.z+0.2,0,0.5,1.0);
    }
    marker_pub.publish(marker_waypoints);
    ros::spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "room_coverage");

    ros::NodeHandle nh;
    tf::TransformListener listener;
    ros::Rate loop_rate(20);

    try{
        listener.lookupTransform("car_link", "map", ros::Time(0), map_current_pose);
    }
        catch (tf::TransformException ex){
        ros::Duration(1.0).sleep();
    }

    marker_pub = nh.advertise<visualization_msgs::Marker>("waypoints_marker", 100);
    Init_WayPoints();
    Init_Marker();

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        if(!ros::ok())
            break;
        ROS_INFO("Waiting for the move_base action server to come up");
        PublishWaypointsMarker();
    }

    int nWPIndex = 0;

    while(ros::ok())
    {

        PublishWaypointsMarker();
        if(nWPIndex >= arWayPoint.size())
        {
            nWPIndex = 0;
            continue;
        }

        ROS_INFO("Go to the WayPoint[%d]",nWPIndex);
        ac.sendGoal(arWayPoint[nWPIndex]);

        ac.waitForResult(ros::Duration(40.0));

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Arrived at WayPoint[%d] !",nWPIndex);
            nWPIndex ++;
        }
        else{
            ac.cancelGoal();
            ROS_INFO("Failed to get to WayPoint[%d] ...",nWPIndex );
            nWPIndex ++;
        }
      
    }

    return 0;
}
