#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <math.h>

#include <actionlib/client/simple_action_client.h>
#include "bit_motion/pickputAction.h"    
#include "bit_motion/locateAction.h"
#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/server/simple_action_server.h>
#include "bit_plan/buildingAction.h"

#include "bit_task/isAddressExist.h"
#include "bit_task/FindMapAddress.h"
#include "bit_task/WriteAddress.h"
#include "bit_control_tool/SetHeight.h"
#include "bit_vision/VisionProc.h"


#define TASK_GET 0
#define TASK_BUILD 1
#define TASK_LOOK_FORWARD 2
#define TASK_LOOK_DIRECT_DOWN 3

# define GetBrickPos        1   
# define GetBrickAngle      2
# define GetPutPos          3
# define GetPutAngle        4
# define GetLPose           5
# define GetBrickPos_only   6
# define NotRun             0

# define InitAngleToPick (0 *3.1416/180.0)


typedef actionlib::SimpleActionClient<bit_motion::locateAction> Client_locate;
typedef actionlib::SimpleActionClient<bit_motion::pickputAction> Client_pickput;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client_movebase;
typedef actionlib::SimpleActionServer<bit_plan::buildingAction> Server;

actionlib_msgs::GoalID cancel_id;
tf::StampedTransform T_ZedOnMap;
tf::Transform T_BrickOnLBase;
tf::Transform T_CarOnBrick;
int new_goal = 1;

class PickPutActionClient   // 取放砖action客户端
{
    public:
        PickPutActionClient(const std::string client_name, bool flag = true):client(client_name, flag) // 使用初始化列表来初始化client
        {
            ClientName = client_name;
        }

        // 客户端开始
        void Start()
        {
            if (!client.isServerConnected())
            {
                // 等待服务器初始化完成
                ROS_INFO_STREAM("Waiting for action server: ["<<ClientName<<"] to start!");
                client.waitForServer();
                ROS_INFO_STREAM("Action server: ["<<ClientName<<"] start!");
            }        
        }

        int sendGoal(bit_motion::pickputGoal goal, double Timeout = 10.0)
        {
            int fail = 0;
            // 发送目标至服务器
            client.sendGoal(goal,
                            boost::bind(&PickPutActionClient::done_cb, this, _1, _2),
                            boost::bind(&PickPutActionClient::active_cb, this),
                            boost::bind(&PickPutActionClient::feedback_cb, this, _1));
            
            // 等待完成，超时时间为10s
            if(client.waitForResult(ros::Duration(Timeout)))    // 如果目标完成
            {
                ROS_INFO_STREAM("Task: ["<<ClientName<<"] finished within the time");
                fail = 0;
            }
            else
            {
                ROS_INFO_STREAM("Task: ["<<ClientName<<"] failed, cancel Goal");
                client.cancelGoal();
                fail = 1;
            }
            ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
            return fail;
        }

    private:
        Client_pickput client;
        std::string ClientName;      // action名称

    private:
        /* action完成时的回调函数，一次性  */
        void done_cb(const actionlib::SimpleClientGoalState& state, const bit_motion::pickputResultConstPtr& result)
        {
            ROS_INFO("DONE:%d", result->finish_state);
        }
        /* action启动时的回调函数，一次性  */
        void active_cb()
        {
            ROS_INFO("ACTIVE");
        }
        /* action收到反馈时的回调函数 */
        void feedback_cb(const bit_motion::pickputFeedbackConstPtr& feedback)
        {
            ROS_INFO_STREAM("THE STATE NOM IS: "<<  feedback -> move_rightnow);
        }
};

class LocateActionClient    // 找砖action客户端
{
    public:
        LocateActionClient(const std::string client_name, bool flag = true):client(client_name, flag) // 使用初始化列表来初始化client
        {
            ClientName = client_name;
        }

        // 客户端开始
        void Start()
        {
            if (!client.isServerConnected())
            {
                // 等待服务器初始化完成
                ROS_INFO_STREAM("Waiting for action server: ["<<ClientName<<"] to start!");
                client.waitForServer();
                ROS_INFO_STREAM("Action server: ["<<ClientName<<"] start!");
            }        
        }

        void sendGoal(bit_motion::locateGoal goal, double Timeout = 10.0)
        {
            // 发送目标至服务器
            client.sendGoal(goal,
                            boost::bind(&LocateActionClient::done_cb, this, _1, _2),
                            boost::bind(&LocateActionClient::active_cb, this),
                            boost::bind(&LocateActionClient::feedback_cb, this, _1));
            
            // 等待完成，超时时间为10s
            if(client.waitForResult(ros::Duration(Timeout)))    // 如果目标完成
            {
                ROS_INFO_STREAM("Task: ["<<ClientName<<"] finished within the time");
            }
            else
            {
                ROS_INFO_STREAM("Task: ["<<ClientName<<"] failed, cancel Goal");
                client.cancelGoal();
            }
            ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
        }

    private:
        Client_locate client;
        std::string ClientName;      // action名称

    private:
        /* action完成时的回调函数，一次性  */
        void done_cb(const actionlib::SimpleClientGoalState& state, const bit_motion::locateResultConstPtr& result)
        {
            ROS_INFO("DONE:%d", result->finish_state);
        }
        /* action启动时的回调函数，一次性  */
        void active_cb()
        {
            ROS_INFO("ACTIVE");
        }
        /* action收到反馈时的回调函数 */
        void feedback_cb(const bit_motion::locateFeedbackConstPtr& feedback)
        {
            ROS_INFO_STREAM("THE STATE NOM IS: "<<feedback->feedback_state);
        }
};

class MoveBaseActionClient    // 移动action客户端
{
    public:
        MoveBaseActionClient(const std::string client_name, bool flag = true):client(client_name, flag) // 使用初始化列表来初始化client
        {
            ClientName = client_name;
        }

        // 客户端开始
        void Start()
        {
            if (!client.isServerConnected())
            {
                // 等待服务器初始化完成
                ROS_INFO_STREAM("Waiting for action server: ["<<ClientName<<"] to start!");
                client.waitForServer();
                ROS_INFO_STREAM("Action server: ["<<ClientName<<"] start!");
            }        
        }

        void sendGoal(move_base_msgs::MoveBaseGoal goal, double Timeout = 10.0)
        {
            // 发送目标至服务器
            client.sendGoal(goal);        

            // 等待完成，超时时间为10s
            if(client.waitForResult(ros::Duration(Timeout)))    // 如果目标完成
            {
                ROS_INFO_STREAM("Task: ["<<ClientName<<"] finished within the time");
            }
            else
            {
                ROS_INFO_STREAM("Task: ["<<ClientName<<"] failed, cancel Goal");
                client.cancelGoal();
            }
            ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
        }

    private:
        Client_movebase client;
        std::string ClientName;      // action名称
};


class BuildingActionServer  // UGV建筑action服务器
{
    public:
        BuildingActionServer(ros::NodeHandle nh, const std::string server_name, bool flag = false):
                        server(nh,server_name,boost::bind(&BuildingActionServer::execute_cb, this, _1),flag)
        {
            server.registerPreemptCallback(boost::bind(&BuildingActionServer::preempt_cb, this));
        }

        void Start()
        {
            server.start();   
        }

        /*
        *建筑 action
        */
        void execute_cb(const bit_plan::buildingGoalConstPtr& goal)
        {
            // 创建 ugv_building 向plan的反馈信息
            bit_plan::buildingFeedback feedback;    /* 创建一个feedback对象 */

            ROS_INFO("The goal brick num is: %d", goal->goal_task.Num);

        /*****************************************************
        *       准备步骤！连接各个动作服务器，等待后续指令
        *****************************************************/
            static PickPutActionClient Task2Client("pickputAction", true);   // 连接取砖动作服务器
            Task2Client.Start();

            // static LocateActionClient LocateClient("locateAction", true); // 连接找砖动作服务器
            // LocateClient.Start();

            static MoveBaseActionClient MoveBaseClient("move_base", true);      // 连接movebase动作服务器
            MoveBaseClient.Start();

        /*****************************************************
        *       准备步骤！创建各个服务访问客户端，等待后续指令
        *****************************************************/
            ros::NodeHandle n("~");
            ros::ServiceClient client_is = n.serviceClient<bit_task::isAddressExist>("isAddressExist");
            bit_task::isAddressExist srv_is;

            ros::ServiceClient client_find = n.serviceClient<bit_task::FindMapAddress>("/address_manage_node/FindMapAddress");
            bit_task::FindMapAddress srv_find;

            // // 查询砖块地址是否存在的服务客户端
            // ros::ServiceClient client_write = n.serviceClient<bit_task::WriteAddress>("isAddrWriteAddressessExist");
            // bit_task::WriteAddress srv_write;

            // 视觉处理的客户端
            ros::ServiceClient client_vision = n.serviceClient<bit_vision::VisionProc>("GetVisionData");
            bit_vision::VisionProc srv_vision;

            // 设定高度的客户端
            ros::ServiceClient client_height = n.serviceClient<bit_control_tool::SetHeight>("SetHeight");
            bit_control_tool::SetHeight srv_height;

            // simple_goal 话题发布
            ros::Publisher simp_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
            geometry_msgs::PoseStamped this_target;

            //cancel
            ros::Publisher simp_cancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);
            
            int fail_cnt = 0;

        /*****************************************************
        *       第一步！判断是否有砖堆信息与放置处信息
        *****************************************************/
            // srv_is.request.AddressToFind = "red";
            // client_is.call(srv_is);
            // if (srv_is.response.flag)   // 如果有砖堆信息
            // {
            //     feedback.task_feedback = "The brick and building position exist";
            //     server.publishFeedback(feedback);
            // }
            // else        // 如果没有砖堆信息
            // {
            //     feedback.task_feedback = "Looking for the position of brick and building";
            //     server.publishFeedback(feedback);
            //     // 找砖和墙位置，
            //     // 然后在建筑位置进行global costmap的设置，放置车辆开过砖墙区域  
            //     //LocateClient.sendGoal(locate_goal, 100);     // 找砖程序客户端

            //     feedback.task_feedback = "The position of brick and building has been found";
            //     server.publishFeedback(feedback);
            // }
            // ROS_INFO_STREAM("Move to the brick type: "<< goal->goal_task.bricks[count].type);
            
            

        /*****************************************************
        *       第二步！抵达每种砖堆，循环拾取每个预设砖块
        *****************************************************/
            ROS_INFO("Move to the first brick\n");

        //2.1 摄像机冲地
            fail_cnt = 0;
            pick_goal.task = TASK_LOOK_DIRECT_DOWN;
            while(Task2Client.sendGoal(pick_goal, 100)  && fail_cnt ++ < 5);   // 发送目标 timeout 100s
            ROS_INFO("Camera OK!");
        //2.2 循环取砖的过程，重要过程，细节很多！根据 goal->goal_task.bricks[count].type 移动至相应颜色砖块处
            fail_cnt = 0;
            for (size_t count = 0; count < goal->goal_task.Num + fail_cnt; count++)
            {
            //2.2.1 询问砖堆位置
                srv_find.request.AddressToFind = goal->goal_task.bricks[count-fail_cnt].type;
                client_find.call(srv_find);

            //2.2.2 移动至砖堆处
            // 降低升降台
                srv_height.request.req_height.x = 320; //最低高度，捡砖位置
                client_height.call(srv_height); // 非阻塞运行，同时

                ROS_INFO("height OK!");
            // 移动直到检测到达砖块上方
                ros::Time start_time = ros::Time::now();  // 超时检测，记录初始时刻
                
                while((ros::Time::now().toSec()-start_time.toSec()) < 100){ // 检测到砖块退出，超时100s
                // 计算位置
                    float radius = srv_find.response.radius + 0.75; //得到的砖块半径加上车体外接圆半径 sqrt(0.45^2 + 0.6^2)

                    this_target.header.frame_id = srv_find.response.AddressPose.header.frame_id;
                    this_target.header.stamp = ros::Time::now();
                    this_target.pose.position.x = srv_find.response.AddressPose.pose.position.x - radius/2.0 * cos(InitAngleToPick + count*3.1416/5); // 目标附近R/2处的圆
                    this_target.pose.position.y = srv_find.response.AddressPose.pose.position.y - radius/2.0 * sin(InitAngleToPick + count*3.1416/5);
                    target_quat = tf::createQuaternionMsgFromYaw(InitAngleToPick + count*3.1416/5);
                    this_target.pose.orientation = target_quat;  //srv_find.response.AddressPose.pose.orientation;//注释掉的为得到转角，但就是固定的，现在为设置转角朝向圆心
                    // 砖堆可能不是圆的，有可能会有别的形状，怎么处理

                            /* ======================= 固定砖堆位置为 map坐标系前2.0M ===================== */
                            // this_target.header.frame_id = "map";
                            // this_target.header.stamp = ros::Time::now();
                            // this_target.pose.position.x = 2.0;
                            // this_target.pose.position.y = 0;
                            // this_target.pose.orientation = tf::createQuaternionMsgFromYaw(3.1416);
                            /* =============================== 删除分割线 =============================== */
                // 发布位置
                    if (new_goal){
                        new_goal = 0;
                        simp_goal_pub.publish(this_target);
                        ROS_INFO("published!");
                    }

                    // move_base_goal.target_pose.header.frame_id = srv_find.response.AddressPose.header.frame_id;
                    // move_base_goal.target_pose.header.stamp = ros::Time::now();
                    // move_base_goal.target_pose.pose.position.x = srv_find.response.AddressPose.pose.position.x;
                    // move_base_goal.target_pose.pose.position.y = srv_find.response.AddressPose.pose.position.y;
                    // move_base_goal.target_pose.pose.orientation = srv_find.response.AddressPose.pose.orientation;
                    // MoveBaseClient.sendGoal(move_base_goal, 100); // 阻塞运行，直到成功之后才继续

                // 调用视觉检测是否到达砖上
                    srv_vision.request.ProcAlgorithm = GetBrickPos_only;
                    srv_vision.request.BrickType = goal->goal_task.bricks[count - fail_cnt].type;
                    
                    client_vision.call(srv_vision);

                    if(srv_vision.response.VisionData.Flag){
                        simp_cancel.publish(cancel_id);
                        break;
                    }   
                }
                // 循环结束，到达了第一块砖的位置

                // 将砖块搬运至车上
                    pick_goal.goal_brick = goal->goal_task.bricks[count - fail_cnt];  // 将队列砖块取出发送给取砖程序
                    pick_goal.task = TASK_GET;

                    if (Task2Client.sendGoal(pick_goal, 100)){    // 发送目标 timeout 100s
                        fail_cnt += 1;      //失败通过失败计数加一来保证砖还是同一块砖，但是位置移动了
                        ROS_INFO("Failed this time");
                    }
                    else{
                        ROS_INFO("Picked %ld brick of %d", count + 1 - fail_cnt, goal->goal_task.Num);
                    } 

                //第一块砖处理结束，有可能失败

            }   
        // 循环每一块砖结束，所有的砖上车
            ROS_INFO("All bricks are on the car!\n");

        /*****************************************************
        *       第三步！转换到看地姿态，准备识别L型架端点， 原先为识别建筑物状态
        *****************************************************/
            ROS_INFO("To look at ground");

        //3.1 转换到看地姿态，准备识别L型架端点
            fail_cnt = 0;
            pick_goal.task = TASK_LOOK_DIRECT_DOWN;
            while( Task2Client.sendGoal(pick_goal, 100) && fail_cnt ++ < 5);   // 发送目标 timeout 100s
            
        /*****************************************************
        *       第四步！移动到建筑物处，循环放置各个砖块
        *****************************************************/
        //4.1 移动至砖块建筑处
            ROS_INFO("moving to the building place");

            //4.1.1 查找砖堆位置信息
                srv_find.request.AddressToFind = "BuildingPlace";
                client_find.call(srv_find);

            //4.1.2 移动至砖墙端点处
                move_base_goal.target_pose.header.frame_id = srv_find.response.AddressPose.header.frame_id;
                move_base_goal.target_pose.header.stamp = ros::Time::now();
                move_base_goal.target_pose.pose.position.x = srv_find.response.AddressPose.pose.position.x;
                move_base_goal.target_pose.pose.position.y = srv_find.response.AddressPose.pose.position.y;
                move_base_goal.target_pose.pose.orientation = srv_find.response.AddressPose.pose.orientation;

                        /* ======================= 临时固定建筑位置为 map原点 ========================= */
                        move_base_goal.target_pose.header.frame_id = "map";
                        move_base_goal.target_pose.header.stamp = ros::Time::now();
                        move_base_goal.target_pose.pose.position.x = 0.0;
                        move_base_goal.target_pose.pose.position.y = 0;
                        move_base_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.1416);
                        /* =============================== 删除分割线 =============================== */

                MoveBaseClient.sendGoal(move_base_goal, 100);
                ROS_INFO_STREAM("Move to the building start point");

            //4.1.3 获取墙基的一端相对于Zed的变换
                srv_vision.request.ProcAlgorithm = GetLPose;
                client_vision.call(srv_vision);
                tf::Stamped<tf::Transform> T_LBaseOnZed;
                geometry_msgs::PoseStamped poset;
                poset.header = srv_vision.response.VisionData.header;
                poset.pose = srv_vision.response.VisionData.Pose;
                tf::poseStampedMsgToTF(poset,T_LBaseOnZed);
            

        //4.2 循环到砖堆处，并放砖搭墙，重要过程，细节很多!
            fail_cnt = 0;
            for (size_t count = 0; count < goal->goal_task.Num + fail_cnt; count++)
            {
            //4.2.1 移动局部位置，需要使得车辆前进线基本对准目标位置
                // 开环位置控制OK吗？直接平移一段距离
                // 通过激光雷达的平面（直线）来生成预定轨迹

                this_target.header.frame_id = "map";
                this_target.header.stamp = ros::Time::now();
                T_BrickOnLBase.setRotation(tf::createIdentityQuaternion());
                T_BrickOnLBase.setOrigin(tf::Vector3(goal->goal_task.bricks[count - fail_cnt].x, goal->goal_task.bricks[count - fail_cnt].y, 0));
                tf::Transform temp_trans = T_ZedOnMap*T_LBaseOnZed*T_BrickOnLBase*T_CarOnBrick;
                
                this_target.pose.position.x = temp_trans.getOrigin().x();
                this_target.pose.position.y = temp_trans.getOrigin().y();
                tf::quaternionTFToMsg(temp_trans.getRotation(),this_target.pose.orientation);
                move_base_goal.target_pose = this_target;
                MoveBaseClient.sendGoal(move_base_goal, 100);
                ROS_INFO_STREAM("Move to this building position");

            //4.2.2 将砖块放置在建筑物处
                pick_goal.goal_brick = goal->goal_task.bricks[count - fail_cnt];  // 将队列砖块取出发送给取砖程序
                pick_goal.task = TASK_BUILD;

                if (Task2Client.sendGoal(pick_goal, 100)){      // 发送目标 timeout 100s 返回是否失败
                    fail_cnt += 1;      //失败通过失败计数加一来保证砖还是同一块砖，但是位置移动了
                    ROS_INFO("Failed this time");
                }
                else{
                    ROS_INFO("Put %ld brick of %d", count + 1- fail_cnt, goal->goal_task.Num);
                }      
            // 第一块砖处理结束，有可能失败
            }
        //结束搭建
        ROS_INFO("COUNT DONE");


        /*****************************************************
        *       第五步！结束，返回result
        *****************************************************/
            server.setSucceeded();   /* 发送result */
        }

    private:
        Server server;
        /* 创建搬运action目标 */
        bit_motion::pickputGoal pick_goal;   
        bit_motion::locateGoal locate_goal;
        move_base_msgs::MoveBaseGoal move_base_goal;

        geometry_msgs::Quaternion target_quat;
        
        // 中断回调函数
        void preempt_cb()
        {
            if(server.isActive())
            {
                server.setPreempted(); // 强制中断
            }
        }        
};

void feedback_cb(move_base_msgs::MoveBaseActionFeedback a)
{
    cancel_id = a.status.goal_id;
    if (a.status.ACTIVE != 1){
        new_goal = 1;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task2_action_know_test");
    ros::NodeHandle nh;

    BuildingActionServer UGVServer(nh, "ugv_building", false);
    UGVServer.Start();


    ros::Subscriber simp_goal_sub = nh.subscribe("move_base/feedback", 1000, feedback_cb);

    tf::TransformListener listener;

    T_CarOnBrick.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));
    T_CarOnBrick.setOrigin(tf::Vector3(0.0, -1.0, 0.0));
    
    while(ros::ok()) 
    { 
        // 获取 zed_link 在 base_link下的坐标
        try{
        listener.lookupTransform("map", "zed_link", ros::Time(0), T_ZedOnMap);
        }
        catch (tf::TransformException ex){
        //ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }
        
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
    } 

    return 0;
}