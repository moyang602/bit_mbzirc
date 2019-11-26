#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tf/transform_listener.h>
#include <sstream>

#include <actionlib/client/simple_action_client.h>
#include "bit_motion/pickputAction.h"    
#include "bit_motion/locateAction.h"
#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/server/simple_action_server.h>
#include "bit_plan/buildingAction.h"

#include "bit_task/isAddressExist.h"
#include "bit_task/FindMapAddress.h"
#include "bit_task/WriteAddress.h"

#include "bit_task/UAV_msg.h"
#include "bit_task/UGV_msg.h"
#include "bit_task/Ground_msg.h"

#define TASK_GET 0
#define TASK_BUILD 1

typedef actionlib::SimpleActionClient<bit_motion::locateAction> Client_locate;
typedef actionlib::SimpleActionClient<bit_motion::pickputAction> Client_pickput;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client_movebase;
typedef actionlib::SimpleActionServer<bit_plan::buildingAction> Server;

struct UAV_data_struct
{
    uint8_t UAV_Num;			// 个体编号
    uint8_t flag_detect_bricks;		// 发现砖堆标志位（++至4，搜索砖堆任务完成）
    double x_r	;			// 红色位置x坐标
    double y_r	;			// 红色位置y坐标
    double x_g	;			// 绿色位置x坐标
    double y_g	;			// 绿色位置y坐标
    double x_b	;			// 蓝色位置x坐标
    double y_b	;			// 蓝色位置y坐标
    double x_o	;			// 橙色位置x坐标
    double y_o	;			// 橙色位置y坐标
    uint8_t flag_detect_L;			// 获取建筑位置标志位
    double x_L_cross;			// L型交点位置x坐标
    double y_L_cross;			// L型交点位置y坐标
    double x_L_1	;			// L型1位置x坐标
    double y_L_1	;			// L型1位置y坐标
    double x_L_2	;			// L型2位置x坐标
    double y_L_2	;			// L型2位置y坐标
    std::string state	;			// 当前任务状态
    uint8_t flag_droped;			// 砖块放置完成标志位
    double x		;			// 当前本机位置x坐标
    double y		;			// 当前本机位置y坐标
    double z		;			// 当前本机位置z坐标
    double vx		;			// 当前本机速度x方向的大小
    double vy		;			// 当前本机速度y方向的大小
    double vz		;			// 当前本机速度z方向的大小
};

UAV_data_struct UAV_data;

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

        void sendGoal(bit_motion::pickputGoal goal, double Timeout = 10.0)
        {
            // 发送目标至服务器
            client.sendGoal(goal,
                            boost::bind(&PickPutActionClient::done_cb, this, _1, _2),
                            boost::bind(&PickPutActionClient::active_cb, this),
                            boost::bind(&PickPutActionClient::feedback_cb, this, _1));
            
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

class MoveBaseActionClient    // 找砖action客户端
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
            bit_plan::buildingResult   result;      /* 创建一个result对象 */
            ROS_INFO("The goal brick num is: %d", goal->goal_task.Num);

            /*****************************************************
            *       连接各个动作服务器，等待后续指令
            *****************************************************/
            static PickPutActionClient Task2Client("pickputAction", true);   // 连接取砖动作服务器
            Task2Client.Start();

            static LocateActionClient LocateClient("locateAction", true); // 连接找砖动作服务器
            LocateClient.Start();

            static MoveBaseActionClient MoveBaseClient("move_base", true);      // 连接movebase动作服务器
            MoveBaseClient.Start();

            /*****************************************************
            *       创建各个服务访问客户端，等待后续指令
            *****************************************************/
            ros::NodeHandle n("~");
            ros::ServiceClient client_is = n.serviceClient<bit_task::isAddressExist>("isAddressExist");
            bit_task::isAddressExist srv_is;

            ros::ServiceClient client_find = n.serviceClient<bit_task::FindMapAddress>("FindMapAddress");
            bit_task::FindMapAddress srv_find;

            ros::ServiceClient client_write = n.serviceClient<bit_task::WriteAddress>("WriteAddress");
            bit_task::WriteAddress srv_write;

            /*****************************************************
            *       判断是否有砖堆信息与放置处信息
            *****************************************************/
            if(UAV_data.flag_detect_bricks!=4)      // 如果没有搜索完砖堆，返回失败
            {
                result.finish_state = 1;
                server.setAborted(result);
            }
            else
            {
                feedback.task_feedback = "The brick and building position exist";
                server.publishFeedback(feedback);
            }

            /*****************************************************
            *       循环搬运每个预设砖块
            *****************************************************/
            for (size_t count = 0; count < goal->goal_task.Num; count++)
            {
                // 根据 goal->goal_task.bricks[count].type 移动至相应颜色砖块处
                double moveX,moveY;
                switch (goal->goal_task.bricks[count].type[0])     // 判断需搬运砖块颜色位置
                {
                    case 'r':
                        moveX = UAV_data.x_r;
                        moveY = UAV_data.y_r;
                        break;

                    case 'g':
                        moveX = UAV_data.x_g;
                        moveY = UAV_data.y_g;
                        break;
                        
                    case 'b':
                        moveX = UAV_data.x_b;
                        moveY = UAV_data.y_b;
                        break;

                    case 'o':
                        moveX = UAV_data.x_o;
                        moveY = UAV_data.y_o;
                        break;
                    default:
                        break;
                }

                // 移动至砖堆处
                move_base_goal.target_pose.header.frame_id = "car_link";
                move_base_goal.target_pose.header.stamp = ros::Time::now();
                move_base_goal.target_pose.pose.position.x = moveX;             // Todo 设置为运动到以XY为圆心的圈上就行
                move_base_goal.target_pose.pose.position.y = moveY;
                target_quat = tf::createQuaternionMsgFromYaw(0);                // Todo 小车运动角度如何设置
                move_base_goal.target_pose.pose.orientation = target_quat;

                MoveBaseClient.sendGoal(move_base_goal, 100);
                ROS_INFO_STREAM("Move to the brick type: "<<goal->goal_task.bricks[count].type);


                // 将砖块搬运至车上
                pick_goal.goal_brick = goal->goal_task.bricks[count];  // 将队列砖块取出发送给取砖程序
                pick_goal.task = TASK_GET;

                Task2Client.sendGoal(pick_goal, 100);   // 发送目标 timeout 100s

                ROS_INFO("Picked %ld brick of %d", count+1, goal->goal_task.Num);
            }
            
            /*****************************************************
            *       移动到观察建筑处，调用检测砖堆状态
            *****************************************************/
            ROS_INFO("the building state is well");

            // 查找砖堆位置信息
            srv_find.request.AddressToFind = "ObservePlace";
            client_find.call(srv_find);

            // 移动至砖堆处
            move_base_goal.target_pose.header.frame_id = srv_find.response.AddressPose.header.frame_id;
            move_base_goal.target_pose.header.stamp = ros::Time::now();
            move_base_goal.target_pose.pose.position.x = srv_find.response.AddressPose.pose.position.x;
            move_base_goal.target_pose.pose.position.y = srv_find.response.AddressPose.pose.position.y;
            move_base_goal.target_pose.pose.orientation = srv_find.response.AddressPose.pose.orientation;

            MoveBaseClient.sendGoal(move_base_goal, 100);
            ROS_INFO_STREAM("Move to the observe place");

            // To do   获取建筑物状态 service


            /*****************************************************
            *       移动到建筑物处，循环放置各个砖块
            *****************************************************/
            // 移动至砖块建筑处
            ROS_INFO("move to the building place");

             // 查找砖堆位置信息
            srv_find.request.AddressToFind = "BuildingPlace";
            client_find.call(srv_find);

            // 移动至砖堆处
            move_base_goal.target_pose.header.frame_id = srv_find.response.AddressPose.header.frame_id;
            move_base_goal.target_pose.header.stamp = ros::Time::now();
            move_base_goal.target_pose.pose.position.x = srv_find.response.AddressPose.pose.position.x;
            move_base_goal.target_pose.pose.position.y = srv_find.response.AddressPose.pose.position.y;
            move_base_goal.target_pose.pose.orientation = srv_find.response.AddressPose.pose.orientation;

            MoveBaseClient.sendGoal(move_base_goal, 100);
            ROS_INFO_STREAM("Move to the building place");


            // 循环
            // 移动局部位置，需要使得车辆前进线对准目标位置
            // 放砖
            for (size_t count = 0; count < goal->goal_task.Num; count++)
            {
                // 将砖块放置在建筑物处
                pick_goal.goal_brick = goal->goal_task.bricks[count];  // 将队列砖块取出发送给取砖程序
                pick_goal.task = TASK_BUILD;

                Task2Client.sendGoal(pick_goal, 100);   // 发送目标 timeout 100s

                ROS_INFO("Put %ld brick of %d", count+1, goal->goal_task.Num);

            }

            ROS_INFO("COUNT DONE");

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

void UAVmsgCallback(const bit_task::UAV_msg::ConstPtr& msg)
{
    UAV_data.UAV_Num	= msg->UAV_Num;		// 个体编号
    UAV_data.flag_detect_bricks	= msg->flag_detect_bricks;	// 发现砖堆标志位（++至4，搜索砖堆任务完成）
    UAV_data.x_r			= msg->x_r;	// 红色位置x坐标
    UAV_data.y_r			= msg->y_r;	// 红色位置y坐标
    UAV_data.x_g			= msg->x_g;	// 绿色位置x坐标
    UAV_data.y_g			= msg->y_g;	// 绿色位置y坐标
    UAV_data.x_b			= msg->x_b;	// 蓝色位置x坐标
    UAV_data.y_b			= msg->y_b;	// 蓝色位置y坐标
    UAV_data.x_o			= msg->x_o;	// 橙色位置x坐标
    UAV_data.y_o			= msg->y_o;	// 橙色位置y坐标
    UAV_data.flag_detect_L	= msg->flag_detect_L;		// 获取建筑位置标志位
    UAV_data.x_L_cross		= msg->x_L_cross;	// L型交点位置x坐标
    UAV_data.y_L_cross		= msg->y_L_cross;	// L型交点位置y坐标
    UAV_data.x_L_1			= msg->x_L_1;	// L型1位置x坐标
    UAV_data.y_L_1			= msg->y_L_1;	// L型1位置y坐标
    UAV_data.x_L_2			= msg->x_L_2;	// L型2位置x坐标
    UAV_data.y_L_2			= msg->y_L_2;	// L型2位置y坐标
    UAV_data.state			= msg->state;	// 当前任务状态
    UAV_data.flag_droped	= msg->flag_droped;		// 砖块放置完成标志位
    UAV_data.x				= msg->x;	// 当前本机位置x坐标
    UAV_data.y				= msg->y;	// 当前本机位置y坐标
    UAV_data.z				= msg->z;	// 当前本机位置z坐标
    UAV_data.vx				= msg->vx;	// 当前本机速度x方向的大小
    UAV_data.vy				= msg->vy;	// 当前本机速度y方向的大小
    UAV_data.vz				= msg->vz;	// 当前本机速度z方向的大小
}


void GroundmsgCallback(const bit_task::Ground_msg::ConstPtr& msg)
{
  
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task2_action");
    ros::NodeHandle nh;

    ros::Subscriber subUAV = nh.subscribe("UAVmsg", 10, UAVmsgCallback);
    ros::Subscriber subGround = nh.subscribe("Groundmsg", 10, GroundmsgCallback);

    BuildingActionServer UGVServer(nh, "ugv_building", false);
    UGVServer.Start();

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }  

    return 0;
}