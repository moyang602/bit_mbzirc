/*
此程序用于读取蓝图并下发UAV与UGV建筑程序
*/
#include <actionlib/client/simple_action_client.h>
#include "bit_task_msgs/buildingAction.h"
#include "bit_task_msgs/BrickInfo.h"
#include <vector>
#include "GetBrick.h"
#include <algorithm>

typedef actionlib::SimpleActionClient<bit_task_msgs::buildingAction> Client;

class BuildingActionClient
{
public:
    BuildingActionClient(const std::string client_name, bool flag = true):client(client_name, flag) // 使用初始化列表来初始化client
    {
        ClientName = client_name;
    }

    // 客户端开始
    void Start()
    {
        // 等待服务器初始化完成
        ROS_INFO_STREAM("Waiting for UGV action server: ["<<ClientName<<"] to start!");
        client.waitForServer();
        ROS_INFO_STREAM("UGV action server: ["<<ClientName<<"] start!");
    }

    bool SendGoal(bit_task_msgs::buildingGoal goal, double Timeout = 10.0)   // 执行成功为true 失败为false
    {
        // 发送目标至服务器
        client.sendGoal(goal,
                        boost::bind(&BuildingActionClient::done_cb, this, _1, _2),
                        boost::bind(&BuildingActionClient::active_cb, this),
                        boost::bind(&BuildingActionClient::feedback_cb, this, _1));
        
        // 等待完成，超时时间Timeout s
        if(client.waitForResult(ros::Duration(Timeout)))    // 如果目标完成
        {
            ROS_INFO_STREAM("Task: ["<<ClientName<<"] finished within the time");
            return true;
        }
        else
        {
            ROS_INFO_STREAM("Task: ["<<ClientName<<"] failed, cancel Goal");
            client.cancelGoal();
            return false;
        }
        //ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
    }

private:
    Client client;
    std::string ClientName;      // action名称
    
private:
    /*
    *action完成时的回调函数，一次性
    */
    void done_cb(const actionlib::SimpleClientGoalState& state, const bit_task_msgs::buildingResultConstPtr& result)
    {
        ROS_INFO("Building task finished, State: %d",result->finish_state);
    }

    /*
    *action启动时的回调函数，一次性
    */
    void active_cb()
    {
        ROS_INFO("Ugv start building");
    }

    /*
    *action收到反馈时的回调函数
    */
    void feedback_cb(const bit_task_msgs::buildingFeedbackConstPtr& feedback)
    {
        ROS_INFO_STREAM("The UGV state is "<<feedback->task_feedback);
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task2_plan");
    
    // 任务变量初始化
    bit_task_msgs::buildingGoal ugv_building_goal;   // action 目标
    std::vector<bit_task_msgs::BrickInfo> ugv_brick; // 砖块信息堆栈
    bit_task_msgs::BrickInfo brick;                  // 单个砖块信息

    /* 建立UGV 堆墙action客户端 */
    BuildingActionClient UGVClient("ugv_building", true);    // 创建小车堆墙action客户端
    UGVClient.Start();

    // 建筑蓝图对象定义
    BrickPlan brickplan;    

    /* 读取建筑蓝图 下发指令*/
    vector<vector<double>> Task;
    vector<string> brick_in_car{"R","R","R","R","G","G","B"};
    for (size_t i = 2; i < 7; i++)
    {
        if (i<2)
        {
            Task = brickplan.get_specified_layer_of_wall_2(i);
        }
        else
        {
            Task = brickplan.get_specified_layer_of_wall_1(brick_in_car, i-2);
        }

        for (size_t j = 0; j < Task.size(); j++)
        {
            // 砖块信息赋值
            brick.Sequence = j; 
            if (i<2)
            {
                brick.type = "O";
            }
            else
            {
                brick.type = brick_in_car[j];
            }
            brick.x = Task[j][0];
            brick.y = Task[j][1];
            brick.z = Task[j][2];

            // 将砖块压入UGV搬运任务堆栈
            ugv_brick.push_back(brick);
        }
        reverse(ugv_brick.begin(), ugv_brick.end());
        /* 调用UAV 与 UGV action 服务器 */
        // UGV action 部分 设置目标值
        ugv_building_goal.goal_task.header.stamp = ros::Time::now();
        ugv_building_goal.goal_task.header.frame_id = "map";
        ugv_building_goal.goal_task.Num = Task.size();    // 从蓝图中获取
        ugv_building_goal.goal_task.bricks = ugv_brick;
        // 发送 UGV任务指令   等待100s
        if (UGVClient.SendGoal(ugv_building_goal, 1500)) // 如果指令搬砖正常
        {
            // 清空 砖块任务堆栈
            ugv_brick.clear();
        }
        else
        {
            // To do  失败的解决程序
        }
        
    }

    return 0;
}