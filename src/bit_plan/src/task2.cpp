/*
此程序用于读取蓝图并下发UAV与UGV建筑程序
*/
#include <actionlib/client/simple_action_client.h>
#include "bit_plan/buildingAction.h"
#include "bit_task/BrickInfo.h"
#include <vector>
#include "blueprint_read.h"

typedef actionlib::SimpleActionClient<bit_plan::buildingAction> Client;

BrickPlan brickplan;    // 建筑蓝图对象定义

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

    bool SendGoal(bit_plan::buildingGoal goal, double Timeout = 10.0)   // 执行成功为true 失败为false
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
    void done_cb(const actionlib::SimpleClientGoalState& state, const bit_plan::buildingResultConstPtr& result)
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
    void feedback_cb(const bit_plan::buildingFeedbackConstPtr& feedback)
    {
        ROS_INFO_STREAM("The UGV state is "<<feedback->task_feedback);
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task2_plan");
    
    // 任务变量初始化
    bit_plan::buildingGoal ugv_building_goal;   // action 目标
    std::vector<bit_task::BrickInfo> ugv_brick; // 砖块信息堆栈
    bit_task::BrickInfo brick;                  // 单个砖块信息

    /* 建立UGV 堆墙action客户端 */
    BuildingActionClient UGVClient("ugv_building", true);    // 创建小车堆墙action客户端
    UGVClient.Start();

    /* 读取建筑蓝图 下发指令*/
    while (!brickplan.fir_wall_isFinished() || !brickplan.sec_wall_isFinished() )     // 当两个channel有一个没建筑完时，则继续
    {
        int count = 0;
        if (!brickplan.fir_wall_isFinished())
        {
            vector<double> cor = brickplan.get_fir_wall_nextbrick_xyz();
            // 砖块信息赋值
            brick.Sequence = count; 
            brick.type = "orange";
            brick.x = cor[0];
            brick.y = cor[1];
            brick.z = cor[2];
            
            count ++;

            // 将砖块压入UGV搬运任务堆栈
            ugv_brick.push_back(brick);
        }
       
        for (; count < 4; count++) // 读取特定数量的砖块
        {
            if (!brickplan.fir_wall_isFinished())
            {
                break;
            }
            
            vector<double> cor = brickplan.get_sec_wall_nextbrick_xyz();
            // 砖块信息赋值
            brick.Sequence = count;
            if (fabs(cor[3] -0.3)<1e-5)
            {
                brick.type = "red";
            }
            else if (fabs(cor[3] -0.6)<1e-5)
            {
                brick.type = "green";
            }
            else if (fabs(cor[3] -1.2)<1e-5)
            {
                brick.type = "blue";
            }
            brick.x = cor[0];
            brick.y = cor[1];
            brick.z = cor[2];

            // 将砖块压入UGV搬运任务堆栈
            ugv_brick.push_back(brick);
        }

        /* 调用UAV 与 UGV action 服务器 */
        // UGV action 部分 设置目标值
        ugv_building_goal.goal_task.header.stamp = ros::Time::now();
        ugv_building_goal.goal_task.header.frame_id = "map";
        ugv_building_goal.goal_task.Num = count;    // 从蓝图中获取
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
     ros::spin();

    return 0;
}