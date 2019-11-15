#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <actionlib/client/simple_action_client.h>
#include "bit_motion/pickputAction.h"    

#include <actionlib/server/simple_action_server.h>
#include "bit_plan/buildingAction.h"


#define TASK_GET 0
#define TASK_BUILD 1

typedef actionlib::SimpleActionClient<bit_motion::pickputAction> Client;
typedef actionlib::SimpleActionServer<bit_plan::buildingAction> Server;

int ok = 1;
/*
 *action完成时的回调函数，一次性
 */
void pickdoneCd(const actionlib::SimpleClientGoalState& state, const bit_motion::pickputResultConstPtr& result)
{
    ROS_INFO("DONE:%d", result->finish_state);
    
    ok = 0;    
}

/*
 *action启动时的回调函数，一次性
 */
void pickactiveCd()
{
    ROS_INFO("ACTIVE");
}

/*
 *action收到反馈时的回调函数
 */
void pickfeedbackCb(const bit_motion::pickputFeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM("THE STATE NOM IS: "<<  feedback -> move_rightnow);
}

/* 创建搬运action目标 */
bit_motion::pickputGoal pick_goal;          
/*
 *建筑 action
 */
void ExecuteBuildingPlan(const bit_plan::buildingGoalConstPtr& goal, Server* as)
{
    bit_plan::buildingFeedback feedback;    /* 创建一个feedback对象 */

    ROS_INFO("The goal brick num is: %d", goal->goal_task.Num);

    /* 判断是否有砖堆信息与放置处信息 */
    if (true)   // 如果有砖堆信息
    {
        feedback.task_feedback = "The brick and building position exist";
        as->publishFeedback(feedback);
    }
    else        // 如果没有砖堆信息
    {
        feedback.task_feedback = "Looking for the position of brick and building";
        as->publishFeedback(feedback);
        // 找砖和墙位置，
        // 然后在建筑位置进行global costmap的设置，放置车辆开过砖墙区域  

        ros::Duration(5).sleep();       // 临时占位     换成action

        feedback.task_feedback = "The position of brick and building has been found";
        as->publishFeedback(feedback);
    }
    
    static Client pickclient("pickputAction", true);   // 调用取砖动作服务器

    ROS_INFO("Waiting for pickput action server to start!");
    pickclient.waitForServer();                 /* 等待取砖服务端响应 */
    ROS_INFO("pickput action server started !");

    // 循环每个预设砖块信息
    for (size_t count = 0; count < goal->goal_task.Num; count++)
    {
        // 根据 goal->goal_task.bricks[count].type 移动至相应颜色砖块处
        ROS_INFO_STREAM("Move to the brick type: "<<goal->goal_task.bricks[count].type);
        
        ros::Duration(3).sleep();       // 临时占位     换成 service

        // 将砖块搬运至车上
        ok = 1;
          
        pick_goal.goal_brick = goal->goal_task.bricks[count];  // 将队列砖块取出发送给取砖程序
        pick_goal.task = TASK_GET;

        pickclient.sendGoal(pick_goal, &pickdoneCd, &pickactiveCd, &pickfeedbackCb);        /* 发送目标，并且定义回调函数 */

        while(ok){;}  // 等待搬运完成

        ROS_INFO("Picked %ld brick of %d", count+1, goal->goal_task.Num);
    }
    
    // 移动到观察建筑处，调用检测砖堆状态
    ROS_INFO("the building state is well");

    ros::Duration(3).sleep();       // 临时占位     换成 service


    // 移动至砖块建筑处
    ROS_INFO("move to the building place");

    ros::Duration(3).sleep();       // 临时占位     换成 service

    // 循环
    // 移动局部位置，需要使得车辆前进线对准目标位置
    // 放砖
    for (size_t count = 0; count < goal->goal_task.Num; count++)
    {
        // 计算出欲放置砖块相对于目前的位置
        // 移动

        ros::Duration(3).sleep();       // 临时占位     换成 service

        // 将砖块放置在建筑物处
        ok = 1;

        pick_goal.goal_brick = goal->goal_task.bricks[count];  // 将队列砖块取出发送给取砖程序
        pick_goal.task = TASK_BUILD;

        pickclient.sendGoal(pick_goal, &pickdoneCd, &pickactiveCd, &pickfeedbackCb);       /* 发送目标，并且定义回调函数 */

        while(ok){;}  // 等待建筑完成

        ROS_INFO("Put %ld brick of %d", count+1, goal->goal_task.Num);

    }

    ROS_INFO("COUNT DONE");

    as->setSucceeded();   /* 发送result */
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task2_action");

    ros::NodeHandle nh;

    Server server(nh, "ugv_building", boost::bind(&ExecuteBuildingPlan, _1, &server), false);

    server.start();

    ros::spin();

    return 0;
}