/*
此程序用于读取蓝图并下发UAV与UGV建筑程序
*/
#include <actionlib/client/simple_action_client.h>
#include "bit_plan/buildingAction.h"
#include "bit_task/BrickInfo.h"
#include <vector>

typedef actionlib::SimpleActionClient<bit_plan::buildingAction> Client;

/*
 *action完成时的回调函数，一次性
 */
void doneCd(const actionlib::SimpleClientGoalState& state, const bit_plan::buildingResultConstPtr& result)
{
    
    ROS_INFO("Building task finished, State: %d",result->finish_state);
    ros::shutdown();
}

/*
 *action启动时的回调函数，一次性
 */
void activeCd()
{
    ROS_INFO("Ugv start building");
}

/*
 *action收到反馈时的回调函数
 */
void feedbackCb(const bit_plan::buildingFeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM("The UGV state is "<<feedback->task_feedback);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task2_plan");
    
    Client client("ugv_building", true);       // 创建小车堆墙action客户端

    /* 等待服务端响应 */
    ROS_INFO("WAITING FOR UGV ACTION SERVER TO START !");
    client.waitForServer();
    ROS_INFO("UGV ACTION SERVER START !");

    bit_plan::buildingGoal ugv_building_goal;
    std::vector<bit_task::BrickInfo> ugv_brick;
    /* 建立UAV 堆墙action客户端 */


    /* 读取建筑蓝图 */

    bit_task::BrickInfo brick;
    brick.Sequence = 1;
    brick.type = 1;
    brick.x = 1;
    brick.y = 1;

    ugv_brick.insert(ugv_brick.begin(),brick);
    ugv_brick.insert(ugv_brick.begin(),brick);
    ugv_brick.insert(ugv_brick.begin(),brick);

    /* 循环调用UAV 与 UGV action 服务器 */

    // UGV action 部分
    ugv_building_goal.goal_task.header.stamp = ros::Time::now();
    ugv_building_goal.goal_task.header.frame_id = "map";
    ugv_building_goal.goal_task.Num = 3;    // 从蓝图中获取
    ugv_building_goal.goal_task.bricks = ugv_brick;
    
    client.sendGoal(ugv_building_goal, &doneCd, &activeCd, &feedbackCb);

    ros::spin();

    return 0;
}