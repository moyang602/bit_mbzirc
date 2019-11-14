#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <actionlib/client/simple_action_client.h>
#include "bit_motion/pickputAction.h"    /* 这个头文件每个人写的的名字都可能不同，package name/action file name + Action.h */


#define TASK_GET 0
#define TASK_BUILD 1

typedef actionlib::SimpleActionClient<bit_motion::pickputAction> Client;
int ok = 1;
/*
 *action完成时的回调函数，一次性
 */
void pickdoneCd(const actionlib::SimpleClientGoalState& state, const bit_motion::pickputResultConstPtr& result)
{
    ROS_INFO("DONE:%d", result->finish_state);
    
    ok = 0;    
    // ros::shutdown();
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


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
        // 如果没有砖堆信息 就找砖和墙位置，
        // 然后在建筑位置进行global costmap的设置，放置车辆开过砖墙区域

        // 循环三个砖堆， 依次到达砖堆位置

            // 循环取砖
            ros::init(argc, argv, "task2_action");

            ok = 1;
            Client pickclient("pickputAction", true); /* 这里的第一次参数要特别注意，我这里起名为action_demo，这个名称关系到server和client之间的配对通讯，两边代码对于这个名称必须要一致，否则两个节点无法相互通讯。 */
            ROS_INFO("Waiting for pickput action server to start!");
            pickclient.waitForServer();                 /* 等待服务端响应 */
            ROS_INFO("pickput action server started !");
            bit_motion::pickputGoal pick_goal;      /* 创建一个目标 */
            
            uint8_t brick_num = 2;      //0, 1
            int8_t i = brick_num -1;
            while (i >= 0){
                ok = 1;

                pick_goal.goal_brick.type = 255;        /* 设置目标对象的值 */
                pick_goal.goal_brick.Sequence = i;
                pick_goal.task = TASK_GET;
            
                pickclient.sendGoal(pick_goal, &pickdoneCd, &pickactiveCd, &pickfeedbackCb);        /* 发送目标，并且定义回调函数 */
                
                while(ok){  // 等待回调
                    ;
                }
                
                i--;
                ros::Time now = ros::Time().now();
                while( (ros::Time().now().toSec() - now.toSec() ) < 2){
                    
                    ROS_INFO("wating,%d", i);
                }  
            }
            
        // 移动到观察建筑处，调用检测砖堆状态

        // 计算出欲放置砖块相对于目前的位置
        // 移动
        
            ros::Time now = ros::Time().now();
            while( (ros::Time().now().toSec() - now.toSec() ) < 8){
                
                ROS_INFO("wating%d", i);
            }  

        // 循环
            // 移动局部位置，需要使得车辆前进线对准目标位置
            // 放砖
            i = brick_num - 1 ;      // 注意append是从0开始
            while( i >= 0){
                ok = 1;
                pick_goal.goal_brick.type = 255;        /* 设置目标对象的值 */
                pick_goal.goal_brick.Sequence = i;
                pick_goal.task = TASK_BUILD;
                
                pickclient.sendGoal(pick_goal, &pickdoneCd, &pickactiveCd, &pickfeedbackCb);        /* 发送目标，并且定义回调函数 */
                
                while(ok){
                    ;
                }
                i --;
                ros::Time now = ros::Time().now();
                while( (ros::Time().now().toSec() - now.toSec() ) < 2){
                    
                    ROS_INFO("wating,%d", i);
                } 
            }
           

            ros::shutdown();
    ros::spin();
    return 0;

}