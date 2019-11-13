#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <actionlib/client/simple_action_client.h>
#include "bit_motion/pickputAction.h"    /* 这个头文件每个人写的的名字都可能不同，package name/action file name + Action.h */


typedef actionlib::SimpleActionClient<bit_motion::pickputAction> Client;

/*
 *action完成时的回调函数，一次性
 */
void doneCd(const actionlib::SimpleClientGoalState& state, const bit_motion::pickputResultConstPtr& result)
{
    ROS_INFO("DONE:%d", result->finish_state);
    ros::shutdown();
}

/*
 *action启动时的回调函数，一次性
 */
void activeCd()
{
    ROS_INFO("ACTIVE");
}

/*
 *action收到反馈时的回调函数
 */
void feedbackCb(const bit_motion::pickputFeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM("THE STATE NOM IS: "<<  feedback -> move_rightnow);
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
        // 如果没有砖堆信息 就找转 

        // 循环三个砖堆， 依次到达砖堆位置

            // 循环取砖
            ros::init(argc, argv, "task2_action");

            /* 定义一个客户端 */
            Client client("pickputAction", true); /* 这里的第一次参数要特别注意，我这里起名为action_demo，这个名称关系到server和client之间的配对通讯，两边代码对于这个名称必须要一致，否则两个节点无法相互通讯。 */

            /* 等待服务端响应 */
            ROS_INFO("WAITING FOR ACTION SERVER TO START !");
            client.waitForServer();
            ROS_INFO("ACTION SERVER START !");

            /* 创建一个目标对象 */
            bit_motion::pickputGoal demo_goal;
            demo_goal.goal_brick.type = 255;  /* 设置目标对象的值 */

            /* 发送目标，并且定义回调函数 */
            client.sendGoal(demo_goal, &doneCd, &activeCd, &feedbackCb);

        // 移动到观察建筑处， 调用检测砖堆状态

        // 循环放砖， 移动位置

        //

    ros::spin();
    return 0;

}