#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <bit_motion/fightfireAction.h>

typedef actionlib::SimpleActionClient<bit_motion::fightfireAction> Client_fightfire;

class FightFireActionClient    // 找砖action客户端
{
    public:
        FightFireActionClient(const std::string client_name, bool flag = true):client(client_name, flag) // 使用初始化列表来初始化client
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

        void sendGoal(bit_motion::fightfireGoal goal, double Timeout = 10.0)
        {
            // 发送目标至服务器
            client.sendGoal(goal,
                            boost::bind(&FightFireActionClient::done_cb, this, _1, _2),
                            boost::bind(&FightFireActionClient::active_cb, this),
                            boost::bind(&FightFireActionClient::feedback_cb, this, _1));
            
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
        Client_fightfire client;
        std::string ClientName;      // action名称

    private:
        /* action完成时的回调函数，一次性  */
        void done_cb(const actionlib::SimpleClientGoalState& state, const bit_motion::fightfireResultConstPtr& result)
        {
            ROS_INFO("DONE:%d", result->finish_state);
        }
        /* action启动时的回调函数，一次性  */
        void active_cb()
        {
            ROS_INFO("ACTIVE");
        }
        /* action收到反馈时的回调函数 */
        void feedback_cb(const bit_motion::fightfireFeedbackConstPtr& feedback)
        {
            ROS_INFO_STREAM("THE STATE NOM IS: "<<feedback->feedback_state);
        }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task3_node");
    ros::NodeHandle nh;

    bit_motion::fightfireGoal fightfire_goal;   

    /*****************************************************
    *       连接动作服务器
    *****************************************************/
    static FightFireActionClient Task3Client("FightFireAction", true);   // 连接灭火动作服务器
    Task3Client.Start();


    /*****************************************************
    *       第一步 获取建筑物位置信息
    *****************************************************/


    /*****************************************************
    *       第二步 找到门口位置
    *****************************************************/


    /*****************************************************
    *       第三步 SLAM遍历室内寻找火源
    *****************************************************/
    
    /*****************************************************
    *       第四步 机械臂灭火
    *****************************************************/
    // 将砖块搬运至车上
    Task3Client.sendGoal(fightfire_goal, 100);

                

    return 0;
}