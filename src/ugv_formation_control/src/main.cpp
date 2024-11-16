#include <cstdlib>
#include <ros/assert.h>
#include "formation_comm.h"
#include "code_debug.h"
#include "leader.h"
#include "follower.h"

int cnt = 0; // TEST

using namespace std;
int main(int argc, char *argv[])
{
    // 将程序的所有本地化设置更改为由环境变量指定的值，解决中文的显示问题(但是，存在多线程不安全问题)
    setlocale(LC_ALL, "");  

    // 输入参数的有效性检查　
    ROS_ASSERT_MSG(argc == 3, "使用方式: rosrun ugv_formation_control controller <agent_name> <agent_id>");
    string my_name = argv[1];
    int my_id = std::atoi(argv[2]);
    ROS_ASSERT_MSG(my_id >= 0 && my_id <= 10, "agent_id 必须在0-10之间, 但你设置为: %d", my_id);
    
    // 定义节点名称
    string node_name = my_name + "_controller";  
    ROS_INFO("节点' %s '已经启动", node_name.c_str());

    // 定义智能体基本信息与话题名称
    int leader_id = 0;
    size_t num_agent = 3;
    std::vector<RobotTopic> agent_topics(num_agent);
    agent_topics.at(0).info.id = 0;
    agent_topics.at(0).info.name = "leader";
    agent_topics.at(0).name.pos = "/leader/odometry/local_filtered";
    agent_topics.at(0).name.vel = "/leader/jackal_velocity_controller/cmd_vel";
    agent_topics.at(1).info.id = 1;
    agent_topics.at(1).info.name = "follower1";
    agent_topics.at(1).name.pos = "/follower1/odometry/local_filtered";
    agent_topics.at(1).name.vel = "/follower1/jackal_velocity_controller/cmd_vel";
    agent_topics.at(2).info.id = 2;
    agent_topics.at(2).info.name = "follower2";
    agent_topics.at(2).name.pos = "/follower2/odometry/local_filtered";
    agent_topics.at(2).name.vel = "/follower2/jackal_velocity_controller/cmd_vel";

    // 定义智能体基本属性
    std::vector<RobotAttributes> initial_pose(num_agent);
    initial_pose.at(0).info.id = 0;
    initial_pose.at(0).info.name = "leader";
    initial_pose.at(0).state.heading = 0;
    initial_pose.at(0).state.pos << 2, 0, 0;
    initial_pose.at(0).state.vel << 0, 0;
    initial_pose.at(0).state.health = NORMAL;
    initial_pose.at(1).info.id = 1;
    initial_pose.at(1).info.name = "follower1";
    initial_pose.at(1).state.heading = 0;
    initial_pose.at(1).state.pos << 0, 1, 0;
    initial_pose.at(1).state.vel << 0, 0;
    initial_pose.at(1).state.health = NORMAL;
    initial_pose.at(2).info.id = 2;
    initial_pose.at(2).info.name = "follower2";
    initial_pose.at(2).state.heading = 0;
    initial_pose.at(2).state.pos << 0, -1, 0;
    initial_pose.at(2).state.vel << 0, 0;
    initial_pose.at(2).state.health = NORMAL;

    ControlParams ctrl_params;
    ctrl_params.lf.k1     = 1.5; 
    ctrl_params.lf.k2     = 1.5;
    ctrl_params.lf.lDsr   = 2.0;
    if (my_id == 1)
        ctrl_params.lf.phiDsr = (135 / 180.0) * M_PI;
    else 
        ctrl_params.lf.phiDsr = (-135 / 180.0) * M_PI;

    int loop_hz = 10;
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Rate rate_10hz(loop_hz);

    FormationCOMM& comm = FormationCOMM::get_instance(nh, agent_topics, my_id, loop_hz);
    if (my_name == "leader") {
        Leader& leader = Leader::get_instance(initial_pose.at(my_id), my_id);

        while(ros::ok()) {
            // leader.sinMove(comm, loop_hz);
            leader.straightMove(comm);
            ros::spinOnce();
            rate_10hz.sleep();
        }
    }
    else {
        Follower& follower = Follower::get_instance(ctrl_params);
        follower.initFormationConfig(initial_pose, TRIANGLE, my_id, leader_id);

        while(ros::ok()) {
            follower.formatting(comm);
            ros::spinOnce();
            rate_10hz.sleep();
        }
    }

    return 0;
}
    // CodeDebug debuger;

    // #ifdef PRINT_ROBOT_STATE 
    //     int cnt = 0;
    // #endif
    // 
    // while(ros::ok())
    // {
    //     #ifdef PRINT_ROBOT_STATE
    //         std::unordered_map<int, RobotAttributes> states = comm.receiveRobotState();
            
    //         if (cnt%10==0) {
    //             debuger.printRobotState(states);
    //         }
    //         // ROS_INFO("cnt=%d",cnt);  // TODO:时序测试
    //         ++cnt;
    //     #endif

    //     follower.formatting(comm);

    //     ros::spinOnce();  // 确保回调函数被调用
    //     rate_10hz.sleep();
    // }

    // return 0;
// }