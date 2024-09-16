#include "formation_comm.h"
#include "code_debug.h"
#include "follower.h"

int cnt = 0; // TEST

using namespace std;
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    // 输入格式检测　与　节点重命名 
    if(argc != 2) {
        ROS_ERROR("Usage: rosrun ugv_formation_control follower_controller <follower_id>");
        return 1;
    }
    string id = argv[1];
    string node_name = "follower" + id + "_controller";

    // 定义智能体基本信息与话题名称　　　// FIXME:修改为从服务器获取
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
    ctrl_params.lf.k1     = 0.4;
    ctrl_params.lf.k2     = 0.4;
    ctrl_params.lf.lDsr   = 2;
    ctrl_params.lf.phiDsr = (135/180.0)*M_PI;


    // ros初始化
    ros::init(argc, argv, node_name);  // TODO: 删掉多余argv参数
    ros::NodeHandle nh;
    ros::Rate rate_10hz(10);

    ROS_INFO("节点名称: %s", node_name.c_str());  // TODO: test line


    FormationCOMM& comm = FormationCOMM::get_instance(nh, agent_topics, 1, 10);

    Follower& follower = Follower::get_instance(ctrl_params);
    follower.initFormationConfig(initial_pose, TRIANGLE, 1, 0);  // TODO:使用ｍａｉｎ传输进来的ｉｄ

    CodeDebug debuger;

    #ifdef PRINT_ROBOT_STATE 
        int cnt = 0;
    #endif

    while(ros::ok())
    {
        #ifdef PRINT_ROBOT_STATE
            std::unordered_map<int, RobotAttributes> states = comm.receiveRobotState();
            
            if (cnt%10==0) {
                debuger.printRobotState(states);
            }
            // ROS_INFO("cnt=%d",cnt);  // TODO:时序测试
            ++cnt;
        #endif

        follower.formatting(comm);

        ros::spinOnce();  // 确保回调函数被调用
        rate_10hz.sleep();
    }

    // ROS_INFO("follower formation controller has been started!");
    return 0;
}