#include "follower.h"

extern int cnt;

Follower::Follower(const ControlParams& params)
{
    pose_dsr_.lDsr   = params.lf.lDsr;
    pose_dsr_.phiDsr = params.lf.phiDsr;
    pose_dsr_.xDsr   = params.lf.lDsr * std::cos(params.lf.phiDsr);  // 
    pose_dsr_.yDsr   = params.lf.lDsr * std::sin(params.lf.phiDsr);

    ROS_INFO("xDsr:%.2f, yDsr:%.2f", pose_dsr_.xDsr, pose_dsr_.yDsr);

    controller.configure(params);  // 控制器初始化
}


Follower& Follower::get_instance(const ControlParams& params)
{
    static Follower instance(params);
    return instance;
}


void Follower::initFormationConfig(std::vector<RobotAttributes>& initial_poses, 
                    const FormationConfig& cfg, const int& my_id, const int& leader_id)
{
    size_t agent_num = initial_poses.size();
    if((cfg==LINE)&&(agent_num!=2) || (cfg==TRIANGLE)&&(agent_num!=3))
    {
        ROS_ERROR("成员数量与队形不相匹配！");
        return;
    }

    cfg_ = cfg;
    myid_ = my_id;
    leader_id_ = leader_id;

    initialPose_map_.reserve(agent_num);
    for(auto& pose : initial_poses)  // 使用非const引用，便于移动
    {
        initialPose_map_.emplace(pose.info.id, std::move(pose));  // FIXME:可能会存在引用问题
        if(pose.info.id == my_id)
            att_ = pose;
    
        // // 测试使用
        // ROS_INFO("name: %s",              initialPose_map_.at(pose.info.id).info.name.c_str());
        // ROS_INFO("heading: %.2f",         initialPose_map_.at(pose.info.id).state.heading);
        // ROS_INFO("pos: %.2f, %.2f, %.2f", initialPose_map_.at(pose.info.id).state.pos[0], 
        //                                   initialPose_map_.at(pose.info.id).state.pos[1],
        //                                   initialPose_map_.at(pose.info.id).state.pos[2]);
        // ROS_INFO("vel: %.2f, %.2f",       initialPose_map_.at(pose.info.id).state.vel[0],
        //                                   initialPose_map_.at(pose.info.id).state.vel[1]);
        // if (initialPose_map_.at(pose.info.id).state.health == NORMAL)
        //     ROS_INFO("health: NORMAL");
        // else if (initialPose_map_.at(pose.info.id).state.health == WARNING)
        //     ROS_INFO("health: WARNING");
        // else if (initialPose_map_.at(pose.info.id).state.health == ERROR)
        //     ROS_INFO("health: ERROR");
        // else
        //     ROS_INFO("health: DONT KNOW");
    }

    // Follower::setTopology(agent_num - 1);
}


void Follower::formatting(FormationCOMM& comm)
{
    std::unordered_map<int, RobotAttributes> odom_pose = comm.receiveRobotState();
    if ((odom_pose.at(leader_id_).state.credibility < 0.2) || (odom_pose.at(myid_).state.credibility < 0.2)) {
        ROS_INFO("不可信的里程计消息，已忽略");
        return;
    }

    att_ = odom_pose.at(myid_);  // 更新自己的状态

    // 额外的：机器人位置需要从里程计坐标系转到论文中的坐标系
    // funny, 初始方向角ｏｄｏｍ是知道的
    // 论文中与Gazebo中的x和y值是相反的
    // 横轴也是反的
    // TODO:测试角度.里程计坐标系的正ｙ轴为0，负ｙ轴为-3.14，以弧度为单位
    // FIXME: ｐｈｉｌ，ｌｘ，ｌｙ第一次输出有问题，已经保存到图片(订阅回调还未开始)
    double yl   =   odom_pose.at(leader_id_).state.pos[0] + initialPose_map_.at(leader_id_).state.pos[0];
    double xl   = -(odom_pose.at(leader_id_).state.pos[1] + initialPose_map_.at(leader_id_).state.pos[1]);
    double yf   =   odom_pose.at(myid_).state.pos[0] + initialPose_map_.at(myid_).state.pos[0];
    double xf   = -(odom_pose.at(myid_).state.pos[1] + initialPose_map_.at(myid_).state.pos[1]);
    double thal =   odom_pose.at(leader_id_).state.heading + M_PI/2;
    double thaf =   odom_pose.at(myid_).state.heading + M_PI/2;

    double lx = -(xl - xf)*std::cos(thal) - (yl - yf)*std::sin(thal);
    double ly =  (xl - xf)*std::sin(thal) - (yl - yf)*std::cos(thal);

    ControlInputs inputs;
    inputs.lf.vl   = odom_pose.at(leader_id_).state.vel[0];
    inputs.lf.wl   = odom_pose.at(leader_id_).state.vel[1];
    inputs.lf.ex   = pose_dsr_.xDsr - lx;
    inputs.lf.ey   = pose_dsr_.yDsr - ly;
    inputs.lf.etha = thaf - thal;

    #ifdef CONTROLLER_INPUT_TEST
        if (cnt%10==0) {
            ROS_INFO("*******************************");
            ROS_INFO("Odometry:");
            ROS_INFO("l_x: %.2f, l_y: %.2f, l_tha: %.2f", odom_pose.at(leader_id_).state.pos[0], 
                                                          odom_pose.at(leader_id_).state.pos[1],
                                                          odom_pose.at(leader_id_).state.heading);
            ROS_INFO("my_x: %.2f, my_y: %.2f, my_tha: %.2f", odom_pose.at(myid_).state.pos[0], 
                                                             odom_pose.at(myid_).state.pos[1],
                                                             odom_pose.at(myid_).state.heading);
            
            ROS_INFO("Credibility:");
            ROS_INFO("l_odom: %.2f, f1_odom: %.2f, f2_odom: %.2f", odom_pose.at(0).state.credibility,
                                                                   odom_pose.at(1).state.credibility,
                                                                   odom_pose.at(2).state.credibility);

            ROS_INFO("---Real Poses:");
            ROS_INFO("l_x: %.2f, l_y: %.2f, l_tha: %.2f", xl, yl, thal);
            ROS_INFO("my_x: %.2f, my_y: %.2f, my_tha: %.2f", xf, yf, thaf);

            ROS_INFO("---在Leader系下我的坐标:");
            ROS_INFO("x: %.2f, y: %.2f", lx, ly);

            ROS_INFO("---控制器输入:");
            ROS_INFO("vl: %.2f, wl: %.2f, ex: %.2f, ey: %.2f, etha: %.2f", inputs.lf.vl, 
                                                                           inputs.lf.wl, 
                                                                           inputs.lf.ex, 
                                                                           inputs.lf.ey,
                                                                           inputs.lf.etha);
        }
        ++cnt;
        if (cnt > 0xFFFFFF)
            cnt = 0;
    #endif

    ControlOutputs outputs = controller.controller(inputs);
    move(comm, outputs.v, outputs.w);
}


// bool Follower::setTopology(const size_t& num)
// {
//     std::vector<double> l_dsr;
//     std::vector<double> phi_dsr;
//     switch(num)
//     {
//         case 1: 
//             l_dsr.push_back(1);
//             phi_dsr.push_back( (180/180)*M_PI );
//             break;

//         case 2:
//             l_dsr.push_back(1);
//             l_dsr.push_back(1);
//             phi_dsr.push_back( (-135/180)*M_PI );
//             phi_dsr.push_back( (135/180)*M_PI );
//             break;
        
//         default:
//             ROS_ERROR("未预定义%d个机器人的编队结构", static_cast<int>(num));
//             return false;
//     }

//     // 将上述数组形式格式化处理为哈希表形式
//     int cnt = 0;
//     poseDsr_map_.reserve(num);
//     for(const auto& pose : initialPose_map_) 
//     {
//         if(pose.first == leader_id_)
//             continue;
        
//         double l_dsr_value = l_dsr[cnt];
//         double phi_dsr_value = phi_dsr[cnt];

//         PoseDsrFmt pose_dsr;
//         pose_dsr.lDsr = l_dsr_value;
//         pose_dsr.phiDsr = phi_dsr_value;
//         pose_dsr.xDsr = l_dsr_value * std::cos(phi_dsr_value);
//         pose_dsr.yDsr = l_dsr_value * std::sin(phi_dsr_value);

//         poseDsr_map_.emplace(pose.first, pose_dsr);

//         ++cnt;
//     }

//     return true;
// }


Follower::~Follower()
{
    
}