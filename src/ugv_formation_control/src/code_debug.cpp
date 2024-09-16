#include "code_debug.h"

CodeDebug::CodeDebug() {

}


void CodeDebug::printRobotState(std::unordered_map<int, RobotAttributes> states)
{
    size_t num_agent = states.size();

    std::stringstream ss;
    ss << "time stamp: " << ros::Time::now() << "\n";


// // --- 统一格式
//     size_t num = states.size()-1;

//     Eigen::Vector3d leader_position = states.at(map.at("leader")).state.pos;
//     Eigen::Vector2d leader_velocity = states.at(map.at("leader")).state.vel;
//     std::vector<Eigen::Vector3d> followers_position(num);
//     std::vector<Eigen::Vector2d> followers_velocity(num);
//     followers_position.at(0) = states.at(map.at("follower1")).state.pos;
//     followers_velocity.at(0) = states.at(map.at("follower1")).state.vel;
//     followers_position.at(1) = states.at(map.at("follower2")).state.pos;
//     followers_velocity.at(1) = states.at(map.at("follower2")).state.vel;

//     // for (size_t idx=0; idx<num; ++idx) {
//     //     followers_position.at(idx) = states.at(map.at("follower" + to_string(idx+1)))
//     //     followers_velocity.at(idx) = followers.at(idx).vel;
//     // }

// // --- 输出
//     std::stringstream ss;

//     ros::Time time_now = ros::Time::now();
//     ss << "timeStamp: " << time_now << "\n";
    
    // 打印标题行
    for(size_t i=0; i<num_agent; ++i) {
        ss << std::setw(18) << states[i].info.name;
    }
    ss << "\n";

    // 打印x, y, z信息
    ss << std::fixed << std::setprecision(3);
    ss << "x:";
    for(size_t i=0; i<num_agent; ++i) {
        ss << std::setw(18) << states[i].state.pos[0];
    }
    ss << "\n";

    ss << "y:";
    for(size_t i=0; i<num_agent; ++i) {
        ss << std::setw(18) << states[i].state.pos[1];
    }
    ss << "\n";

    ss << "z:";
    for (size_t i=0; i<num_agent; ++i) {
        ss << std::setw(18) << states[i].state.pos[2];
    }
    ss << "\n";

    // 打印v, w信息
    ss << "v:";
    for (size_t i=0; i<num_agent; ++i) {
        ss << std::setw(18) << states[i].state.vel[0];
    }
    ss << "\n";

    ss << "w:";
    for (size_t i=0; i<num_agent; ++i) {
        ss << std::setw(18) << states[i].state.vel[1];
    }
    ss << "\n";

    // 使用ROS_INFO打印格式化的字符串
    ROS_INFO("\n%s", ss.str().c_str());
}
