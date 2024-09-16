#ifndef FORMATION_COMM_H
#define FORMATION_COMM_H

#include "topic_info.h"
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

/**
 * @brief FormationCOMM 类用于处理多个机器人间的控制通信。
 */
class FormationCOMM {
public:
    ~FormationCOMM();

    FormationCOMM(const FormationCOMM&)=delete;
    FormationCOMM& operator=(const FormationCOMM&)=delete;

    static FormationCOMM& get_instance(ros::NodeHandle& nh, const std::vector<RobotTopic>& topics, int myid, int loop_hz);

    void setTopics(const std::vector<RobotTopic>& topics);
    std::vector<RobotTopic> getTopics(void) const;
    std::unordered_map<int, RobotAttributes> receiveRobotState(void);  // FIXME:这里增加对机器人健康状态的检测
    bool sendVelInfo(const Eigen::Vector2d& vel) const;


private:
    int myid_;
    int loop_hz_;
    ros::NodeHandle nh_;
    size_t agent_num_;
    std::vector<RobotSystem> agents_;    // 机器人信息列表

    // 哈希表，id->callback count
    std::unordered_map<int, int> idCount_map_;  // 每个机器人话题在每个loop内的回调计数
    // 哈希表，id->agents_(RobotSystem)
    std::unordered_map<int, std::reference_wrapper<RobotSystem> > idAgent_map_; // 这里使用引用，方便对agents_的修改
    // 哈希表，id->states(RobotAttributes)，供receiveRobotState使用
    std::unordered_map<int, RobotAttributes> idState_map_;  // 做为与外界交流的类型

    bool checkUniqueTopics(const std::vector<RobotTopic>& topics);
    FormationCOMM(ros::NodeHandle& nh, const std::vector<RobotTopic>& topics, int myid, int loop_hz);
    void agentsPoseCb(const nav_msgs::Odometry::ConstPtr& msg, size_t idx);
    template<typename T>
    bool waitForTopic(const std::string& topic, double timeout = 5.0);
};

#endif