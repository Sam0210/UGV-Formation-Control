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

/* 处理多个机器人间的控制通信 */
class FormationCOMM {
public:
    ~FormationCOMM();
    FormationCOMM(const FormationCOMM&)=delete;
    FormationCOMM& operator=(const FormationCOMM&)=delete;
    static FormationCOMM& get_instance(ros::NodeHandle& handle, 
                const std::vector<RobotTopic>& topics, int my_id, int loop_hz);
    std::unordered_map<int, RobotAttributes> receiveRobotState(void);
    bool sendVelInfo(const Eigen::Vector2d& vel) const;

private:
    int id;                                                 /* 本智能体id */
    int hz;                                                 /* 通信频率 */
    size_t num;                                             /* 编队成员数量 */
    ros::NodeHandle nh;                                     /* 通信节点句柄 */                      
    std::unordered_map<int, int> cnts;                      /* 记录在一次控制周期内的循环数 */
    std::unordered_map<int, RobotSystem> agents;            /* 信息表　*/
    std::unordered_map<int, RobotAttributes> attrs;         /* 属性表 */

    FormationCOMM(ros::NodeHandle& handle, const std::vector<RobotTopic>& topics,
                  int my_id, int loop_hz);
    bool checkUniqueTopics(const std::vector<RobotTopic>& topics);
    void agentsPoseCb(const nav_msgs::Odometry::ConstPtr& msg, size_t idx);
    template<typename T>
        bool waitForTopic(const std::string& topic, double timeout = 5.0);
};

#endif