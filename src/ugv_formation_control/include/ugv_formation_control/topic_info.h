#ifndef TOPIC_INFO_H
#define TOPIC_INFO_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>

/*
 * 机器人健康状态
 * NORMAL:无异常
 * WARNING:轻微故障
 * ERROR:严重故障
*/
enum HealthStatus {
    NORMAL,
    WARNING,
    ERROR
};

/*
 * 机器人属性
 * id:唯一标识
 * name:自定义名称
*/
struct RobotInfo {
    int id = 0;
    std::string name = "";
};

/*
 * RobotTopic                    // 机器人话题
 * ├── info (RobotInfo)          // 机器人属性
 * │   ├── id (int)              // ID
 * │   └── name (std::string)    // 名称
 * └── name (TopicName)          // 话题名称
 *     ├── pos (Eigen::Vector3d) // 位置
 *     └── vel (Eigen::Vector2d) // 速度
*/
struct RobotTopic {
    RobotInfo info;
    struct TopicName {
        std::string pos = "";
        std::string vel = "";
    } name;
};

/*
 * RobotAttributes                // 机器人属性
 * ├── RobotInfo 　               // 基本信息
 * │   ├── id   (int)             // ID
 * │   └── name (std::string)     // 名称
 * └── RobotState 　              // 状态信息
 *     ├── credibility (double)   // 可信度
 *     ├── heading (double)       // 方向角
 *     ├── pos (Eigen::Vector3d)  // 位置
 *     ├── vel (Eigen::Vector2d)  // 速度
 *     └── health (HealthStatus)  // 健康
*/
struct RobotAttributes {
    RobotInfo info;
    struct RobotState {
        double credibility = 0;
        double heading = 0;
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector2d vel = Eigen::Vector2d::Zero();
        HealthStatus health = NORMAL;
    } state;
};

/*
 * RobotSystem
 * ├── RobotAttributes                // 机器人属性
 * │   ├── RobotInfo                  // 基本信息
 * │   │   ├── id   (int)             // ID
 * │   │   └── name (std::string)     // 名称
 * │   └── RobotState                 // 状态信息
 * │       ├── credibility (double)   // 可信度
 * │       ├── heading (double)       // 方向角
 * │       ├── pos (Eigen::Vector3d)  // 位置
 * │       ├── vel (Eigen::Vector2d)  // 速度
 * |       └── health (HealthStatus)  // 健康
 * ├── VelocityTopic                  // 速度话题
 * │   ├── name  (std::string)        // 名称
 * │   └── puber (ros::Publisher)     // 发布者
 * └── PoseTopic                      // 位姿话题
 *     ├── name  (std::string)        // 名称
 *     └── suber (ros::Subscriber)    // 订阅者 
*/
struct RobotSystem {
    RobotAttributes attributes;  // 机器人属性
    struct VelocityTopic {
        std::string name = "";
        ros::Publisher puber = ros::Publisher();
    } vel_topic;
    struct PoseTopic {
        std::string name = "";
        ros::Subscriber suber = ros::Subscriber();
    } pose_topic;
};

#endif 
