#ifndef TOPIC_INFO_H
#define TOPIC_INFO_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>


// #define PRINT_ROBOT_STATE      // main.cpp　机器人实时状态
#define CONTROLLER_INPUT_TEST  // follower.cpp 控制器输入量检测


// 这里最好是创建两个大的结构体，分别用来管理formation_comm和follower，这样看起来也比较清晰
// 将其中一些通用的结构体放在最上面
//　理清各个cpp函数需要哪些必要信息

// struct RobotInfo {
//     int id;  // 机器人ID
//     std::string name;  // 机器人名称
//     // 机器人的位置和速度

//     // 速度话题结构体
//     struct VelocityTopic {
//         std::string topicName;  // 话题名称
//         ros::Publisher publisher;  // 话题发布者
//     } velocityTopic;  // 速度话题

//     // 位姿话题结构体
//     struct PoseTopic {
//         std::string topicName;  // 话题名称
//         ros::Subscriber subscriber;  // 话题订阅者
//      } poseTopic;  // 位姿话题
// }




// /**
//  * @brief 结构体 TopicName，用于存储话题的名称。
//  */
// struct TopicName {
//     std::string pos;  ///< 机器人位姿的主题名称
//     std::string vel;  ///< 机器人速度的主题名称
// };


// /**
//  * @brief 结构体 COMMode，用于存储话题的通讯方式（发布者和订阅者）。
//  */
// struct COMMode {  
//     ros::Publisher pub;   ///< 发布者对象
//     ros::Subscriber sub;  ///< 订阅者对象
// };


// /**
//  * @brief 结构体 TopicInfo，用于存储话题的名称和通讯方式。
//  */
// struct TopicInfo {
//     TopicName name;  ///< 话题名称
//     COMMode mode;    ///< 话题的通讯方式
// };


// /**
//  * @brief 结构体 RobotState，用于存储机器人的位置和速度。
//  */
// struct RobotState {
//     Eigen::Vector3d pos;  ///< 机器人的三维位置 (x, y, z)   //TODO:结构体成员不能直接在声明时进行初始化
//     Eigen::Vector2d vel;  ///< 机器人的速度，包含线速度和角速度

//     RobotState() 
//         : pos(0,0,0), vel(0,0) {}
// };


// /**
//  * @brief 结构体 RobotInfo，用于存储机器人的状态和话题信息。
//  */
// // RobotInfo
// // │
// // ├── RobotState
// // │   ├── pos  (Eigen::Vector3d)
// // │   └── vel  (Eigen::Vector2d)
// // │
// // └── TopicInfo
// //     ├── TopicName
// //     │   ├── pos  (std::string)
// //     │   └── vel  (std::string)
// //     │
// //     └── COMMode
// //         ├── pub  (ros::Publisher)
// //         └── sub  (ros::Subscriber)
// //　--------
// // RobotProfile
// // │
// // │── RobotInfo
// // |   ├── id     (int)              // 机器人ID
// // |   └── name  (std::string)       // 机器人名称
// // │
// // ├── RobotState
// // │   ├── heading  (Eigen::Vector3d)   // 机器人方向角信息
// // │   ├── pos      (Eigen::Vector3d)   // 机器人位置信息
// // │   └── vel      (Eigen::Vector2d)   // 机器人速度信息
// // │
// // ├── VelocityTopic
// // │   ├── name   (std::string)    // 速度话题名称
// // │   └── puber  (ros::Publisher) // 速度话题发布者
// // │
// // └── PoseTopic
// //     ├── name   (std::string)     // 位姿话题名称
// //     └── suber  (ros::Subscriber) // 位姿话题订阅者
// // ------
// // TopicName
// // ├── pos  (std::string)   // 速度话题名称
// // └── vel  (std::string)   // 位姿话题名称

// struct RobotInfo {
//     RobotState state;  ///< 机器人的状态，包括位置和速度
//     TopicInfo topic;   ///< 机器人的话题信息，包括名称和通讯方式
// };


// 机器人健康状态，避免和robot.h相互包含，妥协之后写在这里
enum HealthStatus {
    NORMAL,
    WARNING,
    ERROR
};


// 机器人属性
struct RobotInfo {
    int id = 0;
    std::string name = "";
};


// RobotTopic  // 机器人话题
// ├── info (RobotInfo)         // 机器人属性
// │   ├── id (int)             // ID
// │   └── name (std::string)   // 名称
// │
// └── name (TopicName)         // 话题名称
//     ├── pos (Eigen::Vector3d) // 位置
//     └── vel (Eigen::Vector2d) // 速度
struct RobotTopic {
    RobotInfo info;

    struct TopicName {
        std::string pos = "";
        std::string vel = "";
    } name;
};


// RobotAttributes // 机器人属性
// │
// ├── RobotInfo 　// 基本信息
// │   ├── id   (int)             // ID
// │   └── name (std::string)     // 名称
// │
// ├── RobotState 　// 状态信息
// │   ├── credibility (double)   // 可信度
// │   ├── heading (double)       // 方向角
// │   ├── pos (Eigen::Vector3d)  // 位置
// │   ├── vel (Eigen::Vector2d)  // 速度
// |   └── health (HealthStatus)  // 健康
struct RobotAttributes 
{
    RobotInfo info;

    struct RobotState {
        double credibility = 0;
        double heading = 0;
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector2d vel = Eigen::Vector2d::Zero();
        HealthStatus health = NORMAL;
    } state;
};


// RobotSystem
// │
// ├── RobotAttributes   // 机器人属性
// │   │
// │   ├── RobotInfo     // 基本信息
// │   │   ├── id   (int)             // ID
// │   │   └── name (std::string)     // 名称
// │   │
// │   └── RobotState    // 状态信息
// │       ├── credibility (double)   // 可信度
// │       ├── heading (double)       // 方向角
// │       ├── pos (Eigen::Vector3d)  // 位置
// │       ├── vel (Eigen::Vector2d)  // 速度
// |       └── health (HealthStatus)  // 健康
// │
// ├── VelocityTopic     // 速度话题
// │   ├── name  (std::string)        // 名称
// │   └── puber (ros::Publisher)     // 发布者
// │
// └── PoseTopic         // 位姿话题
//     ├── name  (std::string)        // 名称
//     └── suber (ros::Subscriber)    // 订阅者 
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
