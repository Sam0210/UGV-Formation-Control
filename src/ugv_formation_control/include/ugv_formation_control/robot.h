#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include "formation_comm.h"

class Robot {  // TODO:只允许有一个实例
public:
    virtual ~Robot() = default;

    // 机器人控制
    void move(FormationCOMM& comm, const double& v, const double& w);
    void stop(FormationCOMM& comm);

    // 访问属性
    RobotAttributes attributes(void) const;
    int id() const;
    std::string name() const;
    Eigen::Vector3d position() const;
    Eigen::Vector2d velocity() const;
    double heading() const;
    HealthStatus health() const;


protected:
    RobotAttributes att_;
};

#endif