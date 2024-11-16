#ifndef LEADER_H
#define LEADER_H

#include "robot.h"
#include "formation_comm.h"
#define _USE_MATH_DEFINES 
#include <cmath>

enum ControlMode {
    POSITION,
    SPEED
};


class Leader : public Robot {
public:
    ~Leader();
    Leader(const Leader&)=delete;
    Leader& operator=(const Leader&)=delete;

    static Leader& get_instance(RobotAttributes& initial_pose, const int& myid);

    void sinMove(FormationCOMM& comm, int loop_hz);  // 控制Leader按照正弦形状移动
    void straightMove(FormationCOMM& comm);

private:
    int myid_;
    int cnter_;
    double w_fixed_;

    Leader(RobotAttributes& initial_pose, const int& myid);
};

#endif