#include "robot.h"


void Robot::move(FormationCOMM& comm, const double& v, const double& w) {
    Eigen::Vector2d vel(v, w);
    comm.sendVelInfo(vel);
}

void Robot::stop(FormationCOMM& comm) {
    Eigen::Vector2d vel(0.0, 0.0);
    comm.sendVelInfo(vel);
}


RobotAttributes Robot::attributes(void) const {
    return att_;
}

int Robot::id() const {
    return att_.info.id;
}

std::string Robot::name() const {
    return att_.info.name;
}

Eigen::Vector3d Robot::position() const {
    return att_.state.pos;
}

Eigen::Vector2d Robot::velocity() const {
    return att_.state.vel;
}

double Robot::heading() const {
    return att_.state.heading;
}

HealthStatus Robot::health() const {
    return att_.state.health;
}
