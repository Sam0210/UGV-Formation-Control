#include "leader.h"

Leader::Leader(ros::NodeHandle& nh, 
    const TopicName& leader_topics, const std::vector<TopicName>& follower_topics) 
{
    id_ = 0;
    name_ = "leader";

    heading_ = 0.0;
    velocity_ << 0.0, 0.0;
    position_ << 0.0, 0.0, 0.0;
    target_position_ << 0.0, 0.0, 0.0;
    desired_velocity_ << 0.0, 0.0;

    health_status_ = NORMAL;
    formation_config_ = TRIANGLE;

    FormationCOMM& comm = FormationCOMM::get_instance(nh, leader_topics, follower_topics);
}

void Leader::move(const double& v, const double& w) {
    Eigen::Vector2d vel(v, w);
    FormationCOMM::get_instance().sendVelInfo(id_, vel);
    // Implement the movement logic here
}

void Leader::stop() {
    // Implement the stop logic here
}

// bool Leader::updatePosition(double dt) {
//     // Implement the position update logic here
// }

Vector3d Leader::getPosition() const {
    return position_;
}

Vector2d Leader::getVelocity() const {
    return velocity_;
}

HealthStatus Leader::checkHealth() const {
    return health_status_;
}

void Leader::updateFormationConfig(const std::string& new_config) {
    formation_config_ = new_config;
}

void Leader::broadcastFormationInfo() {
    // Implement broadcasting logic here
}

// void Leader::setTargetPoint(Vector3d target) {
//     target_point_ = target;
// }

void Leader::setDesiredVelocity(Vector2d velocity) {
    desired_velocity_ = velocity;
}