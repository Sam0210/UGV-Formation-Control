#ifndef LEADER_H
#define LEADER_H

#include "robot.h"
#include "formation_comm.h"

enum ControlMode {
    POSITION,
    SPEED
};

class Leader : public Robot {
public:
    ~Leader();
    Leader(const Leader&)=delete;
    Leader& operator=(const Leader&)=delete;

    static Leader& get_instance();

    Leader(ros::NodeHandle& nh, 
        const TopicName& leader_topics, const std::vector<TopicName>& follower_topics);


    // bool updatePosition(double dt) override;
    Vector3d getPosition() const override;
    Vector2d getVelocity() const override;
    HealthStatus checkHealth() const override;

    void updateFormationConfig(const std::string& new_config);
    void broadcastFormationInfo();
    void setTargetPosition(Vector3d target);
    void setDesiredVelocity(Vector2d velocity);

private:
    int my_id_;


    Leader();
};

#endif