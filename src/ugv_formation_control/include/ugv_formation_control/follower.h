#ifndef FOLLOWER_H
#define FOLLOWER_H

#include "robot.h"
#include "formation_controller.h"
#define _USE_MATH_DEFINES 
#include <cmath>

enum FormationConfig {  
    TRIANGLE,
    LINE
};

struct PoseDsrFmt {
    double lDsr   = 0.0;
    double phiDsr = 0.0;
    double xDsr   = 0.0;
    double yDsr   = 0.0;
};


class Follower : public Robot {
public:  
    ~Follower();
    Follower(const Follower&)=delete;
    Follower& operator=(const Follower&)=delete;
    
    static Follower& get_instance(const ControlParams& params);

    void initFormationConfig(std::vector<RobotAttributes>& initial_poses, 
                const FormationConfig& cfg, const int& my_id, const int& leader_id);
    void formatting(FormationCOMM& comm);
    
    void updateLeaderInfo(const Robot& leader_info);
    void receiveFormationInfo(const std::string& info);
    void computeRelativePosition();
    void setHealth(HealthStatus health_status);


private:
    int myid_;
    int leader_id_;
    PoseDsrFmt pose_dsr_;
    FormationConfig cfg_;
    std::unordered_map<int, RobotAttributes> initialPose_map_;

    FormationController controller;
    // std::unordered_map<int, PoseDsrFmt> poseDsr_map_;

    // bool setTopology(const size_t& num);

    Follower(const ControlParams& params);
};

#endif