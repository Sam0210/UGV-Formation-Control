#include "leader.h"


Leader& Leader::get_instance(RobotAttributes& initial_pose, const int& myid)
{
    static Leader instance(initial_pose, myid);
    return instance;
}


Leader::Leader(RobotAttributes& initial_pose, const int& myid) 
{
    myid_ = myid;
    att_ = initial_pose;

    cnter_ = 0;
    w_fixed_ = M_PI / 6.0;
}


void Leader::sinMove(FormationCOMM& comm, int loop_hz)
{
    ++cnter_;

    if (cnter_ == 10*loop_hz) {
        w_fixed_ = - w_fixed_;
    }
    else if (cnter_ == 30*loop_hz) {
        w_fixed_ = -w_fixed_;
    }
    else if (cnter_ == 40*loop_hz) {
        w_fixed_ = -w_fixed_;
        cnter_ = 0;
    }

    move(comm, 0.3, w_fixed_);
}


void Leader::straightMove(FormationCOMM& comm)
{
    move(comm, 0.3, 0.0);
}


Leader::~Leader()
{
    
}