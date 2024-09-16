#ifndef FORMATION_CONTROLLER_H
#define FORMATION_CONTROLLER_H

#include "topic_info.h"
#include "lf_method.h"
#include "apf_method.h"
#include <unordered_map>

struct ControlInputs {
    LFInputs lf;
    APFInputs apf;
};

struct ControlParams {
    LFParams lf;
    APFParams apf;
};

struct ControlOutputs {
    double v = 0;
    double w = 0;
};


class FormationController : private LFMethod, private APFMethod
{
private:
    ControlParams params_;

public:
    FormationController();
    FormationController(const ControlParams& params);
    ~FormationController();

    void configure(const ControlParams& params);
    ControlOutputs controller(const ControlInputs& input);
};


#endif