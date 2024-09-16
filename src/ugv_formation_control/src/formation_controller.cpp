#include "formation_controller.h"


FormationController::FormationController()
{}

FormationController::FormationController(const ControlParams& params) : 
                LFMethod(params.lf), APFMethod(params.apf), params_(params)
{}


void FormationController::configure(const ControlParams& params) 
{
    params_ = params;
    LFMethod::configure(params.lf);    // 使用namespace方式调用父类重名函数
    APFMethod::configure(params.apf);
}


ControlOutputs FormationController::controller(const ControlInputs& inputs)
{
    LFOutputs output_lf = computeVelocity(inputs.lf);
    // APFOutputs output_apf = computeForce(inputs.apf);

    // TODO: force to velocity

    ControlOutputs outputs;
    outputs.v = output_lf.v;
    outputs.w = output_lf.w;

    return outputs;
}

FormationController::~FormationController() 
{

}