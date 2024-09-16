#include "apf_method.h"


APFMethod::APFMethod()
{}

APFMethod::APFMethod(const APFParams& params) : params_(params) 
{}


void APFMethod::configure(const APFParams& params)
{
    params_ = params;
}


APFOutputs APFMethod::computeForce(const APFInputs& inputs)
{
    APFOutputs outputs;

    return outputs;
}
