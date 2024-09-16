#include "lf_method.h"
#include <cmath>


LFMethod::LFMethod()
{}

LFMethod::LFMethod(const LFParams& params) : params_(params) 
{}


void LFMethod::configure(const LFParams& params)
{
    params_ = params;
}


LFOutputs LFMethod::computeVelocity(const LFInputs& inputs)
{
    double k1     = params_.k1;
    double k2     = params_.k2;
    double lDsr   = params_.lDsr;
    double phiDsr = params_.phiDsr;

    double ex     = inputs.ex;
    double ey     = inputs.ey;
    double etha   = inputs.etha;
    double vl     = inputs.vl;
    double wl     = inputs.wl;

    LFOutputs output;
    output.v = (k1*ex + wl*ey - wl*lDsr*std::sin(phiDsr) + vl)*std::cos(etha) - 
               (-k2*ey + wl*ex - wl*lDsr*std::cos(phiDsr))*std::sin(etha);
    output.w = (-k1*ex - wl*ey - wl*lDsr*std::sin(phiDsr) - vl)*std::sin(etha) - 
               (-k2*ey + wl*ex - wl*lDsr*std::cos(phiDsr))*std::cos(etha);

    return output;
}

