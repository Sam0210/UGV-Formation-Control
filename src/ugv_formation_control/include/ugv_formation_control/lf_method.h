#ifndef LF_METHOD_H
#define LF_METHOD_H

struct LFParams
{
    double k1     = 0;
    double k2     = 0;
    double lDsr   = 0;
    double phiDsr = 0;
};

struct LFInputs {
    double ex     = 0;
    double ey     = 0;
    double etha   = 0;
    double vl     = 0;
    double wl     = 0;
};

struct LFOutputs
{
    double v = 0;
    double w = 0;
};



class LFMethod {
private:
    LFParams params_;

public:
    virtual ~LFMethod() = default;
    LFMethod();
    LFMethod(const LFParams& params);

    LFOutputs computeVelocity(const LFInputs& inputs);  
    void configure(const LFParams& params);
};


#endif