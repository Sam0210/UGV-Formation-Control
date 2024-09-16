#ifndef APF_METHOD_H
#define APF_METHOD_H

struct APFOutputs {
    double fx = 0;
    double fy = 0;
};

struct APFParams
{
    /* data */
};

struct APFInputs
{
    /* data */
};



class APFMethod
{
private:
    APFParams params_;

public:
    virtual ~APFMethod() = default;
    APFMethod();
    APFMethod(const APFParams& params);

    APFOutputs computeForce(const APFInputs& inputs);
    void configure(const APFParams& params);
};



#endif