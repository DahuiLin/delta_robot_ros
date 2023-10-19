#ifndef DELTA_MODEL_HPP
#define DELTA_MODEL_HPP


#define ALPHA1 3/2 * 3.141516
#define ALPHA2 1/6 * 3.141516
#define ALPHA3 5/6 * 3.141516


#include <iostream>
#include <cfloat>
#include <cstring>
#include <cmath>
#include <memory>
#include <Eigen/Eigen>


struct delta_state
{
    double q[3], dq[3], pos[3], dpos[3];
};

class delta_model
{
private:
    double L_arm_, l_forearm_, sb_, sp_;
    double limitx_[2], limity_[2], limitz_[2];
    double freq_;
    double wb_, wp_, ub_, up_;

    std::shared_ptr<delta_state> state_;
    std::shared_ptr<Eigen::Matrix<double,3,3>> JacMatrix_;
    
    bool DK();
    bool IK();
    bool JacobianCal(); 

public:
    delta_model(double L_arm, double l_forearm, double sb, double sp, double freq);
    ~delta_model();

    bool setJointState(double* q, double* dq);
    delta_state getDeltaState() {return *state_;}
    Eigen::Matrix<double,3,3> getJacobian() {return *JacMatrix_;};
};


#endif // DELTA_MODEL_HPP