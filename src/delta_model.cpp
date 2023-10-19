#include "delta_robot/delta_model.hpp"

delta_model::delta_model(double L_arm, double l_forearm, double sb, double sp, double freq): L_arm_(L_arm), l_forearm_(l_forearm), sb_(sb), sp_(sp), freq_(freq){
    
    limitx_[0] = -DBL_MAX;
    limitx_[1] = DBL_MAX;
    limity_[0] = -DBL_MAX;
    limity_[1] = DBL_MAX;
    limitz_[0] = -DBL_MAX;
    limitz_[1] = DBL_MAX;

    std::memset(state_->q, 0, 3);
    std::memset(state_->dq, 0, 3);
    std::memset(state_->dpos, 0, 3);
    
    wb_ = sqrt(3.0)/6.0 * sb_;
    wp_ = sqrt(3.0)/6.0 * sp_;
    ub_ = sqrt(3.0)/3.0 * sb_;
    up_ = sqrt(3.0)/3.0 * sp_;

    state_ = std::make_shared<delta_state>();
    JacMatrix_ = std::make_shared<Eigen::Matrix<double,3,3>>();

    JacMatrix_->setZero();

    if (!DK())
    {
        std::cout << "ERROR DIRECT KINEMATIC";
    }
    
    if (!JacobianCal())
    {
        std::cout << "ERROR JACOBIAN";
    }

}

bool delta_model::DK(){
    double x1 = 0.0;
    double y1 = -wb_-L_arm_*cos(state_->q[0]) + up_;
    double z1 = -L_arm_*sin(state_->q[0]);

    double x2 = (sqrt(3.0)/2.0) * (wb_+L_arm_*cos(state_->q[1])) - sp_/2.0;
    double y2 = 0.5 * (wb_+L_arm_*cos(state_->q[1])-wp_);
    double z2 = -L_arm_*sin(state_->q[1]);

    double x3 = (sqrt(3.0)/2.0) * (wb_+L_arm_*cos(state_->q[2])) + sp_/2.0;
    double y3 = 0.5 * (wb_+L_arm_*cos(state_->q[2])-wp_);
    double z3 = -L_arm_*sin(state_->q[2]);

    double dnm = (y2-y1)*x3-(y3-y1)*x2;
    double w1 = y1 * y1 + z1 * z1;
    double w2 = x2 * x2 + y2 * y2 + z2 * z2;
    double w3 = x3 * x3 + y3 * y3 + z3 * z3;

    double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
    double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

    double a2 = -(z2-z1)*x3+(z3-z1)*x2;
    double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

    double a = a1 * a1 + a2 * a2 + dnm * dnm;
    double b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
    double c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - l_forearm_ * l_forearm_);

    double d = b * b - 4.0 * a * c;

    if (d < 0.0)
    {
        return false;
    }

    state_->pos[2] = -0.5* (b+sqrt(d))/a;
    state_->pos[0] = (a1 * state_->pos[2] + b1) / dnm;
    state_->pos[1] = (a2 * state_->pos[2] + b2) / dnm;
    
    return true;
}

bool delta_model::IK(){
    
}

bool delta_model::JacobianCal(){
    double c1x = state_->pos[0]*cos(state_->q[0]) + state_->pos[1]*sin(state_->q[0])-(wb_-up_);
    double c2x = state_->pos[0]*cos(state_->q[1]) + state_->pos[1]*sin(state_->q[1])-(wb_-up_);
    double c3x = state_->pos[0]*cos(state_->q[2]) + state_->pos[1]*sin(state_->q[2])-(wb_-up_);

    double c1y = -state_->pos[0]*sin(state_->q[0])+state_->pos[1]*cos(state_->q[0]);
    double c2y = -state_->pos[0]*sin(state_->q[1])+state_->pos[1]*cos(state_->q[1]);
    double c3y = -state_->pos[0]*sin(state_->q[2])+state_->pos[1]*cos(state_->q[2]);
    
    double q13 = acos(c1y/l_forearm_);
    double q23 = acos(c2y/l_forearm_);
    double q33 = acos(c3y/l_forearm_);

    double q12 = acos((c1x*c1x+c1y*c1y+state_->pos[2]*state_->pos[2]-L_arm_*L_arm_-(l_forearm_*l_forearm_))/(2.0*L_arm_*l_forearm_*sin(q13)));
    double q22 = acos((c2x*c2x+c2y*c2y+state_->pos[2]*state_->pos[2]-L_arm_*L_arm_-(l_forearm_*l_forearm_))/(2.0*L_arm_*l_forearm_*sin(q23)));
    double q32 = acos((c3x*c3x+c3y*c3y+state_->pos[2]*state_->pos[2]-L_arm_*L_arm_-(l_forearm_*l_forearm_))/(2.0*L_arm_*l_forearm_*sin(q33)));

    double a11,a21,a31,a12,a22,a32,a13,a23,a33;

    a11 = sin(q13)*cos(state_->q[0]+q12)*cos(ALPHA1)-cos(q13)*sin(ALPHA1);
    a21 = sin(q23)*cos(state_->q[1]+q22)*cos(ALPHA2)-cos(q23)*sin(ALPHA2);
    a31 = sin(q33)*cos(state_->q[2]+q32)*cos(ALPHA3)-cos(q33)*sin(ALPHA3);

    a12 = sin(q13)*cos(state_->q[0]+q12)*sin(ALPHA1)+cos(q13)*cos(ALPHA1);
    a22 = sin(q23)*cos(state_->q[1]+q22)*sin(ALPHA2)+cos(q23)*cos(ALPHA2);
    a32 = sin(q33)*cos(state_->q[2]+q32)*sin(ALPHA3)+cos(q33)*cos(ALPHA3);

    a13 = -sin(q13)*sin(state_->q[0]+q12);
    a23 = -sin(q23)*sin(state_->q[1]+q22);
    a33 = -sin(q33)*sin(state_->q[2]+q32);

    double b11, b22, b33;

    b11 = L_arm_*sin(q12)*sin(q13);
    b22 = L_arm_*sin(q22)*sin(q23);
    b33 = L_arm_*sin(q32)*sin(q33);

    Eigen::VectorXd diag(3);
    diag << b11, b22, b33;

    if ((abs(b11)<1e-7) or (abs(b22)<1e-7) or (abs(b33)<1e-7)) {
        return false;
    }

    Eigen::Matrix<double,3,3> JacobQ = diag.asDiagonal();
    Eigen::Matrix<double,3,3> JacobX;
    JacobX << a11, a12, a13,
              a21, a22, a23,
              a31, a32, a33;

    JacMatrix_ = std::make_shared<Eigen::Matrix<double,3,3>>(JacobQ.inverse()*JacobX);
    
    return true;

}

bool delta_model::setJointState(double* q, double* dq){
    
    std::copy(q, q+3, state_->q);
    std::copy(dq, dq+3, state_->dq);
    

    if (!DK()) {
        std::cout << "ERROR DIRECT KINEMATIC";
        return false;
    }
    
    if (!JacobianCal()) {
        std::cout << "ERROR JACOBIAN";
        return false;
    }

    Eigen::Map<Eigen::VectorXd> dq_eig(state_->dq, 3);
    Eigen::VectorXd dpos = (JacMatrix_->inverse()) * dq_eig;

    std::copy(dpos.data(), dpos.data()+3, state_->dpos);


    return true;
    
}

delta_model::~delta_model(){
}