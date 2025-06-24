#include "geometric_controller/lib_gain_learning.hpp"

AdaptiveGain::AdaptiveGain()
{
    // Initialize system matrices
    m_a_(0, 3) = 1.0;
    m_a_(1, 4) = 1.0;
    m_a_(2, 5) = 1.0;

    // B
    m_b_(3, 0) = 1.0;
    m_b_(4, 1) = 1.0;
    m_b_(5, 2) = 1.0;

}

AdaptiveGain::~AdaptiveGain()
{

}


// set learning rate
void AdaptiveGain::setLearningRate(const double &learning_rate)
{
 
    // check if learning rate is positive
    // if (learning_rate <= 0)
    // {
    //     throw std::invalid_argument("Learning rate must be positive.");
    // };

    // set learning rate
    l_ = learning_rate;

    // update learning rate matrix T
    m_T_ = l_ * Eigen::Matrix<double, 21, 21>::Identity();

};


void AdaptiveGain::InputStaticGain(const Eigen::Vector3d & kp, const Eigen::Vector3d &kd)
{
    // // P gain for position controller
    // dw_[0] = kp[0];
    // dw_[6] = kp[1];
    // dw_[11] = kp[2];

    // // D gain for position controller
    // dw_[15] = kd[0];
    // dw_[18] = kd[1];
    // dw_[20] = kd[2];

    // P gain for position controller
    w_[0] = kp[0];
    w_[6] = kp[1];
    w_[11] = kp[2];

    // D gain for position controller
    w_[15] = kd[0];
    w_[18] = kd[1];
    w_[20] = kd[2];
}



void AdaptiveGain::GetInputs(const Eigen::VectorXd &ee){


    // assigen elements' values of dphi_
    dphi_(0, 0) = 2 * ee(0);
    dphi_(1, 0) = ee(1);
    dphi_(1, 1) = ee(0);
    dphi_(2, 0) = ee(2);
    dphi_(2, 2) = ee(0);
    dphi_(3, 0) = ee(3);
    dphi_(3, 3) = ee(0);
    dphi_(4, 0) = ee(4);
    dphi_(4, 4) = ee(0);
    dphi_(5, 0) = ee(5);
    dphi_(5, 5) = ee(0);
    dphi_(6, 1) = 2 * ee(1);
    dphi_(7, 1) = ee(2);
    dphi_(7, 2) = ee(1);
    dphi_(8, 1) = ee(3);
    dphi_(8, 3) = ee(1);
    dphi_(9, 1) = ee(4);
    dphi_(9, 4) = ee(1);
    dphi_(10, 1) = ee(5);
    dphi_(10, 5) = ee(1);
    dphi_(11, 2) = 2 * ee(2);
    dphi_(12, 2) = ee(3);
    dphi_(12, 3) = ee(2);
    dphi_(13, 2) = ee(4);
    dphi_(13, 4) = ee(2);
    dphi_(14, 2) = ee(5);
    dphi_(14, 5) = ee(2);
    dphi_(15, 3) = 2 * ee(3);
    dphi_(16, 3) = ee(4);
    dphi_(16, 4) = ee(3);
    dphi_(17, 3) = ee(5);
    dphi_(17, 5) = ee(3);
    dphi_(18, 4) = 2 * ee(4);
    dphi_(19, 4) = ee(5);
    dphi_(19, 5) = ee(4);
    dphi_(20, 5) = 2 * ee(5);
};


void AdaptiveGain::DoLearnGains(){

    // W2 = W;
    // u = -0.5*inv(R)*B'*dphi_'*W2;
    u_ = -0.5 * m_r_.ldlt().solve(m_b_.transpose()) * dphi_.transpose() * w_;

    // Matlab scripts
    // xi = dphi*A*ee+dphi*B*u;
    // chi = ee'*Q*ee+u'*R*u;

    // dPP = -L*PP+xi*xi';
    // dQQ = -L*QQ+xi*chi;
    // M = PP*W+QQ;

    auto xi = dphi_*m_a_*ee_+dphi_*m_b_*u_;
    auto chi = ee_.transpose()*m_q_*ee_+u_.transpose()*m_r_*u_;

    // dPP = -L*PP+xi*xi';
    // dQQ = -L*QQ+xi*chi;
    // M = PP*W+QQ;

    dpp_ = -l_ * pp_ + xi * xi.transpose();
    dqq_ = -l_ * qq_ + xi * chi;
    m_m_ = pp_*w_ + qq_;

    // learning rate m_T
    //  T = .1*diag([1 1 1 1 1 1 1 1 1 1 ...
    // 1 1 1 1 1 1 1 1 1 1 1]);
    // 0.1 weight, larger learning faster
    dw_ = -m_T_ *m_m_;

    //eval = min(real(eig(PP))); min of real part of eigen values of PP
    // if positive->is learning, else negative-> not learning
};

void AdaptiveGain::ComputeAdaptiveGains()
{
    // // P gain for position controller
    // dw_[0] = kp[0];
    // dw_[6] = kp[1];
    // dw_[11] = kp[2];

    // // D gain for position controller
    // dw_[15] = kd[0];
    // dw_[18] = kd[1];
    // dw_[20] = kd[2];

    // updated learning gains
    // kp_adaptive_ = Eigen::Vector3d(dw_[0], dw_[6], dw_[11]);

    // kd_adaptive_ = Eigen::Vector3d(dw_[15], dw_[18], dw_[20]);

    kp_adaptive_ = Eigen::Vector3d(w_[0], w_[6], w_[11]);

    kd_adaptive_ = Eigen::Vector3d(w_[15], w_[18], w_[20]);

}



void AdaptiveGain::ComputeIntigration()
{

    if(flag_initilise_intigration)
    {
        // compute time eplased
        timer_quest_last_ = timer_quest_current_;
        timer_quest_current_ = std::chrono::steady_clock::now();

        std::chrono::duration<double> duration = timer_quest_current_ - timer_quest_last_;
        double dt = duration.count();  // seconds

        // do one step integration here using midpoint approach
        // TODO use RK4 in the future
        qq_ = qq_ +dqq_ *dt;
        // w_ intial = Kp and Kd
        w_ = w_ +dw_ *dt;
        pp_ = pp_ + dpp_ * dt;

        //eval = min(real(eig(PP))); min of real part of eigen values of PP
        // if positive->is learning, else negative-> not learning

        // eigen values matrix
        Eigen::MatrixXcd m_eigenvalues;

        // eigen slover
        Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(pp_);

        // compute eig(PP)
        m_eigenvalues = eigen_solver.eigenvalues();

        // compute real(eig(PP))
        Eigen::MatrixXd m_eigenvalues_real = m_eigenvalues.real();
        // Eigen::MatrixXd m_eigenvalues_imag = m_eigenvalues.imag();


        // compute min(real(eig(PP)))
        eval_ = m_eigenvalues_real.minCoeff();

    }
    else // if first time called
    {
        timer_quest_current_ = std::chrono::steady_clock::now();
        timer_quest_last_ = timer_quest_current_;
        flag_initilise_intigration = true;
        // First call: don't integrate, return basic controller
        // error_pose_I.setZero();
    };

};

bool AdaptiveGain::learningStatus() const
{
    // check if learning is successful
    if (eval_ > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
