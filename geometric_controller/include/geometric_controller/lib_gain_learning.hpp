#pragma once

#include <Eigen/Dense>
// header for Eigen values
#include <Eigen/Eigenvalues>
#include <cmath>
#include <chrono>


class AdaptiveGain {

private:

    // System matrices
    // Define a 6x6 matrix and initialize it to zero
    Eigen::Matrix<double, 6, 6> m_a_ = Eigen::Matrix<double, 6, 6>::Zero();

    // Define a 6x3 matrix and initialize it to zero
    Eigen::Matrix<double, 6, 3> m_b_ = Eigen::Matrix<double, 6, 3>::Zero();
    Eigen::Matrix<double, 6, 6> m_q_ = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 3, 3> m_r_ = Eigen::Matrix<double, 3, 3>::Identity();

    // Learning rate
    double l_ = 0.1;

    // Initialize learning rate matrix T
    Eigen::Matrix<double, 21, 21> m_T_ = Eigen::Matrix<double, 21, 21>::Identity() * l_;

    Eigen::Matrix<double, 21, 6> dphi_;
    //

    // error of position and velocity
    Eigen::Matrix<double, 6, 1> ee_;
    Eigen::Matrix<double, 21, 21> pp_ = Eigen::Matrix<double, 21, 21>::Ones();;
    Eigen::Matrix<double, 21, 1> qq_ = Eigen::Matrix<double, 21, 1>::Ones();
    Eigen::Matrix<double, 21, 1> w_= Eigen::Matrix<double, 21, 1>::Ones(); // w_(1) kp values, w(12) kd value

    Eigen::Matrix<double,21,1> m_m_;

    //
    double eval_;
    // output
    Eigen::Vector3d u_;

    // adpative p and d gains for position controller
    Eigen::Vector3d kp_adaptive_;
    Eigen::Vector3d kd_adaptive_;

    Eigen::Matrix<double, 21, 21> dpp_;
    Eigen::Matrix<double, 21, 1> dqq_;

    // vector of learning result
    Eigen::Matrix<double, 21, 1> dw_;


    // timer of last integration and current integration
    std::chrono::steady_clock::time_point timer_quest_current_, timer_quest_last_;

    // flag for 1st integration
    bool flag_initilise_intigration = false;


public:

    // constructor
    AdaptiveGain();

    // deconstrutor
    ~AdaptiveGain();

    // input p and d gains for position control
    void InputStaticGain(const Eigen::Vector3d & kp, const Eigen::Vector3d &kd);

    // get inputs for learning
    void GetInputs(const Eigen::VectorXd &ee);


    // run learning process to update dw_
    void DoLearnGains();

    // compute adaptive gain
    void  ComputeAdaptiveGains();


    void ComputeIntigration();

    bool learningStatus() const;;

    // obtain adaptive p gain
    Eigen::Vector3d pGainAdaptive() const {return kp_adaptive_;};

    // obtain adaptive d gain
    Eigen::Vector3d dGainAdaptive() const {return kd_adaptive_;};

    Eigen::Matrix<double, 21, 1> dw() const {return dw_;};

    void setLearningRate(const double &learning_rate);

    double learningRate() const {return l_;};

};

    // // Main function equivalent to the MATLAB fcn
    // void fcn(const Eigen::VectorXd& ee, Eigen::VectorXd& W,
    //          Eigen::MatrixXd& PP, Eigen::VectorXd& QQ, double t,
    //          Eigen::VectorXd& dW, Eigen::MatrixXd& dPP, Eigen::VectorXd& dQQ,
    //          Eigen::Vector3d& u, double& eval, Eigen::Matrix3d& kp, Eigen::Matrix3d& kd) {

    //     // Initialize output variables
    //     dW = Eigen::VectorXd::Zero(21);
    //     dPP = Eigen::MatrixXd::Zero(21, 21);
    //     dQQ = Eigen::VectorXd::Zero(21);
    //     u = Eigen::Vector3d::Zero();
    //     eval = 0.0;
    //     kp = Eigen::Matrix3d::Zero();
    //     kd = Eigen::Matrix3d::Zero();

           // P = complex number
    //     // Call LQR solver (in practice, use a library implementation)
    //     [S, P, K] = solveLQR(A, B, Q, R);

    //     // Construct the Jacobian of the regressor dphi
    //     Eigen::MatrixXd dphi = Eigen::MatrixXd::Zero(21, 6);

    //     // First row
    //     dphi(0, 0) = 2 * ee(0);

    //     // Second row
    //     dphi(1, 0) = ee(1);
    //     dphi(1, 1) = ee(0);

    //     // Third row
    //     dphi(2, 0) = ee(2);
    //     dphi(2, 2) = ee(0);

    //     // Fourth row
    //     dphi(3, 0) = ee(3);
    //     dphi(3, 3) = ee(0);

    //     // Fifth row
    //     dphi(4, 0) = ee(4);
    //     dphi(4, 4) = ee(0);

    //     // Sixth row
    //     dphi(5, 0) = ee(5);
    //     dphi(5, 5) = ee(0);

    //     // Seventh row
    //     dphi(6, 1) = 2 * ee(1);

    //     // Eighth row
    //     dphi(7, 1) = ee(2);
    //     dphi(7, 2) = ee(1);

    //     // Ninth row
    //     dphi(8, 1) = ee(3);
    //     dphi(8, 3) = ee(1);

    //     // Tenth row
    //     dphi(9, 1) = ee(4);
    //     dphi(9, 4) = ee(1);

    //     // Eleventh row
    //     dphi(10, 1) = ee(5);
    //     dphi(10, 5) = ee(1);

    //     // Twelfth row
    //     dphi(11, 2) = 2 * ee(2);

    //     // Thirteenth row
    //     dphi(12, 2) = ee(3);
    //     dphi(12, 3) = ee(2);

    //     // Fourteenth row
    //     dphi(13, 2) = ee(4);
    //     dphi(13, 4) = ee(2);

    //     // Fifteenth row
    //     dphi(14, 2) = ee(5);
    //     dphi(14, 5) = ee(2);

    //     // Sixteenth row
    //     dphi(15, 3) = 2 * ee(3);

    //     // Seventeenth row
    //     dphi(16, 3) = ee(4);
    //     dphi(16, 4) = ee(3);

    //     // Eighteenth row
    //     dphi(17, 3) = ee(5);
    //     dphi(17, 5) = ee(3);

    //     // Nineteenth row
    //     dphi(18, 4) = 2 * ee(4);

    //     // Twentieth row
    //     dphi(19, 4) = ee(5);
    //     dphi(19, 5) = ee(4);

    //     // Twenty-first row
    //     dphi(20, 5) = 2 * ee(5);

    //     // Compute control input u
    //     Eigen::VectorXd W2 = W;
    //     u = -0.5 * R.inverse() * B.transpose() * dphi.transpose() * W2;

    //     // Compute xi and chi
    //     Eigen::VectorXd xi = dphi * A * ee + dphi * B * u;
    //     double chi = ee.transpose() * Q * ee + u.transpose() * R * u;

    //     // Update dPP and dQQ
    //     dPP = -L * PP + xi * xi.transpose();
    //     dQQ = -L * QQ + xi * chi;

    //     // Compute M
    //     Eigen::VectorXd M = PP * W + QQ;

    //     // Compute dW
    //     dW = -T * M;

    //     // Compute eigenvalues for evaluation
    //     Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(PP);
    //     if (eigenSolver.info() == Eigen::Success) {
    //         Eigen::VectorXd eigenvalues = eigenSolver.eigenvalues();
    //         eval = eigenvalues.minCoeff();
    //         double cond = eigenvalues.maxCoeff() / eval;
    //     }

    //     // Set controller gains
    //     kp = Eigen::Matrix3d::Zero();
    //     kp(0, 0) = W(0);
    //     kp(1, 1) = W(6);
    //     kp(2, 2) = W(11);

    //     kd = Eigen::Matrix3d::Zero();
    //     kd(0, 0) = W(15);
    //     kd(1, 1) = W(18);
    //     kd(2, 2) = W(20);
    // }

