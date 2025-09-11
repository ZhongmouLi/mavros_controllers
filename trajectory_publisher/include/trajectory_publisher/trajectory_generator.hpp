#pragma once

#include <memory>
#include "trajectory_publisher/base/trajectory.h"
#include "trajectory_publisher/base/shapetrajectory.h"
#include "trajectory_publisher/base/polynomialtrajectory.h"
#include <ros/package.h>
#include <vector>
#include <array>
#include <sstream>
#include <iomanip> 


enum class TrajectoryType {
    POLYNOMIAL,
    CIRCLE,
    LAMNISCATE,
    STATIONARY
};


struct TrajectoryStrct{

    std::string id;

    bool isRelative{true};

    Eigen::Vector3d off_set{0,0,0};

    TrajectoryType type{TrajectoryType::STATIONARY};  // use enum instead of string

    double duration{0.0};

    virtual ~TrajectoryStrct() = default;
};

// Polynomial trajectory
struct PolynomialTrajStrct : public TrajectoryStrct {

    Eigen::Vector3d start_position{};

    double start_yaw{0.0};

    Eigen::Vector3d end_position{};

    double end_yaw{0.0};

    Eigen::Vector3d startPosition() 
    {
        if (isRelative) {
            return off_set + start_position;
        } else {
            return start_position;
        }
    }; 

    Eigen::Vector3d endPosition() 
    {
        if (isRelative) {
            return off_set + end_position;
        } else {
            return end_position;
        }
    }; 
};

// Circle trajectory
struct CircleTrajStrct : public TrajectoryStrct {

    Eigen::Vector3d circle_axis{};

    double circle_omega{0.0};

    double radius{1.0};
};

class TrajectoryGenerator {

    private:

        std::vector<std::shared_ptr<TrajectoryStrct>> v_trajectory_strct_;

        std::shared_ptr<TrajectoryStrct> active_traj_config_ = nullptr;

        std::shared_ptr<trajectory> ptr_trajectory_ = nullptr;

        TrajectoryType current_trajectory_type_{TrajectoryType::STATIONARY};

        Eigen::Vector3d target_position_{0,0,0}; // Target position
        Eigen::Vector3d target_velocity_{0,0,0}; // Target velocity
        Eigen::Vector3d target_acceleration_{0,0,0}; // Target acceleration

        Eigen::Vector3d last_target_position_{0,0,0}; // Last target position

        Eigen::Vector3d home_position_{0,0,0}; // Home position
        bool isHomeSet_ = false;

        Eigen::Vector3d init_position_{0,0,0}; // Initial position
        bool isInitPositionSet_ = false;


        double dt_{0.01}; // Time step


        double total_trajectory_time_{0.0}; // Total trajectory time

        bool is_trajectry_ended_ = false;

        bool is_trajectory_offset_set_ = false;

        bool is_trajectory_configured_ = false;


        // set circle trajectory 
        void setCircleTrajectory(const Eigen::Vector3d &initial_position, const Eigen::Vector3d &normal_axis, const double &radius, const double &omega) ;

        void setPolyTrajectory(const Eigen::Vector3d &target_post, const double &travelling_time); 

        void setPolyTrajectory(const Eigen::Vector3d &start_post, const Eigen::Vector3d &target_post, const double &travelling_time); 

        void setTrajectoryType(const std::string& trajectory_type);

    public:

        // default constructor
        TrajectoryGenerator() = delete;

        TrajectoryGenerator(const double &dt);

        TrajectoryGenerator(const double &dt, const int &type);


        ~TrajectoryGenerator() ;

        void setHomePosition(const Eigen::Vector3d &home_position);

        void setInitPosition(const Eigen::Vector3d &Init_position);


        void setOffsetForAllSegments(const Eigen::Vector3d& off);
        

        void computeTotalTrajectoryTime();

        void setTrajectoryType(const TrajectoryType &trajectory_type) ;


        void inputTrajectoryStrcutData(const PolynomialTrajStrct &poly) ;
        void inputTrajectoryStrcutData(const CircleTrajStrct &circle) ;

        void chooseAndConfigureByTime(const double &t) ;

      
        void computeTrajectoryAtTime(const double &t); 


    public:

        Eigen::Vector3d currentStartPosition() const;

        Eigen::Vector3d currentEndPosition() const;

        std::string currentTrajectoryType() const;

        std::string currentSegTrajInfor() const;

        std::string TotalTrajectoryInfor() const ;


        Eigen::Vector3d targetPosition() const { return target_position_; }
        Eigen::Vector3d targetVelocity() const { return target_velocity_; }
        Eigen::Vector3d targetAcceleration() const { return target_acceleration_; }

        Eigen::Vector3d initPosition() const {return init_position_;}


        bool isInitPositionSet() const {return isInitPositionSet_;}


        Eigen::Vector3d homePosition() const {return home_position_;}

        bool isHomeSet() const {return isHomeSet_;};

        bool isTrajectoryOffsetSet() const {return is_trajectory_offset_set_;};

        bool isTrajectoryConfigured() const {return is_trajectory_configured_;};

        double totalTime() const {return total_trajectory_time_;};

};