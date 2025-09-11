#pragma once

#include <memory>
#include "trajectory_publisher/base/trajectory.h"
#include "trajectory_publisher/base/shapetrajectory.h"
#include "trajectory_publisher/base/polynomialtrajectory.h"
#include <ros/package.h>
#include <vector>
#include <array>


enum class TrajectoryType {
    POLYNOMIAL,
    CIRCLE,
    LAMNISCATE,
    STATIONARY
};


struct TrajectoryStrct {
    std::string id;
    TrajectoryType type{TrajectoryType::STATIONARY};  // use enum instead of string
    double duration{0.0};
    virtual ~TrajectoryStrct() = default;
};

// Polynomial trajectory
struct PolynomialTrajStrct : public TrajectoryStrct {
    std::array<double, 3> start_position{};
    double start_yaw{0.0};
    std::array<double, 3> end_position{};
    double end_yaw{0.0};
};

// Circle trajectory
struct CircleTrajStrct : public TrajectoryStrct {
    std::array<double, 3> circle_axis{};
    double circle_omega{0.0};
    double radius{1.0};
    std::array<double, 3> initial_pos{};
};

class TrajectoryGenerator {

    private:

        std::vector<std::shared_ptr<TrajectoryStrct>> v_trajectory_strct_;

        std::shared_ptr<TrajectoryStrct> active_traj_config_ = nullptr;

        std::shared_ptr<trajectory> ptr_trajectory_ = nullptr;

        Eigen::Vector3d target_position_{0,0,0}; // Target position
        Eigen::Vector3d target_velocity_{0,0,0}; // Target velocity
        Eigen::Vector3d target_acceleration_{0,0,0}; // Target acceleration

        Eigen::Vector3d homoe_position_{0,0,0}; // Home position
        bool isHomeSet_ = false;

        Eigen::Vector3d init_position_{0,0,0}; // Initial position
        bool isInitPositionSet_ = false;

        double dt_{0.01}; // Time step

    public:

        // default constructor
        TrajectoryGenerator() = delete;

        TrajectoryGenerator(const double &dt);

        TrajectoryGenerator(const double &dt, const int &type);


        ~TrajectoryGenerator() ;

        void setHomePosition(const Eigen::Vector3d &home_position);

        void setInitPosition(const Eigen::Vector3d &Init_position);


        void setTrajectoryType(const TrajectoryType &trajectory_type) ;


        void inputTrajectoryStrcutData(const PolynomialTrajStrct &poly) ;
        void inputTrajectoryStrcutData(const CircleTrajStrct &circle) ;

        void chooseAndConfigureByTime(const double &t) ;

        Eigen::Vector3d initPosition() const {
            return init_position_;
        }


        bool isInitPositionSet() const {
            return isInitPositionSet_;
        }


        Eigen::Vector3d homePosition() const {
            return homoe_position_;
        }

        bool isHomeSet() const {
            return isHomeSet_;
        }



        // double circleRadius() const {
        //     auto shape_ptr = std::dynamic_pointer_cast<shapetrajectory>(ptr_trajectory_);
        //     return shape_ptr->circleRadius();
        // }


        // Eigen::Vector3d circleAxis() const {
        //     auto shape_ptr = std::dynamic_pointer_cast<shapetrajectory>(ptr_trajectory_);
        //     return shape_ptr->circleAxis();
        // }

        // double circleOmega() const {
        //     auto shape_ptr = std::dynamic_pointer_cast<shapetrajectory>(ptr_trajectory_);
        //     return shape_ptr->circleOmega();
        // }



        // Set the trajectory type
        void setTrajectoryType(const std::string& trajectory_type);

        // intilialize the generator based on the type
        void initializeGenerator();


        // set circle trajectory 
        void setCircleTrajectory(const Eigen::Vector3d &initial_position, const Eigen::Vector3d &normal_axis, const double &radius, const double &omega) ;

        void setPolyTrajectory(const Eigen::Vector3d &target_post, const double &travelling_time); 

        void setPolyTrajectory(const Eigen::Vector3d &start_post, const Eigen::Vector3d &target_post, const double &travelling_time); 


        void computeTrajectoryAtTime(const double &t); 

        Eigen::Vector3d targetPosition() const { return target_position_; }
        Eigen::Vector3d targetVelocity() const { return target_velocity_; }
        Eigen::Vector3d targetAcceleration() const { return target_acceleration_; }

    };