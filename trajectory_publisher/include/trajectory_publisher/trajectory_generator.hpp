#pragma once

#include <memory>
#include "trajectory_publisher/base/trajectory.h"
#include "trajectory_publisher/base/shapetrajectory.h"
#include "trajectory_publisher/base/polynomialtrajectory.h"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <vector>


class TrajectoryGenerator {

    private:
        std::shared_ptr<trajectory> generator_;

        double dt_; // Sampling time
        double T_;  // Duration of the trajectory
        int type_;  // Type of trajectory
        int degree_;      // Degree of polynomial

        Eigen::Vector3d init_post{0,0,0}; // Initial position

        // std::vector<Eigen::Vector3d> v_waypoint_; // vector of waypoints

        // std::vector<double> v_travelling_time_; // vector of travelling time

        // enum TrajectoryType 
        enum TrajectoryType {
                POLYNOMIAL,
                CIRCLE,
                LAMNISCATE,
                STATIONARY
            };
        
        TrajectoryType trajectory_type_;    


        Eigen::Vector3d target_position_{0,0,0}; // Target position
        Eigen::Vector3d target_velocity_{0,0,0}; // Target velocity
        Eigen::Vector3d target_acceleration_{0,0,0}; // Target acceleration

    public:


        
        // default constructor
        TrajectoryGenerator() = delete;

        TrajectoryGenerator(const double &dt, const int &type);

        ~TrajectoryGenerator() ;

        void setHomePosition(const Eigen::Vector3d &home_position) {
            init_post = home_position;
            generator_->setInitialPosition(init_post);
        }

        Eigen::Vector3d initPosition() const {
            return init_post;
        }

        double circleRadius() const {
            auto shape_ptr = std::dynamic_pointer_cast<shapetrajectory>(generator_);
            return shape_ptr->circleRadius();
        }


        Eigen::Vector3d circleAxis() const {
            auto shape_ptr = std::dynamic_pointer_cast<shapetrajectory>(generator_);
            return shape_ptr->circleAxis();
        }

        double circleOmega() const {
            auto shape_ptr = std::dynamic_pointer_cast<shapetrajectory>(generator_);
            return shape_ptr->circleOmega();
        }


        Eigen::Vector3d homePosition() const {
            return init_post;
        }

        // Set the trajectory type
        void setTrajectoryType(const std::string& trajectory_type);

        // intilialize the generator based on the type
        void initializeGenerator();


        // set circle trajectory 
        void setCircleTrajectory(const Eigen::Vector3d &normal_axis, const double &radius, const double &omega) ;


        void computeTrajectoryAtTime(const double &t); 

        Eigen::Vector3d targetPosition() const { return target_position_; }
        Eigen::Vector3d targetVelocity() const { return target_velocity_; }
        Eigen::Vector3d targetAcceleration() const { return target_acceleration_; }

    };