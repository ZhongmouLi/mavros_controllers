#include "trajectory_publisher/trajectory_generator.hpp"
   
TrajectoryGenerator:: TrajectoryGenerator(const double &dt, const int &type):
dt_(dt), T_(0.0), type_(type), degree_(0)
{
// Initialize the generator with default values
generator_ = nullptr;
}

// Destructor
TrajectoryGenerator::~TrajectoryGenerator() 
{
generator_.reset();
}


// Set the trajectory type
void TrajectoryGenerator::setTrajectoryType(const std::string& trajectory_type) 
{
if (trajectory_type == "POLYNOMIAL") {
        trajectory_type_ = POLYNOMIAL;
    } else if (trajectory_type == "CIRCLE") {
        trajectory_type_ = CIRCLE;
    } else if (trajectory_type == "LAMNISCATE") {
        trajectory_type_ = LAMNISCATE;
    } else if (trajectory_type == "STATIONARY") {
        trajectory_type_ = STATIONARY;
    } else {
        throw std::invalid_argument("Invalid trajectory type");
    }
};


void TrajectoryGenerator::initializeGenerator() 
{
switch (trajectory_type_) {
    case POLYNOMIAL:
        generator_ = std::make_shared<polynomialtrajectory>(dt_, 0);
        break;

    case CIRCLE:
        generator_ = std::make_shared<shapetrajectory>(dt_,1);
        break;

    case LAMNISCATE:
        generator_ = std::make_shared<shapetrajectory>(dt_,2);
        break;

    case STATIONARY:
        generator_ = std::make_shared<shapetrajectory>(dt_,3);
        break;

    default:
        throw std::runtime_error("Unknown trajectory type enum value");
}
}

void TrajectoryGenerator::setCircleTrajectory(const Eigen::Vector3d &normal_axis, const double &radius, const double &omega) 
{
    auto shape_ptr = std::dynamic_pointer_cast<shapetrajectory>(generator_);
    if (shape_ptr) {
        shape_ptr->initPrimitives(normal_axis, radius, omega);
    } else {
        throw std::runtime_error("Generator is not a shapetrajectory; cannot call initPrimitives");
    }

// generator_->initPrimitives(normal_axis, radius, omega);
};    

void TrajectoryGenerator::computeTrajectoryAtTime(const double &t) 
{
    // Compute the trajectory at time t
    target_position_ = generator_->getPosition(t);
    target_velocity_ = generator_->getVelocity(t);
    target_acceleration_ = generator_->getAcceleration(t);
}


