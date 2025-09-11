#include "trajectory_publisher/trajectory_generator.hpp"
#include <codecvt>
   
TrajectoryGenerator:: TrajectoryGenerator(const double &dt, const int &type):
dt_(dt)
{
    // Initialize the generator with default values
    ptr_trajectory_ = nullptr;
}


TrajectoryGenerator:: TrajectoryGenerator(const double &dt):
dt_(dt)
{
    // Initialize the generator with default values
    ptr_trajectory_ = nullptr;
}


// Destructor
TrajectoryGenerator::~TrajectoryGenerator() 
{
    ptr_trajectory_.reset();
}


// Set the trajectory type
// void TrajectoryGenerator::setTrajectoryType(const std::string& trajectory_type) 
// {
// if (trajectory_type == "POLYNOMIAL") {
//         trajectory_type_ = POLYNOMIAL;
//     } else if (trajectory_type == "CIRCLE") {
//         trajectory_type_ = CIRCLE;
//     } else if (trajectory_type == "LAMNISCATE") {
//         trajectory_type_ = LAMNISCATE;
//     } else if (trajectory_type == "STATIONARY") {
//         trajectory_type_ = STATIONARY;
//     } else {
//         throw std::invalid_argument("Invalid trajectory type");
//     }
// };




void TrajectoryGenerator::setHomePosition(const Eigen::Vector3d &home_position) 
{
            homoe_position_ = home_position;
            // ptr_trajectory_->setInitialPosition(init_post_);
            isHomeSet_ = true;
}




void TrajectoryGenerator::setInitPosition(const Eigen::Vector3d &Init_position) 
{
            init_position_ = Init_position;
            // std::cout<<"fuck setInitPosition 1"<<std::endl;
            // ptr_trajectory_->setInitialPosition(init_position_);
            isInitPositionSet_ = true;
            // std::cout<<"fuck setInitPosition 2"<<std::endl;
}



void TrajectoryGenerator::inputTrajectoryStrcutData(const PolynomialTrajStrct &poly) 
{

    auto ptr_poly_strcut_data = std::make_shared<PolynomialTrajStrct>();
    ptr_poly_strcut_data->id = poly.id;
    ptr_poly_strcut_data->type = TrajectoryType::POLYNOMIAL;
    ptr_poly_strcut_data->duration = poly.duration;

    ptr_poly_strcut_data->start_position = poly.start_position;
    ptr_poly_strcut_data->start_yaw = poly.start_yaw;
    ptr_poly_strcut_data->end_position = poly.end_position;
    ptr_poly_strcut_data->end_yaw = poly.end_yaw;

    v_trajectory_strct_.push_back(ptr_poly_strcut_data);
};   


void TrajectoryGenerator::inputTrajectoryStrcutData(const CircleTrajStrct &circle) 
{

    auto ptr_circle_strcut_data = std::make_shared<CircleTrajStrct>();

    ptr_circle_strcut_data->id = circle.id;
    ptr_circle_strcut_data->type = TrajectoryType::CIRCLE;
    ptr_circle_strcut_data->duration = circle.duration;

    ptr_circle_strcut_data->circle_axis = circle.circle_axis;
    ptr_circle_strcut_data->circle_omega = circle.circle_omega;
    ptr_circle_strcut_data->initial_pos = circle.initial_pos;

    v_trajectory_strct_.push_back(ptr_circle_strcut_data);
};   



void TrajectoryGenerator::setTrajectoryType(const TrajectoryType &trajectory_type) 
{
    switch (trajectory_type) {
        case TrajectoryType::POLYNOMIAL:
            ptr_trajectory_ = std::make_shared<polynomialtrajectory>(dt_);
            break;

        case TrajectoryType::CIRCLE:
            ptr_trajectory_ = std::make_shared<shapetrajectory>(dt_,1);
            break;

        case TrajectoryType::LAMNISCATE:
            ptr_trajectory_ = std::make_shared<shapetrajectory>(dt_,2);
            break;

        case TrajectoryType::STATIONARY:
            ptr_trajectory_ = std::make_shared<shapetrajectory>(dt_,3);
            break;

        default:
            throw std::runtime_error("Unknown trajectory type enum value");
    }
}


void TrajectoryGenerator::chooseAndConfigureByTime(const double &t)
{
    // 1) Find active segment by accumulated duration
    double acc_time = 0.0;

    
    // decide which segment we are in
    for (const auto& seg : v_trajectory_strct_) {
        acc_time += seg->duration;
        if (t <= acc_time) { active_traj_config_ = seg; break; }
    }

    if (!active_traj_config_) {                     // t past total duration
        setTrajectoryType(TrajectoryType::STATIONARY); // default
        return;
    }

    // 2) Set trajectory type
    setTrajectoryType(active_traj_config_->type);

    // 3) Configure generator with segment-specific details
    switch (active_traj_config_->type) {
        case TrajectoryType::POLYNOMIAL: {
            auto p = std::static_pointer_cast<PolynomialTrajStrct>(active_traj_config_);

            // Option A: use your 3-arg overload (start, target, time):
            setPolyTrajectory(
                Eigen::Vector3d{p->start_position[0], p->start_position[1], p->start_position[2]},
                Eigen::Vector3d{p->end_position[0],   p->end_position[1],   p->end_position[2]},
                p->duration
            );
            break;
        }

        case TrajectoryType::CIRCLE: {

            auto c = std::static_pointer_cast<CircleTrajStrct>(active_traj_config_);

            Eigen::Vector3d axis{c->circle_axis[0], c->circle_axis[1], c->circle_axis[2]};

            // Derive radius from your stored positions:
            // use distance from the generator's initial position to the initial_pos of the circle segment.
            // (Adjust if you have an explicit center elsewhere.)

            Eigen::Vector3d segInit{c->initial_pos[0], c->initial_pos[1], c->initial_pos[2]};

            double radius = c->radius; // Use radius from struct

            setCircleTrajectory(segInit, axis, radius, c->circle_omega);
            break;
        }

        case TrajectoryType::LAMNISCATE:
            // No extra parameters in your API for these cases
            break;

        case TrajectoryType::STATIONARY:
            // No extra parameters in your API for these cases
            break;

        default:
            // No extra parameters in your API for these cases
            break;
    }
}


void TrajectoryGenerator::setCircleTrajectory(const Eigen::Vector3d &initial_position, const Eigen::Vector3d &normal_axis, const double &radius, const double &omega) 
{
    auto shape_ptr = std::dynamic_pointer_cast<shapetrajectory>(ptr_trajectory_);
    if (shape_ptr) {
        shape_ptr->initPrimitives(initial_position, normal_axis, radius, omega);
    } else {
        throw std::runtime_error("Generator is not a shapetrajectory; cannot call initPrimitives");
    }

// ptr_trajectory_->initPrimitives(normal_axis, radius, omega);
};    


void TrajectoryGenerator::setPolyTrajectory(const Eigen::Vector3d &target_post, const double &travelling_time) 
{
    auto shape_ptr = std::dynamic_pointer_cast<polynomialtrajectory>(ptr_trajectory_);
    if (shape_ptr) {
        shape_ptr->initPrimitives(target_post, travelling_time);
    } else {
        throw std::runtime_error("Generator is not a polynomialtrajectory; cannot call initPrimitives");
    }
};   

void TrajectoryGenerator::setPolyTrajectory(const Eigen::Vector3d &start_post, const Eigen::Vector3d &target_post, const double &travelling_time) 
{
    auto shape_ptr = std::dynamic_pointer_cast<polynomialtrajectory>(ptr_trajectory_);
    if (shape_ptr) {
        shape_ptr->initPrimitives(start_post,target_post, travelling_time);
    } else {
        throw std::runtime_error("Generator is not a polynomialtrajectory; cannot call initPrimitives");
    }
};   

void TrajectoryGenerator::computeTrajectoryAtTime(const double &t) 
{
    // Compute the trajectory at time t
    target_position_ = ptr_trajectory_->getPosition(t);
    target_velocity_ = ptr_trajectory_->getVelocity(t);
    target_acceleration_ = ptr_trajectory_->getAcceleration(t);
}


