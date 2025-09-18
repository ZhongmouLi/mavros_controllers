#include "trajectory_publisher/trajectory_generator.hpp"
#include <codecvt>
#include <mutex>
   
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
            home_position_ = home_position;
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
    ptr_poly_strcut_data->isRelative = poly.isRelative;
    // ptr_poly_strcut_data->off_set = poly.off_set;
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
    ptr_circle_strcut_data->isRelative = circle.isRelative;
    // ptr_circle_strcut_data->off_set = circle.off_set;
    ptr_circle_strcut_data->type = TrajectoryType::CIRCLE;
    ptr_circle_strcut_data->duration = circle.duration;

    ptr_circle_strcut_data->circle_axis = circle.circle_axis;
    ptr_circle_strcut_data->circle_omega = circle.circle_omega;
    ptr_circle_strcut_data->radius = circle.radius;

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

    // use pre-defined trajectory type if time is within totl time
    if (t>=totalTime())
    {
        current_trajectory_type_ = TrajectoryType::STATIONARY;
        is_trajectry_ended_ = true;
        return;
    };

    // 1) Find active segment by accumulated duration
    double acc_time = 0.0;

    
    // decide which segment we are in
    for (const auto& seg : v_trajectory_strct_) {
        acc_time += seg->duration;
        if (t <= acc_time) { active_traj_config_ = seg; break; }
    }

    current_trajectory_type_ = active_traj_config_->type;

    if (!active_traj_config_) {                     // t past total duration
        setTrajectoryType(TrajectoryType::STATIONARY); // default
        return;
    }

    
    // 2) Set trajectory type
    setTrajectoryType(current_trajectory_type_);

    // 3) Configure generator with segment-specific details
    switch (current_trajectory_type_) {
        case TrajectoryType::POLYNOMIAL: {
            auto p = std::dynamic_pointer_cast<PolynomialTrajStrct>(active_traj_config_);

            // Option A: use your 3-arg overload (start, target, time):
            // Eigen::Vector3d start_pos = initPosition() + Eigen::Vector3d{p->start_position[0], p->start_position[1], p->start_position[2]};
            // Eigen::Vector3d end_pos   = initPosition() + Eigen::Vector3d{p->end_position[0],   p->end_position[1],   p->end_position[2]};

            Eigen::Vector3d start_pos = p->startPosition();

            Eigen::Vector3d end_pos   = p->endPosition();

            setPolyTrajectory(
                start_pos,
                end_pos,
                p->duration
            );

           
            break;
        }

        case TrajectoryType::CIRCLE: {

            auto c = std::dynamic_pointer_cast<CircleTrajStrct>(active_traj_config_);

            Eigen::Vector3d axis{c->circle_axis[0], c->circle_axis[1], c->circle_axis[2]};

            // Derive radius from your stored positions:
            // use distance from the generator's initial position to the initial_pos of the circle segment.
            // (Adjust if you have an explicit center elsewhere.)

            // Eigen::Vector3d segInit{c->initial_pos[0], c->initial_pos[1], c->initial_pos[2]};

            double radius = c->radius; // Use radius from struct
            double angular_velocity = c->circle_omega;

            setCircleTrajectory(initPosition(), axis, radius, angular_velocity);

   
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

    is_trajectory_configured_ = true;

    
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
    if (isTrajectoryOffsetSet() && isTrajectoryConfigured() && current_trajectory_type_ != TrajectoryType::STATIONARY) 
    {
        // Compute the trajectory at time t
        target_position_ = ptr_trajectory_->getPosition(t);
        target_velocity_ = ptr_trajectory_->getVelocity(t);
        target_acceleration_ = ptr_trajectory_->getAcceleration(t);

        last_target_position_ = target_position_;
    }
    else if (isTrajectoryOffsetSet() && isTrajectoryConfigured() && current_trajectory_type_ == TrajectoryType::STATIONARY)
    {
        // Hold last position if stationary
        target_position_ = last_target_position_;
        target_velocity_ = Eigen::Vector3d::Zero();
        target_acceleration_ = Eigen::Vector3d::Zero();
    }
    else {
        throw std::runtime_error("Trajectory offset or configuration not set before computing trajectory.");
    }
}


void TrajectoryGenerator::computeTotalTrajectoryTime()
{
    for (const auto& seg : v_trajectory_strct_) {
        total_trajectory_time_ += seg->duration;
    }
}

void TrajectoryGenerator::setOffsetForAllSegments(const Eigen::Vector3d& off)
{
    for (auto& seg : v_trajectory_strct_) {
        seg->off_set = off;
    }
    is_trajectory_offset_set_ = true;
}

Eigen::Vector3d TrajectoryGenerator::currentStartPosition() const
{
    if (!active_traj_config_) return Eigen::Vector3d::Zero();
    try 
    {
        auto p = std::static_pointer_cast<PolynomialTrajStrct>(active_traj_config_);
        return p->startPosition();
    } 
    catch (const std::bad_cast&) 
    {
        return Eigen::Vector3d::Zero();
    };
}

Eigen::Vector3d TrajectoryGenerator::currentEndPosition() const
{
    if (!active_traj_config_) return Eigen::Vector3d::Zero();
    try 
    {
        auto p = std::static_pointer_cast<PolynomialTrajStrct>(active_traj_config_);
        return p->endPosition();
    } 
    catch (const std::bad_cast&) 
    {
        return Eigen::Vector3d::Zero();
    };
}


std::string TrajectoryGenerator::TotalTrajectoryInfor() const 
{
    if (v_trajectory_strct_.empty()) return "No trajectory segments loaded";

    auto vec3str = [](const Eigen::Vector3d& v) {
        std::ostringstream s; s << std::fixed << std::setprecision(3)
                                << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
        return s.str();
    };
    auto type_cstr = [](TrajectoryType t) {
        switch (t) {
            case TrajectoryType::POLYNOMIAL: return "POLYNOMIAL";
            case TrajectoryType::CIRCLE:     return "CIRCLE";
            case TrajectoryType::LAMNISCATE: return "LAMNISCATE";
            case TrajectoryType::STATIONARY: return "STATIONARY";
            default:                         return "UNKNOWN";
        }
    };

    std::ostringstream oss;
    oss << std::boolalpha;
    oss << "Total segments: " << v_trajectory_strct_.size() << "\n";

    std::size_t idx = 0;
    for (const auto& seg : v_trajectory_strct_) {
        if (!seg) continue;
        oss << "#" << (++idx)
            << " id=" << seg->id
            << ", type=" << type_cstr(seg->type)
            << ", duration=" << seg->duration
            << ", isRelative=" << seg->isRelative
            << ", offset=" << vec3str(seg->off_set);

        switch (seg->type) {
            case TrajectoryType::POLYNOMIAL: {
                auto p = std::static_pointer_cast<PolynomialTrajStrct>(seg);
                const Eigen::Vector3d start_abs = p->isRelative ? (p->off_set + p->start_position) : p->start_position;
                const Eigen::Vector3d end_abs   = p->isRelative ? (p->off_set + p->end_position)   : p->end_position;

                oss << ", start="      << vec3str(p->start_position)
                    << ", start_yaw="  << p->start_yaw
                    << ", end="        << vec3str(p->end_position)
                    << ", end_yaw="    << p->end_yaw
                    << ", start(abs)=" << vec3str(start_abs)
                    << ", end(abs)="   << vec3str(end_abs);
                break;
            }
            case TrajectoryType::CIRCLE: {
                auto c = std::static_pointer_cast<CircleTrajStrct>(seg);
                oss << ", axis="   << vec3str(c->circle_axis)
                    << ", omega="  << c->circle_omega
                    << ", radius=" << c->radius;
                break;
            }
            default:
                break;
        }

        

        oss << "\n";
    }

    oss << "whole_trajectory_time=" << totalTime();

    return oss.str();
}



std::string TrajectoryGenerator::currentSegTrajInfor() const
{
    if (!active_traj_config_) return "No active trajectory segment";

    std::string info;

    auto vec3str = [](const Eigen::Vector3d& v) {
                return std::string("[") + std::to_string(v.x()) + ", "
                                        + std::to_string(v.y()) + ", "
                                        + std::to_string(v.z()) + "]";
            };
            
    if(!is_trajectry_ended_)
    {
       

        info = "id=" + active_traj_config_->id + ", type=";
        switch (active_traj_config_->type) {
            case TrajectoryType::POLYNOMIAL: info += "POLYNOMIAL"; break;
            case TrajectoryType::CIRCLE:     info += "CIRCLE";     break;
            case TrajectoryType::LAMNISCATE: info += "LAMNISCATE"; break;
            case TrajectoryType::STATIONARY: info += "STATIONARY"; break;
            default:                         info += "UNKNOWN";    break;
        }

        // Base fields
        info += ", duration="   + std::to_string(active_traj_config_->duration);
        info += ", isRelative=" + std::string(active_traj_config_->isRelative ? "true" : "false");
        info += ", offset="     + vec3str(active_traj_config_->off_set);

        // Derived fields
        switch (active_traj_config_->type) {
            case TrajectoryType::POLYNOMIAL: {
                auto p = std::static_pointer_cast<PolynomialTrajStrct>(active_traj_config_);
                const Eigen::Vector3d start_abs = p->isRelative ? (p->off_set + p->start_position) : p->start_position;
                const Eigen::Vector3d end_abs   = p->isRelative ? (p->off_set + p->end_position)   : p->end_position;

                info += ", start="      + vec3str(p->start_position);
                info += ", start_yaw="  + std::to_string(p->start_yaw);
                info += ", end="        + vec3str(p->end_position);
                info += ", end_yaw="    + std::to_string(p->end_yaw);
                info += ", start(abs)=" + vec3str(start_abs);
                info += ", end(abs)="   + vec3str(end_abs);
                break;
            }
            case TrajectoryType::CIRCLE: {
                auto c = std::static_pointer_cast<CircleTrajStrct>(active_traj_config_);
                info += ", axis="   + vec3str(c->circle_axis);
                info += ", omega="  + std::to_string(c->circle_omega);
                info += ", radius=" + std::to_string(c->radius);
                break;
            }
            default:
                break;
        }
    }
    else 
    {
        info = "trajectory ENDED, Hovering at " + vec3str(last_target_position_);
    }

    return info;
};

