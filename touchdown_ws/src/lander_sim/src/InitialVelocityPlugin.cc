#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{

class InitialVelocityPlugin : public ModelPlugin
{
  public:

    InitialVelocityPlugin()
    : reset_detected_(false)
    {
    }
    

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model_ = _model;

      this->ros_node_ = gazebo_ros::Node::Get(_sdf);

      // Default velocities
      this->linear_velocity_ =
        ignition::math::Vector3d(0, 0, 0);

      this->angular_velocity_ =
        ignition::math::Vector3d(0, 0, 0);

      // Read SDF parameters
      if (_sdf->HasElement("linear_velocity"))
      {
        this->linear_velocity_ =
          _sdf->Get<ignition::math::Vector3d>(
            "linear_velocity");
      }

      if (_sdf->HasElement("angular_velocity"))
      {
        this->angular_velocity_ =
          _sdf->Get<ignition::math::Vector3d>(
            "angular_velocity");
      }

      // Apply initial velocities
      this->ApplyVelocities();

      // Connect update callback
      this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin(
          std::bind(
            &InitialVelocityPlugin::OnUpdate,
            this));

      RCLCPP_INFO(this->ros_node_->get_logger(), 
                  "[InitialVelocityPlugin] Loaded on model: %s", 
                  this->model_->GetName().c_str());
    }

  private:  

    void ApplyVelocities()
    {
      this->model_->SetLinearVel(
        this->linear_velocity_);

      this->model_->SetAngularVel(
        this->angular_velocity_);

      RCLCPP_INFO(this->ros_node_->get_logger(), 
                  "[InitialVelocityPlugin] Velocities applied | Linear: [%.3f, %.3f, %.3f] | Angular: [%.3f, %.3f, %.3f]",
                  this->linear_velocity_.X(), this->linear_velocity_.Y(), this->linear_velocity_.Z(),
                  this->angular_velocity_.X(), this->angular_velocity_.Y(), this->angular_velocity_.Z());
    }


    void OnUpdate()
    {
      common::Time current_time =
        this->model_->GetWorld()->SimTime();

      // Detect reset world
      if (current_time < this->last_time_)
      {
        RCLCPP_INFO(this->ros_node_->get_logger(), "[InitialVelocityPlugin] World reset detected");

        this->reset_detected_ = true;
      }

      // Apply velocities one frame after reset
      if (this->reset_detected_)
      {
        this->ApplyVelocities();

        this->reset_detected_ = false;
      }

      this->last_time_ = current_time;
    }


    physics::ModelPtr model_;
    gazebo_ros::Node::SharedPtr ros_node_;
    ignition::math::Vector3d linear_velocity_;
    ignition::math::Vector3d angular_velocity_;
    event::ConnectionPtr update_connection_;
    common::Time last_time_;
    bool reset_detected_;
};

// Register plugin
GZ_REGISTER_MODEL_PLUGIN(InitialVelocityPlugin)

}