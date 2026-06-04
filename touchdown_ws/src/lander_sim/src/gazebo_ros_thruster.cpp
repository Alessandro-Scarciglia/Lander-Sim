#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
  class GazeboRosThruster : public ModelPlugin
  {
  public:
    GazeboRosThruster() : thrust_command_(0.0) {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      
      this->ros_node_ = gazebo_ros::Node::Get(_sdf);
      
      std::string link_name = _sdf->Get<std::string>("link_name");
      this->link_ = _model->GetLink(link_name);

      if (!this->link_) {
        RCLCPP_ERROR(this->ros_node_->get_logger(), "Link %s not found!", link_name.c_str());
        return;
      }

      // Thruster position
      if (_sdf->HasElement("offset")) {
        this->offset_ = _sdf->Get<ignition::math::Vector3d>("offset");
      } else {
        this->offset_ = ignition::math::Vector3d(0, 0, 0);
      }

      // Thrust direction
      if (_sdf->HasElement("direction")) {
        this->direction_ = _sdf->Get<ignition::math::Vector3d>("direction");
      } else {
        this->direction_ = ignition::math::Vector3d(0, 0, 1);
      }

      // Topic subscription
      std::string topic_name = _sdf->Get<std::string>("ros_topic");
      this->sub_ = this->ros_node_->create_subscription<std_msgs::msg::Float64>(
        topic_name, 10,
        std::bind(&GazeboRosThruster::OnThrustMsg, this, std::placeholders::_1));

      
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosThruster::OnUpdate, this));

      RCLCPP_INFO(this->ros_node_->get_logger(), "🚀 Thruster loaded on %s at coordinates [%f, %f, %f]", 
                  topic_name.c_str(), this->offset_.X(), this->offset_.Y(), this->offset_.Z());
    }

  private:
    // Save value
    void OnThrustMsg(const std_msgs::msg::Float64::SharedPtr msg)
    {
      this->thrust_command_ = msg->data;
    }

    
    void OnUpdate()
    {
      if (this->thrust_command_ > 0.0) {
        // Force vector calculation (Direction * Magnitude)
        ignition::math::Vector3d force = this->direction_ * this->thrust_command_;
        
        // Apply the force at a specific point      
        this->link_->AddLinkForce(force, this->offset_);
      }
    }

    gazebo_ros::Node::SharedPtr ros_node_;
    physics::LinkPtr link_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    event::ConnectionPtr update_connection_;
    
    ignition::math::Vector3d offset_;
    ignition::math::Vector3d direction_;
    double thrust_command_;
  };

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosThruster)
}