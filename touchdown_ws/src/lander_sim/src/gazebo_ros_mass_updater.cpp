#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Inertial.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/inertia.hpp> 
#include <memory>

namespace gazebo
{
  class GazeboRosMassUpdater : public ModelPlugin
  {
  public:
    GazeboRosMassUpdater() {}
    virtual ~GazeboRosMassUpdater() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->ros_node_ = gazebo_ros::Node::Get(_sdf);

      std::string link_name = "body_link";
      if (_sdf->HasElement("link_name")) {
        link_name = _sdf->Get<std::string>("link_name");
      }
      this->link_ = _model->GetLink(link_name);

      if (!this->link_) {
        RCLCPP_ERROR(this->ros_node_->get_logger(), "Error: Link %s not found!", link_name.c_str());
        return;
      }

      this->sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Inertia>(
        "/lander/set_dynamics", 10,
        std::bind(&GazeboRosMassUpdater::OnDynamicsUpdate, this, std::placeholders::_1));

      RCLCPP_INFO(this->ros_node_->get_logger(), "🚀 Dynamics Updater plugin loaded! Listening on /lander/set_dynamics");
    }

  private:
    void OnDynamicsUpdate(const geometry_msgs::msg::Inertia::SharedPtr msg)
    {
      
      auto inertial = this->link_->GetInertial();      
      
      inertial->SetMass(msg->m);
      inertial->SetIXX(msg->ixx);
      inertial->SetIYY(msg->iyy);
      inertial->SetIZZ(msg->izz);
      inertial->SetIXY(msg->ixy);
      inertial->SetIXZ(msg->ixz);
      inertial->SetIYZ(msg->iyz);      
      ignition::math::Vector3d new_cog(msg->com.x, msg->com.y, msg->com.z);
      inertial->SetCoG(new_cog);      
      
      this->link_->UpdateMass(); 
    }

    gazebo_ros::Node::SharedPtr ros_node_;
    physics::LinkPtr link_;
    rclcpp::Subscription<geometry_msgs::msg::Inertia>::SharedPtr sub_;
  };

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosMassUpdater)
}