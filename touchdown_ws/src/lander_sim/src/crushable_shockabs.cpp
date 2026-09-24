// Model a configurable landing-leg shock absorber inside Gazebo.
// Each update computes crush, damping, rebound, and bottom-out forces, then
// publishes the force/displacement pair used by the Python logger.

// Gazebo, ROS 2, and message interfaces
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <geometry_msgs/msg/point.hpp>

namespace gazebo
{

class CrushableJointPlugin : public ModelPlugin
{
  public:
    CrushableJointPlugin()
    : max_pos_deformation_(0.0), max_neg_deformation_(0.0)
    {
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model_ = _model;
      this->ros_node_ = gazebo_ros::Node::Get(_sdf);

      // Joint name
      std::string joint_name = "shock_absorber_DF";
      if (_sdf->HasElement("joint_name")) {
        joint_name = _sdf->Get<std::string>("joint_name");
      }

      this->joint_ = _model->GetJoint(joint_name);
      if (!this->joint_) {
        RCLCPP_ERROR(this->ros_node_->get_logger(), 
                     "[CrushableJointPlugin] ERROR: Joint '%s' not found!", joint_name.c_str());
        return;
      }
      this->topic_name_ = "crushable_force_displacement";
      if (_sdf->HasElement("topic_name"))
      {
        this->topic_name_ = _sdf->Get<std::string>("topic_name");
      }

      // Joint parameters
      this->compression_dir_ = _sdf->HasElement("compression_dir") ? _sdf->Get<double>("compression_dir") : 1.0;
      this->rebound_damping_ = _sdf->HasElement("rebound_damping") ? _sdf->Get<double>("rebound_damping") : 1000.0;
      this->bidirectional_ = _sdf->HasElement("bidirectional") ? _sdf->Get<bool>("bidirectional") : false;      
      this->dual_stage_ = _sdf->HasElement("dual_stage") ? _sdf->Get<bool>("dual_stage") : false;
      this->stiffness_k_ = _sdf->HasElement("stiffness") ? _sdf->Get<double>("stiffness") : 5e6;
      this->damping_c_ = _sdf->HasElement("damping") ? _sdf->Get<double>("damping") : 15000.0;

      
      if (this->stiffness_k_ <= 0.0) {
          RCLCPP_WARN(this->ros_node_->get_logger(), 
                     "[CrushableJointPlugin] ERROR: stiffness (k) <= 0.");
          this->stiffness_k_ = 5e6;
      }

      if (this->dual_stage_) 
      {
          // Dual stage parameters
          this->crush_force_stage_1_ = _sdf->HasElement("crush_force_stage_1") ? _sdf->Get<double>("crush_force_stage_1") : 20000.0;
          this->stroke_limit_stage_1_ = _sdf->HasElement("stroke_limit_stage_1") ? _sdf->Get<double>("stroke_limit_stage_1") : 0.254;          
          this->crush_force_stage_2_ = _sdf->HasElement("crush_force_stage_2") ? _sdf->Get<double>("crush_force_stage_2") : 42200.0;
          this->total_stroke_limit_ = _sdf->HasElement("total_stroke_limit") ? _sdf->Get<double>("total_stroke_limit") : 0.8128;          
          this->bottomout_stiffness_ = _sdf->HasElement("bottomout_stiffness") ? _sdf->Get<double>("bottomout_stiffness") : 1e7;

          RCLCPP_INFO(this->ros_node_->get_logger(), 
                      "[CrushableJointPlugin] %s: DUAL STAGE ON | F1: %.0fN (<%.2fm) | F2: %.0fN (<%.2fm)", 
                      joint_name.c_str(), this->crush_force_stage_1_, this->stroke_limit_stage_1_, 
                      this->crush_force_stage_2_, this->total_stroke_limit_);
      } 
      else 
      {
          // Single stage parameters
          this->crush_force_ = _sdf->HasElement("crush_force") ? _sdf->Get<double>("crush_force") : 35000.0;
          RCLCPP_INFO(this->ros_node_->get_logger(), 
                      "[CrushableJointPlugin] %s: SINGLE STAGE | Force: %.0fN | Bidir: %s", 
                      joint_name.c_str(), this->crush_force_, this->bidirectional_ ? "SI" : "NO");
      }

      force_disp_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Point>(this->topic_name_, 100);
      RCLCPP_INFO(this->ros_node_->get_logger(), "[CrushableJointPlugin] Publishing on %s", this->topic_name_.c_str());
      // Physics connection
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&CrushableJointPlugin::OnUpdate, this));
    }

  private:  
    void OnUpdate()
    {
      if (!this->joint_) return;      

      double pos = this->joint_->Position(0);
      double vel = this->joint_->GetVelocity(0);
      double force_to_apply = 0.0;      
      
      // Unidirectional double stage
      if (this->dual_stage_) 
      {
          double current_compression = pos * this->compression_dir_;
          double comp_vel = vel * this->compression_dir_;
          
          double current_crush_limit = 0.0;
          if (current_compression >= this->total_stroke_limit_) {
              current_crush_limit = this->crush_force_stage_2_ + this->bottomout_stiffness_ * (current_compression - this->total_stroke_limit_);
          } else if (current_compression <= this->stroke_limit_stage_1_) {
              current_crush_limit = this->crush_force_stage_1_;
          } else {
              current_crush_limit = this->crush_force_stage_2_;
          }

          double elastic_force = this->stiffness_k_ * (current_compression - this->max_pos_deformation_);
          if (elastic_force < 0.0) elastic_force = 0.0;
          
          double applied_f = 0.0;
          
          if (elastic_force >= current_crush_limit) {
              applied_f = current_crush_limit;
              this->max_pos_deformation_ = current_compression - (current_crush_limit / this->stiffness_k_);
          } 
          
          else if (elastic_force > 0.0) {
              applied_f = elastic_force + (this->damping_c_ * comp_vel);              
              if (applied_f > current_crush_limit) applied_f = current_crush_limit;             
              if (applied_f < 0.0) applied_f = 0.0;
          } 
         
          else {
              applied_f = this->rebound_damping_ * comp_vel;
          }

          force_to_apply = -applied_f * this->compression_dir_;
      }
      
      // Bidirectional
      else if (this->bidirectional_) 
      {
          double elastic_pos = this->stiffness_k_ * (pos - this->max_pos_deformation_);
          if (elastic_pos < 0.0) elastic_pos = 0.0;
          
          double f_pos = 0.0;
          if (elastic_pos >= this->crush_force_) {
              f_pos = this->crush_force_;
              this->max_pos_deformation_ = pos - (this->crush_force_ / this->stiffness_k_);
          } else if (elastic_pos > 0.0) {
              f_pos = elastic_pos + (this->damping_c_ * vel);
              if (f_pos > this->crush_force_) f_pos = this->crush_force_;
              if (f_pos < 0.0) f_pos = 0.0;
          }

          double elastic_neg = this->stiffness_k_ * (this->max_neg_deformation_ - pos);
          if (elastic_neg < 0.0) elastic_neg = 0.0;
          
          double f_neg = 0.0;
          if (elastic_neg >= this->crush_force_) {
              f_neg = this->crush_force_;
              this->max_neg_deformation_ = pos + (this->crush_force_ / this->stiffness_k_);
          } else if (elastic_neg > 0.0) {
              f_neg = elastic_neg - (this->damping_c_ * vel); 
              if (f_neg > this->crush_force_) f_neg = this->crush_force_;
              if (f_neg < 0.0) f_neg = 0.0;
          }

          if (f_pos == 0.0 && f_neg == 0.0) {
              force_to_apply = -this->rebound_damping_ * vel;
          } else {
              force_to_apply = -f_pos + f_neg;
          }
      }
      
      // Unidirectional
      else 
      {
          double current_compression = pos * this->compression_dir_;
          double comp_vel = vel * this->compression_dir_;
          
          double elastic_force = this->stiffness_k_ * (current_compression - this->max_pos_deformation_);
          if (elastic_force < 0.0) elastic_force = 0.0;
          
          double applied_f = 0.0;

          if (elastic_force >= this->crush_force_) {
              applied_f = this->crush_force_;
              this->max_pos_deformation_ = current_compression - (this->crush_force_ / this->stiffness_k_);
          } else if (elastic_force > 0.0) {
              applied_f = elastic_force + (this->damping_c_ * comp_vel);
              if (applied_f > this->crush_force_) applied_f = this->crush_force_;
              if (applied_f < 0.0) applied_f = 0.0;
          } else {
              applied_f = this->rebound_damping_ * comp_vel;
          }

          force_to_apply = -applied_f * this->compression_dir_;
      }

      this->joint_->SetForce(0, force_to_apply);
      
      geometry_msgs::msg::Point msg;
      msg.x = pos;              
      msg.y = force_to_apply;   
      msg.z = vel;              
      force_disp_pub_->publish(msg);
    }

    physics::ModelPtr model_;
    physics::JointPtr joint_;
    std::string topic_name_;
    gazebo_ros::Node::SharedPtr ros_node_;
    event::ConnectionPtr update_connection_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr force_disp_pub_;   
    
    
    double crush_force_;
    double rebound_damping_;
    double compression_dir_;
    bool bidirectional_;
    double stiffness_k_;
    double damping_c_;    
    bool dual_stage_;
    double crush_force_stage_1_;
    double stroke_limit_stage_1_;
    double crush_force_stage_2_;
    double total_stroke_limit_;
    double bottomout_stiffness_;    
    double max_pos_deformation_;
    double max_neg_deformation_;
    
};

GZ_REGISTER_MODEL_PLUGIN(CrushableJointPlugin)

}