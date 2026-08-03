#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh> 
#include <gazebo/physics/Collision.hh>      
#include <gazebo/physics/ode/ODESurfaceParams.hh> 
#include <gazebo/common/common.hh>
#include <vector>
#include <string>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
  class DynamicTerrainPlugin : public WorldPlugin
  {
    private: physics::WorldPtr world;
    private: event::ConnectionPtr updateConnection;
    private: gazebo_ros::Node::SharedPtr ros_node_;

    
    private: double kp_surface = 10000.0;
    private: double kd_surface = 100.0;
    private: double mu_surface = 0.5;    
   
    private: double kp_rock = 5000000.0;
    private: double kd_rock = 50000.0;
    private: double mu_rock = 1;
    
    private: double dust_thickness = 0.05; 

    private: std::string terrain_name = "terrain_model";

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      this->world = _world;
      this->ros_node_ = gazebo_ros::Node::Get(_sdf);
      
      if (_sdf->HasElement("terrain_name")) this->terrain_name = _sdf->Get<std::string>("terrain_name");
      if (_sdf->HasElement("kp_surface")) this->kp_surface = _sdf->Get<double>("kp_surface");
      if (_sdf->HasElement("kp_rock")) this->kp_rock = _sdf->Get<double>("kp_rock");
      if (_sdf->HasElement("kd_surface")) this->kd_surface = _sdf->Get<double>("kd_surface");
      if (_sdf->HasElement("kd_rock")) this->kd_rock = _sdf->Get<double>("kd_rock");
      if (_sdf->HasElement("dust_thickness")) this->dust_thickness = _sdf->Get<double>("dust_thickness");
      if (_sdf->HasElement("mu_surface")) this->mu_surface = _sdf->Get<double>("mu_surface");
      if (_sdf->HasElement("mu_rock")) this->mu_rock = _sdf->Get<double>("mu_rock");
      
      
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&DynamicTerrainPlugin::OnUpdate, this));      
      RCLCPP_INFO(this->ros_node_->get_logger(), "[DynamicTerrainPlugin] Loaded on: %s", this->terrain_name.c_str());
    }

    public: void OnUpdate()    {
      
      auto contacts = this->world->Physics()->GetContactManager()->GetContacts();

      for (auto const &contact : contacts)
      {        
        physics::Collision *terrain_collision = nullptr;        
        if (contact->collision1->GetModel()->GetName() == this->terrain_name) {
            terrain_collision = contact->collision1;
        } else if (contact->collision2->GetModel()->GetName() == this->terrain_name) {
            terrain_collision = contact->collision2;
        }
        
        if (terrain_collision != nullptr)
          {            
            double depth = 0.0;
            for (int i = 0; i < contact->count; ++i)
            {
              depth += contact->depths[i];
            }
            if (contact->count > 0) depth /= contact->count;
            
            double alpha = depth / this->dust_thickness;
            if (alpha > 1.0) alpha = 1.0; 
            if (alpha < 0.0) alpha = 0.0;
            
            double current_kp = this->kp_surface + alpha * (this->kp_rock - this->kp_surface);
            double current_kd = this->kd_surface + alpha * (this->kd_rock - this->kd_surface);
            double current_mu = this->mu_surface + alpha * (this->mu_rock - this->mu_surface);
            
            physics::ODESurfaceParams *ode_surface = dynamic_cast<physics::ODESurfaceParams*>(terrain_collision->GetSurface().get());
            if (ode_surface) {
                ode_surface->kp = current_kp;
                ode_surface->kd = current_kd;
            }

            auto friction = terrain_collision->GetSurface()->FrictionPyramid();
            if (friction) {
                friction->SetMuPrimary(current_mu);   
                friction->SetMuSecondary(current_mu); 
            }

            // Debug
            // static int debug_counter = 0;
            // if (debug_counter % 500 == 0) 
            // {
            //  RCLCPP_INFO(this->ros_node_->get_logger(), 
            //    "[DEBUG Terreno] Profondita: %.4f m | Alpha: %.2f | KP: %.1f | KD: %.1f | MU: %.2f", 
            //    depth, alpha, current_kp, current_kd, current_mu);
            //}
            // debug_counter++;            
          }
      }
    }
  };
  
  GZ_REGISTER_WORLD_PLUGIN(DynamicTerrainPlugin)
}