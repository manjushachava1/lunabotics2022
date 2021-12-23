#ifndef ROVER_PLUGIN
#define ROVER_PLUGIN

// Include Gazebo headers.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Include ROS headers so we can communicate with our robot
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"

// Include std::string's because they're pretty darn useful.
#include <string>

#include <cmath>

#define PI 3.14159265

#define SGN(a) (a >= 0 ? 1.0 : -1.0)

#define MAX_DIF_V 0.02
#define DIF_MAX_EFFORT 5

#define MAX_LIN_V 0.0015
#define LIN_MAX_EFFORT 5
namespace gazebo
{
  // Defining our plugin class
  class RoverPlugin : public ModelPlugin
  {
    private:

    public:
    RoverPlugin() {}
    
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
       if(!ros::isInitialized){
         ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed.");
         return;
       }
       ROS_INFO("Rover Plugin Loaded");
       
    }	
  };

  // Gazebo macro to set up the rest of the plugin functionality
  GZ_REGISTER_MODEL_PLUGIN(RoverPlugin)
}

#endif
