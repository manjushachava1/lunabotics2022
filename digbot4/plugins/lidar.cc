#ifndef LAZ_SENSOR_PLUGIN
#define LAS_SENSOR_PLUGIN

// Include Gazebo headers.

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

// Include ROS headers so we can communicate with our robot
#include <ros/ros.h>

// Include std::string's because they're pretty darn useful.
#include <string>

// So we don't have to do gazebo::everything
namespace gazebo
{
  // Defining our plugin class
  class LazSensorPlugin : public RayPlugin
  {
    private:
    sensors::RaySensorPtr _sensor;

    private:
    physics::LinkPtr _l;

    public:
    LazSensorPlugin() {}

    // Runs when the model is loaded
    virtual void Load(sensors::SensorPtr _s, sdf::ElementPtr _sdf)
    {
       if(!ros::isInitialized)
       {
         ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed.");
         return;
       }
       RayPlugin::Load(_s, _sdf);

       _sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_s);
       ROS_INFO("Laser Sensor Plugin Loaded");

       std::string parentName = _sensor->ParentName();
       std::string modelName = "";
       std::string linkName = "";
       modelName = parentName.substr(0, parentName.find(':'));
       linkName = parentName.substr(parentName.find(':') + 2, parentName.size());

       physics::WorldPtr _w = physics::get_world(_sensor->WorldName());
       physics::ModelPtr _m = _w->Models(modelName.c_str());
       _l = _m->GetLink(linkName);
   }

    virtual void OnNewLaserScans()
    {
        std::string out = "";
        std::vector<double> ranges;
        _sensor->Ranges(ranges);
        double yaw = _sensor->Pose().Rot().Yaw() + _l->GetRelativePose().rot.GetYaw();
        for(int i = 0; i < ranges.size(); i++)
        {
          out = out + "[";
          out = out + std::to_string(i);
          out = out + "] ";
          out = out + std::to_string(ranges[i]);
          out = out + ", ";
        }
      
        ROS_INFO("%s", yaw, out.c_str()); 
    };
   };
  

  // Gazebo macro to set up the rest of the plugin functionality
  GZ_REGISTER_SENSOR_PLUGIN(LazSensorPlugin)
}

#endif


