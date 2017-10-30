#include <gazebo_ros_ir_camera_plugin/gazebo_ros_ir_emitter_model.h>
#include "gazebo/physics/physics.hh"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboRosIREmitter)


GazeboRosIREmitter::GazeboRosIREmitter()
{

}

GazeboRosIREmitter::~GazeboRosIREmitter(){}

void GazeboRosIREmitter::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

  this->model_ptr_ = _parent;
  this->world_ptr_ = _parent->GetWorld();

  if (!_sdf->HasElement("frameName"))
  {
    gzerr << "No <link> sdf elements found." << std::endl;
    return;
  }

  sdf::ElementPtr linkElem = _sdf->GetElement("frameName");

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("IR Emitter", "A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO("IR Emitter model initialized");

}

}
