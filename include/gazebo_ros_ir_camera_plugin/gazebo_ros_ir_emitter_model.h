#ifndef GAZEBO_ROS_IR_EMITTER_MODEL_H
#define GAZEBO_ROS_IR_EMITTER_MODEL_H

#include <ros/ros.h>

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{

class GazeboRosIREmitter : public ModelPlugin
{

public:

  GazeboRosIREmitter();

  ~GazeboRosIREmitter();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:

  void onUpdate();

  std::vector<event::ConnectionPtr> connections;

  physics::ModelPtr model_ptr_;

  physics::WorldPtr world_ptr_;

};

}

#endif // GAZEBO_ROS_IR_EMITTER_MODEL_H
