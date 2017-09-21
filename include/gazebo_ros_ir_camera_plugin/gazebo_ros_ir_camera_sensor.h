#ifndef GAZEBO_ROS_RFID_SENSOR_H
#define GAZEBO_ROS_RFID_SENSOR_H

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <gazebo/msgs/logical_camera_image.pb.h>
#include <gazebo_plugins/gazebo_ros_utils.h>


namespace gazebo
{

class GazeboRosIRCamera : public SensorPlugin 
{
public:
  /// \brief Constructor.
  GazeboRosIRCamera();

  /// \brief Destructor.
  virtual ~GazeboRosIRCamera();

  /// \brief Load the sensor.
  /// \param _parent pointer to the sensor.
  /// \param _sdf pointer to the sdf config file.
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

protected:
	/// \brief Gazebo world pointer
	physics::WorldPtr world_ptr_;

	/// \brief parent logical camera sensor
	sensors::SensorPtr parent_sensor_;

  /// \brief Update the sensor.
  virtual void onNewScan(ConstLogicalCameraImagePtr &_msg);

private:
  /// \brief Gaussian noise generator.
  /// \param mu offset value.
  /// \param sigma scaling value.
  double GuassianKernel(double mu, double sigma);

  /// \brief Check the model is with the range of the camera
  /// \param pose position of the model.
  double GetRangeToSensor(const math::Vector3 &pose);

  /// \brief Check the model has IR emmitter attached
  /// \param nested model of the detected model.
  bool CheckContainIREmitter(const physics::ModelPtr &model_p);

  /// \brief ROS NodeHandle pointer.
  ros::NodeHandle* ros_nh_;
  /// \brief ROS Publisher for rfid sensor data.
  ros::Publisher scan_pub_;

  /// \brief frame name for the attached link
  std::string frame_name_;

	/// \brief Node for communicate with gazebo
	transport::NodePtr gz_node_;
	/// \brief Subscribe to the logical camera in gazebo
	transport::SubscriberPtr image_sub_;

	/// \brief namespace for the plugin
	std::string robot_namespace_;
	/// \brief name for advertising the topic
	std::string topic_name_;

};

}


#endif // GAZEBO_ROS_RFID_SENSOR_H
