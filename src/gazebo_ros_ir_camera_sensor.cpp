#include <gazebo/sensors/SensorManager.hh>

#include <gazebo_ros_ir_camera_plugin/gazebo_ros_ir_camera_sensor.h>
#include <gazebo_ir_camera_plugin/IRCamera.h>


namespace gazebo
{

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosIRCamera)

GazeboRosIRCamera::GazeboRosIRCamera():
  ir_camera_connect_count_(0)
{

}

GazeboRosIRCamera::~GazeboRosIRCamera()
{
  this->parent_sensor_->SetActive(false);
  this->camera_queue_.clear();
  this->camera_queue_.disable();
  this->ros_nh_->shutdown();
  delete this->ros_nh_;
}

void GazeboRosIRCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent_sensor_ = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_parent);
	if (!this->parent_sensor_)
	{
		ROS_ERROR_NAMED("ir_camera", "IR camera parent is not LogicalCameraSensor");
		return;
	}

	this->world_ptr_ = physics::get_world(parent_sensor_->WorldName());

	this->robot_namespace_ = "";
	if (_sdf->HasElement("robotNamespace"))
	{
		this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
	}

  this->topic_name_ = "ir_camera_topic";
	if (_sdf->HasElement("topicName"))
	{
		this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
	}

  this->frame_name_ = "ir_camera_link";
  if (_sdf->HasElement("frameName"))
  {
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  this->ros_nh_ = new ros::NodeHandle(this->robot_namespace_ + this->parent_sensor_->Name());

  this->deferred_load_thread_ = boost::thread(
        boost::bind(&GazeboRosIRCamera::LoadThread, this));

//  this->parent_sensor_->SetActive(true);

  ROS_INFO_STREAM("Name: " << this->parent_sensor_->Name() <<
                  ", Subscribling to Gazebo topic: " << this->parent_sensor_->Topic());
}

void GazeboRosIRCamera::LoadThread()
{
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
                     << "unable to load plugin. Load the Gazebo system plugin "
                     << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->parent_sensor_->SetActive(false);

  this->gz_node_ = transport::NodePtr(new transport::Node());
  this->gz_node_->Init(this->parent_sensor_->WorldName());

  this->callback_queue_thread_ = boost::thread(
        boost::bind(&GazeboRosIRCamera::CameraQueueThread, this));

  this->scan_pub_ = this->ros_nh_->advertise<gazebo_ir_camera_plugin::IRCamera>(
        ros_nh_->getNamespace()+this->topic_name_, 1, true);
//  load_connection_.Connect(boost::bind(&GazeboRosIRCamera::Advertise, this));
//  load_connection_();
//  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<gazebo_ir_camera_plugin::IRCamera>(
//        this->topic_name_, 1,
//        boost::bind(&GazeboRosIRCamera::IRImageConnect, this),
//        boost::bind(&GazeboRosIRCamera::IRImageDisconnect, this),
//        ros::VoidPtr(), &this->camera_queue_);

//  this->scan_pub_ = this->ros_nh_->advertise(ao);
  ROS_INFO_STREAM("Publishing to ROS topic: " << this->scan_pub_.getTopic());

  this->image_sub_ = this->gz_node_->Subscribe(this->parent_sensor_->Topic(),
                                               &GazeboRosIRCamera::onNewScan, this);
  this->parent_sensor_->SetActive(true);
}

void GazeboRosIRCamera::Advertise()
{
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<gazebo_ir_camera_plugin::IRCamera>(
        this->topic_name_, 1,
        boost::bind(&GazeboRosIRCamera::IRImageConnect, this),
        boost::bind(&GazeboRosIRCamera::IRImageDisconnect, this),
        ros::VoidPtr(), &this->camera_queue_);

  this->scan_pub_ = this->ros_nh_->advertise(ao);

  ROS_INFO_STREAM("Publishing to ROS topic: " << this->scan_pub_.getTopic());
}

void GazeboRosIRCamera::IRImageConnect()
{
  this->ir_camera_connect_count_++;
  this->parent_sensor_->SetActive(true);
}

void GazeboRosIRCamera::IRImageDisconnect()
{
  this->ir_camera_connect_count_--;
  if (this->ir_camera_connect_count_ <= 0)
    this->parent_sensor_->SetActive(false);
}

void GazeboRosIRCamera::onNewScan(ConstLogicalCameraImagePtr& _msg)
{
  gazebo_ir_camera_plugin::IRCamera ir_cam_msg;
  ir_cam_msg.header.stamp = ros::Time::now();
  ir_cam_msg.header.frame_id = this->frame_name_;

//  ROS_INFO_STREAM("scanned model: " << _msg->model_size());
  math::Pose model_pose;
  for (int i = 0; i < _msg->model_size(); i++)
  {
    std::string model_name = _msg->model(i).name();
    auto model_ptr = this->world_ptr_->GetModel(model_name);

//    ROS_INFO_STREAM(_msg->model(i).name());
    // check has IR LEDs
    if (CheckContainIREmitter(model_ptr))
    {
      math::Vector3 model_position = math::Vector3(msgs::ConvertIgn(_msg->model(i).pose().position()));
      math::Quaternion model_orientation = math::Quaternion(msgs::ConvertIgn(_msg->model(i).pose().orientation()));
      model_pose = math::Pose(model_position, model_orientation);

      double dir = GetRangeToSensor(model_position);
      if (dir != -100000)
      {
        ir_cam_msg.azimuthal_angles.push_back(dir);
        this->scan_pub_.publish(ir_cam_msg);
      }
    }
  }
}

void GazeboRosIRCamera::CameraQueueThread()
{
  static const double timeout = 0.001;
  while (this->ros_nh_->ok())
  {
    /// take care of callback queue
    this->camera_queue_.callAvailable(ros::WallDuration(timeout));
}
}

double GazeboRosIRCamera::GetRangeToSensor(const math::Vector3 &pose)
{
  math::Vector3 sensor_pos = this->parent_sensor_->Pose().Pos();
  double dist = sensor_pos.Distance(pose);

  // check the object is inside the camera image
  double azi_angle = -100000;
  if (dist < 3.0 && dist > 0.01)
  {
    math::Vector3 relative_pos = pose - this->parent_sensor_->Pose().Pos();
    azi_angle = std::atan2(relative_pos[1], relative_pos[0]);
  }

  return azi_angle;
}

bool GazeboRosIRCamera::CheckContainIREmitter(const physics::ModelPtr &model_p)
{
  bool found = false;
  sensors::SensorManager *sensorManager = sensors::SensorManager::Instance();
  for (physics::LinkPtr link : model_p->GetLinks())
  {
    for (unsigned int i = 0; i < link->GetSensorCount(); i++)
    {
      sensors::SensorPtr sensor_p = sensorManager->GetSensor(link->GetSensorName(i));
      if (sensor_p->Type() == "ray")
      {
        found = true;
        return found;
      }
    }
  }
  return found;
}

}
