#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <functional>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class auvpush : public ModelPlugin
  {
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    /// \brief Constructor
    //public: thrusterControl() {}

    public: void Load(physics::ModelPtr _model,physics::LinkPtr _link, sdf::ElementPtr _sdf)
    {
      //output message:
      std::cerr << "\nThe thrusterControl plugin is attach to model[" <<_model->GetName() << "]\n";

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
      }

      this->model = _model;
      this->link=_link;
      //link
      this->link=this->model->GetChildLink("auv::auv::hull");

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =ros::SubscribeOptions::create<std_msgs::Float32>("/" + this->model->GetName() + "/vel_cmd",1,boost::bind(&auvpush::OnRosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =std::thread(std::bind(&auvpush::QueueThread, this));
    }

    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetForce(_msg->data);
      //this->link->AddRelativeForce(ignition::math::Vector3d(_msg->data, 0, 0));
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    public: void SetForce(const double &f)
    {
      this->link->AddRelativeForce(ignition::math::Vector3d(f, 0, 0));
    }

  };
  GZ_REGISTER_MODEL_PLUGIN(auvpush)
}
