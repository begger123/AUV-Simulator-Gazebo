#ifndef _AUV_PLUGIN_HH_
#define _AUV_PLUGIN_HH_

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
#include<string>
#include<iostream>
#include<auvsim_gazebo/ThrusterSpeeds.h>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class AuvPlugin : public ModelPlugin
  {
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;
    private: physics::LinkPtr thrusters[6];
    private: std::string path="auv::auv::thruster_";
    private: std::string k;

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      //output message:
      std::cerr << "\nThe Auv plugin is attach to model[" <<_model->GetName() << "]\n";

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
      }

      this->model = _model;

      for(int i=0;i<6;i++){
        k=std::to_string(i+1);
        this->thrusters[i]=_model->GetChildLink(path+k);
      }

      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =ros::SubscribeOptions::create<auvsim_gazebo::ThrusterSpeeds>("/thruster_speeds",1,boost::bind(&AuvPlugin::OnRosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =std::thread(std::bind(&AuvPlugin::QueueThread, this));
    }

    public: void OnRosMsg(const auvsim_gazebo::ThrusterSpeeds::ConstPtr &_msg)
    {
      std::cerr << "\nThe force value is [" <<_msg->data[0]<< "]\n";
      this->thrusters[0]->AddRelativeForce(ignition::math::Vector3d(_msg->data[0], 0, 0));
      this->thrusters[1]->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
      this->thrusters[2]->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
      this->thrusters[3]->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
      this->thrusters[4]->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
      this->thrusters[5]->AddRelativeForce(ignition::math::Vector3d(_msg->reverse[0], 0, 0));

      //this->SetForce(_msg->data);
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

  /*public: void SetForce(const double &f)
    {

      this->thrusters[0]->AddRelativeForce(ignition::math::Vector3d(f, 0, 0));
      this->thrusters[1]->AddRelativeForce(ignition::math::Vector3d(f, 0, 0));
      this->thrusters[2]->AddRelativeForce(ignition::math::Vector3d(f, 0, 0));
      this->thrusters[3]->AddRelativeForce(ignition::math::Vector3d(f, 0, 0));
      this->thrusters[4]->AddRelativeForce(ignition::math::Vector3d(f, 0, 0));
      this->thrusters[5]->AddRelativeForce(ignition::math::Vector3d(f, 0, 0));
    }*/
  };
  GZ_REGISTER_MODEL_PLUGIN(AuvPlugin)
}
#endif
