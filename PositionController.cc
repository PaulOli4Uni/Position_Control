/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * gz topic -t /box -m gz.msgs.StringMsg -p 'data:"Text"'


 */

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include <tuple>
#include <utility>
#include <limits>
#include <chrono>
#include <iostream>
#include<sstream>


#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Util.hh"

#include "PositionController.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::PositionControllerPrivate
{
  // ------------------------- Funtion Declarations ------------------------
  /// \brief Callback for wrench subscription
  /// \param[in] _msg String message (Containing Pose Message)
  public: void OnPosePub(const msgs::StringMsg &_msg);

  /// \brief Callback for file with wrench info subscription
  /// \param[in] _msg String message (Containing File name)
  public: void OnFilePub(const msgs::StringMsg &_msg);

  public: void UpdateTargets(const std::string message);

  /// \brief Calculates the required force to move model to desired pose and updates the force and torque values
  /// \param[in] _msg String message (Containing Pose message)
  public: void UpdateForce(const gz::sim::UpdateInfo &_info);


  // -------------------------       Variables      ------------------------
  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};
    
  /// \brief Stores the Wrench and Force to apply to the model (once applied, it stores the values for later reference)
  public: math::Vector3d wrench_force = {0, 0, 0};
  public: math::Vector3d wrench_torque = {0, 0, 0};

  /// \brief Allow specifying constant xyz and rpy offsets
  public: gz::math::Pose3d pose_offset = {0, 0, 0, 0, 0, 0};

  /// \brief Specifies the xyz and rpy pose targets
  public: gz::math::Pose3d pose_target = {0, 0, 0, 0, 0, 0};   // <------------------ Might not need!

  /// \brief Specifies the time in which to reach the pose target
  public: float time_target = 0;   // <------------------ Might not need!

  /// \brief Specifies the time at which the pose target should be reached
  public: std::chrono::duration<double> time_destination;

  /// \brief Specifies the size of the timestep
  public: std::chrono::duration<double> time_step_size;

  /// \brief Specifies the number of timesteps before destination is reached to update the force and torque values
  // Larger value -> less precise. Smaller value -> less smooth
  public: int timestep_precision;

  /// \brief Shows if topic sent (through pose topic), true until destination reached
  public: bool on_topic_pose;

  /// \brief Shows if Wrench should be applied during pre-update. (True if it should be applied)
  public: bool apply_wrench;


  // -------------------------------------- Publisher Nodes
  /// \brief Gazebo communication node.
  public:  transport::Node node;
};

//////////////////////////////////////////////////
PositionController::PositionController()
    : dataPtr(std::make_unique<PositionControllerPrivate>())
{
}

//////////////////////////////////////////////////
void PositionController::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Position Controller plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  // ---------------------- Settings from the SDF file.
  // -Necessary

  if (_sdf->HasElement("timestep_precision"))
  {
    this->dataPtr->timestep_precision = _sdf->Get<int>("timestep_precision");
  }
  else {
    gzerr << "Timestep precision MUST be provided"
             << std::endl;
      return;
  }

  // -Optional
  // ---------Subscribe to Pose Topic
  std::vector<std::string> modelTopic_pose_subscriber;
  if (_sdf->HasElement("pose_topic"))
  {
    modelTopic_pose_subscriber.push_back(_sdf->Get<std::string>("pose_topic"));
  }
  else {
    modelTopic_pose_subscriber.push_back(
    "/model/" + this->dataPtr->model.Name(_ecm) + "/pos_contr");
  }
  
  auto modelT_pose_subscriber = validTopic(modelTopic_pose_subscriber);
  this->dataPtr->node.Subscribe(
      modelT_pose_subscriber, &PositionControllerPrivate::OnPosePub, this->dataPtr.get());
    gzmsg << "PositionController subscribing to String Messages (for single Pose msg) on Topic :["
        << modelT_pose_subscriber << "]"
        << std::endl;

  // ---------Subscribe to File Topic
  std::vector<std::string> modelTopic_file_subscriber;
  if (_sdf->HasElement("file_topic"))
  {
    modelTopic_file_subscriber.push_back(_sdf->Get<std::string>("file_topic"));
  }
  else {
    modelTopic_file_subscriber.push_back(
    "/model/" + this->dataPtr->model.Name(_ecm) + "/file_pos_contr");
  }
  
  auto modelT_file_subscriber = validTopic(modelTopic_file_subscriber);
  this->dataPtr->node.Subscribe(
      modelT_file_subscriber, &PositionControllerPrivate::OnFilePub, this->dataPtr.get());
    gzmsg << "PositionController subscribing to String Messages (for Pose file) on Topic :["
        << modelT_file_subscriber << "]"
        << std::endl;


  if (_sdf->HasElement("xyz_offset"))
  {
    this->dataPtr->pose_offset.Pos() = _sdf->Get<gz::math::Vector3d>(
      "xyz_offset");
  }

  if (_sdf->HasElement("rpy_offset"))
  {
    this->dataPtr->pose_offset.Rot() =
      gz::math::Quaterniond(_sdf->Get<gz::math::Vector3d>(
        "rpy_offset"));
  }

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty()) {
    this->dataPtr->canonicalLink = Link(links[0]);
  }

  // Get Time step size (Note that duration has to be parsed to a double)
  std::chrono::duration<double> e = _info.dt;
  this->dataPtr->time_step_size = e.count();
}

//////////////////////////////////////////////////
void PositionController::PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("PositionController::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  if (this->dataPtr->apply_wrench)
  {
    this->dataPtr->apply_wrench = false;
    this->dataPtr->UpdateForce(_info);
  }
  
  std::chrono::duration<double> ElapsedTime = _info.simTime;

  if (this->dataPtr->time_destination.count() == ElapsedTime.count())
  {
    gzmsg << "this is it it this is";
    gzmsg << this->dataPtr->time_destination.count() << "    " << ElapsedTime.count() << std::endl;
  }
  

/*
  math::Vector3d force;
  force.Set(1.0,0,0);
  math::Vector3d torque;
  this->dataPtr->canonicalLink.AddWorldWrench(_ecm, force, torque);
*/
  
  
  //std::chrono::duration<double> ElapsedTime = _info.simTime;
  if (ElapsedTime.count() == 1.0){
    math::Vector3d force;
    force.Set(1000.0,500,0);
    math::Vector3d torque;
    torque.Set(0,0,1000);
    this->dataPtr->canonicalLink.AddWorldWrench(_ecm, force, torque);
        gzmsg << "Wololololololololololololololololololo";
  }
  if (ElapsedTime.count() == 5.0){
    math::Vector3d force;
    force.Set(-1000.0,-500,0);
    math::Vector3d torque;
    torque.Set(0,0,-1000);
    this->dataPtr->canonicalLink.AddWorldWrench(_ecm, force, torque);
  }
  
  if (ElapsedTime.count() == 7.0){
    math::Vector3d force;
    force.Set(0,1000,100);
    math::Vector3d torque;
    //torque.Set(0,0,-1000);
    this->dataPtr->canonicalLink.AddWorldWrench(_ecm, force, torque);
  }

  ////// ------------------------------------ Pose implementation
  
  // std::stack<Entity> toCheck;
  // toCheck.push(this->model.Entity());

  // auto parent = _ecm.Component<components::ParentEntity>(entity);

  // for (const auto &entity : this->entitiesToPublish)
  // auto pose = _ecm.Component<components::Pose>(entity.first);

  
  // Create the pose component if it does not exist.
  auto pos = _ecm.Component<components::Pose>(
      this->dataPtr->model.Entity());
  if (!pos)
  {
    _ecm.CreateComponent(this->dataPtr->model.Entity(),
        components::Pose());
  }

  // Get and set robotBaseFrame to odom transformation.
  const math::Pose3d rawPose = worldPose(this->dataPtr->model.Entity(), _ecm);
  math::Pose3d pose = rawPose * this->dataPtr->pose_offset;
  //gzmsg << pose.Pos().X();
  //gzmsg << rawPose.Pos().X(); gzmsg << rawPose.Pos().Y(); gzmsg << rawPose.Pos().Z();
  //gzmsg << rawPose.Pos().Rot(); gzmsg << rawPose.Pos().Yaw(); 
  //gzmsg << pose.Rot().Yaw();
  //msg.mutable_pose()->mutable_position()->set_y(pose.Pos().Y());
  // msgs::Set(msg.mutable_pose()->mutable_orientation(), pose.Rot());
  // if (this->dimensions == 3)
  // {
  //   msg.mutable_pose()->mutable_position()->set_z(pose.Pos().Z());
  

}

//////////////////////////////////////////////////
void PositionController::PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm)
{
  GZ_PROFILE("PositionController::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;


}

//////////////////////////////////////////////////
void PositionControllerPrivate::OnPosePub(const msgs::StringMsg &_msg)
{
  //std::lock_guard<std::mutex> lock(this->mutex);
  //this->targetVel = _msg;
  this->on_topic_pose = true; this->apply_wrench = true;
  PositionControllerPrivate::UpdateTargets(_msg.data());
}

void PositionControllerPrivate::OnFilePub(const msgs::StringMsg &_msg)
{
  //std::lock_guard<std::mutex> lock(this->mutex);
  //this->targetVel = _msg;
  gzmsg << "File topic has been published to";
}


// Updates the force and torque values to be applied to the model.
// If message format is wrong -> model force will not be updated
void PositionControllerPrivate::UpdateTargets(const std::string message)
{
  //std::string message = _msg.data();
  // gzmsg << message << std::endl;

  std::vector<float> msg_vector;
  std::stringstream s_stream(message); //create string stream from the string
   while(s_stream.good()) {
      std::string substr;
      getline(s_stream, substr, ','); //get first string delimited by comma
      try {
        msg_vector.push_back(std::stof(substr)); // Push string to msg_result and change to float
      }
      catch (const std::invalid_argument& e){
        gzerr << "Invalid argument type given (must be of type float)";
        return;
      }
      
   }   
   if (msg_vector.size() != 7){
    gzerr << "Wrong number of variables have been provided";
    return;
   }

  this->pose_target.Set(msg_vector[0], msg_vector[1], msg_vector[2], msg_vector[3], msg_vector[4], msg_vector[5]);
  this->time_target = msg_vector[6];
  
}

void PositionControllerPrivate::UpdateForce(const gz::sim::UpdateInfo &_info)
{
    
    using fsec = std::chrono::duration<float>;
    auto num = std::chrono::round<std::chrono::nanoseconds>(fsec{this->time_target});
    this->time_destination = _info.simTime + num;
  //gzmsg << this->time_destination;
   //std::chrono::duration<double> ElapsedTime = this->dataptr->_info.simTime;

}
//////////////////////////////////////////////////

GZ_ADD_PLUGIN(PositionController,
              gz::sim::System,
              PositionController::ISystemConfigure,
              PositionController::ISystemPreUpdate,
              PositionController::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(PositionController,
                    "gz::sim::systems::PositionController")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(PositionController,
                    "ignition::gazebo::systems::PositionController")
