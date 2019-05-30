/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
*/

#include <string>

#include <ignition/math/OrientedBox.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport/Node.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/common/UpdateInfo.hh"

#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/World.hh"


#include "ContainPlugin.hh"

namespace gazebo
{
  namespace physics
  {
    typedef boost::weak_ptr<physics::Entity> EntityWeakPtr;
  }

  /// \brief Private data class for the ContainPlugin class
  class ContainPluginPrivate
  {
    /// \brief Connection to world update.
    public: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the world.
    public: physics::WorldPtr world;

    /// \brief Scoped name of the entity we're checking.
    public: std::vector<std::string> entityNames;

    /// \brief Pointer to the entity we're checking.
    public: std::vector<physics::EntityWeakPtr> entities;

    /// \brief Box representing the volume to check.
    public: ignition::math::OrientedBoxd box;

    /// \brief pointer to an entity whose pose the geometry will track
    public: physics::EntityWeakPtr containerEntity;

    /// \brief scoped name of entity to track
    public: std::string containerEntityName;

    /// \brief Ignition transport node for communication
    public: ignition::transport::Node ignNode;

    /// \brief Publisher which publishes contain / doesn't contain messages.
    public: ignition::transport::Node::Publisher containIgnPub;

    /// \brief Namespace for the topics:
    /// /<ns>/contain
    /// /<ns>/enable
    public: std::string ns;

    /// \brief 1 if contains, 0 if doesn't contain, -1 if unset
    public: std::vector<int> contain;

    /// /brief update period for the plugin
    public: common::Time updatePeriod; 
    /// /brief update period for the plugin
    public: common::Time lastUpdateTime; 

  };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ContainPlugin)

/////////////////////////////////////////////////
ContainPlugin::ContainPlugin() : WorldPlugin(),
    dataPtr(new ContainPluginPrivate)
{
}

/////////////////////////////////////////////////
void ContainPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
   // Get new commands/state
  callback_queue_.callAvailable();
    
  // Entity name
  if (!_sdf->HasElement("entity"))
  {
    gzerr << "Missing required parameter <entity>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }
  std::stringstream entity_stream(_sdf->Get<std::string>("entity"));
  std::string buffer;
  while(entity_stream >> buffer) {
    this->dataPtr->entityNames.push_back(buffer);
    this->dataPtr->contain.push_back(-1);
  }
  this->dataPtr->entities.resize(this->dataPtr->entityNames.size());
  for(int i=0; i<this->dataPtr->entityNames.size(); i++) {
    gzmsg << "Loaded entity " << this->dataPtr->entityNames.at(i) << std::endl;
  }
  

  // Namespace
  if (!_sdf->HasElement("namespace"))
  {
    gzerr << "Missing required parameter <namespace>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }
  this->dataPtr->ns = _sdf->Get<std::string>("namespace");

  // Pose
  if (!_sdf->HasElement("pose"))
  {
    gzerr << "Missing required parameter <pose>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }
  auto pose = _sdf->Get<ignition::math::Pose3d>("pose");
  sdf::ParamPtr frameParam = _sdf->GetElement("pose")->GetAttribute("frame");
  if (frameParam)
  {
    this->dataPtr->containerEntityName = frameParam->GetAsString();
  }

  // Geometry
  if (!_sdf->HasElement("geometry"))
  {
    gzerr << "Missing required parameter <geometry>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }
  auto geometryElem = _sdf->GetElement("geometry");

  // Only box for now
  if (!geometryElem->HasElement("box"))
  {
    gzerr << "Missing required parameter <box>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }
  auto boxElem = geometryElem->GetElement("box");

  if (!boxElem->HasElement("size"))
  {
    gzerr << "Missing required parameter <size>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }
  auto size = boxElem->Get<ignition::math::Vector3d>("size");

  this->dataPtr->box = ignition::math::OrientedBoxd(size, pose);

  this->dataPtr->world = _world;

  // Start/stop
  auto enableService = "/" + this->dataPtr->ns + "/enable";

  // Ignition transport service
  this->dataPtr->ignNode.Advertise(enableService,
      &ContainPlugin::EnableIgn, this);

  auto enabled = true;
  if (_sdf->HasElement("enabled"))
    enabled = _sdf->Get<bool>("enabled");
  
  // robot_namespace_
  if (!_sdf->HasElement("robot_namespace"))
  {
    gzerr << "Missing required parameter <robot_namespace>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }
  robot_namespace_ = _sdf->Get<std::string>("robot_namespace");

  if (!_sdf->HasElement("rate"))
  {
    this->dataPtr->updatePeriod = 1.0;
  }
  else {
    double rate = _sdf->Get<double>("rate");
    this->dataPtr->updatePeriod = 1.0/rate;
  }
  
  // robot_namespace_
  if (!_sdf->HasElement("contain_topic"))
  {
    gzerr << "Missing required parameter <contain_topic>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }
  contain_topic_ = _sdf->Get<std::string>("contain_topic");
  
  //ROS things go here
  if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("GMStatusPlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  
  // ROS Publisher
    if (!contain_topic_.empty())
    {
        ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<std_msgs::String>(
            contain_topic_, 1,
            ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), &callback_queue_);
        contain_publisher_ = this->rosnode_->advertise(ops);
    }

  if (enabled)
  {
    this->Enable(true);
  }

  // initialize the time of the last update
  this->dataPtr->lastUpdateTime = 0.0;
}

//////////////////////////////////////////////////
bool ContainPlugin::EnableIgn(const ignition::msgs::Boolean &_req,
    ignition::msgs::Boolean &_res)
{
  bool result = this->Enable(_req.data());
  _res.set_data(result);
  return result;
}

//////////////////////////////////////////////////
bool ContainPlugin::Enable(const bool _enable)
{
  // Already started
  if (_enable && this->dataPtr->updateConnection)
  {
    gzwarn << "Contain plugin is already enabled." << std::endl;
    return false;
  }

  // Already stopped
  if (!_enable && !this->dataPtr->updateConnection)
  {
    gzwarn << "Contain plugin is already disabled." << std::endl;
    return false;
  }

  // Start
  if (_enable)
  {
    // Start update
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ContainPlugin::OnUpdate, this, std::placeholders::_1));

    auto topic = "/" + this->dataPtr->ns + "/contain";

    this->dataPtr->containIgnPub =
        this->dataPtr->ignNode.Advertise<ignition::msgs::Boolean>(topic);

    gzmsg << "Started contain plugin [" << this->dataPtr->ns << "]"
          << std::endl;

    return true;
  }

  // Stop
  {
    this->dataPtr->updateConnection.reset();
    this->dataPtr->containIgnPub = ignition::transport::Node::Publisher();
    for(int i=0; i<this->dataPtr->contain.size(); i++) {
      this->dataPtr->contain.at(i) = -1;
    }

    gzmsg << "Stopped contain plugin [" << this->dataPtr->ns << "]"
          << std::endl;

    return true;
  }
}

/////////////////////////////////////////////////
void ContainPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // Only update if enought time has passed
  if (this->dataPtr->world->SimTime() - this->dataPtr->lastUpdateTime < this->dataPtr->updatePeriod)
  {
    return;
  }
  // set last update time
  this->dataPtr->lastUpdateTime = this->dataPtr->world->SimTime();
  // Only get the entity once
  for(int i=0; i<this->dataPtr->entityNames.size(); i++) {

    physics::EntityPtr entity = this->dataPtr->entities.at(i).lock();
    if (!entity)
    {
      this->dataPtr->entities.at(i) = this->dataPtr->world->EntityByName(
          this->dataPtr->entityNames.at(i));
      entity = this->dataPtr->entities.at(i).lock();
      if (!entity)
      {
        // Could not find entity being tested
        this->PublishContains(false, i);
        continue;
      }
    }

    ignition::math::Vector3d entityInWorldFrame = entity->WorldPose().Pos();

    ignition::math::Vector3d entityInBoxFrame;
    if (!this->dataPtr->containerEntityName.empty())
    {
      physics::EntityPtr referenceEntity = this->dataPtr->containerEntity.lock();
      // box is in a potentially moving reference frame
      if (!referenceEntity)
      {
        this->dataPtr->containerEntity = this->dataPtr->world->EntityByName(
          this->dataPtr->containerEntityName);
        referenceEntity = this->dataPtr->containerEntity.lock();
        if (!referenceEntity)
        {
          // Could not find reference entity
          this->PublishContains(false, i);
          continue;
        }
      }

      auto worldToBox = referenceEntity->WorldPose();
      auto boxToWorld = worldToBox.Inverse();
      // Transform the entity vector from world frame to the frame the box is in
      entityInBoxFrame = (boxToWorld.Rot() * entityInWorldFrame)
        + boxToWorld.Pos();
    }
    else
    {
      // box frame is world frame
      entityInBoxFrame = entityInWorldFrame;
    }

    this->PublishContains(this->dataPtr->box.Contains(entityInBoxFrame), i);
    }
}

//////////////////////////////////////////////////
void ContainPlugin::PublishContains(const bool _contains, int i)
{
  
  int containNow = _contains ? 1 : 0;
  // check if the status of the container has changed
  if (containNow != this->dataPtr->contain.at(i))
  {
    this->dataPtr->contain.at(i) = containNow;
    // publish message only if the entity has just entered the container
    if(containNow==1)
    {
      std_msgs::String msg;
      msg.data = this->dataPtr->entityNames.at(i);
      this->contain_publisher_.publish(msg);
    }
    else{
      // gzmsg << "Entity " << this->dataPtr->entityNames.at(i) << " has left the building" << std::endl;
    }
  }
}
