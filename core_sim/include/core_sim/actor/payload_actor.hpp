// Copyright (C) Microsoft Corporation. All rights reserved.

#ifndef CORE_SIM_INCLUDE_CORE_SIM_ACTOR_PAYLOAD_ACTOR_HPP_
#define CORE_SIM_INCLUDE_CORE_SIM_ACTOR_PAYLOAD_ACTOR_HPP_

#include <memory>
#include <vector>

#include "core_sim/actor.hpp"
#include "core_sim/clock.hpp"
#include "core_sim/earth_utils.hpp"
#include "core_sim/joint.hpp"
#include "core_sim/link.hpp"
#include "core_sim/logger.hpp"

namespace microsoft {
namespace projectairsim {

class ConfigJson;
class Logger;
class TopicManager;
class ServiceManager;
class StateManager;

class PayloadActor : public Actor {
 public:
  PayloadActor();

  typedef microsoft::projectairsim::Transform Pose;

  //---------------------------------------------------------------------------
  // Static config/model
  //
  const std::vector<Link>& GetLinks() const;
  const std::vector<Joint>& GetJoints() const;

  const PhysicsType& GetPhysicsType() const;

  //---------------------------------------------------------------------------
  // Runtime
  //

  const Kinematics& GetKinematics() const;
  void SetKinematics(const Kinematics& kinematics);
  const Pose& GetPoseOffset() const;

  const CollisionInfo& GetCollisionInfo() const;
  void UpdateCollisionInfo(const CollisionInfo& collision_info);
  void SetHasCollided(bool has_collided);

  void BeginUpdate();
  void EndUpdate();

 private:
  friend class Scene;

  PayloadActor(const std::string& id, const Pose& origin, const Logger& logger,
               const TopicManager& topic_manager,
               const std::string& parent_topic_path,
               const ServiceManager& service_manager,
               const StateManager& state_manager);

  void Load(ConfigJson config_json) override;

  class Impl;
  class Loader;
};

}  // namespace projectairsim
}  // namespace microsoft

#endif  // CORE_SIM_INCLUDE_CORE_SIM_ACTOR_PAYLOAD_ACTOR_HPP_