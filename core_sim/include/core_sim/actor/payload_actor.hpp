// Copyright (C) Microsoft Corporation. All rights reserved.

#ifndef CORE_SIM_INCLUDE_CORE_SIM_ACTOR_PAYLOAD_ACTOR_HPP_
#define CORE_SIM_INCLUDE_CORE_SIM_ACTOR_PAYLOAD_ACTOR_HPP_

#include <memory>
#include <vector>

#include "core_sim/actor.hpp"
#include "core_sim/clock.hpp"
#include "core_sim/earth_utils.hpp"
#include "core_sim/environment.hpp"
#include "core_sim/joint.hpp"
#include "core_sim/link.hpp"
#include "core_sim/logger.hpp"
#include "core_sim/message/pose_message.hpp"
#include "core_sim/physics_common_types.hpp"

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

  typedef std::function<void(const Kinematics&, TimeNano)> KinematicsCallback;

  //---------------------------------------------------------------------------
  // Static config/model
  //
  const std::vector<Link>& GetLinks() const;
  const std::vector<Joint>& GetJoints() const;

  bool GetStartLanded() const;
  void SetStartLanded(bool start_landed);

  //---------------------------------------------------------------------------
  // Runtime
  //

  const Kinematics& GetKinematics() const;
  void UpdateKinematics(const Kinematics& kinematics);
  const Environment& GetEnvironment() const;
  void UpdateEnvironment();
  const Pose& GetPoseOffset() const;
  void SetPoseOffset(const Pose& offset);

  const CollisionInfo& GetCollisionInfo() const;
  void UpdateCollisionInfo(const CollisionInfo& collision_info);
  void SetHasCollided(bool has_collided);

  void SetCallbackKinematicsUpdated(const KinematicsCallback& callback);

  const bool InAttachedState() const;
  void SetAttachedState(const bool to_attach);

  const Wrench& GetDragFaceWrench() const;
  void SetDragFaceWrench(const Wrench& drag_wrench);
  const Matrix3x3& GetInertiaMatrix() const;
  const float GetMass() const;
  void SetMass(const float mass);

  void BeginUpdate();
  void EndUpdate();

 private:
  friend class Scene;
  friend class Robot;

  PayloadActor(const std::string& id, const Transform& origin,
               const Logger& logger, const TopicManager& topic_manager,
               const std::string& parent_topic_path,
               const ServiceManager& service_manager,
               const StateManager& state_manager, HomeGeoPoint home_geo_point);

  void Load(ConfigJson config_json) override;

  const Kinematics& ComputeAttachedKinematics(
      const Kinematics& kinematics) const;
  void ComputeInertiaMatrix();

  class Impl;
  class Loader;
};

}  // namespace projectairsim
}  // namespace microsoft

#endif  // CORE_SIM_INCLUDE_CORE_SIM_ACTOR_PAYLOAD_ACTOR_HPP_