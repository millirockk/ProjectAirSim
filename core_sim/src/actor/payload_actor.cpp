// Copyright (C) Microsoft Corporation. All rights reserved.

#include "core_sim/actor/payload_actor.hpp"

#include <memory>

#include "actor_impl.hpp"
#include "constant.hpp"
#include "core_sim/logger.hpp"
#include "core_sim/message/collision_info_message.hpp"
#include "core_sim/message/kinematics_message.hpp"
#include "core_sim/runtime_components.hpp"
#include "json.hpp"

namespace microsoft {
namespace projectairsim {

using json = nlohmann::json;

class PayloadActor::Loader {
 public:
  explicit Loader(PayloadActor::Impl& impl);

  void Load(const json& json);

 private:
  void LoadLinks(const json& json);

  Link LoadLink(const json& json);

  void LoadJoints(const json& json);

  Joint LoadJoint(const json& json);

  PayloadActor::Impl& impl_;
};

class PayloadActor::Impl : public ActorImpl {
 public:
  Impl(const std::string& id, const Transform& origin, const Logger& logger,
       const TopicManager& topic_manager, const std::string& parent_topic_path,
       const ServiceManager& service_manager,
       const StateManager& state_manager);

  void Load(ConfigJson config_json);
  const std::vector<Link>& GetLinks();
  const std::vector<Joint>& GetJoints();

  const CollisionInfo& GetCollisionInfo() const;
  void UpdateCollisionInfo(const CollisionInfo& collision_info);
  void SetHasCollided(bool has_collided);

  const Pose& GetPoseOffset() const;
  void SetPoseOffset(const Pose& offset);
  const Environment& GetEnvironment() const;
  void UpdateEnvironment();

  void LoadKinematics();
  const Kinematics& GetKinematics() const;
  void UpdateKinematics(const Kinematics& kinematics);
  void SetCallbackKinematicsUpdated(const KinematicsCallback& callback);
  const Kinematics ComputeAttachedKinematics(
      const Kinematics& kinematics) const;

  const bool InAttachedState() const;
  void SetAttachedState(const bool to_attach);

  bool GetStartLanded() const;
  void SetStartLanded(bool start_landed);
  void SetHomeGeoPoint(const HomeGeoPoint& home_geo_point);

  const Wrench& GetDragFaceWrench() const;
  void SetDragFaceWrench(const Wrench& drag_wrench);
  const Matrix3x3& GetInertiaMatrix() const;
  void ComputeInertiaMatrix();
  const float GetMass() const;
  void SetMass(const float mass);

  void CreateTopics();
  void OnBeginUpdate() override;
  void OnEndUpdate() override;

 private:
  friend class PayloadActor::Loader;

  PayloadActor::Loader loader_;

  std::vector<Link> links_;
  std::vector<Joint> joints_;

  CollisionInfo collision_info_;
  Kinematics kinematics_;
  Environment environment_;
  bool in_attached_state_ = false;
  bool start_landed_;

  Pose payload_offset_;
  Wrench drag_wrench_;
  Matrix3x3 inertia_;
  float mass_;

  KinematicsCallback callback_kinematics_updated_;

  Topic payload_actor_pose_topic_;
  Topic collision_info_topic_;
  std::vector<std::reference_wrapper<Topic>> topics_;
};

// -----------------------------------------------------------------------------
// class PayloadActor

PayloadActor::PayloadActor() : Actor(nullptr) {}

PayloadActor::PayloadActor(const std::string& id, const Transform& origin,
                           const Logger& logger,
                           const TopicManager& topic_manager,
                           const std::string& parent_topic_path,
                           const ServiceManager& service_manager,
                           const StateManager& state_manager,
                           HomeGeoPoint home_geo_point)
    : Actor(std::shared_ptr<ActorImpl>(new PayloadActor::Impl(
          id, origin, logger, topic_manager, parent_topic_path, service_manager,
          state_manager))) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())
      ->SetHomeGeoPoint(home_geo_point);
}

void PayloadActor::Load(ConfigJson config_json) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->Load(config_json);
}

const std::vector<Link>& PayloadActor::GetLinks() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->GetLinks();
}

const std::vector<Joint>& PayloadActor::GetJoints() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->GetJoints();
}

const Kinematics& PayloadActor::GetKinematics() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->GetKinematics();
}

const Environment& PayloadActor::GetEnvironment() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->GetEnvironment();
}

void PayloadActor::UpdateEnvironment() {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->UpdateEnvironment();
}

void PayloadActor::SetPoseOffset(const Pose& offset) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->SetPoseOffset(offset);
}

const Pose& PayloadActor::GetPoseOffset() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->GetPoseOffset();
}

void PayloadActor::UpdateKinematics(const Kinematics& kinematics) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->UpdateKinematics(kinematics);
}

const Kinematics PayloadActor::ComputeAttachedKinematics(
    const Kinematics& kinematics) const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())
      ->ComputeAttachedKinematics(kinematics);
}

const bool PayloadActor::InAttachedState() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->InAttachedState();
}
void PayloadActor::SetAttachedState(const bool to_attach) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->SetAttachedState(to_attach);
}

bool PayloadActor::GetStartLanded() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->GetStartLanded();
}

void PayloadActor::SetStartLanded(bool start_landed) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->SetStartLanded(start_landed);
}

const Wrench& PayloadActor::GetDragFaceWrench() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->GetDragFaceWrench();
}
void PayloadActor::SetDragFaceWrench(const Wrench& drag_wrench) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())
      ->SetDragFaceWrench(drag_wrench);
}

const Matrix3x3& PayloadActor::GetInertiaMatrix() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->GetInertiaMatrix();
}

void PayloadActor::ComputeInertiaMatrix() {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->ComputeInertiaMatrix();
}

const float PayloadActor::GetMass() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->GetMass();
}

void PayloadActor::SetMass(const float mass) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->SetMass(mass);
}

void PayloadActor::BeginUpdate() {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->BeginUpdate();
}

void PayloadActor::EndUpdate() {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->EndUpdate();
}

const CollisionInfo& PayloadActor::GetCollisionInfo() const {
  return static_cast<PayloadActor::Impl*>(pimpl_.get())->GetCollisionInfo();
}

void PayloadActor::UpdateCollisionInfo(const CollisionInfo& collision_info) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())
      ->UpdateCollisionInfo(collision_info);
}

void PayloadActor::SetHasCollided(bool has_collided) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())->SetHasCollided(has_collided);
}

void PayloadActor::SetCallbackKinematicsUpdated(
    const KinematicsCallback& callback) {
  static_cast<PayloadActor::Impl*>(pimpl_.get())
      ->SetCallbackKinematicsUpdated(callback);
}

// -----------------------------------------------------------------------------
// class PayloadActor::Impl

PayloadActor::Impl::Impl(const std::string& id, const Transform& origin,
                         const Logger& logger,
                         const TopicManager& topic_manager,
                         const std::string& parent_topic_path,
                         const ServiceManager& service_manager,
                         const StateManager& state_manager)
    : ActorImpl(ActorType::kPayloadActor, id, origin,
                Constant::Component::payload_actor, logger, topic_manager,
                parent_topic_path, service_manager, state_manager),
      loader_(*this),
      callback_kinematics_updated_(nullptr),
      start_landed_(false) {
  SetTopicPath();
  CreateTopics();
}

void PayloadActor::Impl::CreateTopics() {
  payload_actor_pose_topic_ =
      Topic("actual_pose", topic_path_, TopicType::kPublished, 0,
            MessageType::kPosestamped);

  collision_info_topic_ =
      Topic("collision_info", topic_path_, TopicType::kPublished, 0,
            MessageType::kCollisionInfo);

  topics_.push_back(payload_actor_pose_topic_);
  topics_.push_back(collision_info_topic_);
}

void PayloadActor::Impl::Load(ConfigJson config_json) {
  // Load PayloadActor from JSON config
  json json = config_json;
  loader_.Load(json);

  // Update initial environment from initial kinematics position (home geo point
  // should have been set during payload construction in the scene).
  UpdateEnvironment();

  // Initialize kinematics from loaded origin
  LoadKinematics();
}

const std::vector<Link>& PayloadActor::Impl::GetLinks() { return links_; }

const std::vector<Joint>& PayloadActor::Impl::GetJoints() { return joints_; }

void PayloadActor::Impl::LoadKinematics() {
  kinematics_.pose.position = origin_.translation_;
  kinematics_.pose.orientation = origin_.rotation_;
}

const Kinematics& PayloadActor::Impl::GetKinematics() const {
  return kinematics_;
}

void PayloadActor::Impl::UpdateKinematics(const Kinematics& kinematics) {
  std::lock_guard<std::mutex> lock(update_lock_);
  kinematics_ = kinematics;
  TimeNano time_stamp = SimClock::Get()->NowSimNanos();

  // check if kinematics is drone passthrough
  if (in_attached_state_) kinematics_ = ComputeAttachedKinematics(kinematics);

  // Process callback for updated kinematics
  auto func = callback_kinematics_updated_;
  if (func != nullptr) {
    func(kinematics_, time_stamp);
  }

  // Publish pose topic
  PoseMessage pose_msg(time_stamp, kinematics_.pose.position,
                       kinematics_.pose.orientation);
  topic_manager_.PublishTopic(payload_actor_pose_topic_, pose_msg);
}

const Kinematics PayloadActor::Impl::ComputeAttachedKinematics(
    const Kinematics& kinematics) const {
  Vector3 new_pos(kinematics.pose.position.x() + payload_offset_.position.x(),
                  kinematics.pose.position.y() + payload_offset_.position.y(),
                  kinematics.pose.position.z() + payload_offset_.position.z());

  return Kinematics(Pose(new_pos, payload_offset_.orientation),
                    kinematics.twist, kinematics.accels);
}

const bool PayloadActor::Impl::InAttachedState() const {
  return in_attached_state_;
}

void PayloadActor::Impl::SetAttachedState(const bool to_attach) {
  in_attached_state_ = to_attach;
}

const Environment& PayloadActor::Impl::GetEnvironment() const {
  return environment_;
}

void PayloadActor::Impl::UpdateEnvironment() {
  std::lock_guard<std::mutex> lock(update_lock_);
  environment_.SetPosition(kinematics_.pose.position);
}

void PayloadActor::Impl::SetPoseOffset(const Pose& offset) {
  std::lock_guard<std::mutex> lock(update_lock_);
  payload_offset_ = offset;
}

const Pose& PayloadActor::Impl::GetPoseOffset() const {
  return payload_offset_;
}

bool PayloadActor::Impl::GetStartLanded() const { return start_landed_; }

void PayloadActor::Impl::SetStartLanded(bool start_landed) {
  start_landed_ = start_landed;
}

const Wrench& PayloadActor::Impl::GetDragFaceWrench() const {
  return drag_wrench_;
}

void PayloadActor::Impl::SetDragFaceWrench(const Wrench& drag_wrench) {
  std::lock_guard<std::mutex> lock(update_lock_);
  drag_wrench_ = drag_wrench;
}

const Matrix3x3& PayloadActor::Impl::GetInertiaMatrix() const {
  return inertia_;
}

void PayloadActor::Impl::ComputeInertiaMatrix() {
  // treat payload as a point mass or link attached to drone
  float dx = payload_offset_.position.x();
  float dy = payload_offset_.position.y();
  float dz = payload_offset_.position.z();

  float inertia_dx = mass_ * (dy * dy + dz * dz);
  float inertia_dy = mass_ * (dx * dx + dz * dz);
  float inertia_dz = mass_ * (dx * dx + dy * dy);

  inertia_(0, 0) = inertia_dx;
  inertia_(1, 1) = inertia_dy;
  inertia_(2, 2) = inertia_dz;
}

const float PayloadActor::Impl::GetMass() const { return mass_; }

void PayloadActor::Impl::SetMass(const float mass) {
  mass_ = mass;
  ComputeInertiaMatrix();
}

void PayloadActor::Impl::SetHomeGeoPoint(const HomeGeoPoint& home_geo_point) {
  std::lock_guard<std::mutex> lock(update_lock_);
  environment_.home_geo_point = home_geo_point;
}

void PayloadActor::Impl::OnBeginUpdate() {
  // Register all topics to be ready for publish/subscribe calls
  for (const auto& topic_ref : topics_) {
    topic_manager_.RegisterTopic(topic_ref.get());
  }
}

void PayloadActor::Impl::OnEndUpdate() {
  callback_kinematics_updated_ = nullptr;
  // Unregister all topics
  for (const auto& topic_ref : topics_) {
    topic_manager_.UnregisterTopic(topic_ref.get());
  }
}

const CollisionInfo& PayloadActor::Impl::GetCollisionInfo() const {
  return collision_info_;
}

void PayloadActor::Impl::UpdateCollisionInfo(
    const CollisionInfo& collision_info) {
  std::lock_guard<std::mutex> lock(update_lock_);
  collision_info_ = collision_info;

  // Publish a collision info topic every time a collision is detected
  if (collision_info_.has_collided) {
    CollisionInfoMessage collision_info_msg(collision_info_);
    topic_manager_.PublishTopic(collision_info_topic_, collision_info_msg);
  }
}

void PayloadActor::Impl::SetHasCollided(bool has_collided) {
  std::lock_guard<std::mutex> lock(update_lock_);
  collision_info_.has_collided = has_collided;
}

void PayloadActor::Impl::SetCallbackKinematicsUpdated(
    const KinematicsCallback& callback) {
  std::lock_guard<std::mutex> lock(update_lock_);
  callback_kinematics_updated_ = callback;
}

// class PayloadActor::Loader
PayloadActor::Loader::Loader(PayloadActor::Impl& impl) : impl_(impl) {}

void PayloadActor::Loader::Load(const json& json) {
  LoadLinks(json);
  LoadJoints(json);

  impl_.is_loaded_ = true;
}

void PayloadActor::Loader::LoadLinks(const json& json) {
  impl_.logger_.LogVerbose(impl_.name_, "[%s] Loading 'links'.",
                           impl_.id_.c_str());

  auto links_json = JsonUtils::GetArray(json, Constant::Config::links);
  if (JsonUtils::IsEmptyArray(links_json)) {
    impl_.logger_.LogWarning(impl_.name_, "[%s] 'links' missing or empty.",
                             impl_.id_.c_str());
  }

  try {
    std::transform(links_json.begin(), links_json.end(),
                   std::back_inserter(impl_.links_),
                   [this](auto& json) { return LoadLink(json); });
  } catch (...) {
    impl_.links_.clear();
    throw;
  }
  impl_.logger_.LogVerbose(impl_.name_, "[%s] 'links' loaded.",
                           impl_.id_.c_str());
}

Link PayloadActor::Loader::LoadLink(const json& json) {
  Link link(impl_.logger_, impl_.topic_manager_, impl_.topic_path_ + "/links");
  link.Load(json);

  return link;
}

void PayloadActor::Loader::LoadJoints(const json& json) {
  impl_.logger_.LogVerbose(impl_.name_, "[%s] Loading 'joints'.",
                           impl_.id_.c_str());

  auto joints_json = JsonUtils::GetArray(json, Constant::Config::joints);
  if (JsonUtils::IsEmptyArray(joints_json)) {
    impl_.logger_.LogWarning(impl_.name_, "[%s] 'joints' missing or empty.",
                             impl_.id_.c_str());
  }

  try {
    std::transform(joints_json.begin(), joints_json.end(),
                   std::back_inserter(impl_.joints_),
                   [this](auto& json) { return LoadJoint(json); });
  } catch (...) {
    impl_.joints_.clear();
    throw;
  }

  impl_.logger_.LogVerbose(impl_.name_, "[%s] 'joints' loaded.",
                           impl_.id_.c_str());
}

Joint PayloadActor::Loader::LoadJoint(const json& json) {
  Joint joint(impl_.logger_, impl_.topic_manager_,
              impl_.topic_path_ + "/joints");
  joint.Load(json);
  return joint;
}

}  // namespace projectairsim
}  // namespace microsoft
