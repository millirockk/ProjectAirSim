// Copyright (C) Microsoft Corporation.  All rights reserved.
// The Unreal PayloadActor implementation.

#include "UnrealPayloadActor.h"

#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>

#include "Components/StaticMeshComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "ProjectAirSim.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "UnrealHelpers.h"
#include "UnrealLogger.h"
#include "core_sim/clock.hpp"
#include "core_sim/physics_common_types.hpp"
#include "core_sim/transforms/transform.hpp"

namespace projectairsim = microsoft::projectairsim;

AUnrealPayloadActor::AUnrealPayloadActor(
    const FObjectInitializer& ObjectInitialize)
    : AActor(ObjectInitialize) {
  PrimaryActorTick.bCanEverTick = true;
}

void AUnrealPayloadActor::Initialize(
    const projectairsim::PayloadActor& InPayloadActor) {
  // Store ptrs to other corresponding components for this payload actor
  this->SimPayloadActor =
      InPayloadActor;  // This makes a copy from the payload actor ref. It does
                       // get another shared_ptr for the same address
                       // of the robot's impl/ActorImpl, but any data
                       // at the robot level would be decoupled so only
                       // store robot data at the impl level.

  // Detect which links are roots based on their joint attachments
  auto RootLinks =
      GetRootLinks(InPayloadActor.GetLinks(), InPayloadActor.GetJoints());
  bool bWithUnrealPhysics = (InPayloadActor.GetPhysicsType() ==
                             projectairsim::PhysicsType::kUnrealPhysics);

  if (bWithUnrealPhysics) {
    // For Unreal-calculated physics, do the updates after Unreal has completed
    // the physics tick calculations.
    PrimaryActorTick.TickGroup = TG_PostPhysics;
  } else {
    // For calculated physics, do the updates during Unreal's world
    // physics tick to stay in sequence with the other components.
    PrimaryActorTick.TickGroup = TG_DuringPhysics;
  }                

  // PrimaryActorTick.TickGroup = TG_PrePhysics;

  // Initialize the configured payload actor component structure
  InitializeId(InPayloadActor.GetID());
  InitializeLinks(InPayloadActor.GetLinks(), RootLinks, bWithUnrealPhysics);
  InitializeJoints(InPayloadActor.GetJoints());

  // Set the initial pose from the payload actor's kinematics
  UpdatePayloadActorTargetPose(InPayloadActor.GetKinematics().pose,
                               0);      // timestamp=0
  MovePayloadActorToTargetPose(false);  // move without collision sweep
}

void AUnrealPayloadActor::InitializeId(const std::string& InId) {
  UnrealHelpers::SetActorName(this, InId);
}

void AUnrealPayloadActor::InitializeLinks(
    const std::vector<projectairsim::Link>& InLinks,
    const std::set<std::string>& InRootLinks, bool bWithUnrealPhysics) {
  std::for_each(InLinks.begin(), InLinks.end(),
                [this, &InRootLinks,
                 bWithUnrealPhysics](const projectairsim::Link& CurLink) {
                  bool bIsRootLink = false;
                  if (InRootLinks.find(CurLink.GetID()) != InRootLinks.end()) {
                    bIsRootLink = true;
                  }
                  PayloadActorLinks.insert(
                      CreateLink(CurLink, bIsRootLink, bWithUnrealPhysics));
                });
}

std::pair<std::string, UUnrealRobotLink*> AUnrealPayloadActor::CreateLink(
    const projectairsim::Link& InLink, bool bIsRootLink,
    bool bWithUnrealPhysics) {
  auto Id = InLink.GetID();

  auto NewLink = NewObject<UUnrealRobotLink>(this, Id.c_str());
  NewLink->Initialize(InLink, bWithUnrealPhysics);

  if (bIsRootLink) {
    PayloadActorRootLink = NewLink;
    RootComponent = NewLink;  // Also set as the USceneComponent's RootComponent
  }

  return {Id, NewLink};
}

void AUnrealPayloadActor::InitializeJoints(
    const std::vector<projectairsim::Joint>& InJoints) {
  std::for_each(InJoints.begin(), InJoints.end(),
                [this](const projectairsim::Joint& CurJoint) {
                  PayloadActorJoints.insert(CreateJoint(CurJoint));
                });
}

std::pair<std::string, UUnrealRobotJoint*> AUnrealPayloadActor::CreateJoint(
    const projectairsim::Joint& InJoint) {
  auto Id = InJoint.GetID();

  auto NewJoint = NewObject<UUnrealRobotJoint>(this, Id.c_str());
  NewJoint->Initialize(InJoint);

  auto Parent = PayloadActorLinks.at(InJoint.GetParentLink().c_str());
  auto Child = PayloadActorLinks.at(InJoint.GetChildLink().c_str());

  NewJoint->ConstraintActor1 = this;
  NewJoint->ConstraintActor2 = this;

  NewJoint->AttachToComponent(Parent,
                              FAttachmentTransformRules::KeepRelativeTransform);
  Child->AttachToComponent(NewJoint,
                           FAttachmentTransformRules::KeepRelativeTransform);
  NewJoint->SetConstrainedComponents(Parent, NAME_None, Child, NAME_None);

  return {Id, NewJoint};
}

std::set<std::string> AUnrealPayloadActor::GetRootLinks(
    const std::vector<projectairsim::Link>& InLinks,
    const std::vector<projectairsim::Joint>& InJoints) {
  std::set<std::string> Roots;

  std::transform(
      InLinks.begin(), InLinks.end(), std::inserter(Roots, Roots.begin()),
      [Roots](const projectairsim::Link& CurLink) { return CurLink.GetID(); });

  std::for_each(InJoints.begin(), InJoints.end(),
                [&Roots](const projectairsim::Joint& CurJoint) {
                  Roots.erase(CurJoint.GetChildLink());
                });

  return Roots;
}

void AUnrealPayloadActor::OnCollisionHit(UPrimitiveComponent* HitComponent,
                                         AActor* OtherActor,
                                         UPrimitiveComponent* OtherComp,
                                         FVector NormalImpulse,
                                         const FHitResult& Hit) {
  if ((OtherActor != nullptr) && (OtherActor != this) &&
      (OtherComp != nullptr)) {
    projectairsim::CollisionInfo NewCollisionInfo;
    NewCollisionInfo.has_collided = true;

    // NEU -> NED_m
    NewCollisionInfo.normal =
        projectairsim::TransformUtils::NeuToNedLinear(projectairsim::Vector3(
            Hit.ImpactNormal.X, Hit.ImpactNormal.Y, Hit.ImpactNormal.Z));

    // NEU_cm -> NEU_m -> NED_m
    NewCollisionInfo.impact_point =
        projectairsim::TransformUtils::NeuToNedLinear(
            projectairsim::TransformUtils::ToMeters(projectairsim::Vector3(
                Hit.ImpactPoint.X, Hit.ImpactPoint.Y, Hit.ImpactPoint.Z)));

    // NEU_cm -> NEU_m -> NED_m
    NewCollisionInfo.position = projectairsim::TransformUtils::NeuToNedLinear(
        projectairsim::TransformUtils::ToMeters(projectairsim::Vector3(
            Hit.Location.X, Hit.Location.Y, Hit.Location.Z)));

    // cm -> m
    NewCollisionInfo.penetration_depth =
        projectairsim::TransformUtils::ToMeters(Hit.PenetrationDepth);

    NewCollisionInfo.time_stamp = projectairsim::SimClock::Get()->NowSimNanos();
    NewCollisionInfo.object_name =
        std::string(TCHAR_TO_UTF8(*(OtherActor->GetName())));

    UPrimitiveComponent* OtherRootComp =
        Cast<class UPrimitiveComponent>(OtherActor->GetRootComponent());
    NewCollisionInfo.segmentation_id =
        OtherRootComp ? OtherRootComp->CustomDepthStencilValue : -1;

    SimPayloadActor.UpdateCollisionInfo(NewCollisionInfo);

    UnrealLogger::Log(
        projectairsim::LogLevel::kTrace,
        TEXT("Collision detected between '%s' and '%s' at z= '%f'"),
        *(HitComponent->GetName()), *(OtherActor->GetName()), Hit.Location.Z);
  }
}

void AUnrealPayloadActor::UpdatePayloadActorTargetPose(
    const projectairsim::Pose& InPose, TimeNano InTimestamp) {
  // TODO Need mutex lock to block writing while these data members are being
  // read in other parts of the code to guarantee pose/timestamp stays in sync?
  PayloadActorTargetPose = InPose;
  TargetPoseUpdatedTimeStamp = InTimestamp;
  bHasTargetPoseUpdated = true;

  // UnrealLogger::Log(projectairsim::LogLevel::kTrace,
  //                   TEXT("UpdatePayloadActorTargetPose xyz=%f, %f, %f"),
  //                   PayloadActorTargetPose.position.x(),
  //                   PayloadActorTargetPose.position.y(),
  //                   PayloadActorTargetPose.position.z());
}

void AUnrealPayloadActor::MovePayloadActorToTargetPose(
    bool bUseCollisionSweep) {
  if (PayloadActorRootLink == nullptr || bHasTargetPoseUpdated == false) return;

  // Clear payload's has_collided flag before trying to move again
  SimPayloadActor.SetHasCollided(false);

  // Copy target pose data in case it gets updated again while processing
  projectairsim::Pose TgtPose = PayloadActorTargetPose;
  bHasTargetPoseUpdated = false;  // done processing target pose, clear flag

  // Use local copy of target pose to do actual payload actor pose update
  const FVector TgtLocNEU =
      UnrealHelpers::ToFVector(projectairsim::TransformUtils::NedToNeuLinear(
          projectairsim::TransformUtils::ToCentimeters(TgtPose.position)));
  const FRotator TgtRot = UnrealHelpers::ToFRotator(TgtPose.orientation);

  // Move UE position
  PayloadActorRootLink->SetWorldLocationAndRotation(
      TgtLocNEU, TgtRot, bUseCollisionSweep, nullptr,
      ETeleportType::TeleportPhysics);
  // If 'bUseCollisionSweep' is true, collision hits during
  // SetWorldLocationAndRotation will be handled by the
  // callback AUnrealPayloadActor::OnCollisionHit() with the FHitResult info

  bHasPayloadActorPoseUpdated = true;

  // UnrealLogger::Log(projectairsim::LogLevel::kTrace,
  //                   TEXT("SetWorldLocationAndRotation xyz=%f, %f, %f"),
  //                   TgtLocNEU.X, TgtLocNEU.Y, TgtLocNEU.Z);
}

void AUnrealPayloadActor::BeginPlay() {
  Super::BeginPlay();

  // Register callback for detected collisions using root mesh's OnComponentHit
  if (PayloadActorRootLink != nullptr) {
    PayloadActorRootLink->OnComponentHit.AddDynamic(
        this, &AUnrealPayloadActor::OnCollisionHit);
  }
}

void AUnrealPayloadActor::EndPlay(const EEndPlayReason::Type EndPlayReason) {
  Super::EndPlay(EndPlayReason);
}

void AUnrealPayloadActor::Tick(float DeltaTime) {
  Super::Tick(DeltaTime);
  // In case of a tick coming through while it should be paused, just return
  if (UGameplayStatics::IsGamePaused(this->GetWorld())) return;

  const bool bUseCollisionSweep =
      PayloadActorRootLink ? PayloadActorRootLink->IsLinkCollisionEnabled()
                           : false;

  // At every tick, get the updated pose and move payload_actor to that pose
  UpdatePayloadActorTargetPose(SimPayloadActor.GetKinematics().pose,
                               0);  // timestamp=0
  MovePayloadActorToTargetPose(bUseCollisionSweep);
}

const microsoft::projectairsim::Kinematics& AUnrealPayloadActor::GetKinematics()
    const {
  return SimPayloadActor.GetKinematics();
}
