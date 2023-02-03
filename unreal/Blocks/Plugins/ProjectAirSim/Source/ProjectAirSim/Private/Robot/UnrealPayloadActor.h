// Copyright (C) Microsoft Corporation.  All rights reserved.

#pragma once

#include <map>
#include <set>
#include <vector>

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Runtime/Core/Public/Templates/SharedPointer.h"
#include "UnrealRobotJoint.h"
#include "UnrealRobotLink.h"
#include "core_sim/actor/payload_actor.hpp"
#include "core_sim/clock.hpp"
#include "core_sim/physics_common_types.hpp"
#include "unreal_physics.hpp"

// comment so that generated.h is always the last include file with clang-format
#include "UnrealPayloadActor.generated.h"

UCLASS()
class AUnrealPayloadActor : public AActor {
  GENERATED_BODY()

 public:
  explicit AUnrealPayloadActor(const FObjectInitializer& ObjectInitialize);

  void Initialize(const microsoft::projectairsim::PayloadActor& InPayloadActor,
                  microsoft::projectairsim::UnrealPhysicsBody* InPhysBody);

  void Tick(float DeltaTime) override;

  const microsoft::projectairsim::Kinematics& GetKinematics() const;

  UFUNCTION()
  void OnCollisionHit(UPrimitiveComponent* HitComponent, AActor* OtherActor,
                      UPrimitiveComponent* OtherComp, FVector NormalImpulse,
                      const FHitResult& Hit);

 protected:
  void BeginPlay() override;

  void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

 private:
  void InitializeId(const std::string& InId);

  void InitializeLinks(
      const std::vector<microsoft::projectairsim::Link>& InLinks,
      const std::set<std::string>& InRootLinks);

  std::pair<std::string, UUnrealRobotLink*> CreateLink(
      const microsoft::projectairsim::Link& InLink, bool bIsRootLink);

  void InitializeJoints(
      const std::vector<microsoft::projectairsim::Joint>& InJoints);

  std::pair<std::string, UUnrealRobotJoint*> CreateJoint(
      const microsoft::projectairsim::Joint& InJoint);

  void SetPayloadActorKinematics(
      const microsoft::projectairsim::Kinematics& InKin, TimeNano InTimestamp);

  void MovePayloadActorToUnrealPose(bool bUseCollisionSweep);

  std::set<std::string> GetRootLinks(
      const std::vector<microsoft::projectairsim::Link>& InLinks,
      const std::vector<microsoft::projectairsim::Joint>& InJoints);

  UUnrealRobotLink* PayloadActorRootLink;

  FCriticalSection UpdateMutex;

  microsoft::projectairsim::PayloadActor SimPayloadActor;
  microsoft::projectairsim::UnrealPhysicsBody* SimPhysicsBody;
  std::map<std::string, UUnrealRobotLink*> PayloadActorLinks;
  std::map<std::string, UUnrealRobotJoint*> PayloadActorJoints;

  bool bHasLinkRotAnglesUpdated = false;

  bool bHasKinematicsUpdated = false;
  microsoft::projectairsim::Kinematics PayloadActorKinematics;
  TimeNano KinematicsUpdatedTimeStamp = 0;

  bool bHasUnrealPoseUpdated = false;
};