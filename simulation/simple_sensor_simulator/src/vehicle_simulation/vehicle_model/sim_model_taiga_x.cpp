// Copyright 2025 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// High-fidelity rigid-body + tire vehicle model backed by a real-time physics
// engine. The PhysX SDK it links against is provided by the physx_vendor
// package (see CMakeLists.txt).

#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_taiga_x.hpp>

#include <PxPhysicsAPI.h>
#include <vehicle2/PxVehicleAPI.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "snippetvehiclecommon/directdrivetrain/DirectDrivetrain.h"

namespace autoware::simulator::simple_planning_simulator
{
namespace
{
using namespace physx;
using namespace physx::vehicle2;

constexpr PxU32 kWheelCount = 6;
constexpr PxU32 kFrontLeftWheel = 0;
constexpr PxU32 kFrontRightWheel = 1;
constexpr double kGravity = 9.80665;
constexpr double kChassisHeight = 0.51;    //!< chassis origin height above the wheel centers [m]
constexpr double kSuspensionRest = 0.24;    //!< suspension rest length / travel [m]

double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }

// ---------------------------------------------------------------------------
// Process-wide physics foundation + vehicle extension are singletons that the
// SDK forbids creating more than once; reference-count them so multiple model
// instances (e.g. several rollout configurations) can coexist and the
// foundation is released only when the last instance is destroyed.
// ---------------------------------------------------------------------------
PxDefaultAllocator g_allocator;
PxDefaultErrorCallback g_error_callback;
PxFoundation * g_foundation = nullptr;
int g_refcount = 0;

void acquire_foundation()
{
  if (g_refcount++ == 0) {
    g_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, g_allocator, g_error_callback);
    if (g_foundation == nullptr) {
      g_refcount = 0;
      throw std::runtime_error("taiga_x: PxCreateFoundation failed");
    }
    if (!PxInitVehicleExtension(*g_foundation)) {
      g_foundation->release();
      g_foundation = nullptr;
      g_refcount = 0;
      throw std::runtime_error("taiga_x: PxInitVehicleExtension failed");
    }
  }
}

void release_foundation()
{
  if (--g_refcount == 0 && g_foundation != nullptr) {
    PxCloseVehicleExtension();
    g_foundation->release();
    g_foundation = nullptr;
  }
}

void clear_response(PxVehicleCommandResponseParams & params)
{
  params.nonlinearResponse.clear();
  params.maxResponse = 0.0f;
  for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; ++i) {
    params.wheelResponseMultipliers[i] = 0.0f;
  }
}

double yaw_from_quat(const PxQuat & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

}  // namespace

// ===========================================================================
// PhysX-backed engine (PIMPL).
// ===========================================================================
struct SimModelTaigaX::Impl
{
  // geometry / mass parameters
  double wheelbase;
  double track_width;
  double mass;
  double inertia_z;
  double cg_offset_x;
  double max_steer;
  double wheel_radius;

  // engine objects
  PxPhysics * physics = nullptr;
  PxDefaultCpuDispatcher * dispatcher = nullptr;
  PxScene * scene = nullptr;
  PxMaterial * material = nullptr;
  PxRigidStatic * ground = nullptr;
  snippetvehicle::DirectDriveVehicle vehicle;
  PxVehiclePhysXMaterialFriction material_friction{};
  PxVehiclePhysXSimulationContext context;
  bool initialized = false;

  PxRigidDynamic * actor() const
  {
    return static_cast<PxRigidDynamic *>(vehicle.mPhysXState.physxActor.rigidBody);
  }

  void configure_base()
  {
    vehicle.mBaseParams.axleDescription.setToDefault();
    const PxU32 front_wheels[2] = {0, 1};
    const PxU32 rear_wheels[4] = {2, 3, 4, 5};
    vehicle.mBaseParams.axleDescription.addAxle(2, front_wheels);
    vehicle.mBaseParams.axleDescription.addAxle(4, rear_wheels);

    vehicle.mBaseParams.frame.setToDefault();
    vehicle.mBaseParams.frame.lngAxis = PxVehicleAxes::ePosX;
    vehicle.mBaseParams.frame.latAxis = PxVehicleAxes::ePosY;
    vehicle.mBaseParams.frame.vrtAxis = PxVehicleAxes::ePosZ;
    vehicle.mBaseParams.scale.setToDefault();
    vehicle.mBaseParams.scale.scale = 1.0f;

    auto & bp = vehicle.mBaseParams;
    const auto m = static_cast<PxReal>(mass);
    const auto wb = static_cast<PxReal>(wheelbase);
    const auto tw = static_cast<PxReal>(track_width);
    const auto wr = static_cast<PxReal>(wheel_radius);
    const auto ch = static_cast<PxReal>(kChassisHeight);
    const auto travel = static_cast<PxReal>(kSuspensionRest);

    bp.rigidBodyParams.mass = m;
    const PxReal body_len = wb + 1.7f;
    const PxReal body_w = tw + 0.35f;
    const PxReal body_h = std::max<PxReal>(0.8f, ch);
    bp.rigidBodyParams.moi = PxVec3(
      m * (body_w * body_w + body_h * body_h) / 12.0f,
      m * (body_len * body_len + body_h * body_h) / 12.0f,
      static_cast<PxReal>(inertia_z));

    clear_response(bp.brakeResponseParams[0]);
    clear_response(bp.brakeResponseParams[1]);
    const auto brake_torque =
      static_cast<PxReal>(std::max(1.0, mass * 9.0 * wheel_radius / kWheelCount));
    bp.brakeResponseParams[0].maxResponse = brake_torque;
    for (PxU32 i = 0; i < kWheelCount; ++i) {
      bp.brakeResponseParams[0].wheelResponseMultipliers[i] = 1.0f;
    }
    bp.brakeResponseParams[1].maxResponse = brake_torque;
    for (PxU32 i = 2; i < kWheelCount; ++i) {
      bp.brakeResponseParams[1].wheelResponseMultipliers[i] = 1.0f;
    }

    clear_response(bp.steerResponseParams);
    bp.steerResponseParams.maxResponse = static_cast<PxReal>(max_steer);
    bp.steerResponseParams.wheelResponseMultipliers[kFrontLeftWheel] = 1.0f;
    bp.steerResponseParams.wheelResponseMultipliers[kFrontRightWheel] = 1.0f;
    bp.ackermannParams[0].wheelIds[0] = 1;
    bp.ackermannParams[0].wheelIds[1] = 0;
    bp.ackermannParams[0].wheelBase = wb;
    bp.ackermannParams[0].trackWidth = tw;
    bp.ackermannParams[0].strength = 1.0f;

    bp.suspensionStateCalculationParams.suspensionJounceCalculationType =
      PxVehicleSuspensionJounceCalculationType::eRAYCAST;
    bp.suspensionStateCalculationParams.limitSuspensionExpansionVelocity = false;

    const PxVec3 com_offset(static_cast<PxReal>(cg_offset_x), 0.0f, 0.0f);
    const PxVec3 wheel_offsets_actor[kWheelCount] = {
      {wb * 0.5f, tw * 0.5f, -ch},
      {wb * 0.5f, -tw * 0.5f, -ch},
      {-wb * 0.5f, tw * 0.5f, -ch},
      {-wb * 0.5f, -tw * 0.5f, -ch},
      {-wb * 0.5f, tw * 0.5f, -ch},
      {-wb * 0.5f, -tw * 0.5f, -ch},
    };

    // Sprung mass split from the longitudinal CG position (front axle carries
    // the share proportional to the rear lever arm and vice versa).
    const double lf = wheelbase * 0.5 - cg_offset_x;
    const double lr = wheelbase * 0.5 + cg_offset_x;
    const double front_sprung = mass * (lr / wheelbase) * 0.5;       // per front wheel
    const double rear_sprung = mass * (lf / wheelbase) * 0.25;       // per rear wheel (4 wheels)
    const PxReal spring_rest = std::max<PxReal>(0.05f, travel * 0.55f);
    const auto wheel_mass = static_cast<PxReal>(std::max(8.0, 0.011 * mass));
    const auto wheel_moi = static_cast<PxReal>(std::max(0.01, 0.5 * wheel_mass * wr * wr));

    for (PxU32 i = 0; i < kWheelCount; ++i) {
      const PxReal sprung = static_cast<PxReal>(i < 2 ? front_sprung : rear_sprung);
      const PxReal k = sprung * static_cast<PxReal>(kGravity) / spring_rest;
      const PxReal c = 2.0f * std::sqrt(k * sprung) * 0.35f;

      bp.wheelParams[i].radius = wr;
      bp.wheelParams[i].halfWidth = 0.15f;
      bp.wheelParams[i].mass = wheel_mass;
      bp.wheelParams[i].moi = wheel_moi;
      bp.wheelParams[i].dampingRate = 1.0f;

      const PxVec3 off = wheel_offsets_actor[i] - com_offset;
      bp.suspensionParams[i].suspensionAttachment =
        PxTransform(off + PxVec3(0.0f, 0.0f, travel), PxQuat(PxIdentity));
      bp.suspensionParams[i].suspensionTravelDir = PxVec3(0.0f, 0.0f, -1.0f);
      bp.suspensionParams[i].suspensionTravelDist = travel;
      bp.suspensionParams[i].wheelAttachment = PxTransform(PxIdentity);

      bp.suspensionComplianceParams[i].wheelToeAngle.clear();
      bp.suspensionComplianceParams[i].wheelCamberAngle.clear();
      bp.suspensionComplianceParams[i].suspForceAppPoint.clear();
      bp.suspensionComplianceParams[i].tireForceAppPoint.clear();
      bp.suspensionComplianceParams[i].suspForceAppPoint.addPair(0.0f, PxVec3(0.0f));
      bp.suspensionComplianceParams[i].tireForceAppPoint.addPair(0.0f, PxVec3(0.0f));

      bp.suspensionForceParams[i].stiffness = k;
      bp.suspensionForceParams[i].damping = c;
      bp.suspensionForceParams[i].sprungMass = sprung;

      bp.tireForceParams[i].latStiffX = 2.0f;
      const PxReal lat_mult = i < 2 ? 6.5f : 13.0f;
      bp.tireForceParams[i].latStiffY = sprung * 18.0f * lat_mult;
      bp.tireForceParams[i].longStiff = sprung * 14.0f * 1.2f;
      bp.tireForceParams[i].camberStiff = 0.0f;
      bp.tireForceParams[i].frictionVsSlip[0][0] = 0.0f;
      bp.tireForceParams[i].frictionVsSlip[0][1] = 1.0f;
      bp.tireForceParams[i].frictionVsSlip[1][0] = 0.1f;
      bp.tireForceParams[i].frictionVsSlip[1][1] = 1.1f;
      bp.tireForceParams[i].frictionVsSlip[2][0] = 1.0f;
      bp.tireForceParams[i].frictionVsSlip[2][1] = 0.6f;
      bp.tireForceParams[i].restLoad = sprung * static_cast<PxReal>(kGravity);
      bp.tireForceParams[i].loadFilter[0][0] = 0.0f;
      bp.tireForceParams[i].loadFilter[0][1] = 0.0f;
      bp.tireForceParams[i].loadFilter[1][0] = 2.0f;
      bp.tireForceParams[i].loadFilter[1][1] = 2.0f;
    }
  }

  void configure_drivetrain()
  {
    auto & dp = vehicle.mDirectDriveParams;
    clear_response(dp.directDriveThrottleResponseParams);
    dp.directDriveThrottleResponseParams.maxResponse =
      static_cast<PxReal>(std::max(1.0, mass * 2.3 * wheel_radius / 4.0));
    for (PxU32 i = 2; i < kWheelCount; ++i) {
      dp.directDriveThrottleResponseParams.wheelResponseMultipliers[i] = 1.0f;
    }
  }

  void configure_physx()
  {
    const PxReal friction = 1.6f;
    material_friction.material = material;
    material_friction.friction = friction;

    auto & pp = vehicle.mPhysXParams;
    pp.physxRoadGeometryQueryParams.roadGeometryQueryType =
      PxVehiclePhysXRoadGeometryQueryType::eRAYCAST;
    pp.physxRoadGeometryQueryParams.defaultFilterData =
      PxQueryFilterData(PxFilterData(0, 0, 0, 0), PxQueryFlag::eSTATIC);
    pp.physxRoadGeometryQueryParams.filterCallback = nullptr;
    pp.physxRoadGeometryQueryParams.filterDataEntries = nullptr;
    for (PxU32 i = 0; i < kWheelCount; ++i) {
      pp.physxMaterialFrictionParams[i].defaultFriction = friction;
      pp.physxMaterialFrictionParams[i].materialFrictions = &material_friction;
      pp.physxMaterialFrictionParams[i].nbMaterialFrictions = 1;
      pp.physxSuspensionLimitConstraintParams[i].restitution = 0.0f;
      pp.physxSuspensionLimitConstraintParams[i].directionForSuspensionLimitConstraint =
        PxVehiclePhysXSuspensionLimitConstraintParams::eROAD_GEOMETRY_NORMAL;
      pp.physxWheelShapeLocalPoses[i] = PxTransform(PxIdentity);
    }
    pp.physxActorCMassLocalPose =
      PxTransform(PxVec3(static_cast<PxReal>(cg_offset_x), 0.0f, 0.0f), PxQuat(PxIdentity));
    pp.physxActorBoxShapeHalfExtents = PxVec3(2.25f, 0.90f, 0.35f);
    pp.physxActorBoxShapeLocalPose = PxTransform(PxIdentity);
  }

  void build()
  {
    acquire_foundation();

    PxTolerancesScale scale;
    physics = PxCreatePhysics(PX_PHYSICS_VERSION, *g_foundation, scale, false, nullptr);
    if (physics == nullptr) throw std::runtime_error("taiga_x: PxCreatePhysics failed");

    PxSceneDesc scene_desc(scale);
    scene_desc.gravity = PxVec3(0.0f, 0.0f, -static_cast<PxReal>(kGravity));
    dispatcher = PxDefaultCpuDispatcherCreate(0);
    scene_desc.cpuDispatcher = dispatcher;
    scene_desc.filterShader = PxDefaultSimulationFilterShader;
    scene = physics->createScene(scene_desc);
    if (scene == nullptr) throw std::runtime_error("taiga_x: createScene failed");

    // Low chassis contact friction: tire-road grip is modeled by the vehicle
    // tire model, not by box contact.
    material = physics->createMaterial(0.02f, 0.02f, 0.0f);
    ground = PxCreatePlane(*physics, PxPlane(0.0f, 0.0f, 1.0f, 0.0f), *material);
    scene->addActor(*ground);

    configure_base();
    configure_drivetrain();
    configure_physx();

    PxCookingParams cooking(scale);
    if (!vehicle.initialize(*physics, cooking, *material, true)) {
      throw std::runtime_error("taiga_x: vehicle initialize failed");
    }
    initialized = true;
    vehicle.mTransmissionCommandState.gear =
      PxVehicleDirectDriveTransmissionCommandState::eFORWARD;

    // Create the PhysX actor and add it to the scene. The chassis rests on the
    // ground at z = chassis_height + suspension_rest.
    const PxReal z0 = static_cast<PxReal>(kChassisHeight + kSuspensionRest);
    const PxTransform initial_pose(PxVec3(0.0f, 0.0f, z0), PxQuat(PxIdentity));
    vehicle.setUpActor(*scene, initial_pose, "taiga_x_vehicle");
    if (auto * a = actor()) {
      a->setSolverIterationCounts(12, 4);
      a->setLinearDamping(0.02f);
      a->setAngularDamping(0.20f);
      a->setMaxAngularVelocity(8.0f);
    }

    context.setToDefault();
    context.frame = vehicle.mBaseParams.frame;
    context.scale = vehicle.mBaseParams.scale;
    context.gravity = scene_desc.gravity;
    context.physxScene = scene;
    context.physxActorUpdateMode = PxVehiclePhysXActorUpdateMode::eAPPLY_ACCELERATION;

    teleport(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  void teleport(double x, double y, double yaw, double vx, double vy, double wz)
  {
    auto * a = actor();
    if (a == nullptr) return;
    const double z = kChassisHeight + kSuspensionRest;
    const PxTransform pose(
      PxVec3(static_cast<PxReal>(x), static_cast<PxReal>(y), static_cast<PxReal>(z)),
      PxQuat(static_cast<PxReal>(yaw), PxVec3(0.0f, 0.0f, 1.0f)));
    a->setGlobalPose(pose);

    const PxVec3 fwd(static_cast<PxReal>(std::cos(yaw)), static_cast<PxReal>(std::sin(yaw)), 0.0f);
    const PxVec3 lat(static_cast<PxReal>(-std::sin(yaw)), static_cast<PxReal>(std::cos(yaw)), 0.0f);
    a->setLinearVelocity(fwd * static_cast<PxReal>(vx) + lat * static_cast<PxReal>(vy));
    a->setAngularVelocity(PxVec3(0.0f, 0.0f, static_cast<PxReal>(wz)));

    const auto spin = static_cast<PxReal>(vx / std::max(1e-3, wheel_radius));
    for (PxU32 i = 0; i < kWheelCount; ++i) {
      vehicle.mBaseState.wheelRigidBody1dStates[i].rotationSpeed = spin;
    }
    a->wakeUp();
  }

  void step(double throttle, double brake, double steer_norm, int gear, double dt)
  {
    vehicle.mCommandState.setToDefault();
    vehicle.mCommandState.nbBrakes = 1;
    vehicle.mCommandState.brakes[0] = static_cast<PxReal>(clamp01(brake));
    vehicle.mCommandState.throttle = static_cast<PxReal>(clamp01(throttle));
    vehicle.mCommandState.steer = static_cast<PxReal>(std::clamp(steer_norm, -1.0, 1.0));
    vehicle.mTransmissionCommandState.gear =
      gear < 0 ? PxVehicleDirectDriveTransmissionCommandState::eREVERSE
               : (gear > 0 ? PxVehicleDirectDriveTransmissionCommandState::eFORWARD
                           : PxVehicleDirectDriveTransmissionCommandState::eNEUTRAL);

    vehicle.step(static_cast<PxReal>(dt), context);
    scene->simulate(static_cast<PxReal>(dt));
    scene->fetchResults(true);
  }

  void release()
  {
    if (initialized) {
      vehicle.destroy();
      initialized = false;
    }
    if (ground != nullptr) { ground->release(); ground = nullptr; }
    if (scene != nullptr) { scene->release(); scene = nullptr; }
    if (dispatcher != nullptr) { dispatcher->release(); dispatcher = nullptr; }
    if (material != nullptr) { material->release(); material = nullptr; }
    if (physics != nullptr) { physics->release(); physics = nullptr; }
    release_foundation();
  }

  ~Impl() { release(); }
};

// ===========================================================================
// SimModelInterface implementation
// ===========================================================================
SimModelTaigaX::SimModelTaigaX(
  double wheelbase, double track_width, double mass, double inertia_z, double cg_offset_x,
  double max_steer, double max_accel, double max_brake, double wheel_radius, double fixed_dt)
: SimModelInterface(6 /* dim x */, 4 /* dim u */),
  impl_(std::make_unique<Impl>()),
  max_steer_(std::max(1e-3, max_steer)),
  max_accel_(std::max(1e-3, max_accel)),
  max_brake_(std::max(1e-3, max_brake)),
  fixed_dt_(std::max(1e-4, fixed_dt))
{
  impl_->wheelbase = wheelbase;
  impl_->track_width = track_width;
  impl_->mass = mass;
  impl_->inertia_z = inertia_z;
  impl_->cg_offset_x = cg_offset_x;
  impl_->max_steer = max_steer_;
  impl_->wheel_radius = wheel_radius;
  impl_->build();
  syncStateFromEngine();
}

SimModelTaigaX::~SimModelTaigaX() = default;

void SimModelTaigaX::setFullState(
  double x, double y, double yaw, double vx, double vy, double wz, double ax, double steer)
{
  impl_->teleport(x, y, yaw, vx, vy, wz);
  ax_ = ax;
  steer_ = steer;
  syncStateFromEngine();
}

void SimModelTaigaX::syncStateFromEngine()
{
  auto * a = impl_->actor();
  if (a == nullptr) return;
  const auto pose = a->getGlobalPose();
  const double yaw = yaw_from_quat(pose.q);
  const auto v = a->getLinearVelocity();
  const auto w = a->getAngularVelocity();
  const double vx = v.x * std::cos(yaw) + v.y * std::sin(yaw);
  const double vy = -v.x * std::sin(yaw) + v.y * std::cos(yaw);
  state_(IDX::X) = pose.p.x;
  state_(IDX::Y) = pose.p.y;
  state_(IDX::YAW) = yaw;
  state_(IDX::VX) = vx;
  state_(IDX::VY) = vy;
  state_(IDX::WZ) = w.z;
}

double SimModelTaigaX::getX() { return state_(IDX::X); }
double SimModelTaigaX::getY() { return state_(IDX::Y); }
double SimModelTaigaX::getYaw() { return state_(IDX::YAW); }
double SimModelTaigaX::getVx() { return state_(IDX::VX); }
double SimModelTaigaX::getVy() { return state_(IDX::VY); }
double SimModelTaigaX::getAx() { return ax_; }
double SimModelTaigaX::getWz() { return state_(IDX::WZ); }
double SimModelTaigaX::getSteer() { return steer_; }

void SimModelTaigaX::update(const double & dt)
{
  const double accel_des = input_(IDX_U::ACCX_DES);
  const double steer_des = input_(IDX_U::STEER_DES);
  const double gear_val = input_(IDX_U::GEAR);

  double throttle = 0.0;
  double brake = 0.0;
  if (accel_des >= 0.0) {
    throttle = clamp01(accel_des / max_accel_);
  } else {
    brake = clamp01(-accel_des / max_brake_);
  }
  const double steer_norm = std::clamp(steer_des / max_steer_, -1.0, 1.0);

  using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
  int gear = 1;  // forward
  if (gear_val == GearCommand::REVERSE || gear_val == GearCommand::REVERSE_2) {
    gear = -1;
  } else if (gear_val == GearCommand::NEUTRAL || gear_val == GearCommand::PARK ||
             gear_val == GearCommand::NONE) {
    gear = 0;
  }

  const double prev_vx = state_(IDX::VX);

  // Advance the engine over the outer dt as ceil(dt / fixed_dt) equal substeps,
  // keeping the total integrated time exactly dt.
  const int n_sub = std::max(1, static_cast<int>(std::ceil(dt / fixed_dt_)));
  const double sub = dt / static_cast<double>(n_sub);
  for (int i = 0; i < n_sub; ++i) {
    impl_->step(throttle, brake, steer_norm, gear, sub);
  }
  syncStateFromEngine();

  ax_ = (state_(IDX::VX) - prev_vx) / dt;
  // measured steering angle = mean of the two front wheel steer responses
  steer_ = 0.5 * (impl_->vehicle.mBaseState.steerCommandResponseStates[kFrontLeftWheel] +
                  impl_->vehicle.mBaseState.steerCommandResponseStates[kFrontRightWheel]);
}

Eigen::VectorXd SimModelTaigaX::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & /*input*/)
{
  // The physics engine performs its own integration; no closed-form derivative.
  return Eigen::VectorXd::Zero(state.size());
}

}  // namespace autoware::simulator::simple_planning_simulator
