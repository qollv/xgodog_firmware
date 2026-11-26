#include "motion/gait.h"
#include "motion/walk_planner.h"
#include "motion/gait_engine.h"
#include "motion/time.h"
#include "motion/kinematics.h"
#include "sensor/attitude.h"
#include <Arduino.h>
#include "math.h"
#include <cmath>

uint8_t gait_reinit = 0;
namespace {
xgo::WalkPlanner high_walk{xgo::HighWalkConfig()};
xgo::GaitEngine ordinary_gait;
float periodBeforeOverride = 0;
bool lead_02 = true;

bool takeLead02() {
    const bool now = lead_02;
    lead_02 = !lead_02;
    return now;
}

void applyFeet(const xgo::WalkFeet &feet) {
    for (int i=0; i<4; ++i) {
        leg[i].x_step = feet[i].x;
        leg[i].y_step = feet[i].y;
        leg[i].z_step = feet[i].z;
    }
}

xgo::WalkInput fillStride(float period_scale, float yaw_scale, float lift_mm, bool moving, float legYaw) {
    xgo::WalkInput input;
    input.moving = moving;
    input.cycle_s = gm.T / SYSTEM_TICK_HZ * period_scale;
    input.lift_mm = lift_mm;
    const float vx = MoveMode == MOVE_MODE_WHEEL || MoveMode == MOVE_MODE_WHEEL2 ? 0 : body.vx;
    const float yaw = legYaw * yaw_scale;
    for (int i=0; i<4; ++i) {
        input.stride[i] = {vx + YAW_STRIDE_SCALE*sign.nppn[i]*yaw,
            body.vy + sign.ppnn[i]*yaw, 0};
        input.nominal[i] = {sign.ppnn[i]*bodyConfig.length-body.x,
            -sign.nppn[i]*(bodyConfig.width+bodyConfig.L_d)-body.y, 0};
    }
    return input;
}

float lowTrotPeriod() {
    return xgo_version == XGOLITE ? LOW_TROT_PERIOD_LITE : LOW_TROT_PERIOD;
}

float lowTrotStrideX() {
    return xgo_version == XGOLITE ? LOW_TROT_STRIDE_X_LITE : LOW_TROT_STRIDE_X;
}
}

void GaitResetWalk() {
    high_walk.reset();
    ordinary_gait.reset();
    applyFeet({});
}

static void prepareLowTrot(float *vx) {
    if (gm.gait_type == GAIT_WALK) {
        gm.T = SEC_2_TICK(WALK_PERIOD);
        return;
    }
    if (gm.gait_type != GAIT_LOW_TROT) return;
    gm.T = SEC_2_TICK(lowTrotPeriod());
    const float xmax = lowTrotStrideX();
    if (vx) {
        if (*vx > xmax) *vx = xmax;
        if (*vx < -xmax) *vx = -xmax;
    }
}

static bool updateWalk(bool moving, float legYaw) {
    const bool want_high = gm.gait_type == GAIT_HIGH_WALK;
    if (!want_high && !high_walk.active())
        return false;

    xgo::WalkFeet current;
    bool displaced = false;
    for (int i=0; i<4; ++i) {
        current[i] = {leg[i].x_step, leg[i].y_step, leg[i].z_step};
        displaced |= std::fabs(current[i].x)+std::fabs(current[i].y)+std::fabs(current[i].z) > 0.001f;
    }

    const float dt = motion_time::stepSeconds();
    xgo::WalkInput stop;
    stop.moving = false;
    if (!want_high && high_walk.active()) { applyFeet(high_walk.update(stop, dt)); return true; }

    if (want_high && !high_walk.active() && (moving || (gait_reinit && displaced))) {
        high_walk.setLeadPair(takeLead02());
        high_walk.begin(current);
    }
    if (want_high) gait_reinit = 0;
    const float lift = gm.z_step > 50.0f ? gm.z_step : 50.0f;
    applyFeet(high_walk.update(
        fillStride(high_walk.config().period_scale, high_walk.config().yaw_scale, lift, moving, legYaw), dt));
    return true;
}

bool GaitSetType(uint8_t type)
{
    if (type != GAIT_TROT && type != GAIT_WALK && type != GAIT_HIGH_WALK
        && type != GAIT_LOW_TROT && type != GAIT_MARKTIME)
        return false;
    if (gm.gait_type != type) {
        const bool override_period = (type == GAIT_LOW_TROT || type == GAIT_WALK);
        const bool was_override = (gm.gait_type == GAIT_LOW_TROT || gm.gait_type == GAIT_WALK);
        if (override_period && !was_override)
            periodBeforeOverride = gm.T;
        else if (!override_period && was_override && periodBeforeOverride > 0)
            gm.T = periodBeforeOverride;
        gm.gait_type = type;
        gait_reinit = 1;
        if (type == GAIT_LOW_TROT)
            gm.T = SEC_2_TICK(lowTrotPeriod());
        else if (type == GAIT_WALK)
            gm.T = SEC_2_TICK(WALK_PERIOD);
    }
    return true;
}

namespace {
float limitSupportOffset(float offset, float lift) {
    constexpr float kDegreesToRadians = 0.0174532925f;
    const kinematics::Geometry geometry(bodyConfig);
    kinematics::Point bases[4];
    if (!kinematics::solveBody(geometry,
        {body.x, body.y, body.z, body.rol, body.pit, body.yaw}, sign, bases)) return 0;
    // Reserve travel for swing overtravel and stop braking when shifting feet.
    const float stanceSeconds = fmaxf(0.2f, gm.T / SYSTEM_TICK_HZ)
        * (1.0f - fminf(0.5f, fmaxf(0.05f, sr)));
    const float excursion = fmaxf(1.2f, 1.0f + xgo::GaitEngine::BrakeUs*0.000001f/stanceSeconds);
    for (unsigned i=0; i<4; ++i) {
        const float knee = (motor[3*i].max_angle + 90.0f) * kDegreesToRadians;
        const float reachSquared = geometry.upperSquared + geometry.lowerSquared
            - geometry.kneeDenominator * cosf(knee);
        const float y = std::fabs(bases[i].y) + excursion*std::fabs(gm.y_step[i]);
        const float z = bases[i].z + leg[i].z_bias;
        const float minY = fmaxf(0, std::fabs(bases[i].y)-excursion*std::fabs(gm.y_step[i]));
        const float minZ = z - fminf(60, fmaxf(0, lift));
        const float minLegSquared = minZ*minZ + minY*minY - geometry.lateralSquared;
        const float minKnee = (motor[3*i].min_angle + 90.0f) * kDegreesToRadians;
        const float foldedReachSquared = geometry.upperSquared + geometry.lowerSquared
            - geometry.kneeDenominator * cosf(minKnee);
        const float foldedHeight = geometry.lower + geometry.upper
            * cosf(motor[3*i+1].max_angle * kDegreesToRadians);
        // At very low poses, leave the original gait alone: added placement
        // must not over-fold the knee or hip near maximum lift.
        if (minZ <= 0 || minLegSquared <= fmaxf(foldedReachSquared, foldedHeight*foldedHeight)) return 0;
        const float xSquared = reachSquared - y*y - z*z + geometry.lateralSquared;
        if (!std::isfinite(xSquared) || xSquared <= 0) return 0;
        const float reach = sqrtf(xSquared), stride = excursion*std::fabs(gm.x_step[i]);
        if (std::fabs(bases[i].x) + stride > reach) return 0;
        if (offset > 0) offset = fminf(offset, fmaxf(0, reach-bases[i].x-stride));
        else offset = fmaxf(offset, fminf(0, -reach-bases[i].x+stride));
    }
    return offset;
}

float reverseSupportOffset(bool moving, float lift) {
    constexpr float kDegreesToRadians = 0.0174532925f;
    // Reverse trot can pitch nose-down at each support exchange. Place the
    // support slightly ahead of the body in response to the measured error,
    // rather than imposing a permanent rearward body.x or changing its API.
    // Positive pitch is nose-down in both the IMU and the body IK convention.
    if (!moving || xgo_version != XGOMINI || hasWheel != 1 ||
        gm.gait_type != GAIT_TROT || body.vx >= -5.0f ||
        (MoveMode != MOVE_MODE_LEG && MoveMode != MOVE_MODE_HYBRID) ||
        Action_Ctx.id != 0 || Action_Ctx.loop || inCalibration ||
        imu.stable || isIMUInit != 2 || !imu_health::fresh((uint32_t)millis()) ||
        !std::isfinite(body.vx) || !std::isfinite(imu.pit) || !std::isfinite(body.pit) ||
        !std::isfinite(body.z) || std::fabs(imu.pit) > 45.0f)
        return 0;

    // A small proportional placement correction has no integral wind-up.
    // Ignore 1 degree of noise; use only 35% of the geometric projection.
    // The engine limits placement to +/-6 mm and blends it using its existing
    // trajectories. Stopping drains the current step and centres all feet.
    const float error = imu.pit - body.pit;
    const float magnitude = std::fabs(error) - 1.0f;
    if (magnitude <= 0 || std::fabs(error) > 30.0f || body.z <= 0) return 0;
    const float offset = 0.35f * body.z * sinf(copysignf(magnitude, error) * kDegreesToRadians);
    return limitSupportOffset(offset, lift);
}

xgo::GaitInput ordinaryInput(bool moving) {
    xgo::GaitInput input;
    input.moving = moving;
    input.kind = gm.gait_type;
    input.walk = gm.gait_type == GAIT_WALK;
    input.period_s = gm.T / SYSTEM_TICK_HZ;
    input.swing_ratio = input.walk ? WALK_SR : gm.gait_type == GAIT_LOW_TROT ? LOW_TROT_SWING : sr;
    for (int i=0; i<4; ++i) {
        input.stride[i] = {gm.x_step[i], gm.y_step[i], 0};
        if (gm.gait_type == GAIT_MARKTIME) input.stride[i] = {};
        input.lift[i] = gm.gait_type == GAIT_LOW_TROT
            ? (i < 2 ? LOW_TROT_LIFT_FRONT : LOW_TROT_LIFT_REAR)
            : input.walk ? WALK_LIFT
            : gm.gait_type == GAIT_TROT && std::fabs(gm.T-bodyConfig.SLOW_T)<0.5f
                ? TROT_SLOW_LIFT : gm.z_step;
    }
    input.support_x_mm = reverseSupportOffset(moving, input.lift[0]);
    return input;
}
xgo::WalkFeet currentFeet() {
    xgo::WalkFeet feet;
    for (int i=0; i<4; ++i) feet[i] = {leg[i].x_step, leg[i].y_step, leg[i].z_step};
    return feet;
}
}

bool GaitIsActive() { return ordinary_gait.active() || high_walk.active(); }

void GaitUpdateMotion(bool moving, float legYaw) {
    const bool wheelMode = MoveMode == MOVE_MODE_WHEEL2;
    const bool want_high = gm.gait_type == GAIT_HIGH_WALK;
    const bool mark = gm.gait_type == GAIT_MARKTIME;
    // Wheel mode uses the legs for lateral motion and large heading errors.
    // Forward wheel travel and small differential corrections do not step.
    moving = wheelMode ? moving && (std::fabs(body.vy) > 1.0f || std::fabs(legYaw) > 1.0f)
                       : moving || mark;
    float vx = body.vx;
    prepareLowTrot(&vx);
    for (int i=0; i<4; ++i) {
        gm.x_step[i] = mark ? 0 : (MoveMode == MOVE_MODE_WHEEL || wheelMode ? 0 : vx)
            + YAW_STRIDE_SCALE*sign.nppn[i]*legYaw;
        gm.y_step[i] = mark ? 0 : body.vy + sign.ppnn[i]*legYaw;
    }
    bool has_stride = mark;
    for (int i=0; i<4; ++i)
        has_stride |= std::fabs(gm.x_step[i])>1 || std::fabs(gm.y_step[i])>1;
    // High walk retains its deployed wheel/translation dispatch; its own
    // planner applies stride limits and the support-centroid transfer.
    if (!want_high) moving &= has_stride;

    // Only one planner owns the feet. Finish its stop before switching gait types.
    if (ordinary_gait.active()) {
        applyFeet(ordinary_gait.update(ordinaryInput(moving && !want_high), motion_time::stepMicroseconds()));
        gait_reinit = 0;
        return;
    }
    if (high_walk.active() || (want_high && moving)) {
        updateWalk(moving && want_high, legYaw);
        return;
    }
    if (moving && !want_high) {
        ordinary_gait.begin(currentFeet(), takeLead02());
        applyFeet(ordinary_gait.update(ordinaryInput(true), motion_time::stepMicroseconds()));
        gait_reinit = 0;
    }
}
