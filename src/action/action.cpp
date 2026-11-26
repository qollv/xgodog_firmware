#include "action/action.h"
#include "robot/state.h"
#include "motor/motor.h"
#include "motion/time.h"
#include "motion/gait.h"
#include <Arduino.h>
#include <math.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr uint32_t kResetTicks = SEC_2_TICK(0.6);
constexpr uint8_t kResetMotorSpeed = 40;
// 255 interrupt: let the gait land and center the feet before Reset.
constexpr uint32_t kStopSettleTimeoutMs = 600;
std::atomic<bool> stopPendingReset{false};
uint32_t stopStartMs = 0;
std::atomic<uint32_t> resetStartedMs{0};
constexpr uint32_t kLoopPauseTicks = SEC_2_TICK(1.0);
constexpr uint16_t LOW_SPEED = 90;
constexpr uint16_t MEDIUM_SPEED = 160;

enum ActionId : uint8_t {
    Get_Down_ID = 1,
    Get_Up_ID = 2,
    Crawling_ID = 3,
    Turn_Around_ID = 4,
    Squat_ID = 6,
    Turn_Roll_ID = 7,
    Turn_Pitch_ID = 8,
    Turn_Yaw_ID = 9,
    Turn_RPY_ID = 10,
    Pee_ID = 11,
    Sit_Down_ID = 12,
    Wave_ID = 13,
    Stretch_ID = 14,
    Swing_Pitch_ID = 15,
    Swing_Roll_ID = 16,
    Ask_For_Food_ID = 17,
    Find_Food_ID = 18,
    Handshake_ID = 19,
    Show_Arm_ID = 20,
    PushUp_ID = 21,
    Turn_Pitch_Yaw_ID = 22,
    Turn_Z_Yaw_ID = 23,
    Turn_X_Roll_ID = 24,
    ARM_SWING_ID = 25,
    Arm_Wave_ID = 26,
    Arm_Dance_ID = 27,
    GrabHigh_ID = 0x80,
    GrabSide_ID = 0x84,
};

enum class PoseMode : uint8_t { Motor = 1, Body = 2 };

// Motor poses use 12 joint angles + 5 arm values. Body poses use
// position/RPY, velocity, speed preset, 6 oscillation periods + 5 arm values.
struct Posture {
    uint32_t duration_tick;
    PoseMode mode;
    uint16_t motor_speed;
    float data[21];
};

struct ActionDefinition {
    uint8_t num_postures;
    const Posture* postures;
    bool (*update)(uint32_t elapsedTick);
};

template <size_t N>
constexpr ActionDefinition keyframes(const Posture (&postures)[N]) {
    static_assert(N < 255, "Action frame index must fit in uint8_t");
    return {static_cast<uint8_t>(N), postures, nullptr};
}

const uint8_t kLoopActions[] = {Get_Down_ID,Get_Up_ID,Crawling_ID,Turn_Around_ID,Squat_ID,Turn_Roll_ID,
    Turn_Pitch_ID,Turn_Yaw_ID,Turn_RPY_ID,Pee_ID,Sit_Down_ID,Wave_ID,Stretch_ID,
    Swing_Pitch_ID,Swing_Roll_ID,Ask_For_Food_ID,Find_Food_ID,Handshake_ID,Show_Arm_ID,
    PushUp_ID,Turn_Pitch_Yaw_ID,Turn_Z_Yaw_ID,Turn_X_Roll_ID,ARM_SWING_ID,Arm_Wave_ID,Arm_Dance_ID};

//姿态定义 整机模式 六维位姿+三维速度+步频+六维周期
//         舵机模式 12个舵机
// 最后5个数字 机械臂控制模式 如果是0 后面四个数字就是 51 52 53 54
// 如果是1 后面四个数字就是 51 x y z
//Get_Down
const Posture Get_Down_Postures[] = {
    {SEC_2_TICK(2.0),  PoseMode::Body, LOW_SPEED, {0, 0, -100.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Get_Up
const Posture Get_Up_Postures[] = {
    {SEC_2_TICK(2.0),  PoseMode::Body, LOW_SPEED, {0, 0, 990, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Crawling
const Posture Crawling_Postures[] = {
    {SEC_2_TICK(3.5),  PoseMode::Body, MEDIUM_SPEED, {0, 0, -100.0, 0, 0, 0,   8, 0, 0, SLOW_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    // Settle frame: zero motion so the gait brakes and centers the feet.
    {SEC_2_TICK(0.7),  PoseMode::Body, MEDIUM_SPEED, {0, 0, -100.0, 0, 0, 0,   0, 0, 0, SLOW_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

// Turn_Around
const Posture Turn_Around_Postures[] = {
    {SEC_2_TICK(3.8),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,   0, 0, 0.5, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    // Settle frame: zero motion so the gait brakes and centers the feet.
    {SEC_2_TICK(0.7),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,   0, 0, 0,   NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Squat
const Posture Squat_Postures[] = {
    {SEC_2_TICK(0.5),  PoseMode::Body, 0, {0, 0, 990, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Body, 0, {0, 0, -100.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Body, 0, {0, 0, 990, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Body, 0, {0, 0, -100.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Body, 0, {0, 0, 990, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Body, 0, {0, 0, -100.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Turn_Roll
const Posture Turn_Roll_Postures[] = {
    {SEC_2_TICK(3.0),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,  0, 0, 0,NORMAL_SPEED,  0, 0, 0, 50, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Turn_Pitch
const Posture Turn_Pitch_Postures[] = {
    {SEC_2_TICK(3.0),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,  0, 0, 0,NORMAL_SPEED,  0, 0, 0, 0, 50, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Turn_Yaw
const Posture Turn_Yaw_Postures[] = {
    {SEC_2_TICK(3.0),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,  0, 0, 0,NORMAL_SPEED,  0, 0, 0, 0, 0, 50,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Turn_RPY
const Posture Turn_RPY_Postures[] = {
    {SEC_2_TICK(2.0),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,  0, 0, 0,NORMAL_SPEED,  0, 0, 0, 1, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(2.0),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,  0, 0, 0,NORMAL_SPEED,  0, 0, 0, 0, 1, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(2.0),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,  0, 0, 0,NORMAL_SPEED,  0, 0, 0, 0, 0, 1,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Pee
const Posture Pee_Postures[] = {
    //秒数             模式         舵机速度   参数
    {SEC_2_TICK(1.5),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, -0.1, -21.0, 90.0, -0.1, -21.0, 90.0, -0.1, -21.0, 90.0, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(1.5),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, -1.4, 68.7, -4.4, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.2),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, 3.5, 68.7, -30.0, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.2),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, -11.2, 68.7, -30.0, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.2),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, 3.5, 68.7, -30.0, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.2),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, -11.2, 68.7, -30.0, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.2),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, 3.5, 68.7, -30.0, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(1.2),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, -11.2, 68.7, -30.0, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.2),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, 3.5, 68.7, -30.0, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.2),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, -11.2, 68.7, -30.0, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.2),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, 3.5, 68.7, -30.0, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(1.2),  PoseMode::Motor, LOW_SPEED, {-21.0, 90.0, 4.1, -1.4, 68.7, -4.4, -11.2, 68.7, -30.0, -21.0, 90.0, 4.1,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Sit_Down
const Posture Sit_Down_Postures[] = {
    {SEC_2_TICK(3.0),  PoseMode::Motor, LOW_SPEED, {50.1, 38.3, -0.1, 50.1, 38.3, -0.1, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Wave
const Posture Wave_Postures[] = {
    {SEC_2_TICK(1.0),  PoseMode::Motor, LOW_SPEED, {8.4, 38.3, -0.1, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {-25.9, -34.6, -0.1, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {-25.9, -34.6, 11.2, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {-25.9, -34.6, -17.1, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {-25.9, -34.6, 11.2, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {-25.9, -34.6, -17.1, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {-25.9, -34.6, 11.2, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {-25.9, -34.6, -17.1, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {-25.9, -34.6, 11.2, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {-25.9, -34.6, -17.1, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {-25.9, -34.6, 11.2, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {2.5, 44.4, -0.1, 8.4, 38.3, 24.1, 3.5, 35.3, -0.1, 3.5, 35.3, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Stretch
const Posture Stretch_Postures[] = {
    {SEC_2_TICK(3.7),  PoseMode::Motor, LOW_SPEED, {55.0, -50.8, -0.1, 55.0, -50.8, -0.1, 10, 40.5, -0.1, 10, 40.5, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Motor, LOW_SPEED, {40.3, 32.3, -0.1, 40.3, 32.3, -0.1, 8.4, 50.5, -0.1, 8.4, 50.5, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(3.7),  PoseMode::Motor, LOW_SPEED, {45.2, 44.4, -0.1, 45.2, 44.4, -0.1, 55.0, 80.9, -0.1, 55.0, 80.9, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Swing_Pitch
const Posture Swing_Pitch_Postures[] = {
    {SEC_2_TICK(4.0),  PoseMode::Body, MEDIUM_SPEED, {0, 0, 0.0-10.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   40, 0, 0, 0, 60, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Swing_Roll
const Posture Swing_Roll_Postures[] = {
    {SEC_2_TICK(4.0),  PoseMode::Body, MEDIUM_SPEED, {0, 0, 0.0-10.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 5, 0, 40, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Ask_For_Food
const Posture Ask_For_Food_Postures[] = {
    {SEC_2_TICK(1.0),  PoseMode::Body, LOW_SPEED, {0, 0, -20, 0, -20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(3.5),  PoseMode::Body, LOW_SPEED, {0, 0, -20, 0, -20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 1,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Find_Food
const Posture Find_Food_Postures[] = {
    {SEC_2_TICK(0.5),  PoseMode::Body, LOW_SPEED, {15, 0, -20, 0, 20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.3),  PoseMode::Body, LOW_SPEED,         {15, 0, -20, 0, 20, 15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 10, 15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 20, 15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 10, 15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 20, 15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.3),  PoseMode::Body, LOW_SPEED,         {15, 0, -20, 0, 20, -15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 10, -15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 20, -15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 10, -15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 20, -15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Body, LOW_SPEED, {0, 0, -20, 0, 5, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(2.5),  PoseMode::Body, MEDIUM_SPEED, {0, 0, -20, 0, 5, 0,   8, 0, 0, SLOW_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.5),  PoseMode::Body, LOW_SPEED, {15, 0, -20, 0, 20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.3),  PoseMode::Body, LOW_SPEED,         {15, 0, -20, 0, 20, 15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 10, 15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 20, 15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 10, 15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 20, 15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.3),  PoseMode::Body, LOW_SPEED,         {15, 0, -20, 0, 20, -15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 10, -15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 20, -15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 10, -15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.1),  PoseMode::Body, 0,         {15, 0, -20, 0, 20, -15,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Handshake
const Posture Handshake_Postures[] = {
    {SEC_2_TICK(0.8),  PoseMode::Motor, LOW_SPEED, {50.1, 38.3, -0.1, 50.1, 20.0, -0.1, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.8),  PoseMode::Motor, LOW_SPEED, {50.1, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {-6.3, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {8.4, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {-6.3, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {8.4, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {-6.3, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {8.4, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {-6.3, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(1.7),  PoseMode::Motor, 0, {8.4, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {-6.3, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {8.4, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {-6.3, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {8.4, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {-6.3, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {8.4, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.15),  PoseMode::Motor, 0, {-6.3, 14.0, -0.1, 50.1, 20.0, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(1.7),  PoseMode::Motor, 0, {8.4, 14.0, -0.1, 50.1, 38.3, 15.9, -30.8, 71.8, -0.1, -30.8, 71.8, -0.1,   0, 0.0, 85.0, -90.0, 0.0}},
};

//showarm
const Posture ShowArm_Postures[] = {
    {SEC_2_TICK(0.75),    PoseMode::Body, LOW_SPEED, {0, 0, 0.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   2, 0.0, L2, 0.0, L1}},
    {SEC_2_TICK(3.5),     PoseMode::Body, 0,         {0, 0, 0.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 50, 0, 0, 0,   2, 0.0, L2, 0.0, L1}},
    {SEC_2_TICK(3.5),     PoseMode::Body, 0,         {0, 0, 0.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 50, 0, 50, 0,   2, 0.0, L2, 0.0, L1}},
};

//PushUp_ID
const Posture PushUp_Postures[] = {
    {SEC_2_TICK(1.2),  PoseMode::Motor, MEDIUM_SPEED, {35.2, 30.0, -0.1, 35.2, 30.0, -0.1, 55.0, 80.9, -0.1, 55.0, 80.9, -0.1,    0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.8),  PoseMode::Motor, MEDIUM_SPEED, {-40.0, 70.0, -0.1, -40.0, 70.0, -0.1, 55.0, 80.9, -0.1, 55.0, 80.9, -0.1,    0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.8),  PoseMode::Motor, MEDIUM_SPEED, {35.2, 30.0, -0.1, 35.2, 30.0, -0.1, 55.0, 80.9, -0.1, 55.0, 80.9, -0.1,    0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.8),  PoseMode::Motor, MEDIUM_SPEED, {-40.0, 70.0, -0.1, -40.0, 70.0, -0.1, 55.0, 80.9, -0.1, 55.0, 80.9, -0.1,    0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.8),  PoseMode::Motor, MEDIUM_SPEED, {35.2, 30.0, -0.1, 35.2, 30.0, -0.1, 55.0, 80.9, -0.1, 55.0, 80.9, -0.1,    0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.8),  PoseMode::Motor, MEDIUM_SPEED, {-40.0, 70.0, -0.1, -40.0, 70.0, -0.1, 55.0, 80.9, -0.1, 55.0, 80.9, -0.1,    0, 0.0, 85.0, -90.0, 0.0}},
};

//Turn_Pitch_Yaw_ID			22
const Posture Turn_Pitch_Yaw_Postures[] = {
    {SEC_2_TICK(5),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 45, 30,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Turn_Z_Yaw_ID				23
const Posture Turn_Z_Yaw_Postures[] = {
    {SEC_2_TICK(5),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 45, 0, 0, 30,   0, 0.0, 85.0, -90.0, 0.0}},
};

//Turn_X_Roll_ID				24
const Posture Turn_X_Roll_Postures[] = {
    {SEC_2_TICK(5),  PoseMode::Body, 0, {0, 0, 0.0, 0, 0, 0,   0, 0, 0, NORMAL_SPEED,   45, 0, 0, 20 ,0, 0,   0, 0.0, 85.0, -90.0, 0.0}},
};

//ARM_SWING 大臂/小臂每0.2s切换，转台每0.8s切换±45
const Posture Arm_Swing_Postures[] = {
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 0.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 0.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 0.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 0.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 0.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 0.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 0.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 0.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 10, 0.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 10, 0.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 10, 0.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 10, 0.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, -10, 0.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, -10, 0.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, -10, 0.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, -10, 0.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 5.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, -5.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 5.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, -5.0, 0, 0, -20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, 45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 5.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, -5.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, 5.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0,  15.0, -15.0, -45.0}},
    {SEC_2_TICK(0.2),  PoseMode::Body, LOW_SPEED, {0, 0, -5.0, 0, 0, 20,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0, 0, 0,   0,  0.0, -15.0,  15.0, -45.0}},
};

// 机械臂挥手：5 秒，1 Hz 连续轨迹。
bool ActionArmWave(uint32_t elapsedTick){
    const float duration = 5.0f;
    const float freq = 1.0f;
    float t = (float)elapsedTick / (float)SYSTEM_TICK_HZ;
    if (t >= duration) {
        return true;
    }

    float phase = 2.0f * (float)PI * freq * t;
    float theta = atanf(50.0f*sinf(phase)/80.0f);
    xgo_mode = XGO_MODE_BODY;
    armCtrlMode = 1;
    armPosMode = 0;
    motor[12].des_angle = 0;
    arm_x = 80.0f/cosf(theta);
    arm_z = 40.0f + 50.0f * cosf(phase);
    motor[15].des_angle = theta*57.3f;
    body.yaw = 15*cosf(phase);
    body.pit = 15*sinf(phase);
    return false;
}

// 机械臂舞蹈：5秒内按 sin 轨迹连续摆动
bool ActionArmDance(uint32_t elapsedTick){
    const float duration = 5.0f;
    const float freq = 2.0f;
    float t = (float)elapsedTick / (float)SYSTEM_TICK_HZ;
    if (t >= duration) {
        return true;
    }

    float phase = 2.0f * (float)PI * freq * t;
    xgo_mode = XGO_MODE_BODY;
    armCtrlMode = 1;
    armPosMode = 0;
    body.z = 90+20.0f * cosf(phase);
    arm_x = 90+30.0f * cosf(phase);
    arm_z = 50.0f;
    return false;
}

//Stand_Up
const Posture Stand_Up_Postures[] = {
    {SEC_2_TICK(1.5),  PoseMode::Motor, 0, {50, 30, 0.0, 50, 30, 0.0, 50.0, 30.0, 0.0, 50.0, 30.0, 0.0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.3),  PoseMode::Motor, 0, {-80, 80, 0.0, -80, 80, 0.0, 0.0, 55.0, 0.0, 0.0, 55.0, 0.0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(0.3),  PoseMode::Motor, 0, {50, 20, 0.0,  50, 20, 0.0,  -30.0, 30.0, 0.0,  -30.0, 30.0, 0.0,   0, 0.0, 85.0, -90.0, 0.0}},
    {SEC_2_TICK(10.2),  PoseMode::Motor, 0, {-50, 80, 0.0, -50, 80, 0.0, 0.0, 100.0, 0.0, 0.0, 100.0, 0.0,   0, 0.0, 85.0, -90.0, 0.0}},
};
const Posture GrabHigh_Postures[] = {
    {SEC_2_TICK(1.5),  PoseMode::Body, LOW_SPEED, {0, 0, 10, 0, -15, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, -65, L2, 0, L1}},
    {SEC_2_TICK(1),  PoseMode::Body, LOW_SPEED, {0, 0, 10, 0, -15, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, -65, 130.0, 0, 60}},
    {SEC_2_TICK(1),  PoseMode::Body, LOW_SPEED, {0, 0, 10, 0, -15, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, 65, 130.0, 0, 60}},
    {SEC_2_TICK(1),  PoseMode::Body, LOW_SPEED, {0, 0, 10, 0, -15, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, 65, 50.0, 0, 50}},
};
const Posture GrabMiddle_Postures[] = {
    {SEC_2_TICK(1.5), PoseMode::Body, LOW_SPEED, {0, 0, 5, 0, -4, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, -65, L2, 0, L1}},
    {SEC_2_TICK(1),  PoseMode::Body, LOW_SPEED, {0, 0, 5, 0, -4, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, -65, 145.0, 0, 40}},
    {SEC_2_TICK(1),  PoseMode::Body, LOW_SPEED, {0, 0, 5, 0, -4, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, 65, 145.0, 0, 40}},
    {SEC_2_TICK(1),  PoseMode::Body, LOW_SPEED, {0, 0, 5, 0, -4, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, 65, 90.0, 0, 60}},
};
const Posture GrabLow_Postures[] = {
    {SEC_2_TICK(1.5),  PoseMode::Body, LOW_SPEED, {0, 0, -20, 0, 20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, -65, L2, 0, L1}},
    {SEC_2_TICK(1),  PoseMode::Body, LOW_SPEED, {0, 0, -20, 0, 20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, -65, 110.0, 0, -60}},
    {SEC_2_TICK(1),  PoseMode::Body, LOW_SPEED, {0, 0, -20, 0, 20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, 65, 110.0, 0, -60}},
    {SEC_2_TICK(1),  PoseMode::Body, LOW_SPEED, {0, 0, -20, 0, 20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, 65, 90.0, 0, 60}},
};
const Posture GrabLowSide_Postures[] = {
    {SEC_2_TICK(1.0),  PoseMode::Body, LOW_SPEED, {15, 0, -25, 0, 20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, -65, L2, 40, L1}},
    {SEC_2_TICK(0.8),  PoseMode::Body, LOW_SPEED, {15, 0, -25, 0, 20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, -65, 120.0, 40, -60}},
    {SEC_2_TICK(0.8),  PoseMode::Body, LOW_SPEED, {15, 0, -25, 0, 20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, 65, 120.0, 40, -60}},
    {SEC_2_TICK(0.8),  PoseMode::Body, LOW_SPEED, {15, 0, -25, 0, 20, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, 65, 90.0, 40, 60}},
};
const Posture GrabSide_Postures[] = {
    {SEC_2_TICK(0.9),  PoseMode::Body, LOW_SPEED, {0, 15, -25, -15, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, -65, L2, 90, L1}},
    {SEC_2_TICK(0.7),  PoseMode::Body, LOW_SPEED, {0, 15, -25, -15, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, -65, 120.0, 90, -60}},
    {SEC_2_TICK(0.7),  PoseMode::Body, LOW_SPEED, {0, 15, -25, -15, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, 65, 120.0, 90, -60}},
    {SEC_2_TICK(0.7),  PoseMode::Body, LOW_SPEED, {0, 15, -25, -15, 0, 0,   0, 0, 0, NORMAL_SPEED,   0, 0, 0, 0 ,0, 0,   1, 65, 90.0, 0, 60}},
};


const ActionDefinition kStandardActions[] = {
    {},
    keyframes(Get_Down_Postures),
    keyframes(Get_Up_Postures),
    keyframes(Crawling_Postures),
    keyframes(Turn_Around_Postures),
    {},
    keyframes(Squat_Postures),
    keyframes(Turn_Roll_Postures),
    keyframes(Turn_Pitch_Postures),
    keyframes(Turn_Yaw_Postures),
    keyframes(Turn_RPY_Postures),
    keyframes(Pee_Postures),
    keyframes(Sit_Down_Postures),
    keyframes(Wave_Postures),
    keyframes(Stretch_Postures),
    keyframes(Swing_Pitch_Postures),
    keyframes(Swing_Roll_Postures),
    keyframes(Ask_For_Food_Postures),
    keyframes(Find_Food_Postures),
    keyframes(Handshake_Postures),
    keyframes(ShowArm_Postures),
    keyframes(PushUp_Postures),
    keyframes(Turn_Pitch_Yaw_Postures),
    keyframes(Turn_Z_Yaw_Postures),
    keyframes(Turn_X_Roll_Postures),
    keyframes(Arm_Swing_Postures),
    {0, nullptr, ActionArmWave},
    {0, nullptr, ActionArmDance},
    {},
    keyframes(Stand_Up_Postures),
};

const ActionDefinition kGrabActions[] = {
    keyframes(GrabHigh_Postures),
    keyframes(GrabMiddle_Postures),
    keyframes(GrabLow_Postures),
    keyframes(GrabLowSide_Postures),
    keyframes(GrabSide_Postures),
};

const ActionDefinition* findAction(uint8_t id) {
    const ActionDefinition* action = nullptr;
    if (id < sizeof(kStandardActions) / sizeof(kStandardActions[0])) {
        action = &kStandardActions[id];
    } else if (id >= GrabHigh_ID && id <= GrabSide_ID) {
        action = &kGrabActions[id - GrabHigh_ID];
    }
    return action && (action->num_postures || action->update) ? action : nullptr;
}

bool foldArmPending = false;
uint32_t foldArmDeadlineMs = 0;
portMUX_TYPE foldArmMux = portMUX_INITIALIZER_UNLOCKED;

void setFoldedArm() {
    motor[13].des_angle = 90;
    motor[14].des_angle = -89;
    motor[15].des_angle = 0;
    foldArmPending = false;
}

void updateFoldArm() {
    const uint32_t now = millis();
    portENTER_CRITICAL(&foldArmMux);
    if (foldArmPending && static_cast<int32_t>(now - foldArmDeadlineMs) >= 0) {
        setFoldedArm();
    }
    portEXIT_CRITICAL(&foldArmMux);
}

void applyArmPose(const float* data) {
    if (data[0] == 0) {
        armCtrlMode = 0;
        for (unsigned i = 0; i < 4; ++i) {
            motor[12 + i].des_angle = data[1 + i];
        }
    } else if (data[0] > 0) {
        armCtrlMode = 1;
        armPosMode = data[0] > 1 ? 1 : 0;
        motor[12].des_angle = data[1];
        arm_x = data[2];
        motor[15].des_angle = data[3];
        arm_z = data[4];
    }
}

void applyPosture(const Posture& posture) {
    const float* data = posture.data;
    if (posture.mode == PoseMode::Motor) {
        xgo_mode = XGO_MODE_MOTOR;
        for (unsigned i = 0; i < 12; ++i) {
            motor[i].des_angle = data[i];
        }
        applyArmPose(data + 12);
    } else if (posture.mode == PoseMode::Body) {
        xgo_mode = XGO_MODE_BODY;
        body.x = data[0];
        body.y = data[1];
        // Values below 50 mm are relative to the model's standing height.
        body.z = abs(data[2]) < 50.0f ? bodyConfig.z_init + data[2] : data[2];
        body.rol = data[3];
        body.pit = data[4];
        body.yaw = data[5];
        body.vx = data[6];
        body.vy = data[7];
        imu.yaw_speed = data[8];
        if (data[9] == NORMAL_SPEED) gm.T = bodyConfig.NORMAL_T;
        else if (data[9] == SLOW_SPEED) gm.T = bodyConfig.SLOW_T;
        else if (data[9] == FAST_SPEED) gm.T = bodyConfig.FAST_T;
        else if (data[9] == MID_SPEED) gm.T = bodyConfig.MID_T;
        body.x_T = data[10];
        body.y_T = data[11];
        body.z_T = data[12];
        body.rol_T = data[13];
        body.pit_T = data[14];
        body.yaw_T = data[15];
        applyArmPose(data + 16);
    }
}
} // namespace

void Reset(uint8_t mode) {
    stopPendingReset.store(false, std::memory_order_relaxed);
    if (mode > 0) {
        Action_Ctx.id = 0;
        Action_Ctx.frame = 0;
    }
    if (mode > 1) Action_Ctx.loop = false;
    Action_Ctx.cur_tick = 0;
    Action_Ctx.next_tick = 0;
    Reset_Body();
    SetLegMotorSpeed(kResetMotorSpeed);
    SetArmMotorSpeed(kResetMotorSpeed);
    resetStartedMs.store((uint32_t)millis(), std::memory_order_relaxed);
    FoldArm();
}

bool Action_IsRecovering() {
    return (uint32_t)((uint32_t)millis() - resetStartedMs.load(std::memory_order_relaxed))
        < kResetTicks * 1000u / SYSTEM_TICK_HZ;
}

void FoldArm() {
    armCtrlMode = 0;
    const uint32_t now = millis();
    portENTER_CRITICAL(&foldArmMux);
    if (motor[14].fb_angle > 40) {
        motor[13].des_angle = 0;
        motor[14].des_angle = 0;
        foldArmDeadlineMs = now + 300u;
        foldArmPending = true;
    } else {
        setFoldedArm();
    }
    portEXIT_CRITICAL(&foldArmMux);
}

void Falldown() {
    xgo_mode = XGO_MODE_MOTOR;
    FoldArm();
    SetLegMotorSpeed(50);
    SetArmMotorSpeed(50);
    for (unsigned leg = 0; leg < 4; ++leg) {
        motor[leg * 3].des_angle = -90;
        motor[leg * 3 + 1].des_angle = 90;
        motor[leg * 3 + 2].des_angle = -20;
    }
}

void SetAction(uint8_t id) {
    Action_Ctx.id = id;
    Action_Ctx.frame = 0;
    Action_Ctx.cur_tick = 0;
    Action_Ctx.next_tick = 0;
}

// Zero the motion command and wait (bounded) for the gait to land and center.
static bool waitGaitSettled()
{
    body.vx = 0;
    body.vy = 0;
    body.vyaw = 0;
    imu.yaw_speed = 0;
    yaw_track_flag = 0;
    if (gm.gait_type == GAIT_MARKTIME) GaitSetType(GAIT_TROT);
    if (!GaitIsActive()) return true;
    const uint32_t now = millis();
    if (!stopPendingReset.exchange(true, std::memory_order_relaxed)) stopStartMs = now;
    return (now - stopStartMs) >= kStopSettleTimeoutMs;
}

void ActionManager() {
    updateFoldArm();
    if (Action_Ctx.id == 255) {
        if (!waitGaitSettled()) return;
        Reset(2);
        return;
    }
    if (Action_Ctx.loop) {
        if (Action_Ctx.id == 0) Action_Ctx.loop_tick += motion_time::wholeTicks();
        if (Action_Ctx.loop_tick >= kLoopPauseTicks) {
            Action_Ctx.id = kLoopActions[Action_Ctx.loop_id];
            Action_Ctx.loop_tick -= kLoopPauseTicks;
            if (++Action_Ctx.loop_id >= sizeof(kLoopActions) / sizeof(kLoopActions[0])) {
                Action_Ctx.loop_id = 0;
            }
        }
    }
    if (Action_Ctx.id == 0) return;
    const ActionDefinition* action = findAction(Action_Ctx.id);
    if (!action) return;

    Action_Ctx.cur_tick += motion_time::wholeTicks();
    // Both action forms share the standing reset and its 600 ms deadline.
    if (Action_Ctx.frame == 0 &&
        (action->update || Action_Ctx.cur_tick >= Action_Ctx.next_tick)) {
        // A new action can interrupt a moving one; settle before the reset.
        if (!waitGaitSettled()) return;
        Reset(0);
        Action_Ctx.frame = 1;
        Action_Ctx.next_tick = kResetTicks;
        return;
    }
    if (Action_Ctx.cur_tick < Action_Ctx.next_tick) return;

    if (action->update) {
        if (Action_Ctx.frame == 1) {
            Action_Ctx.frame = 2;
            SetLegMotorSpeed(60);
            SetArmMotorSpeed(160);
        }
        if (action->update(Action_Ctx.cur_tick - Action_Ctx.next_tick)) Reset(1);
        return;
    }

    const uint8_t frame = Action_Ctx.frame++;
    if (frame > action->num_postures) {
        Reset(1);
        return;
    }
    const Posture& posture = action->postures[frame - 1];
    SetLegMotorSpeed(posture.motor_speed);
    SetArmMotorSpeed(160);
    applyPosture(posture);
    // Keep deadline remainder; emit at most one keyframe per control tick.
    Action_Ctx.next_tick += posture.duration_tick;
}

// Factory exercise mode is a deployed protocol command, not a host test.
namespace {
constexpr uint32_t kExerciseTaskStack = 2048;
TaskHandle_t exerciseTaskHandle = nullptr;
uint8_t exerciseMode = 0;

void exerciseTask(void*) {
    for (;;) {
        if (exerciseMode == 0) {
            exerciseTaskHandle = nullptr;
            vTaskDelete(nullptr);
        }
        SetLegMotorSpeed(0);
        SetArmMotorSpeed(0);
        if (exerciseMode == 2) {
            body.vx = bodyConfig.vx_max;
            vTaskDelay(pdMS_TO_TICKS(2000));
            body.vx = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            body.vx = bodyConfig.vx_min;
            vTaskDelay(pdMS_TO_TICKS(2000));
            body.vx = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            body.vy = bodyConfig.vy_max;
            vTaskDelay(pdMS_TO_TICKS(2000));
            body.vy = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            body.vy = bodyConfig.vy_min;
            vTaskDelay(pdMS_TO_TICKS(2000));
            body.vy = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            imu.yaw_speed = imu.yaw_speed_max;
            vTaskDelay(pdMS_TO_TICKS(2000));
            imu.yaw_speed = 0;
            body.vyaw = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            imu.yaw_speed = imu.yaw_speed_min;
            vTaskDelay(pdMS_TO_TICKS(2000));
            imu.yaw_speed = 0;
            body.vyaw = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        float* const periods[] = {
            &body.x_T, &body.y_T, &body.z_T, &body.rol_T, &body.pit_T, &body.yaw_T
        };
        for (float* period : periods) {
            *period = 0x20;
            vTaskDelay(pdMS_TO_TICKS(2000));
            *period = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
} // namespace

void SetTestMode(uint8_t mode) {
    exerciseMode = mode;
    if (mode != 0 && exerciseTaskHandle == nullptr) {
        xTaskCreate(exerciseTask, "exercise", kExerciseTaskStack, nullptr, 0, &exerciseTaskHandle);
    }
}
