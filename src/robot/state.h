#pragma once
#include "robot/types.h"
#include "motion/config.h"
#include <string>

#define XGOMINI 1
#define XGOLITE 2

#define XGO_FIRMWARE_VERSION "-732-YB"

struct RobotProfile {
    BodyParam_t body;
    float stepHeight;
    char firmwareVersion[11];
};

// Pure configuration builder. Unknown models retain the historical Lite fallback;
// wheel capability is enabled only for the established flag value of one.
RobotProfile RobotProfile_Create(uint8_t model, uint8_t wheel);

/* 版本 */
extern char firmwareVersion[11];
extern uint8_t hasWheel;

/* 状态 */
#define XGO_MODE_BODY 0
#define XGO_MODE_LEG 1
#define XGO_MODE_MOTOR 2
extern Odom odom;

//轮足混合运动模式
#define WHEEL_ORDER_MAX 230 // Signed drive command limit; not a torque limit.
#define WHEEL_OPEN_LOOP_GAIN 100.0f
#define MOVE_MODE_HYBRID 0
#define MOVE_MODE_WHEEL 1
#define MOVE_MODE_LEG 2
#define MOVE_MODE_WHEEL2 3
extern uint8_t MoveMode;

extern uint8_t xgo_mode;
extern uint8_t isReceiveMsg;
extern uint8_t inCalibration;
extern Sign sign;
extern uint8_t feedback_flag;
extern uint8_t isIMUErr;
extern uint8_t isMotorErr;
extern long motorRevTime;
/* 运动 */
extern uint8_t yaw_track_flag;
extern PID_t move_yaw_pid;
extern GaitManager_t gm;

extern float sr;

/* 动作 */
extern ActionContext_t Action_Ctx;

/* 机械臂 */
extern uint8_t armCtrlMode;
extern uint8_t armPosMode;
extern float arm_x;
extern float arm_z;
extern float arm_theta;
extern float arm_r;
#define L1 60.0f
#define L2 100.0f

#define ARM_X_MAX (L1+L2)
#define ARM_X_MIN -(L1+L2)/2
#define ARM_Z_MAX (L1+L2)
#define ARM_Z_MIN -L2

/* 机器人构型 */
extern uint8_t xgo_version;
extern Body_t body;
extern Leg_t leg[4];
extern BodyParam_t bodyConfig;

/* 电机：12 个腿关节和 4 个机械臂/旋转关节 */
#define MOTOR_NUM 16
#define MOTOR_UNKNOWN 0
#define MOTOR_009   1
#define MOTOR_3032  2
#define MOTOR_RC05P 3
extern Motor_t motor[MOTOR_NUM];
extern Motor_t ex_motor[10];
extern uint8_t motor_version;
/* 轮子 */
extern Wheel_t wheel[4];

/* IMU */
extern imu_t imu;
extern uint8_t imu_type;
extern volatile uint8_t isIMUInit;
// Reset by Init_Motor; published by servo TX after final startup setup/PD.
extern volatile bool motorInitializationComplete;

/* BLE */
extern std::string BLEName;

/*外设*/
extern uint8_t user_rgb[12];

/* Debug（协议 main 128，sub 1~10） */
extern float debug_var[10];

void Init_Globals();
void Reset_Body();
void Init_Config();
