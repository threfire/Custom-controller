#ifndef JOINT_IMPEDANCE_H
#define JOINT_IMPEDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* =========================
 * 单位约定
 * q, q_ref          : rad
 * dq, dq_ref        : rad/s
 * tau_xxx           : N*m   (关节侧力矩)
 * current command   : A     (电机侧电流)
 * dt_s              : s
 * =========================
 */

typedef struct
{
    float kp;                       /* 位置刚度增益 */
    float kd;                       /* 速度阻尼增益 */

    float torque_limit;             /* 关节侧力矩限幅, <=0 表示关闭 */
    float torque_rate_limit;        /* 关节侧力矩变化率限幅(N*m/s), <=0 表示关闭 */

    float vel_lpf_alpha;            /* 速度一阶低通滤波系数, 推荐 0.8~0.95 */

    float torque_constant;          /* 电机力矩常数 Kt, 单位 N*m/A, 电机侧 */
    float gear_ratio;               /* 减速比, 例如 6.0f, 9.0f */
    float transmission_efficiency;  /* 传动效率, 0~1, 推荐先取 1.0f */

    bool  enable_angle_wrap;        /* 对位置误差做[-pi, pi]归一化, 多圈关节通常关掉 */
} JointImpConfig_t;

typedef struct
{
    JointImpConfig_t cfg;

    bool  enabled;
    bool  has_prev_measurement;

    /* 测量量 */
    float q_meas;                   /* 当前关节角 */
    float dq_meas;                  /* 当前关节角速度 */
    float q_prev;                   /* 上次关节角 */
    float dq_lpf;                   /* 滤波后的速度内部状态 */

    /* 参考量 */
    float q_ref;
    float dq_ref;

    /* 外部前馈 */
    float tau_gravity;              /* 重力补偿前馈 */
    float tau_feedforward;          /* 额外前馈, 比如轨迹前馈/摩擦补偿等 */

    /* 分项输出, 便于调试 */
    float tau_p;
    float tau_d;
    float tau_raw;
    float tau_cmd;
    float current_cmd;
} JointImpController_t;

/* 默认参数 */
void JointImp_DefaultConfig(JointImpConfig_t *cfg);

/* 初始化 */
void JointImp_Init(JointImpController_t *ctrl, const JointImpConfig_t *cfg);

/* 复位内部状态, 并把参考位置锁到当前值 */
void JointImp_Reset(JointImpController_t *ctrl, float q_now);

/* 开关控制器 */
void JointImp_SetEnable(JointImpController_t *ctrl, bool enable);

/* 使能并锁定当前姿态为参考点 */
void JointImp_EnableHoldCurrent(JointImpController_t *ctrl);

/* 更新测量: 只给位置, 内部差分估速 */
void JointImp_UpdateMeasurement(JointImpController_t *ctrl, float q_meas, float dt_s);

/* 更新测量: 外部直接给位置和速度 */
void JointImp_UpdateMeasurementWithVelocity(JointImpController_t *ctrl, float q_meas, float dq_meas);

/* 设置参考 */
void JointImp_SetReference(JointImpController_t *ctrl, float q_ref, float dq_ref);

/* 设置增益 */
void JointImp_SetGains(JointImpController_t *ctrl, float kp, float kd);

/* 设置重力补偿 */
void JointImp_SetGravityTorque(JointImpController_t *ctrl, float tau_g);

/* 设置额外前馈 */
void JointImp_SetFeedforwardTorque(JointImpController_t *ctrl, float tau_ff);

/* 计算关节侧力矩命令 */
float JointImp_ComputeTorque(JointImpController_t *ctrl, float dt_s);

/* 关节力矩 -> 电机电流 */
float JointImp_TorqueToCurrent(const JointImpController_t *ctrl, float tau_joint_cmd);

/* 直接计算电流命令 */
float JointImp_ComputeCurrentCmd(JointImpController_t *ctrl, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* JOINT_IMPEDANCE_H */
