#include "joint_impedance.h"

#include <string.h>

#define JOINT_IMP_EPSILON   (1.0e-6f)
#define JOINT_IMP_PI        (3.14159265358979323846f)
#define JOINT_IMP_TWO_PI    (6.28318530717958647692f)

/**
 * @brief  浮点数限幅
 * @param  x        输入值
 * @param  min_val  下限
 * @param  max_val  上限
 * @retval 限幅后的结果
 * @note   常用于力矩限幅、力矩变化率限幅等场景
 */
static float joint_imp_clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

/**
 * @brief  浮点数绝对值
 * @param  x 输入值
 * @retval x 的绝对值
 * @note   仅用于本文件内部，避免额外依赖 math.h
 */
static float joint_imp_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

/**
 * @brief  将角度归一化到 [-pi, pi]
 * @param  x 输入角度，单位 rad
 * @retval 归一化后的角度
 * @note
 *         适用于单圈旋转关节的角度误差处理。
 *         如果是多圈关节，通常不建议启用该功能。
 */
static float joint_imp_wrap_pi(float x)
{
    while (x > JOINT_IMP_PI)
    {
        x -= JOINT_IMP_TWO_PI;
    }

    while (x < -JOINT_IMP_PI)
    {
        x += JOINT_IMP_TWO_PI;
    }

    return x;
}

/**
 * @brief  一阶低通滤波器，用于速度信号平滑
 * @param  alpha  滤波系数，范围建议 0.8 ~ 0.95
 * @param  prev   上一时刻滤波结果
 * @param  input  当前输入值
 * @retval 当前滤波输出
 * @note
 *         alpha 越大，滤波越强，响应越慢。
 *         该函数主要用于位置差分后的速度去噪。
 */
static float joint_imp_filter_velocity(float alpha, float prev, float input)
{
    alpha = joint_imp_clampf(alpha, 0.0f, 0.9999f);
    return alpha * prev + (1.0f - alpha) * input;
}

/**
 * @brief  填充一组默认控制参数
 * @param  cfg 指向配置结构体的指针
 * @retval 无
 * @note
 *         调用后可在外部按具体关节参数继续修改：
 *         - kp / kd
 *         - 力矩限幅
 *         - 力矩变化率限幅
 *         - 电机力矩常数、减速比、效率等
 */
void JointImp_DefaultConfig(JointImpConfig_t *cfg)
{
    if (cfg == (void *)0)
    {
        return;
    }

    cfg->kp = 0.0f;
    cfg->kd = 0.0f;

    cfg->torque_limit = 0.0f;
    cfg->torque_rate_limit = 0.0f;

    cfg->vel_lpf_alpha = 0.80f;

    cfg->torque_constant = 0.0f;
    cfg->gear_ratio = 1.0f;
    cfg->transmission_efficiency = 1.0f;

    cfg->enable_angle_wrap = false;
}

/**
 * @brief  初始化关节阻抗控制器对象
 * @param  ctrl 控制器对象指针
 * @param  cfg  配置参数指针
 * @retval 无
 * @note
 *         该函数会清零控制器内部状态，并拷贝一份配置参数。
 *         调用后建议再执行一次 JointImp_Reset()，将参考位置锁定到当前角度。
 */
void JointImp_Init(JointImpController_t *ctrl, const JointImpConfig_t *cfg)
{
    if ((ctrl == (void *)0) || (cfg == (void *)0))
    {
        return;
    }

    memset(ctrl, 0, sizeof(JointImpController_t));
    ctrl->cfg = *cfg;
}

/**
 * @brief  复位控制器内部状态，并将参考位置设为当前值
 * @param  ctrl  控制器对象指针
 * @param  q_now 当前关节角度，单位 rad
 * @retval 无
 * @note
 *         适用于以下场景：
 *         - 上电初始化
 *         - 模式切换
 *         - 故障恢复
 *         - 重新锁定当前姿态为目标姿态
 */
void JointImp_Reset(JointImpController_t *ctrl, float q_now)
{
    if (ctrl == (void *)0)
    {
        return;
    }

    ctrl->has_prev_measurement = false;

    ctrl->q_meas = q_now;
    ctrl->q_prev = q_now;
    ctrl->dq_meas = 0.0f;
    ctrl->dq_lpf  = 0.0f;

    ctrl->q_ref  = q_now;
    ctrl->dq_ref = 0.0f;

    ctrl->tau_p = 0.0f;
    ctrl->tau_d = 0.0f;
    ctrl->tau_raw = 0.0f;
    ctrl->tau_cmd = 0.0f;
    ctrl->current_cmd = 0.0f;
}

/**
 * @brief  设置控制器使能状态
 * @param  ctrl   控制器对象指针
 * @param  enable true: 使能, false: 关闭
 * @retval 无
 * @note
 *         关闭时会清零当前输出项，避免旧命令残留。
 *         但不会修改参考位置和配置参数。
 */
void JointImp_SetEnable(JointImpController_t *ctrl, bool enable)
{
    if (ctrl == (void *)0)
    {
        return;
    }

    ctrl->enabled = enable;

    if (enable == false)
    {
        ctrl->tau_p = 0.0f;
        ctrl->tau_d = 0.0f;
        ctrl->tau_raw = 0.0f;
        ctrl->tau_cmd = 0.0f;
        ctrl->current_cmd = 0.0f;
    }
}

/**
 * @brief  使能控制器，并将当前测量位置锁定为参考位置
 * @param  ctrl 控制器对象指针
 * @retval 无
 * @note
 *         常用于“保持当前姿态”模式：
 *         - q_ref 设为当前 q_meas
 *         - dq_ref 设为 0
 *         对纯阻尼模式也可使用，但此时 kp 通常设为 0。
 */
void JointImp_EnableHoldCurrent(JointImpController_t *ctrl)
{
    if (ctrl == (void *)0)
    {
        return;
    }

    ctrl->q_ref = ctrl->q_meas;
    ctrl->dq_ref = 0.0f;
    ctrl->enabled = true;
}

/**
 * @brief  仅根据关节位置更新测量量，内部使用差分法估算角速度
 * @param  ctrl  控制器对象指针
 * @param  q_meas 当前关节角度，单位 rad
 * @param  dt_s   控制周期，单位 s
 * @retval 无
 * @note
 *         速度估算公式：
 *             dq_raw = (q[k] - q[k-1]) / dt
 *         然后经过一阶低通滤波得到 dq_meas。
 *         如果你的驱动器能直接提供速度，优先建议用
 *         JointImp_UpdateMeasurementWithVelocity()。
 */
void JointImp_UpdateMeasurement(JointImpController_t *ctrl, float q_meas, float dt_s)
{
    float dq_raw = 0.0f;

    if (ctrl == (void *)0)
    {
        return;
    }

    ctrl->q_meas = q_meas;

    if ((dt_s > JOINT_IMP_EPSILON) && (ctrl->has_prev_measurement == true))
    {
        dq_raw = (q_meas - ctrl->q_prev) / dt_s;
        ctrl->dq_lpf = joint_imp_filter_velocity(ctrl->cfg.vel_lpf_alpha, ctrl->dq_lpf, dq_raw);
        ctrl->dq_meas = ctrl->dq_lpf;
    }
    else
    {
        /* 首次进入时没有上一拍数据，速度先置零 */
        ctrl->dq_meas = 0.0f;
        ctrl->dq_lpf = 0.0f;
        ctrl->has_prev_measurement = true;
    }

    ctrl->q_prev = q_meas;
}

/**
 * @brief  使用外部给定的关节位置和关节速度更新测量量
 * @param  ctrl    控制器对象指针
 * @param  q_meas  当前关节角度，单位 rad
 * @param  dq_meas 当前关节角速度，单位 rad/s
 * @retval 无
 * @note
 *         如果外部速度是电机侧速度，调用前应先换算成关节侧速度：
 *             dq_joint = dq_motor / gear_ratio
 *         本函数内部仍会对速度做一次低通滤波。
 */
void JointImp_UpdateMeasurementWithVelocity(JointImpController_t *ctrl, float q_meas, float dq_meas)
{
    if (ctrl == (void *)0)
    {
        return;
    }

    ctrl->q_meas = q_meas;
    ctrl->q_prev = q_meas;
    ctrl->has_prev_measurement = true;

    ctrl->dq_lpf = joint_imp_filter_velocity(ctrl->cfg.vel_lpf_alpha, ctrl->dq_lpf, dq_meas);
    ctrl->dq_meas = ctrl->dq_lpf;
}

/**
 * @brief  设置控制参考值
 * @param  ctrl   控制器对象指针
 * @param  q_ref  参考关节角，单位 rad
 * @param  dq_ref 参考关节角速度，单位 rad/s
 * @retval 无
 * @note
 *         - 纯阻尼模式下可令 kp = 0，此时 q_ref 基本不起作用
 *         - 完整阻抗模式下，q_ref / dq_ref 决定虚拟弹簧和阻尼的目标状态
 */
void JointImp_SetReference(JointImpController_t *ctrl, float q_ref, float dq_ref)
{
    if (ctrl == (void *)0)
    {
        return;
    }

    ctrl->q_ref = q_ref;
    ctrl->dq_ref = dq_ref;
}

/**
 * @brief  设置阻抗增益
 * @param  ctrl 控制器对象指针
 * @param  kp   位置刚度增益
 * @param  kd   速度阻尼增益
 * @retval 无
 * @note
 *         - 纯阻尼模式：kp = 0, kd > 0
 *         - 位置保持/阻抗模式：kp > 0, kd > 0
 */
void JointImp_SetGains(JointImpController_t *ctrl, float kp, float kd)
{
    if (ctrl == (void *)0)
    {
        return;
    }

    ctrl->cfg.kp = kp;
    ctrl->cfg.kd = kd;
}

/**
 * @brief  设置重力补偿前馈力矩
 * @param  ctrl  控制器对象指针
 * @param  tau_g 重力补偿力矩，单位 N*m（关节侧）
 * @retval 无
 * @note
 *         一般由外部动力学模型实时计算得到。
 *         最终输出力矩中会直接叠加该项。
 */
void JointImp_SetGravityTorque(JointImpController_t *ctrl, float tau_g)
{
    if (ctrl == (void *)0)
    {
        return;
    }

    ctrl->tau_gravity = tau_g;
}

/**
 * @brief  设置额外前馈力矩
 * @param  ctrl   控制器对象指针
 * @param  tau_ff 额外前馈力矩，单位 N*m（关节侧）
 * @retval 无
 * @note
 *         可用于扩展：
 *         - 轨迹前馈
 *         - 摩擦补偿
 *         - 惯性补偿
 *         - 其他自定义前馈
 */
void JointImp_SetFeedforwardTorque(JointImpController_t *ctrl, float tau_ff)
{
    if (ctrl == (void *)0)
    {
        return;
    }

    ctrl->tau_feedforward = tau_ff;
}

/**
 * @brief  计算关节侧力矩命令
 * @param  ctrl 控制器对象指针
 * @param  dt_s 控制周期，单位 s
 * @retval 计算得到的关节侧力矩命令，单位 N*m
 * @note
 *         控制律如下：
 *             tau_cmd = kp * (q_ref - q_meas)
 *                     + kd * (dq_ref - dq_meas)
 *                     + tau_gravity
 *                     + tau_feedforward
 *
 *         之后依次执行：
 *         1) 力矩变化率限幅
 *         2) 力矩绝对值限幅
 *
 *         若控制器未使能，则返回 0。
 */
float JointImp_ComputeTorque(JointImpController_t *ctrl, float dt_s)
{
    float pos_err;
    float vel_err;
    float tau;
    float tau_prev;
    float delta_limit;

    if (ctrl == (void *)0)
    {
        return 0.0f;
    }

    if (ctrl->enabled == false)
    {
        ctrl->tau_p = 0.0f;
        ctrl->tau_d = 0.0f;
        ctrl->tau_raw = 0.0f;
        ctrl->tau_cmd = 0.0f;
        return 0.0f;
    }

    /* 位置误差 */
    pos_err = ctrl->q_ref - ctrl->q_meas;
    if (ctrl->cfg.enable_angle_wrap == true)
    {
        pos_err = joint_imp_wrap_pi(pos_err);
    }

    /* 速度误差 */
    vel_err = ctrl->dq_ref - ctrl->dq_meas;

    /* P/D 分项，便于外部调试观察 */
    ctrl->tau_p = ctrl->cfg.kp * pos_err;
    ctrl->tau_d = ctrl->cfg.kd * vel_err;

    /* 原始力矩输出 */
    tau = ctrl->tau_p + ctrl->tau_d + ctrl->tau_gravity + ctrl->tau_feedforward;
    ctrl->tau_raw = tau;

    /* 力矩变化率限幅，限制单周期内变化过快 */
    if ((ctrl->cfg.torque_rate_limit > 0.0f) && (dt_s > JOINT_IMP_EPSILON))
    {
        tau_prev = ctrl->tau_cmd;
        delta_limit = ctrl->cfg.torque_rate_limit * dt_s;
        tau = joint_imp_clampf(tau, tau_prev - delta_limit, tau_prev + delta_limit);
    }

    /* 力矩绝对值限幅 */
    if (ctrl->cfg.torque_limit > 0.0f)
    {
        tau = joint_imp_clampf(tau, -ctrl->cfg.torque_limit, ctrl->cfg.torque_limit);
    }

    ctrl->tau_cmd = tau;
    return ctrl->tau_cmd;
}

/**
 * @brief  将关节侧力矩命令换算为电机电流命令
 * @param  ctrl          控制器对象指针
 * @param  tau_joint_cmd 关节侧力矩命令，单位 N*m
 * @retval 电机电流命令，单位 A
 * @note
 *         近似换算关系：
 *             tau_joint = Kt * Iq * gear_ratio * efficiency
 *
 *         因此：
 *             Iq = tau_joint / (Kt * gear_ratio * efficiency)
 *
 *         注意：
 *         - Kt 应为电机侧力矩常数
 *         - 若驱动器内部已做额外换算，请按驱动器接口调整
 */
float JointImp_TorqueToCurrent(const JointImpController_t *ctrl, float tau_joint_cmd)
{
    float eta;
    float gr;
    float kt;
    float denom;

    if (ctrl == (void *)0)
    {
        return 0.0f;
    }

    kt  = ctrl->cfg.torque_constant;
    gr  = ctrl->cfg.gear_ratio;
    eta = ctrl->cfg.transmission_efficiency;

    if (eta <= 0.0f)
    {
        eta = 1.0f;
    }

    if (gr <= JOINT_IMP_EPSILON)
    {
        gr = 1.0f;
    }

    denom = kt * gr * eta;

    if (joint_imp_absf(denom) < JOINT_IMP_EPSILON)
    {
        return 0.0f;
    }

    return (tau_joint_cmd / denom);
}

/**
 * @brief  计算并返回电机电流命令
 * @param  ctrl 控制器对象指针
 * @param  dt_s 控制周期，单位 s
 * @retval 电机电流命令，单位 A
 * @note
 *         本函数内部流程为：
 *         1) 调用 JointImp_ComputeTorque() 计算关节侧力矩
 *         2) 调用 JointImp_TorqueToCurrent() 换算为电流命令
 *
 *         适合外部控制循环中直接调用。
 */
float JointImp_ComputeCurrentCmd(JointImpController_t *ctrl, float dt_s)
{
    float tau_cmd;

    if (ctrl == (void *)0)
    {
        return 0.0f;
    }

    tau_cmd = JointImp_ComputeTorque(ctrl, dt_s);
    ctrl->current_cmd = JointImp_TorqueToCurrent(ctrl, tau_cmd);
    return ctrl->current_cmd;
}