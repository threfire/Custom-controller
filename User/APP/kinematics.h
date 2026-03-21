#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>

#define KIN_EPS 1e-6f
#define KIN_PI  3.141592653589793f

/* 工具末端相对于关节6坐标系的偏移（沿z轴正向） */
#define TOOL_OFFSET_Z -0.08f   // 单位：米

/**
 * 正运动学：计算从基座到指定关节的齐次变换矩阵
 * @param theta  关节角度数组（6个，单位：弧度）
 * @param joint  目标关节编号（0~6，0表示基座，6表示末端）
 * @param T       输出变换矩阵（16个元素，按行主序）
 */
void forward_kinematics(const float theta[6], int joint, float T[16]);

/**
 * 逆运动学：根据末端位姿计算关节角度（多解，选择最接近当前角度的解）
 * @param T_target  目标末端位姿（16个元素，按行主序）—— 这里应为球腕中心位姿
 * @param theta_cur 当前关节角度（6个，单位：弧度）
 * @param theta_out 输出的关节角度（6个，单位：弧度）
 * @return 1 表示求解成功，0 表示无解
 */
int inverse_kinematics_puma(const float T_target[16], const float theta_cur[6], float theta_out[6]);

/**
 * 根据期望的工具末端位姿，计算目标关节角度（自动处理工具偏移）
 * @param T_tool_des  期望的工具末端位姿（16个元素，按行主序）
 * @param theta_cur   当前关节角度（6个，弧度）
 * @param theta_des   输出：目标关节角度（6个，弧度）
 * @return 1 表示成功，0 表示无解
 */
int compute_desired_joint_angles(const float T_tool_des[16], const float theta_cur[6], float theta_des[6]);

/**
 * 根据当前关节角度，计算工具末端位姿（正运动学+工具偏移）
 * @param theta       当前关节角度（6个，弧度）
 * @param T_tool      输出：工具末端位姿（16个元素，按行主序）
 */
void compute_tool_pose(const float theta[6], float T_tool[16]);
/**
 * 工具坐标系下的相对平移
 * @param T_current 当前工具末端位姿（4x4矩阵，行主序）
 * @param dx,dy,dz  在工具坐标系下的平移量（米）
 * @param T_new     输出：新工具末端位姿
 */
void tool_relative_translate(const float T_current[16], float dx, float dy, float dz, float T_new[16]);

/**
 * 工具坐标系下的相对旋转（绕工具坐标系的X、Y、Z轴依次旋转）
 * @param T_current 当前工具末端位姿
 * @param rx,ry,rz  绕工具X、Y、Z轴的旋转角（弧度，按顺序：先绕X，再绕Y，最后绕Z）
 * @param T_new     输出：新工具末端位姿
 */
void tool_relative_rotate_rpy(const float T_current[16], float rx, float ry, float rz, float T_new[16]);

/**
 * 工具坐标系下的通用相对变换（用户提供相对变换矩阵）
 * @param T_current 当前工具末端位姿
 * @param T_rel     相对变换矩阵（在工具坐标系下描述的变换）
 * @param T_new     输出：新工具末端位姿
 */
void tool_relative_transform(const float T_current[16], const float T_rel[16], float T_new[16]);

/**
 * 生成平移变换矩阵（相对于当前坐标系）
 * @param dx,dy,dz  平移量（米）
 * @param T_rel     输出：平移变换矩阵（4x4）
 */
void make_translation_matrix(float dx, float dy, float dz, float T_rel[16]);

/**
 * 生成绕X轴旋转的变换矩阵
 * @param angle  旋转角（弧度）
 * @param T_rel  输出：旋转变换矩阵
 */
void make_rotation_x(float angle, float T_rel[16]);

/**
 * 生成绕Y轴旋转的变换矩阵
 */
void make_rotation_y(float angle, float T_rel[16]);

/**
 * 生成绕Z轴旋转的变换矩阵
 */
void make_rotation_z(float angle, float T_rel[16]);

#endif
