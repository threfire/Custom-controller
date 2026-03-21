#include "kinematics.h"

// 机械臂的改进DH参数（与重力补偿模块一致）
static const float DH_a[6] = {
    0.0f,
	0.0f,
	0.130f,
	0.0f,
	0.0f,
	0.0f
};
static const float DH_alpha[6] = {
    0.0f,
	-90.0f * KIN_PI / 180.0f,
	0.0f,
    -90.0f * KIN_PI / 180.0f,
	90.0f * KIN_PI / 180.0f,
    -90.0f * KIN_PI / 180.0f
};
static const float DH_d[6] = {
    0.0f,
	0.042f,
	0.0f,
	0.115f,
	0.0f,
	0.0f
};

// 关节2的垂直偏移（与DH_d[1]相同，用于几何公式）
static const float d2 = 0.042f;
static const float a2 = 0.130f;
static const float d3 = 0.115f;

/* ------------------------ 辅助矩阵生成函数 ------------------------ */
// 辅助函数：矩阵乘法（4x4）
static void mat_mul44(const float A[16], const float B[16], float C[16]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i*4 + j] = A[i*4+0]*B[0*4+j] + A[i*4+1]*B[1*4+j] +
                         A[i*4+2]*B[2*4+j] + A[i*4+3]*B[3*4+j];
        }
    }
}
/* 生成平移矩阵 */
void make_translation_matrix(float dx, float dy, float dz, float T_rel[16]) {
    // 初始化为单位矩阵
    for (int i = 0; i < 16; i++) T_rel[i] = (i % 5 == 0) ? 1.0f : 0.0f;
    T_rel[3]  = dx;
    T_rel[7]  = dy;
    T_rel[11] = dz;
}
/* 生成绕X轴旋转矩阵 */
void make_rotation_x(float angle, float T_rel[16]) {
    float c = cosf(angle), s = sinf(angle);
    for (int i = 0; i < 16; i++) T_rel[i] = 0.0f;
    T_rel[0] = 1; T_rel[5] = c; T_rel[6] = -s;
    T_rel[9] = s; T_rel[10] = c; T_rel[15] = 1;
}
/* 生成绕Y轴旋转矩阵 */
void make_rotation_y(float angle, float T_rel[16]) {
    float c = cosf(angle), s = sinf(angle);
    for (int i = 0; i < 16; i++) T_rel[i] = 0.0f;
    T_rel[0] = c; T_rel[2] = s; T_rel[5] = 1;
    T_rel[8] = -s; T_rel[10] = c; T_rel[15] = 1;
}
/* 生成绕Z轴旋转矩阵 */
void make_rotation_z(float angle, float T_rel[16]) {
    float c = cosf(angle), s = sinf(angle);
    for (int i = 0; i < 16; i++) T_rel[i] = 0.0f;
    T_rel[0] = c; T_rel[1] = -s; T_rel[4] = s; T_rel[5] = c;
    T_rel[10] = 1; T_rel[15] = 1;
}


// 正运动学：计算从基座到关节joint的齐次变换矩阵
void forward_kinematics(const float theta[6], int joint, float T[16]) {
    // 初始化基座坐标系为单位阵
    for (int i = 0; i < 16; i++) T[i] = (i % 5 == 0) ? 1.0f : 0.0f;

    float T_prev[16];
    for (int i = 0; i < 16; i++) T_prev[i] = T[i];

    for (int i = 0; i < joint; i++) {
        float ct = cosf(theta[i]);
        float st = sinf(theta[i]);
        float ca = cosf(DH_alpha[i]);
        float sa = sinf(DH_alpha[i]);

        // 改进D-H变换矩阵 A_i
        float A_i[16] = {
            ct,      -st,      0,      DH_a[i],
            st*ca,   ct*ca,   -sa,   -DH_d[i]*sa,
            st*sa,   ct*sa,   ca,     DH_d[i]*ca,
            0,       0,       0,      1
        };

        float T_curr[16];
        mat_mul44(T_prev, A_i, T_curr);
        for (int j = 0; j < 16; j++) T_prev[j] = T_curr[j];
    }

    // 复制结果
    for (int i = 0; i < 16; i++) T[i] = T_prev[i];
}

// 逆运动学主函数
int inverse_kinematics_puma(const float T_target[16], const float theta_cur[6], float theta_out[6]) {
    // 提取末端位置和旋转矩阵
    float px = T_target[3];
    float py = T_target[7];
    float pz = T_target[11];

    float R06[9] = {
        T_target[0], T_target[1], T_target[2],
        T_target[4], T_target[5], T_target[6],
        T_target[8], T_target[9], T_target[10]
    };

    // 腕心位置（d6=0）
    float Pw[3] = {px, py, pz};

    // 用于存储多解
    float solutions[8][6];  // 最多8组解
    int sol_count = 0;

    // 计算关节1的两个可能值
    float theta1_vals[2];
    theta1_vals[0] = atan2f(py, px);
    theta1_vals[1] = theta1_vals[0] + KIN_PI;

    for (int i1 = 0; i1 < 2; i1++) {
        float theta1 = theta1_vals[i1];
        float c1 = cosf(theta1);
        float s1 = sinf(theta1);
        // 计算R = px/c1 或 py/s1，避免除以零
        float R;
        if (fabsf(c1) > KIN_EPS) R = px / c1;
        else if (fabsf(s1) > KIN_EPS) R = py / s1;
        else continue;  // 奇异

        float s = pz - d2;  // z坐标偏移

        // 计算D = (R^2 + s^2 - a2^2 - d3^2) / (2*a2*d3)
        float D = (R*R + s*s - a2*a2 - d3*d3) / (2.0f * a2 * d3);
        if (fabsf(D) > 1.0f + KIN_EPS) continue;  // 无解
        if (D > 1.0f) D = 1.0f;
        if (D < -1.0f) D = -1.0f;

        // 关节3的两个可能值
        float theta3_vals[2];
        float s3_sqrt = sqrtf(1.0f - D*D);
        theta3_vals[0] = atan2f( s3_sqrt, D);   // 肘部向上
        theta3_vals[1] = atan2f(-s3_sqrt, D);   // 肘部向下

        for (int i3 = 0; i3 < 2; i3++) {
            float theta3 = theta3_vals[i3];
            float c3 = cosf(theta3);
            float s3 = sinf(theta3);

            // 计算关节2
            float numerator   = (d3*c3 + a2)*s - d3*s3*R;
            float denominator = (d3*c3 + a2)*R + d3*s3*s;
            float theta2 = atan2f(numerator, denominator);

            // 前三关节角度
            float theta123[3] = {theta1, theta2, theta3};

            // 计算前三关节的变换矩阵，得到R03
            float T03[16];
            forward_kinematics(theta123, 3, T03);
            float R03[9] = {
                T03[0], T03[1], T03[2],
                T03[4], T03[5], T03[6],
                T03[8], T03[9], T03[10]
            };

            // 计算R36 = R03^T * R06
            float R36[9];
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    R36[r*3 + c] = R03[0*3 + r]*R06[0*3 + c] +
                                   R03[1*3 + r]*R06[1*3 + c] +
                                   R03[2*3 + r]*R06[2*3 + c];
                }
            }

            // 解腕关节
            float s5 = sqrtf(R36[0*3+2]*R36[0*3+2] + R36[1*3+2]*R36[1*3+2]);
            float theta4, theta5, theta6;

            if (s5 > KIN_EPS) {
                theta5 = atan2f(s5, R36[2*3+2]);
                theta4 = atan2f(R36[1*3+2] / s5, R36[0*3+2] / s5);
                theta6 = atan2f(-R36[2*3+1] / s5, R36[2*3+0] / s5);
            } else {
                // 奇异情况（θ5 = 0 或 π）
                theta5 = (R36[2*3+2] > 0) ? 0.0f : KIN_PI;
                theta4 = 0.0f;  // 可以任意设定
                theta6 = atan2f(R36[1*3+0], R36[0*3+0]);
            }

            // 存储解（实际关节角度，无偏置）
            solutions[sol_count][0] = theta1;
            solutions[sol_count][1] = theta2;
            solutions[sol_count][2] = theta3;
            solutions[sol_count][3] = theta4;
            solutions[sol_count][4] = theta5;
            solutions[sol_count][5] = theta6;
            sol_count++;

            // 第二组腕关节解（θ5取π-θ5，θ4和θ6相应改变）
            if (s5 > KIN_EPS) {
                theta5 = atan2f(s5, -R36[2*3+2]);
                theta4 = atan2f(R36[1*3+2] / s5, -R36[0*3+2] / s5);
                theta6 = atan2f(R36[2*3+1] / s5, -R36[2*3+0] / s5);
                solutions[sol_count][0] = theta1;
                solutions[sol_count][1] = theta2;
                solutions[sol_count][2] = theta3;
                solutions[sol_count][3] = theta4;
                solutions[sol_count][4] = theta5;
                solutions[sol_count][5] = theta6;
                sol_count++;
            }
        }
    }

    if (sol_count == 0) return 0;  // 无解

    // 从所有解中选择最接近当前关节角度的解
    int best_idx = 0;
    float min_diff = 1e9f;
    for (int i = 0; i < sol_count; i++) {
        float diff = 0.0f;
        for (int j = 0; j < 6; j++) {
            float d = fabsf(solutions[i][j] - theta_cur[j]);
            // 考虑角度周期性，将差限制在 [-π, π]
            d = fminf(d, 2.0f*KIN_PI - d);
            diff += d;
        }
        if (diff < min_diff) {
            min_diff = diff;
            best_idx = i;
        }
    }

    // 输出最佳解
    for (int j = 0; j < 6; j++) {
        theta_out[j] = solutions[best_idx][j];
        // 将角度归一化到 [-π, π]
        while (theta_out[j] > KIN_PI) theta_out[j] -= 2.0f*KIN_PI;
        while (theta_out[j] < -KIN_PI) theta_out[j] += 2.0f*KIN_PI;
    }

    return 1;
}
/**
 * 根据期望的工具末端位姿，计算球腕中心（关节6原点）的位姿
 * @param T_tool_des  期望的工具末端位姿（16个元素，按行主序）
 * @param T_wrist_des 输出：球腕中心期望位姿（16个元素）
 */
void tool_to_wrist(const float T_tool_des[16], float T_wrist_des[16]) {
    // 复制旋转部分（前3x3）
    for (int i = 0; i < 12; i++) T_wrist_des[i] = T_tool_des[i];
    
    // 计算球腕中心位置 = 工具位置 - 工具在基座下的偏移向量
    // 工具偏移向量在基座下的表示 = R_tool * [0, 0, TOOL_OFFSET_Z]^T
    float offset_local[3] = {0, 0, TOOL_OFFSET_Z};
    float offset_global[3];
    offset_global[0] = T_tool_des[0]*offset_local[0] + T_tool_des[1]*offset_local[1] + T_tool_des[2]*offset_local[2];
    offset_global[1] = T_tool_des[4]*offset_local[0] + T_tool_des[5]*offset_local[1] + T_tool_des[6]*offset_local[2];
    offset_global[2] = T_tool_des[8]*offset_local[0] + T_tool_des[9]*offset_local[1] + T_tool_des[10]*offset_local[2];
    
    T_wrist_des[3]  = T_tool_des[3]  - offset_global[0];
    T_wrist_des[7]  = T_tool_des[7]  - offset_global[1];
    T_wrist_des[11] = T_tool_des[11] - offset_global[2];
    
    // 最后一行保持 [0 0 0 1]
    T_wrist_des[12] = 0; T_wrist_des[13] = 0; T_wrist_des[14] = 0; T_wrist_des[15] = 1;
}
/* ------------------------ 新增封装函数 ------------------------ */
int compute_desired_joint_angles(const float T_tool_des[16], const float theta_cur[6], float theta_des[6]) {
    float T_wrist_des[16];
    tool_to_wrist(T_tool_des, T_wrist_des);
    return inverse_kinematics_puma(T_wrist_des, theta_cur, theta_des);
}

void compute_tool_pose(const float theta[6], float T_tool[16]) {
    float T_wrist[16];
    forward_kinematics(theta, 6, T_wrist);   // 球腕中心位姿

    // 工具偏移在基座下的表示
    float offset_local[3] = {0, 0, TOOL_OFFSET_Z};
    float offset_global[3];
    offset_global[0] = T_wrist[0]*offset_local[0] + T_wrist[1]*offset_local[1] + T_wrist[2]*offset_local[2];
    offset_global[1] = T_wrist[4]*offset_local[0] + T_wrist[5]*offset_local[1] + T_wrist[6]*offset_local[2];
    offset_global[2] = T_wrist[8]*offset_local[0] + T_wrist[9]*offset_local[1] + T_wrist[10]*offset_local[2];

    // 复制旋转部分
    for (int i = 0; i < 12; i++) T_tool[i] = T_wrist[i];
    // 位置 = 球腕中心位置 + 偏移
    T_tool[3]  = T_wrist[3]  + offset_global[0];
    T_tool[7]  = T_wrist[7]  + offset_global[1];
    T_tool[11] = T_wrist[11] + offset_global[2];
    T_tool[12] = 0; T_tool[13] = 0; T_tool[14] = 0; T_tool[15] = 1;
}

/* ------------------------ 工具坐标系变换函数 ------------------------ */
void tool_relative_transform(const float T_current[16], const float T_rel[16], float T_new[16]) {
    mat_mul44(T_current, T_rel, T_new);
}

void tool_relative_translate(const float T_current[16], float dx, float dy, float dz, float T_new[16]) {
    float T_rel[16];
    make_translation_matrix(dx, dy, dz, T_rel);
    tool_relative_transform(T_current, T_rel, T_new);
}

void tool_relative_rotate_rpy(const float T_current[16], float rx, float ry, float rz, float T_new[16]) {
    // 先绕X，再绕Y，最后绕Z（ZYX顺序，RPY）
    float Tx[16], Ty[16], Tz[16], T_tmp[16], T_rel[16];
    make_rotation_x(rx, Tx);
    make_rotation_y(ry, Ty);
    make_rotation_z(rz, Tz);
    // T_rel = Tx * Ty * Tz
    mat_mul44(Tx, Ty, T_tmp);
    mat_mul44(T_tmp, Tz, T_rel);
    tool_relative_transform(T_current, T_rel, T_new);
}
