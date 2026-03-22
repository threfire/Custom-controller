#include "kinematics.h"

/* 机械臂的改进 DH 参数（与重力补偿模块一致） */
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

/* 为解析逆解单独保留的几何参数 */
static const float d2 = 0.042f;
static const float a2 = 0.130f;
static const float d3 = 0.115f;

/* alpha 是常量，预计算三角函数 */
static const float DH_ca[6] = {
    1.0f,   /* cos(0)     */
    0.0f,   /* cos(-pi/2) */
    1.0f,   /* cos(0)     */
    0.0f,   /* cos(-pi/2) */
    0.0f,   /* cos(pi/2)  */
    0.0f    /* cos(-pi/2) */
};

static const float DH_sa[6] = {
    0.0f,   /* sin(0)     */
   -1.0f,   /* sin(-pi/2) */
    0.0f,   /* sin(0)     */
   -1.0f,   /* sin(-pi/2) */
    1.0f,   /* sin(pi/2)  */
   -1.0f    /* sin(-pi/2) */
};

/* ------------------------ 辅助函数 ------------------------ */

static float wrap_pi(float x)
{
    while (x > KIN_PI)  x -= 2.0f * KIN_PI;
    while (x < -KIN_PI) x += 2.0f * KIN_PI;
    return x;
}

/* 把 angle 映射到与 ref 最接近的 2pi 等价分支 */
static float nearest_equiv(float angle, float ref)
{
    while (angle - ref > KIN_PI)  angle -= 2.0f * KIN_PI;
    while (angle - ref < -KIN_PI) angle += 2.0f * KIN_PI;
    return angle;
}

static float clampf_ik(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 4x4矩阵乘法：C = A * B，行主序 */
static void mat_mul44(const float A[16], const float B[16], float C[16])
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i * 4 + j] =
                A[i * 4 + 0] * B[0 * 4 + j] +
                A[i * 4 + 1] * B[1 * 4 + j] +
                A[i * 4 + 2] * B[2 * 4 + j] +
                A[i * 4 + 3] * B[3 * 4 + j];
        }
    }
}

/* 仅由前三关节计算 R03，严格匹配当前 FK */
static void calc_R03_from_theta123(float t1, float t2, float t3, float R03[9])
{
    float c1  = cosf(t1);
    float s1  = sinf(t1);
    float c23 = cosf(t2 + t3);
    float s23 = sinf(t2 + t3);

    /* row-major */
    R03[0] =  c1 * c23;   R03[1] = -c1 * s23;   R03[2] = -s1;
    R03[3] =  s1 * c23;   R03[4] = -s1 * s23;   R03[5] =  c1;
    R03[6] = -s23;        R03[7] = -c23;        R03[8] =  0.0f;
}

/* Rout = Rleft^T * Rright，3x3，row-major */
static void mat3_mul_transpose_left(const float Rleft[9], const float Rright[9], float Rout[9])
{
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            Rout[r * 3 + c] =
                Rleft[0 * 3 + r] * Rright[0 * 3 + c] +
                Rleft[1 * 3 + r] * Rright[1 * 3 + c] +
                Rleft[2 * 3 + r] * Rright[2 * 3 + c];
        }
    }
}

/**
 * 根据期望的工具末端位姿，计算腕心（关节6原点）位姿
 * T_tool_des:  工具末端位姿
 * T_wrist_des: 腕心位姿
 */
static void tool_to_wrist(const float T_tool_des[16], float T_wrist_des[16])
{
    /* 先复制旋转部分和最后一行 */
    for (int i = 0; i < 16; i++) {
        T_wrist_des[i] = T_tool_des[i];
    }

    /* 工具相对腕心沿局部 z 正方向偏移 TOOL_OFFSET_Z */
    float offset_local[3] = {0.0f, 0.0f, TOOL_OFFSET_Z};
    float offset_global[3];

    offset_global[0] = T_tool_des[0] * offset_local[0] + T_tool_des[1] * offset_local[1] + T_tool_des[2]  * offset_local[2];
    offset_global[1] = T_tool_des[4] * offset_local[0] + T_tool_des[5] * offset_local[1] + T_tool_des[6]  * offset_local[2];
    offset_global[2] = T_tool_des[8] * offset_local[0] + T_tool_des[9] * offset_local[1] + T_tool_des[10] * offset_local[2];

    T_wrist_des[3]  = T_tool_des[3]  - offset_global[0];
    T_wrist_des[7]  = T_tool_des[7]  - offset_global[1];
    T_wrist_des[11] = T_tool_des[11] - offset_global[2];

    T_wrist_des[12] = 0.0f;
    T_wrist_des[13] = 0.0f;
    T_wrist_des[14] = 0.0f;
    T_wrist_des[15] = 1.0f;
}

/* ------------------------ 相对变换矩阵生成 ------------------------ */

void make_translation_matrix(float dx, float dy, float dz, float T_rel[16])
{
    for (int i = 0; i < 16; i++) {
        T_rel[i] = (i % 5 == 0) ? 1.0f : 0.0f;
    }
    T_rel[3]  = dx;
    T_rel[7]  = dy;
    T_rel[11] = dz;
}

void make_rotation_x(float angle, float T_rel[16])
{
    float c = cosf(angle);
    float s = sinf(angle);

    for (int i = 0; i < 16; i++) T_rel[i] = 0.0f;

    T_rel[0]  = 1.0f;
    T_rel[5]  = c;
    T_rel[6]  = -s;
    T_rel[9]  = s;
    T_rel[10] = c;
    T_rel[15] = 1.0f;
}

void make_rotation_y(float angle, float T_rel[16])
{
    float c = cosf(angle);
    float s = sinf(angle);

    for (int i = 0; i < 16; i++) T_rel[i] = 0.0f;

    T_rel[0]  = c;
    T_rel[2]  = s;
    T_rel[5]  = 1.0f;
    T_rel[8]  = -s;
    T_rel[10] = c;
    T_rel[15] = 1.0f;
}

void make_rotation_z(float angle, float T_rel[16])
{
    float c = cosf(angle);
    float s = sinf(angle);

    for (int i = 0; i < 16; i++) T_rel[i] = 0.0f;

    T_rel[0]  = c;
    T_rel[1]  = -s;
    T_rel[4]  = s;
    T_rel[5]  = c;
    T_rel[10] = 1.0f;
    T_rel[15] = 1.0f;
}

/* ------------------------ 正运动学 ------------------------ */

void forward_kinematics(const float theta[6], int joint, float T[16])
{
    for (int i = 0; i < 16; i++) {
        T[i] = (i % 5 == 0) ? 1.0f : 0.0f;
    }

    float T_prev[16];
    for (int i = 0; i < 16; i++) {
        T_prev[i] = T[i];
    }

    for (int i = 0; i < joint; i++) {
        float ct = cosf(theta[i]);
        float st = sinf(theta[i]);
        float ca = DH_ca[i];
        float sa = DH_sa[i];

        /* 改进 DH 变换矩阵 */
        float A_i[16] = {
            ct,      -st,      0.0f,    DH_a[i],
            st * ca,  ct * ca, -sa,    -DH_d[i] * sa,
            st * sa,  ct * sa,  ca,     DH_d[i] * ca,
            0.0f,     0.0f,     0.0f,   1.0f
        };

        float T_curr[16];
        mat_mul44(T_prev, A_i, T_curr);

        for (int j = 0; j < 16; j++) {
            T_prev[j] = T_curr[j];
        }
    }

    for (int i = 0; i < 16; i++) {
        T[i] = T_prev[i];
    }
}

/* ------------------------ 解析逆运动学 ------------------------ */

int inverse_kinematics_puma(const float T_target[16], const float theta_cur[6], float theta_out[6])
{
    const float px = T_target[3];
    const float py = T_target[7];
    const float pz = T_target[11];

    const float R06[9] = {
        T_target[0],  T_target[1],  T_target[2],
        T_target[4],  T_target[5],  T_target[6],
        T_target[8],  T_target[9],  T_target[10]
    };

    /* 最多 2(肩) * 2(肘) * 2(腕) = 8 组解 */
    float solutions[8][6];
    int sol_count = 0;

    /*
     * 对应当前 FK 的前三轴位置关系：
     *
     * x = -d2*sin(t1) + cos(t1) * (a2*cos(t2) - d3*sin(t2+t3))
     * y =  d2*cos(t1) + sin(t1) * (a2*cos(t2) - d3*sin(t2+t3))
     * z = -a2*sin(t2) - d3*cos(t2+t3)
     *
     * 令:
     * q = cos(t1)*x + sin(t1)*y = a2*cos(t2) - d3*sin(t2+t3)
     * v = -z = a2*sin(t2) + d3*cos(t2+t3)
     *
     * 且:
     * -sin(t1)*x + cos(t1)*y = d2
     *
     * 所以 theta1 不能直接 atan2(py, px)，必须把 d2 的侧偏算进去。
     */

    float r_xy = sqrtf(px * px + py * py);
    if (r_xy < fabsf(d2) - 1e-6f) {
        return 0;
    }

    float q_abs_sq = r_xy * r_xy - d2 * d2;
    if (q_abs_sq < 0.0f && q_abs_sq > -1e-8f) q_abs_sq = 0.0f;
    if (q_abs_sq < 0.0f) return 0;

    float q_abs = sqrtf(q_abs_sq);
    float phi = atan2f(py, px);

    float q_candidates[2] = { q_abs, -q_abs };

    for (int ishoulder = 0; ishoulder < 2; ishoulder++) {
        float q_nom = q_candidates[ishoulder];

        /*
         * 由：
         * q  =  cos(t1)*px + sin(t1)*py
         * d2 = -sin(t1)*px + cos(t1)*py
         * 得：
         * theta1 = atan2(py, px) - atan2(d2, q)
         */
        float theta1 = wrap_pi(phi - atan2f(d2, q_nom));

        float c1 = cosf(theta1);
        float s1 = sinf(theta1);

        /* 用求出的 theta1 反算 q，更稳 */
        float q = c1 * px + s1 * py;
        float v = -pz;

        /*
         * q = a2*cos(t2) - d3*sin(t2+t3)
         * v = a2*sin(t2) + d3*cos(t2+t3)
         *
         * 推得：
         * q^2 + v^2 = a2^2 + d3^2 - 2*a2*d3*sin(t3)
         *
         * ==> sin(t3) = (a2^2 + d3^2 - q^2 - v^2) / (2*a2*d3)
         */
        float s3 = (a2 * a2 + d3 * d3 - q * q - v * v) / (2.0f * a2 * d3);

        if (s3 < -1.0f - 1e-5f || s3 > 1.0f + 1e-5f) {
            continue;
        }
        s3 = clampf_ik(s3, -1.0f, 1.0f);

        float c3_abs_sq = 1.0f - s3 * s3;
        if (c3_abs_sq < 0.0f && c3_abs_sq > -1e-8f) c3_abs_sq = 0.0f;
        if (c3_abs_sq < 0.0f) continue;

        float c3_abs = sqrtf(c3_abs_sq);
        float c3_candidates[2] = { c3_abs, -c3_abs };

        for (int ielbow = 0; ielbow < 2; ielbow++) {
            float c3 = c3_candidates[ielbow];
            float theta3 = wrap_pi(atan2f(s3, c3));

            /*
             * [q, v]^T = Rot(t2) * [A, B]^T
             * A = a2 - d3*sin(t3)
             * B = d3*cos(t3)
             *
             * ==> theta2 = atan2(v, q) - atan2(B, A)
             */
            float A = a2 - d3 * s3;
            float B = d3 * c3;
            float theta2 = wrap_pi(atan2f(v, q) - atan2f(B, A));

            /* 算 R36 = R03^T * R06 */
            float R03[9];
            float R36[9];
            calc_R03_from_theta123(theta1, theta2, theta3, R03);
            mat3_mul_transpose_left(R03, R06, R36);

            /*
             * 当前 DH 下腕部旋转矩阵可写为：
             *
             * [ -s4*s6 + c4*c5*c6,   -s4*c6 - c4*c5*s6,   -c4*s5 ]
             * [        s5*c6,               -s5*s6,          c5    ]
             * [ -s4*c5*c6 - c4*s6,    s4*c5*s6 - c4*c6,    s4*s5 ]
             *
             * 于是：
             * theta4 = atan2(r33, -r13)
             * theta5 = atan2(±sqrt(r13^2 + r33^2), r23)
             * theta6 = atan2(-r22, r21)
             */

            float r11 = R36[0];
            float r12 = R36[1];
            float r13 = R36[2];
            float r21 = R36[3];
            float r22 = R36[4];
            float r23 = R36[5];
            float r31 = R36[6];
            float r32 = R36[7];
            float r33 = R36[8];

            (void)r11;
            (void)r31;
            (void)r32;

            float s5_abs = sqrtf(r13 * r13 + r33 * r33);

            if (s5_abs > 1e-6f) {
                float theta4_a = wrap_pi(atan2f(r33, -r13));
                float theta5_a = wrap_pi(atan2f(s5_abs, r23));
                float theta6_a = wrap_pi(atan2f(-r22, r21));

                if (sol_count < 8) {
                    solutions[sol_count][0] = theta1;
                    solutions[sol_count][1] = theta2;
                    solutions[sol_count][2] = theta3;
                    solutions[sol_count][3] = theta4_a;
                    solutions[sol_count][4] = theta5_a;
                    solutions[sol_count][5] = theta6_a;
                    sol_count++;
                }

                if (sol_count < 8) {
                    solutions[sol_count][0] = theta1;
                    solutions[sol_count][1] = theta2;
                    solutions[sol_count][2] = theta3;
                    solutions[sol_count][3] = wrap_pi(theta4_a + KIN_PI);
                    solutions[sol_count][4] = wrap_pi(-theta5_a);
                    solutions[sol_count][5] = wrap_pi(theta6_a + KIN_PI);
                    sol_count++;
                }
            } else {
                /* 腕部奇异：s5 ≈ 0 */
                float theta4 = 0.0f;
                float theta5;
                float theta6;

                if (r23 >= 0.0f) {
                    /* theta5 ≈ 0，此时只剩 theta4 + theta6 */
                    theta5 = 0.0f;
                    theta6 = wrap_pi(atan2f(-r12, r11));
                } else {
                    /* theta5 ≈ pi，此时只剩 theta4 - theta6 */
                    theta5 = KIN_PI;
                    theta6 = wrap_pi(atan2f(r12, -r11));
                }

                if (sol_count < 8) {
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
    }

    if (sol_count == 0) {
        return 0;
    }

    /* 选与当前关节角最接近的那组：按当前所在 2pi 分支比较 */
    int best_idx = 0;
    float min_cost = 1e30f;

    for (int i = 0; i < sol_count; i++) {
        float cost = 0.0f;
        for (int j = 0; j < 6; j++) {
            float cand = nearest_equiv(solutions[i][j], theta_cur[j]);
            float d = cand - theta_cur[j];
            cost += fabsf(d);
        }
        if (cost < min_cost) {
            min_cost = cost;
            best_idx = i;
        }
    }

    /* 输出时也贴到当前分支，不强行压回 [-pi, pi] */
    for (int j = 0; j < 6; j++) {
        theta_out[j] = nearest_equiv(solutions[best_idx][j], theta_cur[j]);
    }

    return 1;
}

/* ------------------------ 封装接口 ------------------------ */

int compute_desired_joint_angles(const float T_tool_des[16], const float theta_cur[6], float theta_des[6])
{
    float T_wrist_des[16];
    tool_to_wrist(T_tool_des, T_wrist_des);
    return inverse_kinematics_puma(T_wrist_des, theta_cur, theta_des);
}

void compute_tool_pose(const float theta[6], float T_tool[16])
{
    float T_wrist[16];
    forward_kinematics(theta, 6, T_wrist);

    float offset_local[3] = {0.0f, 0.0f, TOOL_OFFSET_Z};
    float offset_global[3];

    offset_global[0] = T_wrist[0] * offset_local[0] + T_wrist[1] * offset_local[1] + T_wrist[2]  * offset_local[2];
    offset_global[1] = T_wrist[4] * offset_local[0] + T_wrist[5] * offset_local[1] + T_wrist[6]  * offset_local[2];
    offset_global[2] = T_wrist[8] * offset_local[0] + T_wrist[9] * offset_local[1] + T_wrist[10] * offset_local[2];

    for (int i = 0; i < 16; i++) {
        T_tool[i] = T_wrist[i];
    }

    T_tool[3]  = T_wrist[3]  + offset_global[0];
    T_tool[7]  = T_wrist[7]  + offset_global[1];
    T_tool[11] = T_wrist[11] + offset_global[2];

    T_tool[12] = 0.0f;
    T_tool[13] = 0.0f;
    T_tool[14] = 0.0f;
    T_tool[15] = 1.0f;
}

/* ------------------------ 工具坐标系相对变换 ------------------------ */

void tool_relative_transform(const float T_current[16], const float T_rel[16], float T_new[16])
{
    mat_mul44(T_current, T_rel, T_new);
}

void tool_relative_translate(const float T_current[16], float dx, float dy, float dz, float T_new[16])
{
    float T_rel[16];
    make_translation_matrix(dx, dy, dz, T_rel);
    tool_relative_transform(T_current, T_rel, T_new);
}

void tool_relative_rotate_rpy(const float T_current[16], float rx, float ry, float rz, float T_new[16])
{
    float Tx[16], Ty[16], Tz[16], T_tmp[16], T_rel[16];

    make_rotation_x(rx, Tx);
    make_rotation_y(ry, Ty);
    make_rotation_z(rz, Tz);

    /* T_rel = Tx * Ty * Tz */
    mat_mul44(Tx, Ty, T_tmp);
    mat_mul44(T_tmp, Tz, T_rel);

    tool_relative_transform(T_current, T_rel, T_new);
}