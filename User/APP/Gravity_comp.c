/*-------------------------------------------------------------------------
 * 重力补偿计算模块 (Gravity_comp.c)
 * 说明：根据关节角度计算机械臂各关节的重力补偿力矩
 * 依赖：标准C库math.h，提供三角函数等
 * 使用前必须根据实际机器人修改：连杆质量、质心位置
 * 注意：本代码采用改进DH约定，关节i的轴线为坐标系{i}的Z轴，
 *       轴线上一点取坐标系{i}的原点。
 *-------------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>   // 仅用于示例，实际使用时可根据需要移除
#include "Gravity_comp.h"

/* 重力加速度 (m/s^2) */
#define GRAVITY 9.81f
#define M_PI 3.141592653589793f

/* 机械臂连杆数 */
#define N_JOINTS 6

/* 改进D-H参数（根据您提供的表格，单位：米，角度已转换为弧度） */
static const float DH_a[N_JOINTS] = {
    0.0f,    /* a0 */
    0.0f,    /* a1 */
    0.130f,  /* a2 */
    0.039f,  /* a3 */
    0.0f,    /* a4 */
    0.0f     /* a5 */
};

static const float DH_alpha[N_JOINTS] = {
    0.0f,                             /* alpha0 */
    -90.0f * M_PI / 180.0f,           /* alpha1 */
    0.0f,                             /* alpha2 */
    -90.0f * M_PI / 180.0f,           /* alpha3 */
    90.0f * M_PI / 180.0f,            /* alpha4 */
    -90.0f * M_PI / 180.0f            /* alpha5 */
};

static const float DH_d[N_JOINTS] = {
    0.0f,    /* d1 */
    0.042f,  /* d2 */
    0.0f,    /* d3 */
    0.115f,  /* d4 */
    0.0f,    /* d5 */
    0.0f     /* d6 */
};

/* ====================== 需要用户根据实际机器人修改的数据 ====================== */
/* 各连杆质量 (kg) - 请替换为实际值 */
static const float link_mass[N_JOINTS] = {
    0.0f,    /* 连杆1质量 */
    0.290f,  /* 连杆2质量 */
    0.140f,  /* 连杆3质量 */
    0.150f,  /* 连杆4质量 */
    0.050f,  /* 连杆5质量 */
    0.095f   /* 连杆6质量 */
};

/* 各连杆质心在自身连杆坐标系中的位置 (x, y, z) 单位：米 - 请替换为实际值 */
static const float link_com[N_JOINTS][3] = {
    {0.0f, 0.0f, 0.1f},      /* 连杆1质心 (坐标系{1}) */
    {0.13f, 0.0f, -0.028f},  /* 连杆2质心 (坐标系{2}) */
    {0.0f, 0.0f, 0.039f},    /* 连杆3质心 (坐标系{3}) */
    {0.0f, 0.0f, -0.046f},   /* 连杆4质心 (坐标系{4}) */
    {0.0f, 0.0f, 0.00f},     /* 连杆5质心 (坐标系{5}) */
    {0.0f, 0.0f, 0.035f}     /* 连杆6质心 (坐标系{6}) */
};
/* ============================================================================ */

/*------------------------ 向量与矩阵运算辅助函数 ------------------------*/
static void vec_add(const float a[3], const float b[3], float c[3]) {
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
}

static void vec_sub(const float a[3], const float b[3], float c[3]) {
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
}

static void vec_cross(const float a[3], const float b[3], float c[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

static float vec_dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void mat_mul(const float A[16], const float B[16], float C[16]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i * 4 + j] = A[i * 4 + 0] * B[0 * 4 + j] +
                           A[i * 4 + 1] * B[1 * 4 + j] +
                           A[i * 4 + 2] * B[2 * 4 + j] +
                           A[i * 4 + 3] * B[3 * 4 + j];
        }
    }
}

static void transform_point(const float T[16], const float p_in[3], float p_out[3]) {
    p_out[0] = T[0] * p_in[0] + T[1] * p_in[1] + T[2] * p_in[2] + T[3];
    p_out[1] = T[4] * p_in[0] + T[5] * p_in[1] + T[6] * p_in[2] + T[7];
    p_out[2] = T[8] * p_in[0] + T[9] * p_in[1] + T[10] * p_in[2] + T[11];
}

/*------------------------ 运动学计算 ------------------------*/
static void compute_transforms(const float theta[N_JOINTS], float T0_i[7][16]) {
    // 初始化基座坐标系为单位阵
    for (int i = 0; i < 16; i++) T0_i[0][i] = (i % 5 == 0) ? 1.0f : 0.0f;

    float T_prev[16];
    for (int i = 0; i < 16; i++) T_prev[i] = T0_i[0][i];

    for (int i = 0; i < N_JOINTS; i++) {
        float ct = cosf(theta[i]);
        float st = sinf(theta[i]);
        float ca = cosf(DH_alpha[i]);
        float sa = sinf(DH_alpha[i]);

        // 改进D-H变换矩阵 A_i (从{i}到{i-1})
        float A_i[16] = {
            ct,      -st,      0,      DH_a[i],
            st * ca,  ct * ca, -sa,   -DH_d[i] * sa,
            st * sa,  ct * sa,  ca,    DH_d[i] * ca,
            0,        0,        0,      1
        };

        float T_curr[16];
        mat_mul(T_prev, A_i, T_curr);   // T_curr = T_prev * A_i
        for (int j = 0; j < 16; j++) T0_i[i + 1][j] = T_curr[j];
        for (int j = 0; j < 16; j++) T_prev[j] = T_curr[j];
    }
}

/*------------------------ 重力补偿计算 ------------------------*/
void gravity_compensation(const float theta[N_JOINTS], float tau[N_JOINTS]) {
    // 1. 计算所有连杆变换矩阵 T0_i[i] (i=0..6)
    float T0_i[7][16];
    compute_transforms(theta, T0_i);

    // 2. 提取各坐标系原点位置 p_i 和 Z轴方向 z_i (基坐标系下)
    float p[7][3];
    float z[7][3];
    for (int i = 0; i <= N_JOINTS; i++) {
        p[i][0] = T0_i[i][3];
        p[i][1] = T0_i[i][7];
        p[i][2] = T0_i[i][11];

        z[i][0] = T0_i[i][2];
        z[i][1] = T0_i[i][6];
        z[i][2] = T0_i[i][10];
    }

    // 3. 计算各连杆质心在基坐标系下的位置 pc[i] (i=1..6)
    float pc[7][3];   // pc[0] 未使用
    for (int i = 1; i <= N_JOINTS; i++) {
        transform_point(T0_i[i], link_com[i - 1], pc[i]);
    }

    // 4. 初始化力矩为零
    for (int j = 0; j < N_JOINTS; j++) tau[j] = 0.0f;

    // 5. 对每个关节 j，累加所有连杆 i (i >= j) 的重力贡献
    //    注意：在改进DH中，关节j的轴线为坐标系{j}的Z轴，且通过坐标系{j}的原点
    for (int j = 1; j <= N_JOINTS; j++) {   // 关节编号 j 从1到6
        float *p_joint = p[j];   // 关节轴线上一点 (坐标系{j}原点)
        float *z_joint = z[j];   // 关节轴线方向

        for (int i = j; i <= N_JOINTS; i++) {
            float f[3] = {0.0f, 0.0f, -link_mass[i - 1] * GRAVITY}; // 重力矢量
            float r[3];
            vec_sub(pc[i], p_joint, r);    // 力臂向量
            float m[3];
            vec_cross(r, f, m);             // 力矩向量
            tau[j - 1] += vec_dot(m, z_joint); // 投影到关节轴线
        }
    }
}
