/*-------------------------------------------------------------------------
 * 重力补偿计算模块 (gravity_compensation.c)
 * 说明：根据关节角度计算机械臂各关节的重力补偿力矩
 * 依赖：标准C库math.h，提供三角函数等
 * 使用前需根据实际机器人修改：DH参数、连杆质量、质心位置
 *-------------------------------------------------------------------------*/

#include <math.h>
#include "gravity_compensation.h"

/* 重力加速度 (m/s^2) */
#define GRAVITY 9.81f
#define M_PI 3.14f
/* 机械臂连杆数 */
#define N_JOINTS 6

/* 改进D-H参数 (根据PUMA560构型，单位：米，角度需转换为弧度) */
/* a[i-1] : 连杆长度 (i从1到6) */
static const float DH_a[N_JOINTS] = {
    0.0f,    /* a0 */
    0.0f,    /* a1 */
    1.0f,    /* a2 (大臂长度) */
    0.2f,    /* a3 (小臂长度) */
    0.0f,    /* a4 */
    0.0f     /* a5 */
};

/* alpha[i-1] : 连杆扭角 (弧度) */
static const float DH_alpha[N_JOINTS] = {
    0.0f,                /* alpha0 */
    -90.0f * M_PI/180.0f, /* alpha1 */
    0.0f,                /* alpha2 */
    -90.0f * M_PI/180.0f, /* alpha3 */
    90.0f * M_PI/180.0f,  /* alpha4 */
    -90.0f * M_PI/180.0f  /* alpha5 */
};

/* d[i] : 连杆偏距 (沿Zi轴) */
static const float DH_d[N_JOINTS] = {
    0.0f,    /* d1 */
    0.0f,    /* d2 */
    0.5f,    /* d3 (偏距) */
    0.8f,    /* d4 (手腕点偏移) */
    0.0f,    /* d5 */
    0.0f     /* d6 */
};

/* 各连杆质量 (kg) - 示例值，需用户根据实际修改 */
static const float link_mass[N_JOINTS] = {
    0.0f,   /* 连杆1质量 */
    0.290f,   /* 连杆2质量 */
    0.140f,   /* 连杆3质量 */
    0.150f,   /* 连杆4质量 (含手腕) */
    0.050f,   /* 连杆5质量 */
    0.095f    /* 连杆6质量 (含末端) */
};

/* 各连杆质心在自身连杆坐标系中的位置 (x, y, z) 单位：米 */
static const float link_com[N_JOINTS][3] = {
    {0.0f, 0.0f, 0.1f},   /* 连杆1质心 */
    {0.5f, 0.0f, 0.0f},   /* 连杆2质心 (沿X2方向偏移) */
    {0.1f, 0.0f, 0.2f},   /* 连杆3质心 */
    {0.0f, 0.0f, 0.1f},   /* 连杆4质心 (靠近手腕) */
    {0.0f, 0.0f, 0.05f},  /* 连杆5质心 */
    {0.0f, 0.0f, 0.03f}   /* 连杆6质心 */
};

/*------------------------ 向量与矩阵运算辅助函数 ------------------------*/

/* 3x1向量加法: c = a + b */
static void vec_add(const float a[3], const float b[3], float c[3]) {
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
}

/* 3x1向量减法: c = a - b */
static void vec_sub(const float a[3], const float b[3], float c[3]) {
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
}

/* 3x1向量叉积: c = a × b */
static void vec_cross(const float a[3], const float b[3], float c[3]) {
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

/* 点积: 返回 a·b */
static float vec_dot(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/* 4x4齐次矩阵乘法: C = A * B (矩阵按行优先存储) */
static void mat_mul(const float A[16], const float B[16], float C[16]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i*4+j] = A[i*4+0]*B[0*4+j] + A[i*4+1]*B[1*4+j] +
                       A[i*4+2]*B[2*4+j] + A[i*4+3]*B[3*4+j];
        }
    }
}

/* 用齐次矩阵变换点: p_out = T * p_in (p_in为齐次坐标[x,y,z,1]^T) */
static void transform_point(const float T[16], const float p_in[3], float p_out[3]) {
    p_out[0] = T[0]*p_in[0] + T[1]*p_in[1] + T[2]*p_in[2] + T[3];
    p_out[1] = T[4]*p_in[0] + T[5]*p_in[1] + T[6]*p_in[2] + T[7];
    p_out[2] = T[8]*p_in[0] + T[9]*p_in[1] + T[10]*p_in[2] + T[11];
}

/*------------------------ 运动学计算 ------------------------*/

/**
 * 计算所有连杆坐标系相对于基座{0}的变换矩阵
 * @param theta[6]  关节角度 (弧度)
 * @param T0_i[7][16] 输出数组，T0_i[0]为单位阵(基座)，T0_i[i]为连杆i坐标系(i=1..6)
 */
static void compute_transforms(const float theta[N_JOINTS], float T0_i[7][16]) {
    /* 初始化基座坐标系为单位阵 */
    for (int i = 0; i < 16; i++) T0_i[0][i] = (i % 5 == 0) ? 1.0f : 0.0f;

    float T_prev[16];  /* 当前累积变换 */
    for (int i = 0; i < 16; i++) T_prev[i] = T0_i[0][i];

    for (int i = 0; i < N_JOINTS; i++) {
        float ct = cosf(theta[i]);
        float st = sinf(theta[i]);
        float ca = cosf(DH_alpha[i]);
        float sa = sinf(DH_alpha[i]);

        /* 改进D-H变换矩阵 A_i (从{i-1}到{i}) */
        float A_i[16] = {
            ct,      -st,      0,      DH_a[i],
            st*ca,   ct*ca,    -sa,   -DH_d[i]*sa,
            st*sa,   ct*sa,    ca,     DH_d[i]*ca,
            0,       0,        0,      1
        };

        float T_curr[16];
        mat_mul(T_prev, A_i, T_curr);   /* T_curr = T_prev * A_i */
        /* 保存结果到 T0_i[i+1] */
        for (int j = 0; j < 16; j++) T0_i[i+1][j] = T_curr[j];

        /* 更新T_prev为当前变换 */
        for (int j = 0; j < 16; j++) T_prev[j] = T_curr[j];
    }
}

/*------------------------ 重力补偿计算 ------------------------*/

/**
 * 计算关节重力补偿力矩
 * @param theta[6]  当前关节角度 (弧度)
 * @param tau[6]    输出补偿力矩 (Nm)，需在调用前分配空间
 */
void gravity_compensation(const float theta[N_JOINTS], float tau[N_JOINTS]) {
    /* 1. 计算所有连杆变换矩阵 T0_i[i] (i=0..6) */
    float T0_i[7][16];
    compute_transforms(theta, T0_i);

    /* 2. 提取各坐标系原点位置 p_i (基坐标系下) 和 Z轴方向 z_i */
    float p[7][3];    /* p[i] 坐标系{i}原点位置 */
    float z[7][3];    /* z[i] 坐标系{i}的Z轴方向 (基坐标系下) */

    for (int i = 0; i <= N_JOINTS; i++) {
        /* 原点: 平移部分 */
        p[i][0] = T0_i[i][3];
        p[i][1] = T0_i[i][7];
        p[i][2] = T0_i[i][11];

        /* Z轴: 旋转矩阵第三列 (索引2,6,10) */
        z[i][0] = T0_i[i][2];
        z[i][1] = T0_i[i][6];
        z[i][2] = T0_i[i][10];
    }

    /* 3. 计算各连杆质心在基坐标系下的位置 pc[i] (i=1..6) */
    float pc[7][3];   /* pc[i] 连杆i质心位置，i从1开始，pc[0]未使用 */
    for (int i = 1; i <= N_JOINTS; i++) {
        transform_point(T0_i[i], link_com[i-1], pc[i]);
    }

    /* 4. 初始化力矩为0 */
    for (int j = 0; j < N_JOINTS; j++) tau[j] = 0.0f;

    /* 5. 对每个关节j，累加所有连杆i (i>=j) 的重力贡献 */
    for (int j = 1; j <= N_JOINTS; j++) {   /* 关节编号j从1到6 */
        /* 关节j的轴线过点 p[j-1] (即坐标系{j-1}原点)，方向为 z[j-1] */
        float *p_joint = p[j-1];   /* 关节轴线上一点 */
        float *z_joint = z[j-1];   /* 关节轴线方向 */

        for (int i = j; i <= N_JOINTS; i++) {
            /* 连杆i质心重力矢量 (基坐标系) */
            float f[3] = {0.0f, 0.0f, -link_mass[i-1] * GRAVITY};

            /* 计算力臂向量 r = pc[i] - p_joint */
            float r[3];
            vec_sub(pc[i], p_joint, r);

            /* 力矩向量 = r × f */
            float m[3];
            vec_cross(r, f, m);

            /* 关节j的标量力矩 = m · z_joint */
            tau[j-1] += vec_dot(m, z_joint);
        }
    }
}
