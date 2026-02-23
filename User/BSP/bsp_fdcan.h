#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"
#include "stdbool.h"

#define hcan_t FDCAN_HandleTypeDef


//支持MIT协议才用
#define P_MIN -12.5663704f		//位置最小值
#define P_MAX 12.5663704f		//位置最大值
#define V_MIN -45			//速度最小值
#define V_MAX 45			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
//需要根据每个电机的不同来选择
//所以建议在送入发送函数之前进行限幅，
#define T_MIN -1.0f			//转矩最大值
#define T_MAX 1.0f			//转矩最小值

typedef struct
{
    //原始数据
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    //计算后的数据
    float pos;
    float vel;
    float tor; //电机反馈的力矩
    float Kp;
    float Kd;
    float t_mos; //mos温度
    float t_motor; //电机温度
		
		float motor_t;//计算出的电机力矩
		uint32_t last_fdb_time; //电机反馈时间
} MITMeasure_t;

typedef struct {
	
	__IO bool rxFrameFlag;
}CAN_t;
// 声明全局CAN结构体
extern __IO CAN_t can;

/* 错误状态枚举（可根据需要扩展） */
typedef enum {
    CAN_ERROR_NONE         = 0x00,
    CAN_ERROR_WARNING      = 0x01,   // 错误警告
    CAN_ERROR_PASSIVE      = 0x02,   // 错误被动
    CAN_ERROR_BUS_OFF      = 0x04,   // 总线关闭
    CAN_ERROR_PROTOCOL_ARB = 0x08,   // 协议错误（仲裁阶段）
    CAN_ERROR_PROTOCOL_DATA= 0x10,   // 协议错误（数据阶段）
    CAN_ERROR_STUFF        = 0x20,   // 填充错误
    CAN_ERROR_FORM         = 0x40,   // 格式错误
    CAN_ERROR_ACK          = 0x80,   // 应答错误
    CAN_ERROR_CRC          = 0x100,  // CRC错误
	CAN_ERROR_SEND		   = 0x200,  //发送失败
} CAN_ErrorStatus;

/* 声明全局错误状态变量 */
extern __IO CAN_ErrorStatus can_error_status;

// 添加ZDT兼容接收函数声明
void ZDT_Compatible_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);

void bsp_can_init(void);
void can1_filter_init(void);
void can2_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void can_SendCmd(uint8_t *cmd, uint32_t len);
//void fdcan2_rx_callback(void);
//void fdcan3_rx_callback(void);

void CAN_cmd_MIT(FDCAN_HandleTypeDef *hcan,uint16_t id, float _pos, float _vel,
float _KP, float _KD, float _torq);

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x, float x_min, float x_max, int bits);

#endif /* __BSP_FDCAN_H_ */

