#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"
#include "stdbool.h"

#define hcan_t FDCAN_HandleTypeDef

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
void can_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void can_SendCmd(uint8_t *cmd, uint32_t len);
void RS_MOTOR_PRE(FDCAN_HandleTypeDef *hcan,uint16_t id);
uint8_t fdcan2send_test(FDCAN_HandleTypeDef *hcan,uint16_t id);
//void fdcan2_rx_callback(void);
//void fdcan3_rx_callback(void);

#endif /* __BSP_FDCAN_H_ */

