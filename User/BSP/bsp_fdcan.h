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

// 添加ZDT兼容接收函数声明
void ZDT_Compatible_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);

void bsp_can_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void can_SendCmd(uint8_t *cmd, uint32_t len);
//void fdcan2_rx_callback(void);
//void fdcan3_rx_callback(void);

#endif /* __BSP_FDCAN_H_ */

