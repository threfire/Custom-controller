#include "bsp_fdcan.h"
#include "string.h"

__IO CAN_t can = {0};

/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN初始化
************************************************************************
**/
void bsp_can_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //启动FDCAN
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_EXTENDED_ID;                       // 改为扩展ID
	fdcan_filter.FilterIndex = 0;                                  // 滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           // 过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = 0x0000;                               // 滤波器ID1
	fdcan_filter.FilterID2 = 0x0000;                               // 滤波器ID2

	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
	
	// 配置全局滤波器：拒绝所有不匹配的帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
		FDCAN_REJECT, 
		FDCAN_REJECT, 
		FDCAN_REJECT_REMOTE, 
		FDCAN_REJECT_REMOTE);
		
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：要发送的数据
* @param:       len：要发送的数据长度
* @retval:     	0-成功, 1-失败
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier = id;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    
    // 经典CAN模式只支持最多8字节数据长度
    if(len > 8) {
        len = 8; // 限制为8字节
    }
    pTxHeader.DataLength = len << 16; // 经典CAN模式下数据长度配置
    
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 经典CAN模式关闭比特率切换
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;   // 经典CAN帧格式
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK) 
		return 1; // 失败
	return 0; // 成功	
}

/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       rec_id：接收到的ID（扩展ID的高16位）
* @param:       buf：接收数据缓冲区
* @retval:     	接收到的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len = 0;
	
	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
	{
		// 提取扩展ID的高16位作为接收ID（保持与原有代码兼容）
		*rec_id = (uint16_t)(pRxHeader.Identifier >> 8);
		
		// 经典CAN模式下，数据长度直接就是字节数
		len = pRxHeader.DataLength >> 16;
		if(len > 8) {
			len = 8; // 确保不超过8字节
		}
		
		return len; // 返回数据长度
	}
	return 0;	
}
//void ZDT_Compatible_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount)
//{
//	FDCAN_RxHeaderTypeDef pRxHeader;
//	uint8_t buf[8] = {0};
//	
//	if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
//	{
//		// 设置接收帧标志
//		can.rxFrameFlag = true;
//		
//		// 填充CAN_RxMsg结构体以兼容ZDT协议
//		can.CAN_RxMsg.ExtId = pRxHeader.Identifier;
//		can.CAN_RxMsg.DLC = pRxHeader.DataLength >> 16;
//		if(can.CAN_RxMsg.DLC > 8) {
//			can.CAN_RxMsg.DLC = 8;
//		}
//		memcpy(can.CAN_RxMsg.Data, buf, can.CAN_RxMsg.DLC);
//		
//		// 填充CAN_TxMsg的ExtId（ZDT协议需要）
//		can.CAN_TxMsg.ExtId = (pRxHeader.Identifier & 0xFF00) << 8; // 地址放在高8位
//		
//		// 复制数据到输出缓冲区
//		*rxCount = can.CAN_RxMsg.DLC;
//		
//		// 第一个字节是地址（从扩展ID的高8位获取）
//		rxCmd[0] = (uint8_t)(pRxHeader.Identifier >> 8);
//		
//		// 后续字节是数据
//		for(uint8_t i = 0; i < can.CAN_RxMsg.DLC && i < 7; i++) {
//			rxCmd[i + 1] = can.CAN_RxMsg.Data[i];
//		}
//		
//		// 如果数据长度小于8，调整rxCount
//		if(can.CAN_RxMsg.DLC > 0) {
//			*rxCount = can.CAN_RxMsg.DLC + 1; // 包括地址字节
//		}
//	}
//	else
//	{
//		can.rxFrameFlag = false;
//		*rxCount = 0;
//	}
//}
/**
************************************************************************
* @brief:      	can_SendCmd
* @param:       cmd：命令数据
* @param:       len：数据长度
* @retval:     	void
* @details:    	发送CAN命令（适配原有ZDT协议）
************************************************************************
**/
void can_SendCmd(uint8_t *cmd, uint32_t len)
{
    uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
    uint8_t send_buffer[8] = {0};
    FDCAN_TxHeaderTypeDef pTxHeader;

    // 去除ID地址和功能码后的数据长度
    j = len - 2;

    // 分包发送
    while(i < j)
    {
        // 剩余数据长度
        k = j - i;

        // 配置发送头
        pTxHeader.Identifier = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum; // 扩展ID格式
        pTxHeader.IdType = FDCAN_EXTENDED_ID; // 使用扩展ID
        pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
        
        // 第一个字节是功能码
        send_buffer[0] = cmd[1];
        
        // 小于8字节数据
        if(k < 8)
        {
            for(l = 0; l < k; l++, i++) 
            { 
                send_buffer[l + 1] = cmd[i + 2]; 
            }
            pTxHeader.DataLength = (k + 1) << 16; // 数据长度
        }
        // 大于等于8字节数据，分包发送，每包最多7个数据字节
        else
        {
            for(l = 0; l < 7; l++, i++) 
            { 
                send_buffer[l + 1] = cmd[i + 2]; 
            }
            pTxHeader.DataLength = 8 << 16; // 固定8字节
        }
        
        pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        pTxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 经典CAN模式关闭比特率切换
        pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;   // 经典CAN帧格式
        pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        pTxHeader.MessageMarker = 0;

        // 发送数据
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, send_buffer);
        
        // 记录发送的包序号
        packNum++;
        
        // 清空发送缓冲区
        memset(send_buffer, 0, sizeof(send_buffer));
    }
}

uint8_t rx_data1[8] = {0};
uint16_t rec_id1;
void fdcan1_rx_callback(void)
{
	fdcanx_receive(&hfdcan1, &rec_id1, rx_data1);

}

//uint8_t rx_data2[8] = {0};
//uint16_t rec_id2;
//void fdcan2_rx_callback(void)
//{
//	fdcanx_receive(&hfdcan2, &rec_id2, rx_data2);
//}
//uint8_t rx_data3[8] = {0};
//uint16_t rec_id3;
//void fdcan3_rx_callback(void)
//{
//	fdcanx_receive(&hfdcan3, &rec_id3, rx_data3);
//}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}
//	if(hfdcan == &hfdcan2)
//	{
//		fdcan2_rx_callback();
//	}
//	if(hfdcan == &hfdcan3)
//	{
//		fdcan3_rx_callback();
//	}
}
