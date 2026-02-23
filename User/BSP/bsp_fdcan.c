#include "bsp_fdcan.h"
#include "string.h"
#include "servo_mapping.h"

__IO CAN_t can = {0};
__IO CAN_ErrorStatus can_error_status = CAN_ERROR_NONE;

uint8_t rx_data1[8] = {0};
uint16_t rec_id1;

uint8_t rx_data2[8] = {0};
uint16_t rec_id2;

uint8_t RS_MOTOR_PRE_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFD}; // 灵足电机私有模式
uint8_t send_test[8]={1, 2, 3, 4, 5, 6, 7, 8};
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
	uint32_t *pRAM = (uint32_t*)(SRAMCAN_BASE + (260 * 4));
	// 假设FDCAN2总配置需要最大100字（可根据实际估算，安全起见可清零剩余所有）
	for (int i = 0; i < 512 - 260; i++) {
		pRAM[i] = 0x00000000;
	}
	HAL_StatusTypeDef status = HAL_FDCAN_Start(&hfdcan2);
	if (status != HAL_OK) {
		// 启动失败，记录错误
		can_error_status |= CAN_ERROR_SEND;
	}
	// 即使返回 HAL_OK，也要手动验证 INIT 位是否清零
	uint32_t timeout = 1000;
	while ((hfdcan2.Instance->CCCR & FDCAN_CCCR_INIT) && timeout--) {
		// 等待
	}
	if (timeout == 0) {
		// INIT 位仍未清零，启动失败
		can_error_status |= CAN_ERROR_SEND;
	}
	hfdcan2.Instance->CCCR |= FDCAN_CCCR_CCE;   // 允许配置更改
	hfdcan2.Instance->CCCR &= ~FDCAN_CCCR_INIT; // 清除 INIT
	while (hfdcan2.Instance->CCCR & FDCAN_CCCR_INIT); // 等待清除
	HAL_FDCAN_ActivateNotification(&hfdcan1, 
                               FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                               FDCAN_IT_ERROR_WARNING |
                               FDCAN_IT_ERROR_PASSIVE |
                               FDCAN_IT_BUS_OFF |
                               FDCAN_IT_ARB_PROTOCOL_ERROR |
                               FDCAN_IT_DATA_PROTOCOL_ERROR,
                               0);
	if(HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	{
		can_error_status |= CAN_ERROR_SEND;
		Error_Handler();
	}
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
	HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
	// 配置全局滤波器：拒绝所有不匹配的帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
//		FDCAN_REJECT, 
//		FDCAN_REJECT, 
		FDCAN_ACCEPT_IN_RX_FIFO0,  // 接收所有标准帧
		FDCAN_ACCEPT_IN_RX_FIFO0,  // 接收所有扩展帧
		FDCAN_FILTER_REMOTE, 
		FDCAN_FILTER_REMOTE);
	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, 
//		FDCAN_REJECT, 
//		FDCAN_REJECT, 
		FDCAN_ACCEPT_IN_RX_FIFO0,  // 接收所有标准帧
		FDCAN_ACCEPT_IN_RX_FIFO0,  // 接收所有扩展帧
		FDCAN_FILTER_REMOTE, 
		FDCAN_FILTER_REMOTE);	
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
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
	
	 memset(&pTxHeader, 0, sizeof(FDCAN_TxHeaderTypeDef));
	
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
	return 2; // 成功	
}

void RS_MOTOR_PRE(FDCAN_HandleTypeDef *hcan,uint16_t id){
	fdcanx_send_data(hcan, id, RS_MOTOR_PRE_MODE, 8);
}
uint8_t fdcan2send_test(FDCAN_HandleTypeDef *hcan,uint16_t id){
	
	return	fdcanx_send_data(hcan, id, send_test, 8);
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
        if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, send_buffer) != HAL_OK)
		{
			can_error_status = CAN_ERROR_SEND;
		}
        
        // 记录发送的包序号
        packNum++;
        
        // 清空发送缓冲区
        memset(send_buffer, 0, sizeof(send_buffer));
    }
}

void fdcan1_rx_callback(void)
{
	uint8_t len = fdcanx_receive(&hfdcan1, &rec_id1, rx_data1);  // 获取实际数据长度
    process_zdt_can_frame(rec_id1, rx_data1, len);               // 解析 ZDT 数据帧
}

void fdcan2_rx_callback(void)
{
	fdcanx_receive(&hfdcan2, &rec_id2, rx_data2);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}
	if(hfdcan == &hfdcan2)
	{
		fdcan2_rx_callback();
	}
//	if(hfdcan == &hfdcan3)
//	{
//		fdcan3_rx_callback();
//	}
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) {
		
        uint32_t hal_error = HAL_FDCAN_GetError(hfdcan);
		
        /* 清除旧状态（可根据需求选择清除全部或保留累积状态） */
        can_error_status = CAN_ERROR_NONE;
        
        /* 映射 HAL 错误到自定义状态 */
        if (hal_error & FDCAN_IT_ERROR_WARNING)   can_error_status |= CAN_ERROR_WARNING;
        if (hal_error & FDCAN_IT_ERROR_PASSIVE)   can_error_status |= CAN_ERROR_PASSIVE;
        if (hal_error & FDCAN_IT_BUS_OFF)   can_error_status |= CAN_ERROR_BUS_OFF;
        if (hal_error & FDCAN_IT_ARB_PROTOCOL_ERROR) can_error_status |= CAN_ERROR_PROTOCOL_ARB;
        if (hal_error & FDCAN_IT_DATA_PROTOCOL_ERROR) can_error_status |= CAN_ERROR_PROTOCOL_DATA;
       
    }
}
