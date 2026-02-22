#include "servo_mapping.h"
#include "crc.h"
#include "cmsis_os.h"
#include "ZDT_X42_V2.h"
#include "Gravity_comp.h"

extern uint8_t uart7_rebuffer[SERVO_RX_BUF_NUM];

uint8_t transmitBuffer[9];
uint8_t len;
static uint8_t pos_txbuf[6];  // 55 55 id 03 28 checksum
static uint8_t load_txbuf[7];  // 55 55 id 03 28 checksum

float theta[MOTORS_NUM];//rad,弧度制2π~-2π
float tau[MOTORS_NUM];

servoMapping ServoMap;
Servo Servos[SERVOS_NUM];
moterMapHeader   MoterMap;


Servo                   LastServos[SERVOS_NUM];
customController_t      CustomController;
JudgeTxFrame            txFrame;
FrequencyCheck		   taskFrequencyCheck;

MotorCurrentInfo MotorCurrents[SERVOS_NUM];

void JustFloat_send(moterMapHeader *MoterMap);
void testtask(void);
uint16_t lowV(Servo* servo, uint16_t currentAngle) {
    float dPower = 1.0f;
    uint16_t filtered = (uint16_t)(currentAngle * dPower + (1 - dPower) * servo->lastFilteredAngle);
    servo->lastFilteredAngle = filtered;
    return filtered;
}

void TaskFrequencyCheck(uint8_t tasknum)
{
	taskFrequencyCheck.timtick[tasknum] ++;
	if(taskFrequencyCheck.timtick[tasknum] >999)
	{
		taskFrequencyCheck.timtick[tasknum] = 0;
		taskFrequencyCheck.Frequency[tasknum] = 0;
	}
}
void TaskFrequencycount(uint8_t tasknum)
{
	if(taskFrequencyCheck.lastFrequency[tasknum]<= taskFrequencyCheck.Frequency[tasknum]) taskFrequencyCheck.lastFrequency[tasknum] = taskFrequencyCheck.Frequency[tasknum];
	taskFrequencyCheck.Frequency[tasknum] ++;
}

#if controller_mode == servo_controller

void Servo_Task(void)
{
    FillServoPacket_POS(ServoMap.ZX_Data.transmitIndex, pos_txbuf);//填充发送帧:pos
    HAL_UART_Transmit_DMA(&huart7, pos_txbuf, 6);//发送
    
    ServoMap.ZX_Data.transmitIndex += 1;//切换下一个舵机
    if (ServoMap.ZX_Data.transmitIndex == 6)//舵机序号复位
        ServoMap.ZX_Data.transmitIndex = 0;
	
    CustomController_AngleMapping();//角度映射计算
	
	
}
void Robot_Task(void)
{

		//if (ServoMap.ZX_Data.dataEnable == 0x01) {
        CustomController_StructSend(&MoterMap);
		//}
		//JustFloat_send(&MoterMap);
	
}
#endif

#if controller_mode == zdt_controller

void Servo_Task(void)
{
	
}

void Robot_Task(void)
{
	
}
void motor_mapping_init(void)
{
	for (uint8_t i = 0; i < SERVOS_NUM; i++)
    {
        MotorCurrents[i].id = i + 1;               // ID从1开始
        MotorCurrents[i].target_current = 0;
        MotorCurrents[i].actual_current = 0;
        MotorCurrents[i].voltage = 0;
        MotorCurrents[i].temperature = 0;
        MotorCurrents[i].error_code = 0;
        MotorCurrents[i].enabled = 0;
		MotorCurrents[i].position = 0.0f;
		MotorCurrents[i].dir = 0;
    }
}
void Read_zdt_Pos(void)
{
	ZDT_X42_V2_Read_Sys_Params(3 ,S_CPOS);
	ZDT_X42_V2_Read_Sys_Params(4 ,S_CPOS);
	ZDT_X42_V2_Read_Sys_Params(5 ,S_CPOS);
	ZDT_X42_V2_Read_Sys_Params(6 ,S_CPOS);
}
void Set_Taget_Torque(void)
{
	/*计算重力补偿*/
	gravity_compensation(&theta[MOTORS_NUM], &tau[MOTORS_NUM]);
	/*发送补偿力矩*/
	ZDT_X42_V2_Torque_Control(3, 0, 1,10,0);
	ZDT_X42_V2_Torque_Control(4, 0, 1,10,0);
	ZDT_X42_V2_Torque_Control(5, 0, 1,10,0);
	ZDT_X42_V2_Torque_Control(6, 0, 1,10,0);
//	for(uint8_t i;i<6;i ++)
//	{
//		ZDT_X42_V2_Torque_Control(i, MotorCurrents[i].dir, tau[i]/2,tau[i],0);
//	}
}
#endif
/**
  * @brief  处理从 CAN 接收到的 ZDT 电机位置数据帧
  * @param  can_id  电机 ID（从 CAN 扩展 ID 高 16 位提取）
  * @param  data    指向 8 字节数据缓冲区的指针（已通过 CAN 接收）
  * @param  len     实际接收到的数据长度（由 fdcanx_receive 返回）
  * @retval None
  */
void process_zdt_can_frame(uint16_t can_id, uint8_t *data, uint8_t len)
{
    // 校验功能码 (0x36) 和帧尾 (0x6B)
    if (len < 7 || data[0] != 0x36 || data[6] != 0x6B) {
        return;  // 无效帧
    }

    uint8_t id = (uint8_t)can_id;  // 低 8 位即为电机 ID
    if (id < 1 || id > MOTORS_NUM) {
        return;
    }

    // 解析 4 字节位置值（大端序：高位在前）
    int32_t raw_pos = ((int32_t)data[2] << 24) |
                      ((int32_t)data[3] << 16) |
                      ((int32_t)data[4] << 8)  |
                      data[5];

    // 符号位：data[1] 为 0x00 正，0x01 负
    if (data[1] == 0x01) {
        raw_pos = -raw_pos;
    }

    // 转换为浮点数（单位：度），原始值放大了 10 倍
    float position = raw_pos / 10.0f;

    // 存入对应电机的结构体（ID 从 1 开始，数组索引为 id-1）
    MotorCurrents[id - 1].position = position;
	if(position >= 0)
		MotorCurrents[id - 1].dir = 0;
	else
		MotorCurrents[id - 1].dir = 1;
}
/**
* @brief  从电机位置得到dh参数θ角
* @param  MotorCurrents 电机参数结构体
  * @param  
  * @param  
  * @retval None
  */
void Get_theta(MotorCurrentInfo * MotorCurrents[MOTORS_NUM])
{
	theta[0] = PI/2;
	theta[1] = -MotorCurrents[1]->position;
	theta[2] = -PI/2 - MotorCurrents[2]->position;
	theta[3] = - MotorCurrents[3]->position;
	theta[4] = - MotorCurrents[4]->position;
	theta[5] = - MotorCurrents[5]->position;
}
/**
  * @brief  填充位置查询数据包
  * @param  id: 舵机ID
  * @param  packet: 输出数据包缓冲区（至少6字节）
  * @retval None
  */
void FillServoPacket_POS(uint8_t id, uint8_t* packet)
{
    packet[0] = 0x55;
    packet[1] = 0x55;
    packet[2] = id;        // ID
    packet[3] = 0x03;      // 数据长度
    packet[4] = SERVO_POS_READ;      // 指令
    
    // 计算校验和: Checksum = ~(ID + Length + Cmd)
    uint8_t checksum = ~(id + packet[3] + packet[4]);
    packet[5] = checksum;
}
/**
  * @brief  填充卸载掉电数据包
  * @param  id: 舵机ID
  * @param  packet: 输出数据包缓冲区（至少6字节）
  * @retval None
  */
void FillServoPacket_LOAD(uint8_t id, uint8_t* packet)
{
    packet[0] = 0x55;
    packet[1] = 0x55;
    packet[2] = id;        // ID
    packet[3] = 0x04;      // 数据长度
    packet[4] = SERVO_LOAD_OR_UNLOAD_WRITE;      // 指令
	packet[5] = 0x00;
    
    // 计算校验和: Checksum = ~(ID + Length + Cmd)
    uint8_t checksum = ~(id + packet[3] + packet[4])+packet[5];
    packet[6] = checksum;
}
/**
  * @brief  线性缩放
  * @param  original_value      //初始值
  * @param  old_min             //最小初始值
  * @param  old_max             //最大初始值
  * @param  new_min             //最小新值
  * @param  new_max             //最大新值
  * @retval new_value
  */
float ScaleValue(float original_value, float old_min, float old_max, float new_min, float new_max) {
    float new_value = new_min + ((original_value - old_min) * (new_max - new_min)) / (old_max - old_min);
    return new_value;
}

/**
  * @brief  舵机映射初始化
  * @param  None
  * @retval None
  */
void Servo_Mapping_Init(void)
{
    ServoMap.ZX_Data.receiveFlag   = 0x01;
    ServoMap.ZX_Data.transmitIndex = 0x00;
//		MoterMap.actuator = 0x01;
		ServoMap.ZX_Data.errorId = 0x00;
    for (uint8_t i = 1; i < SERVOS_NUM; i++)
    {
        Servos[i].id    = i;
    }
		Servos[0].angle = 505;
		Servos[1].angle =  1500;
		Servos[2].angle = 1500;
		Servos[3].angle =  1000;
		Servos[4].angle = 2450;
		Servos[5].angle =  833;

		
    for (uint8_t i = 1; i < SERVOS_NUM; i++)
    {
        LastServos[i].id    = Servos[i].id;
        LastServos[i].angle = Servos[i].angle;
    }
	for(uint8_t i;i<2;i++){
		FillServoPacket_LOAD(ServoMap.ZX_Data.transmitIndex, load_txbuf);
		HAL_UART_Transmit_DMA(&huart7, load_txbuf, sizeof(load_txbuf));//发送
		ServoMap.ZX_Data.transmitIndex += 1;//切换下一个舵机
		if (ServoMap.ZX_Data.transmitIndex == 6)//舵机序号复位
		{
			ServoMap.ZX_Data.transmitIndex = 0;}
	}
//    buzzer_on(5,5000);
//    HAL_Delay(100);
//    buzzer_off();
//    HAL_Delay(100);
//    buzzer_on(5,5000);
//    HAL_Delay(100);
//    buzzer_off();
//    HAL_Delay(100);
//    buzzer_on(5,5000);
//    HAL_Delay(100);
//    buzzer_off();
//    HAL_Delay(100);
//    buzzer_on(5,5000);
//    HAL_Delay(100);
//    buzzer_off();
//    HAL_Delay(100);
}

/**
  * @brief  自定义控制器数据发送
  * @param  *data      //电机映射数据
  * @retval None
  */
void CustomController_StructSend(moterMapHeader *data)
{

    //设置帧头
    CustomController.txFrameHeader.SOF = 0XA5;
    CustomController.txFrameHeader.DataLength = sizeof(moterMapHeader);
    CustomController.txFrameHeader.Seq = 0;
    //将帧头复制到发送帧数据中
    memcpy(txFrame.data, &CustomController.txFrameHeader, sizeof(xFrameHeader));
    //计算并添加CRC8校验码
    Append_CRC8_Check_Sum(txFrame.data, sizeof(xFrameHeader));
    //设置数据帧头
    CustomController.CmdID = 0x302;  // 数据帧ID
    //将映射数据复制到数据帧中
    CustomController.moterMap = *data;
    //将数据帧的其余部分复制到发送帧数据中
    memcpy(txFrame.data + sizeof(xFrameHeader),
           (uint8_t*)&CustomController.CmdID,
           sizeof(CustomController.CmdID) + sizeof(moterMapHeader));
     // 计算并添加CRC16校验码
    Append_CRC16_Check_Sum(txFrame.data, sizeof(xFrameHeader) + sizeof(CustomController.CmdID) + sizeof(CustomController.moterMap)+2);
    // 设置发送帧的总长度
    txFrame.frameLength = sizeof(xFrameHeader) + sizeof(CustomController.CmdID) + sizeof(CustomController.moterMap)+ sizeof(CustomController.FrameTail);
    // 初始化发送缓冲区
    memset(txFrame.data + txFrame.frameLength, 0, 40 - txFrame.frameLength);
    // 发送数据
    HAL_UART_Transmit_DMA(&huart1, txFrame.data, txFrame.frameLength);

}

void UpdateServoAngle(Servo Servos[],Servo LastServos[]) {
    for (uint8_t i = 1; i < 8; i++) {
        if (Servos[i].angle < ANGLE_MIN || Servos[i].angle  > ANGLE_MAX) 
            Servos[i].angle = LastServos[i].angle;
         else 
            LastServos[i].angle = Servos[i].angle;
    }
}
uint16_t Filtering_angle[SERVOS_NUM];
void CustomController_AngleMapping(void)
{
		//舵机数据超限判断
//    UpdateServoAngle(Servos,LastServos);
		//按键自锁判断
//		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) == GPIO_PIN_RESET)
//			ServoMap.ZX_Data.enableCnt++;
//		else
//			ServoMap.ZX_Data.enableCnt = 0;
//		if(ServoMap.ZX_Data.enableCnt > 10)
//        {
//            MoterMap.enable = !MoterMap.enable;
//						if(MoterMap.enable == 0x01)
//							MoterMap.actuator = 0x01;
//            ServoMap.ZX_Data.enableCnt = 0;
//        }
//            		
//    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15) == GPIO_PIN_RESET)
//        ServoMap.ZX_Data.actuatorCnt++;
//		else
//				ServoMap.ZX_Data.actuatorCnt = 0;
//    if(ServoMap.ZX_Data.actuatorCnt > 10)
//    {   
//        MoterMap.actuator = !MoterMap.actuator;
//        ServoMap.ZX_Data.actuatorCnt = 0;
//    }
		//判断超时（信号丢失）时间
//		for(uint8_t i = 1;i < SERVOS_NUM ;i++)
//		{
//			Servos[i].cnt++;
//			ServoMap.ZX_Data.totalCnt += Servos[i].cnt;
//			if(Servos[i].cnt < 20 && Servos[i].cnt >= 0 && ServoMap.ZX_Data.totalCnt == 28)
//			{
//				ServoMap.ZX_Data.errorId = 0xff;
//				ServoMap.ZX_Data.dataEnable = 0x01;
//			}
//			else
//				ServoMap.ZX_Data.dataEnable = 0x00;
//			if(Servos[i].cnt >= 20)
//			{
//				ServoMap.ZX_Data.errorId = Servos[i].id;
//				ServoMap.ZX_Data.dataEnable = 0x00;
//			}
//                
//		}
//		ServoMap.ZX_Data.totalCnt = 0;
		
		for(uint8_t i = 0; i < SERVOS_NUM; i++) {
			Filtering_angle[i] = lowV(&Servos[i], Servos[i].angle);
		}
		MoterMap.online = 1;
		//舵机数据映射
		MoterMap.j5         = ScaleValue(Filtering_angle[5],J5_MIN,J5_MAX,2*PI/3,-2*PI/3);
		MoterMap.j4       	= ScaleValue(Filtering_angle[4],J4_MIN,J4_MAX,2*PI/3,-2*PI/3);
		MoterMap.j3      	= ScaleValue(Filtering_angle[3],J3_MIN,J3_MAX,2*PI/3,-2*PI/3);
		MoterMap.j2         = - ScaleValue(Filtering_angle[2],J2_MIN,J2_MAX,2*PI/3,-2*PI/3);
		MoterMap.j1         = - ScaleValue(Filtering_angle[1],J1_MIN,J1_MAX,2*PI/3,-2*PI/3);
		MoterMap.j0         = - ScaleValue(Filtering_angle[0],J0_MIN,J0_MAX,2*PI/3,-2*PI/3);

		LIMIT(MoterMap.j5       ,-2*PI/3	,2*PI/3	);
		LIMIT(MoterMap.j4     	,-2*PI/3	,2*PI/3	);
		LIMIT(MoterMap.j3    	,-2*PI/3	,2*PI/3	);
		LIMIT(MoterMap.j2       ,-2*PI/3	,2*PI/3	);
		LIMIT(MoterMap.j1       ,-2*PI/3	,2*PI/3	);
		LIMIT(MoterMap.j0       ,-2*PI/3	,2*PI/3	);
      
}


//void Servo_Task(void){
//    BuildServoQueryPacket(ServoMap.ZX_Data.transmitIndex, pos_txbuf);
//    if(ServoMap.ZX_Data.errorReceiveCnt > 9)                // 如果接收错误计数超过9
//    {
//        ServoMap.ZX_Data.errorReceiveCnt = 1;              // 重置接收错误计数为1
//        ServoMap.ZX_Data.errorTransmitCnt = 10;            // 将传输错误计数设置为10
//    }
//    if(ServoMap.ZX_Data.errorReceiveCnt == 0)              // 当接收错误计数为0（即没有接收错误）
//    {     
//        HAL_UART_Transmit_DMA(&huart7, pos_txbuf, 6);    	// 使用DMA中断发送
//        if(ServoMap.ZX_Data.errorTransmitCnt > 0)          // 如果传输错误计数大于0
//            ServoMap.ZX_Data.errorTransmitCnt--;           // 传输错误计数递减
//    }   
//    ServoMap.ZX_Data.receiveFlag = 0x00;                   // 清除接收标志
//	ServoMap.ZX_Data.transmitIndex += 1;					//transmitIndex的自增
//    if (ServoMap.ZX_Data.transmitIndex == 8)
//		ServoMap.ZX_Data.transmitIndex = 1;
//    // 修改后的接收数据处理 - 直接解析二进制数据
//    if(uart7_rebuffer[0] == 0x55 && uart7_rebuffer[1] == 0x55 && 
//       uart7_rebuffer[3] == 0x05 && uart7_rebuffer[4] == 0x28)  // 长度变为0x05，因为有位置数据
//    {   
//        uint8_t id = uart7_rebuffer[2];  // 直接获取ID
//        
//        int16_t position = (int16_t)((uart7_rebuffer[6] << 8) | uart7_rebuffer[5]);// 提取位置数据：低字节在前，高字节在后
//        // 更新舵机数据
//        Servos[id].cnt = 0;
//        Servos[id].id = id;
//        Servos[id].angle = position;   
//        // 接收数组数据清空
//        for (uint8_t i = 0; i < 10; i++)
//            uart7_rebuffer[i] = 0;
//    }
//    CustomController_AngleMapping();
//}

//void Robot_Task(void){
//    if ((uart7_rebuffer[0] != 0x55 || uart7_rebuffer[1] != 0x55 || uart7_rebuffer[3] != 0x05 || uart7_rebuffer[4] != 0x28) && ServoMap.ZX_Data.errorTransmitCnt == 0)
//    {
//        if(ServoMap.ZX_Data.errorReceiveCnt < 10)  // 防止数组越界
//        {
//            __HAL_UART_DISABLE_IT(&huart7,UART_IT_IDLE);
//            
//            uart7_rebuffer[ServoMap.ZX_Data.errorReceiveCnt] = 0;	//清空接收缓冲区
//        }
//        ServoMap.ZX_Data.errorReceiveCnt++;  
//    }
//    if (ServoMap.ZX_Data.errorTransmitCnt == 10)    //当传输错误计数为10时会重新启用UART中断
//    {
//        __HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE);
//        ServoMap.ZX_Data.errorReceiveCnt = 0;
//    }
//    
//    if(MoterMap.actuator == 0)
//        ServoMap.ZX_Data.recoveryCnt++;
//    if(MoterMap.actuator == 0 && ServoMap.ZX_Data.recoveryCnt > 50) 
//    {
//        MoterMap.actuator = 1;
//        ServoMap.ZX_Data.recoveryCnt = 0;
//    }
//    if (ServoMap.ZX_Data.dataEnable == 0x01)
//        CustomController_StructSend(&MoterMap);
//}
GPIO_PinState key_lev;
void Get_Keynum(void)
{
	key_lev = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    if(key_lev == GPIO_PIN_RESET)
    {
		MoterMap.clamp = 1;
    }
	else if(key_lev == GPIO_PIN_SET)
	{
		MoterMap.clamp = 0;
    }
}


#define CH_COUNT 6
typedef struct Frame {
    float fdata[CH_COUNT];
    unsigned char tail[4];
}JustFloat;
JustFloat frame = {
    .tail = {0x00, 0x00, 0x80, 0x7f}
};

void JustFloat_send(moterMapHeader *MoterMap)
{
	frame.fdata[0] = MoterMap->j0;
	frame.fdata[1] = MoterMap->j1;
	frame.fdata[2] = MoterMap->j2;
	frame.fdata[3] = MoterMap->j3;
	frame.fdata[4] = MoterMap->j4;
	frame.fdata[5] = MoterMap->j5;
	
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&frame, sizeof(frame));
}