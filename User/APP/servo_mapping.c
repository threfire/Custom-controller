#include "servo_mapping.h"
#include "crc.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ZDT_X42_V2.h"
#include "Gravity_comp.h"
#include "joint_impedance.h"

/* ================= 常量定义 ================= */
#define debug 0
// 不同关节的力矩转换系数
#define TAU_MAP_PARAM1 2000
#define TAU_MAP_PARAM2 8.5f
#define TAU_MAP_PARAM3 4900
#define TAU_MAP_PARAM4 9000
#define TAU_MAP_PARAM5 13600
#define TAU_MAP_PARAM6 26000

// 滤波系数
#define FILTER_ALPHA           0.2f

#define CH_COUNT 6

#define JOINT_NUM 6

#define V_DT_S 0.001f
/* ================= 静态变量 ================= */
static uint8_t pos_txbuf[6];  // 55 55 id 03 28 checksum 机位置查询帧缓冲区
static uint8_t load_txbuf[7];  // 55 55 id 03 28 checksum 舵机加载/卸载帧缓冲区
uint16_t send_tau[6];
float send_tau_mit;

JointImpController_t g_joint_imp[JOINT_NUM];
float g_tau_cmd[JOINT_NUM];      /* 最终关节力矩命令 = 重力补偿 + 阻抗/阻尼 + 其他前馈 */

// JustFloat 格式发送帧
typedef struct Frame {
    float fdata[CH_COUNT];
    unsigned char tail[4];
}JustFloat;
static JustFloat frame = {
    .tail = {0x00, 0x00, 0x80, 0x7f}
};

//关节力矩计算参数常量定义
static const float g_kp_init[JOINT_NUM] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
static const float g_kd_init[JOINT_NUM] = {
    -0.008f, 0.05f, 0.09f, 0.04f, 0.02f, 0.008f
};
static const float g_torque_limit_init[JOINT_NUM] = {
    1.0f, 1.2f, 1.5f, 1.0f, 0.8f, 0.6f
};
static const float g_torque_rate_limit_init[JOINT_NUM] = {
    30.0f, 40.0f, 50.0f, 40.0f, 30.0f, 20.0f
};
static const float g_gear_ratio_init[JOINT_NUM] = {
    1.0f, 6.0f, 6.0f, 6.0f, 6.0f, 6.0f
};
static const float g_kt_init[JOINT_NUM] = {
    0.08f, 0.08f, 0.08f, 0.08f, 0.08f, 0.08f
};
/* ================= 全局变量定义 ================= */
uint8_t transmitBuffer[9];
uint8_t len;
uint16_t Filtering_angle[SERVOS_NUM];
// 原始角度数组（弧度）
float theta[MOTORS_NUM];//rad,弧度制2π~-2π
// 力矩数组（用于重力补偿）
float tau[MOTORS_NUM];

GPIO_PinState key_lev;

servoMapping ServoMap;
Servo Servos[SERVOS_NUM];
moterMapHeader   MoterMap;
Servo                   LastServos[SERVOS_NUM];
customController_t      CustomController;
JudgeTxFrame            txFrame;
FrequencyCheck		   taskFrequencyCheck;
MotorCurrentInfo MotorCurrents[SERVOS_NUM];

// 外部变量
extern uint8_t uart7_rebuffer[SERVO_RX_BUF_NUM];
extern MITMeasure_t MIT_MOTOR_MEASURE;
extern uint8_t datapack_ordorcount;
/* ================= 静态函数声明 ================= */
static void JustFloat_send(moterMapHeader *MoterMap);

	

uint16_t lowV(Servo* servo, uint16_t currentAngle) {
    float dPower = 1.0f;
    uint16_t filtered = (uint16_t)(currentAngle * dPower + (1 - dPower) * servo->lastFilteredAngle);
    servo->lastFilteredAngle = filtered;
    return filtered;
}

/* ================= 任务频率检测 ================= */
void TaskFrequencyCheck(uint8_t tasknum) {
    taskFrequencyCheck.timtick[tasknum]++;
    if (taskFrequencyCheck.timtick[tasknum] > 999) {   // 1ms 中断，1000 次 = 1秒
        taskFrequencyCheck.timtick[tasknum] = 0;
        // 保存上一秒的频率值
        taskFrequencyCheck.lastFrequency[tasknum] = taskFrequencyCheck.Frequency[tasknum];
        taskFrequencyCheck.Frequency[tasknum] = 0;
    }
}

void TaskFrequencycount(uint8_t tasknum) {
    taskFrequencyCheck.Frequency[tasknum]++;
}
/* ================= 舵机通信包构建 ================= */
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
/* ================= 辅助函数 ================= */
/**
  * @brief  力矩限幅
  * @param  None
  * @retval None
  */
uint16_t Limite_tauqe(uint16_t send_tauqe, uint16_t max, uint16_t min)
{
    if(send_tauqe > max) send_tauqe = max;
    if(send_tauqe < min) send_tauqe = min;
    return send_tauqe;
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
static float ScaleValue(float original_value, float old_min, float old_max, float new_min, float new_max) {
    float new_value = new_min + ((original_value - old_min) * (new_max - new_min)) / (old_max - old_min);
    return new_value;
}
/* ================= 初始化函数 ================= */
/**
 * @brief  
 * @param  
 * @retval 
 * @note   
 */
void JointControllers_Init(void)
{
    uint8_t i;
    JointImpConfig_t cfg;

    for (i = 0; i < JOINT_NUM; i++)
    {
        JointImp_DefaultConfig(&cfg);

        cfg.kp = g_kp_init[i];
        cfg.kd = g_kd_init[i];
        cfg.torque_limit = g_torque_limit_init[i];
        cfg.torque_rate_limit = g_torque_rate_limit_init[i];
        cfg.vel_lpf_alpha = 0.90f;

        cfg.torque_constant = g_kt_init[i];
        cfg.gear_ratio = g_gear_ratio_init[i];
        cfg.transmission_efficiency = 1.0f;
        cfg.enable_angle_wrap = false;

        JointImp_Init(&g_joint_imp[i], &cfg);
        JointImp_Reset(&g_joint_imp[i], 0.0f);
        JointImp_EnableHoldCurrent(&g_joint_imp[i]);

        g_tau_cmd[i] = 0.0f;
    }
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
	for(uint8_t i = 0;i<2;i++)
	{
		FillServoPacket_LOAD(ServoMap.ZX_Data.transmitIndex, load_txbuf);
		HAL_UART_Transmit_DMA(&huart7, load_txbuf, sizeof(load_txbuf));//发送
		ServoMap.ZX_Data.transmitIndex += 1;//切换下一个舵机
		if (ServoMap.ZX_Data.transmitIndex == 6)//舵机序号复位
		{
			ServoMap.ZX_Data.transmitIndex = 0;}
	}
}
/**
  * @brief  电机映射初始化
  * @param  None
  * @retval None
  */
void motor_mapping_init(void)
{
	MoterMap.online = 0;
	JointControllers_Init();
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
	Motor_ENABLE(&hfdcan2 , 0x02);
	Motor_save_zero(&hfdcan2 , 0x02);
}
/* ================= 角度映射核心部分 ================= */
/**
  * @brief  舵机数据映射
  * @param  None
  * @retval None
  */
static void map_servo_to_joint(void)
{
	for(uint8_t i = 0; i < SERVOS_NUM; i++) 
	{
			Filtering_angle[i] = lowV(&Servos[i], Servos[i].angle);
	}
	MoterMap.j5         = ScaleValue(Filtering_angle[5],J5_MIN,J5_MAX,2*PI/3,-2*PI/3);
	MoterMap.j4       	= ScaleValue(Filtering_angle[4],J4_MIN,J4_MAX,2*PI/3,-2*PI/3);
	MoterMap.j3      	= ScaleValue(Filtering_angle[3],J3_MIN,J3_MAX,2*PI/3,-2*PI/3);
	MoterMap.j2         = - ScaleValue(Filtering_angle[2],J2_MIN,J2_MAX,2*PI/3,-2*PI/3);
	MoterMap.j1         = - ScaleValue(Filtering_angle[1],J1_MIN,J1_MAX,2*PI/3,-2*PI/3);
	MoterMap.j0         = - ScaleValue(Filtering_angle[0],J0_MIN,J0_MAX,2*PI/3,-2*PI/3);
}
/**
* @brief  电机数据映射
  * @param  None
  * @retval None
  */
static void map_zdt_to_joint(void)
{
	MoterMap.j0 = MotorCurrents[0].position * PI / 180.0f;    // 
	MoterMap.j1 = PI/2 - MotorCurrents[1].position;             // 
	MoterMap.j2 = - PI + MotorCurrents[2].position * PI / 180.0f;    // 
	MoterMap.j3 = MotorCurrents[3].position * PI / 180.0f;      // 
	MoterMap.j4 = 1.2f*MotorCurrents[4].position * PI / 180.0f;      // 
	MoterMap.j5 = MotorCurrents[5].position * PI / 180.0f;      // 
}
/**
* @brief  自控角度映射
  * @param  None
  * @retval None
  */
void CustomController_AngleMapping(void)
{
	
	#if CONTROLLER_MODE == CONTROLLER_MODE_SERVO		//舵机自控
	
	map_servo_to_joint();
	
	#elif CONTROLLER_MODE == CONTROLLER_MODE_ZDT		//电机自控
	
	map_zdt_to_joint();
	
	#endif
	
	//输出限幅
	LIMIT(MoterMap.j0       ,-PI/2		,PI/2	);
	LIMIT(MoterMap.j1     	, -PI/2	    , PI/2	);
	LIMIT(MoterMap.j2    	, -11*PI/12	,-PI/4	);
	LIMIT(MoterMap.j3       ,-PI		,PI	);
	LIMIT(MoterMap.j4       ,-2*PI/3	,2*PI/3	);
	LIMIT(MoterMap.j5       ,-2*PI/3	,2*PI/3	);  
}
/* ================= 从电机位置得到DH参数θ角 ================= */
/**
  * @brief  从电机位置得到dh参数θ角
  * @param  MotorCurrents 电机参数结构体
  * @retval None
	ps1如果遇到大小变化正确，但是符号相反等情况，要么修改alpha要么修改theta，取反应该就能让符号符合期望
	ps2建模没问题的情况下，一般修改符号取反即可
  */
void Get_theta(MotorCurrentInfo *motor_currents, uint8_t id)
{
	if	(id == 0)		theta[0] = motor_currents[0].position;
	else if(id == 1)		theta[1] = motor_currents[1].position;  
	else if(id == 2)		theta[2] = PI/2 - motor_currents[2].position*PI / 180.0f;
	else if(id == 3)		theta[3] = - motor_currents[3].position*PI / 180.0f;
	else if(id == 4)		theta[4] = - motor_currents[4].position*PI / 180.0f;
	else if(id == 5)		theta[5] = motor_currents[5].position*PI / 180.0f;
}
/* ================= 处理ZDT CAN帧 ================= */
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
    if (len < 7 || data[0] != 0x36 || data[6] != 0x6B)
	{
        return;  // 无效帧
    }
    uint8_t id = (uint8_t)can_id;  // 低 8 位即为电机 ID
    if (id < 1 || id > MOTORS_NUM)
	{
        return;
    }
    // 解析 4 字节位置值（大端序：高位在前）
    int32_t raw_pos = ((int32_t)data[2] << 24) |
                      ((int32_t)data[3] << 16) |
                      ((int32_t)data[4] << 8)  |
                      data[5];
    // 符号位：data[1] 为 0x00 正，0x01 负
    if (data[1] == 0x01)
	{
        raw_pos = -raw_pos;
    }
    // 转换为浮点数（单位：度），原始值放大了 10 倍
    float position = raw_pos / 10.0f;
    // 存入对应电机的结构体（ID 从 1 开始，数组索引为 id-1）
    MotorCurrents[id - 1].position = position;
	//获取θ角(转换为rad）
	Get_theta(MotorCurrents, id - 1);
}
/* ================= 计算发送力矩 ================= */
static void Update_All_Joint_Impedance(float dt_s)
{
    uint8_t i;

    for (i = 0; i < JOINT_NUM; i++)
    {
		/* 1) 更新关节位置和速度 */
		JointImp_UpdateMeasurementWithVelocity(&g_joint_imp[i], theta[i], MotorCurrents[i].dq);
        /* 2) 写入重力补偿 */
        JointImp_SetGravityTorque(&g_joint_imp[i], tau[i]);

        /* 3) 额外前馈，暂时设 0 */
        JointImp_SetFeedforwardTorque(&g_joint_imp[i], 0.0f);

        /* 4) 计算最终关节力矩 */
		if(i != 1)		//关节1是mit电机不需要自行计算
		{
			g_tau_cmd[i] = JointImp_ComputeTorque(&g_joint_imp[i], dt_s);
		}
    }
}
/**
  * @brief  计算发送力矩
  * @param  send_tauqe
  * @param  tauqe
  * @retval None
  */
void calc_send_torque(uint16_t* send_tauqe, float * tauqe)
{
	// 设置方向
    for(uint8_t i = 0; i < 6; i++)
    {
        if(tauqe[i] >= 0)
            MotorCurrents[i].dir = 1;
        else
            MotorCurrents[i].dir = 0;
    }
	// 关节2（索引2）对应MIT电机，力矩单独处理
	send_tau_mit = tau[1]*TAU_MAP_PARAM2;
	if(send_tau_mit > 9.6f)send_tau_mit = 9.6f;
	else if(send_tau_mit < -9.6f)send_tau_mit = -9.6f;
	// 其余关节
	send_tauqe[0] = (uint16_t)(fabsf(tauqe[0]) * TAU_MAP_PARAM1);
    send_tauqe[0] = Limite_tauqe(send_tauqe[0], 2500, 0);
    send_tauqe[2] = (uint16_t)(fabsf(tauqe[2]) * TAU_MAP_PARAM3);
    send_tauqe[2] = Limite_tauqe(send_tauqe[2], 2500, 0);   // 接收返回值
    send_tauqe[3] = (uint16_t)(fabsf(tauqe[3]) * TAU_MAP_PARAM4);
    send_tauqe[3] = Limite_tauqe(send_tauqe[3], 2000, 0);
    send_tauqe[4] = (uint16_t)(fabsf(tauqe[4]) * TAU_MAP_PARAM5);
    send_tauqe[4] = Limite_tauqe(send_tauqe[4], 2000, 0);
    send_tauqe[5] = (uint16_t)(fabsf(tauqe[5]) * TAU_MAP_PARAM6);
    send_tauqe[5] = Limite_tauqe(send_tauqe[5], 1000, 0);
}
/* ================= 发送自定义控制器数据 ================= */
/**
  * @brief  自定义控制器数据发送
  * @param  *data	电机映射数据
  * @retval None
  */
void CustomController_StructSend(moterMapHeader *data)
{
	datapack_ordorcount = HAL_GetTick();
	data->res[0] = datapack_ordorcount>>8;
	data->res[1] = datapack_ordorcount & 0xff;
    //设置帧头
    CustomController.txFrameHeader.SOF = 0XA5;
    CustomController.txFrameHeader.DataLength = sizeof(moterMapHeader);
    CustomController.txFrameHeader.Seq = 0;
    //将帧头复制到发送帧数据中
    memcpy(txFrame.data, &CustomController.txFrameHeader, sizeof(xFrameHeader));
    //计算并添加CRC8校验码
    Append_CRC8_Check_Sum(txFrame.data, sizeof(xFrameHeader));
    //设置数据帧头
    CustomController.CmdID = 0x0302;  // 数据帧ID
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
    HAL_UART_Transmit_DMA(&huart7, txFrame.data, txFrame.frameLength);
}
/* ================= JustFloat 格式发送 ================= */
/**
  * @brief  JustFloat 格式发送
  * @param  *MoterMap	电机映射数据
  * @retval None
  */
static void JustFloat_send(moterMapHeader *MoterMap)
{
	frame.fdata[0] = MoterMap->j0;
	frame.fdata[1] = MoterMap->j1;
	frame.fdata[2] = MoterMap->j2;
	frame.fdata[3] = MoterMap->j3;
	frame.fdata[4] = MoterMap->j4;
	frame.fdata[5] = MoterMap->j5;
	
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&frame, sizeof(frame));
}
/* ================= 按键处理 ================= */
/**
  * @brief  按键处理
  * @param  None
  * @retval None
  */
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
/* ================= 任务函数（根据两种模式不同实现） ================= */

#if CONTROLLER_MODE == CONTROLLER_MODE_SERVO		//舵机模式

void Servo_Task(void)
{
	// 发送位置查询指令
    FillServoPacket_POS(ServoMap.ZX_Data.transmitIndex, pos_txbuf);//填充发送帧:pos
    HAL_UART_Transmit_DMA(&huart7, pos_txbuf, 6);//发送
    // 更新发送索引
    ServoMap.ZX_Data.transmitIndex += 1;//切换下一个舵机
    if (ServoMap.ZX_Data.transmitIndex == 6)//舵机序号复位
        ServoMap.ZX_Data.transmitIndex = 0;
	// 角度映射
    CustomController_AngleMapping();//角度映射计算
	
	
}
void Robot_Task(void)
{
	// 发送映射数据
	CustomController_StructSend(&MoterMap);
	//JustFloat_send(&MoterMap);
	
}

#elif CONTROLLER_MODE == CONTROLLER_MODE_ZDT		//电机模式

void Servo_Task(void)
{
	Read_zdt_Pos();
	//角度映射计算
	CustomController_AngleMapping();
	//计算重力补偿
	gravity_compensation(theta, tau);
}

void Robot_Task(void)
{

    // 计算每个关节的角速度
    for (uint8_t i = 0; i < JOINT_NUM; i++)
    {
        // 获取最新的关节角度
        float theta_now = theta[i];
        // 基于差分法计算角速度
        float dq = (theta_now - MotorCurrents[i].theta_prev) / (ROBOT_TASK_PERIOD_S);  // xPeriod为任务周期

        // 更新上次的关节角度
        MotorCurrents[i].theta_prev = theta_now;

        // 存储计算得到的角速度
        MotorCurrents[i].dq = dq;
    }
	// 计算并发送力矩指令
    Update_All_Joint_Impedance(ROBOT_TASK_PERIOD_S);  // 将周期转换为秒
	// 计算发送力矩值
	calc_send_torque(send_tau,g_tau_cmd);
	//发送力矩
	Set_Taget_Torque();
}

#endif

/* ================= ZDT模式下的任务实现 ================= */
#if CONTROLLER_MODE == CONTROLLER_MODE_ZDT
/**
  * @brief  读取位置
  * @param  None
  * @retval None
  */
void Read_zdt_Pos(void)
{
	//关节1和关节3
	USER_ZDT_X42_V2_Read_Sys_Params(&hfdcan2, 1 ,S_CPOS);
	USER_ZDT_X42_V2_Read_Sys_Params(&hfdcan2, 3 ,S_CPOS);
	//关节四五六
	USER_ZDT_X42_V2_Read_Sys_Params(&hfdcan1, 4 ,S_CPOS);
	USER_ZDT_X42_V2_Read_Sys_Params(&hfdcan1, 5 ,S_CPOS);
	USER_ZDT_X42_V2_Read_Sys_Params(&hfdcan1, 6 ,S_CPOS);


}
/**
  * @brief  设置目标力矩
  * @param  None
  * @retval None
  */
void Set_Taget_Torque(void)
{
	#if !debug		//正常发送补偿力矩
	
	USER_ZDT_X42_V2_Torque_Control(&hfdcan2, 1, MotorCurrents[0].dir, 15000,send_tau[0],0);//1为顺时针
	CAN_cmd_MIT(&hfdcan2, 0x02, 0, 0, 0, 0.03, send_tau_mit);
	USER_ZDT_X42_V2_Torque_Control(&hfdcan2, 3, MotorCurrents[2].dir, 25000,send_tau[2],0);//1为顺时针
	
	USER_ZDT_X42_V2_Torque_Control(&hfdcan1, 4, MotorCurrents[3].dir, 15000,send_tau[3],0);//1为顺时针
	USER_ZDT_X42_V2_Torque_Control(&hfdcan1, 5, MotorCurrents[4].dir, 25000,send_tau[4],0);
	USER_ZDT_X42_V2_Torque_Control(&hfdcan1, 6, MotorCurrents[5].dir, 15000,send_tau[5],0);
	#endif
	
	#if debug		//发送0力矩
	
	USER_ZDT_X42_V2_Torque_Control(&hfdcan2, 1, MotorCurrents[0].dir, 10,10,0);//1为顺时针
	CAN_cmd_MIT(&hfdcan2, 0x02, 0, 0, 0, 0.12, 0);
	USER_ZDT_X42_V2_Torque_Control(&hfdcan2, 3, MotorCurrents[2].dir, 10,10,0);//1为顺时针

	USER_ZDT_X42_V2_Torque_Control(&hfdcan1, 4, MotorCurrents[3].dir, 10,10,0);//1为顺时针
	USER_ZDT_X42_V2_Torque_Control(&hfdcan1, 5, MotorCurrents[4].dir, 10,10,0);
	USER_ZDT_X42_V2_Torque_Control(&hfdcan1, 6, MotorCurrents[5].dir, 10,10,0);
	#endif
}
#endif
