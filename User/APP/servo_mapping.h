#ifndef __SERVO_MAPPING_H__
#define __SERVO_MAPPING_H__

#include "main.h"

#define servo_controller 0
#define zdt_controller 1
/* =================两个自控选择 ================= */
#define controller_mode zdt_controller

#define TASKNUM 3
#define NORMALTASK 0
#define GETTASK 1
#define SENDTASK 2

#define SERVO_RX_BUF_NUM 10
#define SERVO_POS_READ 0X1C	//28
							//参数 1 ：舵机当前角度位置值的低八位。
							//参数 2 ：舵机当前角度位置值的高八位，无默认值。
							//说明：返回的角度位置值要转换成 signed short int 型数据，因为读出的角度可能为负值
#define SERVO_LOAD_OR_UNLOAD_WRITE 0X1F	//31参数 1 ：舵机内部电机是否卸载掉电，范围0 或 1 ，0 代表卸载掉电，此时舵机 无力矩输出。1 代表装载电机，此时舵机有力矩输出，默认值 0。

#define SERVOS_NUM 0X06
#define MOTORS_NUM 6
#define JUDGE_MAX_TX_LENGTH 40
#define PI 3.1415926f
/*
机械臂相关的一些方向定义
- 机械臂水平向前时设置J0关节为0位置，左旋（-2π/3）——2π/3右旋
    - 机械臂竖直向上时设置J1 J2关节为0位置，前（-2π/3）——2π/3后
    - J3关节0位置待定
    - 定义机械臂水平向前时的J0关节位置为0，从上往下看，逆时针为正方向
    - 定义机械臂竖直向上时的J1 J2关节位置为0，从机械臂右侧看，逆时针为正方向（注：J1 J2关节的合位置为联动位置）
    - 定义机械臂J3水平(同步带位于两侧)时的J3关节位置为0，从吸盘方向看，逆时针为正方向
    - 定义J4为末端机构右侧（上视，J3向前）电机，J5为末端机构左侧电机

    - 定义虚拟J4关节用来衡量末端机构的pitch, 虚拟J5关节用来衡量末端机构的roll
    - 定义J4正方向为：当J3归中时，从机械臂右侧看，逆时针为正方向
    - 定义J5正方向为：当J3归中时，吸盘方向看，0位置为2.06，逆时针为负，顺时针为正，一圈为6.28（2π）
    - vj4和j4 j5的关系：vj4 = (j4 - j5)/
*/
#define J5_MAX 1000
#define J5_MID 500
#define J5_MIN 0
#define J4_MAX 1000
#define J4_MIN 0
#define J3_MAX 1000
#define J3_MIN 0
#define J2_MAX 1000
#define J2_MIN 0
#define J1_MAX 1000
#define J1_MIN 0
#define J0_MAX 1000
#define J0_MIN 0
#define ANGLE_MID 1500
#define ANGLE_MIN 0
#define ANGLE_MAX 1000

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#define ENABLE_KEY 0X01
#define ACTUATOR_KEY 0X02

uint8_t handleButtonPress(uint8_t KEY);

/* 电机电流信息结构体 */
typedef struct {
    uint8_t id;                 // 电机ID (1~6)
    int16_t target_current;     // 目标电流值
    int16_t actual_current;     // 实际电流值
    uint16_t voltage;           // 电压值 (mV)
    int8_t temperature;         // 温度值 (℃)
    uint8_t error_code;         // 错误码
    uint8_t enabled;            // 使能状态
    float position;             // 当前角度位置 (度)
	uint8_t dir;
} MotorCurrentInfo;

typedef struct 
{
	uint16_t	Frequency[TASKNUM];
	uint16_t	lastFrequency[TASKNUM];
	uint16_t 	timtick[TASKNUM];
}FrequencyCheck;

typedef struct 
{
    uint8_t id;       //舵机id
    uint16_t angle;   //角度值
	uint32_t	cnt;  //计时
	uint16_t lastFilteredAngle;
}Servo;

typedef struct 
{
    struct 
    {
        uint8_t receiveFlag;    	//接收标志位
        uint8_t transmitIndex;  	//发送索引
        uint16_t errorReceiveCnt; //错误接收计时
		uint16_t errorTransmitCnt;//错误接收计时
        uint8_t dataEnable;     	//数据使能
		uint16_t totalCnt;				//总错误计时
		uint8_t	enableCnt;				//使能按键计时
		uint8_t	actuatorCnt;			//气泵按键计时	
		uint8_t recoveryCnt;			//气泵自恢复计时
        uint8_t errorId;                //超时错误舵机id
    }ZX_Data;
}servoMapping;

typedef __packed struct 
{   
    uint8_t online;     //使能
    uint8_t clamp;   //气泵
    float j5;           //大臂末端电机
    float j4;           //肘关节电机
    float j3;            //大抬升电机
    float j2;        //PITCH电机
    float j1;         //ROLL电机
    float j0;           //YAW电机
}moterMapHeader;               //电机映射数据		26字节

typedef __packed struct 
{
    uint8_t     SOF;
    uint16_t    DataLength;
    uint8_t     Seq;
    uint8_t     CRC8;
}xFrameHeader;              //帧头结构体		5字节

typedef __packed struct
{
    xFrameHeader                    txFrameHeader;  //帧头			5字节
    uint16_t                        CmdID;          //命令码			2字节
    moterMapHeader                  moterMap;       //电机映射数据段	26字节
    uint16_t                        FrameTail;      //CRC16 			2字节
}customController_t;		//											35字节

typedef struct
{
    uint8_t data[JUDGE_MAX_TX_LENGTH];
    uint16_t frameLength;
}JudgeTxFrame;


extern Servo            				Servos[SERVOS_NUM];
extern servoMapping     				ServoMap;
extern moterMapHeader          	    	MoterMap;
extern FrequencyCheck		   			taskFrequencyCheck;
/* 声明全局电机电流数组 */
extern MotorCurrentInfo MotorCurrents[MOTORS_NUM];

void TaskFrequencyCheck(uint8_t tasknum);
void TaskFrequencycount(uint8_t tasknum);
void Servo_Mapping_Init(void);
void CustomController_StructSend(moterMapHeader *data);
void CustomController_AngleMapping(void);
void FillServoPacket_POS(uint8_t id, uint8_t* packet);
void FillServoPacket_LOAD(uint8_t id, uint8_t* packet);
void buzzer_on(uint16_t psc, uint16_t pwm);
void buzzer_off(void);
void Servo_Task(void);
void Robot_Task(void);
void Get_Keynum(void);
void motor_mapping_init(void);
void Read_zdt_Pos(void);
void Set_Taget_Torque(void);
void Get_theta(MotorCurrentInfo *motor_currents);
void process_zdt_can_frame(uint16_t can_id, uint8_t *data, uint8_t len);
#endif
