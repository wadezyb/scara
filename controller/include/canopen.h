#ifndef _CANOPEN_H_
#define _CANOPEN_H_
#include <stdint.h>

#define CAN_SPECIFIER (0X12)
#define CAN_NODE1 (0X01)
#define CAN_NODE2 (0X02)
#define CAN_NODE3 (0X03)
#define CAN_NODE4 (0X04)
#define CAN_NODE_MASTER (0X05)

/*����ṹ��*/
typedef struct _CMDOBJ
{
  uint16_t 	ID;
  uint16_t 	Index;			//����
  uint8_t 	SubIndex;		//������
  int32_t 	Param;			//����
  int32_t 	LastParam;	//��һ�εĲ��� ��Ϊ����
}COBJ;


typedef struct _DEVICEOBJ
{
  char Node;
  COBJ TargetPosition;
  COBJ TargetVelocity;
  COBJ ControlWord;
  COBJ StatusWord;
  COBJ ActualPosition;
}DEVICEOBJ;

/* Drive data */
#define MANUFACTURER_DEVICE_NAME_INDEX 			0X1008
#define MANUFACTURER_DEVICE_NAME 						("Smart Drive")
#define MANUFACTURER_HADVARE_VERSION_INDEX 	0X1009
#define MANUFACTURER_HADVARE_VERSION 				("v7.4")
#define MANUFACTURER_SOFTWARE_VERSION_INDEX	0X100A
#define MANUFACTURER_SOFTWARE_VERSION				("2013-11")


/* Control Word 0X6040 */
#define CONTROLWORD_INDEX	0x6040
#define CONTROLWORD_BIT0 	0X0001			//Switch on
#define CONTROLWORD_BIT1 	0X0002			//Enable voltage
#define CONTROLWORD_BIT2 	0X0004			//QuickStop
#define CONTROLWORD_BIT3 	0X0008			//Enable operation
#define CONTROLWORD_BIT4 	0x0010			//Operation mode specific
#define CONTROLWORD_BIT5 	0X0020			//Operation mode specific
#define CONTROLWORD_BIT6 	0X0040			//Operation mode specific
#define CONTROLWORD_BIT7 	0X0080			//Fault reset
#define CONTROLWORD_BIT8 	0X0100			//Halt
#define CONTROLWORD_BIT9 	0X0200			//Reserved
#define CONTROLWORD_BIT10 0X0400			//Reserved
#define CONTROLWORD_BIT11 0X0800			//manufacturer specific
#define CONTROLWORD_BIT12 0X1000			//manufacturer specific
#define CONTROLWORD_BIT13 0X2000			//manufacturer specific
#define CONTROLWORD_BIT14 0X4000			//manufacturer specific
#define CONTROLWORD_BIT15 0X8000			//manufacturer specific
//map
#define CONTROLWORD_SWITCH_ON 				CONTROLWORD_BIT0
#define CONTROLWORD_ENABLE_VOLTAGE 		CONTROLWORD_BIT1
#define CONTROLWORD_QUICK_STOP				CONTROLWORD_BIT2
#define CONTROLWORD_ENABLE_OPERATION	CONTROLWORD_BIT3
#define CONTROLWORD_FAULT_RESET				CONTROLWORD_BIT7
#define CONTROLWORD_CLEAR_POSITION 		CONTROLWORD_BIT4		//user defined
#define CONTROLWORD_UPDATE_POSITION 	CONTROLWORD_BIT5		//user defined


/* STATUSWORD 0X6041 */
#define STATUSWORD_INDEX 		0X6041
#define STATUSWORD_BIT0 	0X0001			//Ready to switch on
#define STATUSWORD_BIT1 	0X0002			//Switched on
#define STATUSWORD_BIT2 	0X0004			//Operation enabled
#define STATUSWORD_BIT3 	0X0008			//Fault
#define STATUSWORD_BIT4 	0x0010			//Voltage enabled
#define STATUSWORD_BIT5 	0X0020			//Quick stop
#define STATUSWORD_BIT6 	0X0040			//Switch on disable
#define STATUSWORD_BIT7 	0X0080			//Warning
#define STATUSWORD_BIT8 	0X0100			//Manufacture specific
#define STATUSWORD_BIT9 	0X0200			//Remote
#define STATUSWORD_BIT10 0X0400			// Target reached
#define STATUSWORD_BIT11 0X0800			// Internal limit active
#define STATUSWORD_BIT12 0X1000			// Operation Mode bit0
#define STATUSWORD_BIT13 0X2000			// Operation Mode bit1
#define STATUSWORD_BIT14 0X4000			// Operation Mode bit2
#define STATUSWORD_BIT15 0X8000			// Operation Mode bit3
// Self Define
#define STATUSWORD_READY_BIT STATUSWORD_BIT0
#define STATUSWORD_SWITCH_BIT STATUSWORD_BIT1
#define STATUSWORD_FAULT_BIT STATUSWORD_BIT3
#define STATUSWORD_VOLTAGE_BIT STATUSWORD_BIT4


//FOR READING STATUS
#define STATE_START										0X00
#define STATE_NOT_READY_TO_SWITCH_ON 	0X01
#define STATE_SWITCH_ON_DISABLED 			0X02
#define STATE_READY_TO_SWITCH_ON 			0X03
#define STATE_SWITCHED_ON 						0X04
#define STATE_OPERATION_ENABLED 			0X05
#define STATE_QUICK_STOP_ACTIVE 			0X06
#define STATE_FAULT_REACTION_ACTIVE 	0X07
#define STATE_FAULT 									0X08


/* Modes of Operation 0X6060 */
#define MODES_OF_OPERATION_INDEX 			0X6060
#define PROFILE_POSITION_MODE					1
#define VELOCITY_MODE									2
#define PROFILE_VELOCITY_MODE					3
#define TORQUE_PROFILE_MODE						4
#define HOMING_MODE										6
#define INTERPOLATED_POSITION_MODE		7
#define PROFILE_PWM_MODE							8	//user defined

/* Modes of Operation Display 0X6061 */
#define MODES_OF_OPERATION_DISPLAY_INDEX 0X6061
#define PROFILE_POSITION_DISPLAY_MODE					1
#define VELOCITY_DISPLAY_MODE									2
#define PROFILE_VELOCITY_DISPLAY_MODE					3
#define TORQUE_PROFILE_DISPLAY_MODE						4
#define HOMING_DISPLAY_MODE										6
#define INTERPOLATED_POSITION_DISPLAY_MODE		7

/* Motor Type 0x6402 */
#define MOTORTYPE_INDEX 						0X6402
#define MOTORTYPE_NONSTANDARDMOTOR 	0X00	//Non-standard motor
#define MOTORTYPE_BDCMOTOR 					0X01	//Phase modulated DC motor
#define MOTORTYPE_PMSMOTOR					0X02	//PM synchronous motor
#define MOTORTYPE_SPMBLMOTOR				0X0A	//Sinusoidal PM BL motor
#define MOTORTYPE_TPMBLMOTOR 				0x0b 	//Trapezoidal PM BL motor

/* CurrentLoop PID Parameters */
#define CURRENTLOOPPID_INDEX 				0XFF01
#define CURRENTLOOPPID_KCP_SUBINDEX 0X0001
#define CURRENTLOOPPID_KCI_SUBINDEX 0X0002
#define CURRENTLOOPPID_KCFF_SUBINDEX 0X0003

/* Torque */
#define TARGETTORQUE_INDEX 					0X6071
#define MAXTORQUE_INDEX							0X6072
#define MAXCURRENT_INDEX						0X6073
#define TORQUE_DEMAND_VALUE_INDEX		0X6074
#define MOTOR_RATED_CURRENT_INDEX		0X6075
#define MOTOR_RATED_TORQUE_INDEX		0X6076
#define TORQUE_ACTUAL_VALUE_INDEX		0X6077
#define CURRENT_ACTUAL_VALUE_INDEX	0X6078
#define DC_LINK_CIRQUIT_VOLTAGE_INDEX	0X6079
#define TORQUE_SLOP_INDEX						0X6087
#define TORQUE_PROFILE_TYPE_INDEX		0X6088
#define POWER_STAGE_PARAMETERS_INDEX	0X60F7
#define TORQUE_CONTROL_PARAMETERS		0X60F6

/* Velocity */
#define TARGET_VELOCITY_INDEX 				0X60FF

/* Position */
#define POSITION_DEMAND_VALUE_INDEX 	0X6062
#define POSITION_ACTUAL_VALUE_INDEX 	0X6064
#define FOLLOWING_ERROR_WINDOW_INDEX 	0X6065
#define FOLLOWING_ERROR_TIME_OUT_INDEX 0X6066
#define POSITION_WINDOW_INDEX					0X6067
#define POSITION_WINDOW_TIME_INDEX		0X6068
#define FOLLOWING_ERROR_ACTUAL_VALUE_INDEX	0X60F4
#define CONTROL_EFFORT_INDEX 					0X60FA
#define POSITION_CONTROL_PARAMETER_SET_INDEX 0X60FB
#define TARGET_POSITION_INDEX					0X607A
#define POSITION_RANG_LIMIT_INDEX			0X607B
#define HOME_OFFSET_INDEX							0X607C
#define SOFTWARE_POSITION_LIMIT				0X607D


#endif
