/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#define SERVODEV_ID 0x0061

#warning "Version hard 1.0"
#define HW_VER_H 0x0001
#define HW_VER_L 0x0002

#warning "Version soft 1.3"
#define FW_VER_H 0x0003
#define FW_VER_L 0x0009

#define FW_BUILD 0x0013
#define FW_RVSN 0x0005

#define CONFIG 65


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#ifndef __cplusplus
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif

#define CAN_SIZE  64

#define FLASH_PAGE_SIZE  1024

#define GPIO_SUN_CAN_LED_Port     GPIOE
#define GPIO_SUN_CAN_LED_Pin      GPIO_Pin_2

#define GPIO_SRV_CAN_LED_Port     GPIOE
#define GPIO_SRV_CAN_LED_Pin      GPIO_Pin_3

#define GPIO_MCU_Status_LED_Port  GPIOE
#define GPIO_MCU_Status_LED_Pin   GPIO_Pin_4


/* Exported types ------------------------------------------------------------*/
extern volatile uint32_t sunReceiveHead;
extern volatile uint32_t sunReceiveTail;
extern CanRxMsg  sunCanBox[CAN_SIZE];


extern volatile uint32_t srvReceiveHead;
extern volatile uint32_t srvReceiveTail;
extern CanRxMsg  srvCanBox[CAN_SIZE];

extern CanTxMsg  sunCanTx;
extern CanTxMsg  srvCanTx;

extern CanRxMsg *sunPtrBox;
extern CanRxMsg *srvPtrBox;

extern volatile uint32_t SUN_sourceID;
extern volatile uint32_t SUN_targetID;

extern volatile uint32_t systemTickValue;

typedef enum
{
  XY_MOVE_NONE = 0,
  XY_CONT_LEFT,
  XY_CONT_RIGHT,
  XY_DIST_LEFT,
  XY_DIST_RIGHT,
  XY_STOP
} XY_MOVE_MODES;

typedef enum
{
  servoContWrd_00 = 0x00,
  servoContWrd_04 = 0x04,
  servoContWrd_06 = 0x06,
  servoContWrd_07 = 0x07,
  servoContWrd_08 = 0x08,
  servoContWrd_80 = 0x80, //err_rst_ContW
  servoContWrd_0B = 0x0B,
  servoContWrd_4F = 0x4F,
  servoContWrd_5F = 0x5F,
  servoContWrd_8F = 0x8F,
  servoContWrd_10F = 0x10F,
  servoOpMode_00  = 0x00,
  servoOpMode_PP  = 0x01,
  servoOpMode_PV  = 0x03,
  servoOpMode_PT  = 0x04,
  servoOpMode_HM  = 0x06,
  servoOpMode_IP  = 0x07
} servoStates;

typedef struct
{
  unsigned char fAckAccDec;
  unsigned char fAckQSAcc;
  unsigned char fAckVel;
  unsigned char fAckPassedDist;
} ackFlags;

typedef struct
{
  unsigned char fAnswer_1400_1;
  unsigned char fAnswer_1401_1;
  unsigned char fAnswer_1402_1;
  unsigned char fAnswer_1403_1;
} answersFlags_140x_1;

typedef struct
{
  unsigned char fAnswer_1600_0;
  unsigned char fAnswer_1601_0;
  unsigned char fAnswer_1602_0;
  unsigned char fAnswer_1603_0;
} answersFlags_160x_0;

typedef struct
{
  unsigned char fAnswer_1800_1;
  unsigned char fAnswer_1801_1;
  unsigned char fAnswer_1802_1;
  unsigned char fAnswer_1803_1;
} answersFlags_180x_1;

typedef struct
{
  unsigned char fAnswer_1A00_0;
  unsigned char fAnswer_1A01_0;
  unsigned char fAnswer_1A02_0;
  unsigned char fAnswer_1A03_0;
} answersFlags_1A0x_0;

// состояние SRV = CLIENT_ALIVE при наличии сообщения 0x703 0x7F от SRV (0x703 = 0x700 (тип сообщения) + SRV_CAN_StdId (CAN-адрес SRV, в данном случае = 0x003)
//состояние SRV = CLIENT_DEAD при отсутствии этого сообщения через определенный период ( устанавливается в прерывании Htbt таймера (TIM_6) )
typedef enum
{
  CLIENT_DEAD = 0,
  CLIENT_ALIVE = 1
}clientStateVars;

//typedef union
//{
//  int i;
//  float f;
//}flintValues;

typedef union
{
  float f;
  unsigned char ba[4];
}float2bytearr;

typedef struct
{
  uint32_t  address;
  uint32_t  command;
  uint32_t  mcuStatus;
} TargetStruct;

typedef struct
{
  uint32_t  counter;
  uint32_t  head;
  uint32_t  tail;
  uint32_t  data[16];
} QueueStruct;

typedef struct
{
uint8_t inputNum;
uint8_t inputType;
}srvInput;

typedef enum
{
  noneFuncMode = 0x00,
  initFuncMode = 0x01,
  idleFuncMode = 0x02,
  initFaultFuncMode = 0x03,
  posMovFuncMode = 0x04,
  infMovFuncMode = 0x05,
  posQSFuncMode = 0x06,
  writeParamFuncMode = 0x07
}srvFuncMode;

#define SUN_TYPEMSG     (sunPtrBox->Data[0])
#define SUN_CHANNEL     (sunPtrBox->Data[1])
#define SUN_CMD         ( (uint16_t)sunPtrBox->Data[2]+(((uint16_t)sunPtrBox->Data[3])<<8) )
#define SUN_CMD_LOW     (sunPtrBox->Data[2])
#define SUN_CMD_HIGH    (sunPtrBox->Data[3])
#define SUN_DL0         (sunPtrBox->Data[4])
#define SUN_DL1         (sunPtrBox->Data[5])
#define SUN_DL2         (sunPtrBox->Data[6])
#define SUN_DL3         (sunPtrBox->Data[7])

#define SUN_DL32        ( (uint32_t)sunPtrBox->Data[4]+(((uint32_t)sunPtrBox->Data[5])<<8)+\
(((uint32_t)sunPtrBox->Data[6])<<16)+(((uint32_t)sunPtrBox->Data[7])<<24) )

#define SUN_DL16        ( (uint32_t)sunPtrBox->Data[4]+(((uint32_t)sunPtrBox->Data[5])<<8) )
#define SUN_DL16_L      ( (uint16_t)sunPtrBox->Data[4]+(((uint16_t)sunPtrBox->Data[5])<<8) )
#define SUN_DL16_H      ( (uint16_t)sunPtrBox->Data[6]+(((uint16_t)sunPtrBox->Data[7])<<8) )


#define SRV_TYPEMSG     (srvPtrBox->Data[0])
#define SRV_CHANNEL     (srvPtrBox->Data[1])
#define SRV_CMD         ( (uint16_t)srvPtrBox->Data[2]+(((uint16_t)srvPtrBox->Data[3])<<8) )
#define SRV_CMD_LOW     (srvPtrBox->Data[2])
#define SRV_CMD_HIGH    (srvPtrBox->Data[3])
#define SRV_DL0         (srvPtrBox->Data[4])
#define SRV_DL1         (srvPtrBox->Data[5])
#define SRV_DL2         (srvPtrBox->Data[6])
#define SRV_DL3         (srvPtrBox->Data[7])

#define SRV_DL32        ( (uint32_t)srvPtrBox->Data[4]+(((uint32_t)srvPtrBox->Data[5])<<8)+\
(((uint32_t)srvPtrBox->Data[6])<<16)+(((uint32_t)srvPtrBox->Data[7])<<24) )

#define SRV_DL16        ( (uint32_t)srvPtrBox->Data[4]+(((uint32_t)srvPtrBox->Data[5])<<8) )
#define SRV_DL16_L      ( (uint16_t)srvPtrBox->Data[4]+(((uint16_t)srvPtrBox->Data[5])<<8) )
#define SRV_DL16_H      ( (uint16_t)srvPtrBox->Data[6]+(((uint16_t)srvPtrBox->Data[7])<<8) )

#define SRV_OBJ         ( (uint16_t)srvPtrBox->Data[1]+(((uint16_t)srvPtrBox->Data[2])<<8) )
#define SRV_OBJ_LOW     (srvPtrBox->Data[1])
#define SRV_OBJ_HIGH    (srvPtrBox->Data[2])
#define SRV_SUBINDEX    (srvPtrBox->Data[3])
#define SRV_TPDO_STATE_01  ( (uint16_t)srvPtrBox->Data[0]+(((uint16_t)srvPtrBox->Data[1])<<8) )
#define SRV_TPDO_STATE_23  ( (uint16_t)srvPtrBox->Data[2]+(((uint16_t)srvPtrBox->Data[3])<<8) )


#define SRV_ANSWER_0        (srvPtrBox->Data[0])

#define SRV_ANSWER_LOW ( (uint32_t)srvPtrBox->Data[0]+(((uint32_t)srvPtrBox->Data[1])<<8)+\
(((uint32_t)srvPtrBox->Data[2])<<16)+(((uint32_t)srvPtrBox->Data[3])<<24) )

#define SRV_ANSWER_HIGH ( (uint32_t)srvPtrBox->Data[4]+(((uint32_t)srvPtrBox->Data[5])<<8)+\
(((uint32_t)srvPtrBox->Data[6])<<16)+(((uint32_t)srvPtrBox->Data[7])<<24) )


#define TX_SET(a)	sunCanTx.ExtId = a; sunCanTx.StdId = 0;
#define TX_SET1(a,b)	sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 1; sunCanTx.Data[0] = b;

#define TX_SET2(a,b,c)  sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 2;\
sunCanTx.Data[0] = b; sunCanTx.Data[1] = c;

#define TX_SET3(a,b,c,d)  sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 3;\
sunCanTx.Data[0] = b; sunCanTx.Data[1] = c; sunCanTx.Data[2] = d;

#define TX_SET4(a,b,c,d,e)  sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 4;\
sunCanTx.Data[0] = b; sunCanTx.Data[1] = c;\
  sunCanTx.Data[2] = d; sunCanTx.Data[3] = e;

#define TX_SET5(a,b,c,d,e,f)  sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 5;\
  sunCanTx.Data[0] = b; sunCanTx.Data[1] = c;\
    sunCanTx.Data[2] = d; sunCanTx.Data[3] = e;\
      sunCanTx.Data[4] = f;

#define TX_SET6(a,b,c,d,e,f,g)  sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 6;\
      sunCanTx.Data[0] = b; sunCanTx.Data[1] = c;\
        sunCanTx.Data[2] = d; sunCanTx.Data[3] = e;\
          sunCanTx.Data[4] = f; sunCanTx.Data[5] = g;

#define TX_SET7(a,b,c,d,e,f,g,h)  sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 7;\
          sunCanTx.Data[0] = b; sunCanTx.Data[1] = c;\
            sunCanTx.Data[2] = d; sunCanTx.Data[3] = e;\
              sunCanTx.Data[4] = f; sunCanTx.Data[5] = g;\
                sunCanTx.Data[6] = h;

#define TX_SET8(a,b,c,d,e,f,g,h,i) sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 8;\
                sunCanTx.Data[0] = b; sunCanTx.Data[1] = c;\
                  sunCanTx.Data[2] = d; sunCanTx.Data[3] = e;\
                    sunCanTx.Data[4] = f; sunCanTx.Data[5] = g;\
                      sunCanTx.Data[6] = h; sunCanTx.Data[7] = i;



#define SUN_TRANSFER_4(a,b,c,d,e)  {sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 4;\
                      sunCanTx.Data[0] = b; sunCanTx.Data[1] = c;\
                        sunCanTx.Data[2] = d; sunCanTx.Data[3] = e;\
                          SUN_resultTx = CAN_Transmit(CAN1, &sunCanTx);}

#define SUN_TRANSFER_5(a,b,c,d,e,f)  {sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 5;\
sunCanTx.Data[0] = b; sunCanTx.Data[1] = c;\
  sunCanTx.Data[2] = d; sunCanTx.Data[3] = e;\
    sunCanTx.Data[4] = f;\
      SUN_resultTx = CAN_Transmit(CAN1, &sunCanTx);}


#define SUN_TRANSFER_6(a,b,c,d,e,f,g)  {sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 6;\
sunCanTx.Data[0] = b; sunCanTx.Data[1] = c;\
  sunCanTx.Data[2] = d; sunCanTx.Data[3] = e;\
    sunCanTx.Data[4] = f; sunCanTx.Data[5] = g;\
      SUN_resultTx = CAN_Transmit(CAN1, &sunCanTx);}


#define SUN_TRANSFER_8(a,b,c,d,e,f,g,h,i) { sunCanTx.ExtId = a; sunCanTx.StdId = 0; sunCanTx.DLC = 8;\
sunCanTx.Data[0] = b; sunCanTx.Data[1] = c;\
  sunCanTx.Data[2] = d; sunCanTx.Data[3] = e;\
    sunCanTx.Data[4] = f; sunCanTx.Data[5] = g;\
      sunCanTx.Data[6] = h; sunCanTx.Data[7] = i;\
        SUN_resultTx = CAN_Transmit(CAN1, &sunCanTx);}

//------------------------------------------------------------------------------

#define SRV_TRANSFER_2(a,b,c)  {srvCanTx.ExtId = 0; srvCanTx.StdId = a; srvCanTx.DLC = 2;\
srvCanTx.Data[0] = b; srvCanTx.Data[1] = c;\
  SRV_resultTx = CAN_Transmit(CAN2, &srvCanTx);}

#define SRV_TRANSFER_4(a,b,c,d,e)  {srvCanTx.ExtId = 0; srvCanTx.StdId = a; srvCanTx.DLC = 4;\
srvCanTx.Data[0] = b; srvCanTx.Data[1] = c;\
  srvCanTx.Data[2] = d; srvCanTx.Data[3] = e;\
    SRV_resultTx = CAN_Transmit(CAN2, &srvCanTx);}

#define SRV_TRANSFER_5(a,b,c,d,e,f)  {srvCanTx.ExtId = 0; srvCanTx.StdId = a; srvCanTx.DLC = 5;\
srvCanTx.Data[0] = b; srvCanTx.Data[1] = c;\
  srvCanTx.Data[2] = d; srvCanTx.Data[3] = e;\
    srvCanTx.Data[4] = f;\
      SRV_resultTx = CAN_Transmit(CAN2, &srvCanTx);}


#define SRV_TRANSFER_6(a,b,c,d,e,f,g)  {srvCanTx.ExtId = 0; srvCanTx.StdId = a; srvCanTx.DLC = 6;\
srvCanTx.Data[0] = b; srvCanTx.Data[1] = c;\
  srvCanTx.Data[2] = d; srvCanTx.Data[3] = e;\
    srvCanTx.Data[4] = f; srvCanTx.Data[5] = g;\
      SRV_resultTx = CAN_Transmit(CAN2, &srvCanTx);}


#define SRV_TRANSFER_8(a,b,c,d,e,f,g,h,i) {srvCanTx.ExtId = 0; srvCanTx.StdId = a; srvCanTx.DLC = 8;\
srvCanTx.Data[0] = b; srvCanTx.Data[1] = c;\
  srvCanTx.Data[2] = d; srvCanTx.Data[3] = e;\
    srvCanTx.Data[4] = f; srvCanTx.Data[5] = g;\
      srvCanTx.Data[6] = h; srvCanTx.Data[7] = i;\
        SRV_resultTx = CAN_Transmit(CAN2, &srvCanTx);}

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
