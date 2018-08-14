/*TODO: Иногда TargetReachedBit из-за дрожания вала движка равен нулю в то время, когда приходит команда на движение из SUN-CAN,
и тогда команда игнорится.

Как правило, чуть позже вал стабилизируется и TargetReachedBit становится равным единице, но команда уже проигнорирована и движение не выполняется
(наблюдал в реверсе, когда команды движения поступают к плате сразу же после подтверждения платой о выполнении предыдущей команды
(когда TargetReachedBit = 1 в первый раз после остановки двигателя), в этот период вал движка дрожит и TargetReachedBit меняет значение между 0 и 1 несколько раз) - ПОДУМАТЬ КАК РЕШИТЬ */

// V_srv_max = 3000[round/min (rpm)] - максимальная скорость двигателя, хотя самому приводу можно задать эту скорость выше, но снизится ресурс мотора.
// 1round = 1280 000*(0x6093.2/0x6093.1) [PUU]
// 0x6093.1 & 0x6093.2 - коэффициенты электронного редуктора (Electronic Gear Ratio: 0x6093.1 = Р1-44; 0x6093.2 = Р1-45)
// при 0x6093.1 = 1 и 0x6093.2 = 1 получим V_srv_max = (3000 rpm * 1280 000 PUU) / 60 = 64 000 000 [PUU/s]
//---
// Ускорение привода Acc_srv (0x6083) - время в миллисекундах, за которое привод должен разогнаться от 0 rpm до 3000 rpm (V_srv_max)
// Торможение привода Decc_srv (0x6084) - время в миллисекундах, за которое привод должен остановиться от 3000 rpm (V_srv_max) до 0 rpm
// Ускорение привода и торможение привода задается через SUN-CAN одной величиной, поэтому они равны.
// Если заданная скорость профиля (V_prof) отличается от V_srv_max, то фактические величины ускорения привода и торможения привода будут также пропорционально отличаться.
// Т.е. при V_prof = V_srv_max/2, то фактические времена разгона и торможения будут также вдвое меньше, чем Acc_srv и Decc_srv.
// Так как мы задаем извне Acc_sun (разгон/торможение) в м/с^2, а V_sun скорость в м/с, то параметры скорости и разгона/торможения, которые мы будем писать в привод, должны быть пересчитаны.
// Таким образом, получаем: V_prof[PUU/s] = Kp[PUU/m]*V_sun[m/s], Кр зависит от механики (балка/редуктор и т.д., т.к. от передаточного числа редуктора и механики балки зависит, сколько PUU надо намотать мотору, чтобы каретка проехала заданное число метров)
// Acc_srv [milliseconds] = ((V_sun/Acc_sun)*(V_prof/V_srv_max)) * 1000 = xxxx [millseconds] {но результат (Acc_srv) - целое число (не float), поэтому в коде нужно использовать формулу Acc_srv [milliseconds] = (V_sun * V_prof * 1000) / (Acc_sun * V_srv_max), т.е. сначала всё перемножаем, потом уже делим}


/* ! ! ! =======================================================================
Параметр команды 0x607A (Target Position) - 2х байтный, при больших значениях расстояния, задаваемых через SUN-CAN команду (SUN_CMD_DISTANCE_MOVE - 0x22), при пересчёте метров (SUN-CAN) в PUU (единицы ASDA-CANOpen) через коэффициент Kp, при определенных значениях Кр (больших, зависит от механики) может получится результат, выходящий за размер 2х байтов
В этом случае привод или даст ошибку или начнёт бесконечное движение.  Для избежания этой ситуации нужно использовать коэффициенты электронной редукции привода (у ASDA это Obj.0x6093.1 (Р1-44 - Electronic Gear Numerator) и Obj.0x6093.2 (Р1-45 - Electronic Gear Denominator))
Величина значений этих параметров выбирается таким образом, чтобы обеспечить достаточную точность. Т.е., например, 1 PUU = 0.5 мкм перемещения механики (каретка/портал/лента)

Если srvDefGearNum = 1 и srvDefGearFeed = 1, то:
Кр = 1280000 - двигатель делает один оборот ротора,
Кр = 44131387 - коэффициент, при котором каретка проходит 1 метр на балке Rexroth через редуктор 7Ч-М-50-10-ПЦ24 от Приводной Техники, если srvDefGearNum = 1 и srvDefGearFeed = 1.

!!! При этом переполнение параметра 0x607A наступит уже при задании дистанции около 50м через SUN-CAN команду (SUN_CMD_DISTANCE_MOVE - 0x22)

---*---
Если srvDefGearNum = 22 и srvDefGearFeed = 1, то:
Kp_new для сохранения прежних условий должен быть 44131388/22 = 2005972,18.

!!! При этом переполнение параметра 0x607A наступит уже при более высоком значении дистанции (около 1000м) через SUN-CAN команду (SUN_CMD_DISTANCE_MOVE - 0x22)

!! Вообще Кр(srvDefGearNum, srvDefGearFeed) = (Кр(1,1) * srvDefGearFeed)/srvDefGearNum !!
========================================================================! ! ! */


#include "main.h"
#include "SUN_CAN.h"
#include "CAN_SRV_Converter.h"
#include "string.h"
#include "stm32f4xx_flash.h"
#include "ringbuffer.h"


RCC_ClocksTypeDef RCC_Clocks;

uint32_t  moduleAddress;

volatile uint32_t systemTickValue = 0;

volatile uint32_t sourceId, sourceId_POS_MOVE, sourceId_INF_MOVE, sourceId_Coeff, sourceId_PosVel, sourceId_QSAcc, sourceId_AccDec, sourceId_QSCmd, sourceId_HaltCmd;
volatile uint32_t targetID;

uint32_t msgCanOpenId, srvCanOpenId = 0x3;
volatile uint32_t srvTargetId;

volatile uint32_t sunReceiveHead;
volatile uint32_t sunReceiveTail;
CanRxMsg  sunCanBox[CAN_SIZE];
CanRxMsg *sunPtrBox;

volatile uint32_t srvReceiveHead;
volatile uint32_t srvReceiveTail;
CanRxMsg  srvCanBox[CAN_SIZE];
CanRxMsg *srvPtrBox;

CanTxMsg sunCanTx;
uint8_t SUN_resultTx;

CanTxMsg srvCanTx;
uint8_t SRV_resultTx;

volatile uint32_t mcuStatusLedTimer;
volatile uint32_t sunCanLedTimer;
volatile uint32_t srvCanLedTimer;

volatile uint8_t currentMcuLedState;

volatile uint8_t initStep;
volatile uint8_t procStep;

uint8_t srvCurrFuncMode;
volatile uint8_t srvPrevFuncMode;
volatile uint32_t srvCurrOperationMode;
volatile uint32_t srvCurrControlWord;
volatile uint32_t srvCurrStatusWord;
volatile uint32_t srvPrevStatusWord;
volatile uint32_t srvCurrAcc;
volatile uint32_t srvCurrDec;
volatile uint32_t srvCurrQuickStopDec;
volatile uint32_t srvCurrVel;
volatile uint32_t srvCurrTargetPos;
volatile int32_t srvPrevActualPos;
volatile int32_t srvCurrActualPos;
volatile int32_t srvPassedDist;
volatile uint32_t srvCurrTpdoInit;
volatile uint8_t srvCurrPosMoveState;

srvInput srvInputs[3]; // номера входов в приводе, их типы (NO/NC) и текущее состояние. Индексы массива 0, 1 и 2 соответствуют входам EMG, RL, LL.

volatile uint16_t srvCurrDIState;
volatile uint16_t srvPrevDIState;

volatile uint32_t srvDefOM;
volatile uint32_t srvDefCW;

volatile uint32_t srvDefAcc;
volatile uint32_t srvDefDec;
volatile uint32_t srvDefQuickStopDec;

volatile uint32_t srvDefVel;
volatile uint32_t srvDefTargetPos;

volatile uint32_t srvDefGearNum;
volatile uint32_t srvDefGearFeed;

volatile uint32_t srvMaxVel;

/* признак необходимости перезапроса параметра после ответа привода TM_SRV_60 */
uint32_t fNeedToGetParamValue;

///* объединение (union) для конвертации [float <-> int] без лишних заморочек */
//flintValues flintConvert;

/* объединение (union) для конвертации [float <-> byte[4]] без лишних заморочек */
float2bytearr f2bConvert;

clientStateVars srvHtbtState; // см. детали в объявлении (main.h)


/* 0й бит  (ReadyToSwitchOn) в ответе о состоянии (статусе) SRV (0x6041) */
unsigned char srvReadyToSwitchOnBit;

/* 5й бит (QuickStop) в ответе о состоянии (статусе) SRV (0x6041) */
unsigned char srvCurrQSBit;

/* 10й бит (Target Reached) в ответе о состоянии (статусе) SRV (0x6041) */
unsigned char srvCurrTargetReachedBit;
unsigned char srvPrevTargetReachedBit;

/* 12й бит  (Set Point Acknowledge) в ответе о состоянии (статусе) SRV (0x6041) */
unsigned char srvSetPointAcknowledgeBit;

answersFlags_140x_1 srvAnswersFlags_140x_1;
answersFlags_160x_0 srvAnswersFlags_160x_0;

answersFlags_180x_1 srvAnswersFlags_180x_1;
answersFlags_1A0x_0 srvAnswersFlags_1A0x_0;

unsigned char srvTPDO1AnswerFlag;
unsigned char srvTPDO2AnswerFlag;
unsigned char srvTPDO3AnswerFlag;

unsigned char srvSyncCounter;

bufStruct inMsgServoBuff;
bufStruct outMsgServoBuff;

unsigned char readBufDataResult;
unsigned char srvSkipBuffReadingFlag;

uint32_t srvEmgAnswer;

//***********#######@@@@@@*************
int32_t tmpDist = 581810/128;
uint32_t tmpSpeed = 0x40302010;
uint32_t tmpSpeedVal = 0;
//***********#######@@@@@@*************


//при выполнении QuickStop правильно настроенный привод должен вернуть
// 1. srvCurrQSBit (5й бит (QuickStop) в ответе о состоянии (статусе) SRV (0x6041)) в TPDO #сообщении при изменении этого бита (при этом нулевой бит в srvQSCurrStateFlag установится в единицу)
// 2. после посылки команды выполнения (controlWord_0B) привод в случае успешного выполнения вернёт подтверждение в виде 60 40 60 00 00 00 00 00 ,
// для проверки, что текущий режим именно такой (0x0B), отправляем запрос чтения текущего режима в виде 40 40 60 00 00 00 00 00.
// В ответ привод должен ответить 43 60 40 00 0B 00 00 00 - тогда в этом случае (если 0x0B) первый бит в srvQSCurrStateFlag установится в единицу
// Если srvQSCurrStateFlag = 0x3 (оба бита srvQSCurrStateFlag равны единице), то высылаем подтверждение выполнения команды QuickStop
volatile uint8_t srvQSCurrStateFlag;


#pragma section = "FLASH"
#pragma location = "FLASH"
__root const float flashKp = 2005972.18f;
// Если srvDefGearNum = 1 и srvDefGearFeed = 1, то:
// Кр = 1280000 - двигатель делает один оборот ротора,
// Кр = 44131387 - коэффициент, при котором каретка проходит 1 метр на балке Rexroth через редуктор 7Ч-М-50-10-ПЦ24 от Приводной Техники, если srvDefGearNum = 1 и srvDefGearFeed = 1.

// Если srvDefGearNum = 22 и srvDefGearFeed = 1, то:
// Kp_new для сохранения прежних условий должен быть 44131388/22 = 2005972,18.
// !! Вообще Кр(srvDefGearNum, srvDefGearFeed) = (Кр(1,1) * srvDefGearFeed)/srvDefGearNum !!


/* Коэффициент пересчета из единиц SUN_CAN в единицы ASDA */
volatile float *Kp;

volatile float sunDefAcc;
volatile float sunDefQuickStopAcc;
volatile float sunDefVel;
volatile float sunDefTargetPos;

volatile float sunCurrAcc;
volatile float sunCurrQuickStopAcc;
volatile float sunCurrVel;
volatile float sunCurrTargetPos;

/* флаги получения команд с подтверждением к исполнению и с подтверждением об окончании выполнения */
ackFlags sunCmdAckFlags;


/* Private function prototypes -----------------------------------------------*/
static void NVIC_Config(void);
static void GPIO_Config(void);
static void CAN_Config(void);

void initData(void);
void initServo(srvFuncMode srvCurrMode);

void srvSendCmd(bufStruct srvCmdData);

void Htbt_timer_init(void);
void Htbt_timer_start_ms(uint32_t n_msec);
uint32_t  setTimeout(void);
uint32_t  passedTimeout(uint32_t time, uint32_t limit);


uint32_t WriteFlashSector(uint32_t sector, uint32_t addr, uint32_t data);

/*** Main ***/
int main(void)
{     
  uint32_t sunDummy, srvDummy;
  uint32_t timerSunCan = 0, timerSrvCan = 0, timerCanBus = 0, timerNodeRst = 0, timerErrRstCntw = 0;
  
  /*!< At this stage the microcontroller clock setting is already configured,
  this is done through SystemInit() function which is called from startup
  files before to branch to application main.
  To reconfigure the default setting of SystemInit() function,
  refer to system_stm32f4xx.c file */
  
  RCC_GetClocksFreq(&RCC_Clocks);
  
  /* Setup SysTick Timer for 1 msec interrupts */
  if (SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000))
  {
    while(1);
  }
  
  GPIO_Config();
  
  /* NVIC configuration */
  NVIC_Config();
  
  /* CAN configuration */
  CAN_Config();
  
  sunCanTx.IDE = CAN_ID_EXT;
  sunCanTx.RTR = CAN_RTR_DATA;
  
  srvCanTx.IDE = CAN_ID_STD;
  srvCanTx.RTR = CAN_RTR_DATA;
  
  initData();
  
  initBuff();
  
  //Htbt_timer_init();
  
  uint8_t asc = 0;
  
  /* Infinite loop */
  while(1)
  {
    if(passedTimeout(timerCanBus, 3000))
    {
      timerCanBus = setTimeout();
      SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|ID_BROADCAST, TM_STATE_4, CH_COMMON_FE, (uint8_t)EC_INFO, (uint8_t)(EC_INFO>>8), (uint8_t)SERVODEV_ID, (uint8_t)((HW_VER_H<<4)|(HW_VER_L)), (uint8_t)((FW_VER_H<<4)|(FW_VER_L)), INFO_CAN_BUS);
    }
    
    // == < сброс СУ при нажатии кнопки TEST на плате > ==
    if((!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)) && (asc == 0))
    {
      initData();
      
      SRV_TRANSFER_2(0x00, 0x81, 0x03);
      asc = 1;
    }
    
    if((GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)) && (asc == 1))
    {
      asc = 0;
    }// == </ сброс СУ при нажатии кнопки TEST на плате > ==
    
    // == < Мигаем светодиодами статуса MCU_Status, SUN-CAN, SRV-CAN > ==
    if(passedTimeout(mcuStatusLedTimer, 750))
    {
      mcuStatusLedTimer = setTimeout();
      
      GPIO_ToggleBits(GPIO_MCU_Status_LED_Port, GPIO_MCU_Status_LED_Pin);       // ВКЛ/ВЫКЛ светодиод сигнализации об отправке/получении SUN_CAN сообщения при ПРИЕМЕ SUN_CAN сообщения
    }
    
    if(passedTimeout(sunCanLedTimer, 30))
    {
      GPIO_WriteBit(GPIO_SUN_CAN_LED_Port, GPIO_SUN_CAN_LED_Pin, (BitAction)0); // ВЫКЛ светодиод сигнализации об отправке/получении SUN_CAN сообщения при ПРИЕМЕ SUN_CAN сообщения
    }
    
    if(passedTimeout(srvCanLedTimer, 30))
    {
      GPIO_WriteBit(GPIO_SRV_CAN_LED_Port, GPIO_SRV_CAN_LED_Pin, (BitAction)0); // ВЫКЛ светодиод сигнализации об отправке/получении SRV_CAN сообщения при ПРИЕМЕ SRV_CAN сообщения
    }// == </ Мигаем светодиодами статуса MCU_Status, SUN-CAN, SRV-CAN > ==
    
    
    /*** SUN-CAN Message Handler ***/
    timerSunCan = setTimeout();
    while( !passedTimeout(timerSunCan, 6) )
    {
      __disable_irq();
      sunDummy = sunReceiveHead;
      __enable_irq();
      
      if(sunDummy == sunReceiveTail) break;
      sunPtrBox = &sunCanBox[sunReceiveTail];
      sourceId = (sunPtrBox->ExtId>>13)&0x00001FFF;
      targetID = (sunPtrBox->ExtId)&0x00001FFF;
      
      switch(SUN_TYPEMSG)
      {
      case TM_WRITE_4:
      case TM_WRITE_ACK_4:
        {
          switch(SUN_CMD)
          {
          case SUN_POS_DEF_VEL: // пишем в привод скорость перемещения через SUN-CAN
            {
              f2bConvert.ba[0] = sunPtrBox->Data[4];
              f2bConvert.ba[1] = sunPtrBox->Data[5];
              f2bConvert.ba[2] = sunPtrBox->Data[6];
              f2bConvert.ba[3] = sunPtrBox->Data[7];
              
              sunDefVel = f2bConvert.f;
              
              srvDefVel = (uint32_t)((*Kp) * sunDefVel);
              
              srvDefAcc = (uint32_t)((srvDefGearFeed * sunDefVel * (float)srvMaxVel * 1000) / (srvDefGearNum * sunDefAcc * (float)srvDefVel)); // пересчёт значения ускорения и торможения в единицы ASDA (в миллисекунды) исходя из НОВОГО значения SUN_Speed
              
              srvDefDec = srvDefAcc;
              
              srvDefQuickStopDec = (uint32_t)((srvDefGearFeed * sunDefVel * (float)srvMaxVel * 1000) / (srvDefGearNum * sunDefQuickStopAcc * (float)srvDefVel));
              
              inMsgServoBuff.arr[0] = 0x03;
              inMsgServoBuff.arr[1] = srvCanOpenId;
              inMsgServoBuff.arr[2] = (uint8_t)(srvDefVel & 0xFF);
              inMsgServoBuff.arr[3] = (uint8_t)((srvDefVel & 0xFF00) >> 8);
              inMsgServoBuff.arr[4] = (uint8_t)((srvDefVel & 0xFF0000) >> 16);
              inMsgServoBuff.arr[5] = (uint8_t)((srvDefVel & 0xFF000000) >> 24);
              inMsgServoBuff.arr[6] = (uint8_t)(srvDefQuickStopDec & 0xFF);
              inMsgServoBuff.arr[7] = (uint8_t)((srvDefQuickStopDec & 0xFF00) >> 8);
              inMsgServoBuff.arr[8] = (uint8_t)((srvDefQuickStopDec & 0xFF0000) >> 16);
              inMsgServoBuff.arr[9] = (uint8_t)((srvDefQuickStopDec & 0xFF000000) >> 24);              
              
              if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
              {
                __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
              }
              
              
              inMsgServoBuff.arr[0] = 0x04;
              inMsgServoBuff.arr[1] = srvCanOpenId;
              inMsgServoBuff.arr[2] = (uint8_t)(srvDefAcc & 0xFF);
              inMsgServoBuff.arr[3] = (uint8_t)((srvDefAcc & 0xFF00) >> 8);
              inMsgServoBuff.arr[4] = (uint8_t)((srvDefAcc & 0xFF0000) >> 16);
              inMsgServoBuff.arr[5] = (uint8_t)((srvDefAcc & 0xFF000000) >> 24);
              inMsgServoBuff.arr[6] = (uint8_t)(srvDefDec & 0xFF);
              inMsgServoBuff.arr[7] = (uint8_t)((srvDefDec & 0xFF00) >> 8);
              inMsgServoBuff.arr[8] = (uint8_t)((srvDefDec & 0xFF0000) >> 16);
              inMsgServoBuff.arr[9] = (uint8_t)((srvDefDec & 0xFF000000) >> 24);
              
              if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
              {
                __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
              }
              
              SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_PosVel, TM_ANSWER_4, CH_X, (uint8_t)SUN_POS_DEF_VEL, (uint8_t)(SUN_POS_DEF_VEL>>8),  (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));              
            }
            break;
            
          case SUN_POS_DEF_ACC_DEC: // пишем в привод ускорение/торможение через SUN-CAN
            {
              f2bConvert.ba[0] = sunPtrBox->Data[4];
              f2bConvert.ba[1] = sunPtrBox->Data[5];
              f2bConvert.ba[2] = sunPtrBox->Data[6];
              f2bConvert.ba[3] = sunPtrBox->Data[7];
              
              sunDefAcc = f2bConvert.f;
              
              srvDefAcc = (uint32_t)((srvDefGearFeed * /*toConfig->*/sunDefVel * (float)srvMaxVel * 1000) / (srvDefGearNum * sunDefAcc * (float)srvDefVel));
              
              srvDefDec = srvDefAcc;
              
              inMsgServoBuff.arr[0] = 0x04;
              inMsgServoBuff.arr[1] = srvCanOpenId;
              inMsgServoBuff.arr[2] = (uint8_t)(srvDefAcc & 0xFF);
              inMsgServoBuff.arr[3] = (uint8_t)((srvDefAcc & 0xFF00) >> 8);
              inMsgServoBuff.arr[4] = (uint8_t)((srvDefAcc & 0xFF0000) >> 16);
              inMsgServoBuff.arr[5] = (uint8_t)((srvDefAcc & 0xFF000000) >> 24);
              inMsgServoBuff.arr[6] = (uint8_t)(srvDefDec & 0xFF);
              inMsgServoBuff.arr[7] = (uint8_t)((srvDefDec & 0xFF00) >> 8);
              inMsgServoBuff.arr[8] = (uint8_t)((srvDefDec & 0xFF0000) >> 16);
              inMsgServoBuff.arr[9] = (uint8_t)((srvDefDec & 0xFF000000) >> 24);
              
              if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
              {
                __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
              }              
              
              SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_AccDec, TM_ANSWER_4, CH_X, (uint8_t)SUN_POS_DEF_ACC_DEC, (uint8_t)(SUN_POS_DEF_ACC_DEC>>8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));              
            }
            break;
            
          case SUN_POS_DEF_QS_ACC: // пишем в привод ускорение/торможение через SUN-CAN
            {
              f2bConvert.ba[0] = sunPtrBox->Data[4];
              f2bConvert.ba[1] = sunPtrBox->Data[5];
              f2bConvert.ba[2] = sunPtrBox->Data[6];
              f2bConvert.ba[3] = sunPtrBox->Data[7];
              
              sunDefQuickStopAcc = f2bConvert.f;
              
              srvDefQuickStopDec = (uint32_t)((srvDefGearFeed * /*toConfig->*/sunDefVel * (float)srvMaxVel * 1000) / (srvDefGearNum * /*toConfig->*/sunDefQuickStopAcc * (float)srvDefVel));
              
              inMsgServoBuff.arr[0] = 0x03;
              inMsgServoBuff.arr[1] = 0x03;
              inMsgServoBuff.arr[2] = (uint8_t)(srvDefVel & 0xFF);
              inMsgServoBuff.arr[3] = (uint8_t)((srvDefVel & 0xFF00) >> 8);
              inMsgServoBuff.arr[4] = (uint8_t)((srvDefVel & 0xFF0000) >> 16);
              inMsgServoBuff.arr[5] = (uint8_t)((srvDefVel & 0xFF000000) >> 24);
              inMsgServoBuff.arr[6] = (uint8_t)(srvDefQuickStopDec & 0xFF);
              inMsgServoBuff.arr[7] = (uint8_t)((srvDefQuickStopDec & 0xFF00) >> 8);
              inMsgServoBuff.arr[8] = (uint8_t)((srvDefQuickStopDec & 0xFF0000) >> 16);
              inMsgServoBuff.arr[9] = (uint8_t)((srvDefQuickStopDec & 0xFF000000) >> 24);
              
              if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
              {
                __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
              }               
              
              SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_QSAcc, TM_ANSWER_4, CH_X, (uint8_t)SUN_POS_DEF_QS_ACC, (uint8_t)(SUN_POS_DEF_QS_ACC>>8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));
            }
            break;
            
          case SUN_K_COEFF: // TODO: запись во флэш
            {
              f2bConvert.ba[0] = sunPtrBox->Data[4];
              f2bConvert.ba[1] = sunPtrBox->Data[5];
              f2bConvert.ba[2] = sunPtrBox->Data[6];
              f2bConvert.ba[3] = sunPtrBox->Data[7];
              
              //              if(WriteFlashSector(FLASH_Sector_2, (uint32_t)&flashKp, *(uint32_t*)(&f2bConvert.f)) == TRUE)
              //              {
              
              float tmpK = f2bConvert.f;
              
              Kp = &tmpK;
              
              // Kp = (float*)&flashKp;
              
              // printf("Kp = %.2f \r\n", Kp);
              
              // Пересчёт со вновь записанным значением коэффициента и отправка в привод новых значений srvTargetPos, srvVel, srvAcc, srvDecc и srvQuickStopAcc.
              // При этом sunTargetPos, sunVel, sunAcc и sunQuickStopAcc сохраняем неизменными. Величины srvXXX пересчитываются и записываются относительно неизменившихся величин sunYYY с учетом нового Kp.
              srvDefVel = (uint32_t)((*Kp) * sunDefVel);
              
              srvDefAcc = (uint32_t)((srvDefGearFeed * sunDefVel * (float)srvMaxVel * 1000) / (srvDefGearNum * sunDefAcc * (float)srvDefVel)); // пересчёт значения ускорения и торможения в единицы ASDA (в миллисекунды) исходя из НОВОГО значения SUN_Speed
              
              srvDefDec = srvDefAcc;
              
              srvDefQuickStopDec = (uint32_t)((srvDefGearFeed * sunDefVel * (float)srvMaxVel * 1000) / (srvDefGearNum * sunDefQuickStopAcc * (float)srvDefVel));
              
              SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_Coeff, TM_ANSWER_4, CH_X,  (uint8_t)SUN_K_COEFF, (uint8_t)(SUN_K_COEFF>>8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));
              
              inMsgServoBuff.arr[0] = 0x03;
              inMsgServoBuff.arr[1] = srvCanOpenId;
              inMsgServoBuff.arr[2] = (uint8_t)(srvDefVel & 0xFF);
              inMsgServoBuff.arr[3] = (uint8_t)((srvDefVel & 0xFF00) >> 8);
              inMsgServoBuff.arr[4] = (uint8_t)((srvDefVel & 0xFF0000) >> 16);
              inMsgServoBuff.arr[5] = (uint8_t)((srvDefVel & 0xFF000000) >> 24);
              inMsgServoBuff.arr[6] = (uint8_t)(srvDefQuickStopDec & 0xFF);
              inMsgServoBuff.arr[7] = (uint8_t)((srvDefQuickStopDec & 0xFF00) >> 8);
              inMsgServoBuff.arr[8] = (uint8_t)((srvDefQuickStopDec & 0xFF0000) >> 16);
              inMsgServoBuff.arr[9] = (uint8_t)((srvDefQuickStopDec & 0xFF000000) >> 24);              
              
              if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
              {
                __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
              }
              
              
              inMsgServoBuff.arr[0] = 0x04;
              inMsgServoBuff.arr[1] = srvCanOpenId;
              inMsgServoBuff.arr[2] = (uint8_t)(srvDefAcc & 0xFF);
              inMsgServoBuff.arr[3] = (uint8_t)((srvDefAcc & 0xFF00) >> 8);
              inMsgServoBuff.arr[4] = (uint8_t)((srvDefAcc & 0xFF0000) >> 16);
              inMsgServoBuff.arr[5] = (uint8_t)((srvDefAcc & 0xFF000000) >> 24);
              inMsgServoBuff.arr[6] = (uint8_t)(srvDefDec & 0xFF);
              inMsgServoBuff.arr[7] = (uint8_t)((srvDefDec & 0xFF00) >> 8);
              inMsgServoBuff.arr[8] = (uint8_t)((srvDefDec & 0xFF0000) >> 16);
              inMsgServoBuff.arr[9] = (uint8_t)((srvDefDec & 0xFF000000) >> 24);
              
              if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
              {
                __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
              }
              
              //              }
            }
            break;
          } //~switch(SUN_CMD)
        }
        break; //~TM_WRITE_4, TM_WRITE_ACK_4
        
      case TM_READ_4:
        {
          switch(SUN_CMD)
          {
          case AD_CONFIG:
            if( SUN_CHANNEL == CH_COMMON_FE )
            {
              SUN_DL0 = (uint8_t)CONFIG;
              SUN_DL1 = (uint8_t)(CONFIG << 8);
              SUN_DL2 = (uint8_t)(CONFIG << 16);
              SUN_DL3 = (uint8_t)(CONFIG << 24);              
              
              SUN_TRANSFER_8( PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId, TM_ANSWER_4, SUN_CHANNEL, SUN_CMD_LOW, SUN_CMD_HIGH, SUN_DL0, SUN_DL1, SUN_DL2, SUN_DL3 );
            }
            break;
            
          case AD_HARDWARE:
            if( SUN_CHANNEL == CH_COMMON_FE )
            {
              SUN_DL0 = (uint8_t)HW_VER_L;
              SUN_DL1 = (uint8_t)(HW_VER_L << 8);
              SUN_DL2 = (uint8_t)HW_VER_H;
              SUN_DL3 = (uint8_t)(HW_VER_H << 8);
              
              SUN_TRANSFER_8( PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId, TM_ANSWER_4, SUN_CHANNEL, SUN_CMD_LOW, SUN_CMD_HIGH, SUN_DL0, SUN_DL1, SUN_DL2, SUN_DL3 );
            }
            break;
            
          case AD_EXTENDED_FIRMWARE:
            if( SUN_CHANNEL == CH_COMMON_FE )
            {
              SUN_DL0 = (uint8_t)FW_BUILD;
              SUN_DL1 = (uint8_t)(FW_BUILD << 8);
              SUN_DL2 = (uint8_t)FW_RVSN;
              SUN_DL3 = (uint8_t)(FW_RVSN << 8);
              
              SUN_TRANSFER_8( PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId, TM_ANSWER_4, SUN_CHANNEL, SUN_CMD_LOW, SUN_CMD_HIGH, SUN_DL0, SUN_DL1, SUN_DL2, SUN_DL3 );
            }
            break;
            
          case AD_DETAILED_FIRMWARE:
            if( SUN_CHANNEL == CH_COMMON_FE )
            {
              SUN_DL0 = (uint8_t)FW_VER_L;
              SUN_DL1 = (uint8_t)(FW_VER_L << 8);
              SUN_DL2 = (uint8_t)FW_VER_H;
              SUN_DL3 = (uint8_t)(FW_VER_H << 8);
              
              SUN_TRANSFER_8( PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId, TM_ANSWER_4, SUN_CHANNEL, SUN_CMD_LOW, SUN_CMD_HIGH, SUN_DL0, SUN_DL1, SUN_DL2, SUN_DL3 );
            }
            break;
            
          case AD_INFO:
            if(( SUN_CHANNEL == CH_COMMON_FE ) || ( SUN_CHANNEL == CH_COMMON_00 ))
            {
              SUN_DL0 = (uint8_t)SERVODEV_ID;
              SUN_DL1 = (uint8_t)((HW_VER_H << 4)|(HW_VER_L));
              SUN_DL2 = (uint8_t)((FW_VER_H << 4)|(FW_VER_L));
              SUN_DL3 = INFO_PRIVATE;
              
              SUN_TRANSFER_8( PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId, TM_ANSWER_4, SUN_CHANNEL, SUN_CMD_LOW, SUN_CMD_HIGH, SUN_DL0, SUN_DL1, SUN_DL2, SUN_DL3 );
            }
            break;
            
            //****< TEMP >****
          case 0x1004: // maximum Acc/Decc
            {
              SUN_DL0 = 0x00;
              SUN_DL1 = 0x00;
              SUN_DL2 = 0xA0;
              SUN_DL3 = 0x40;
              
              SUN_TRANSFER_8( PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId, TM_ANSWER_4, SUN_CHANNEL, SUN_CMD_LOW, SUN_CMD_HIGH, SUN_DL0, SUN_DL1, SUN_DL2, SUN_DL3 );
            }
            break;
            
          case 0x0008: // currentPos in pulses
            {
              SUN_DL0 = 0x00;
              SUN_DL1 = 0x00;
              SUN_DL2 = 0x00;
              SUN_DL3 = 0x00;
              
              SUN_TRANSFER_8( PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId, TM_ANSWER_4, SUN_CHANNEL, SUN_CMD_LOW, SUN_CMD_HIGH, SUN_DL0, SUN_DL1, SUN_DL2, SUN_DL3 );
            }
            break;
            //****</ TEMP >****
            
          case SUN_POS_CUR_COORD_M:
            {
              // если это координата привода или его пройденный путь на текущий момент, то
              // add SYNC msg in queue and set needCurrentCoordinateAnswerFlag to send answer on according TPDO #answer for SYNC message 
              // иначе отсылать значение, насчитанное платой со внешнего энкодера, подключенного к ней
            }
            break;
            
          case SUN_POS_DEF_VEL: // запрос чтения заданной скорости привода в пересчете в [м/с]
            {
              sourceId_PosVel = sourceId;
              
              //add SYNC msg in queue and set needVelocityAnswerFlag to send answer on according TPDO #answer for SYNC message
              sunCmdAckFlags.fAckVel = 1;
              
              inMsgServoBuff.arr[0] = 0x00;
              inMsgServoBuff.arr[1] = 0x80;
              inMsgServoBuff.arr[2] = 0x00;
              inMsgServoBuff.arr[3] = 0x00;
              inMsgServoBuff.arr[4] = 0x00;
              inMsgServoBuff.arr[5] = 0x00;
              inMsgServoBuff.arr[6] = 0x00;
              inMsgServoBuff.arr[7] = 0x00;
              inMsgServoBuff.arr[8] = 0x00;
              inMsgServoBuff.arr[9] = 0x00;
              
              if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
              {
                __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
              }                           
            }
            break;
            
          case SUN_POS_DEF_ACC_DEC:
            {
              sourceId_AccDec = sourceId; 
              
              //add SYNC msg in queue and set needAccDecAnswerFlag to send answer on according TPDO #answer for SYNC message
              sunCmdAckFlags.fAckAccDec = 1;
              
              inMsgServoBuff.arr[0] = 0x00;
              inMsgServoBuff.arr[1] = 0x80;
              inMsgServoBuff.arr[2] = 0x00;
              inMsgServoBuff.arr[3] = 0x00;
              inMsgServoBuff.arr[4] = 0x00;
              inMsgServoBuff.arr[5] = 0x00;
              inMsgServoBuff.arr[6] = 0x00;
              inMsgServoBuff.arr[7] = 0x00;
              inMsgServoBuff.arr[8] = 0x00;
              inMsgServoBuff.arr[9] = 0x00;
              
              if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
              {
                __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
              }              
            }
            break;
            
          case SUN_POS_DEF_QS_ACC:
            {
              sourceId_QSAcc = sourceId;
              
              //add SYNC msg in queue and set needQSAccAnswerFlag to send answer on according TPDO #answer for SYNC message
              sunCmdAckFlags.fAckQSAcc = 1;
              
              inMsgServoBuff.arr[0] = 0x00;
              inMsgServoBuff.arr[1] = 0x80;
              inMsgServoBuff.arr[2] = 0x00;
              inMsgServoBuff.arr[3] = 0x00;
              inMsgServoBuff.arr[4] = 0x00;
              inMsgServoBuff.arr[5] = 0x00;
              inMsgServoBuff.arr[6] = 0x00;
              inMsgServoBuff.arr[7] = 0x00;
              inMsgServoBuff.arr[8] = 0x00;
              inMsgServoBuff.arr[9] = 0x00;
              
              if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
              {
                __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
              }
            }
            break;
            
          case SUN_K_COEFF:
            {
              sourceId_Coeff = sourceId;
              
              f2bConvert.f = (*Kp);
              
              SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_Coeff, TM_ANSWER_4, CH_X, (uint8_t)SUN_K_COEFF, (uint8_t)(SUN_K_COEFF>>8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));
            }
            break;
            
          } //</switch(SUN_CMD)>
        }
        break;
        
      case TM_READ_2:
        {
          switch(SUN_CMD)
          {
            /* Инверсия !(uint8_t)((srvCurrDIState & (1<<i)) >> i) фактического состояния входа привода (ASDA), получаемого платой, нужна согласно протоколу SUNCAN.
            /* Т.е. ASDA CanOpen "Input On" = 1, тогда как в SUNCAN физический "Physical Input On" = 0 (соответственно, ASDA CanOpen "Input Off" = 0, тогда как в SUNCAN физический "Physical Input Off" = 1)
            /* Логическое состояние LL, RL и EMG в SUNCAN зависит от заданного типа датчика/входа - если это N.O., то SUNCAN "Logic Input On" = !"Physical Input On" (инвертируем), если же датчик N.C., то SUNCAN "Logic Input On" = "Physical Input On".
            /* У ASDA тип входа (P2-10 и.т.д.) srvCurrDIType[i] = 1 - N.O., srvCurrDIType[i] == 0 - N.C. */
          case SUN_GET_LL_STATE:
            if( SUN_CHANNEL == CH_X )
            {
              SUN_DL0 = !(uint8_t)((srvCurrDIState & (1 << (srvInputs[2].inputNum - 1))) >> (srvInputs[2].inputNum - 1));
              SUN_DL1 = (srvInputs[2].inputType == 1) ? ((uint8_t)((srvCurrDIState & (1 << (srvInputs[2].inputNum - 1))) >> (srvInputs[2].inputNum - 1))):(!(uint8_t)((srvCurrDIState & (1 << (srvInputs[2].inputNum - 1))) >> (srvInputs[2].inputNum - 1)));
              
              SUN_TRANSFER_6( PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId, TM_ANSWER_2, SUN_CHANNEL, SUN_CMD_LOW, SUN_CMD_HIGH, SUN_DL0, SUN_DL1 );
            }
            break;
            
          case SUN_GET_RL_STATE:
            if( SUN_CHANNEL == CH_X )
            {
              SUN_DL0 = !(uint8_t)((srvCurrDIState & (1 << (srvInputs[1].inputNum - 1))) >> (srvInputs[1].inputNum - 1));
              SUN_DL1 = (srvInputs[1].inputType == 1) ? ((uint8_t)((srvCurrDIState & (1 << (srvInputs[1].inputNum - 1))) >> (srvInputs[1].inputNum - 1))):(!(uint8_t)((srvCurrDIState & (1 << (srvInputs[1].inputNum - 1))) >> (srvInputs[1].inputNum - 1)));
              
              SUN_TRANSFER_6( PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId, TM_ANSWER_2, SUN_CHANNEL, SUN_CMD_LOW, SUN_CMD_HIGH, SUN_DL0, SUN_DL1 );
            }
            break;
            
            
          case SUN_GET_STP_STATE:
            if( SUN_CHANNEL == CH_X )
            {
              SUN_DL0 = !(uint8_t)((srvCurrDIState & (1 << (srvInputs[0].inputNum - 1))) >> (srvInputs[0].inputNum - 1));
              SUN_DL1 = (srvInputs[0].inputType == 1) ? ((uint8_t)((srvCurrDIState & (1 << (srvInputs[0].inputNum - 1))) >> (srvInputs[0].inputNum - 1))):(!(uint8_t)((srvCurrDIState & (1 << (srvInputs[0].inputNum - 1))) >> (srvInputs[0].inputNum - 1)));
              
              SUN_TRANSFER_6( PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId, TM_ANSWER_2, SUN_CHANNEL, SUN_CMD_LOW, SUN_CMD_HIGH, SUN_DL0, SUN_DL1 );
            }
            break;
          }//</switch(SUN_CMD)>
        }
        break;
        
      case TM_CMD_6:
      case TM_CMD_ACK_6:
        {
          switch(SUN_CMD)
          {
            case(SUN_CMD_DISTANCE_MOVE):
              {                                                   
                if(srvCurrFuncMode == idleFuncMode)
                {                                    
                  f2bConvert.ba[0] = sunPtrBox->Data[4];
                  f2bConvert.ba[1] = sunPtrBox->Data[5];
                  f2bConvert.ba[2] = sunPtrBox->Data[6];
                  f2bConvert.ba[3] = sunPtrBox->Data[7];
                  
                  sourceId_POS_MOVE = sourceId; // запоминаем отправителя команды, чтобы после выполнения перемещения отправить ему сообщение с подтверждением выполнения заданной им команды
                  
                  if(SUN_TYPEMSG == TM_CMD_ACK_6)
                  {
                    // отсылаем отправителю команды сообщение с подтверждением о приеме команды к исполнению
                    SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_POS_MOVE, TM_ACK_CMD_6, CH_X, (uint8_t)SUN_CMD_DISTANCE_MOVE, (uint8_t)(SUN_CMD_DISTANCE_MOVE>>8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));                                      
                  }
                  
                  srvDefTargetPos = (int)((*Kp) * f2bConvert.f); // пересчет из единиц SUN_CAN [m] в единицы ASD-A2 [PUU]. Kp - коэффициент пересчета
                  
                  if(srvEmgAnswer != (uint32_t)0x00) // возникла какая-то ошибка (сработал концевик, например)
                  {                    
                    inMsgServoBuff.arr[0] = 0x02;
                    inMsgServoBuff.arr[1] = srvCanOpenId;
                    inMsgServoBuff.arr[2] = 0x8F;
                    inMsgServoBuff.arr[3] = 0x00;
                    inMsgServoBuff.arr[4] = 0x00;
                    inMsgServoBuff.arr[5] = 0x00;
                    inMsgServoBuff.arr[6] = 0x00;
                    inMsgServoBuff.arr[7] = 0x00;
                    inMsgServoBuff.arr[8] = 0x00;
                    inMsgServoBuff.arr[9] = 0x00;
                    
                    if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
                    {
                      __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
                    }
                    else
                    {
                      srvCurrFuncMode = posMovFuncMode;
                    }
                  }                  
                  
                  inMsgServoBuff.arr[0] = 0x02;
                  inMsgServoBuff.arr[1] = srvCanOpenId;
                  inMsgServoBuff.arr[2] = 0x4F;
                  inMsgServoBuff.arr[3] = 0x00;
                  inMsgServoBuff.arr[4] = 0x00;
                  inMsgServoBuff.arr[5] = 0x00;
                  inMsgServoBuff.arr[6] = 0x00;
                  inMsgServoBuff.arr[7] = 0x00;
                  inMsgServoBuff.arr[8] = 0x00;
                  inMsgServoBuff.arr[9] = 0x00;
                  
                  if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
                  {
                    __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
                  }
                  else
                  {
                    srvCurrFuncMode = posMovFuncMode;
                  }
                  
                  
                  inMsgServoBuff.arr[0] = 0x02;
                  inMsgServoBuff.arr[1] = srvCanOpenId;
                  inMsgServoBuff.arr[2] = 0x5F;
                  inMsgServoBuff.arr[3] = 0x00;
                  inMsgServoBuff.arr[4] = (uint8_t)(srvDefTargetPos & 0xFF);
                  inMsgServoBuff.arr[5] = (uint8_t)((srvDefTargetPos & 0xFF00) >> 8);
                  inMsgServoBuff.arr[6] = (uint8_t)((srvDefTargetPos & 0xFF0000) >> 16);
                  inMsgServoBuff.arr[7] = (uint8_t)((srvDefTargetPos & 0xFF000000) >> 24);                                   
                  inMsgServoBuff.arr[8] = 0x00;
                  inMsgServoBuff.arr[9] = 0x00;
                  
                  if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
                  {
                    __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
                  }
                  else
                  {
                    srvCurrFuncMode = posMovFuncMode;
                  }
                }                
                else
                {
                  // отказ в принятии команды к исполнению (привод находится не в состоянии покоя) - выдаем код ошибки 0х01 для УПП
                  SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_POS_MOVE, TM_ERROR_CMD, CH_X, (uint8_t)SUN_CMD_DISTANCE_MOVE, (uint8_t)(SUN_CMD_DISTANCE_MOVE>>8), (uint8_t)0x01, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00);
                }
              }
              break; //=</CMD_MOVE_DISTANCE>=
              
              case(SUN_CMD_SPEED_MOVE_RIGHT):
                case(SUN_CMD_SPEED_MOVE_LEFT):
                  {
                    if(srvCurrFuncMode == idleFuncMode)
                    {
                      srvCurrFuncMode = infMovFuncMode;
                      
                      sourceId_INF_MOVE = sourceId; // запоминаем отправителя команды, чтобы после выполнения перемещения отправить ему сообщение с подтверждением выполнения заданной им команды
                      
                      f2bConvert.ba[0] = sunPtrBox->Data[4];
                      f2bConvert.ba[1] = sunPtrBox->Data[5];
                      f2bConvert.ba[2] = sunPtrBox->Data[6];
                      f2bConvert.ba[3] = sunPtrBox->Data[7];
                      
                      sunDefVel = f2bConvert.f;
                      
                      srvDefVel = (uint32_t)((*Kp) * sunDefVel);
                      
                      srvDefAcc = (uint32_t)((srvDefGearFeed * sunDefVel * (float)srvMaxVel * 1000) / (srvDefGearNum * sunDefAcc * (float)srvDefVel)); // пересчёт значения ускорения и торможения в единицы ASDA (в миллисекунды) исходя из НОВОГО значения SUN_Speed
                      
                      srvDefDec = srvDefAcc;
                      
                      srvDefQuickStopDec = (uint32_t)((srvDefGearFeed * sunDefVel * (float)srvMaxVel * 1000) / (srvDefGearNum * sunDefQuickStopAcc * (float)srvDefVel));
                      
                      if(SUN_CMD == SUN_CMD_SPEED_MOVE_RIGHT)
                      {
                        srvDefTargetPos = 0x80000001;
                      }
                      else if(SUN_CMD == SUN_CMD_SPEED_MOVE_LEFT)
                      {
                        srvDefTargetPos = 0x7FFFFFFF;
                      }
                      
                      if(SUN_TYPEMSG == TM_CMD_ACK_6)
                      {
                        SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_INF_MOVE, TM_ACK_CMD_6, CH_X, (uint8_t)SUN_CMD, (uint8_t)(SUN_CMD >> 8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));
                      }
                      
                      srvQSCurrStateFlag &= ~(1 << 0); // т.к. движение началось (т.е. команда принята на выполнение), то сбрасываем нулевой бит стоп-флага о том, что ранее от привода было получено подтверждение о выполнении QuickStop                      
                      
                      inMsgServoBuff.arr[0] = 0x03;
                      inMsgServoBuff.arr[1] = srvCanOpenId;
                      inMsgServoBuff.arr[2] = (uint8_t)(srvDefVel & 0xFF);
                      inMsgServoBuff.arr[3] = (uint8_t)((srvDefVel & 0xFF00) >> 8);
                      inMsgServoBuff.arr[4] = (uint8_t)((srvDefVel & 0xFF0000) >> 16);
                      inMsgServoBuff.arr[5] = (uint8_t)((srvDefVel & 0xFF000000) >> 24);
                      inMsgServoBuff.arr[6] = (uint8_t)(srvDefQuickStopDec & 0xFF);
                      inMsgServoBuff.arr[7] = (uint8_t)((srvDefQuickStopDec & 0xFF00) >> 8);
                      inMsgServoBuff.arr[8] = (uint8_t)((srvDefQuickStopDec & 0xFF0000) >> 16);
                      inMsgServoBuff.arr[9] = (uint8_t)((srvDefQuickStopDec & 0xFF000000) >> 24);              
                      
                      if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
                      {
                        __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
                      }
                      
                      
                      inMsgServoBuff.arr[0] = 0x04;
                      inMsgServoBuff.arr[1] = srvCanOpenId;
                      inMsgServoBuff.arr[2] = (uint8_t)(srvDefAcc & 0xFF);
                      inMsgServoBuff.arr[3] = (uint8_t)((srvDefAcc & 0xFF00) >> 8);
                      inMsgServoBuff.arr[4] = (uint8_t)((srvDefAcc & 0xFF0000) >> 16);
                      inMsgServoBuff.arr[5] = (uint8_t)((srvDefAcc & 0xFF000000) >> 24);
                      inMsgServoBuff.arr[6] = (uint8_t)(srvDefDec & 0xFF);
                      inMsgServoBuff.arr[7] = (uint8_t)((srvDefDec & 0xFF00) >> 8);
                      inMsgServoBuff.arr[8] = (uint8_t)((srvDefDec & 0xFF0000) >> 16);
                      inMsgServoBuff.arr[9] = (uint8_t)((srvDefDec & 0xFF000000) >> 24);
                      
                      if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
                      {
                        __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
                      }
                      
                      
                      inMsgServoBuff.arr[0] = 0x02;
                      inMsgServoBuff.arr[1] = srvCanOpenId;
                      inMsgServoBuff.arr[2] = 0x4F;
                      inMsgServoBuff.arr[3] = 0x00;
                      inMsgServoBuff.arr[4] = 0x00;
                      inMsgServoBuff.arr[5] = 0x00;
                      inMsgServoBuff.arr[6] = 0x00;
                      inMsgServoBuff.arr[7] = 0x00;
                      inMsgServoBuff.arr[8] = 0x00;
                      inMsgServoBuff.arr[9] = 0x00;
                      
                      if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
                      {
                        __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
                      }
                      
                      
                      inMsgServoBuff.arr[0] = 0x02;
                      inMsgServoBuff.arr[1] = srvCanOpenId;
                      inMsgServoBuff.arr[2] = 0x5F;
                      inMsgServoBuff.arr[3] = 0x00;
                      inMsgServoBuff.arr[4] = (uint8_t)(srvDefTargetPos & 0xFF);
                      inMsgServoBuff.arr[5] = (uint8_t)((srvDefTargetPos & 0xFF00) >> 8);
                      inMsgServoBuff.arr[6] = (uint8_t)((srvDefTargetPos & 0xFF0000) >> 16);
                      inMsgServoBuff.arr[7] = (uint8_t)((srvDefTargetPos & 0xFF000000) >> 24);                                   
                      inMsgServoBuff.arr[8] = 0x00;
                      inMsgServoBuff.arr[9] = 0x00;
                      
                      if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
                      {
                        __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
                      }                      
                    }
                    else
                    {                             
                      // отказ в принятии команды к исполнению - TODO: добавить коды ошибки, например, текущий режим (srvCurrFuncMode), при котором выполнение данной команды невозможно
                      SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_INF_MOVE, TM_ERROR_CMD, CH_X, (uint8_t)SUN_CMD_DISTANCE_MOVE, (uint8_t)(SUN_CMD_DISTANCE_MOVE>>8), (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00);
                    }
                  }
                  break;
          }// </switch(SUN_CMD)>
        }
        break;
        
      case TM_CMD_ACK_2:
        {
          switch(SUN_CMD)
          {
            case(SUN_CMD_STOP):
              {                
                sourceId_QSCmd = sourceId;
                
                srvSkipBuffReadingFlag = 1;
                
                clearBuff();
                
                /* Send RPDO 0.1 = 0B & RPDO 0.2 = 0x00000000 (0B 00 00 00 00 00 00 00) */
                SRV_TRANSFER_8(0x200 + srvCanOpenId, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
                
                srvSkipBuffReadingFlag = 0;
              }
              break;
              
              case(SUN_CMD_HALT):
                {                 
                  sourceId_HaltCmd = sourceId;
                  
                  srvSkipBuffReadingFlag = 1;
                  
                  clearBuff();
                  
                  /* Send RPDO 0.1 = 10F & RPDO 0.2 = 0x00000000 (0F 01 00 00 00 00 00 00) */
                  SRV_TRANSFER_8(0x200 + srvCanOpenId, 0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
                  
                  srvSkipBuffReadingFlag = 0;
                }
                break;
          }//</switch(SUN_CMD)>
        }
        break;
      }//</typemsg>
      
      memset(&sunCanBox[sunReceiveTail], 0, sizeof(CanRxMsg));
      sunReceiveTail++;
      sunReceiveTail &= (CAN_SIZE-1);
      continue;      
    }
    
    /*** SRV-CAN Message Handler ***/
    timerSrvCan = setTimeout();    
    while( !passedTimeout(timerSrvCan, 6) )
    {
      __disable_irq();
      srvDummy = srvReceiveHead;
      __enable_irq();
      
      if(srvDummy == srvReceiveTail) break;
      
      srvPtrBox = &srvCanBox[srvReceiveTail];
      
      msgCanOpenId = srvPtrBox->StdId;      
      
      
      if(msgCanOpenId == ((uint32_t)0x700 + srvCanOpenId))
      {
        if((procStep == 0xFF) && (initStep == 0x00))
        {
          if(srvCurrFuncMode == initFuncMode) // если проходит процедура инициализации
          {
            if(SRV_ANSWER_0 == 0x00)
            {
              procStep = 0xFF;
              initStep = 0x01; //первый шаг инициализации
            }
          }
        }
      }
      
      else if(msgCanOpenId == ((uint32_t)0x080 + srvCanOpenId)) //ответ (2ой ответ после ответа 0х703 00) на команду сброса ошибки (ErrRst) или сообщение о наличии(возникновении) ошибки
      {
        srvEmgAnswer = (SRV_ANSWER_LOW << 16) + (SRV_ANSWER_HIGH);                
                
        if(srvCurrFuncMode == initFuncMode) // если проходит процедура инициализации
        {
          if((procStep == 0xFF) && (initStep == 0x01))
          {                       
            procStep = 0x01; // Send ControlWord = ErrorReset to Servo
            initStep = 0xFF;
          }
        }
        else
        {                   
          if(srvEmgAnswer == (uint32_t)0x00)
          {
            srvSkipBuffReadingFlag = 0;
          }
          else
          {
            srvCurrFuncMode = idleFuncMode; // для восстановления возможности принятия и исполнения команд перемещения привода. возможно, стОит устанавливать другое состояние, для корректной выдачи
            srvSkipBuffReadingFlag = 0;
          }
        }
      }
      
      else if(msgCanOpenId == ((uint32_t)0x180 + srvCanOpenId))
      {                     
        srvCurrTargetReachedBit = (SRV_TPDO_STATE_01 >> 10) & 0x1; // из сообщения TPDO# (байты 0 и 1) получаем состояние TargetReachedBit (10й бит)                      
        
        srvPrevStatusWord = srvCurrStatusWord;
        srvCurrStatusWord = SRV_TPDO_STATE_01;
        
        srvCurrControlWord = (SRV_TPDO_STATE_23 & 0x0000FFFF);                
        
        if(srvCurrFuncMode == posMovFuncMode)
        {        
          if((srvPrevStatusWord == 0x1237) && (srvCurrStatusWord == 0x1637) && (srvCurrControlWord == 0x5F))// Statusword = 37 16
          {
            // ставим флаг необходимости выдачи в SUN-CAN ответа об окончании перемещения на заданное расстояние(sunCmdAckFlags.fAckPassedDist) с величиной реально пройденной дистанции, 
            // помещаем в буфер отправки сообщение SYNC, чтобы получить в соответствующем TPDO# текущую координату и из неё вычислить пройденное расстояние (см. процедуру парсинга соответстующего TPDO #), 
            // после этого выдать ответ об окончании выполнения перемещения с величиной реально пройденного приводом расстояния 
            
            sunCmdAckFlags.fAckPassedDist = 1;
            
            inMsgServoBuff.arr[0] = 0x00;
            inMsgServoBuff.arr[1] = 0x80;
            inMsgServoBuff.arr[2] = 0x00;
            inMsgServoBuff.arr[3] = 0x00;
            inMsgServoBuff.arr[4] = 0x00;
            inMsgServoBuff.arr[5] = 0x00;
            inMsgServoBuff.arr[6] = 0x00;
            inMsgServoBuff.arr[7] = 0x00;
            inMsgServoBuff.arr[8] = 0x00;
            inMsgServoBuff.arr[9] = 0x00;
            
            if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
            {
              __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
            }
            
            srvCurrFuncMode = idleFuncMode; 
            
            srvSkipBuffReadingFlag = 0;
          }
          else if((srvCurrStatusWord == 0x0617) && (srvCurrControlWord == 0x0B))
          {           
            // Ставим флаг необходимости выдачи в SUN-CAN ответа об ошибке (т.к. выполнение прекращено командой SUN_CMD_STOP) 
            // в процессе выполнения перемещения на заданное расстояние(sunCmdAckFlags.fAckPassedDist) с величиной реально пройденной дистанции, 
            // помещаем в буфер отправки сообщение SYNC, чтобы получить в соответствующем TPDO #текущую координату 
            // и из неё вычислить пройденное расстояние (см. процедуру парсинга соответстующего TPDO #), 
            // после этого выдать ответ об ошибке в процессе выполнения перемещения с величиной реально пройденного приводом расстояния            
            // Также нужно выдать ответ об удачном выполнении команды SUN_CMD_STOP.
            
            inMsgServoBuff.arr[0] = 0x00;
            inMsgServoBuff.arr[1] = 0x80;
            inMsgServoBuff.arr[2] = 0x00;
            inMsgServoBuff.arr[3] = 0x00;
            inMsgServoBuff.arr[4] = 0x00;
            inMsgServoBuff.arr[5] = 0x00;
            inMsgServoBuff.arr[6] = 0x00;
            inMsgServoBuff.arr[7] = 0x00;
            inMsgServoBuff.arr[8] = 0x00;
            inMsgServoBuff.arr[9] = 0x00;
            
            if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
            {
              __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
            }
            
            srvCurrFuncMode = idleFuncMode;
            
            srvSkipBuffReadingFlag = 0;            
          }
        }  
        
        else if(srvCurrFuncMode == infMovFuncMode)
        {
          if((srvCurrStatusWord == 0x0617) && (srvCurrControlWord == 0x0B))
          {                                   
            // После остановки привода помещаем в буфер отправки сообщение SYNC, чтобы получить в соответствующем TPDO #текущую координату 
            // и из неё вычислить пройденное расстояние (см. процедуру парсинга соответстующего TPDO #), 
            // после этого выдать ответ об окончании выполнения бесконечного перемещения с заданной скоростью, 
            // но пройденное расстояние в этом ответе выдавать не нужно (не предусмотрено протоколом)
            // текущая координата же нужна для дальнейших вычислений пройденного расстояния в случае получения команды перемещения на заданное расстояние.            
            // Также нужно выдать ответ об удачном выполнении команды SUN_CMD_STOP.
            inMsgServoBuff.arr[0] = 0x00;
            inMsgServoBuff.arr[1] = 0x80;
            inMsgServoBuff.arr[2] = 0x00;
            inMsgServoBuff.arr[3] = 0x00;
            inMsgServoBuff.arr[4] = 0x00;
            inMsgServoBuff.arr[5] = 0x00;
            inMsgServoBuff.arr[6] = 0x00;
            inMsgServoBuff.arr[7] = 0x00;
            inMsgServoBuff.arr[8] = 0x00;
            inMsgServoBuff.arr[9] = 0x00;
            
            if(putDataIntoBuff(inMsgServoBuff) == 0) // if buffer overrun
            {
              __NOP(); // выдать ошибку о проблемах связи, т.к. буфер переполнен - сообщения поступают, но не обрабатываются и не уходят из буфера
            }
            
            srvCurrFuncMode = idleFuncMode;
            
            srvSkipBuffReadingFlag = 0;            
          }
        }
        
        else if(srvCurrFuncMode == initFuncMode)
        {
          if((srvCurrStatusWord == 0x0250) && ((srvCurrControlWord == 0x04) || (srvCurrControlWord == 0x80)))
          {          
            procStep = 0x3D;
          }
          
          else if((srvCurrStatusWord == 0x0250) && (srvCurrControlWord == 0x06))
          {
            procStep = 0x3E;
          }
          
          else if((srvCurrStatusWord == 0x0637) && (srvCurrControlWord == 0x4F))
          {
            procStep = 0x4E;
          }
          
          else if((srvCurrStatusWord == 0x1637) && (srvCurrControlWord == 0x5F))
          {
            procStep = 0x4F;
          }          
        }
      }
      
      else if(msgCanOpenId == ((uint32_t)0x280 + srvCanOpenId)) // получили TPDO #1.x
      {
        srvCurrVel = SRV_ANSWER_LOW;
        srvCurrQuickStopDec = (SRV_ANSWER_HIGH & 0x0000FFFF);        
        
        if(srvCurrFuncMode == initFuncMode) // если проходит процедура инициализации
        {
          // поставим флаг, что ответ получен
          srvTPDO1AnswerFlag = 0x01;
        }
        
        if(sunCmdAckFlags.fAckVel == 1)
        {
          sunCmdAckFlags.fAckVel = 0;
          
          sunCurrVel = (float)srvCurrVel / (*Kp);
          
          f2bConvert.f = sunCurrVel;
          
          SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_PosVel, TM_ANSWER_4, CH_X, (uint8_t)SUN_POS_DEF_VEL, (uint8_t)(SUN_POS_DEF_VEL>>8),  (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));              
        }
        
        if(sunCmdAckFlags.fAckQSAcc == 1)
        {
          sunCmdAckFlags.fAckQSAcc = 0;
          
          sunCurrVel = (float)srvCurrVel / (*Kp);
          
          sunCurrQuickStopAcc = (float)((srvDefGearFeed * sunCurrVel * (float)srvMaxVel * 1000) / (srvDefGearNum * srvCurrQuickStopDec * (float)srvCurrVel));                     
          
          f2bConvert.f = sunCurrQuickStopAcc; 
          
          SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_QSAcc, TM_ANSWER_4, CH_X, (uint8_t)SUN_POS_DEF_QS_ACC, (uint8_t)(SUN_POS_DEF_QS_ACC>>8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));
        }
      }
      
      else if(msgCanOpenId == ((uint32_t)0x380 + srvCanOpenId)) // получили TPDO #2.x
      {
        srvCurrAcc = (SRV_ANSWER_LOW & 0x0000FFFF);
        srvCurrDec = (SRV_ANSWER_HIGH & 0x0000FFFF);
        
        if(srvCurrFuncMode == initFuncMode) // если проходит процедура инициализации
        {
          // поставим флаг, что ответ получен
          srvTPDO2AnswerFlag = 0x01;
        }
        
        if(sunCmdAckFlags.fAckAccDec == 1)
        {
          sunCmdAckFlags.fAckAccDec = 0;
          
          if(srvCurrAcc == srvCurrDec)
          {
            sunCurrVel = (float)srvCurrVel / (*Kp);
            
            sunCurrAcc = (float)((srvDefGearFeed * sunCurrVel * (float)srvMaxVel * 1000) / (srvDefGearNum * srvCurrAcc * (float)srvCurrVel));                     
            
            f2bConvert.f = sunCurrAcc; 
            
            SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_QSAcc, TM_ANSWER_4, CH_X, (uint8_t)SUN_POS_DEF_ACC_DEC, (uint8_t)(SUN_POS_DEF_ACC_DEC>>8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));
          }
        }
      }
      
      else if(msgCanOpenId == ((uint32_t)0x480 + srvCanOpenId)) // получили TPDO #3.x
      {
        srvCurrActualPos = SRV_ANSWER_LOW;
        srvCurrDIState = (SRV_ANSWER_HIGH & 0x0000FFFF);
        
        srvPassedDist = srvCurrActualPos - srvPrevActualPos;
        
        if(sunCmdAckFlags.fAckPassedDist == 1)
        {              
          sunCmdAckFlags.fAckPassedDist = 0;
          
          f2bConvert.f = (float)srvPassedDist / (*Kp); // // пересчет из единиц ASD-A2 [PUU] в единицы SUN_CAN [m]. Kp - коэффициент пересчета
          
          // отсылаем отправителю команды сообщение с подтверждением об окончании выполнения команды (уже выполнили) и реально пройденном расстоянии
          SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_POS_MOVE, TM_EXECUTE_CMD_6, CH_X, (uint8_t)SUN_CMD_DISTANCE_MOVE, (uint8_t)(SUN_CMD_DISTANCE_MOVE>>8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));                        
        }
        
        srvPrevActualPos = srvCurrActualPos;                
        
        srvPrevDIState = srvCurrDIState;
        
        if(srvCurrFuncMode == initFuncMode) // если проходит процедура инициализации
        {          
          // поставим флаг, что ответ получен
          srvTPDO3AnswerFlag = 0x01;
          
          if(procStep == 0xFF)
          {
            if(srvSyncCounter < 2)
            {
              procStep = 0x4F;
              
              srvSyncCounter++;
              
              if(srvSyncCounter == 1)
              {
                sunCmdAckFlags.fAckVel = 1;
                sunCmdAckFlags.fAckAccDec = 1;
                sunCmdAckFlags.fAckQSAcc = 1;
              }
            }
            else
            {
              procStep = 0x50;
            }
          }
        }
      }
      
      else if(msgCanOpenId == ((uint32_t)0x580 + srvCanOpenId))
      {
        switch(SRV_TYPEMSG) // тип сообщения
        {         
        case TM_SRV_43:
        case TM_SRV_4B:
        case TM_SRV_4F:
          {
            switch(SRV_OBJ)
            {
            case SRV_OBJ_220A: // DI1..DI8 Type & Func
            case SRV_OBJ_220B:
            case SRV_OBJ_220C:
            case SRV_OBJ_220D:
            case SRV_OBJ_220E:
            case SRV_OBJ_220F:
            case SRV_OBJ_2210:
            case SRV_OBJ_2211:
              {
                if(srvCurrFuncMode == initFuncMode) // если проходит процедура инициализации
                {
                  switch ((uint8_t)(SRV_ANSWER_HIGH & 0x000000FF))
                  {
                  case 0x21:
                    srvInputs[0].inputNum = (uint8_t)(SRV_OBJ - SRV_OBJ_220A) + 0x01;
                    srvInputs[0].inputType = (uint8_t)((SRV_ANSWER_HIGH & 0x0000FF00) >> 8);
                    
                    //                    printf("inputNum = %x; inputType = %x \n", srvInputs[0].inputNum, srvInputs[0].inputType);
                    break;
                    
                  case 0x22:
                    srvInputs[1].inputNum = (uint8_t)(SRV_OBJ - SRV_OBJ_220A) + 0x01;
                    srvInputs[1].inputType = (uint8_t)((SRV_ANSWER_HIGH & 0x0000FF00) >> 8);
                    
                    //                    printf("inputNum = %x; inputType = %x \n", srvInputs[1].inputNum, srvInputs[1].inputType);
                    break;
                    
                  case 0x23:
                    srvInputs[2].inputNum = (uint8_t)(SRV_OBJ - SRV_OBJ_220A) + 0x01;
                    srvInputs[2].inputType = (uint8_t)((SRV_ANSWER_HIGH & 0x0000FF00) >> 8);
                    
                    //                    printf("inputNum = %x; inputType = %x \n", srvInputs[2].inputNum, srvInputs[2].inputType);
                    break;
                  }
                  
                  procStep = 0x3E + (uint8_t)(SRV_OBJ - SRV_OBJ_220A) + 0x01; // Go to Reading next (after current) DI Func and Type
                }
              }
              break;
              
            case SRV_OBJ_2224: // EDI9..EDI14 Type & Func
            case SRV_OBJ_2225:
            case SRV_OBJ_2226:
            case SRV_OBJ_2227:
            case SRV_OBJ_2228:
            case SRV_OBJ_2229:
              {
                if(srvCurrFuncMode == initFuncMode) // если проходит процедура инициализации
                {
                  switch ((uint8_t)(SRV_ANSWER_HIGH & 0x000000FF))
                  {
                  case 0x21:
                    srvInputs[0].inputNum = (uint8_t)(SRV_OBJ - SRV_OBJ_2224) + 0x08 + 0x01;
                    srvInputs[0].inputType = (uint8_t)((SRV_ANSWER_HIGH & 0x0000FF00) >> 8);
                    break;
                    
                  case 0x22:
                    srvInputs[1].inputNum = (uint8_t)(SRV_OBJ - SRV_OBJ_2224) + 0x08 + 0x01;
                    srvInputs[1].inputType = (uint8_t)((SRV_ANSWER_HIGH & 0x0000FF00) >> 8);
                    break;
                    
                  case 0x23:
                    srvInputs[2].inputNum = (uint8_t)(SRV_OBJ - SRV_OBJ_2224) + 0x08 + 0x01;
                    srvInputs[2].inputType = (uint8_t)((SRV_ANSWER_HIGH & 0x0000FF00) >> 8);
                    break;
                  }
                  
                  procStep = 0x46 + (uint8_t)(SRV_OBJ - SRV_OBJ_2224) + 0x01; // Go to Reading next (after current) EDI Func and Type
                }
              }
              break;
              
            case SRV_OBJ_2407: // DIs current state
              {
                srvCurrDIState = SRV_ANSWER_HIGH;
                
                srvPrevDIState = srvCurrDIState;
                
                procStep = 0x4D;
              }
              break;
              
            case SRV_OBJ_6040: // Control Word
              {
                switch((SRV_DL1<<8) + SRV_DL0)
                {
                case servoContWrd_04:
                case servoContWrd_80:
                  {
                    /* на этом шаге ранее мы отправили команду "ErrorReset to Servo (2B 40 60 00 80 00 00 00)", получили
                    подтверждение о её выполнении и сделали запрос на чтение параметра, устанавливаемого этой командой */
                    if(procStep == 0xFF)
                    {
                      if(srvCurrFuncMode == initFuncMode) // если проходит процедура инициализации
                      {
                        procStep = 0x02; // Go to Clear RPDO 0.x
                      }
                    }
                  } //=< /SRV_OBJ_6040 >=
                  break;                  
                } //=< /switch((SRV_DL1<<8) + SRV_DL0) >=
              } //=< /case SRV_OBJ_6040 >=
              break;
              
            case SRV_OBJ_6060: //обрабатываем ответ привода на запрос чтения OperationMode
              {
                srvCurrOperationMode = (uint32_t)SRV_ANSWER_HIGH;
                
                if(srvCurrFuncMode == initFuncMode) // если проходит процедура инициализации
                {
                  if(srvCurrOperationMode == srvDefOM)
                  {
                    procStep = 0x3C;
                  }
                }                
              } // =< /case SRV_OBJ_6060 >=
              break;
              
            case SRV_OBJ_6081: //обрабатываем ответ привода на запрос чтения Profile Velocity
              {
                srvCurrVel = (uint32_t)SRV_ANSWER_HIGH;
                
                sunCurrVel = (float)srvCurrVel / (*Kp);
                
                f2bConvert.f = sunCurrVel;
                
                SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_PosVel, TM_ANSWER_4, CH_X, (uint8_t)SUN_POS_DEF_VEL, (uint8_t)(SUN_POS_DEF_VEL>>8),  (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));
              } // =< /case SRV_OBJ_6081 >=
              break;
              
              
            case SRV_OBJ_6083: //обрабатываем ответ привода на запрос чтения Profile Acceleration / Profile Deceleration
              {
                srvCurrAcc = (uint32_t)SRV_ANSWER_HIGH;
              }
              
            case SRV_OBJ_6084:
              {              
                srvCurrDec = (uint32_t)SRV_ANSWER_HIGH;
                
                if(srvCurrAcc == srvCurrDec) //величины должны совпадать, т.к. извне задаётся одно значение одной командой
                {                
                  sunCurrAcc = (float)((srvDefGearFeed * sunCurrVel * (float)srvMaxVel * 1000) / (srvDefGearNum * srvCurrAcc * (float)srvCurrVel));
                  
                  f2bConvert.f = sunCurrAcc;
                  
                  SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_AccDec, TM_ANSWER_4, CH_X, (uint8_t)SUN_POS_DEF_ACC_DEC, (uint8_t)(SUN_POS_DEF_ACC_DEC>>8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));
                }
              } // =< /case SRV_OBJ_6083... >=
              break;              
              
              
            case SRV_OBJ_6085: //обрабатываем ответ привода на запрос чтения Profile QuickStop Acceleration
              {
                srvCurrQuickStopDec = (uint32_t)SRV_ANSWER_HIGH;
                
                sunCurrVel = (float)srvCurrVel / (*Kp);
                
                sunCurrQuickStopAcc = (float)((srvDefGearFeed * sunCurrVel * (float)srvMaxVel * 1000) / (srvDefGearNum * srvCurrQuickStopDec * (float)srvCurrVel));
                
                f2bConvert.f = sunCurrQuickStopAcc;
                
                SUN_TRANSFER_8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_QSAcc, TM_ANSWER_4, CH_X, (uint8_t)SUN_POS_DEF_QS_ACC, (uint8_t)(SUN_POS_DEF_QS_ACC>>8), (uint8_t)(f2bConvert.ba[0]), (uint8_t)(f2bConvert.ba[1]), (uint8_t)(f2bConvert.ba[2]), (uint8_t)(f2bConvert.ba[3]));
              } // =< /case SRV_OBJ_6085 >=
              break;              
              
            } // =< /switch(SRV_OBJ) >=
          } // =< /case TM_SRV_43... >=
          break;
          
        case TM_SRV_60:
          {
            switch(SRV_OBJ)
            {
            case SRV_OBJ_6040:
              {
                if((fNeedToGetParamValue == 0x00006040) && (srvCurrFuncMode == posQSFuncMode))
                {
                  SUN_TRANSFER_4(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|sourceId_QSCmd, TM_ACK_CMD_2, CH_X, (uint8_t)SUN_CMD_STOP, (uint8_t)(SUN_CMD_STOP >>8));
                }
              }
              break;
              
            case SRV_OBJ_1600:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x00)
                  {
                    if(srvAnswersFlags_160x_0.fAnswer_1600_0 == 0)
                    {
                      procStep = 0x03;
                      
                      srvAnswersFlags_160x_0.fAnswer_1600_0 = 1;
                    }
                    else  if(srvAnswersFlags_160x_0.fAnswer_1600_0 == 1)
                    {
                      procStep = 0x17;
                      
                      srvAnswersFlags_160x_0.fAnswer_1600_0 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x01)
                  {
                    procStep = 0x0B;
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {
                    procStep = 0x0C;
                  }
                }
              }
              break;            
              
            case SRV_OBJ_1601:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x00)
                  {
                    if(srvAnswersFlags_160x_0.fAnswer_1601_0 == 0)
                    {
                      procStep = 0x04;
                      
                      srvAnswersFlags_160x_0.fAnswer_1601_0 = 1;
                    }
                    else  if(srvAnswersFlags_160x_0.fAnswer_1601_0 == 1)
                    {
                      procStep = 0x18;
                      
                      srvAnswersFlags_160x_0.fAnswer_1601_0 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x01)
                  {
                    procStep = 0x0D;
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {
                    procStep = 0x0E;
                  }
                }
              }
              break;            
              
            case SRV_OBJ_1602:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x00)
                  {
                    if(srvAnswersFlags_160x_0.fAnswer_1602_0 == 0)
                    {
                      procStep = 0x05;
                      
                      srvAnswersFlags_160x_0.fAnswer_1602_0 = 1;
                    }
                    else  if(srvAnswersFlags_160x_0.fAnswer_1602_0 == 1)
                    {
                      procStep = 0x19;
                      
                      srvAnswersFlags_160x_0.fAnswer_1602_0 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x01)
                  {
                    procStep = 0x0F;
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {
                    procStep = 0x10;
                  }
                }
              }
              break;
              
            case SRV_OBJ_1603:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x00)
                  {
                    if(srvAnswersFlags_160x_0.fAnswer_1603_0 == 0)
                    {
                      procStep = 0x06;
                      
                      srvAnswersFlags_160x_0.fAnswer_1603_0 = 1;
                    }
                    else if(srvAnswersFlags_160x_0.fAnswer_1603_0 == 1)
                    {
                      procStep = 0x1A;
                      
                      srvAnswersFlags_160x_0.fAnswer_1603_0 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x01)
                  {
                    procStep = 0x11;
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {
                    procStep = 0x12;
                  }
                }
              }
              break;
              
            case SRV_OBJ_1400:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x01)
                  {  
                    if(srvAnswersFlags_140x_1.fAnswer_1400_1 == 0)
                    {
                      procStep = 0x07;
                      
                      srvAnswersFlags_140x_1.fAnswer_1400_1 = 1;
                    }
                    else if(srvAnswersFlags_140x_1.fAnswer_1400_1 == 1)
                    {
                      procStep = 0x1B;
                      
                      srvAnswersFlags_140x_1.fAnswer_1400_1 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {                  
                    procStep = 0x13;
                  }
                }
              }
              break;            
              
            case SRV_OBJ_1401:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x01)
                  {  
                    if(srvAnswersFlags_140x_1.fAnswer_1401_1 == 0)
                    {
                      procStep = 0x08;
                      
                      srvAnswersFlags_140x_1.fAnswer_1401_1 = 1;
                    }
                    else if(srvAnswersFlags_140x_1.fAnswer_1401_1 == 1)
                    {
                      procStep = 0x1C;
                      
                      srvAnswersFlags_140x_1.fAnswer_1401_1 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {                  
                    procStep = 0x14;
                  }                  
                }
              }
              break;            
              
            case SRV_OBJ_1402:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x01)
                  {  
                    if(srvAnswersFlags_140x_1.fAnswer_1402_1 == 0)
                    {
                      procStep = 0x09;
                      
                      srvAnswersFlags_140x_1.fAnswer_1402_1 = 1;
                    }
                    else if(srvAnswersFlags_140x_1.fAnswer_1402_1 == 1)
                    {
                      procStep = 0x1D;
                      
                      srvAnswersFlags_140x_1.fAnswer_1402_1 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {                  
                    procStep = 0x15;
                  }                  
                }
              }
              break;
              
            case SRV_OBJ_1403:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x01)
                  {  
                    if(srvAnswersFlags_140x_1.fAnswer_1403_1 == 0)
                    {
                      procStep = 0x0A;
                      
                      srvAnswersFlags_140x_1.fAnswer_1403_1 = 1;
                    }
                    else if(srvAnswersFlags_140x_1.fAnswer_1403_1 == 1)
                    {
                      procStep = 0x1E;
                      
                      srvAnswersFlags_140x_1.fAnswer_1403_1 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {                  
                    procStep = 0x16;
                  }                  
                }
              }
              break; 
              
            case SRV_OBJ_1A00:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x00)
                  {
                    if(srvAnswersFlags_1A0x_0.fAnswer_1A00_0 == 0)
                    {
                      procStep = 0x1F;
                      
                      srvAnswersFlags_1A0x_0.fAnswer_1A00_0 = 1;
                    }
                    else  if(srvAnswersFlags_1A0x_0.fAnswer_1A00_0 == 1)
                    {
                      procStep = 0x34;
                      
                      srvAnswersFlags_1A0x_0.fAnswer_1A00_0 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x01)
                  {
                    procStep = 0x27;
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {
                    procStep = 0x28;
                  }
                  else if(SRV_SUBINDEX == 0x03)
                  {
                    procStep = 0x29;
                  } 
                }
              }
              break;            
              
            case SRV_OBJ_1A01:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x00)
                  {
                    if(srvAnswersFlags_1A0x_0.fAnswer_1A01_0 == 0)
                    {
                      procStep = 0x20;
                      
                      srvAnswersFlags_1A0x_0.fAnswer_1A01_0 = 1;
                    }
                    else  if(srvAnswersFlags_1A0x_0.fAnswer_1A01_0 == 1)
                    {
                      procStep = 0x35;
                      
                      srvAnswersFlags_1A0x_0.fAnswer_1A01_0 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x01)
                  {
                    procStep = 0x2A;
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {
                    procStep = 0x2B;
                  }
                }
              }
              break;            
              
            case SRV_OBJ_1A02:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x00)
                  {
                    if(srvAnswersFlags_1A0x_0.fAnswer_1A02_0 == 0)
                    {
                      procStep = 0x21;
                      
                      srvAnswersFlags_1A0x_0.fAnswer_1A02_0 = 1;
                    }
                    else  if(srvAnswersFlags_1A0x_0.fAnswer_1A02_0 == 1)
                    {
                      procStep = 0x36;
                      
                      srvAnswersFlags_1A0x_0.fAnswer_1A02_0 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x01)
                  {
                    procStep = 0x2C;
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {
                    procStep = 0x2D;
                  }
                }
              }
              break;
              
            case SRV_OBJ_1A03:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x00)
                  {
                    if(srvAnswersFlags_1A0x_0.fAnswer_1A03_0 == 0)
                    {
                      procStep = 0x22;
                      
                      srvAnswersFlags_1A0x_0.fAnswer_1A03_0 = 1;
                    }
                    else  if(srvAnswersFlags_1A0x_0.fAnswer_1A03_0 == 1)
                    {
                      procStep = 0x37;
                      
                      srvAnswersFlags_1A0x_0.fAnswer_1A03_0 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x01)
                  {
                    procStep = 0x2E;
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {
                    procStep = 0x2F;
                  }
                }
              }
              break;              
              
            case SRV_OBJ_1800:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x01)
                  {  
                    if(srvAnswersFlags_180x_1.fAnswer_1800_1 == 0)
                    {
                      procStep = 0x23;
                      
                      srvAnswersFlags_180x_1.fAnswer_1800_1 = 1;
                    }
                    else if(srvAnswersFlags_180x_1.fAnswer_1800_1 == 1)
                    {
                      procStep = 0x38;
                      
                      srvAnswersFlags_180x_1.fAnswer_1800_1 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {                  
                    procStep = 0x30;
                  }
                }
              }
              break;            
              
            case SRV_OBJ_1801:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x01)
                  {  
                    if(srvAnswersFlags_180x_1.fAnswer_1801_1 == 0)
                    {
                      procStep = 0x24;
                      
                      srvAnswersFlags_180x_1.fAnswer_1801_1 = 1;
                    }
                    else if(srvAnswersFlags_180x_1.fAnswer_1801_1 == 1)
                    {
                      procStep = 0x39;
                      
                      srvAnswersFlags_180x_1.fAnswer_1801_1 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {                  
                    procStep = 0x31;
                  }                  
                }
              }
              break;            
              
            case SRV_OBJ_1802:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x01)
                  {  
                    if(srvAnswersFlags_180x_1.fAnswer_1802_1 == 0)
                    {
                      procStep = 0x25;
                      
                      srvAnswersFlags_180x_1.fAnswer_1802_1 = 1;
                    }
                    else if(srvAnswersFlags_180x_1.fAnswer_1802_1 == 1)
                    {
                      procStep = 0x3A;
                      
                      srvAnswersFlags_180x_1.fAnswer_1802_1 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {                  
                    procStep = 0x32;
                  }                  
                }
              }
              break;
              
            case SRV_OBJ_1803:
              {
                if(srvCurrFuncMode == initFuncMode)
                {
                  if(SRV_SUBINDEX == 0x01)
                  {  
                    if(srvAnswersFlags_180x_1.fAnswer_1803_1 == 0)
                    {
                      procStep = 0x26;
                      
                      srvAnswersFlags_180x_1.fAnswer_1803_1 = 1;
                    }
                    else if(srvAnswersFlags_180x_1.fAnswer_1803_1 == 1)
                    {
                      procStep = 0x3B;
                      
                      srvAnswersFlags_180x_1.fAnswer_1803_1 = 0;
                    }
                  }
                  else if(SRV_SUBINDEX == 0x02)
                  {                  
                    procStep = 0x33;
                  }                  
                }
              }
              break;               
              
            } // =< /switch(SRV_OBJ) >=
            
            if(fNeedToGetParamValue)
            {
              /* Re-Ask Data if we got Exec_Confirm msg */
              SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, (fNeedToGetParamValue & 0xFF), ((fNeedToGetParamValue & 0xFF00) >> 8), ((fNeedToGetParamValue & 0xFF0000) >> 16), ((fNeedToGetParamValue & 0xFF000000) >> 24), 0x00, 0x00, 0x00);
              
              fNeedToGetParamValue = 0x00000000;
            }
          }
          break; //~case TM_SRV_60
          
        } // =< /switch(SRV_TYPEMSG) >=
      } // =< /if(msgCanOpenId == ((uint32_t)0x580 + srvCanOpenId)) >=
      
      memset(&srvCanBox[srvReceiveTail], 0, sizeof(CanRxMsg));
      srvReceiveTail++;
      srvReceiveTail &= (CAN_SIZE-1);
      continue;
      /* ~SRV_CAN end */
    } //~while( !passedTimeout(timerSrvCan, 6)  )
    
    //**************************************************************************
    
    initServo(srvCurrFuncMode);
    //-------------------------
    
    if(srvSkipBuffReadingFlag == 0)
    {
      memset(outMsgServoBuff.arr, 0x00, 10); 
      readBufDataResult = getDataFromBuff(&outMsgServoBuff);
    }
    
    if(readBufDataResult > 0)
    {         
      if(outMsgServoBuff.arr[2] == 0x8F)
      {
        srvSkipBuffReadingFlag = 1;
        
        // текущее сообщение отправляем
        SRV_TRANSFER_8((uint32_t)((outMsgServoBuff.arr[0] << 8) + outMsgServoBuff.arr[1]), outMsgServoBuff.arr[2], outMsgServoBuff.arr[3], outMsgServoBuff.arr[4], outMsgServoBuff.arr[5], outMsgServoBuff.arr[6], outMsgServoBuff.arr[7], outMsgServoBuff.arr[8], outMsgServoBuff.arr[9]);            
        
        memset(outMsgServoBuff.arr, 0x00, 10); //TODO: возможно, это обнуление лишнее - проверить
        
        readBufDataResult = 0;  // обнуляем, чтобы здесь не зациклиться 
      }
      
      else if(outMsgServoBuff.arr[2] == 0x5F)
      {  
        srvSkipBuffReadingFlag = 1;
        
        if((srvCurrControlWord == 0x4F) && (srvCurrStatusWord == 0x0637))// Statusword = 37 06
        {                         
          // текущее сообщение отправляем
          SRV_TRANSFER_8((uint32_t)((outMsgServoBuff.arr[0] << 8) + outMsgServoBuff.arr[1]), outMsgServoBuff.arr[2], outMsgServoBuff.arr[3], outMsgServoBuff.arr[4], outMsgServoBuff.arr[5], outMsgServoBuff.arr[6], outMsgServoBuff.arr[7], outMsgServoBuff.arr[8], outMsgServoBuff.arr[9]);            
          
          memset(outMsgServoBuff.arr, 0x00, 10); //TODO: возможно, это обнуление лишнее - проверить
          
          readBufDataResult = 0;  // обнуляем, чтобы здесь не зациклиться        
        }
      }
      else
      {   
        srvSendCmd(outMsgServoBuff);                    
      }
    }    
  }
} // </ main >

/*** = = = - - - < User Functions >- - - = = = ***/
#ifdef NDEBUG

// получаем КАН-адрес устройства по состоянию переключателей на его адресных пинах (Pin.A4-PinA.7)
// такие сложные расчёты нужны для соответствия выставленных переключателей получаемому адресу
uint16_t  GetAddress(void)
{
  volatile uint16_t CAN_ADR_dummy;
  // получаем состояние всех пинов порта, выделяем состояние пинов Pin.A4-PinA.7, остальные биты зануляем,
  // получаем dummy вида 0000 0000 abcd 0000, при этом a(или b или с или d) = 1, если движок переключателя в положении off
  // и а(или b или с или d) = 0, если движок переключателя в положении on,
  // далее инвертируем биты, чтобы  a(или b или с или d) = 0, если движок переключателя в положении off
  // и а(или b или с или d) = 1, если движок переключателя в положении on,
  CAN_ADR_dummy = (((GPIOD->IDR)&0x00F0)>>4) ^ 0x000F;
  // далее сдвигаем биты вправо чтобы получить dummy в виде 0000 0000 0000 abcd
  CAN_ADR_dummy = ((CAN_ADR_dummy & 0x55) << 1) | ((CAN_ADR_dummy & 0xAA) >> 1);
  CAN_ADR_dummy = ((CAN_ADR_dummy & 0xCC) >> 2) | ((CAN_ADR_dummy & 0x33) << 2);
  // добавляем к установленному адресу базовый адрес 0х0060, чтобы не попасть на адреса серводрайвов
  CAN_ADR_dummy = CAN_ADR_dummy + 0x0060;
  // ограничиваем адрес сверху, чтобы не залезть в группу устройств с адресами 0х007х
  (CAN_ADR_dummy>0x006F)?(CAN_ADR_dummy = 0x006F):(0);
  return CAN_ADR_dummy;
}

uint16_t GetHardVersion(void)
{
  //  return (VERSION_HARD);
}

uint32_t  GetSerialNum(void)
{
  return (uint32_t)0x00000002;
}

#pragma location = "TARGET"
__root __no_init TargetStruct target;

union {
  uint8_t   byte[FLASH_PAGE_SIZE];
  uint16_t  halfword[FLASH_PAGE_SIZE/2];
  uint32_t  word[FLASH_PAGE_SIZE/4];
}flashPage;


uint32_t WriteFlashSector(uint32_t sector, uint32_t addr, uint32_t data)
{
  uint32_t temp = TRUE;
  
  __disable_irq();
  FLASH_Unlock();
  if(FLASH_EraseSector(sector, VoltageRange_3) == FLASH_COMPLETE)
  {
    temp = FLASH_ProgramWord(addr, data);
    if(temp != FLASH_COMPLETE)
    {
      temp = FALSE; return FALSE;
    }
    if(temp == FLASH_COMPLETE) temp = TRUE;
  }
  else temp = FALSE;
  FLASH_Lock();
  __enable_irq();
  return temp;
}
#endif

uint32_t  setTimeout(void)
{
  uint32_t  Time_dummy;
  do
  {
    __disable_irq();
    Time_dummy = systemTickValue;
    __enable_irq();
  }
  while(!Time_dummy);
  return (Time_dummy);
}

//------------------------------------------------------------------------------
//  Контроль таймаута.
// Вход: значение таймера, граница.
// Выход: 0 - таймаут не истек, 1 - таймаут истек.
//------------------------------------------------------------------------------
uint32_t  passedTimeout(uint32_t time, uint32_t limit)
{
  uint32_t result;
  __disable_irq();
  result = systemTickValue-time;
  __enable_irq();
  return (result < limit)?0:1;
}

/* Функция инициализации Heartbeat таймера */
void Htbt_timer_init(void)
{
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  TIM_TimeBaseStructInit(&TIMER_InitStructure);
  
  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_ClockDivision = 0;
  TIMER_InitStructure.TIM_Prescaler = 45000-1; // частота отсчетов
  TIMER_InitStructure.TIM_Period = 1; // предел счета, по достижению происходит переполнение и прерывание
  TIM_TimeBaseInit(TIM6, &TIMER_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  // считаем один раз
  TIM_SelectOnePulseMode(TIM6, TIM_OPMode_Single);
}


/* Функция запуска Heartbeat таймера со временем срабатывания прерывания n_msec (в миллисекундах) */
void Htbt_timer_start_ms(uint32_t n_msec)
{
  TIM_Cmd(TIM6, DISABLE);
  TIM6->CNT = 0;
  
  TIM6->PSC = 45000-1; // частота отсчетов (точнее, делитель тактовой частоты таймера = 2 * частота шины APB1 (PCKL1)) !!! АХТУНГ: макс значение = UInt16_t (т.е. 65535), чтобы таймер считал с интервалом 1 переполнение за 1 миллисекунду, сюда в предделитель нужно записать 90000-1
  //                     (частота тактирования таймера = 90 Mhz, т.е. 2*частота шины APB1 (PCKL1)), тогда таймер будет каждую миллисекунду увеличивать свой счетчик.
  //                      НО столько записать нельзя, т.к. этот регистр = UInt16_t, поэтому сюда пишем 45000-1. А в регистре предела счета (число отсчетов, при котором происходит переполнение) удваиваем нужное нам значение.
  
  TIM6->ARR = (uint16_t)(2 * n_msec); // предел счета, по достижению происходит переполнение и прерывание (про 2 * n_msec см выше)
  
  // для того чтобы установился новый PSC
  TIM6->EGR |= TIM_EGR_UG;
  TIM6->SR &= ~TIM_SR_UIF;
  
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM6, ENABLE);
}


void initData(void)
{
  //  flintConvert.f = 0.0f;
  //  flintConvert.i = 0;
  
  f2bConvert.f = 0.0f;
  f2bConvert.ba[0] = 0;
  f2bConvert.ba[1] = 0;
  f2bConvert.ba[2] = 0;
  f2bConvert.ba[3] = 0;
  
  // тут задаем параметры для инициализации привода (надо сделать их чтение и запись из памяти)
  sunDefAcc = 5.0f;
  sunDefQuickStopAcc = 10.0f;
  
  // при инициализации привода будем задавать очень малую скорость (занулить мы её не можем, иначе srvDefAcc, srvDefDec и srvDefQSAcc рассчитываются неверно) и занулять дистанцию
  sunDefVel = 1.0f; //0.000001f;
  sunDefTargetPos = 0.0f;
  
  
  // K_rnd_puu = 1280000 * srvDefGearFeed / srvDefGearNum; // переводим round -> PUU с учетом значений коэффициента электронной редукции (Electronic Gear Ratio). При увеличении srvDefGearNum пропорционально увеличивается фактически проходимое расстояние(т.е. фактическое число оборотов (rounds)) и фактическая скорость вращения, но возвращаемое число PUU (например, при чтении значения 0x6064) остается прежним.
  
  // 1280000 - число импульсов энкодера на 1 оборот энкодера( = ротора двигателя)
  
  // Если srvDefGearNum = 1 и srvDefGearFeed = 1, то:
  // Кр(1 ,1) = 1280000 - двигатель делает один оборот ротора,
  // Кр(1 ,1) = 44131387 - коэффициент, при котором каретка проходит 1 метр на балке Rexroth через редуктор 7Ч-М-50-10-ПЦ24 от Приводной Техники, если srvDefGearNum = 1 и srvDefGearFeed = 1.
  
  // Если srvDefGearNum = 22 и srvDefGearFeed = 1, то:
  // Kp_new(22 ,1) для "каретка проходит 1 метр на балке Rexroth через редуктор 7Ч-М-50-10-ПЦ24 от Приводной Техники" должен быть 44131388/22 = 2005972.18
  // Kp_new(22 ,1) для "двигатель делает один оборот ротора" должен быть 1280000/22 = 58181.82
  // !! Вообще Кр(srvDefGearNum, srvDefGearFeed) = (Кр(1,1) * srvDefGearFeed)/srvDefGearNum !!
  
  Kp = (float*)&flashKp;
  
  srvMaxVel = (3000/60) * 1280000; //3000rpm (максимальная скорость двигателя) переводим в PUU/sec для дальнейшего использования в расчётах
  
  srvDefGearNum = 22;  //(0x6093.1) - P1-44
  srvDefGearFeed = 1; //(0x6093.2) - P1-45
  
  // пересчет значений из sunDefParams в аналогичные srvDefParams для записи в привод
  srvDefVel = (uint32_t)((*Kp) * sunDefVel);
  
  srvDefAcc = (uint32_t)((srvDefGearFeed * sunDefVel * (float)srvMaxVel * 1000) / (srvDefGearNum * sunDefAcc * (float)srvDefVel)); // пересчёт значения ускорения и торможения в единицы ASDA (в миллисекунды) исходя из НОВОГО значения SUN_Speed
  
  srvDefDec = srvDefAcc;
  
  srvDefQuickStopDec = (uint32_t)((srvDefGearFeed * sunDefVel * (float)srvMaxVel * 1000) / (srvDefGearNum * sunDefQuickStopAcc * (float)srvDefVel));
  
  
  sunCmdAckFlags.fAckAccDec = 0;
  sunCmdAckFlags.fAckQSAcc = 0;
  sunCmdAckFlags.fAckVel = 0;
  sunCmdAckFlags.fAckPassedDist = 0;
  
  sunCurrAcc = 0.0f;
  sunCurrVel = 0.0f;
  sunCurrTargetPos = 0.0f;
  
  srvCurrFuncMode = noneFuncMode;
  
  srvCurrOperationMode = (uint32_t)0xFFFFFFFF;
  srvCurrControlWord = (uint32_t)0xFFFFFFFF;
  srvCurrAcc = (uint32_t)0xFFFFFFFF;
  srvCurrDec = (uint32_t)0xFFFFFFFF;
  srvCurrQuickStopDec = (uint32_t)0xFFFFFFFF;
  srvCurrVel = (uint32_t)0xFFFFFFFF;
  srvCurrTargetPos = (uint32_t)0xFFFFFFFF;
  srvCurrActualPos = (int32_t)0x00000000;
  srvPrevActualPos = (int32_t)0x00000000;
  srvPassedDist = (int32_t)0x00000000;
  srvCurrTpdoInit = (uint32_t)0xFFFFFFFF;
  srvCurrPosMoveState = 0x00;
  srvCurrDIState = (uint16_t) 0x0000;
  srvPrevDIState = (uint16_t) 0x0000;
  
  fNeedToGetParamValue = (uint32_t)0x00;
  
  srvHtbtState = CLIENT_DEAD;
  
  srvAnswersFlags_140x_1.fAnswer_1400_1 = 0;
  srvAnswersFlags_140x_1.fAnswer_1401_1 = 0;
  srvAnswersFlags_140x_1.fAnswer_1402_1 = 0;
  srvAnswersFlags_140x_1.fAnswer_1403_1 = 0;
  
  srvAnswersFlags_160x_0.fAnswer_1600_0 = 0;
  srvAnswersFlags_160x_0.fAnswer_1601_0 = 0;
  srvAnswersFlags_160x_0.fAnswer_1602_0 = 0;
  srvAnswersFlags_160x_0.fAnswer_1603_0 = 0;  
  
  
  srvAnswersFlags_180x_1.fAnswer_1800_1 = 0;
  srvAnswersFlags_180x_1.fAnswer_1801_1 = 0;
  srvAnswersFlags_180x_1.fAnswer_1802_1 = 0;
  srvAnswersFlags_180x_1.fAnswer_1803_1 = 0;
  
  srvAnswersFlags_1A0x_0.fAnswer_1A00_0 = 0;
  srvAnswersFlags_1A0x_0.fAnswer_1A01_0 = 0;
  srvAnswersFlags_1A0x_0.fAnswer_1A02_0 = 0;
  srvAnswersFlags_1A0x_0.fAnswer_1A03_0 = 0; 
  
  
  srvTPDO1AnswerFlag = 0x00;
  srvTPDO2AnswerFlag = 0x00;
  srvTPDO3AnswerFlag = 0x00;
  
  srvReadyToSwitchOnBit = 0;
  srvCurrQSBit = 0;
  
  srvSetPointAcknowledgeBit = 1;
  
  srvCurrTargetReachedBit = 0;
  srvPrevTargetReachedBit = 0;
  
  srvQSCurrStateFlag = 0x0;
  
  // данные значения OperationWord и ControlWord используем при инициализации привода
  srvDefOM = (uint32_t)servoOpMode_PP;
  srvDefCW = (uint32_t)servoContWrd_8F;
  
  initStep = 0x00;
  procStep = 0x00;
  srvSyncCounter = 0x00;
  
  srvCanOpenId = (uint32_t)0x03;
  
  memset(&sunCanBox[0], 0,sizeof(sunCanBox));
  sunReceiveHead = 0;
  sunReceiveTail = 0;
  
  memset(&srvCanBox[0], 0, sizeof(srvCanBox));
  srvReceiveHead = 0;
  srvReceiveTail = 0;
  
  memset(&srvInputs[0], 0, sizeof(srvInputs));
  memset(&srvInputs[0], 0, sizeof(srvInputs));
  
  mcuStatusLedTimer = 0;
  sunCanLedTimer    = 0;
  srvCanLedTimer    = 0;
  
  currentMcuLedState = 0;
  
  readBufDataResult = 0;
  srvSkipBuffReadingFlag = 0;
  
  srvEmgAnswer = (uint32_t)0xFF;
}

void initServo(srvFuncMode srvCurrMode)
{
  if((srvCurrMode == noneFuncMode) || (srvCurrMode == initFuncMode))
  {
    srvCurrFuncMode = initFuncMode;
    
    switch(procStep)
    {
    case 0x00:
      {
        if(initStep == 0x00)
        {
          /* Send Reset_Node Cmd to Servo */
          SRV_TRANSFER_8(0x00, 0x81, srvCanOpenId, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);  // Target_Std_Id = 0x00, Cmd = 0x81, Cmd_Param = srvCanOpenId(Servo CAN_Id/CAN-Address)
        }
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x01:
      {
        /* Send ControlWord = ErrorReset to Servo (2B 60 40 00 servoContWrd_80 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2B, 0x40, 0x60, 0x00, servoContWrd_80, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00006040;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
      // ==< RPDOs >==        
      
    case 0x02:
      {
        /* Clear RPDO 0.x mapping (2F 00 16 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;  
      
    case 0x03:
      {
        /* Clear RPDO 1.x mapping (2F 01 16 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x04:
      {
        /* Clear RPDO 2.x mapping (2F 02 16 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x02, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x05:
      {
        /* Clear RPDO 3.x mapping (2F 03 16 00 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x03, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x06:
      {
        /* Turn Off RPDO 0.x (23 00 14 01 srvCanOpenId 02 00 80) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x00, 0x14, 0x01, (uint8_t)srvCanOpenId, 0x02, 0x00, 0x80);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;  
      
    case 0x07:
      {
        /* Turn Off RPDO 1.x (23 01 14 01 srvCanOpenId 03 00 80) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x01, 0x14, 0x01, (uint8_t)srvCanOpenId, 0x03, 0x00, 0x80);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x08:
      {
        /* Turn Off RPDO 2.x (23 02 14 01 srvCanOpenId 04 00 80) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x02, 0x14, 0x01, (uint8_t)srvCanOpenId, 0x04, 0x00, 0x80);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x09:
      {
        /* Turn Off RPDO 3.x (23 03 14 01 srvCanOpenId 05 00 80) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x03, 0x14, 0x01, (uint8_t)srvCanOpenId, 0x05, 0x00, 0x80);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x0A:
      {
        /* Set RPDO 0.1 mapping = ControlWord(23 00 16 01 10 00 40 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x00, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x0B:
      {
        /* Set RPDO 0.2 mapping = Profile Position(23 00 16 02 20 00 7A 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x00, 0x16, 0x02, 0x20, 0x00, 0x7A, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x0C:
      {
        /* Set RPDO 1.1 mapping = Profile Velocity(23 01 16 01 20 00 81 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x01, 0x16, 0x01, 0x20, 0x00, 0x81, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;      
      
    case 0x0D:
      {
        /* Set RPDO 1.2 mapping = Profile QuickStopAcceleration(23 01 16 02 20 00 85 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x01, 0x16, 0x02, 0x20, 0x00, 0x85, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x0E:
      {
        /* Set RPDO 2.1 mapping = Profile Acceleration(23 02 16 01 20 00 83 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x02, 0x16, 0x01, 0x20, 0x00, 0x83, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;      
      
    case 0x0F:
      {
        /* Set RPDO 2.2 mapping = Profile Deceleration(23 02 16 02 20 00 84 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x02, 0x16, 0x02, 0x20, 0x00, 0x84, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x10:
      {
        /* Set RPDO 3.1 mapping = Profile Acceleration(23 03 16 01 20 00 83 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x03, 0x16, 0x01, 0x20, 0x00, 0x83, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;      
      
    case 0x11:
      {
        /* Set RPDO 3.2 mapping = Profile Deceleration(23 03 16 02 20 00 84 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x3, 0x16, 0x02, 0x20, 0x00, 0x84, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;      
      
    case 0x12:
      {
        /* Set RPDO 0.x mode = 0xFF (2F 00 14 02 FF 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x00, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;  
      
    case 0x13:
      {
        /* Set RPDO 1.x mode = 0xFF (2F 01 14 02 FF 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x01, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x14:
      {
        /* Set RPDO 2.x mode = 0xFF (2F 02 14 02 FF 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x02, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x15:
      {
        /* Set RPDO 3.x mode = 0xFF (2F 03 14 02 FF 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x03, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;      
      
    case 0x16:
      {
        /* Enable RPDO 0.x (2F 00 16 00 02 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x00, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;  
      
    case 0x17:
      {
        /* Enable RPDO 1.x (2F 01 16 00 02 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x01, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x18:
      {
        /* Enable RPDO 2.x (2F 02 16 00 02 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x02, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x19:
      {
        /* Enable RPDO 3.x (2F 03 16 00 00 02 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x03, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;     
      
    case 0x1A:
      {
        /* Turn On RPDO 0.x (23 00 14 01 srvCanOpenId 02 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x00, 0x14, 0x01, (uint8_t)srvCanOpenId, 0x02, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;  
      
    case 0x1B:
      {
        /* Turn On RPDO 1.x (23 01 14 01 srvCanOpenId 03 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x01, 0x14, 0x01, (uint8_t)srvCanOpenId, 0x03, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x1C:
      {
        /* Turn On RPDO 2.x (23 02 14 01 srvCanOpenId 04 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x02, 0x14, 0x01, (uint8_t)srvCanOpenId, 0x04, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x1D:
      {
        /* Turn On RPDO 3.x (23 03 14 01 srvCanOpenId 05 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x03, 0x14, 0x01, (uint8_t)srvCanOpenId, 0x05, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;      
      
      // ==< /RPDOs >==      
      
      // ==< TPDO #s >==
      
    case 0x1E:
      {
        /* Clear TPDO #0.x mapping (2F 00 1A 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;  
      
    case 0x1F://0x20
      {
        /* Clear TPDO #1.x mapping (2F 01 1A 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x20:
      {
        /* Clear TPDO #2.x mapping (2F 02 1A 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x02, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x21:
      {
        /* Clear TPDO #3.x mapping (2F 03 1A 00 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x03, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x22:
      {
        /* Turn Off TPDO #0.x (23 00 18 01 (0x80 + srvCanOpenId) 01 00 80) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x00, 0x18, 0x01, (0x80 + (uint8_t)srvCanOpenId), 0x01, 0x00, 0x80);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;  
      
    case 0x23:
      {
        /* Turn Off TPDO #1.x (23 01 18 01 (0x80 + (uint8_t)srvCanOpenId) 02 00 80) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x01, 0x18, 0x01, (0x80 + (uint8_t)srvCanOpenId), 0x02, 0x00, 0x80);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x24:
      {
        /* Turn Off TPDO #2.x (23 02 18 01 (0x80 + (uint8_t)srvCanOpenId) 03 00 80) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x02, 0x18, 0x01, (0x80 + (uint8_t)srvCanOpenId), 0x03, 0x00, 0x80);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x25:
      {
        /* Turn Off TPDO #3.x (23 03 18 01 (0x80 + (uint8_t)srvCanOpenId) 04 00 80) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x03, 0x18, 0x01, (0x80 + (uint8_t)srvCanOpenId), 0x04, 0x00, 0x80);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x26:
      {
        /* Set TPDO #0.1 mapping = StatusWord(23 00 1A 01 10 00 41 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x27:
      {
        /* Set TPDO #0.2 mapping = ControlWord(23 00 1A 02 10 00 40 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x00, 0x1A, 0x02, 0x10, 0x00, 0x40, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x28:
      {
        /* Set TPDO #0.3 mapping = DIs State (23 00 1A 03 10 00 07 24) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x00, 0x1A, 0x03, 0x10, 0x00, 0x07, 0x24);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x29:
      {
        /* Set TPDO #1.1 mapping = Profile Velocity(23 01 1A 01 20 00 81 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x01, 0x1A, 0x01, 0x20, 0x00, 0x81, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;      
      
    case 0x2A:
      {
        /* Set TPDO #1.2 mapping = Profile QuickStopAcceleration(23 01 1A 02 20 00 85 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x01, 0x1A, 0x02, 0x20, 0x00, 0x85, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x2B:
      {
        /* Set TPDO #2.1 mapping = Profile Acceleration(23 02 1A 01 20 00 83 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x02, 0x1A, 0x01, 0x20, 0x00, 0x83, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;      
      
    case 0x2C:
      {
        /* Set TPDO #2.2 mapping = Profile Deceleration(23 02 1A 02 20 00 84 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x02, 0x1A, 0x02, 0x20, 0x00, 0x84, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x2D:
      {
        /* Set TPDO #3.1 mapping = Current Position (23 03 1A 01 20 00 64 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x03, 0x1A, 0x01, 0x20, 0x00, 0x64, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;      
      
    case 0x2E:
      {
        /* Set TPDO #3.2 mapping = DIs Actual State (23 03 1A 02 20 00 FD 60) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x3, 0x1A, 0x02, 0x20, 0x00, 0xFD, 0x60);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;      
      
    case 0x2F://0x30
      {
        /* Set TPDO #0.x mode = 0xFF (2F 00 18 02 FF 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x00, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;  
      
    case 0x30:
      {
        /* Set TPDO #1.x mode = 0x01 (2F 01 18 02 01 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x01, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x31:
      {
        /* Set TPDO #2.x mode = 0x01 (2F 02 18 02 01 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x02, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x32:
      {
        /* Set TPDO #3.x mode = 0x01 (2F 03 18 02 01 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x03, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x33:
      {
        /* Enable TPDO #0.x (2F 00 1A 00 03 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x00, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;  
      
    case 0x34:
      {
        /* Enable TPDO #1.x (2F 01 1A 00 02 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x01, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x35:
      {
        /* Enable TPDO #2.x (2F 02 1A 00 02 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x02, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x36:
      {
        /* Enable RPDO 3.x (2F 03 1A 00 00 02 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x03, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x37:
      {
        /* Turn On TPDO #0.x (23 00 18 01 (0x80 + srvCanOpenId) 01 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x00, 0x18, 0x01, (0x80 + (uint8_t)srvCanOpenId), 0x01, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;  
      
    case 0x38:
      {
        /* Turn On TPDO #1.x (23 01 18 01 (0x80 + (uint8_t)srvCanOpenId) 02 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x01, 0x18, 0x01, (0x80 + (uint8_t)srvCanOpenId), 0x02, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x39:
      {
        /* Turn On TPDO #2.x (23 02 18 01 (0x80 + (uint8_t)srvCanOpenId) 03 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x02, 0x18, 0x01, (0x80 + (uint8_t)srvCanOpenId), 0x03, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x3A:
      {
        /* Turn On TPDO #3.x (23 03 18 01 (0x80 + (uint8_t)srvCanOpenId) 04 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x23, 0x03, 0x18, 0x01, (0x80 + (uint8_t)srvCanOpenId), 0x04, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;                        
      // ==< /TPDO #s >==       
      
    case 0x3B:
      {
        /* Send OperationMode = Profile Position (2F 60 60 00 01 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x2F, 0x60, 0x60, 0x00, (srvDefOM & 0xFF), ((srvDefOM & 0xFF00) >> 8), ((srvDefOM & 0xFF0000) >> 16), ((srvDefOM & 0xFF000000) >> 24));
        fNeedToGetParamValue = 0x00006060;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x3C:
      {
        /* Send StartRemote Cmd (01 03 00 00 00 00 00 00) */
        SRV_TRANSFER_8(0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        srvStartRemoteFlag = 0x01;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x3D:
      {
        /* Send RPDO 0.1 = 06 & RPDO 0.2 = 0x00000000 (06 00 00 00 00 00 00 00) */
        SRV_TRANSFER_8(0x200 + srvCanOpenId, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
        
        //        srvReadyToSwitchOnFlag = 0x01;
        
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x3E:
      {
        /* Read srvDI01Func Value (40 0A 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x0A, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x3F://0x40
      {
        /* Read srvDI02Func Value (40 0B 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x0B, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x40:
      {
        /* Read srvDI03Func Value (40 0C 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x0C, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x41:
      {
        /* Read srvDI04Func Value (40 0D 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x0D, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x42:
      {
        /* Read srvDI05Func Value (40 0E 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x0E, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x43:
      {
        /* Read srvDI06Func Value (40 0F 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x0F, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x44:
      {
        /* Read srvDI07Func Value (40 10 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x10, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x45:
      {
        /* Read srvDI08Func Value (40 11 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x11, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x46:
      {
        /* Read srvEDI09Func Value (40 24 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x24, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x47:
      {
        /* Read srvEDI10Func Value (40 25 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x25, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x48:
      {
        /* Read srvEDI11Func Value (40 26 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x26, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x49:
      {
        /* Read srvEDI12Func Value (40 27 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x27, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x4A:
      {
        /* Read srvEDI13Func Value (40 28 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x28, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x4B:
      {
        /* Read srvEDI14Func Value (40 29 22 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x29, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x4C:
      {
        /* Read DI Current State (40 07 24 00 00 00 00 00) */
        SRV_TRANSFER_8(0x600 + srvCanOpenId, 0x40, 0x07, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00);
        fNeedToGetParamValue = 0x00000000;
        
        //        printf("here we are: %04X \n\r", procStep);
        
        procStep = 0xFF;
      }
      break;
      
    case 0x4D:
      {          
        /* Send RPDO 0.1 = 4F & RPDO 0.2 = 0x00000000 (4F 00 00 00 00 00 00 00) */
        SRV_TRANSFER_8(0x200 + srvCanOpenId, 0x4F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
        
        fNeedToGetParamValue = 0x00000000;        
        
        procStep = 0xFF;
      }
      break;
      
    case 0x4E:
      {
        /* Send RPDO 1.1 = srvDefVel &  RPDO 1.2 = srvDefQuickStopDec (srvDefVel srvDefVel srvDefVel srvDefVel srvDefQuickStopDec srvDefQuickStopDec srvDefQuickStopDec srvDefQuickStopDec) */
        SRV_TRANSFER_8(0x300 + srvCanOpenId, (uint8_t)(srvDefVel & 0xFF), (uint8_t)((srvDefVel & 0xFF00) >> 8), (uint8_t)((srvDefVel & 0xFF0000) >> 16), (uint8_t)((srvDefVel & 0xFF000000) >> 24), (uint8_t)(srvDefQuickStopDec & 0xFF), (uint8_t)((srvDefQuickStopDec & 0xFF00) >> 8), (uint8_t)((srvDefQuickStopDec & 0xFF0000) >> 16), (uint8_t)((srvDefQuickStopDec & 0xFF000000) >> 24));
        
        /* Send RPDO 2.1 = srvDefAcc &  RPDO 2.2 = srvDefDec (srvDefAcc srvDefAcc 00 00 srvDefDec srvDefDec 00 00) */
        SRV_TRANSFER_8(0x400 + srvCanOpenId, (uint8_t)(srvDefAcc & 0xFF), (uint8_t)((srvDefAcc & 0xFF00) >> 8), (uint8_t)((srvDefAcc & 0xFF0000) >> 16), (uint8_t)((srvDefAcc & 0xFF000000) >> 24), (uint8_t)(srvDefDec & 0xFF), (uint8_t)((srvDefDec & 0xFF00) >> 8), (uint8_t)((srvDefDec & 0xFF0000) >> 16), (uint8_t)((srvDefDec & 0xFF000000) >> 24));
        
        
        /* Send RPDO 0.1 = 5F & RPDO 0.2 = 0x00000000 (5F 00 00 00 00 00 00 00) */
        SRV_TRANSFER_8(0x200 + srvCanOpenId, 0x5F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);                                   
        
        fNeedToGetParamValue = 0x00000000;                
        
        procStep = 0xFF;
      }
      break; 
      
    case 0x4F:
      {               
        /* Send SYNC cmd (00 00 00 00 00 00 00 00) */
        SRV_TRANSFER_8(0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);            
        
        srvTPDO1AnswerFlag = 0x00;
        srvTPDO2AnswerFlag = 0x00;
        srvTPDO1AnswerFlag = 0x00;
        
        fNeedToGetParamValue = 0x00000000;
        
        procStep = 0xFF;
      }
      break;           
      
    case 0x50:
      {         
        if((srvTPDO1AnswerFlag == 0x01) && (srvTPDO2AnswerFlag == 0x01) && (srvTPDO3AnswerFlag == 0x01))
        {
          if((srvCurrVel == srvDefVel) && (srvCurrQuickStopDec == srvDefQuickStopDec) && (srvCurrAcc == srvDefDec) && (srvCurrTargetReachedBit == 1))
          {            
            srvCurrFuncMode = idleFuncMode;
          }
        }               
        
        procStep = 0xFF;        
      }
      break;
    } //=< /switch(procStep) >=
    
  } //=< /if((srvCurrMode == noneFuncMode) && (srvCurrMode == initFuncMode)) >=
}

void srvSendCmd(bufStruct srvCmdData)
{        
  SRV_TRANSFER_8((uint32_t)(((uint16_t)srvCmdData.arr[0] << 8) + (uint16_t)srvCmdData.arr[1]), srvCmdData.arr[2], srvCmdData.arr[3], srvCmdData.arr[4], srvCmdData.arr[5], srvCmdData.arr[6], srvCmdData.arr[7], srvCmdData.arr[8], srvCmdData.arr[9]);
  
  if(SRV_resultTx == CAN_TxStatus_NoMailBox) //если при отправке сообщения нет свободного почтового CAN-ящика
  {
    srvSkipBuffReadingFlag = 1; //ставим флаг запрета ЧТЕНИЯ из буфера (запись в буфер по прежнему возможна)
    
    readBufDataResult = 1; // ставим признак того, что у нас есть считанные из буфера данные (но мы не можем их отправить - ящик занят)
  }
  else // если ящик не переполняется
  {
    srvSkipBuffReadingFlag = 0; // продолжаем читать из буфера
    
    readBufDataResult = 0; // ставим признак, что у нас нет данных из буфера для отправки в привод и нужно снова заглянуть в буфер - вдруг данные там появились)
  }                       
}

static void GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable the GPIOC, GPIOD, GPIOE peripheral */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* Configure MCO2 pin(PC9) in alternate function */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* SYSCLK/4 clock selected to output on MCO2 pin(PC9)*/
  RCC_MCO2Config(RCC_MCO2Source_SYSCLK, RCC_MCO2Div_4);
  
  /* Configure outputs PE2 - PE4 - SUN_CAN, SRV_CAN & Status LEDs */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  /*  Configure address input pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  /*  Configure input pin PE6 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  moduleAddress = GetAddress();
}

static void CAN_Config(void)
{
  // CAN1 - SUNCAN
  // CAN2 - CANOpen
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  
  /********************** CAN GPIOs configuration *****************************/
  
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);// CAN2_RX
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);// CAN2_TX
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);// CAN1_RX
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);// CAN1_TX
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* CAN configuration ********************************************************/
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);
  
  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = ENABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  
  /* CAN Baudrate = 500 kBps (CAN clocked at 45 MHz) */
  /*To get 500kbps: [45MHz(max freq of APB1 bus) / 10(CAN_Prescaler)] / [1(CAN_SJW) + 3(CAN_BS1) + 5(CAN_BS2)] = 0.5MHz(i.e. 500kbps)*/
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStructure.CAN_Prescaler = 10;
  
  CAN_Init(CAN1, &CAN_InitStructure);
  CAN_Init(CAN2, &CAN_InitStructure);
  
  CAN_FilterInitStructure.CAN_FilterNumber = 0; // CAN1 [0..13] - SUNCAN
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = (moduleAddress<<3)&0xFFF8;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFF8;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  //  /* CAN filter init - All incoming IDs are allowed */
  //  CAN_FilterInitStructure.CAN_FilterNumber = 1; // CAN1 [0..13] - SUNCAN
  //  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  //  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  //  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  //  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  //  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  //  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFF8;
  //  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
  //  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  //
  //  CAN_FilterInit(&CAN_FilterInitStructure);
  
  
  /* CAN filter init - All incoming IDs are allowed */
  CAN_FilterInitStructure.CAN_FilterNumber = 14; // CAN2 [14..27] - CANOpen
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0; //0 - было
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  
  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
  
  CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);
  
  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(CAN2, CAN_IT_FMP1, ENABLE);
  
  CAN_ITConfig(CAN2, CAN_IT_TME, ENABLE);
}

static void NVIC_Config(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  /* CAN_1 */
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* CAN_2 */
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x5;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*** = = = - - - </ User Functions >- - - = = = ***/

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
