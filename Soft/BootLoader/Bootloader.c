/*****************************************************************************
BootLoader для платы ASICanConverter на MCU STM32F437VIT6.
Тип MCU задается через #define.
Версия загрузчика 2.x. Версия загрузчика обозначается мажорной и
минорной частями. Загрузчик должен быть совместим с application.
Данная проверка выполняется в PC-приложении при загрузки прошивки по
шине CAN.
Установить текущую аппаратную версию в переменной 'bootHardware',
текущую программную версию загрузчика в переменной 'bootFirmware'
SVN: $Revision:$
Compiler: IAR C/C++ Compiler for ARM 5.30.0.51174 (5.30.0.51174)
*****************************************************************************/

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_can.h"
#include "SUN_CAN.h"
#include "canbus.h"
#include "version.h"

#define GPIO_IN           ((uint32_t)0x00)
#define GPIO_OUT          ((uint32_t)0x01)
#define GPIO_AF           ((uint32_t)0x02)
#define GPIO_ANALOG       ((uint32_t)0x03)


#define Shift_Pin_0   0
#define Shift_Pin_1   4
#define Shift_Pin_2   8
#define Shift_Pin_3   12
#define Shift_Pin_4   16
#define Shift_Pin_5   20
#define Shift_Pin_6   24
#define Shift_Pin_7   28
#define Shift_Pin_8   0
#define Shift_Pin_9   4
#define Shift_Pin_10  8
#define Shift_Pin_11  12
#define Shift_Pin_12  16
#define Shift_Pin_13  20
#define Shift_Pin_14  24
#define Shift_Pin_15  28


/* CAN Master Control Register bits */
#define MCR_INRQ     ((uint32_t)0x00000001) /* Initialization request */
#define MCR_SLEEP    ((uint32_t)0x00000002) /* Sleep mode request */
#define MCR_TXFP     ((uint32_t)0x00000004) /* Transmit FIFO priority */
#define MCR_RFLM     ((uint32_t)0x00000008) /* Receive FIFO locked mode */
#define MCR_NART     ((uint32_t)0x00000010) /* No automatic retransmission */
#define MCR_AWUM     ((uint32_t)0x00000020) /* Automatic wake up mode */
#define MCR_ABOM     ((uint32_t)0x00000040) /* Automatic bus-off management */
#define MCR_TTCM     ((uint32_t)0x00000080) /* time triggered communication */
#define MCR_RESET    ((uint32_t)0x00008000) /* time triggered communication */
#define MCR_DBF      ((uint32_t)0x00010000) /* software master reset */

/* CAN Master Status Register bits */
#define MSR_INAK     ((uint32_t)0x00000001)    /* Initialization acknowledge */
#define MSR_WKUI     ((uint32_t)0x00000008)    /* Wake-up interrupt */
#define MSR_SLAKI    ((uint32_t)0x00000010)    /* Sleep acknowledge interrupt */
/* CAN Filter Master Register bits */
#define FMR_FINIT    ((uint32_t)0x00000001) /* Filter init mode */

/* Настройки bootloader. Они вынесены в область памяти flash, таким образом можно менять
значение с помощью утилиты SRecord без компиляции проекта. */
typedef struct  {
  uint32_t  manufId;            /* Код производителя. Для STMicroelectronics = 2*/
  uint32_t  mcuId;              /* Код микроконтроллера */
  uint32_t  flashSize2N;        /* Объем flash памяти в степени 2 */
  uint32_t  flashPageSize2N;    /* Размер страницы flash памяти в формате степени двойки. */
  uint32_t  flashPageSize;      /* Размер страницы, в байтах */
  uint32_t  maxFlashSizeToPage; /* Максимальный адрес flash памяти для данного MCU */
  uint32_t  dataAdress;         /* Адрес блока информации о приложении:
  dataAdress+0:  указатель на flash память, где хранится CRC32 Application;
  dataAdress+4:  адрес размещения startup функции */
}SettingsStruct;

#define POLY 0x04C11DB7

#define RCC_AHB1Periph_GPIO_CAN    RCC_AHB1Periph_GPIOA
#define GPIO_CAN                   GPIOA
#define GPIO_Pin_CAN_RX            GPIO_Pin_11
#define GPIO_Pin_CAN_TX            GPIO_Pin_12

/* Это версия bootFirmware, преобразованная в код BCD. 
BOOT_FIRMWARE_FMT =  (bootFirmware&0x0000000F) | ((bootFirmware>>4)&0x000000F0)
*/
#define BOOT_FIRMWARE_FMT  (0x20)

#define ERR_SUCC        0
#define ERR_FLASH_PRG   1
#define ERR_FLASH_VER   2
#define ERR_CMD_FAIL    3

typedef  void (*funcptr)(void);
funcptr Application;
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
RCC_ClocksTypeDef  RCC_Clocks;

extern uint32_t __checksum;
__no_init CanTxMsg Can_Tx;
__no_init CanRxMsg  Can_Box[CAN_SIZE];
__no_init uint32_t  moduleAddress;

/* Выбрать конфугурацию для требуемого MCU. 
Выбор осуществляется через глобальный define в настройках IDE IAR */
/* Конфигурация для STM32F437VIT6 */
#if defined (STM32F437VIT6)
#define FLASH_PAGE_SIZE         (1024)
#warning "STM32F437VIT6!"
#pragma location="flash_Settings"
__root const SettingsStruct settings = {
  .manufId = 0x00000002,
  .mcuId = 0x00000001,
  .flashSize2N = 17,
  .flashPageSize2N = 10,
  .flashPageSize = (uint32_t)FLASH_PAGE_SIZE,
  .maxFlashSizeToPage = 0x0801FFFF,
  .dataAdress = 0x08001000,
};
#else 
#warning "!!=== MCU Core is undefined ===!!"
#endif


SettingsStruct *config = (SettingsStruct*)&settings;

#pragma location = "TARGET"
__no_init __root TargetStruct target;

typedef union{
  uint32_t  word[FLASH_PAGE_SIZE/4];
  uint16_t  halfWord[FLASH_PAGE_SIZE/2];
  uint8_t   byte[FLASH_PAGE_SIZE];
}BufferUnion;
__no_init BufferUnion buffer;
__no_init uint32_t Key[4];
__no_init uint8_t key_i, key_j;
__no_init uint8_t S[256];

uint8_t WriteFlashPage(uint32_t flashStartAdr, uint8_t *dataPage);
void    rc4_init  (uint8_t *key);
uint8_t rc4_output(void);

uint8_t canTransmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);

// note remember to feed size (4 for crc32) zeroes through the slow
// crc algorithm to make the fast and the slow compute the same checksum
uint32_t  slow_crc32(uint32_t sum, uint8_t *p, uint32_t len)
{
  while (len--)
  {
    uint16_t i;
    uint8_t byte = *p++;
    
    for (i = 0; i < 8; ++i)
    {
      uint32_t osum = sum;
      sum <<= 1;
      if (byte & 0x80)
        sum |= 1 ;
      if (osum & 0x80000000)
        sum ^= POLY;
      byte <<= 1;
    }
  }
  return sum;
}

uint32_t checkCrc32(uint32_t *crc32)
{
  uint32_t *pchecksum;
  uint32_t dummy32;
  uint32_t zero=0;
  
#ifdef NDEBUG
  pchecksum = (uint32_t *)(*((uint32_t *)(config->dataAdress)));
  if(pchecksum > (uint32_t *)(config->maxFlashSizeToPage-3) ) return 0;  
  if ((uint32_t)pchecksum < (uint32_t)(*((uint32_t *)(config->dataAdress + 4)))) return 0; 
  
  if ((uint32_t)(*((uint32_t *)(config->dataAdress + 4))) > (config->maxFlashSizeToPage-3)) return 0;          
  if ((uint32_t)(*((uint32_t *)(config->dataAdress + 4))) < 0x08001000) return 0;
#else
  pchecksum = &__checksum;
#endif
  dummy32 = (uint32_t)0xFFFFFFFF;
  dummy32 = slow_crc32(dummy32, (uint8_t *) (*((uint32_t *)(config->dataAdress + 4))), ((uint32_t)pchecksum - (uint32_t) (*((uint32_t *)(config->dataAdress + 4 )))) );
  dummy32 = slow_crc32(dummy32, (uint8_t *)&zero,4);
  *crc32 = dummy32;
  if(dummy32 != (*pchecksum)) return 0;
  else return 1;
}

#pragma inline=forced
uint32_t CAN_Config(void)
{
  uint32_t wait_ack = 0x00000000;
  uint32_t filter_number_bit_pos;
  uint8_t InitStatus = CANINITFAILED;
  /* CAN register init */
  RCC->APB1RSTR |= RCC_APB1Periph_CAN1;
  RCC->APB1RSTR &= ~RCC_APB1Periph_CAN1;
  
  CAN1->MCR &= ~MCR_SLEEP;  /* exit from sleep mode */
  CAN1->MCR |= MCR_INRQ ; /* Request initialisation */
  /* Wait the acknowledge */
  while (((CAN1->MSR & MSR_INAK) != MSR_INAK) && (wait_ack != ((uint32_t)0x0000FFFF))){
    wait_ack++;
  }
  /* ...and check acknowledged */
  if ((CAN1->MSR & MSR_INAK) == MSR_INAK){
    CAN1->MCR &= 0x00010001;
    /* Set the bit timing register */
    CAN1->BTR = ((uint32_t)0x02 << 16) | ((uint32_t)0x04 << 20) | ((uint32_t)8 - 1);
    
    /* Request leave initialisation */
    CAN1->MCR &= ~MCR_INRQ;
    
    /* Wait the acknowledge */
    wait_ack = 0x00;
    while (((CAN1->MSR & MSR_INAK) == MSR_INAK) && (wait_ack != ((uint32_t)0x0000FFFF))){
      wait_ack++;
    }
    /* ...and check acknowledged */
    if ((CAN1->MSR & MSR_INAK) != MSR_INAK){
      InitStatus = CANINITOK ;
      filter_number_bit_pos = (uint32_t)0x00000007;
      CAN1->FMR |= FMR_FINIT; /* Initialization mode for the filter */
      CAN1->FA1R &= ~(uint32_t)filter_number_bit_pos; /* Filter Deactivation */
      CAN1->FS1R |= filter_number_bit_pos;  /* 32-bit scale for the filter */
      /* Set filter for moduleAddress */
      /* 32-bit identifier or First 32-bit identifier */
      CAN1->sFilterRegister[0].FR1 = (0x1FFFFFF8 & (uint32_t)((0x00026000|moduleAddress)<<3)); // фильтр #0 на получение сообщений от аплодера (CAN_ID = 0x13, здесь это 0x00026000) к устройству (CAN_ID = moduleAddress (0x71/0x72) - зависит от положения движков КАН-адресного переключателя)
      /* 32-bit mask or Second 32-bit identifier */
      CAN1->sFilterRegister[0].FR2 = (uint32_t)0x1FFFFFF8; // маска фильтра #0
      /* Set filter for ID_DRIVER_ZLR */
      /* 32-bit identifier or First 32-bit identifier */
      CAN1->sFilterRegister[1].FR1 = (0x1FFFFFF8 & (uint32_t)((0x00026000|ID_DRIVER_ZLR)<<3)); // фильтр #1 на получение сообщений от аплодера (CAN_ID = 0x13, здесь это 0x00026000) к устройству по его DEV_ID(для УИУ 2.0 DEV_ID = 0x71)
      /* 32-bit mask or Second 32-bit identifier */
      CAN1->sFilterRegister[1].FR2 = (uint32_t)0x1FFFFFF8; // маска фильтра #1
      /* Set filter for ID_BROADCAST */
      /* 32-bit identifier or First 32-bit identifier */
      CAN1->sFilterRegister[2].FR1 = (0x1FFFFFF8 & (uint32_t)((0x00026000|ID_BROADCAST)<<3)); // фильтр #2 на получение сообщений от аплодера (CAN_ID = 0x13, здесь это 0x00026000) к устройству по общему КАН-адресу (CAN_ID = ID_BROADCAST)
      /* 32-bit mask or Second 32-bit identifier */
      CAN1->sFilterRegister[2].FR2 = (uint32_t)0x1FFFFFF8; // маска фильтра #2
      /*Id/Mask mode for the filter*/
      CAN1->FM1R &= ~(uint32_t)filter_number_bit_pos;
      /* FIFO 0 assignation for the filter */
      CAN1->FFA1R &= ~(uint32_t)filter_number_bit_pos;
      /* Filter activation */
      CAN1->FA1R |= filter_number_bit_pos;
      /* Leave the initialisation mode for the filter */
      CAN1->FMR &= ~FMR_FINIT;
    } 
  }
  return InitStatus;
}

#pragma inline=forced
void RCC_Config(void)
{
  uint32_t tmpreg = 0;
  
  /* Check the parameters */  
  tmpreg = RCC->CFGR;
  
  /* Clear RCC->CFGR[15:13] bits */
  tmpreg &= ((uint32_t)0xFFFF1FFF);
  
  /* Set RCC->CFGR[15:13] bits */
  tmpreg |= RCC_SYSCLK_Div8 ;
  
  /* Store the new value */
  RCC->CFGR = tmpreg;
  
  
  /* GPIO clock enable */  
  RCC->AHB1ENR = RCC_AHB1Periph_GPIO_CAN|RCC_AHB1Periph_GPIOA|
    RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC;
  
  /* CAN1 Periph clock enable */
  RCC->APB1ENR = RCC_APB1Periph_CAN1;
}

#pragma inline=forced
void GPIO_Config(void)
{//(http://zx-pk.ru/threads/23308-mikrokontrollery-stm32-quot-s-nulya-quot.html?p=707914&viewfull=1#post707914)
  
  //Set B8 = CAN1_RX and B9 = CAN1_TX
  GPIOB->AFR[1] |= (GPIO_AF_CAN1 << (4 * 0)) | (GPIO_AF_CAN1 << (4 * 1));
  
  
}


#pragma section = "SECTION_SERIAL"
#pragma location= "SECTION_SERIAL"
#pragma optimize=no_inline
__root uint32_t  GetSerialNum(void)
{ 
  return (*(__IO uint32_t*)0x08000D00); //Bootloader *.icf file & version.h
}

#pragma section = "SECTION_SERIAL"
#pragma location= "SECTION_SERIAL"
#pragma optimize=no_inline
__root uint32_t  GetHardVersion(void)
{ 
  return (*(__IO uint32_t*)0x08000D04); //Bootloader *.icf file & version.h
}

#pragma section = "SECTION_SERIAL"
#pragma location= "SECTION_SERIAL"
#pragma optimize=no_inline
__root uint16_t  GetAddress(void)
{
  // делаем инверсию, т.к. при установке движка адресного переключателя в пололжение ON
  // в соответствии с обозначением на корпусе переключателя замыкаются контакты соответствующего переключателя, при этом один из контактов переключателя
  // подклюен к GND, а второй подтянут через резистор к Vcc и к адресным пинам контроллера. Таким образом, при положении выключателя в положение OFF на пине контроллера имеем Vcc (лог.1),
  return (((~((uint16_t)GPIOA->IDR>>4)) & 0x000F)|0x0070);
}


uint8_t canTransmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage)
{
  if ( CAN1->TSR&0x04000000 )
  {
    /* Set up the Id */
    CAN1->sTxMailBox[0].TIR = ((uint32_t)0x00000000);
    
    CAN1->sTxMailBox[0].TIR |= ((TxMessage->ExtId<<3) | 0x04 );
    
    /* Set up the DLC */
    CAN1->sTxMailBox[0].TDTR &= (uint32_t)0xFFFFFFF0;
    CAN1->sTxMailBox[0].TDTR |= TxMessage->DLC;
    
    /* Set up the data field */
    CAN1->sTxMailBox[0].TDLR = (((uint32_t)TxMessage->Data[3] << 24) | 
                                ((uint32_t)TxMessage->Data[2] << 16) |
                                 ((uint32_t)TxMessage->Data[1] << 8) | 
                                    ((uint32_t)TxMessage->Data[0]));
    
    CAN1->sTxMailBox[0].TDHR = (((uint32_t)TxMessage->Data[7] << 24) | 
                                ((uint32_t)TxMessage->Data[6] << 16) |
                                 ((uint32_t)TxMessage->Data[5] << 8) |
                                    ((uint32_t)TxMessage->Data[4]));
    
    /* Request transmission */
    CAN1->sTxMailBox[0].TIR |= ((uint32_t)0x00000001);
    
    while(!(CAN1->TSR & (1<<1)))
    {
//    GPIOA->BSRR |= GPIO_BSRR_BS3; //ProcLED on
    }
//    GPIOA->BRR |= GPIO_BRR_BR3; //ProcLED off
        
    return 0;
  } 
  else return CAN_NO_MB;
}

void canReceive(CanRxMsg* RxMessage)
{
  /* Get the Id */
  RxMessage->IDE = (uint8_t)0x04 & CAN1->sFIFOMailBox[0].RIR;
  RxMessage->ExtId = (uint32_t)0x1FFFFFFF & (CAN1->sFIFOMailBox[0].RIR >> 3);
  
  /* Get the DLC */
  RxMessage->DLC = (uint8_t)0x0F & CAN1->sFIFOMailBox[0].RDTR;
  /* Get the data field */
  RxMessage->Data[0] = (uint8_t)0xFF & CAN1->sFIFOMailBox[0].RDLR;
  RxMessage->Data[1] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR >> 8);
  RxMessage->Data[2] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR >> 16);
  RxMessage->Data[3] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR >> 24);
  RxMessage->Data[4] = (uint8_t)0xFF & CAN1->sFIFOMailBox[0].RDHR;
  RxMessage->Data[5] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR >> 8);
  RxMessage->Data[6] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR >> 16);
  RxMessage->Data[7] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR >> 24);
  /* Release the FIFO */
  CAN1->RF0R = (uint32_t)0x00000020;  
}

__task void main(void)
{
  uint32_t temp = 0;
  uint16_t source_id, i;
  uint32_t dummy32, address, writeAddress;  
  register uint32_t RxHead = 0;
  register uint32_t RxTail = 0;
  CanRxMsg  *ptrBox;
  
  /* Setup STM32 system (clock, PLL and Flash configuration) */
  SystemInit();
  /* System clocks configuration ---------------------------------------------*/
  RCC_Config();
  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Config();
  /* 
  --------------------------------------------------------------------------------------------------------------
  |    31    |    30    |    29    |   28    |    27   |   26    |    25    |  24  |  23 - 2  |   1    |   0   |  
  --------------------------------------------------------------------------------------------------------------
  | LPWRRSTF | WWDGRSTF | IWDGRSTF | SFTRSTF | PORRSTF | PINRSTF | Reserved | RMVF | Reserved | LSIRDY | LSION |
  --------------------------------------------------------------------------------------------------------------
  
  Биты с именами вида xRSTF являются флагами, несущими информацию о причине сброса микроконтроллера. Установка в 1 происходит аппаратно после сброса микроконтроллера по соответствующей причине, для сброса флагов требуется запись 1 в бит RMVF.
  
  LPWRRSTF: Low-power reset flag - флаг сброса в результате падения напряжения ниже заданного значения (сброс от системы Low-power management).
  
  WWDGRSTF: Window watchdog reset flag - флаг устанавливается при сбросе от оконного сторожевого таймера.
  
  IWDGRSTF: Independent watchdog reset flag - флаг устанавливается при сбросе от независимого сторожевого таймера.
  
  SFTRSTF: Software reset flag - флаг устанавливается при программном сбросе.
  
  PORRSTF: POR/PDR reset flag - флаг устанавливается при POR/PDR сбросе (Power-on/power-down, сброс при включении питания).
  
  PINRSTF: PIN reset flag - флаг устанавливается при сбросе импульсом сброса на выводе NRST микроконтроллера.
  
  RMVF: Remove reset flag - флаг для сброса всех флагов xRSTF; сброс флагов происходит при записи 1 в этот бит.
  
  LSIRDY: Internal low-speed oscillator ready - флаг готовности внутреннего низкочастотного RC-генератора на 40 кГц LSI. Устанавливается и сбрасывается аппаратно, значение 1 указывает на готовность генератора. После сброса бита LSION требуется 3 такта 40 кГц генератора для перехода бита LSIRDY к значению 0.
  
  LSION: Internal low-speed oscillator enable - бит устанавливается и сбрасывается программно; значение 1 включает внутренний RC-генератор LSI с частотой 40 кГц.*/
  
  temp = RCC->CSR&0xFC000000;
  RCC->CSR |= ((uint32_t)0x01000000); //set 24th bit(RCC->CSR.RMVF = 1) (RMVF: Remove reset flag - флаг для сброса всех флагов xRSTF; сброс флагов происходит при записи 1 в этот бит.)
  
  if( temp == 0x0C000000) 
  {
    target.mcuStatus = INFO_POWER;
  }
  else
  {
    if(( temp == 0x04000000) || ( temp == 0x14000000))
    {
      target.mcuStatus = INFO_RESET;
    } 
    else
    {
      if( temp == 0x24000000) 
      {
        target.mcuStatus = INFO_WDOG;
      } 
      else 
        target.mcuStatus = 0;
    }
  }
  moduleAddress = GetAddress();
  
  /* CAN configuration */
  CAN_Config();
  
  Can_Tx.IDE = CAN_ID_EXT;
  Can_Tx.RTR = CAN_RTR_DATA;
  
  __disable_irq();
  
  if( target.mcuStatus ) 
  {
    if(!checkCrc32(&dummy32)) 
    {
      dummy32 = GetHardVersion();
      dummy32 = ((dummy32&0x000F0000)>>12) | (dummy32&0x0000000F);
      TX_SET8(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|((uint32_t)ID_BROADCAST), TM_STATE_4, CH_BOOTLOADER,
              (uint8_t)EC_INFO, (uint8_t)(EC_INFO>>8),(uint8_t)ID_DRIVER_ZLR, (uint8_t)dummy32, (uint8_t)((bootFirmware&0x0000000F) | ((bootFirmware>>4)&0x000000F0)), target.mcuStatus);
    } 
    else 
    {
      target.command = 0;
      
      Application = (funcptr)(*(uint32_t*)((*((uint32_t*)(config->dataAdress + 4)))+4)); // jump to main firmware code
      /* Initialize user application's Stack Pointer */
      __set_MSP((*(uint32_t*)((*((uint32_t*)(config->dataAdress + 4)))))); // jump to main firmware code
      Application(); // jump to main firmware code
    }
  } 
  else 
  {
    TX_SET4(PRIORITY_NORMAL|SOURCE_ID(moduleAddress)|(target.address), TM_EXECUTE_CMD_2, CH_BOOTLOADER, (uint8_t)CMD_BOOTRUN, (uint8_t)(CMD_BOOTRUN>>8));
  }
  
  canTransmit(CAN1, &Can_Tx);
  
  // --------------- [a004] ---------------------------
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG counter clock: LSI/8 */
  IWDG_SetPrescaler(IWDG_Prescaler_8);  // [m003]
  
  /* Set counter reload value to obtain 512ms IWDG TimeOut.*/
  IWDG_SetReload(2560);
  
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
#warning "IWDG is Activated!"
  IWDG_Enable();
  
  while( 1 ) 
  {        
    IWDG_ReloadCounter();    
    
    while(CAN1->RF0R&0x00000003) 
    {
      temp = (RxHead+1)&(CAN_SIZE-1);
      if(temp != RxTail)
      {
        canReceive(&Can_Box[RxHead]);
        RxHead++;
        RxHead &= (CAN_SIZE-1);
      } 
      else break;
    }
    
    if( RxTail != RxHead ) 
    {
      ptrBox = &Can_Box[RxTail];
      source_id = (ptrBox->ExtId>>13)&0x00001FFF;
      if(CHANNEL == CH_BOOTLOADER) 
      {
        /* С целью оптимизации загрузчика по размеру заполняю структуру CAN-сообщения
        для отправки. Дальше по мере анализа принятого CAN-сообщения редактирую
        некоторые элементы. */
        Can_Tx.ExtId = PRIORITY_HIGH|SOURCE_ID(moduleAddress)|(source_id); 
        Can_Tx.DLC = 8;
        Can_Tx.Data[1] = CHANNEL;
        Can_Tx.Data[2] = CMD_LOW;
        Can_Tx.Data[3] = CMD_HIGH;
        Can_Tx.Data[4] = DL0;
        Can_Tx.Data[5] = DL1;
        Can_Tx.Data[6] = DL2;
        Can_Tx.Data[7] = DL3;
        
        switch(TYPEMSG) 
        {   
        case TM_READ_4:
          Can_Tx.Data[0] = TM_ANSWER_4;
          temp = 0;
          switch(CMD) 
          {
          case AD_BOOTINFO:
            dummy32 = GetHardVersion();
            dummy32 = ((dummy32&0x000F0000)>>12) | (dummy32&0x0000000F);
            Can_Tx.Data[4] = ID_DRIVER_ZLR;
            Can_Tx.Data[5] = (uint8_t)dummy32;
            Can_Tx.Data[6] = (uint8_t)bootFirmware;
            Can_Tx.Data[7] = (uint8_t)(bootFirmware>>8);
            break;
            
          case AD_SERIALNUM:
            dummy32 = GetSerialNum();
            Can_Tx.Data[4] = (uint8_t)dummy32;
            Can_Tx.Data[5] = (uint8_t)(dummy32>>8);
            Can_Tx.Data[6] = (uint8_t)(uint8_t)(dummy32>>16);
            Can_Tx.Data[7] = (uint8_t)(dummy32>>24);
            break;
            
          case AD_CPU_ID: 
            Can_Tx.Data[4] = (uint8_t)(config->manufId);
            Can_Tx.Data[5] = (uint8_t)(config->mcuId);
            Can_Tx.Data[6] = (uint8_t)config->flashSize2N;
            Can_Tx.Data[7] = (uint8_t)(config->flashPageSize2N);
            break;
            
          case AD_CRC32:
            checkCrc32(&dummy32);
            Can_Tx.Data[4] = (uint8_t)dummy32;
            Can_Tx.Data[5] = (uint8_t)(dummy32>>8);
            Can_Tx.Data[6] = (uint8_t)(uint8_t)(dummy32>>16);
            Can_Tx.Data[7] = (uint8_t)(dummy32>>24);
            break;
            
          default:temp = 1;
          break;
          }
          if(!temp) 
          { 
            canTransmit(CAN1, &Can_Tx); 
          }
          break;
          
        case TM_CMD_ACK_6:
          switch(CMD)
          { 
          case CMD_BOOTEXIT:  
            Can_Tx.Data[0] = TM_ACK_CMD_6; 
            Can_Tx.Data[4] = (uint8_t)0x00;
            Can_Tx.Data[5] = (uint8_t)0x00;
            Can_Tx.Data[6] = (uint8_t)0x00;
            Can_Tx.Data[7] = (uint8_t)0x00;
            canTransmit(CAN1, &Can_Tx);
            
            target.address = source_id;
            target.command = CMD;
            
            __disable_irq;
            
            IWDG_ReloadCounter();
            
            FLASH_Lock();
            
            Application = (funcptr)(*(__IO uint32_t*)((*((__IO uint32_t*)(config->dataAdress + 4)))+4));
            /* Initialize user application's Stack Pointer */
            __set_MSP((*(__IO uint32_t*)((*((__IO uint32_t*)(config->dataAdress + 4))))));                           
            Application();
            break; 
            
          case CMD_PROGEEPROM:
            /* EEPROM programming in not supported ! */
            temp = ERR_CMD_FAIL;
            goto  err_prog;
            
          case CMD_BOOTPROG:
            address = config->dataAdress + DL32;
            Can_Tx.Data[0] = TM_ACK_CMD_6;
            canTransmit(CAN1, &Can_Tx);
            Key[0] = KEY0;
            Key[1] = KEY1;
            Key[2] = KEY2;
            Key[3] = DL32;
            rc4_init((uint8_t *)(&Key[0]));
            for(i=0; i<config->flashPageSize; i++) buffer.byte[i] = buffer.byte[i]^rc4_output();
            
            temp = WriteFlashPage(address, &(buffer.byte[0]));
            if (temp == ERR_SUCC) 
            {
              for(i=0; i<(config->flashPageSize/4); i++) 
              {
                if( *(__IO uint32_t *)address != buffer.word[i]) 
                {
                  temp = ERR_FLASH_VER; break;
                }
                address += 4;
              }
            }
            if(temp == ERR_SUCC) 
            {
              Can_Tx.Data[0] = TM_EXECUTE_CMD_6;
            } 
            else 
            {
            err_prog:
              Can_Tx.Data[0] = TM_ERROR_EXECUTE_CMD;
              Can_Tx.Data[4] = (uint8_t)temp;
              Can_Tx.Data[5] = (uint8_t)0;
              Can_Tx.Data[6] = (uint8_t)0;
              Can_Tx.Data[7] = (uint8_t)0;
            }
            canTransmit(CAN1, &Can_Tx);
            break;
            
          case CMD_CHECKCRC:
            if(!checkCrc32(&dummy32)) 
            {
              Can_Tx.Data[0] = TM_ERROR_EXECUTE_CMD;
              Can_Tx.Data[4] = (uint8_t)255;
              Can_Tx.Data[5] = (uint8_t)255;
              Can_Tx.Data[6] = (uint8_t)255;
              Can_Tx.Data[7] = (uint8_t)255;
            } 
            else 
            {
              Can_Tx.Data[0] = TM_EXECUTE_CMD_6;
              Can_Tx.Data[4] = (uint8_t)0;
              Can_Tx.Data[5] = (uint8_t)0;
              Can_Tx.Data[6] = (uint8_t)0;
              Can_Tx.Data[7] = (uint8_t)0;
            }
            canTransmit(CAN1, &Can_Tx);
            break;
          }
          break; // End of TM_CMD_ACK_6.
          
        case TM_BOOT_WR_DWORD:
          writeAddress = ((uint32_t)ptrBox->Data[2]+((uint32_t)ptrBox->Data[3]<<8))&(FLASH_PAGE_SIZE - 1);
          buffer.word[writeAddress>>2] = DL32;
          Can_Tx.Data[0] = TM_BOOT_ANSWER_DWORD;
          Can_Tx.Data[4] = buffer.byte[writeAddress];
          Can_Tx.Data[5] = buffer.byte[writeAddress+1];
          Can_Tx.Data[6] = buffer.byte[writeAddress+2];
          Can_Tx.Data[7] = buffer.byte[writeAddress+3];
          canTransmit(CAN1, &Can_Tx);
          break;
        }
      }
      RxTail++;
      RxTail &= (CAN_SIZE-1);
    }
  }
}

#pragma inline=forced  
uint8_t WriteFlashPage(uint32_t flashStartAdr, uint8_t *dataPage)
{ uint32_t temp = ERR_FLASH_PRG;
uint16_t dummy, data;
uint32_t address;
uint8_t  *flashData;
if( (flashStartAdr >= config->dataAdress) && (flashStartAdr <= config->maxFlashSizeToPage) && !(flashStartAdr & (config->flashPageSize-1)) ) 
{
  FLASH_Unlock();
  IWDG_ReloadCounter();
  /* Clear All pending flags */
//  FLASH_ClearFlag( FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
  /* Запись в сектор данных.
  Блок данных leasingID[4] не должны перезатираться
  bootloader-ом. Поэтому сначала считываю их из flash-памяти, чтобы потом
  перезаписать. */
  if (flashStartAdr <= 0x080013FF ) 
  {
    flashData = (uint8_t *)(0x08001008);
    for (address = 0; address < 16; address++ ) 
      dataPage[address+8] = *flashData++;
  }
  
  if(/*FLASH_ErasePage(flashStartAdr) == */FLASH_COMPLETE) 
  {
    address = flashStartAdr;
    for(dummy = 0; dummy < (config->flashPageSize/2); dummy++) 
    {
      data = *dataPage++;
      data |= (((uint16_t)(*dataPage++))<<8);
      if (FLASH_ProgramHalfWord(address, data) != FLASH_COMPLETE) 
      {
        temp = ERR_FLASH_PRG; break;
      } 
      else 
      {
        temp = ERR_SUCC;
        address += 2;
      }
      IWDG_ReloadCounter();
    }
  }
  FLASH_Lock();
} 
return temp;
}

// key_length - key length in bytes;
void rc4_init(uint8_t *key)
{
  register  uint8_t temp, i, j;
  i = 0;
  do{
    S[i] = i;
  }while(++i);
  
  j = 0;
  i = 0;
  do{
    j = (j + key[i&0x0F] + S[i]);
    temp = S[i];
    S[i] = S[j];
    S[j] = temp;
  }while(++i);
  key_i = key_j = 0;
}

uint8_t rc4_output(void)
{
  uint8_t temp;
  key_i++;
  key_j = (key_j + S[key_i]);
  temp = S[key_j];
  S[key_j] = S[key_i];
  S[key_i] = temp;
  return S[(temp + S[key_j])&0xFF];
}

