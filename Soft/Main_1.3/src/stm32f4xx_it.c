#include "stm32f4xx_it.h"
#include "main.h"
#include "SUN_CAN.h"
#include "stdio.h"


CanRxMsg SUN_RxMessage;
CanRxMsg SRV_RxMessage;

extern volatile uint32_t sunCanLedTimer;
extern volatile uint32_t srvCanLedTimer;

extern clientStateVars srvHtbtState;

void CAN1_RX0_IRQHandler(void)
{
  uint32_t SUN_IRQ_dummy;
  
  SUN_IRQ_dummy = (sunReceiveHead+1)&(CAN_SIZE-1);
  if(SUN_IRQ_dummy != sunReceiveTail)
  {
    CAN_Receive(CAN1, CAN_FIFO0, &sunCanBox[sunReceiveHead]);
    sunReceiveHead++;
    sunReceiveHead &= (CAN_SIZE-1);
    
    GPIO_WriteBit(GPIO_SUN_CAN_LED_Port, GPIO_SUN_CAN_LED_Pin, (BitAction)1); // ВКЛ светодиод сигнализации об отправке/получении SUN_CAN сообщения при ПРИЕМЕ SUN_CAN сообщения 
    sunCanLedTimer = setTimeout();
  }    
} 


void CAN1_RX1_IRQHandler(void)
{
  uint32_t SUN_IRQ_dummy;
  
  SUN_IRQ_dummy = (sunReceiveHead+1)&(CAN_SIZE-1);
  if(SUN_IRQ_dummy != sunReceiveTail)
  {
    CAN_Receive(CAN1, CAN_FIFO1, &sunCanBox[sunReceiveHead]);
    sunReceiveHead++;
    sunReceiveHead &= (CAN_SIZE-1);

    GPIO_WriteBit(GPIO_SUN_CAN_LED_Port, GPIO_SUN_CAN_LED_Pin, (BitAction)1); // ВКЛ светодиод сигнализации об отправке/получении SUN_CAN сообщения при ПРИЕМЕ SUN_CAN сообщения 
    sunCanLedTimer = setTimeout();
  }
}


void CAN1_TX_IRQHandler(void)
{
  GPIO_WriteBit(GPIO_SUN_CAN_LED_Port, GPIO_SUN_CAN_LED_Pin, (BitAction)1); // ВКЛ светодиод сигнализации об отправке/получении SUN_CAN сообщения при ПРИЕМЕ SUN_CAN сообщения 
  
  sunCanLedTimer = setTimeout();
  
  if (CAN_GetITStatus(CAN1,CAN_IT_TME))
  {          
    CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
  }
}

void CAN2_RX0_IRQHandler(void)
{
  uint32_t SRV_IRQ_dummy;
  
  SRV_IRQ_dummy = (srvReceiveHead+1)&(CAN_SIZE-1);
  if(SRV_IRQ_dummy != srvReceiveTail)
  {
    CAN_Receive(CAN2, CAN_FIFO0, &srvCanBox[srvReceiveHead]);
    srvReceiveHead++;
    srvReceiveHead &= (CAN_SIZE-1); 
    
    GPIO_WriteBit(GPIO_SRV_CAN_LED_Port, GPIO_SRV_CAN_LED_Pin, (BitAction)1); // ВКЛ светодиод сигнализации об отправке/получении SRV_CANOpen сообщения при ПРИЕМЕ SRV_CANOpen сообщения
    srvCanLedTimer = setTimeout();
  }    
}  

void CAN2_TX_IRQHandler(void)
{
  GPIO_WriteBit(GPIO_SRV_CAN_LED_Port, GPIO_SRV_CAN_LED_Pin, (BitAction)1); // ВКЛ светодиод сигнализации об отправке/получении SUN_CAN сообщения при ПРИЕМЕ SUN_CAN сообщения 
  
  srvCanLedTimer = setTimeout();
  
  if (CAN_GetITStatus(CAN2,CAN_IT_TME))
  {          
    CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
  }
}

// Прерывание TIM6
void TIM6_DAC_IRQHandler ( )
{
  /* Так как этот обработчик вызывается и для ЦАП, нужно проверять,
  * произошло ли прерывание по переполнению счётчика таймера TIM6.   */
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
  {   
    if(srvHtbtState == CLIENT_ALIVE) 
    {
//      srvParamsRequestFlags.fAlive = 1; // устанавливаем данный флаг, тем самым разрешая дальнейшую работу с SRV (SRV на связи с платой)
      //      GPIO_WriteBit(GPIOE, GPIO_Pin_2, (BitAction)1);
    }
    else
    {
//      srvParamsRequestFlags.fAlive = 0; // сбрасываем данный флаг, тем самым запрещая дальнейшую работу с SRV (у платы нет связи с SRV, т.е. плата за установленный период не получила Htbt-сообщение 703 7F от SRV, принимаем это как аварию)
      //      GPIO_WriteBit(GPIOE, GPIO_Pin_2, (BitAction)0);       
    }
    
    srvHtbtState = CLIENT_DEAD;           
    
    // Очищаем бит обрабатываемого прерывания
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);    
  } // __if
} // ___TIM6_DAC_IRQHandler

/**
* @brief  This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_Handler(void)
{
  systemTickValue++; 
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval None
*/
/*void PPP_IRQHandler(void)
{
}*/

/**
* @}
*/ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
