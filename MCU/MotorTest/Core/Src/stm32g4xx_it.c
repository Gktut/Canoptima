/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ABS(N) ((N<0)?(-N):(N))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint16_t step_1=0;
uint16_t step_2=0;


uint8_t TXbuf[5] = {0};

int16_t change_in_NS = 0;
int16_t change_in_EW = 0;


uint32_t rasp_uart_counter = 0;
uint8_t rasp_uart_freq = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void go_near_zero(void);

void reset_position(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
typedef enum{
	none = 0,
	north,
	south,
	east,
	west,
	northwest,
	southwest,
	southeast,
	northeast
}movement_direction;

extern movement_direction canopy_dir;
extern uint16_t canopy_step;

extern uint8_t direction_1;
extern uint8_t direction_2;


extern int16_t abs_pos_fb;
extern int16_t abs_pos_lr;

extern uint8_t RXbuf[100];
uint8_t RXbuf_past[100] = {0};

extern uint8_t north_initialized;
extern uint8_t south_initialized;
extern uint8_t west_initialized;
extern uint8_t east_initialized;

extern uint8_t north_allowed;
extern uint8_t south_allowed;
extern uint8_t west_allowed;
extern uint8_t east_allowed;

extern uint16_t NS_len;
extern uint16_t EW_len;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
//on limit switch
	if(north_initialized == 0 && (canopy_dir == north)){
		north_initialized = 1;
		step_1 = 0;
		step_2 = 0;
	}
	else if(north_allowed == 1) north_allowed = 0;
	
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
//arka limit switch
	if(south_initialized == 0 && (canopy_dir == south)){
		south_initialized = 1;
		NS_len = 5000-step_1;
		step_1 = 0;
		step_2 = 0;
	}
	if(south_allowed == 1) south_allowed = 0;
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
//sol limit siwtch
	if(west_initialized == 0&& (canopy_dir == west)){
		west_initialized = 1;
		step_1 = 0;
		step_2 = 0;
	}
	if(west_allowed == 1) west_allowed = 0;
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
//sag limit switch
	if(east_initialized == 0&& (canopy_dir == east)) {
		east_initialized = 1;
		EW_len = 5000-step_1;
		step_1 = 0;
		step_2 = 0;
	}
	if(east_allowed == 1) east_allowed = 0;
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
		rasp_uart_counter++;
			if(canopy_dir==north){
			direction_1 = 1;
			direction_2 = 0;
			if(canopy_step != 0){
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_fb += canopy_step;
			}
		}
			canopy_step = 0;
		}
		else if(canopy_dir==south){
			direction_1 = 0;
			direction_2 = 1;
			if(canopy_step != 0){
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)){
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_fb -= canopy_step;
				}
			}
			canopy_step = 0;
		}
		else if(canopy_dir==west){
			direction_1 = 0;
			direction_2 = 0;
			if(canopy_step != 0){
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)){
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_lr += canopy_step;
				}
			}
			canopy_step = 0;
		}
		else if(canopy_dir==east){
			direction_1 = 1;
			direction_2 = 1;
			if(canopy_step != 0){
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)){
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_lr -= canopy_step;
				}
			}
			canopy_step = 0;
		}
		else if(canopy_dir==northeast){
			direction_1 = 1;
			direction_2 = 1;
			if(canopy_step != 0){
				if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==1) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1)){
					//iki limitte basili degil hareketi yap
					step_1 = canopy_step;
					step_2 = 0;
					abs_pos_fb += canopy_step/2;
					abs_pos_lr -= canopy_step/2;
				}
				else if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==0) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1)){
				//	canopy_dir = north;
					direction_1 = 1;
					direction_2 = 0;
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_fb += canopy_step;
				}
				else if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==1) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0)){
				//	canopy_dir = east;
					direction_1 = 1;
					direction_2 = 1;
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_lr -= canopy_step;
				}
			}
			canopy_step = 0;
		}
		else if(canopy_dir==southeast){
			direction_1 = 1;
			direction_2 = 1;
			if(canopy_step != 0){
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)){
					step_1 = 0;
					step_2 = canopy_step;
					abs_pos_fb -= canopy_step/2;
					abs_pos_lr -= canopy_step/2;
				}
				else if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==0) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==1)){
				//	canopy_dir = south;
					direction_1 = 0;
					direction_2 = 1;
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_fb -= canopy_step;
				}
				else if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==1) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==0)){
				//	canopy_dir = east;
					direction_1 = 1;
					direction_2 = 1;
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_lr -= canopy_step;
				}
			}
			canopy_step = 0;
		}
		else if(canopy_dir==southwest){
			direction_1 = 0;
			direction_2 = 1;
			if(canopy_step != 0){
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)){
					step_1 = canopy_step;
					step_2 = 0;
					abs_pos_fb -= canopy_step/2;
					abs_pos_lr += canopy_step/2;
				}
				else if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==0) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==1)){
				//	canopy_dir = south;
					direction_1 = 0;
					direction_2 = 1;
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_fb -= canopy_step;
				}
				else if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==1) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==0)){
				//	canopy_dir = west;
					direction_1 = 0;
					direction_2 = 0;
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_lr += canopy_step;
				}
			}
			canopy_step = 0;
		}
		else if(canopy_dir==northwest){
			direction_1 = 0;
			direction_2 = 0;
			if(canopy_step != 0){
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
					step_1 = 0;
					step_2 = canopy_step;
					abs_pos_fb += canopy_step/2;
					abs_pos_lr += canopy_step/2;
				}
				else if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==0) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1)){
				//	canopy_dir = north;
					direction_1 = 1;
					direction_2 = 0;
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_fb += canopy_step;
				}
				else if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==1) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0)){
				//	canopy_dir = west;
					direction_1 = 0;
					direction_2 = 0;
					step_1 = canopy_step;
					step_2 = canopy_step;
					abs_pos_lr += canopy_step;
				}
			}
			canopy_step = 0;
		}
		HAL_GPIO_WritePin(Direction_1_GPIO_Port, Direction_1_Pin, direction_1);
		HAL_GPIO_WritePin(Direction_2_GPIO_Port, Direction_2_Pin, direction_2);
	if(step_1>0){
		step_1--;
		TIM1->CCR1 = 4000;
	}
	else
		TIM1->CCR1=0;
	
		if(step_2>0){
		step_2--;
		TIM1->CCR2 = 4000;
	}
	else
		TIM1->CCR2=0;
	
  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	if(USART1 == huart1.Instance){
    if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)){
      __HAL_UART_CLEAR_IDLEFLAG(&huart1);
       
      HAL_UART_DMAStop(&huart1);
			uint8_t data_length_uart1  = sizeof(RXbuf) - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
			
			rasp_uart_freq = 1250/rasp_uart_counter;
			rasp_uart_counter = 0;
			
			
			//0xdd (01 north 02 south) (01 east 02 west) 0xdd
			if(RXbuf[0] == 0XDD && RXbuf[3] == 0xDD && (step_1 == 0) && (step_2 == 0)){
				if(RXbuf[1] == 0x01 && RXbuf[2] == 0x00){
				//north
					canopy_dir = north;
					canopy_step = 49;
				}
			else if(RXbuf[1] == 0x01 && RXbuf[2] == 0x01){
				//northeast
					canopy_dir = northeast;
					canopy_step = 49;
				}
			else if(RXbuf[1] == 0x01 && RXbuf[2] == 0x02){
				//northwest
					canopy_dir = northwest;
					canopy_step = 49;
				}
			else if(RXbuf[1] == 0x02 && RXbuf[2] == 0x00){
				//south
					canopy_dir = south;
					canopy_step = 49;
				}
			else if(RXbuf[1] == 0x02 && RXbuf[2] == 0x01){
				//southeast
					canopy_dir = southeast;
					canopy_step = 49;
				}
			else if(RXbuf[1] == 0x02 && RXbuf[2] == 0x02){
				//southwest
					canopy_dir = southwest;
					canopy_step = 49;
				}
			else if(RXbuf[1] == 0x00 && RXbuf[2] == 0x01){
				//east
					canopy_dir = east;
					canopy_step = 49;
				}
			else if(RXbuf[1] == 0x00 && RXbuf[2] == 0x02){
				//west
					canopy_dir = west;
					canopy_step = 49;
				}
			else if(RXbuf[1] == 0x00 && RXbuf[2] == 0x00){
				//none
					canopy_dir = none;
					canopy_step = 0;
				}
			else if(RXbuf[1] == 0x03 && RXbuf[2] == 0x03){
//					//zero mode calculate and go to center. actually close to the center
//					//target is NS_len/2 EW_len/2
//					//current pos is abs_pos_fb abs_pos_lr
//					change_in_NS =  abs_pos_fb - NS_len/2;
//					change_in_EW = abs_pos_lr - EW_len/2;
//				
//					if((change_in_NS > 0) &&  (change_in_EW > 0))
//					{
//						//hem kuzeyindeyim hem batisindayim targetin. Dolayisi ile SE yönünde gitmeliyim
//						canopy_dir = southeast;
//						//step sayisini bulmak icin söyle bir durum var, ikisinden ufak olani al onu step sayisi yap
//						(change_in_NS > change_in_EW) ? (canopy_step = ABS(change_in_EW)) : (canopy_step = ABS(change_in_NS));
//					}
//					else if((change_in_NS > 0) &&  (change_in_EW < 0))
//					{
//						//hem kuzeyindeyim hem dogusundayim targetin. Dolayisi ile Sw yönünde gitmeliyim
//						canopy_dir = southwest;
//						//step sayisini bulmak icin söyle bir durum var, ikisinden ufak olani al onu step sayisi yap
//						(change_in_NS > change_in_EW) ? (canopy_step = ABS(change_in_EW)) : (canopy_step = ABS(change_in_NS));
//					}
//					else if((change_in_NS < 0) &&  (change_in_EW < 0))
//					{
//						//hem güneyimeyim hem dogusundayim targetin. Dolayisi ile NW yönünde gitmeliyim
//						canopy_dir = northwest;
//						//step sayisini bulmak icin söyle bir durum var, ikisinden ufak olani al onu step sayisi yap
//						(change_in_NS > change_in_EW) ? (canopy_step = ABS(change_in_EW)) : (canopy_step = ABS(change_in_NS));
//					}
//					else if((change_in_NS < 0) &&  (change_in_EW > 0))
//					{
//						//hem güneyindeyim hem batisindayim targetin. Dolayisi ile NE yönünde gitmeliyim
//						canopy_dir = northeast;
//						//step sayisini bulmak icin söyle bir durum var, ikisinden ufak olani al onu step sayisi yap
//						(change_in_NS > change_in_EW) ? (canopy_step = ABS(change_in_EW)) : (canopy_step = ABS(change_in_NS));
//					}
//					else if(change_in_NS == 0)
//					{
//						//NS yönünde bi hareket yapmamaliyyim ama WE de yapmaliyim. eger büyükse 0dan o zaman batisindasin doguya git
//						(change_in_EW>0) ? (canopy_dir = east) :(canopy_dir = west) ;
//						canopy_step = ABS(change_in_EW);
//			
//					}
//					else if(change_in_EW == 0)
//					{
//						//EW yönünde bi hareket yapmamaliyyim ama NS de yapmaliyim. eger büyükse 0dan o zaman kuzeydesin güneye git
//						(change_in_NS>0) ? (canopy_dir = south) :(canopy_dir = north) ;
//						canopy_step = ABS(change_in_NS);
//			
//					}
//					else
//					{
//						canopy_step = 0;
//						canopy_dir = 0;
//					}
//					
					//go_near_zero();
////////					direction_1 = 1;
////////					direction_2 = 1;
////////					step_1 = 0;
////////					step_2 = canopy_step;
////////					uint8_t dummy=0;
////////					for(uint16_t k=0; k<2500; k++){
////////						dummy++;
////////					}						
					
					NVIC_SystemReset();
				}
			}
////////			if(RXbuf[0] == 0xDD && RXbuf[2] == 0xDD){
////////				canopy_dir = RXbuf[1];
////////				canopy_step = 49;
////////			}
				
			 //memset(RXbuf,0,data_length_uart1);
			//send data back to raspberry, data format 0XAA [north limit=1 0x01 else 0x00] [same for south][same for east] [same for west]
				TXbuf[0] = 0xAA;
				TXbuf[1] = !(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));//north
				TXbuf[2] = !(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));//south
				TXbuf[3] = !(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7));//east
				TXbuf[4] = !(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4));//west
				//HAL_UART_Transmit_IT(&huart1, (uint8_t*)TXbuf, 5);
			 HAL_UART_Receive_DMA(&huart1, RXbuf, sizeof(RXbuf));
			
		}
	}

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
//	 if(USART2 == huart2.Instance){
//    if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)){
//      __HAL_UART_CLEAR_IDLEFLAG(&huart2);
//       
//      HAL_UART_DMAStop(&huart2);
//			 HAL_UART_Receive_DMA(&huart2, RXbuf, sizeof(RXbuf));
//		}
//	}

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void go_near_zero(void){
					//zero mode calculate and go to center. actually close to the center
					//target is NS_len/2 EW_len/2
					//current pos is abs_pos_fb abs_pos_lr
					change_in_NS =  abs_pos_fb - NS_len/2;
					change_in_EW = abs_pos_lr - EW_len/2;
				
					if((change_in_NS > 0) &&  (change_in_EW > 0))
					{
						//hem kuzeyindeyim hem batisindayim targetin. Dolayisi ile SE yönünde gitmeliyim
						canopy_dir = southeast;
						//step sayisini bulmak icin söyle bir durum var, ikisinden ufak olani al onu step sayisi yap
						(change_in_NS > change_in_EW) ? (canopy_step = 2*ABS(change_in_EW)) : (canopy_step = 2*ABS(change_in_NS));
					}
					else if((change_in_NS > 0) &&  (change_in_EW < 0))
					{
						//hem kuzeyindeyim hem dogusundayim targetin. Dolayisi ile Sw yönünde gitmeliyim
						canopy_dir = southwest;
						//step sayisini bulmak icin söyle bir durum var, ikisinden ufak olani al onu step sayisi yap
						(change_in_NS > change_in_EW) ? (canopy_step = 2*ABS(change_in_EW)) : (canopy_step = 2*ABS(change_in_NS));
					}
					else if((change_in_NS < 0) &&  (change_in_EW < 0))
					{
						//hem güneyimeyim hem dogusundayim targetin. Dolayisi ile NW yönünde gitmeliyim
						canopy_dir = northwest;
						//step sayisini bulmak icin söyle bir durum var, ikisinden ufak olani al onu step sayisi yap
						(change_in_NS > change_in_EW) ? (canopy_step = 2*ABS(change_in_EW)) : (canopy_step = 2*ABS(change_in_NS));
					}
					else if((change_in_NS < 0) &&  (change_in_EW > 0))
					{
						//hem güneyindeyim hem batisindayim targetin. Dolayisi ile NE yönünde gitmeliyim
						canopy_dir = northeast;
						//step sayisini bulmak icin söyle bir durum var, ikisinden ufak olani al onu step sayisi yap
						(change_in_NS > change_in_EW) ? (canopy_step = 2*ABS(change_in_EW)) : (canopy_step = 2*ABS(change_in_NS));
					}
					else if(change_in_NS == 0)
					{
						//NS yönünde bi hareket yapmamaliyyim ama WE de yapmaliyim. eger büyükse 0dan o zaman batisindasin doguya git
						(change_in_EW>0) ? (canopy_dir = east) :(canopy_dir = west) ;
						canopy_step = ABS(change_in_EW);
			
					}
					else if(change_in_EW == 0)
					{
						//EW yönünde bi hareket yapmamaliyyim ama NS de yapmaliyim. eger büyükse 0dan o zaman kuzeydesin güneye git
						(change_in_NS>0) ? (canopy_dir = south) :(canopy_dir = north) ;
						canopy_step = ABS(change_in_NS);
			
					}
					else
					{
						canopy_step = 0;
						canopy_dir = 0;
					}
					
}

void reset_position(void){
	canopy_dir = north;
	canopy_step = 5000;
	while(north_initialized == 0){
		HAL_Delay(1);
	}
	HAL_Delay(300);
	canopy_dir = south;
	canopy_step = 5000;
	while(south_initialized == 0){
		HAL_Delay(1);
	}
	HAL_Delay(300);
	TIM1->PSC = 16;
	canopy_dir = north;
	canopy_step = NS_len/2;
	HAL_Delay(2*NS_len);
	TIM1->PSC = 16;
	canopy_dir = west;
	canopy_step = 5000;
	while(west_initialized == 0){
		HAL_Delay(1);
	}
	HAL_Delay(300);
	canopy_dir = east;
	canopy_step = 5000;
	while(east_initialized == 0){
		HAL_Delay(1);
	}
	HAL_Delay(300);
	TIM1->PSC = 16;
	canopy_dir = west;
	canopy_step = EW_len/2;
	HAL_Delay(2*EW_len);


}

/* USER CODE END 1 */
