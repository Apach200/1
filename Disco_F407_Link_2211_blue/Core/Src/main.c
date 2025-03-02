/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  *			Aliexpress_Disco407green__EvolutionBoard
  *
  *			NodeID = 0x3A
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "Encoder.h"
#include "format_out.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

RTC_DateTypeDef DateToUpdate;//  = {02, 01, 21, 25};  //; ={0,0,0};   //21jan2025
RTC_TimeTypeDef sTime;// = {19, 24, 0,0,0,0,0};; // ={0,0,0};      ///19h16min
//					    0,//uint8_t Hours; Max_Data=12 if the RTC_HourFormat_12; Max_Data=23 if the RTC_HourFormat_24
//						0,//uint8_t Minutes; Max_Data = 59
//						0,//uint8_t Seconds; Max_Data = 59 */
//						0,//uint8_t TimeFormat;Specifies the RTC AM/PM Time.
//						0,//uint32_t SecondFraction;parameter corresponds to a time unit range between [0-1] Second with [1 Sec / SecondFraction +1] granularity
//						0,//uint32_t DayLightSaving;  This interface is deprecated.
//						0//uint32_t StoreOperation;

uint8_t Tx_Array[16]={0x51,0x62,0x73,0x84,0x55,0x46,0x87,0x18,0x29,0x10,0x11,0x12,0x13,0x14,0x15,0x33};
uint8_t Rx_Array[16]={0};
uint8_t Rx_Array_0[16]={0};
uint8_t Rx_Array_1[16]={0};
uint32_t Array_32u[16]={0};
uint8_t Array_8u[16]={0x54,0x34,0x21,0xea,0xf3,0x7a,0xd4,0x46};
char Message_to_Terminal[128]={};
char Message_to_Terminal_1[128]={};
char Message_to_Terminal_2[128]={};
char Message_to_Terminal_3[128]={};
uint8_t Array_from_Terminal[128]={0};
uint8_t Length_of_Message;
uint8_t Length_of_Ext_Var=0;
uint8_t Local_Count=0;
uint32_t Duration_of_the_CO_process;
uint64_t Count_of_while1=0;
float ChipTemperature;


/////////////////////////////////////////Istarik_begin
char trans_str[128] = {0};
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[16]={8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};
uint8_t RxData[16]={8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};
uint32_t TxMailbox = 0;

///////////////////////////////////////Istarik_end

uint16_t count0=0;



CAN_TxHeaderTypeDef Tx_Header;
CAN_RxHeaderTypeDef Rx_Header;
uint32_t            TxMailbox;
uint32_t            tmp32u_1   = 0x1e1f1a1b;
uint32_t            tmp32u_0   = 0x0e0f0a0b;
uint64_t            tmp64u_0   = 0x0e1f1a1b56789a;
uint64_t            tmp64u_1   = 0x0e1f1a1b56789a;
uint32_t            Ticks;
uint32_t            Ticks_1;
uint32_t            Ticks_2;

char String_H[16]={8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};
char String_L[]={"String_for_Test_UART_"};
char buff[16]={8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

char String_LCD[32];
char String_2_UART[128];
uint16_t L_str;
int16_t currCounter=0 ;
int32_t prevCounter =0;


uint8_t Menu_step=0;
const uint16_t Datum[64]={0,0,
						  0x30,0x31, 0x30,0x32, 0x30,0x33, 0x30,0x34, 0x30,0x35, 0x30,0x36, 0x30,0x37, 0x30,0x38, 0x30,0x39, 0x31,0x30,
					      0x31,0x31, 0x31,0x32, 0x31,0x33, 0x31,0x34, 0x31,0x35, 0x31,0x36, 0x31,0x37, 0x31,0x38, 0x31,0x39, 0x32,0x30,
					      0x32,0x31, 0x32,0x32, 0x32,0x33, 0x32,0x34, 0x32,0x35, 0x32,0x36, 0x32,0x37, 0x32,0x38, 0x32,0x39, 0x33,0x30,
					      0x33,0x31
					     };



uint8_t Node_ID_Read=0xff;
uint32_t Data_u32;

extern TIM_HandleTypeDef htim4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_interface_Test(void);
void UART_interface_Test(void);
void Board_Name_to_Terminal(void);
int16_t Encoder_to_LCD(void);
Encoder_Status encoderStatus;
static void CAN_Config(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/////////////////////////////////////////Istarik_begin
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
    	//count0++;
        if(RxHeader.ExtId == 0x00014323)
        {}
        	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
       // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET);
       // while(huart2.gState!=HAL_UART_STATE_READY){;}
        Length_of_Message=sprintf(
								 trans_str,
								// "ID %04X%04X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X  \n",
								 "F0__ID%04X%04X 0x%02X\r\n",///  IDE=%d RTR=%d
								 (uint16_t)(RxHeader.ExtId>>16),(uint16_t)(RxHeader.ExtId & 0x0FFFF),
								 //(uint16_t)(RxHeader.IDE & 0x0FFFF),
								 //(uint16_t)(RxHeader.RTR & 0x0FFFF)
								 RxData[0]//,RxData[1],RxData[2],RxData[3],RxData[4],RxData[5],RxData[6],RxData[7]
								);

          //   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET);

//        else if(RxHeader.ExtId == 0x00024323)
//        {
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET);
//            Length_of_Message=sprintf(
//    								 trans_str,
//    								// "ID %04X%04X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X  \n",
//    								 "ID %04X%04X \n",
//    								 (uint16_t)(RxHeader.ExtId>>16),(uint16_t)(RxHeader.ExtId & 0x0FFFF)//,
//    								 //RxData[0],RxData[1],RxData[2],RxData[3],RxData[4],RxData[5],RxData[6],RxData[7]
//    								);
//        }
    }else{HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_SET);}

HAL_UART_Transmit(&huart2, (uint8_t*)trans_str, Length_of_Message,10);
}



void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
    {
    	//count0++;
//        if(RxHeader.ExtId == 0x00014323)
//        { }
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_SET);
        Length_of_Message=sprintf(
								 trans_str,
								// "ID %04X%04X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X  \n",
								 "__F1__ID%04X%04X 0x%02X\r\n",
								 (uint16_t)(RxHeader.ExtId>>16),(uint16_t)(RxHeader.ExtId & 0x0FFFF),
								 RxData[0]//,RxData[1],RxData[2],RxData[3],RxData[4],RxData[5],RxData[6],RxData[7]
								);

         HAL_UART_Transmit(&huart2, (uint8_t*)trans_str, Length_of_Message, 10);

    }





}







void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_SET);
    sprintf(trans_str,"ER CAN %lu %08lX", er, er);
    HAL_UART_Transmit(&huart2, (uint8_t*)trans_str, strlen(trans_str), 100);
}
/////////////////////////////////////////Istarik_end

/* Timer interrupt function executes every 1 ms */
void
HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

    if (htim == &htim4)
    		{
    		//canopen_app_interrupt();
    		}

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_CAN1_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
 // MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  //	HAL_RTC_SetTime(&hrtc, &sTime,        RTC_FORMAT_BIN);
  //	HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
//  HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
//  HAL_RTC_GetTime(&hrtc, &sTime,        RTC_FORMAT_BIN);
  //Datum_Time_from_PC(DateToUpdate, sTime);
  Encoder_Config();  // configure the encoders timer
  Encoder_Init();    // start the encoders timer
  LCD_ini();
  // Logo_to_1602LCD();
  Datum_to_1602LCD();
  //GPIO_Blink_Test(GPIOA, GPIO_PIN_7|GPIO_PIN_6, 25, 33); 						// for_STM32F4XX_Ali_pcb
   GPIO_Blink_Test(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, 10, 20);// blink_at_Discovery_EVB

  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UART_Receive_DMA(&huart2, Array_from_Terminal, sizeof Array_from_Terminal );
  Board_Name_to_Terminal();
 CAN_interface_Test();
//CAN_Config();
HAL_Delay(1);

/////////////////////////////////////////Istarik_begin

  TxHeader.StdId = 0;
  TxHeader.ExtId = 0x0378;
  TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
  TxHeader.IDE = CAN_ID_EXT;   // CAN_ID_EXT /// CAN_ID_STD
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = 0;


  for(uint8_t i = 0; i < 8; i++)
  {
      TxData[i] = (i + 10);
  }

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(
		  	  	  	  	  	  &hcan1,
							  	   CAN_IT_RX_FIFO0_MSG_PENDING
								  |CAN_IT_RX_FIFO1_MSG_PENDING
								  | CAN_IT_ERROR
								  | CAN_IT_BUSOFF
								  | CAN_IT_LAST_ERROR_CODE
							  );

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_RESET);//Green
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET);//Orange
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_RESET);//RED
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_RESET);//Blue

  while (1)
  {
//	  TxHeader.ExtId = 0x0378;
//	  TxData[0] = 90;
//
//      while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
//
//      if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//      	  	  {  HAL_UART_Transmit(&huart2, (uint8_t*)"ER SEND\n", 8, 100); }
//
//      HAL_Delay(500);
//
//      TxHeader.ExtId = 0x0126;
//	  TxData[0] = 100;
//
//      while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
//
//      if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//      {
//    	  HAL_UART_Transmit(&huart2, (uint8_t*)"ER SEND\n", 8, 100);
//      }
//
//      HAL_Delay(500);
		Encoder_to_LCD();
		RTC_update_and_Terminal(1999);

//		if(RxData[0]>252)
//			{
//			uint16_t LLL= sprintf(&String_H[7],"=0x%04X",count0>>8);
//			HAL_UART_Transmit_IT(&huart2, (uint8_t*)String_H, LLL+LLL);
//			}

if(HAL_GetTick()==2048+1024)
{
	CAN_FilterTypeDef  sFilterConfig;
	  sFilterConfig.FilterBank = 4;
	  sFilterConfig.FilterActivation = DISABLE;
	  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) { Error_Handler(); }
}

if(HAL_GetTick()==2048+2048)
{
	CAN_FilterTypeDef  sFilterConfig;
	  sFilterConfig.FilterMode  = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterBank = 4;
	  sFilterConfig.FilterIdHigh =(uint16_t) (0x00000107>>13);
	  sFilterConfig.FilterIdLow  =((uint16_t) (0x00000107<<3))|0x0004;
	  sFilterConfig.FilterMaskIdHigh = (uint16_t) (0x00000137>>13);
	  sFilterConfig.FilterMaskIdLow  = ((uint16_t) (0x00000137<<3))|0x0004;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterActivation = ENABLE;

	  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) { Error_Handler(); }
}


  }

  /////////////////////////////////////////Istarik_end


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//Green
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);//Orange

 Local_Count=0;
Ticks = HAL_GetTick();
Ticks_2 = HAL_GetTick();

DWT->CTRL |= 1 ; // Enable_the_Counter_of_Core_circles

//**********************************************************************************************
//Ticks_2=HAL_GetTick();
//do
//{
//
//}while (HAL_GetTick() - Ticks_2<100);

#if 1
uint32_t Delta_T[4]={0};/////Measurement_of_duration_canopen_app_process();
#endif

//**********************************************************************************************
Local_Count=0;
Ticks_2=HAL_GetTick();
 while (1)		/// 	while (HAL_GetTick() - Ticks_2<4123) //   	while (HAL_GetTick() - Ticks_2<5123)	///
	{
			switch (Local_Count)
				{
				case 0:
//																			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET );
					Local_Count=1;
					break;

				case 1:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET );
					DWT->CYCCNT = 0; // reset the counter

					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET );
					Local_Count=2;		HAL_Delay(4);
					Delta_T[1]= DWT->CYCCNT;
					break;

				case 2:
//					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET );
					DWT->CYCCNT = 0; // reset the counter
					Delta_T[2]= DWT->CYCCNT;
//					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET );
					Local_Count=3;
					break;

				case 3:
//					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET );
					DWT->CYCCNT = 0; // reset the counter
					Delta_T[3]= DWT->CYCCNT;
//					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET );
					Local_Count=0;
					break;

				default:
					Local_Count=0;
					break;
		}///switch (Local_Count)


		Encoder_to_LCD();
		RTC_update_and_Terminal(1999);

	}///while (HAL_GetTick() - Ticks<4123)




#if 1
///Measurement_of_duration_canopen_app_process_Results
uint16_t LLL = sprintf(
					Message_to_Terminal,
					"\n\r\n\r Duration_of_the_CO_process*6nc = %d\n\r",
					(uint16_t)Duration_of_the_CO_process*6
					);

 LLL = LLL + sprintf(
					Message_to_Terminal+LLL,
					"\n\r Delta_T[0] = 0x%04X%04X -> %d ns\n\r",
					(uint16_t)(Delta_T[0] >> 16 ),
					(uint16_t)(Delta_T[0] & 0x0FFFF ),
					(uint16_t)(((float)Delta_T[0]*6.3333) )
					);

 LLL = LLL + sprintf(
					Message_to_Terminal+LLL,
					"\n\r Delta_T[0] = 0x%04X%04X -> %d ns\n\r",
					(uint16_t)(Delta_T[1] >> 16 ),
					(uint16_t)(Delta_T[1] & 0x0FFFF ),
					(uint16_t)(((float)Delta_T[1]*6.3333) )
					);

 LLL = LLL + sprintf(
					Message_to_Terminal+LLL,
					"\n\r Delta_T[0] = 0x%04X%04X -> %d ns\n\r",
					(uint16_t)(Delta_T[2] >> 16 ),
					(uint16_t)(Delta_T[2] & 0x0FFFF ),
					(uint16_t)(((float)Delta_T[2]*6.3333) )
					);

 LLL = LLL + sprintf(
					Message_to_Terminal+LLL,
					"\n\r Delta_T[0] = 0x%04X%04X -> %d ns\n\r\n\r\n\r\n\r\n\r\n\r\n\r",
					(uint16_t)(Delta_T[3] >> 16 ),
					(uint16_t)(Delta_T[3] & 0x0FFFF ),
					(uint16_t)(((float)Delta_T[3]*6.3333) )
					);

while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
HAL_UART_Transmit_IT( &TerminalInterface, (uint8_t*)(Message_to_Terminal), LLL);
//while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
#endif///Measurement_of_duration_canopen_app_process();

//**********************************************************************************************

//HAL_CAN_GetRxMessage(hcan, RxFifo, pHeader, aData);
while (2)
{
Encoder_to_LCD();
RTC_update_and_Terminal(1999);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CAN_interface_Test(void)
{

	 Tx_Header.IDE    = CAN_ID_EXT;
	 Tx_Header.DLC    = 8;
	 Tx_Header.StdId  = 0;
	 Tx_Header.ExtId  = 0x2211;
	 Tx_Header.RTR    = CAN_RTR_DATA;

 HAL_CAN_Start(&hcan1);
 HAL_Delay(1500);
 for(uint16_t cnt=0;cnt<16;cnt++){Tx_Array[cnt]=Tx_Array[cnt]+1;}

 Message_2_UART_u32("Tx_Header.ExtId",Tx_Header.ExtId);

 HAL_CAN_AddTxMessage( &hcan1,&Tx_Header,Tx_Array+8, &TxMailbox );
 HAL_CAN_AddTxMessage( &hcan1,&Tx_Header,Array_8u, &TxMailbox );

 if(HAL_CAN_AddTxMessage( &hcan1,
		 					&Tx_Header,
							Tx_Array, &TxMailbox )==HAL_OK
 	 )
	  {  /* Wait transmission complete */
	  //while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
		  for(uint8_t cnt=0;cnt<16;cnt++)
		  	  {
			  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);//orange
			  HAL_Delay(32);
		  	  }
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,GPIO_PIN_RESET);//orange
	  }

}///CAN_interface_Test(void)

//////////////////////////////////////////////

void UART_interface_Test(void)
{
	// Test_Terminal__ASCII
	  Length_of_Message = sprintf( Message_to_Terminal,
			  	  	  	  	  	  	  "Rx_Array[0]=0x%x, Rx_Array[1]= 0x%x, Rx_Array[2]= 0x%x, Rx_Array[3]= 0x%x \n\r",
									   Rx_Array[0],Rx_Array[1],Rx_Array[2],Rx_Array[3]
								 );
	  TerminalInterface.gState = HAL_UART_STATE_READY;
	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);

//    Test_Terminal__HEX
//
//	  HAL_Delay(500);
//	  Local_Count = sizeof String_L;
//	  String_L[Local_Count-1] = 0x0d;
//	  TerminalInterface.gState = HAL_DMA_STATE_READY;
//	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)(String_L), Local_Count);

}



//////////////////////////////////////////////////
void Board_Name_to_Terminal(void)
{
	const char Message_0[]={"   ******************************************\n\r"};
//	const char Message_3[]={"*  Upper Blackboard  STM32F4XX__Ali3032 *\n\r"};
//	const char Message_3[]={"*  Lower Blackboard  STM32F4XX__Ali0867  *\n\r"};
//	const char Message_3[]={"*  STM32F4DISCOVERY Green_board China    *\n\r"};
//	const char Message_3[]={"*  STM32F4DISCO Greenboard_STLINK_4323   *\n\r"};
//	const char Message_3[]={"*  STM32F4DISCO Greenboard_STLINK_2734   *\n\r"};
	const char Message_3[]={"*  STM32F4DISCOVERY Blue_board Orig2211  *\n\r"};
//	const char Message_5[]={"*       *\n\r"};
	char Array_for_Messages[128]={};
	uint16_t Msg_Length;
//	uint32_t Chip_ID_96bit[4]={};
//	uint16_t  *pChip_ID_96bit =(uint16_t*)Chip_ID_96bit ;

//	Chip_ID_96bit[0] = HAL_GetUIDw0();
//	Chip_ID_96bit[1] = HAL_GetUIDw1();
//	Chip_ID_96bit[2] = HAL_GetUIDw2();

	Msg_Length = sizeof(Message_0);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_0, Msg_Length);
////	HAL_UART_Transmit( &TerminalInterface, (uint8_t*)Message_0, Msg_Length,1);

	Msg_Length = sizeof(Message_3);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_3, Msg_Length);
////	HAL_UART_Transmit( &TerminalInterface, (uint8_t*)Message_3, Msg_Length,1);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "*  SystemClock = %d MHz                 *\n\r",
						  (uint16_t)(HAL_RCC_GetSysClockFreq()/1000000)
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);
////	HAL_UART_Transmit( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length,1);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "   *  Unical_ID %X%X%X%X%X%X        *\n\r",
						  (uint16_t)(HAL_GetUIDw2()>>16),(uint16_t)(HAL_GetUIDw2() & 0x0000FFFF),
						  (uint16_t)(HAL_GetUIDw1()>>16),(uint16_t)(HAL_GetUIDw1() & 0x0000FFFF),
						  (uint16_t)(HAL_GetUIDw0()>>16),(uint16_t)(HAL_GetUIDw0() & 0x0000FFFF)

						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "   *  Device identifier %X%X                *\n\r",
						  (uint16_t)(HAL_GetDEVID()>>16), (uint16_t)(HAL_GetDEVID() & 0x0000FFFF)
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "   *  Device revision identifier %X%X      *\n\r",
						  (uint16_t)( HAL_GetREVID()>>16 ),
						  (uint16_t)( HAL_GetREVID() & 0x0000FFFF )
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

	Msg_Length = sizeof(Message_0);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_0, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Array_for_Messages[0]=0x0a;		Array_for_Messages[1]=0x0d;
	Array_for_Messages[2]=0x0a;		Array_for_Messages[3]=0x0d;
	Array_for_Messages[4]=0x0a;		Array_for_Messages[5]=0x0d;
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)(Array_for_Messages), 6);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
}

/////////////////////////////////////////////////////////////////////

  int16_t Encoder_to_LCD(void)
  {
		encoderStatus = Encoder_Get_Status();

		  switch(encoderStatus) {
		    case Incremented:
		    	currCounter++;
				//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET  );
				sprintf(String_LCD,"%04d",currCounter);
				LCD_SetPos(11, 0);
				HAL_Delay(10);
				LCD_String(String_LCD);HAL_Delay(10);

				L_str=	snprintf(buff, sizeof(buff), "\n\r %04d ", currCounter);
				while(TerminalInterface.gState != HAL_UART_STATE_READY ){;}
				HAL_UART_Transmit_IT(&TerminalInterface, (uint8_t*)buff, L_str);
				//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET  );
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET  );
		      break;
		    case Decremented:
		    	currCounter--;
				//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET  );
				sprintf(String_LCD,"%04d",currCounter);
				LCD_SetPos(11,0);HAL_Delay(10);

				LCD_String(String_LCD);HAL_Delay(10);

				L_str=	snprintf(buff, sizeof(buff), "\n\r %04d ", currCounter);
				while(TerminalInterface.gState != HAL_UART_STATE_READY ){;}
				HAL_UART_Transmit_IT(&TerminalInterface, (uint8_t*)buff, L_str);
				//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET  );
				HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET  );
		      break;

		    case Neutral:

		    	break;
		    default: break;
		  }///switch(encoderStatus)

		    	if(HAL_GetTick()-Ticks_1 >1500){
											//Ticks_1=HAL_GetTick();
											//Get_Time_to_LCD(0,1);
		    								}

		    	// RTC_update_and_Terminal(1999);
		    	return (currCounter);
  }


  ///////////////////////////////////////////////////////////////////////////////
  /**
    * @brief  Configures the CAN.
    * @param  None
    * @retval None
    */
  static void CAN_Config(void)
  {
    CAN_FilterTypeDef  sFilterConfig;
    const uint32_t Board_ID1 = 0x00014323;
    const uint32_t Board_ID2 = 0x00024323;
    const uint32_t B1its_17_0_IDE_RTR = (Board_ID1<<3)& (0x00FFFFFF ) & (0x00FFFFFF>>3 ) & (~0x00000003) ;
    const uint32_t B2its_17_0_IDE_RTR = (Board_ID2<<3)& (0x00FFFFFF ) & (0x00FFFFFF>>3 ) & (~0x00000003) ;

    /* Configure the CAN peripheral */
   extern CAN_HandleTypeDef hcan1;
    /* Configure the CAN Filter */
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterIdHigh = B1its_17_0_IDE_RTR>>16;
    sFilterConfig.FilterIdLow = B1its_17_0_IDE_RTR & 0x0FFFF;
    sFilterConfig.FilterMaskIdHigh = B1its_17_0_IDE_RTR>>16;
    sFilterConfig.FilterMaskIdLow = B1its_17_0_IDE_RTR & 0x0FFFF;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    //HAL_CAN_Stop(&hcan1);

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    	{Error_Handler();
}  /* Filter configuration Error */




    sFilterConfig.FilterBank = 1;
    sFilterConfig.FilterIdHigh = B2its_17_0_IDE_RTR>>16;
    sFilterConfig.FilterIdLow = B2its_17_0_IDE_RTR & 0x0FFFF;
    sFilterConfig.FilterMaskIdHigh = B2its_17_0_IDE_RTR>>16;
    sFilterConfig.FilterMaskIdLow = B2its_17_0_IDE_RTR & 0x0FFFF;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    	{Error_Handler();
 }  /* Filter configuration Error */


    /* Start the CAN peripheral */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {  Error_Handler(); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); }   /* Start Error */


    /* Activate CAN RX notification */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {  Error_Handler(); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); }  /* Notification Error */

  }

///////////////////////////////////////////////////////////////////////////////




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
