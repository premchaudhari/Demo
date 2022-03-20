/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ep.h"
#include "LCD16X2.h"
#include "ftoa.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
char adcVal2[10];
char Scale_mm_val[10];
char res[20];
float offset = 0.0;
int j=0;
uint32_t data2[5] = {0};
uint32_t Rx_Data[5];
float setpoint;
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
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void lcd_init(void);
void lcd_cmd(unsigned char);
void lcd_data(unsigned char);
void lcd_string(char *p);
uint32_t adcCount=0;
float mmconversion=0.0;
float adcVal;
char buf[10];
char d1,d2,d3,d4;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void msdelay(unsigned int time)
{
	int t1,t2;
	for(t1=0;t1<time;t1++)
		for(t2=0;t2<1104;t2++);
}
void key_check()
{
	int k=1;
loop:
    HAL_Delay(500);
	if(GPIO_PIN_RESET==HAL_GPIO_ReadPin(GPIOB, key1_Pin))
		k=k+1;
	if(k>2)
		k=0;

	switch(k)
	{
	case 1:
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("set Point:");
		ftoa(setpoint, res, 1);
		lcd_put_cur(0, 11);
		lcd_send_string(res);
		break;
	case 2:
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("Offset:");
		ftoa(offset, res, 1);
		lcd_put_cur(0, 8);
		lcd_send_string(res);
		if(GPIO_PIN_RESET==HAL_GPIO_ReadPin(GPIOB, key2_Pin))
		{
			HAL_Delay(500);
			offset=offset+0.1;
			ftoa(offset, res, 1);
			lcd_put_cur(0, 8);
			lcd_send_string(res);
		}
		if(GPIO_PIN_RESET==HAL_GPIO_ReadPin(GPIOB, key3_Pin))
		{
			HAL_Delay(500);
			if(offset>0)
			offset=offset-0.1;
			ftoa(offset, res, 1);
			lcd_put_cur(0, 8);
			lcd_send_string(res);
		}
		break;
	}
	if(GPIO_PIN_RESET==HAL_GPIO_ReadPin(GPIOB, key4_Pin))
	{
		k=0;
		data2[0]=offset*10;
		data2[1]=setpoint*10;
		Flash_Write_Data(0x0801FC00 , (uint32_t *)data2,5);
		lcd_clear();
	}
	else
		goto loop;

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
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	  lcd_init ();
//	  lcd_put_cur(0, 0);
//	  lcd_send_string("HELLO ");
//	  lcd_send_string("WORLD ");
//	  lcd_send_string("FROM");
//
//	  lcd_put_cur(1, 0);
//	  lcd_send_string("CONTROLLERS TECH");
//	  HAL_Delay(3000);


		  lcd_put_cur(0, 2);
		  lcd_send_string("Welcome ");
		  lcd_put_cur(1, 5);
		  lcd_send_string("To ");
		  lcd_put_cur(2, 0);
		  lcd_send_string("Borgwarner ");
		   for(int i=0;i<5;i++)
		   {
			   lcd_send_cmd(0x1c);		    /* Shift entire display to right */
			   HAL_Delay(300);
		   }
		   for(int i=5;i>0;i--)
		   {
			   lcd_send_cmd(0x18);	        /* Shift entire display to left */
			   HAL_Delay(300);
		   }
	  lcd_clear();
     // HAL_Delay(2000);
	  Flash_Read_Data(0x0801FC00 ,Rx_Data, 5);
	  offset=(float)Rx_Data[0]/(float)10;
	  setpoint=(float)Rx_Data[1]/(float)10;

	while (1)
	{
		/* USER CODE END WHILE */
		HAL_ADC_Start_IT(&hadc1);
		if(GPIO_PIN_RESET==HAL_GPIO_ReadPin(GPIOB, key1_Pin))
		{
			key_check();
		}
		else
		{
			//lcd_clear();
			itoa(adcCount, adcVal2, 10);
			lcd_put_cur(0, 0);
		    lcd_send_string("ADC Count:");
			lcd_send_string(adcVal2);
			ftoa(adcVal, Scale_mm_val, 1);
			// LCD_String_xy(3,0,"Scale:");
			lcd_put_cur(1, 5);
			lcd_send_string(Scale_mm_val);
			lcd_send_string("mm");
			//check for value of scale with offset
			if(GPIO_PIN_RESET!=HAL_GPIO_ReadPin(GPIOB, key4_Pin))
			{
				setpoint=adcVal;
			}
			if((adcVal<168)&&(GPIO_PIN_RESET==HAL_GPIO_ReadPin(GPIOB, Phenumaticc_Pin)))
			{
				if((setpoint<(adcVal+setpoint))&&(setpoint>(setpoint-offset)))
				{
					//component ok
					HAL_GPIO_WritePin(GPIOB, componentok_Pin, GPIO_PIN_SET);
					j=0;
				}
				else
				{
					//component Not ok
					HAL_GPIO_WritePin(GPIOB, componentNOTok_Pin, GPIO_PIN_SET);
					j++;
					if(j==5)
					{
						j=0;
						//alarm relay
						HAL_GPIO_WritePin(GPIOB, Alarm_Pin, GPIO_PIN_SET);
					}

				}
			}
		}
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adcCount = HAL_ADC_GetValue(&hadc1);
	adcVal = (178.0 - (float)((float)adcCount * 0.04524));
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, D1_Pin|D2_Pin|D3_Pin|D4_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, RS_Pin|EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : D1_Pin D2_Pin D3_Pin D4_Pin */
	GPIO_InitStruct.Pin = D1_Pin|D2_Pin|D3_Pin|D4_Pin|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/*Configure GPIO pins : RS_Pin EN_Pin */
	GPIO_InitStruct.Pin = RS_Pin|EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : key1_Pin key2_Pin key3_Pin key4_Pin */
	GPIO_InitStruct.Pin = key1_Pin|key2_Pin|key3_Pin|key4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
