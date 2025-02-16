/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "ms5611.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "accel.h"
#include "ms5611.h"
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









/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  ms5611_init();
  BNO055_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

//  HAL_TIM_PWM_Start (&htim3,TIM_CHANNEL_2);
//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 10000);

  ms5611_config (&hi2c1, 0x77, 0x04, HAL_Delay);

  BNO055_Init_I2C(&hi2c2);


  HAL_StatusTypeDef ac = HAL_I2C_IsDeviceReady(&hi2c1, 0x77, 1, 100);
  if (ac == HAL_OK){

	  printf("ms5611 aktif \n");

  }
  else{
	  printf("cihaz takilmadi \n");
  }


  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
  GPIO_PinState	AnaPD = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
  GPIO_PinState	SurPD = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);



  void ucus ( float altitude_m,float hiz, float sicaklik, float basinc){



  	if (altitude_m == 0){
  		if (AnaPD == GPIO_PIN_SET || SurPD == GPIO_PIN_SET){
  			printf("inis sonlandi");
  		}
  		else{
  			HAL_Delay(6);
  			ucus(altitude_m, hiz, sicaklik, basinc);
  			  		}
  	}
  	else{
  		if (hiz < 746){
  			HAL_Delay(6);
  			ucus(altitude_m, hiz, sicaklik, basinc);
  		}
  		else{
  			while(hiz==0){
  				HAL_Delay(1);
  				printf("hiziniz maka");
  				HAL_Delay(20);
  				kurtarma(altitude_m);
  			}


  		}
  	}

  }

  void kurtarma(float altitude_m){
  	if (6000<altitude_m<10000){
  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
  		HAL_Delay(6);
  		kurtarma(altitude_m);
  }
  	else{
  		if (4000<altitude_m<6000){
  			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, SET);
  			while (altitude_m == 0){
  				HAL_Delay(1);
  			}
  			if (altitude_m == 0){
  				printf("roket indi man");
  			}
  		else{
  			HAL_Delay(6);
  			kurtarma(altitude_m);
  		}
  	}




  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  int hizdata[6];
	  float sicaklik = ms5611_read_rawTemp(&hi2c1, 0x77);
	  float basinc = ms5611_read_rawPress(&hi2c1, 0x77);
	  float irtifa;
	  float hiz = GetAccelData(&hi2c2, hizdata);
	  float yon = BNO055_Get_CalibrationBNO055_Get_Calibration(&hi2c2);




		ms5611_getTemperatureAndPressure(&sicaklik,&basinc,&irtifa);

		ucus(altitude_m, hiz, sicaklik, basinc);








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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
