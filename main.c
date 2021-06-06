/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
#define PWR_MGMT_l 0x6B // Power Management Register
#define MPU6050_ADDR 0xD0 //device address
#define SMPLRT_DIV 0x19 //sample rate divider
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

#define WHO_AM_I_REG 0x75 //after sending this device should return 0x68

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
float AX,AY,AZ;
float GX,GY,GZ;
char buf[4];
char buf_uart[30];
int size=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MPU6050_Init(void)
{
	uint8_t check, data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1,&check,1,1000);

	if(check == 104) //if device is present
	{
		//wake up sensor
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR, PWR_MGMT_l,1,&data,1,1000); //wake up the sensor
		//set DATA RATE of 1kHz
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV,1,&data,1,1000);
		//set accelerometer configuration at ACCEL_config register
		// XA_ST=0, YA_ST=0,ZA_ST, FS_SEL=0 -> +/-2g
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &data,1,1000);
		//set Gyroscope in GYRO_CONFIG register
		//XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +/- 250'/s
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &data,1,1000);
	}
}

void MPU6050_READ_ACC(void)
{
	uint8_t Rec_Data[6];
	float sensitivity = 16384.0;
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);

	AX = Accel_X_RAW/sensitivity;
	AY = Accel_Y_RAW/sensitivity;
	AZ = Accel_Z_RAW/sensitivity;
}

void MPU6050_READ_GYRO(void)
{
	uint8_t Data[6];
	float sensitivity = 131.0;

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, 1, Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Data[0]<<8 | Data[1]);
	Gyro_Y_RAW = (int16_t)(Data[2]<<8 | Data[3]);
	Gyro_Z_RAW = (int16_t)(Data[4]<<8 | Data[5]);

	GX = Gyro_X_RAW/sensitivity*(3.14/30);
	GY = Gyro_Y_RAW/sensitivity*(3.14/30);
	GZ = Gyro_Z_RAW/sensitivity*(3.14/30);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  lcd_init();
  MPU6050_Init();

  lcd_send_string("initialized");

  HAL_Delay(1000);

  lcd_clear();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	MPU6050_READ_ACC();
	MPU6050_READ_GYRO();

	  //Printing on LCD
	lcd_put_cur(0,0);
	//lcd_send_string("X");
	sprintf(buf, "%0.1f",AX);
	lcd_send_string(buf);

	lcd_put_cur(0,6);
	//lcd_send_string("Y");
	sprintf(buf, "%0.1f",AY);
	lcd_send_string(buf);

	lcd_put_cur(0,11);
	//lcd_send_string("Z");
	sprintf(buf, "%0.1f",AZ);
	lcd_send_string(buf);

	lcd_put_cur(1,0);
	//lcd_send_string("A");
	sprintf(buf, "%3d",(int)GX);
	lcd_send_string(buf);

	lcd_put_cur(1,5);
	//lcd_send_string("B");
	sprintf(buf, "%3d",(int)GY);
	lcd_send_string(buf);

	lcd_put_cur(1,10);
	//lcd_send_string("C");
	sprintf(buf, "%3d",(int)GZ);
	lcd_send_string(buf);

	//Sending to UART
	sprintf(buf_uart,"%0.1f %0.1f %0.1f %3d %3d %3d\n",
			AX, AY, AZ, (int)GX, (int)GY, (int)GZ);
	size=sizeof(buf_uart);
	HAL_UART_Transmit(&huart2, (uint8_t*)(buf_uart), size, 2000);

	HAL_Delay(250);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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
