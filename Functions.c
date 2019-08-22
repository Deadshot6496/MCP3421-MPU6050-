/*author karthik*/



/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t mpu6050_wakeup[2]= {0x6B,0x00};

uint8_t gyro_config[2] = {0x1B,0x10};

uint8_t accel_config[2] = {0x1C,0x10};



uint8_t gyro_add = 0x43, accel_add = 0x3B;

int16_t x,y,z,x_acc,y_acc,z_acc;

float pitch=0,roll=0,yaw=0,pitch_angle=0,roll_angle=0,yaw_angle=0, a_pitch_cal, a_roll_cal, a_yaw_cal;

float p_cal = 0, r_cal = 0, y_cal = 0;

int16_t gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z;

float a_pitch_angle,a_yaw_angle,a_roll_angle,SCALAR;

float pitch_output, roll_output;
uint8_t imu_start = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

int16_t get_gyro_avg(char x, uint16_t buff );

int16_t get_accel_avg(char x, uint16_t buff );

void setup_gyro();

void calibrate_gyro();

void calibrate_accel();

void get_gyro();

void get_accel();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)
	{

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	setup_gyro();
	
	calibrate_gyro();
	
		
	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		get_accel();
    get_gyro();
		
		/********Calibrate Gyro ****/
		pitch = gyro_x - p_cal;
		roll = gyro_y - r_cal;
		yaw = gyro_z - y_cal;
		/**********END**********/
		
		/******GET ANGLES From RAW GYRO DATA******/
		pitch_angle += (pitch / 32.8) * 0.0010 * 2;//Integration for dt = 0.001. 
		roll_angle += (roll / 32.8) * 0.0010 * 2;// Multiplied 2 for angle correction from 45 to 90 deg
		yaw_angle += (yaw / 32.8) * 0.0010 * 2;
		/******GET ANGLES From RAW GYRO DATA******/
		
		/********GET ANGLES ffrom ACCELEROMETER************/
		pitch_angle += roll_angle * sin((yaw / 32.8) * 0.0010 * 2 * (3.14/180));
		
		roll_angle -= pitch_angle * sin((yaw / 32.8) * 0.0010 * 2 * (3.14/180));
		/*******END**********/
		SCALAR = sqrt((accel_x * accel_x)+(accel_y * accel_y)+(accel_z * accel_z));	//Tried own simplification from nowmalisation of gravitational vector
		
		/*******Simplified Formula from NXP Aerospace ACCEL DOCS*****/
		a_pitch_angle = atan2(accel_y, accel_z) *(180/3.14);
		a_roll_angle = atan2((-1 * accel_x),sqrt((accel_y*accel_y)+(accel_z*accel_z))) * (180/3.14);
		/*******END*****/
		
		
		/******ACCELEROMETER Offset correction******/
		a_pitch_angle -= 6.3;
		a_roll_angle -= 8.1;
		/******ACCELEROMETER Offset correction******/
		
		if(imu_start)
		{
			/**********Complementary Filter******/
		  pitch_angle = (pitch_angle )* 0.995 + a_pitch_angle * 0.005;
			roll_angle = roll_angle * 0.9996 + a_roll_angle *0.0004;
			/**********Complementary Filter END******/
		}
		else{
			/*********GYRO OFFSET COrrection******/
			pitch_angle = a_pitch_angle;
			roll_angle = a_roll_angle;
			imu_start = 1;
			/*********GYRO OFFSET COrrection END******/
		}
		
		pitch_output = /*pitch_output * 0.9 + */pitch_angle;	// No need to add another filter for values 0.995 and 0.005
		roll_output = roll_angle;
		
		HAL_Delay(1);			// dt = 0.001
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 320;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int16_t get_gyro_avg(char x, uint16_t buff )
{
		int32_t avg[3] = {0,0,0};			// Buffer
		int16_t mov_avg[3] = {0,0,0};			//result
		uint8_t gyro_data[6];
													// address set to (0x69)  address pin pulled up 
												// implemented to calibrate
	/****MOVING AVG ALGORITHM START*****/

			for(uint16_t j = 0; j<buff; j++)
			{
				HAL_I2C_Master_Transmit(&hi2c1,0xD2,&gyro_add,1,500);		//
				HAL_I2C_Master_Receive(&hi2c1,0xD3,gyro_data,6,500);
				gyro_x = ((gyro_data[0] << 8) | gyro_data[1]);
				gyro_y = ((gyro_data[2] << 8) | gyro_data[3]);
				gyro_z = ((gyro_data[4] << 8) | gyro_data[5]);
				avg[0] += gyro_x;
				avg[1] += gyro_y;
				avg[2] += gyro_z;
			}
			HAL_Delay(1);
			mov_avg[0] = avg[0] / buff;
			mov_avg[1] = avg[1] / buff;
			mov_avg[2] = avg[2] / buff;

		/****MOVING AVG ALGORITHM END*****/
			if(mov_avg[0] > (gyro_x + 50) || mov_avg[0] < (gyro_x - 50) || mov_avg[1] > (gyro_y + 50) || mov_avg[0] < (gyro_y - 50) || mov_avg[2] > (gyro_z + 50) || mov_avg[2] < (gyro_z - 50))
			{
				get_gyro_avg(x,buff);
			}
			else
			{
				return ((x=='x') ? mov_avg[0] : (x=='y') ? mov_avg[1] : (x=='z') ? mov_avg[2] : 0);
			}
		
}

void get_gyro()
{
	uint8_t gyro_data[6];
				HAL_I2C_Master_Transmit(&hi2c1,0xD2,&gyro_add,1,500);		//
				HAL_I2C_Master_Receive(&hi2c1,0xD3,gyro_data,6,500);
				gyro_x = ((gyro_data[0] << 8) | gyro_data[1]);
				gyro_y = ((gyro_data[2] << 8) | gyro_data[3]);
				gyro_z = ((gyro_data[4] << 8) | gyro_data[5]);
}

int16_t get_accel_avg(char x, uint16_t buff )
{
		int64_t avg[3];			// Buffer
		int16_t mov_avg[3];			//result
		uint8_t accel_data[6];
	// address set to (0x69)  address pin pulled up 
												// implemented to reduce noise
	/****MOVING AVG ALGORITHM START*****/

			for(uint16_t j = 0; j<buff; j++)
			{
			HAL_I2C_Master_Transmit(&hi2c1,0xD2,&accel_add,1,500);
			HAL_I2C_Master_Receive(&hi2c1,0xD3,accel_data,6,500);
			accel_x = accel_data[0] << 8 | accel_data[1];
			accel_y = accel_data[2] << 8 | accel_data[3];
			accel_z = accel_data[4] << 8 | accel_data[5];
			avg[0] += accel_x;
			avg[1] += accel_y;
			avg[2] += accel_z;
			HAL_Delay(1);
			}
			mov_avg[0] = avg[0] / buff;
			mov_avg[1] = avg[1] / buff;
			mov_avg[2] = avg[2] / buff;

		/****MOVING AVG ALGORITHM END*****/
		return ((x=='x') ? mov_avg[0] : (x=='y') ? mov_avg[1] : mov_avg[2]);
}

void get_accel()
{
	uint8_t accel_data[6];
	
			HAL_I2C_Master_Transmit(&hi2c1,0xD2,&accel_add,1,500);
			HAL_I2C_Master_Receive(&hi2c1,0xD3,accel_data,6,500);
			accel_x = accel_data[0] << 8 | accel_data[1];
			accel_y = accel_data[2] << 8 | accel_data[3];
			accel_z = accel_data[4] << 8 | accel_data[5];	
}

void setup_gyro()
{
	HAL_I2C_Master_Transmit(&hi2c1,0xD2,mpu6050_wakeup,2,500);
	HAL_I2C_Master_Transmit(&hi2c1,0xD2,gyro_config,2,100);
	HAL_I2C_Master_Transmit(&hi2c1,0xD2,accel_config,2,100);
	HAL_Delay(1);
}

void calibrate_gyro()
{
	p_cal = (float)get_gyro_avg('x',500);
	HAL_Delay(1);
	r_cal = (float)get_gyro_avg('y',500);
	HAL_Delay(1);
	y_cal = (float)get_gyro_avg('z',500);
	HAL_Delay(1);
}

void calibrate_accel()
{
	a_pitch_cal = get_accel_avg('x',500);
	a_roll_cal = get_accel_avg('y',500);
	a_yaw_cal = get_accel_avg('z', 500);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
