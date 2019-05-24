		
    
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	GPIOB->ODR |= (1<<7) | (1<<6);
	HAL_UART_Receive_DMA(&huart1,rxData,8);
	adcInit();
	error.err = 52225.0968;   // Calibrated Value
	
	
	HAL_I2C_Master_Transmit(&hi2c1,0xD2,mpu6050_wakeup,2,500);
			
  /* USER CODE END 2 */
			
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
		checkAdc();

		x = get_gyro('x',100);
		y = get_gyro('y', 100);
		z = get_gyro('z', 100);
  
  }
  /* USER CODE END 3 */
}



void adcInit()
{
	
	HAL_I2C_Master_Transmit(&hi2c1,0xD1,&CONFIG_DATA,1,500);
	//HAL_I2C_Master_Receive_DMA(&hi2c1,0xD1,raw_data,4);
	//HAL_I2C_Master_Receive_IT(&hi2c1,0xD1,raw_data,4); 
	checkAdc();
	/*{
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }   
	}*/
}

uint32_t checkAdc()
{
	HAL_I2C_Master_Receive(&hi2c1,0xD1,raw_data,4,500);
	i_test++;
		if(raw_data[3] == CONFIG_DATA)
		{
			adc_data = raw_data[0] << 16 | raw_data[1] << 8 | raw_data[2];
			old_data = adc_data;
		}
		
		if(adc_data == 0 || adc_data > 2000000)
		{
			//adcInit();
			
			adc_data = old_data;
			
		}
	return adc_data;
}

void update_Coil()
{
  Registers[0] = adc_data;
	Registers[1] = get_temperature() * 10;
	
		 if((Coils[0] & (0x01)) == 0x01)
		{
			Motor_ON;
		}
		else 
		{
			Motor_OFF;
		}
		
		if((Coils[0] & (0x02)) == 0x02)
		{
			Heater_ON;
		}
		else 
		{
			Heater_OFF;
		}
}

float get_temperature()
{
	RTD = (error.err*adc_data/(131072-adc_data)) + 1;
	Resistance = -0.00000007*RTD*RTD + 0.0084*RTD + 99.63;
	Temperature =  0.0004*Resistance*Resistance + 2.4869*Resistance - 253.24;
	return Temperature;
}

int16_t get_gyro(char x, uint16_t buff )
{
		int32_t avg[3];			// Buffer
		int16_t mov_avg[3];			//result
												// address set to (0x69)
												// implemented to reduce noise
	/****MOVING AVG ALGORITHM START*****/
		for(uint8_t i = 0; i<3; i++)
		{
			for(uint16_t j = 0; j<buff; j++)
			{
				HAL_I2C_Master_Transmit(&hi2c1,0xD2,&gyro_add,1,500);		//add pin pulled up 
				HAL_I2C_Master_Receive(&hi2c1,0xD3,gyro_data,6,500);
				gyro_x = ((gyro_data[0] << 8) | gyro_data[1]);
				gyro_y = ((gyro_data[2] << 8) | gyro_data[3]);
				gyro_z = ((gyro_data[4] << 8) | gyro_data[5]);
				avg[i] = (i==0) ? (avg[i]+gyro_x) : (i==1) ? (avg[i]+gyro_y) : (avg[i] + gyro_z);
			}
			mov_avg[i] = avg[i] / 500;
			avg[i] = 0;
		}
		/****MOVING AVG ALGORITHM END*****/
		return ((x=='x') ? mov_avg[0] : (x=='y') ? mov_avg[1] : mov_avg[2]);
	}
