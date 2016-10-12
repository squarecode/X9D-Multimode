/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "iface_cc2500.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
	
	
	return ch;
}
void CC2500_WriteReg(uint8_t address, uint8_t data)
{
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_RESET);
	//CC25_CSN_off;
	HAL_SPI_Transmit(&hspi1,&address,1,1000);
	//SPI_Write(address); 
	
	//HAL_Delay(xyz);
	//NOP();

	HAL_SPI_Transmit(&hspi1,&data,1,1000);
	//SPI_Write(data);
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_SET);
	//CC25_CSN_on;
} 
void CC2500_ReadRegisterMulti(uint8_t address, uint8_t data[], uint8_t length)
{
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_RESET);
	//CC25_CSN_off
	
	uint8_t txData = ((uint8_t) CC2500_READ_BURST )| address; 
	HAL_SPI_Transmit(&hspi1,&txData,1,1000);
	
	HAL_SPI_Receive(&hspi1,data,length,1000);
	
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_SET);
	//CC25_CSN_on;
}

uint8_t CC2500_ReadReg(uint8_t address)
{ 
	uint8_t result;
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_RESET);
	//CC25_CSN_off
	
	uint8_t txData = ((uint8_t) CC2500_READ_SINGLE )| address; 
	HAL_SPI_Transmit(&hspi1,&txData,1,1000);
	//SPI_Write(CC2500_READ_SINGLE | address);
	
	HAL_SPI_Receive(&hspi1,&result,1,1000);
	//result = SPI_Read();  
	
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_SET);
	//CC25_CSN_on;
	
	return(result); 
} 

void CC2500_ReadData(uint8_t *dpbuffer, uint8_t len)
{
	CC2500_ReadRegisterMulti(CC2500_3F_RXFIFO, dpbuffer, len);
}

void CC2500_Strobe(uint8_t state)
{
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_RESET);
	//CC25_CSN_off
	
	HAL_SPI_Transmit(&hspi1,&state,1,1000);
	
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_SET);
	//CC25_CSN_on;
}

static void CC2500_WriteRegisterMulti(uint8_t address, uint8_t data[], uint8_t length)
{
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_RESET);
	//CC25_CSN_off
	
	uint8_t txData = ((uint8_t) CC2500_WRITE_BURST )| address; 
	HAL_SPI_Transmit(&hspi1,&txData,1,1000);
	
	HAL_SPI_Transmit(&hspi1,data,length,1000);	
	
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_SET);
	//CC25_CSN_on;
}

void CC2500_WriteData(uint8_t *dpbuffer, uint8_t len)
{
	CC2500_Strobe(CC2500_SFTX);
	CC2500_WriteRegisterMulti(CC2500_3F_TXFIFO, dpbuffer, len);
	CC2500_Strobe(CC2500_STX);
}

void CC2500_SetTxRxMode(uint8_t mode)
{
	if(mode == TX_EN)
	{//from deviation firmware
		CC2500_WriteReg(CC2500_02_IOCFG0, 0x2F | 0x40);
		CC2500_WriteReg(CC2500_00_IOCFG2, 0x2F);
	}
	else
		if (mode == RX_EN)
		{
			CC2500_WriteReg(CC2500_02_IOCFG0, 0x2F);
			CC2500_WriteReg(CC2500_00_IOCFG2, 0x2F | 0x40);
		}
		else
		{
			CC2500_WriteReg(CC2500_02_IOCFG0, 0x2F);
			CC2500_WriteReg(CC2500_00_IOCFG2, 0x2F);
		}
}

uint8_t CC2500_Reset()
{
	CC2500_Strobe(CC2500_SRES);
	HAL_Delay(1);
	CC2500_SetTxRxMode(TXRX_OFF);
	return CC2500_ReadReg(CC2500_0E_FREQ1) == 0xC4;//check if reset
}
void CC2500_SetPower()
{
	uint8_t power=CC2500_BIND_POWER;
	if(IS_BIND_DONE_on)
		power=IS_POWER_FLAG_on?CC2500_HIGH_POWER:CC2500_LOW_POWER;
	if(IS_RANGE_FLAG_on)
		power=CC2500_RANGE_POWER;
	if(prev_power != power)
	{
		CC2500_WriteReg(CC2500_3E_PATABLE, power);
		prev_power=power;
	}
}

void __attribute__((unused)) frsky2way_init(uint8_t bind)
{
	// Configure cc2500 for tx mode
	//
	for(uint8_t i=0;i<36;i++)
	{
		uint8_t reg=cc2500_conf[i][0];
		uint8_t val=cc2500_conf[i][1];
		
		if(reg==CC2500_0C_FSCTRL0)
			val=option;
		else
			if(reg==CC2500_1B_AGCCTRL2)
				val=bind ? 0x43 : 0x03;
		CC2500_WriteReg(reg,val);
	}
	prev_option = option ;

	CC2500_SetTxRxMode(TX_EN);
	CC2500_SetPower();
	
	CC2500_Strobe(CC2500_SIDLE);	

	CC2500_WriteReg(CC2500_09_ADDR, bind ? 0x03 : rx_tx_addr[3]);
	CC2500_WriteReg(CC2500_07_PKTCTRL1, 0x05);
	CC2500_Strobe(CC2500_SIDLE);	// Go to idle...
	//
	CC2500_WriteReg(CC2500_0A_CHANNR, 0x00);
	CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);
	CC2500_Strobe(CC2500_SFRX);
}


uint16_t initFrSky_2way()
{
	if(IS_AUTOBIND_FLAG_on)
	{
		frsky2way_init(1);
		state = FRSKY_BIND;
	}
	else
	{
		frsky2way_init(0);
		state = FRSKY_DATA2;
	}
	return 10000;
}	

static uint8_t __attribute__((unused)) get_chan_num(uint16_t idx)
{
	uint8_t ret = (idx * 0x1e) % 0xeb;
	if(idx == 3 || idx == 23 || idx == 47)
		ret++;
	if(idx > 47)
		return 0;
	return ret;
}

void __attribute__((unused)) frsky2way_build_bind_packet()
{
	//11 03 01 d7 2d 00 00 1e 3c 5b 78 00 00 00 00 00 00 01
	//11 03 01 19 3e 00 02 8e 2f bb 5c 00 00 00 00 00 00 01
	packet[0] = 0x11;                
	packet[1] = 0x03;                
	packet[2] = 0x01;                
	packet[3] = rx_tx_addr[3];
	packet[4] = rx_tx_addr[2];
	uint16_t idx = ((state -FRSKY_BIND) % 10) * 5;
	packet[5] = idx;
	packet[6] = get_chan_num(idx++);
	packet[7] = get_chan_num(idx++);
	packet[8] = get_chan_num(idx++);
	packet[9] = get_chan_num(idx++);
	packet[10] = get_chan_num(idx++);
	packet[11] = 0x00;
	packet[12] = 0x00;
	packet[13] = 0x00;
	packet[14] = 0x00;
	packet[15] = 0x00;
	packet[16] = 0x00;
	packet[17] = 0x01;
}

// Channel value is multiplied by 1.5
uint16_t convert_channel_frsky(uint8_t num)
{
	return Servo_data[num] + Servo_data[num]/2;
}

static void __attribute__((unused)) frsky2way_data_frame()
{//pachet[4] is telemetry user frame counter(hub)
	//11 d7 2d 22 00 01 c9 c9 ca ca 88 88 ca ca c9 ca 88 88
	//11 57 12 00 00 01 f2 f2 f2 f2 06 06 ca ca ca ca 18 18
	packet[0] = 0x11;             //Length
	packet[1] = rx_tx_addr[3];
	packet[2] = rx_tx_addr[2];
	packet[3] = counter;//	
	#if defined TELEMETRY
		packet[4] = telemetry_counter;
	#else
		packet[4] = 0x00;
	#endif

	packet[5] = 0x01;
	
	packet[10] = 0;
	packet[11] = 0;
	packet[16] = 0;
	packet[17] = 0;
	for(uint8_t i = 0; i < 8; i++)
	{
		uint16_t value;
			value = convert_channel_frsky(i);
		if(i < 4)
		{
			packet[6+i] = value & 0xff;
			packet[10+(i>>1)] |= ((value >> 8) & 0x0f) << (4 *(i & 0x01));
		} 
		else
		{
			packet[8+i] = value & 0xff;
			packet[16+((i-4)>>1)] |= ((value >> 8) & 0x0f) << (4 * ((i-4) & 0x01));
		}
	}
} 
void frsky_check_telemetry(uint8_t *pkt,uint8_t len)
{
	if(pkt[1] == rx_tx_addr[3] && pkt[2] == rx_tx_addr[2] && len ==(pkt[0] + 3))
	{	   
		for (uint8_t i=3;i<len;i++)
			pktt[i]=pkt[i];				 
		telemetry_link=1;
		if(pktt[6])
			telemetry_counter=(telemetry_counter+1)%32;
		//
#if defined FRSKYX_CC2500_INO
		if ((pktt[5] >> 4 & 0x0f) == 0x08)
			{  
				seq_last_sent = 8;
				seq_last_rcvd = 0;
				pass=0;
			} 
			else
			{
				if ((pktt[5] >> 4 & 0x03) == (seq_last_rcvd + 1) % 4)
					seq_last_rcvd = (seq_last_rcvd + 1) % 4;
				else
					pass=0;//reset if sequence wrong
			}
#endif
	}
}

uint16_t ReadFrSky_2way()
{ 
	if (state < FRSKY_BIND_DONE)
	{
		frsky2way_build_bind_packet();
		CC2500_Strobe(CC2500_SIDLE);
		CC2500_WriteReg(CC2500_0A_CHANNR, 0x00);
		CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);		
		CC2500_Strobe(CC2500_SFRX);//0x3A
		CC2500_WriteData(packet, packet[0]+1);
		state++;
		return 9000;
	}
	if (state == FRSKY_BIND_DONE)
	{
		state = FRSKY_DATA2;
		frsky2way_init(0);
		counter = 0;
		BIND_DONE;
	}
	else
		if (state == FRSKY_DATA5)
		{
			CC2500_Strobe(CC2500_SRX);//0x34 RX enable
			state = FRSKY_DATA1;	
			return 9200;
		}
		
		
	counter = (counter + 1) % 188;	
	if (state == FRSKY_DATA4)
	{	//telemetry receive
		CC2500_SetTxRxMode(RX_EN);
		CC2500_Strobe(CC2500_SIDLE);
		CC2500_WriteReg(CC2500_0A_CHANNR, get_chan_num(counter % 47));
		CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);
		state++;
		return 1300;
	}
	else
	{
		if (state == FRSKY_DATA1)
		{
			len = CC2500_ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
			printf("LENGTH %d \r\n",len);
			if (len && len<=MAX_PKT)//27 bytes
			{
				CC2500_ReadData(pkt, len);	//received telemetry packets			
				frsky_check_telemetry(pkt,len);	//check if valid telemetry packets and buffer them.
			}			
			CC2500_SetTxRxMode(TX_EN);
			CC2500_SetPower();	// Set tx_power
		}
		CC2500_Strobe(CC2500_SIDLE);
		CC2500_WriteReg(CC2500_0A_CHANNR, get_chan_num(counter % 47));
		if ( prev_option != option )
		{
			CC2500_WriteReg(CC2500_0C_FSCTRL0,option);	// Frequency offset hack 
			prev_option = option ;
		}
		CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);
		CC2500_Strobe(CC2500_SFRX);        
		frsky2way_data_frame();
		CC2500_WriteData(packet, packet[0]+1);
		state++;
	}				
	return state == FRSKY_DATA4 ? 7500 : 9000;		
}
void proces_sport_data(uint8_t data)
{
	switch (pass)
	{
		case 0:
			if (data == START_STOP)
			{//waiting for 0x7e
				index2 = 0;
				pass = 1;
			}
			break;		
		case 1:
			if (data == START_STOP)	// Happens if missed packet
			{//waiting for 0x7e
				index2 = 0;
				pass = 1;
				break;		
			}
			if(data == BYTESTUFF)//if they are stuffed
				pass=2;
			else
				if (index2 < MAX_PKTX)		
					pktx[index2++] = data;		
			break;
		case 2:	
			if (index2 < MAX_PKTX)	
				pktx[index2++] = data ^ STUFF_MASK;	//unstuff bytes	
			pass=1;
			break;	
	} // end switch
	if (index2 >= FRSKY_SPORT_PACKET_SIZE)
	{//8 bytes no crc 
		if ( sport )
		{
			// overrun!
		}
		else
		{
			uint8_t i ;
			for ( i = 0 ; i < FRSKY_SPORT_PACKET_SIZE ; i += 1 )
			{
				pktx1[i] = pktx[i] ;	// Double buffer
			}
			sport = 1;//ok to send
		}
		pass = 0;//reset
	}
}

void TelemetryUpdate()
{
		if(telemetry_link)
		{		
			printf("### telemetry link! ###\r\n");
			if(pktt[4] & 0x80)
				rssi=pktt[4] & 0x7F ;
			else 
				RxBt = (pktt[4]<<1) + 1 ;
			for (uint8_t i=0; i < pktt[6]; i++)
			proces_sport_data(pktt[7+i]);
			telemetry_link=0;
		}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	//init servo positions to center
	Servo_data[0] = 1500;
	Servo_data[1] = 1500;
	Servo_data[2] = 1500;
	Servo_data[3] = 1500;
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

	printf("Hello from CC2500 Testbed!\r\n");
	
	HAL_GPIO_WritePin(CS_CC2500_GPIO_Port,CS_CC2500_Pin,GPIO_PIN_SET); //CC25_CSN_on;
	
	HAL_Delay(100); //Wait for CC2500 to come up
	
	//TODO: Read status of bind button
	
	
	//Reset CC2500
	CC2500_Reset();
	HAL_Delay(100);
	prev_power=0xFD;		// unused power value
	
	pass=0;
	telemetry_link=0;
	
	//BIND???
	if(IS_BIND_BUTTON_FLAG_on)
		AUTOBIND_FLAG_on;
	if(IS_AUTOBIND_FLAG_on)
		BIND_IN_PROGRESS;			// Indicates bind in progress for blinking bind led
	else
		BIND_DONE;
	
	
	

	/*????????????????????????????????????
	CTRL1_off;	//antenna RF2
	CTRL2_on;
	??????????????????????????????????????*/
	
	int next_callback = initFrSky_2way();
	remote_callback = ReadFrSky_2way;
	
	
	if(next_callback>32000)
	{ // next_callback should not be more than 32767 so we will wait here...
		HAL_Delay((next_callback-2000)>>10);
		// between 2-3ms left at this stage
	}
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(next_callback!=0)
		{			
			TelemetryUpdate();
			HAL_Delay(next_callback>>10);
			next_callback = 0;
		}
		else
		{
				next_callback = remote_callback();
		}

		/*
		HAL_Delay(100);
		*/
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
		/*HAL_Delay(100);
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
  
		*/
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_CC2500_GPIO_Port, CS_CC2500_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_CC2500_Pin */
  GPIO_InitStruct.Pin = CS_CC2500_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_CC2500_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
