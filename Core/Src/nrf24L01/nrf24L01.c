/*
 * nrf24L01.c
 *
 *  Created on: Apr 6, 2023
 *      Author: odemki
 */


#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
//#include "cmsis_os.h"

#include "nrf24L01/nrf24L01.h"
#include "cmsis_os.h"
#include "queue.h"

#define TX_ADR_WIDTH 3
#define TX_PLOAD_WIDTH 18    // 5


extern char str1[40];
extern uint8_t buf1[40];

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xb3,0xb4,0x01};
uint8_t RX_BUF[TX_PLOAD_WIDTH] = {0};

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;

//extern osMessageQueueId_t RQueueHandle;
extern osMessageQueueId_t DATAQueueHandle;

uint8_t NRF24_ReadReg(uint8_t addr);
static void NRF24_ToggleFeatures(void);
static void NRF24_WriteReg(uint8_t addr, uint8_t dt);
static void NRF24_FlushRX(void);
static void NRF24_FlushTX(void);
void NRF24L01_RX_Mode(void);
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes);

// -------------------------------------------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
	micros *= (SystemCoreClock / 1000000) / 7;   // 9

	/* Wait till done */
	while (micros--) ;
}
// -------------------------------------------------------------------------------------
void NRF24_init_TX(void)
{
	 CE_RESET;
	 //DelayMicro(5000);
	 osDelay(5);
	// HAL_Delay(5);
	 NRF24_WriteReg(CONFIG, 0x0a); 			// Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:0 (Transmitter)
	 //DelayMicro(5000);
//	 HAL_Delay(5);
	 osDelay(5);

	 NRF24_WriteReg(EN_AA, 0x01); 			// Enable pipe 0
	 NRF24_WriteReg(EN_RXADDR, 0x01); 		// Enable Pipe 0
	 NRF24_WriteReg(SETUP_AW, 0x01); 		// Setup address width=3 bytes
	 NRF24_WriteReg(SETUP_RETR, 0x5F);		// 1500us, 15 retrans

	 NRF24_ToggleFeatures();

	 NRF24_WriteReg(FEATURE, 0);
	 NRF24_WriteReg(DYNPD, 0);
	 NRF24_WriteReg(STATUS, 0x70);			// Reset flags for IRQ
	 NRF24_WriteReg(RF_CH, 76); 			//  2476 MHz
	 //NRF24_WriteReg(RF_SETUP, 0x06);		// TX_PWR:0dBm, Datarate:1Mbps      - WAS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	 NRF24_WriteReg(RF_SETUP, 0x26);  		// TX_PWR:0dBm, Datarate:250kbps	- New version

	 NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);

	 NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);

	 NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH);	 //Number of bytes in RX payload in data pipe 0

	 NRF24L01_RX_Mode();
	 LED_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24_init_RX(void)
{
	 CE_RESET;
	 //DelayMicro(5000);
	 osDelay(5);
//	 HAL_Delay(5);
	 NRF24_WriteReg(CONFIG, 0x0a); 			// Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:0 (Transmitter)
	 //DelayMicro(5000);
//	 HAL_Delay(5);
	 osDelay(5);

	 NRF24_WriteReg(EN_AA, 0x02); 			// Enable pipe1
	 NRF24_WriteReg(EN_RXADDR, 0x02); 		// Enable Pipe1
	 NRF24_WriteReg(SETUP_AW, 0x01); 		// Setup address width=3 bytes
	 NRF24_WriteReg(SETUP_RETR, 0x5F);		// 1500us, 15 retrans

	 NRF24_ToggleFeatures();

	 NRF24_WriteReg(FEATURE, 0);
	 NRF24_WriteReg(DYNPD, 0);
	 NRF24_WriteReg(STATUS, 0x70);			// Reset flags for IRQ
	 NRF24_WriteReg(RF_CH, 76); 			//   2476 MHz
	 //NRF24_WriteReg(RF_SETUP, 0x06);		// TX_PWR:0dBm, Datarate:1Mbps  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< WAS
	 NRF24_WriteReg(RF_SETUP, 0x26); 	    // TX_PWR:0dBm, Datarate:250kbps	<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	 NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);

	 NRF24_Write_Buf(RX_ADDR_P1, TX_ADDRESS, TX_ADR_WIDTH);

	 NRF24_WriteReg(RX_PW_P1, TX_PLOAD_WIDTH);	 //Number of bytes in RX payload in data pipe 1

	 NRF24L01_RX_Mode();
	 LED_OFF;
}
// -------------------------------------------------------------------------------------
uint8_t NRF24_ReadReg(uint8_t addr)
{
	uint8_t dt=0, cmd;
	CS_ON;

	HAL_SPI_TransmitReceive(&hspi2, &addr, &dt, 1, 1000);

	if (addr != STATUS)		//если адрес равен адрес регистра статус то и возварщаем его состояние
	{
		cmd = 0xFF;
		HAL_SPI_TransmitReceive(&hspi2, &cmd, &dt, 1, 1000);
	}

	CS_OFF;

	return dt;
}
// -------------------------------------------------------------------------------------
void NRF24_WriteReg(uint8_t addr, uint8_t dt)
{
	addr |= W_REGISTER;//включим бит записи в адрес

	CS_ON;

	HAL_SPI_Transmit(&hspi2,&addr,1,1000);	//отправим адрес в шину
	HAL_SPI_Transmit(&hspi2,&dt,1,1000);	//отправим данные в шину

	CS_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24_ToggleFeatures(void)
{
	uint8_t dt[1] = {ACTIVATE};

	CS_ON;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);

	DelayMicro(1);

	dt[0] = 0x73;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);
	CS_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24_Read_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
	CS_ON;
	HAL_SPI_Transmit(&hspi2,&addr,1,1000);				//отправим адрес в шину

	HAL_SPI_Receive(&hspi2,pBuf,bytes,1000);			//отправим данные в буфер

	CS_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
	addr |= W_REGISTER;									//включим бит записи в адрес

	CS_ON;
	HAL_SPI_Transmit(&hspi2,&addr,1,1000);				//отправим адрес в шину

	DelayMicro(1);

	HAL_SPI_Transmit(&hspi2,pBuf,bytes,1000);			//отправим данные в буфер

	CS_OFF;
}
// -------------------------------------------------------------------------------------
static void NRF24_FlushRX(void)
{
	uint8_t dt[1] = {FLUSH_RX};

	CS_ON;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);
	DelayMicro(1);
	CS_OFF;
}
// -------------------------------------------------------------------------------------
static void NRF24_FlushTX(void)
{
	uint8_t dt[1] = {FLUSH_TX};

	CS_ON;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);
	DelayMicro(1);
	CS_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24L01_RX_Mode(void)
{
	uint8_t regval=0x00;
	regval = NRF24_ReadReg(CONFIG);

	regval |= (1<<PWR_UP)|(1<<PRIM_RX);

	NRF24_WriteReg(CONFIG, regval);
	CE_SET;

	DelayMicro(150);

	// Flush buffers
	NRF24_FlushRX();
	NRF24_FlushTX();
}
// -------------------------------------------------------------------------------------
void testReadWriteSetingd(void)
{
	uint8_t dt_reg=0;

	dt_reg = NRF24_ReadReg(CONFIG);
	sprintf(str1,"CONFIG: 0x%02Xrn",dt_reg);

	//  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

	dt_reg = NRF24_ReadReg(EN_AA);
	sprintf(str1,"EN_AA: 0x%02Xrn",dt_reg);

	//  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	dt_reg = NRF24_ReadReg(EN_RXADDR);
	sprintf(str1,"EN_RXADDR: 0x%02Xrn",dt_reg);

	//  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	dt_reg = NRF24_ReadReg(STATUS);
	sprintf(str1,"STATUS: 0x%02Xrn",dt_reg);

//	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	dt_reg = NRF24_ReadReg(RF_SETUP);
	sprintf(str1,"RF_SETUP: 0x%02Xrn",dt_reg);

//	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

    NRF24_Read_Buf(TX_ADDR,buf1,3);
	sprintf(str1,"TX_ADDR: 0x%02X, 0x%02X, 0x%02Xrn",buf1[0],buf1[1],buf1[2]);

//	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	NRF24_Read_Buf(RX_ADDR_P0,buf1,3);
	sprintf(str1,"RX_ADDR: 0x%02X, 0x%02X, 0x%02Xrn",buf1[0],buf1[1],buf1[2]);

//	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);



}
// -------------------------------------------------------------------------------------
void testDelay(void)
{
	//HAL_GPIO_TogglePin(GPIOC, LED_Pin);
	DelayMicro(1000);

}
// -------------------------------------------------------------------------------------
void NRF24L01_TX_Mode(uint8_t *pBuf)
{
	NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
	CE_RESET;

	// Flush buffers
	NRF24_FlushRX();
	NRF24_FlushTX();
}
// -------------------------------------------------------------------------------------
void NRF24_Transmit(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
	CE_RESET;
	CS_ON;

	HAL_SPI_Transmit(&hspi2,&addr,1,1000);			// Send address in buss
	DelayMicro(1);
	HAL_SPI_Transmit(&hspi2,pBuf,bytes,1000);		// Send data in buss

	CS_OFF;
	CE_SET;
}
// -------------------------------------------------------------------------------------
uint8_t NRF24L01_Send(uint8_t *pBuf)
{
	uint8_t status=0x00, regval=0x00;

	NRF24L01_TX_Mode(pBuf);

	regval = NRF24_ReadReg(CONFIG);
	regval |= (1<<PWR_UP);
	regval &= ~(1<<PRIM_RX);							// Turn on TX mode

	NRF24_WriteReg(CONFIG,regval);
	DelayMicro(150);

	uint8_t dt_reg = NRF24_ReadReg(CONFIG);				// For debug

	NRF24_Transmit(WR_TX_PLOAD, pBuf, TX_PLOAD_WIDTH);
	CE_SET;
	DelayMicro(15); 			// minimum 10us high pulse (Page 21)
	CE_RESET;

	while((GPIO_PinState)IRQ == GPIO_PIN_SET) {}

	//HAL_GPIO_TogglePin(GPIOA, TEST_OUT_Pin);    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< for debug latency

	status = NRF24_ReadReg(STATUS);

	if(status&TX_DS) 			//tx_ds == 0x20   If data was transmitted
	{
	    LED_TGL;
	    NRF24_WriteReg(STATUS, 0x20);
	}
	else if(status&MAX_RT)
	{
		NRF24_WriteReg(STATUS, 0x10);
		NRF24_FlushTX();
	}

	regval = NRF24_ReadReg(OBSERVE_TX);

	NRF24L01_RX_Mode();

	return regval;

}
// -------------------------------------------------------------------------------------
void NRF24L01_Transmit(void)
{
	static uint8_t retr_cnt, dt;
	static uint16_t i=1, retr_cnt_full;

//	HAL_Delay(500);
	osDelay(500);

	memcpy(buf1,(uint8_t*)&i,2);

	if(retr_cnt_full>999)
	{
		retr_cnt_full=999;
	}
	memcpy(buf1+2,(uint8_t*)&retr_cnt_full,2);


	char test_tx_data[] = "123\n\r";

	dt = NRF24L01_Send(test_tx_data);
	//dt = NRF24L01_Send(buf1);
	retr_cnt = dt & 0x0F;

	i++;
	retr_cnt_full += retr_cnt;

	if(i>=999)
	{
		i=1;
	}
}
// ------------------------------------------------------------------------------------
void led_test(void)
{
	 // LED_ON;
	 LED_OFF;
}
// -------------------------------------------------------------------------------------
void NRF24L01_Receive(void)
{
	uint8_t status=0x01;
	uint16_t dt=0;

	while((GPIO_PinState)IRQ == GPIO_PIN_SET) {}			//	 Замінити це на нотифікацію від зміни ножки або семафор

	status = NRF24_ReadReg(STATUS);

	sprintf(str1,"STATUS: 0x%02X\r\n",status);
	//HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
	LED_TGL;

	DelayMicro(10);

	status = NRF24_ReadReg(STATUS);

	if(status & 0x40)			// If new data in RX buffer available
	{
		NRF24_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);
	    dt = *(int16_t*)RX_BUF;
	    //Clear_7219();
	    //Number_7219(dt);
	    dt = *(int16_t*)(RX_BUF+2);
	    //NumberL_7219(dt);
	    NRF24_WriteReg(STATUS, 0x40);

	    //char test_data[10] = "TEST\n\r";
	    HAL_UART_Transmit(&huart1, RX_BUF, sizeof(RX_BUF), 1000);

	  }
}
// -------------------------------------------------------------------------------------
void NRF24L01_Transmit_Real_Data(uint8_t* data)
{

//	static uint8_t retr_cnt, dt;
//	static uint16_t i=1, retr_cnt_full;
//
//	//	HAL_Delay(500);
//	osDelay(500);
//
//	memcpy(buf1,(uint8_t*)&i,2);
//
//	if(retr_cnt_full>999)
//	{
//		retr_cnt_full=999;
//	}
//	memcpy(buf1+2,(uint8_t*)&retr_cnt_full,2);


	char test_tx_data[] = "123\n\r";

	NRF24L01_Send(data);
	//dt = NRF24L01_Send(buf1);
//	retr_cnt = dt & 0x0F;
//
//	i++;
//	retr_cnt_full += retr_cnt;
//
//	if(i>=999)
//	{
//		i=1;
//	}

}
// -------------------------------------------------------------------------------------
void NRF24L01_Receive_Real_Data(void)
{
	uint8_t status=0x01;
	uint16_t dt=0;

	while((GPIO_PinState)IRQ == GPIO_PIN_SET) {}			//	 Замінити це на нотифікацію від зміни ножки або семафор

	status = NRF24_ReadReg(STATUS);

	sprintf(str1,"STATUS: 0x%02X\r\n",status);
	//HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
	LED_TGL;

	DelayMicro(10);

	status = NRF24_ReadReg(STATUS);

	if(status & 0x40)			// If new data in RX buffer available
	{
		NRF24_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);
	    dt = *(int16_t*)RX_BUF;
	    //Clear_7219();
	    //Number_7219(dt);
	    dt = *(int16_t*)(RX_BUF+2);
	    //NumberL_7219(dt);
	    NRF24_WriteReg(STATUS, 0x40);



	    char str[50] = {0};

	    char str_main_buf[100] = {0};

	    uint16_t rx_data = 0;
	    char str_buf[15] = {0};

	    sprintf(str, "---------\n\r", rx_data);
	    HAL_UART_Transmit(&huart1, str, sizeof(str), 1000);

	    memset(str_buf, 0, sizeof(str_buf));

	    uint8_t r = 1;
	    for(uint8_t i = 0; i <= 12; i = i+2)
	    {

	    	rx_data = RX_BUF[0+i];
	    	rx_data = rx_data + (RX_BUF[1+i] * 256);
	    	sprintf(str, "R%d: %d  ", r, rx_data);

	    	strcat(str_main_buf, str);
	    	r++;
	    }

	    HAL_UART_Transmit(&huart1, str_main_buf, sizeof(str_main_buf), 1000);
	    //Зробити чергу довжиною в str_main_buf і парсити її в тасці StartDefaultTask


	    DATA DATA_t;



	    DATA_t.R1 = RX_BUF[0];
	    DATA_t.R1 = DATA_t.R1 + (RX_BUF[1] * 256);

	    DATA_t.R2 = RX_BUF[2];
	    DATA_t.R2 = DATA_t.R2 + (RX_BUF[3] * 256);

	    DATA_t.R3 = RX_BUF[4];
	   	DATA_t.R3 = DATA_t.R3 + (RX_BUF[5] * 256);

	   	DATA_t.R4 = RX_BUF[6];
	   	DATA_t.R4 = DATA_t.R4 + (RX_BUF[7] * 256);

	   	DATA_t.R5 = RX_BUF[8];
	   	DATA_t.R5 = DATA_t.R5 + (RX_BUF[9] * 256);

	   	DATA_t.joystick_X = RX_BUF[10];
	   	DATA_t.joystick_X = DATA_t.joystick_X + (RX_BUF[11] * 256);

	   	DATA_t.joystick_Y = RX_BUF[12];
	   	DATA_t.joystick_Y = DATA_t.joystick_Y + (RX_BUF[13] * 256);

		DATA_t.joystick_button = RX_BUF[14];
		DATA_t.joystick_button = DATA_t.joystick_button + (RX_BUF[15] * 256);


	    xQueueSendToBack(DATAQueueHandle, &DATA_t, 1000);



	  }
}





