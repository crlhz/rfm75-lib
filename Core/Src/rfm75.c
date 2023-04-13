  /*
  ******************************************************************************
  *
  *		File:	rfm75.c
  *		Name:	STM32 library for GFSK transceiver - RFM75-S
  *	  Author:	crlhz
  *	  	Date: 	2023-04-08
  *
  *	  github:	https://github.com/crlhz
  *	 contact:	crlhz@proton.me
  *
  ******************************************************************************
  */

#include "rfm75.h"

//***PRIVATE FUNCION DECLARATIONS***//
uint8_t* spi_transaction(uint8_t cmd, uint8_t* data);
//**********************************//


//***PUBLIC FUNCTIONS***//
void rfm_init(){
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	spi_transaction(CMD_W_REGISTER, 0x03);	//PWR_UP, PRX
}


void rfm_transmit(uint8_t* data){
	spi_transaction(CMD_W_REGISTER, 0x02);	//PWR_UP, PTX
	spi_transaction(CMD_W_TX_PAYLOAD, data);
}


uint8_t* rfm_receive(uint8_t size){
	uint8_t tx[size];
	return spi_transaction(CMD_R_RX_PAYLOAD, tx);
}
//**********************//


//***PRIVATE FUNCIONS***//
uint8_t* spi_transaction(uint8_t cmd, uint8_t* data){
	uint8_t size = sizeof(data);
	uint8_t rx[size+1];
	uint8_t tx[size+1];
	uint8_t i = 0;
	tx[0] = CMD_W_REGISTER;

	for(i = 0; i < size + 1; i++){
		tx[i + 1] = data[i];
	}

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, tx, rx, size+1, 100);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	return *rx;
}
//**********************//
