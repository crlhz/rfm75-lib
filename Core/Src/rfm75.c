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
uint8_t* spi_transaction(uint8_t cmd, uint8_t* data, uint8_t size);
//**********************************//


//***PUBLIC FUNCTIONS***//
void rfm_init(){
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

}


void rfm_transmit(uint8_t* data){
	spi_transaction(CMD_W_TX_PAYLOAD, data, sizeof(data));
}


uint8_t* rfm_receive(uint8_t size){
	uint8_t tx[size];
	return spi_transaction(CMD_R_RX_PAYLOAD, tx, size);
}
//**********************//


//***PRIVATE FUNCIONS***//
uint8_t* spi_transaction(uint8_t cmd, uint8_t* data, uint8_t size){
	uint8_t rx[size];
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, data, rx, size, 100);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	return *rx;
}
//**********************//
