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

static SPI_HandleTypeDef *hspi_rfm;


//***PRIVATE FUNCION DECLARATIONS***//
static uint8_t* spi_transaction(uint8_t cmd, uint8_t* data, uint8_t size);
//**********************************//


//***PUBLIC FUNCTIONS***//
void rfm_init(SPI_HandleTypeDef *hspi){
	hspi_rfm = hspi;
	uint8_t x = 0x03;
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	spi_transaction(CMD_W_REGISTER, &x, 1);	//PWR_UP, PRX
}


void rfm_transmit(uint8_t* data, uint8_t size){
	uint8_t x = 0x02;
	spi_transaction(CMD_W_REGISTER, &x, 1);	//PWR_UP, PTX
	spi_transaction(CMD_W_TX_PAYLOAD, data, size);
}


uint8_t* rfm_receive(uint8_t size){
	uint8_t tx[size];	//in receiving mode MOSI data is meaningless
	return spi_transaction(CMD_R_RX_PAYLOAD, tx, size);
}

uint8_t rfm_read_register(uint8_t address){
	uint8_t tx;
	return *spi_transaction(CMD_R_REGISTER | address, &tx, 1);
}

void rfm_mask_rx(uint8_t on){
	uint8_t config = rfm_read_register(B0_CONFIG);
	on &= 0b01000000;
	config |= on;
	spi_transaction(CMD_W_REGISTER | B0_CONFIG, &config, 1);
}

void rfm_mask_tx(uint8_t on){
	uint8_t config = rfm_read_register(B0_CONFIG);
	on &= 0b00100000;
	config |= on;
	spi_transaction(CMD_W_REGISTER | B0_CONFIG, &config, 1);
}

void rfm_mask_max(uint8_t on){
	uint8_t config = rfm_read_register(B0_CONFIG);
	on &= 0b00010000;
	config |= on;
	spi_transaction(CMD_W_REGISTER | B0_CONFIG, &config, 1);
}

void rfm_enable_crc(uint8_t on){
	uint8_t config = rfm_read_register(B0_CONFIG);
	on &= 0b00001000;
	config |= on;
	spi_transaction(CMD_W_REGISTER | B0_CONFIG, &config, 1);
}

void rfm_set_crc_scheme(uint8_t n){
	uint8_t config = rfm_read_register(B0_CONFIG);
	n -= 1;	//2-1=1 -> 2 bytes, 1-1=0 -> 1 byte
	n <<= 2;
	config |= n;
	spi_transaction(CMD_W_REGISTER | B0_CONFIG, &config, 1);
}

void rfm_power_on(){
	uint8_t config = rfm_read_register(B0_CONFIG);
	config |= 0b00000010;
	spi_transaction(CMD_W_REGISTER | B0_CONFIG, &config, 1);
}

void rfm_power_off(){
	uint8_t config = rfm_read_register(B0_CONFIG);
	config &= 0b11111101;
	spi_transaction(CMD_W_REGISTER | B0_CONFIG, &config, 1);
}

void rfm_set_mode(uint8_t mode){
	uint8_t config = rfm_read_register(B0_CONFIG);
	config |= mode;
	spi_transaction(CMD_W_REGISTER | B0_CONFIG, &config, 1);
}

void rfm_set_auto_aa(uint8_t pipe){
	uint8_t reg = rfm_read_register(B0_EN_AA);
	reg |= (1 << pipe);
	spi_transaction(CMD_W_REGISTER | B0_EN_AA, &reg, 1);
}

void rfm_enable_pipe(uint8_t pipe){
	uint8_t reg = rfm_read_register(B0_EN_RXADDR);
	reg |= (1 << pipe);
	spi_transaction(CMD_W_REGISTER | B0_EN_RXADDR, &reg, 1);
}

void rfm_set_addr_width(uint8_t width){
	uint8_t reg = width - 2;
	spi_transaction(CMD_W_REGISTER | B0_SETUP_AW, &reg, 1);
}

void rfm_set_ret_delay(uint16_t delay){
	uint8_t reg = rfm_read_register(B0_SETUP_RETR);
	delay /= 2 - 1;	//250->0, 500->1 ...  4000->15
	reg |= (delay << 4);
	spi_transaction(CMD_W_REGISTER | B0_SETUP_RETR, &reg, 1);
}

void rfm_set_ret_count(uint8_t n){
	uint8_t reg = rfm_read_register(B0_SETUP_RETR);
	reg |= n;
	spi_transaction(CMD_W_REGISTER | B0_SETUP_RETR, &reg, 1);
}

void rfm_set_freq_channel(uint16_t n){

}

void rfm_set_air_dr(uint16_t rate){
	uint8_t reg = rfm_read_register(B0_RF_SETUP);
	switch(rate){
		case 250:
			reg |= 0b00100000;
			break;
		case 1000:
			reg |= 0b0000000
	}
}

//**********************//


//***PRIVATE FUNCIONS***//
static uint8_t* spi_transaction(uint8_t cmd, uint8_t* data, uint8_t size){
	uint8_t rx[size+1];
	uint8_t tx[size+1];
	uint8_t i = 0;
	tx[0] = cmd;

	for(i = 0; i < size + 1; i++){
		tx[i + 1] = data[i];
	}

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi_rfm, tx, rx, size+1, 100);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	return *rx;
}
//**********************//
