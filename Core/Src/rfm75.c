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
static void spi_transaction(uint8_t* tx, uint8_t* rx, uint8_t size);
static void spi_bank_0();
static void spi_bank_1();
//**********************************//


//***PUBLIC FUNCTIONS***//
void rfm_init(SPI_HandleTypeDef *hspi){
	hspi_rfm = hspi;
	//enter power down mode
	rfm_power_off();
	HAL_GPIO_WritePin(RFM_CE_GPIO_Port, RFM_CE_Pin, GPIO_PIN_RESET);
}

void rfm_standby(){
	rfm_power_on();
	HAL_GPIO_WritePin(RFM_CE_GPIO_Port, RFM_CE_Pin, GPIO_PIN_RESET);
}

void rfm_tx_mode(){
	rfm_power_on();
	rfm_set_mode(0);
	HAL_GPIO_WritePin(RFM_CE_GPIO_Port, RFM_CE_Pin, GPIO_PIN_SET);
}

void rfm_rx_mode(){
	rfm_power_on();
	rfm_set_mode(1);
	HAL_GPIO_WritePin(RFM_CE_GPIO_Port, RFM_CE_Pin, GPIO_PIN_SET);
}

void rfm_transmit(uint8_t* data, uint8_t size){
	uint8_t rx[size+1];
	uint8_t tx[size+1];
	tx[0] = CMD_W_TX_PAYLOAD;
	uint8_t i = 0;

	for(i=0;i<size+1;i++){
		tx[i+1] = data[i];
	}
	spi_transaction(&tx, &rx, size+1);
}

void rfm_receive(uint8_t* rx, uint8_t size){//OK
	uint8_t* tx = CMD_R_RX_PAYLOAD;
	spi_transaction(&tx, &rx, size);
}

uint8_t rfm_read_register(uint8_t address){//OK
	uint8_t* tx = address;
	uint8_t rx[2];
	spi_transaction(&tx, &rx, 2);
	return rx[1];
}

void rfm_write_register(uint8_t address, uint8_t data){//OK
	address = address | CMD_W_REGISTER;
	uint8_t tx[] = {address, data};
	uint8_t* rx;
	spi_transaction(tx, &rx, 2);
}

void rfm_mask_rx(uint8_t on){//OK
	uint8_t config = rfm_read_register(B0_CONFIG);
	config = (on) ? (config | (1 << 6)) : (config & ~(1 << 6));
	uint8_t tx[] = {CMD_W_REGISTER | B0_CONFIG, config};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_mask_tx(uint8_t on){//OK
	uint8_t config = rfm_read_register(B0_CONFIG);
	config = (on) ? (config | (1 << 5)) : (config & ~(1 << 5));
	uint8_t tx[] = {CMD_W_REGISTER | B0_CONFIG, config};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_mask_max(uint8_t on){//OK
	uint8_t config = rfm_read_register(B0_CONFIG);
	config = (on) ? (config | (1 << 4)) : (config & ~(1 << 4));
	uint8_t tx[] = {CMD_W_REGISTER | B0_CONFIG, config};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_enable_crc(uint8_t on){//OK
	uint8_t config = rfm_read_register(B0_CONFIG);
	config = (on) ? (config | (1 << 3)) : (config & ~(1 << 3));
	uint8_t tx[] = {CMD_W_REGISTER | B0_CONFIG, config};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_set_crc_scheme(uint8_t n){//OK
	uint8_t config = rfm_read_register(B0_CONFIG);
	n -= 1;	//2-1=1 -> 2 bytes, 1-1=0 -> 1 byte
	config = (n) ? (config | (1 << 2)) : (config & ~(1 << 2));
	uint8_t tx[] = {CMD_W_REGISTER | B0_CONFIG, config};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_power_on(){//OK
	uint8_t config = rfm_read_register(B0_CONFIG);
	config = (config | (1 << 1));
	uint8_t tx[] = {CMD_W_REGISTER | B0_CONFIG, config};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_power_off(){//OK
	uint8_t config = rfm_read_register(B0_CONFIG);
	config = (config & ~(1 << 1));
	uint8_t tx[] = {CMD_W_REGISTER | B0_CONFIG, config};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_set_mode(uint8_t mode){//OK
	uint8_t config = rfm_read_register(B0_CONFIG);
	config = (mode) ? (config | 1) : (config & ~(1));
	uint8_t tx[] = {CMD_W_REGISTER | B0_CONFIG, config};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_set_auto_aa(uint8_t pipe){//OK
	uint8_t reg = rfm_read_register(B0_EN_AA);
	reg |= (1 << pipe);
	uint8_t tx[] = {CMD_W_REGISTER | B0_EN_AA, reg};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_enable_pipe(uint8_t pipe){//OK
	uint8_t reg = rfm_read_register(B0_EN_RXADDR);
	reg |= (1 << pipe);
	uint8_t tx[] = {CMD_W_REGISTER | B0_EN_RXADDR, reg};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_set_addr_width(uint8_t width){//OK
	uint8_t reg = width - 2;
	uint8_t tx[] = {CMD_W_REGISTER | B0_SETUP_AW, reg};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_set_ret_delay(uint16_t delay){//OK
	uint8_t reg = rfm_read_register(B0_SETUP_RETR);
	delay /= 2 - 1;	//250->0, 500->1 ...  4000->15
	reg &= 0b00001111;	//reset values before OR
	reg |= (delay << 4);
	uint8_t tx[] = {CMD_W_REGISTER | B0_SETUP_RETR, reg};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_set_ret_count(uint8_t n){//OK
	uint8_t reg = rfm_read_register(B0_SETUP_RETR);
	reg &= 0b11110000;	//reset values before OR
	reg |= n;
	uint8_t tx[] = {CMD_W_REGISTER | B0_SETUP_RETR, reg};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_set_freq_channel(uint16_t n){

}

void rfm_set_air_dr(uint16_t rate){//OK
	uint8_t reg = rfm_read_register(B0_RF_SETUP);
	reg &= 0b11010000;	//reset values before OR

	switch(rate){
	case 250:
		reg |= 0b00100000;
		break;

	case 1000:	//00->do nothing
		break;

	case 2000:
		reg |= 0b00001000;
		break;
	}
	uint8_t tx[] = {CMD_W_REGISTER | B0_RF_SETUP, reg};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_pll_lock(uint8_t on){

}

void rfm_set_tx_pwr(uint8_t pwr){

}

void rfm_set_lna_gain(uint8_t mode){//OK
	uint8_t reg = rfm_read_register(B0_RF_SETUP);
	reg = (mode) ? (reg | 1) : (reg & ~(1));
	uint8_t tx[] = {CMD_W_REGISTER | B0_RF_SETUP, reg};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_set_rxcore_addr(uint8_t pipe, uint8_t* address){//OK
	uint8_t tx[6];
	uint8_t* rx;
	uint8_t i = 0;
	if(pipe == 0){
		tx[0] = CMD_W_REGISTER | B0_RX_ADDR_P0;
	}
	if(pipe == 1){
		tx[0] = CMD_W_REGISTER | B0_RX_ADDR_P1;
	}

	for(i=0;i<5;i++){
		tx[i+1] = address[i];
	}

	spi_transaction(&tx, &rx, 6);
}

void rfm_set_rx_addr(uint8_t pipe, uint8_t address){//OK
	uint8_t tx[] = {CMD_W_REGISTER | pipe + 10, address}; 	//RX_ADDR_P2->2+10
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_set_tx_addr(uint8_t* address){//OK
	uint8_t tx[6];
	uint8_t* rx;
	uint8_t i = 0;
	tx[0] = CMD_W_REGISTER | B0_TX_ADDR;

	for(i=0;i<5;i++){
		tx[i+1] = address[i];
	}
	spi_transaction(&tx, &rx, 6);
}

void rfm_set_rx_size(uint8_t pipe, uint8_t size){//OK
	uint8_t tx[] = {CMD_W_REGISTER | pipe + 17, size};	//B0_RX_PW_P0->0+17
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_set_dyn_payload(uint8_t pipe, uint8_t on){//OK
	uint8_t reg = rfm_read_register(B0_DYNPD);
	reg = (on) ? (reg | (1 << pipe)) : (reg & ~(1 << pipe));
	uint8_t tx[] = {CMD_W_REGISTER | B0_DYNPD, reg};
	uint8_t* rx;
	spi_transaction(&tx, &rx, 2);
}

void rfm_flush_tx(){//OK
	uint8_t* tx = CMD_FLUSH_TX;
	uint8_t* rx;
	spi_transaction(&tx, &rx, 1);
}

void rfm_flush_rx(){//OK
	uint8_t* tx = CMD_FLUSH_RX;
	uint8_t* rx;
	spi_transaction(&tx, &rx, 1);
}

void rfm_reuse_tx(){//OK
	uint8_t* tx = CMD_REUSE_TX_PL;
	uint8_t* rx;
	spi_transaction(&tx, &rx, 1);
}

void rfm_read_id(uint8_t* rx){//OK
	uint8_t* tx = CMD_R_REGISTER | B1_CHIP_ID;
	spi_bank_1();
	spi_transaction(&tx, rx, 5);
	spi_bank_0();
}

uint8_t rfm_read_rbank(){//OK
	uint8_t* tx = CMD_R_REGISTER | B0_STATUS;
	uint8_t rx[2];
	spi_transaction(&tx, &rx, 2);
	return rx[1] & 0b10000000;
}

uint8_t rfm_pipe_avlb(){//OK
	uint8_t* tx = CMD_R_REGISTER | B0_STATUS;
	uint8_t rx[2];
	spi_transaction(&tx, &rx, 2);
	return rx[1] & 0b00001110;
}

uint8_t rfm_tx_fifo_full(){//OK
	uint8_t* tx = CMD_R_REGISTER | B0_STATUS;
	uint8_t rx[2];
	spi_transaction(&tx, &rx, 2);
	return rx[1] & 0b00000001;
}

//**********************//


//***PRIVATE FUNCIONS***//
static void spi_transaction(uint8_t* tx, uint8_t* rx, uint8_t size){
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi_rfm, tx, rx, size, 100);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
}

static void spi_bank_0(){
	uint8_t* tx = CMD_R_REGISTER | B0_STATUS;
	uint8_t rx[2];
	spi_transaction(&tx, &rx, 2);

	if(rx[1] & 0b10000000){
		uint8_t txn[] = {CMD_ACTIVATE, 0x53};
		spi_transaction(txn, rx, 2);
	}
}

static void spi_bank_1(){
	uint8_t* tx = CMD_R_REGISTER | B0_STATUS;
	uint8_t rx[2];
	spi_transaction(&tx, &rx, 2);

	if(!(rx[1] & 0b10000000)){
		uint8_t txn[] = {CMD_ACTIVATE, 0x53};
		spi_transaction(txn, rx, 2);
	}
}
//**********************//




























