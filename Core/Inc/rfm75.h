  /*
  ******************************************************************************
  *
  *		File:	rfm75.h
  *		Name:	STM32 library for GFSK transceiver - RFM75-S
  *	  Author:	crlhz
  *	  	Date: 	2023-04-08
  *
  *	  github:	https://github.com/crlhz
  *	 contact:	crlhz@proton.me
  *
  ******************************************************************************
  */

#ifndef INC_RFM75_H_
#define INC_RFM75_H_

#include <stdint.h>
#include "spi.h"
#include "gpio.h"

//***SPI COMMANDS***//
#define CMD_R_REGISTER			0b00000000
#define CMD_W_REGISTER			0b00100000
#define CMD_R_RX_PAYLOAD		0b01100001
#define CMD_W_TX_PAYLOAD		0b10100001
#define CMD_FLUSH_TX			0b11100001
#define CMD_FLUSH_RX			0b11100010
#define CMD_REUSE_TX_PL			0b11100011
#define CMD_ACTIVATE			0b01010000
#define CMD_R_RX_PL_WID			0b01100000
#define CMD_W_ACK_PAYLOAD		0b10101000
#define CMD_W_TX_PAYLOAD_NOACK	0b10110000
#define CMD_NOP					0b11111111
//******************//


//***REGISTER BANK 0***//
#define B0_CONFIG		0x00
#define B0_EN_AA		0x01
#define B0_EN_RXADDR	0x02
#define B0_SETUP_AW		0x03
#define B0_SETUP_RETR	0x04
#define B0_RF_CH		0x05
#define B0_RF_SETUP		0x06
#define B0_STATUS		0x07
#define B0_OBSERVE_TX	0x08
#define B0_CD			0x09
#define B0_RX_ADDR_P0	0x0A
#define B0_RX_ADDR_P1	0x0B
#define B0_RX_ADDR_P2	0x0C
#define B0_RX_ADDR_P3	0x0D
#define B0_RX_ADDR_P4	0x0E
#define B0_RX_ADDR_P5	0x0F
#define B0_TX_ADDR		0x10
#define B0_RX_PW_P0		0x11
#define B0_RX_PW_P1		0x12
#define B0_RX_PW_P2		0x13
#define B0_RX_PW_P3		0x14
#define B0_RX_PW_P4		0x15
#define B0_RX_PW_P5		0x16
#define B0_FIFO_STATUS	0x17
#define B0_DYNPD		0x1C
#define B0_FEATURE		0x1D
//********************//


//***REGISTER BANK 1***//
#define B1_00 			0x00
#define B1_01			0x01
#define B1_02			0x02
#define B1_03			0x03
#define B1_04			0x04
#define B1_RSSI_EN		0x05
#define B1_CHIP_ID		0x08
#define B1_0C			0x0C
#define B1_NEW_FEATURE	0x0D
#define B1_RAMP			0x0E
//********************//


//module initialization
void rfm_init(SPI_HandleTypeDef *hspi);

//transmit data
void rfm_transmit(uint8_t* data, uint8_t size);

//receive data
uint8_t* rfm_receive(uint8_t size);

//mask interrupt caused by RX_DR
//0->interrupt not masked, 1->interrupt masked
void rfm_mask_rx(uint8_t on);

//mask interrupt caused by TX_DC
//0->interrupt not masked, 1->interrupt masked
void rfm_mask_tx(uint8_t on);

//mask interrupt caused by MAX_RT
//0->interrupt not masked, 1->interrupt masked
void rfm_mask_max(uint8_t on);

//enable crc
//0->disabled, 1->enabled
void rfm_enable_crc(uint8_t on);

//crc encoding scheme
//available values: 1 / 2 (bytes)
void rfm_crc_scheme(uint8_t n);

//rfm module power on
void rfm_power_on();

//rfm module power off
void rfm_power_off();

//RX/TX control
//0->PTX, 1-> PRX
void rfm_mode(uint8_t mode);

//enable "auto acknowledgement" function in specific pipe
//available values: 0 - 5 (pipe)
void rfm_auto_aa(uint8_t pipe);

//enabled rx address
//available values: 0 - 5 (pipe)
void rfm_enable_pipe(uint8_t pipe);

//set RX/TX address field width
//available values: 3 - 5 (bytes)
void rfm_address_width(uint8_t width);

//set auto retransmission delay
//available values: 250 - 4000 (us)
void rfm_ret_delay(uint16_t delay);

//set auto retransmission count
//available values: 0 - 15 (number of retransmission)
void rfm_ret_count(uint8_t n);

//set frequency channel
//available values: ?
void rfm_freq_channel(uint16_t n);

//set air data rate
//available values: 250, 1000, 2000 (Mbps)
void rfm_air_dr(uint16_t rate);

//force pll lock signal
//0->unlocked, 1->locked
void rfm_pll_lock(uint8_t on);

//set rf output power in TX mode
//available values: ?
void rfm_tx_pwr(uint8_t pwr);

//set lna gain
//0->low gain, 1->high gain
void rfm_lna_gain(uint8_t mode);




//read bank selection state
uint8_t rfm_read_rbank();




#endif /* INC_RFM75_H_ */
