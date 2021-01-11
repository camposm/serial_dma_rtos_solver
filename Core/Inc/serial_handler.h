/*
 * serial_handler.h
 *
 *  Created on: Jul 7, 2020
 *      Author: mark
 */

#ifndef INC_SERIAL_HANDLER_H_
#define INC_SERIAL_HANDLER_H_

#include <stdint.h>

#define SERIAL_PORT_RX_BUFFER_SIZE          128u
#define SERIAL_PORT_TX_BUFFER_SIZE          1024u
#define MAX_TRANSMIT_REQUESTS               16u

extern uint8_t serial_rx_buffer[SERIAL_PORT_RX_BUFFER_SIZE];

uint16_t write_to_tx_buffer (const uint8_t* data, uint16_t data_size);

void handle_rx_data (void);
void process_tx_complete (void);

#endif /* INC_SERIAL_HANDLER_H_ */
