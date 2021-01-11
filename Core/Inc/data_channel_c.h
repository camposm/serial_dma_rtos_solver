/*
 * data_channel_c.h
 *
 *  Created on: Jul 10, 2020
 *      Author: mark
 */

#include <stdint.h>

#ifndef INC_DATA_CHANNEL_C_H_
#define INC_DATA_CHANNEL_C_H_

#ifdef __cplusplus
extern "C" {
#endif

uint8_t*    get_left_motor_output (void);
uint32_t    get_left_motor_output_size (void);

uint8_t*    get_right_motor_output (void);
uint32_t    get_right_motor_output_size (void);

#ifdef __cplusplus
}
#endif

#endif /* INC_DATA_CHANNEL_C_H_ */
