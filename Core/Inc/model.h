/*
 * model.h
 *
 *  Created on: Jul 9, 2020
 *      Author: mark
 */

#include <stdint.h>

#ifndef INC_MODEL_H_
#define INC_MODEL_H_

#ifdef __cplusplus

typedef struct
{
    float setpoint_input;
    float disturbance_input;

    struct output_t
    {
        uint8_t command[4];
        uint8_t model_index;
        uint8_t output_index;
        uint8_t padding[2];
        uint32_t packet_time;
        float last_10ms_output[10];
    } output;

} DCMotor_io_t;

extern "C" {
#endif

void update_model_setpoint (uint8_t model_index, uint8_t input_index, float value);
void solve_10ms (uint32_t sim_time);
void update_setpoint (float left_m, float right_m);
void get_outputs (float* left_o, float* right_o);

#ifdef __cplusplus
}
#endif


#endif /* INC_MODEL_H_ */
