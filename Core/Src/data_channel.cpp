/*
 * data_channel.cpp
 *
 *  Created on: Jul 10, 2020
 *      Author: mark
 */

#include "data_channel.h"
#include "data_channel_c.h"

DataChannel::DataChannel() :
    simulation_discrete_time{0},
    data0{*reinterpret_cast<Logical_t*>(raw_data0)},
    data1{*reinterpret_cast<Logical_t*>(raw_data1)}
{
    for (uint32_t idx = 0; idx < sizeof(Logical_t); ++idx)
    {
        raw_data0[idx] = 0;
        raw_data1[idx] = 0;
    }

    const uint8_t cmd[] = "OUT#";
    for (uint32_t idx = 0; idx < 4; ++idx)
    {
        data1.left_motor.output.command[idx] = cmd[idx];
        data1.right_motor.output.command[idx] = cmd[idx];
    }

    // find a better location for this
    data1.left_motor.output.model_index = 0;
    data1.left_motor.output.output_index = 0;
    data1.right_motor.output.model_index = 1;
    data1.right_motor.output.output_index = 0;
}

void DataChannel::update (void)
{
    for (uint32_t idx = 0; idx < sizeof(Logical_t); ++idx)
    {
        raw_data0[idx] = raw_data1[idx];
    }
}

// C binding

DataChannel& get_data_channel (void)
{
    static DataChannel data_channel;

    return data_channel;
}

extern "C" uint8_t* get_left_motor_output (void)
{
    return reinterpret_cast<uint8_t*>(&get_data_channel().get_data_instant_k_plus_one().left_motor.output);
}

extern "C" uint32_t get_left_motor_output_size (void)
{
    return sizeof(DCMotor_io_t::output_t);
}

extern "C" uint8_t* get_right_motor_output (void)
{
    return reinterpret_cast<uint8_t*>(&get_data_channel().get_data_instant_k_plus_one().right_motor.output);
}

extern "C" uint32_t get_right_motor_output_size (void)
{
    return sizeof(DCMotor_io_t::output_t);
}
