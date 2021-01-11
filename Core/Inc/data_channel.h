/*
 * data_channel.h
 *
 *  Created on: Jul 10, 2020
 *      Author: mark
 */

#include <stdint.h>
#include <model.h>

#ifndef SRC_DATA_CHANNEL_H_
#define SRC_DATA_CHANNEL_H_

class DataChannel;

DataChannel& get_data_channel (void);

class DataChannel
{
public:
    typedef struct
    {
        DCMotor_io_t    left_motor;
        DCMotor_io_t    right_motor;
    } Logical_t;

    Logical_t&  get_data_instant_k (void)
    {
        return data0;
    }

    Logical_t&  get_data_instant_k_plus_one (void)
    {
        return data1;
    }

    void update (void);

private:
    DataChannel (void);

    // simulation progresses in steps of 10ms - each unit here is 10ms
    uint32_t        simulation_discrete_time;
    uint8_t         raw_data0[sizeof(Logical_t)];    // data t_k
    uint8_t         raw_data1[sizeof(Logical_t)];    // data t_(k+1)

    Logical_t&      data0;
    Logical_t&      data1;

    friend DataChannel& get_data_channel (void);
};

#endif /* SRC_DATA_CHANNEL_H_ */
