/*
 * serial_handler.c
 *
 *  Created on: Jul 7, 2020
 *      Author: mark
 */

#include "serial_handler.h"
#include "main.h"
#include "m_mutex.h"
#include "model.h"

//
// TYPES
//
typedef enum
{
    EMPTY_TRANSMISSION_SLOT = 0,
    WAITING_FOR_TRANSMISSION_START = 1,
    ONGOING_TRANSMISSION = 2
} tx_dma_status_t;

typedef struct
{
    uint16_t            start_position;
    uint16_t            length;
    uint32_t            index;
    tx_dma_status_t     status;
} tx_dma_request_t;

//
// The buffers
//
uint8_t serial_rx_buffer[SERIAL_PORT_RX_BUFFER_SIZE] = {0};
static uint8_t serial_tx_buffer[SERIAL_PORT_TX_BUFFER_SIZE] = {0};

//
// Control variables
//
static volatile uint16_t            tx_write_ptr = 0;   // first available position to write
static volatile uint16_t            available_space = SERIAL_PORT_TX_BUFFER_SIZE;
static volatile uint8_t             is_transmitting = 0;
static volatile tx_dma_request_t    transmit_requests[MAX_TRANSMIT_REQUESTS] = {0};
static volatile uint16_t            request_position = 0;

static mutex_t ctrl_mtx = 0;

void start_tx_dma_transfer (void);

#define MIN_COMMAND_SIZE    12

typedef struct m_command
{
    uint8_t     start_marker;
    uint8_t     command;
    uint8_t     padding;
    uint8_t     xor_k;
    union m_payload
    {
        struct  m_setpoint
        {
            uint8_t     model_index;
            uint8_t     input_index;
            uint8_t     padding[2];
            float       value;
            //
        } setpoint;
        uint8_t     payload[8];
    } payload;
    //
} Command_t;

typedef union raw_command
{
    Command_t   cmd;
    uint8_t     data[sizeof(Command_t)];
} RawCommand_t;

void handle_rx_data (void)
{
    static uint16_t last_position = 0;
    uint16_t current_position = SERIAL_PORT_RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
    RawCommand_t    cmd;

    uint16_t received = 0;
    if (current_position >= last_position)
    {
        received = current_position - last_position;
    }
    else
    {
        received = SERIAL_PORT_RX_BUFFER_SIZE - last_position + current_position;
    }

    if (received > 0)
    {
        for (uint16_t counter = 0; counter < received; )
        {
            if (serial_rx_buffer[last_position] == '@' && (received - counter) >= MIN_COMMAND_SIZE)
            {
                uint8_t xor_exp = 0xCA;
                for (uint16_t idx = 0; idx < MIN_COMMAND_SIZE; ++idx)
                {
                    cmd.data[idx] = serial_rx_buffer[last_position + idx];
                    if (idx != 3)
                    {
                        xor_exp ^= serial_rx_buffer[last_position + idx];
                    }
                }
                if (xor_exp == cmd.cmd.xor_k)
                {
                    update_model_setpoint(
                        cmd.cmd.payload.setpoint.model_index,
                        cmd.cmd.payload.setpoint.input_index,
                        cmd.cmd.payload.setpoint.value
                    );
                    LL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                }
                last_position = (last_position + MIN_COMMAND_SIZE) % SERIAL_PORT_RX_BUFFER_SIZE;
                counter += MIN_COMMAND_SIZE;
            }
            else
            {
                last_position = (last_position + 1) % SERIAL_PORT_RX_BUFFER_SIZE;
                counter += 1;
            }
        }
    }
}

static uint32_t dbg_count = 0;

uint16_t write_to_tx_buffer (const uint8_t* data, uint16_t data_size)
{
    static uint32_t position_on_queue = 0;

    uint16_t written = 0;

    lock_mutex(&ctrl_mtx);

    //uint32_t old_primask = __get_PRIMASK();
    //__disable_irq();
    if (available_space == 0 || transmit_requests[request_position].status != EMPTY_TRANSMISSION_SLOT)
    {
        unlock_mutex(&ctrl_mtx);
        //__set_PRIMASK(old_primask);
        return 0;
    }

    if (dbg_count == 46)
    {
        asm("nop");
    }

    if (data_size <= available_space)
    {
        uint16_t old_start_tx = tx_write_ptr;

        for (uint16_t idx = 0; idx < data_size; ++idx)
        {
            serial_tx_buffer[tx_write_ptr] = data[idx];
            tx_write_ptr = (tx_write_ptr + 1u) % SERIAL_PORT_TX_BUFFER_SIZE;
        }

        available_space -= data_size;
        written = data_size;

        if ((old_start_tx + data_size) > SERIAL_PORT_TX_BUFFER_SIZE)
        {
            // 1st part
            uint16_t transmitted_1st_part = SERIAL_PORT_TX_BUFFER_SIZE - old_start_tx;
            transmit_requests[request_position].start_position = old_start_tx;
            transmit_requests[request_position].status = WAITING_FOR_TRANSMISSION_START;
            transmit_requests[request_position].index = position_on_queue;
            transmit_requests[request_position].length = transmitted_1st_part;

            request_position = (request_position + 1u) % MAX_TRANSMIT_REQUESTS;
            position_on_queue += 1;

            if (transmit_requests[request_position].status == EMPTY_TRANSMISSION_SLOT)
            {
                transmit_requests[request_position].start_position = 0;
                transmit_requests[request_position].status = WAITING_FOR_TRANSMISSION_START;
                transmit_requests[request_position].index = position_on_queue;
                transmit_requests[request_position].length = data_size - transmitted_1st_part;

                request_position = (request_position + 1u) % MAX_TRANSMIT_REQUESTS;
                position_on_queue += 1;
            }
            else
            {
                // this is a huge fuck up.. data was lost
                unlock_mutex(&ctrl_mtx);
                //__set_PRIMASK(old_primask);
                return 0;
            }
        }
        else
        {
            transmit_requests[request_position].start_position = old_start_tx;
            transmit_requests[request_position].status = WAITING_FOR_TRANSMISSION_START;
            transmit_requests[request_position].index = position_on_queue;
            transmit_requests[request_position].length = data_size;
            request_position = (request_position + 1u) % MAX_TRANSMIT_REQUESTS;
            position_on_queue += 1;
        }
    }
    else
    {
        return 0;
    }

    if (!is_transmitting)
    {
        unlock_mutex(&ctrl_mtx);
        //__set_PRIMASK(old_primask);
        start_tx_dma_transfer();
    }
    else
    {
        unlock_mutex(&ctrl_mtx);
        //__set_PRIMASK(old_primask);
    }

    dbg_count += 1;

    return written;
}

void start_tx_dma_transfer (void)
{
    static uint32_t next_to_transmit = 0;
    //uint32_t old_primask;

    // Check if DMA is active
    // Must be set to 0
    // TODO: why must we disable interrupts to start a dma request?
    //old_primask = __get_PRIMASK();
    //__disable_irq();

    // Disable channel if enabled
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

    // Clear all flags
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_ClearFlag_HT2(DMA1);
    LL_DMA_ClearFlag_GI2(DMA1);
    LL_DMA_ClearFlag_TE2(DMA1);

    lock_mutex(&ctrl_mtx);
    for (uint8_t idx = 0; idx < MAX_TRANSMIT_REQUESTS; ++idx)
    {
        if ((transmit_requests[idx].index == next_to_transmit) &&
            (transmit_requests[idx].status == WAITING_FOR_TRANSMISSION_START))
        {
            // Start DMA transfer
            LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, transmit_requests[idx].length);
            LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t) &serial_tx_buffer[transmit_requests[idx].start_position]);

            transmit_requests[idx].status = ONGOING_TRANSMISSION;
            is_transmitting = 1;
            next_to_transmit += 1;

            // Start new transfer
            LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

            break;
        }
    }
    unlock_mutex(&ctrl_mtx);

    //__set_PRIMASK(old_primask);
}

void process_tx_complete (void)
{
    lock_mutex(&ctrl_mtx);

    is_transmitting = 0;
    for (uint8_t idx = 0; idx < MAX_TRANSMIT_REQUESTS; ++idx)
    {
        if (transmit_requests[idx].status == ONGOING_TRANSMISSION)
        {
            available_space += transmit_requests[idx].length;
            transmit_requests[idx].status = EMPTY_TRANSMISSION_SLOT;
            //transmit_requests[idx].length = 0;
            //transmit_requests[idx].start_position = 0;
            break;
        }
    }
    // trigger another transfer, if needed
    unlock_mutex(&ctrl_mtx);
    start_tx_dma_transfer();
}
