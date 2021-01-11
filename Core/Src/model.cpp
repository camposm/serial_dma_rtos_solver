/*
 * model.cpp
 *
 *  Created on: Jun 27, 2020
 *      Author: mark
 */

#include <stdint.h>
#include <cmath>
#include "model.h"
#include "data_channel.h"


namespace DC_Motor
{

constexpr float     time_scale            = 1000.0f;
constexpr float     J_mod                = (3.2284e-6f * time_scale * time_scale);
constexpr float     b_mod                = (3.5077e-6f * time_scale);
constexpr float     K_mod                = (0.0274f * time_scale);
constexpr float     R_mod                = (4.0f * time_scale);
constexpr float     L_mod                = (2.75e-6f * time_scale * time_scale);

constexpr float     integration_factor     = (1.0f / time_scale);
constexpr float     rads_to_rpm         = 9.549f;
constexpr float     C_one                = (time_scale * rads_to_rpm);

constexpr float     minus_b_over_J         = (-b_mod / J_mod);
constexpr float     K_over_J            = (K_mod / J_mod);
constexpr float     minus_K_over_L        = (-K_mod / L_mod);
constexpr float     minus_R_over_L         = (-R_mod / L_mod);
constexpr float     one_over_L             = (1.0f / L_mod);

class StateSpace
{
public:
    explicit StateSpace (float& input0) :
        state{0.0f},
        input{input0},
        output{0.0f}
    {
        // blank
    }

    constexpr static float A[9] = {
            0.0f,     1.0f,                0.0f,
            0.0f,    minus_b_over_J,        K_over_J,
            0.0f,    minus_K_over_L,        minus_R_over_L
    };

    constexpr static float B[3] = {
            0.0f,    0.0f,    one_over_L
    };

    constexpr static float C[3] = {
            0.0f,    C_one,    0.0f
    };

    constexpr static float m_integration_factor = integration_factor;

    float   state[3];
    float&  input;
    float   output;
};

} // ns DC_Motor

namespace Solver {

template <typename T>
class Solver3
{
public:
    Solver3 (float& input0) : model(input0)
    {
        // blank
    }

    void step (void)
    {
        // x_dot = A * x + B * u
        float xdot0 = model.state[1];
        float xdot1 = (model.state[1] * T::A[4]) + (model.state[2] * T::A[5]);
        float xdot2 = (model.state[1] * T::A[7]) + (model.state[2] * T::A[8]) + (model.input * T::B[2]);

        // integrate
        model.state[0] += T::m_integration_factor * xdot0;
        model.state[1] += T::m_integration_factor * xdot1;
        model.state[2] += T::m_integration_factor * xdot2;

        // compute output
        model.output = T::C[1] * model.state[1];

    }

    T    model;
};

} // ns Solver

namespace {

DataChannel&    dc{get_data_channel()};

Solver::Solver3<DC_Motor::StateSpace>    left_motor(dc.get_data_instant_k().left_motor.setpoint_input);
Solver::Solver3<DC_Motor::StateSpace>    right_motor(dc.get_data_instant_k().right_motor.setpoint_input);

auto& output_left{dc.get_data_instant_k_plus_one().left_motor.output};
auto& output_right{dc.get_data_instant_k_plus_one().right_motor.output};

}

extern "C" void update_model_setpoint (uint8_t model_index, uint8_t input_index, float value)
{
    if (std::isfinite(value) && (value > -12.1) && (value < 12.1))
    {
        if (model_index == 0)
        {
            dc.get_data_instant_k_plus_one().left_motor.setpoint_input = value;
        }
        else
        {
            dc.get_data_instant_k_plus_one().right_motor.setpoint_input = value;
        }
    }
}

extern "C" void update_setpoint (float left_m, float right_m)
{
    dc.get_data_instant_k_plus_one().left_motor.setpoint_input = left_m;
    dc.get_data_instant_k_plus_one().right_motor.setpoint_input = right_m;
}

extern "C" void get_outputs (float* left_o, float* right_o)
{
    *left_o = left_motor.model.output;
    *right_o = right_motor.model.output;
}

extern "C" void solve_10ms (uint32_t sim_time)
{
    for (uint32_t idx = 0; idx < 10; ++idx)
    {
        for (uint32_t step_cnt = 0; step_cnt < 1000; ++step_cnt)
        {
            left_motor.step();
            right_motor.step();
        }

        output_left.last_10ms_output[idx] = left_motor.model.output;
        output_right.last_10ms_output[idx] = right_motor.model.output;
        dc.update();
    }

    output_left.packet_time = sim_time;
    output_right.packet_time = sim_time;
}

