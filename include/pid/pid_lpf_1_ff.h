/**
 * @file pid_lpf_1_ff.h
 * @brief PID controller with first-order low-pass filtered derivative and feed-forward.
 *
 * @details
 * This file defines the @ref control_system::pid::PID_LPF_1_FF class,
 * which implements a discrete-time proportional-integral-derivative
 * (PID) controller with a first-order low-pass filtered derivative
 * contribution and feed-forward control.
 *
 * The controller uses configurable proportional, integral, and
 * derivative gains together with a derivative filter cutoff frequency.
 * A feed-forward contribution can be added to the PID controller output.
 *
 * Previous controller outputs and control errors are maintained as
 * internal state variables.
 *
 * The controller output is constrained by a configurable maximum
 * output limit.
 *
 * @tparam T Numeric data type used for the controller calculations.
 *
 * @author Abhinay Kumar
 * @version 1.0.0
 * @date 2026-08-11
 *
 * @copyright
 * Copyright (c) 2026 Abhinay Kumar
 */

#ifndef CONTROL_SYSTEM_PID_PID_LPF_1_FF_H
#define CONTROL_SYSTEM_PID_PID_LPF_1_FF_H

#include <cstdint>
#include "../utility/utility.h"

namespace control_system
{
    /**
     * @namespace control_system
     * @brief Contains control-system algorithms and controllers.
     */

    namespace pid
    {
        /**
         * @namespace pid
         * @brief Contains proportional-integral-derivative controller implementations.
         */

        /**
         * @class PID_LPF_1_FF
         * @brief PID controller with first-order low-pass filtered derivative and feed-forward.
         *
         * @details
         * The PID_LPF_1_FF class implements a discrete-time PID controller
         * with a first-order low-pass filter applied to the derivative
         * contribution.
         *
         * The controller also supports a feed-forward contribution that
         * can be supplied to the controller output.
         *
         * The controller is configured using the sampling time,
         * proportional gain, integral gain, derivative gain, derivative
         * filter cutoff frequency, previous controller outputs, and
         * maximum output limit.
         *
         * @tparam T Numeric data type used for the controller.
         *
         * @par Example
         * @code{.cpp}
         * control_system::pid::PID_LPF_1_FF<double> controller;
         *
         * controller.init(
         *     0.001,
         *     Kp,
         *     Ki,
         *     Kd,
         *     10.0,
         *     0.0,
         *     0.0,
         *     100.0
         * );
         *
         * double output = controller.update(reference, measurement, feed_forward);
         * @endcode
         */
        template <typename T>
        class PID_LPF_1_FF
        {
        public:
            /**
             * @brief Constructs a PID controller with filtered derivative
             * and feed-forward support.
             *
             * @details
             * Constructs the controller with its parameters and internal
             * state initialized to their default values.
             */
            PID_LPF_1_FF();

            /**
             * @brief Initializes the PID controller.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] Ki_ Integral gain.
             * @param[in] Kd_ Derivative gain.
             * @param[in] fc_ Derivative filter cutoff frequency in Hz.
             * @param[in] u_k_1_ Previous controller output.
             * @param[in] u_k_2_ Controller output from two iterations ago.
             * @param[in] u_max_ Maximum allowed controller output.
             *
             * @details
             * Initializes the PID controller parameters, derivative filter
             * parameters, previous controller outputs, and output limit.
             */
            void init(T dt_, T Kp_, T Ki_, T Kd_, T fc_, T u_k_1_, T u_k_2_, T u_max_);

            /**
             * @brief Sets the PID controller parameters.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] Ki_ Integral gain.
             * @param[in] Kd_ Derivative gain.
             * @param[in] fc_ Derivative filter cutoff frequency in Hz.
             * @param[in] u_k_1_ Previous controller output.
             * @param[in] u_k_2_ Controller output from two iterations ago.
             * @param[in] u_max_ Maximum allowed controller output.
             *
             * @details
             * Updates the PID gains, sampling time, derivative filter
             * cutoff frequency, previous controller outputs, and output
             * limit.
             */
            void set_param(T dt_, T Kp_, T Ki_, T Kd_, T fc_, T u_k_1_, T u_k_2_, T u_max_);

            /**
             * @brief Computes the PID control output with feed-forward.
             *
             * @param[in] x_0 Reference or desired value.
             * @param[in] x Current or measured value.
             * @param[in] u_ff_ Feed-forward control input.
             *
             * @return PID controller output including the feed-forward
             * contribution and limited by the configured maximum output.
             *
             * @details
             * Calculates the proportional, integral, and derivative
             * contributions. The derivative contribution is filtered
             * using the configured first-order low-pass filter, and the
             * feed-forward contribution is incorporated into the final
             * controller output.
             */
            T update(T x_0, T x, T u_ff_ = 0.0);

            /**
             * @brief Resets the PID controller state.
             *
             * @details
             * Resets the stored controller outputs, control errors,
             * feed-forward value, and startup state.
             */
            void reset();

            /**
             * @brief Merges an external controller output into the PID state.
             *
             * @param[in] u_k_1_ Previous or externally provided controller
             * output.
             *
             * @details
             * Updates the internal previous-output state using the
             * specified controller output.
             */
            void merge(T u_k_1_);

            /**
             * @brief Sets the controller sampling time.
             *
             * @param[in] dt_ Sampling time in seconds.
             */
            void set_dt(T dt_);

            /**
             * @brief Sets the proportional gain.
             *
             * @param[in] Kp_ Proportional gain.
             */
            void set_Kp(T Kp_);

            /**
             * @brief Sets the integral gain.
             *
             * @param[in] Ki_ Integral gain.
             */
            void set_Ki(T Ki_);

            /**
             * @brief Sets the derivative gain.
             *
             * @param[in] Kd_ Derivative gain.
             */
            void set_Kd(T Kd_);

            /**
             * @brief Sets the derivative filter cutoff frequency.
             *
             * @param[in] fc_ Cutoff frequency in Hz.
             */
            void set_fc(T fc_);

            /**
             * @brief Sets the feed-forward control input.
             *
             * @param[in] u_ff_ Feed-forward control input.
             */
            void set_ff(T u_ff_);

            /**
             * @brief Sets the previous controller output.
             *
             * @param[in] u_k_1_ Previous controller output.
             */
            void set_u_0(T u_k_1_);

            /**
             * @brief Sets the maximum controller output.
             *
             * @param[in] u_max_ Maximum allowed controller output.
             */
            void set_u_max(T u_max_);

            /**
             * @brief Gets the controller sampling time.
             *
             * @return Sampling time in seconds.
             */
            T get_dt();

            /**
             * @brief Gets the proportional gain.
             *
             * @return Proportional gain.
             */
            T get_Kp();

            /**
             * @brief Gets the integral gain.
             *
             * @return Integral gain.
             */
            T get_Ki();

            /**
             * @brief Gets the derivative gain.
             *
             * @return Derivative gain.
             */
            T get_Kd();

            /**
             * @brief Gets the derivative filter cutoff frequency.
             *
             * @return Cutoff frequency in Hz.
             */
            T get_fc();

            /**
             * @brief Gets the feed-forward control input.
             *
             * @return Feed-forward control input.
             */
            T get_ff();

            /**
             * @brief Gets the previous controller output.
             *
             * @return Previous controller output.
             */
            T get_u_k_1();

            /**
             * @brief Gets the previous control error.
             *
             * @return Control error from the previous controller iteration.
             */
            T get_e_k_1();

            /**
             * @brief Gets the control error from two iterations ago.
             *
             * @return Control error from two controller iterations ago.
             */
            T get_e_k_2();

            /**
             * @brief Gets the maximum controller output.
             *
             * @return Maximum allowed controller output.
             */
            T get_u_max();

        private:
            /**
             * @brief Controller sampling time.
             *
             * @details
             * Sampling period of the discrete-time PID controller
             * in seconds.
             */
            T dt = 0.0;

            /**
             * @brief Previous controller output.
             *
             * @details
             * Stores the controller output from the previous iteration.
             */
            T u_k_1 = 0.0;

            /**
             * @brief Controller output from two iterations ago.
             */
            T u_k_2 = 0.0;

            /**
             * @brief Previous control error.
             *
             * @details
             * Stores the control error from the previous iteration.
             */
            T e_k_1 = 0.0;

            /**
             * @brief Control error from two iterations ago.
             */
            T e_k_2 = 0.0;

            /**
             * @brief Proportional gain.
             */
            T Kp = 0.0;

            /**
             * @brief Integral gain.
             */
            T Ki = 0.0;

            /**
             * @brief Derivative gain.
             */
            T Kd = 0.0;

            /**
             * @brief Time constant of the derivative low-pass filter.
             *
             * @details
             * Time constant associated with the first-order derivative
             * filter.
             */
            T tau = 0.0;

            /**
             * @brief Feed-forward control input.
             */
            T u_ff = 0.0;

            /**
             * @brief Maximum controller output.
             *
             * @details
             * Defines the maximum allowed magnitude of the controller
             * output.
             */
            T u_max = 9999999999;

            /**
             * @brief Controller startup state.
             *
             * @details
             * Indicates the startup state of the discrete-time PID
             * controller.
             */
            uint8_t start = 0;
        };

#include "pid_lpf_1_ff.tpp"
    }
}

#endif