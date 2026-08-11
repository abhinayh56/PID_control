/**
 * @file pd_lpf_1.h
 * @brief Proportional-Derivative (PD) controller with first-order low-pass filtering.
 *
 * @details
 * This file defines the @ref control_system::pd::PD_LPF_1 class, which
 * implements a discrete-time proportional-derivative (PD) controller
 * with a first-order low-pass filter associated with the derivative
 * action.
 *
 * The controller uses configurable proportional and derivative gains,
 * sampling time, derivative filter cutoff frequency, and maximum output
 * limit.
 *
 * The previous control error and controller output are maintained as
 * internal state variables.
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

#ifndef CONTROL_SYSTEM_PD_PD_LPF_1_H
#define CONTROL_SYSTEM_PD_PD_LPF_1_H


#include "../utility/utility.h"


namespace control_system
{
    /**
     * @namespace control_system
     * @brief Contains control-system algorithms and controllers.
     */

    namespace pd
    {
        /**
         * @namespace pd
         * @brief Contains proportional-derivative controller implementations.
         */

        /**
         * @class PD_LPF_1
         * @brief PD controller with first-order low-pass filtered derivative.
         *
         * @details
         * The PD_LPF_1 class implements a discrete-time proportional-
         * derivative controller with a first-order low-pass filter
         * associated with the derivative action.
         *
         * The controller uses the sampling time, proportional gain,
         * derivative gain, and derivative filter cutoff frequency to
         * calculate the control output.
         *
         * The controller maintains the previous control error and
         * controller output as internal state and applies a configurable
         * maximum output limit.
         *
         * @tparam T Numeric data type used for the controller.
         *
         * @par Example
         * @code{.cpp}
         * control_system::pd::PD_LPF_1<double> controller;
         *
         * controller.init(
         *     0.001,
         *     Kp,
         *     Kd,
         *     10.0,
         *     100.0
         * );
         *
         * double output = controller.update(reference, measurement);
         * @endcode
         */
        template <typename T>
        class PD_LPF_1
        {
        public:

            /**
             * @brief Constructs a PD controller with filtered derivative.
             *
             * @details
             * Constructs the controller with its parameters and internal
             * state initialized to their default values.
             */
            PD_LPF_1();

            /**
             * @brief Initializes the PD controller.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] Kd_ Derivative gain.
             * @param[in] fc_ Derivative filter cutoff frequency in Hz.
             * @param[in] u_max_ Maximum allowed controller output.
             *
             * @details
             * Initializes the controller sampling time, proportional and
             * derivative gains, derivative filter cutoff frequency, and
             * output limit.
             */
            void init(T dt_, T Kp_, T Kd_, T fc_, T u_max_);

            /**
             * @brief Sets the PD controller parameters.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] Kd_ Derivative gain.
             * @param[in] fc_ Derivative filter cutoff frequency in Hz.
             * @param[in] u_max_ Maximum allowed controller output.
             *
             * @details
             * Updates the controller sampling time, proportional and
             * derivative gains, derivative filter cutoff frequency, and
             * output limit.
             */
            void set_param(T dt_, T Kp_, T Kd_, T fc_, T u_max_);

            /**
             * @brief Computes the PD control output.
             *
             * @param[in] x_0 Reference or desired value.
             * @param[in] x Current or measured value.
             *
             * @return PD controller output limited by the configured
             * maximum output.
             *
             * @details
             * Calculates the proportional and derivative contributions
             * using the current and previous control errors. The
             * derivative action is associated with a first-order
             * low-pass filter.
             */
            T update(T x_0, T x);

            /**
             * @brief Resets the PD controller state.
             *
             * @details
             * Resets the stored control error, previous controller
             * output, and startup state.
             */
            void reset();

            /**
             * @brief Merges an external controller output into the PD state.
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
             * @brief Gets the previous control error.
             *
             * @return Control error from the previous controller iteration.
             */
            T get_e_k_1();

            /**
             * @brief Gets the previous controller output.
             *
             * @return Previous controller output.
             */
            T get_u_k_1();

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
             * Sampling period of the discrete-time PD controller
             * in seconds.
             */
            T dt = 0.0;

            /**
             * @brief Previous control error.
             *
             * @details
             * Stores the control error from the previous iteration.
             */
            T e_k_1 = 0.0;

            /**
             * @brief Proportional gain.
             */
            T Kp = 0.0;

            /**
             * @brief Derivative gain.
             */
            T Kd = 0.0;

            /**
             * @brief Controller startup state.
             *
             * @details
             * Indicates whether the controller is in its startup state.
             */
            bool start = true;

            /**
             * @brief Previous controller output.
             *
             * @details
             * Stores the controller output from the previous iteration.
             */
            T u_k_1 = 0.0;

            /**
             * @brief Time constant of the derivative low-pass filter.
             *
             * @details
             * Time constant associated with the first-order derivative
             * filter.
             */
            T tau = 0.0;

            /**
             * @brief Maximum controller output.
             *
             * @details
             * Defines the maximum allowed magnitude of the controller
             * output.
             */
            T u_max = 9999999999;
        };


#include "pd_lpf_1.tpp"
    }
}


#endif