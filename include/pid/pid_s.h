/**
 * @file pid_s.h
 * @brief PID controller with setpoint weighting.
 *
 * @details
 * This file defines the @ref control_system::pid::PID_S class, which
 * implements a proportional-integral-derivative (PID) controller with
 * configurable derivative filtering and feed-forward control.
 *
 * The controller provides access to the individual proportional,
 * integral, and derivative contributions as well as the final control
 * output.
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

#ifndef CONTROL_SYSTEM_PID_PID_S_H
#define CONTROL_SYSTEM_PID_PID_S_H

#include "filters.h"
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
         * @class PID_S
         * @brief PID controller with configurable derivative filtering.
         *
         * @details
         * The PID_S class implements a PID controller with configurable
         * proportional, integral, and derivative time constants.
         *
         * The derivative term can optionally be filtered using a first-order
         * low-pass filter. A feed-forward term can also be included in the
         * controller output.
         *
         * The controller provides configurable integral and output limits
         * and maintains its control state between successive updates.
         *
         * @tparam T Numeric data type used for the controller.
         *
         * @par Example
         * @code{.cpp}
         * control_system::pid::PID_S<double> controller;
         *
         * controller.init(
         *     0.001,
         *     Kp,
         *     Ti,
         *     Td,
         *     I_max,
         *     u_max
         * );
         *
         * double output = controller.update(reference, measurement);
         * @endcode
         */
        template <typename T>
        class PID_S
        {
        public:
            /**
             * @brief Constructs a PID controller.
             *
             * @details
             * Constructs the controller with its parameters and internal
             * state initialized to their default values.
             */
            PID_S();

            /**
             * @brief Initializes the PID controller.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] T_i_ Integral time constant.
             * @param[in] T_d_ Derivative time constant.
             * @param[in] I_max_ Maximum allowed integral contribution.
             * @param[in] u_max_ Maximum allowed controller output.
             * @param[in] d_filter_ Enables or disables derivative filtering.
             * @param[in] fc_ Cutoff frequency of the derivative filter in Hz.
             *
             * @details
             * Initializes the PID controller parameters and optionally
             * configures the derivative low-pass filter.
             */
            void init(T dt_, T Kp_, T T_i_, T T_d_, T I_max_, T u_max_, bool d_filter_ = false, T fc_ = 10.0);

            /**
             * @brief Sets the PID controller parameters.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] T_i_ Integral time constant.
             * @param[in] T_d_ Derivative time constant.
             * @param[in] I_max_ Maximum allowed integral contribution.
             * @param[in] u_max_ Maximum allowed controller output.
             * @param[in] d_filter_ Enables or disables derivative filtering.
             * @param[in] fc_ Cutoff frequency of the derivative filter in Hz.
             *
             * @details
             * Updates the PID controller parameters and derivative filter
             * configuration.
             */
            void set_param(T dt_, T Kp_, T T_i_, T T_d_, T I_max_, T u_max_, bool d_filter_ = false, T fc_ = 10.0);

            /**
             * @brief Computes the PID control output.
             *
             * @param[in] x_0 Reference or desired value.
             * @param[in] x Current or measured value.
             * @param[in] u_ff_ Feed-forward control input.
             *
             * @return PID controller output.
             *
             * @details
             * Calculates the proportional, integral, derivative, and
             * feed-forward contributions and combines them to produce
             * the controller output.
             *
             * The integral contribution and controller output are subject
             * to their configured limits.
             */
            T update(T x_0, T x, T u_ff_ = 0.0);

            /**
             * @brief Resets the PID controller state.
             *
             * @details
             * Resets the stored control error, proportional, integral,
             * derivative, and controller output states.
             */
            void reset();

            /**
             * @brief Merges an external controller output into the PID state.
             *
             * @param[in] u_k_1_ Previous or externally provided controller
             * output.
             *
             * @details
             * Updates the controller state using an externally provided
             * previous controller output.
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
             * @brief Sets the integral time constant.
             *
             * @param[in] T_i_ Integral time constant.
             */
            void set_Ti(T T_i_);

            /**
             * @brief Sets the derivative time constant.
             *
             * @param[in] T_d_ Derivative time constant.
             */
            void set_Td(T T_d_);

            /**
             * @brief Sets the maximum integral contribution.
             *
             * @param[in] I_max_ Maximum allowed integral contribution.
             */
            void set_I_max(T I_max_);

            /**
             * @brief Sets the maximum controller output.
             *
             * @param[in] u_max_ Maximum allowed controller output.
             */
            void set_u_max(T u_max_);

            /**
             * @brief Enables or disables derivative filtering.
             *
             * @param[in] d_filter_ Derivative filter enable state.
             */
            void set_d_filter(bool d_filter_);

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
             * @brief Gets the integral time constant.
             *
             * @return Integral time constant.
             */
            T get_T_i();

            /**
             * @brief Gets the derivative time constant.
             *
             * @return Derivative time constant.
             */
            T get_T_d();

            /**
             * @brief Gets the maximum integral contribution.
             *
             * @return Maximum allowed integral contribution.
             */
            T get_I_max();

            /**
             * @brief Gets the maximum controller output.
             *
             * @return Maximum allowed controller output.
             */
            T get_u_max();

            /**
             * @brief Gets the derivative filter enable state.
             *
             * @return @c true if derivative filtering is enabled,
             * otherwise @c false.
             */
            bool get_d_filter();

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
             * @brief Gets the proportional contribution.
             *
             * @return Current proportional contribution.
             */
            T get_P();

            /**
             * @brief Gets the integral contribution.
             *
             * @return Current integral contribution.
             */
            T get_I();

            /**
             * @brief Gets the derivative contribution.
             *
             * @return Current derivative contribution.
             */
            T get_D();

            /**
             * @brief Gets the controller output.
             *
             * @return Current PID controller output.
             */
            T get_u();

            /**
             * @brief Gets the previous control error.
             *
             * @return Control error from the previous controller iteration.
             */
            T get_e_k_1();

        private:
            /**
             * @brief First-order low-pass filter used for derivative filtering.
             */
            LPF_1<T> lpf;

            /**
             * @brief Controller sampling time.
             *
             * @details
             * Sampling period of the discrete-time PID controller in seconds.
             */
            T dt = 0.0;

            /**
             * @brief Proportional gain.
             */
            T Kp = 0.0;

            /**
             * @brief Integral time constant.
             */
            T T_i = 0.0;

            /**
             * @brief Derivative time constant.
             */
            T T_d = 0.0;

            /**
             * @brief Maximum allowed integral contribution.
             */
            T I_max = 0.0;

            /**
             * @brief Maximum allowed controller output.
             */
            T u_max = 0.0;

            /**
             * @brief Derivative filter enable state.
             */
            bool d_filter = false;

            /**
             * @brief Cutoff frequency of the derivative filter.
             *
             * @details
             * Cutoff frequency in Hz.
             */
            T fc = 0.0;

            /**
             * @brief Previous control error.
             */
            T e_k_1 = 0.0;

            /**
             * @brief Proportional contribution.
             */
            T P = 0.0;

            /**
             * @brief Integral contribution.
             */
            T I = 0.0;

            /**
             * @brief Derivative contribution.
             */
            T D = 0.0;

            /**
             * @brief PID controller output.
             */
            T u = 0.0;

            /**
             * @brief Feed-forward control input.
             */
            T u_ff = 0.0;

            /**
             * @brief Controller startup state.
             *
             * @details
             * Indicates whether the controller is in its startup state.
             */
            bool start = true;
        };

#include "pid_s.tpp"
    }
}

#endif