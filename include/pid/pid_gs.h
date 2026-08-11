/**
 * @file pid_gs.h
 * @brief PID controller with gain scheduling.
 *
 * @details
 * This file defines the @ref control_system::pid::PID_GS class, which
 * implements a discrete-time proportional-integral-derivative (PID)
 * controller with gain scheduling.
 *
 * The controller supports configurable proportional, integral, and
 * derivative gains, integral and output limits, optional derivative
 * filtering, and feed-forward control.
 *
 * The PID gains can be scheduled according to an operating condition
 * using user-provided lookup tables. The controller interpolates the
 * gain values between lookup-table points.
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

#ifndef CONTROL_SYSTEM_PID_PID_GS_H
#define CONTROL_SYSTEM_PID_PID_GS_H

#include <cstdint>
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
         * @class PID_GS
         * @brief PID controller with gain scheduling.
         *
         * @details
         * The PID_GS class implements a discrete-time PID controller whose
         * proportional, integral, and derivative gains can be adjusted
         * according to a configurable operating condition.
         *
         * Gain scheduling is implemented using lookup tables containing
         * operating-condition values and corresponding proportional,
         * integral, and derivative gains. Gain values between lookup-table
         * points can be obtained through interpolation.
         *
         * The derivative contribution can optionally be filtered using
         * a first-order low-pass filter. A feed-forward contribution can
         * also be added to the controller output.
         *
         * The integral contribution and final controller output are
         * subject to configurable limits.
         *
         * @tparam T Numeric data type used for the controller.
         *
         * @par Example
         * @code{.cpp}
         * control_system::pid::PID_GS<double> controller;
         *
         * controller.init(
         *     0.001,
         *     Kp,
         *     Ki,
         *     Kd,
         *     I_max,
         *     u_max
         * );
         *
         * controller.set_gain_lookup_table(
         *     op_cond_arr,
         *     Kp_arr,
         *     Ki_arr,
         *     Kd_arr,
         *     len
         * );
         *
         * double output = controller.update(
         *     reference,
         *     measurement,
         *     feed_forward,
         *     operating_condition
         * );
         * @endcode
         */
        template <typename T>
        class PID_GS
        {
        public:
            /**
             * @brief Constructs a gain-scheduled PID controller.
             *
             * @details
             * Constructs the controller with its parameters and internal
             * state initialized to their default values.
             */
            PID_GS();

            /**
             * @brief Initializes the PID controller.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Initial proportional gain.
             * @param[in] Ki_ Initial integral gain.
             * @param[in] Kd_ Initial derivative gain.
             * @param[in] I_max_ Maximum allowed integral contribution.
             * @param[in] u_max_ Maximum allowed controller output.
             * @param[in] d_filter_ Enables or disables derivative filtering.
             * @param[in] fc_ Derivative filter cutoff frequency in Hz.
             *
             * @details
             * Initializes the controller parameters and optionally
             * configures the derivative low-pass filter.
             */
            void init(T dt_, T Kp_, T Ki_, T Kd_, T I_max_, T u_max_, bool d_filter_ = false, T fc_ = 10.0);

            /**
             * @brief Sets the PID controller parameters.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] Ki_ Integral gain.
             * @param[in] Kd_ Derivative gain.
             * @param[in] I_max_ Maximum allowed integral contribution.
             * @param[in] u_max_ Maximum allowed controller output.
             * @param[in] d_filter_ Enables or disables derivative filtering.
             * @param[in] fc_ Derivative filter cutoff frequency in Hz.
             *
             * @details
             * Updates the controller parameters and derivative filter
             * configuration.
             */
            void set_param(T dt_, T Kp_, T Ki_, T Kd_, T I_max_, T u_max_, bool d_filter_ = false, T fc_ = 10.0);

            /**
             * @brief Computes the gain-scheduled PID control output.
             *
             * @param[in] x_0 Reference or desired value.
             * @param[in] x Current or measured value.
             * @param[in] u_ff_ Feed-forward control input.
             * @param[in] op_cond_ Current operating condition used for
             * gain scheduling.
             *
             * @return PID controller output.
             *
             * @details
             * Updates the PID gains according to the supplied operating
             * condition, calculates the proportional, integral, derivative,
             * and feed-forward contributions, and produces the controller
             * output.
             *
             * The integral contribution and final controller output are
             * subject to their configured limits.
             */
            T update(T x_0, T x, T u_ff_ = 0.0, T op_cond_ = 0.0);

            /**
             * @brief Resets the PID controller state.
             *
             * @details
             * Resets the stored control error, proportional, integral,
             * derivative, controller output, and startup state.
             */
            void reset();

            /**
             * @brief Merges an external controller output into the PID state.
             *
             * @param[in] u_k_1_ Previous or externally provided controller
             * output.
             *
             * @details
             * Updates the internal controller state using the specified
             * previous controller output.
             */
            void merge(T u_k_1_);

            /**
             * @brief Configures the gain-scheduling lookup tables.
             *
             * @param[in] op_cond_arr_ Array containing operating-condition
             * values.
             * @param[in] Kp_arr_ Array containing proportional gain values.
             * @param[in] Ki_arr_ Array containing integral gain values.
             * @param[in] Kd_arr_ Array containing derivative gain values.
             * @param[in] len_arr_ Number of elements in each lookup table.
             *
             * @details
             * Configures the lookup tables used to determine the
             * proportional, integral, and derivative gains as a function
             * of the operating condition.
             */
            void set_gain_lookup_table(T *op_cond_arr_, T *Kp_arr_, T *Ki_arr_, T *Kd_arr_, uint16_t len_arr_);

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
             * @return Current proportional gain.
             */
            T get_Kp();

            /**
             * @brief Gets the integral gain.
             *
             * @return Current integral gain.
             */
            T get_Ki();

            /**
             * @brief Gets the derivative gain.
             *
             * @return Current derivative gain.
             */
            T get_Kd();

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
             * @brief Updates the PID gains according to the operating condition.
             *
             * @param[in] op_cond_ Current operating condition.
             *
             * @details
             * Determines the proportional, integral, and derivative gains
             * corresponding to the supplied operating condition using the
             * configured gain lookup tables.
             */
            void update_gain(T op_cond_);

            /**
             * @brief Linearly interpolates a value between two lookup-table points.
             *
             * @param[in] x Input value to interpolate.
             * @param[in] x1 Lower or first independent-variable value.
             * @param[in] y1 Value corresponding to @p x1.
             * @param[in] x2 Upper or second independent-variable value.
             * @param[in] y2 Value corresponding to @p x2.
             *
             * @return Interpolated value corresponding to @p x.
             */
            T interpolate(T x, T x1, T y1, T x2, T y2);

            /**
             * @brief Controller sampling time.
             *
             * @details
             * Sampling period of the discrete-time PID controller
             * in seconds.
             */
            T dt = 0.0;

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
             * @brief Operating-condition lookup table.
             *
             * @details
             * Stores the operating-condition values corresponding to
             * the scheduled PID gains.
             */
            T *op_cond_arr;

            /**
             * @brief Proportional-gain lookup table.
             */
            T *Kp_arr;

            /**
             * @brief Integral-gain lookup table.
             */
            T *Ki_arr;

            /**
             * @brief Derivative-gain lookup table.
             */
            T *Kd_arr;

            /**
             * @brief Number of entries in the gain lookup tables.
             */
            uint16_t len_arr;

            /**
             * @brief Controller startup state.
             *
             * @details
             * Indicates whether the controller is in its startup state.
             */
            bool start = true;
        };

#include "pid_gs.tpp"
    }
}

#endif