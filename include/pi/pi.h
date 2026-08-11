/**
 * @file pi.h
 * @brief Proportional-Integral (PI) controller.
 *
 * @details
 * This file defines the @ref control_system::pi::PI class, which
 * implements a discrete-time proportional-integral (PI) controller.
 *
 * The controller combines proportional and integral actions to generate
 * a control output based on the error between a reference value and the
 * measured or current value.
 *
 * The controller maintains the previous control error and previous
 * controller output as internal state variables and applies a configurable
 * maximum output limit.
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

#ifndef CONTROL_SYSTEM_PI_PI_H
#define CONTROL_SYSTEM_PI_PI_H


#include "../utility/utility.h"


namespace control_system
{
    /**
     * @namespace control_system
     * @brief Contains control-system algorithms and controllers.
     */

    namespace pi
    {
        /**
         * @namespace pi
         * @brief Contains proportional-integral controller implementations.
         */

        /**
         * @class PI
         * @brief Discrete-time proportional-integral controller.
         *
         * @details
         * The PI class implements a discrete-time proportional-integral
         * controller using configurable proportional and integral gains.
         *
         * The controller uses the sampling time for its discrete-time
         * integral action and maintains the previous control error and
         * controller output as internal state.
         *
         * @tparam T Numeric data type used for the controller.
         *
         * @par Example
         * @code{.cpp}
         * control_system::pi::PI<double> controller;
         *
         * controller.init(
         *     0.001,
         *     Kp,
         *     Ki,
         *     0.0,
         *     100.0
         * );
         *
         * double output = controller.update(reference, measurement);
         * @endcode
         */
        template <typename T>
        class PI
        {
        public:

            /**
             * @brief Constructs a PI controller.
             *
             * @details
             * Constructs the controller with its parameters and internal
             * state initialized to their default values.
             */
            PI();

            /**
             * @brief Initializes the PI controller.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] Ki_ Integral gain.
             * @param[in] u_k_1_ Previous controller output.
             * @param[in] u_max_ Maximum allowed controller output.
             *
             * @details
             * Initializes the controller sampling time, proportional and
             * integral gains, previous controller output, and output limit.
             */
            void init(T dt_, T Kp_, T Ki_, T u_k_1_, T u_max_);

            /**
             * @brief Sets the PI controller parameters.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] Ki_ Integral gain.
             * @param[in] u_k_1_ Previous controller output.
             * @param[in] u_max_ Maximum allowed controller output.
             *
             * @details
             * Updates the controller sampling time, gains, previous
             * controller output, and maximum output limit.
             */
            void set_param(T dt_, T Kp_, T Ki_, T u_k_1_, T u_max_);

            /**
             * @brief Computes the PI control output.
             *
             * @param[in] x_0 Reference or desired value.
             * @param[in] x Current or measured value.
             *
             * @return PI controller output limited by the configured
             * maximum output.
             *
             * @details
             * Calculates the proportional and integral contributions
             * using the current control error and the stored controller
             * state.
             */
            T update(T x_0, T x);

            /**
             * @brief Resets the PI controller state.
             *
             * @details
             * Resets the stored previous control error, previous
             * controller output, and startup state.
             */
            void reset();

            /**
             * @brief Merges an external controller output into the PI state.
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
             * Sampling period of the discrete-time PI controller
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
             * @brief Integral gain.
             */
            T Ki = 0.0;

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
             * Indicates whether the controller is in its startup state.
             */
            bool start = true;
        };


#include "pi.tpp"
    }
}


#endif