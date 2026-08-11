/**
 * @file pd.h
 * @brief Proportional-Derivative (PD) controller.
 *
 * @details
 * This file defines the @ref control_system::pd::PD class, which
 * implements a discrete-time proportional-derivative (PD) controller.
 *
 * The controller combines proportional and derivative actions to
 * generate a control output based on the error between a reference
 * value and the measured or current value.
 *
 * The controller maintains the previous control error as internal
 * state and applies a configurable maximum output limit.
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

#ifndef CONTROL_SYSTEM_PD_PD_H
#define CONTROL_SYSTEM_PD_PD_H

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
         * @class PD
         * @brief Discrete-time proportional-derivative controller.
         *
         * @details
         * The PD class implements a discrete-time proportional-derivative
         * controller using configurable proportional and derivative gains.
         *
         * The controller uses the sampling time for its discrete-time
         * derivative action and maintains the previous control error as
         * internal state.
         *
         * @tparam T Numeric data type used for the controller.
         *
         * @par Example
         * @code{.cpp}
         * control_system::pd::PD<double> controller;
         *
         * controller.init(
         *     0.001,
         *     Kp,
         *     Kd,
         *     100.0
         * );
         *
         * double output = controller.update(reference, measurement);
         * @endcode
         */
        template <typename T>
        class PD
        {
        public:
            /**
             * @brief Constructs a PD controller.
             *
             * @details
             * Constructs the controller with its parameters and internal
             * state initialized to their default values.
             */
            PD();

            /**
             * @brief Initializes the PD controller.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] Kd_ Derivative gain.
             * @param[in] u_max_ Maximum allowed controller output.
             *
             * @details
             * Initializes the controller sampling time, proportional and
             * derivative gains, and output limit.
             */
            void init(T dt_, T Kp_, T Kd_, T u_max_);

            /**
             * @brief Sets the PD controller parameters.
             *
             * @param[in] dt_ Controller sampling time in seconds.
             * @param[in] Kp_ Proportional gain.
             * @param[in] Kd_ Derivative gain.
             * @param[in] u_max_ Maximum allowed controller output.
             *
             * @details
             * Updates the controller sampling time, proportional and
             * derivative gains, and maximum output limit.
             */
            void set_param(T dt_, T Kp_, T Kd_, T u_max_);

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
             * using the current control error and the stored previous
             * control error.
             */
            T update(T x_0, T x);

            /**
             * @brief Resets the PD controller state.
             *
             * @details
             * Resets the stored previous control error and startup state.
             */
            void reset();

            /**
             * @brief Merges an external controller output into the PD state.
             *
             * @param[in] u_k_1_ Previous or externally provided controller
             * output.
             *
             * @details
             * Updates the controller state using the specified previous
             * controller output.
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
             * @brief Gets the previous control error.
             *
             * @return Control error from the previous controller iteration.
             */
            T get_e_k_1();

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
             * @brief Maximum controller output.
             *
             * @details
             * Defines the maximum allowed magnitude of the controller
             * output.
             */
            T u_max = 9999999999;
        };

#include "pd.tpp"
    }
}

#endif