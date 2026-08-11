/**
 * @file p.h
 * @brief Proportional (P) controller.
 *
 * @details
 * This file defines the @ref control_system::proportional::P class,
 * which implements a proportional controller.
 *
 * The controller generates an output proportional to the difference
 * between the reference input and the measured or current value.
 *
 * The controller output is limited by the configured maximum output
 * value.
 *
 * The class is templated to support different numeric types such as
 * @c float and @c double.
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

#ifndef CONTROL_SYSTEM_PROPORTIONAL_P_H
#define CONTROL_SYSTEM_PROPORTIONAL_P_H

#include "../utility/utility.h"

namespace control_system
{
    /**
     * @namespace control_system
     * @brief Contains control-system algorithms and controllers.
     */

    namespace proportional
    {
        /**
         * @namespace proportional
         * @brief Contains proportional control implementations.
         */

        /**
         * @class P
         * @brief Proportional controller.
         *
         * @details
         * The P class implements a proportional controller in which the
         * control output is proportional to the control error.
         *
         * The controller uses a configurable proportional gain and
         * maximum output limit.
         *
         * @tparam T Numeric data type used for the controller.
         *
         * @par Example
         * @code{.cpp}
         * control_system::proportional::P<double> controller;
         *
         * controller.init(2.0, 100.0);
         *
         * double output = controller.update(reference, measurement);
         * @endcode
         */
        template <typename T>
        class P
        {
        public:
            /**
             * @brief Constructs a proportional controller.
             *
             * @details
             * Constructs the controller with its parameters initialized
             * to their default values.
             */
            P();

            /**
             * @brief Initializes the proportional controller.
             *
             * @param[in] Kp_ Proportional gain.
             * @param[in] u_max_ Maximum allowed controller output.
             *
             * @details
             * Initializes the proportional gain and output limit.
             */
            void init(T Kp_, T u_max_);

            /**
             * @brief Sets the controller parameters.
             *
             * @param[in] Kp_ Proportional gain.
             * @param[in] u_max_ Maximum allowed controller output.
             *
             * @details
             * Updates the proportional gain and maximum output limit.
             */
            void set_param(T Kp_, T u_max_);

            /**
             * @brief Computes the proportional control output.
             *
             * @param[in] x_0 Reference or desired value.
             * @param[in] x Current or measured value.
             *
             * @return Proportional control output limited to the configured
             * maximum output.
             */
            T update(T x_0, T x);

            /**
             * @brief Resets the controller.
             *
             * @details
             * Resets the controller state to its initial condition.
             */
            void reset();

            /**
             * @brief Sets the proportional gain.
             *
             * @param[in] Kp_ Proportional gain.
             */
            void set_Kp(T Kp_);

            /**
             * @brief Sets the maximum controller output.
             *
             * @param[in] u_max_ Maximum allowed controller output.
             */
            void set_u_max(T u_max_);

            /**
             * @brief Gets the proportional gain.
             *
             * @return Proportional gain.
             */
            T get_Kp();

            /**
             * @brief Gets the maximum controller output.
             *
             * @return Maximum allowed controller output.
             */
            T get_u_max();

        private:
            /**
             * @brief Proportional gain.
             */
            T Kp = 0.0;

            /**
             * @brief Maximum controller output.
             *
             * @details
             * Defines the upper magnitude limit applied to the controller
             * output.
             */
            T u_max = 9999999999;
        };

#include "../../src/proportional/p.tpp"
    }
}

#endif