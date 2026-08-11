/**
 * @file i.h
 * @brief Integral (I) controller.
 *
 * @details
 * This file defines the @ref control_system::integral::I class, which
 * implements a discrete-time integral controller.
 *
 * The controller generates a control output by integrating the error
 * between a reference value and the measured or current value over
 * time.
 *
 * The controller maintains the previous output as internal state and
 * applies a configurable maximum output limit.
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

#ifndef CONTROL_SYSTEM_INTEGRAL_I_H
#define CONTROL_SYSTEM_INTEGRAL_I_H


#include "../utility/utility.h"


namespace control_system
{
	/**
	 * @namespace control_system
	 * @brief Contains control-system algorithms and controllers.
	 */

	namespace integral
	{
		/**
		 * @namespace integral
		 * @brief Contains integral controller implementations.
		 */

		/**
		 * @class I
		 * @brief Discrete-time integral controller.
		 *
		 * @details
		 * The I class implements a discrete-time integral controller
		 * using a configurable integral gain and sampling time.
		 *
		 * The controller maintains its previous output as internal state
		 * and applies a configurable maximum output limit.
		 *
		 * @tparam T Numeric data type used for the controller.
		 *
		 * @par Example
		 * @code{.cpp}
		 * control_system::integral::I<double> controller;
		 *
		 * controller.init(
		 *     0.001,
		 *     Ki,
		 *     0.0,
		 *     100.0
		 * );
		 *
		 * double output = controller.update(reference, measurement);
		 * @endcode
		 */
		template <typename T>
		class I
		{
		public:

			/**
			 * @brief Constructs an integral controller.
			 *
			 * @details
			 * Constructs the controller with its parameters and internal
			 * state initialized to their default values.
			 */
			I();

			/**
			 * @brief Initializes the integral controller.
			 *
			 * @param[in] dt_ Controller sampling time in seconds.
			 * @param[in] Ki_ Integral gain.
			 * @param[in] u_k_1_ Previous controller output.
			 * @param[in] u_max_ Maximum allowed controller output.
			 *
			 * @details
			 * Initializes the controller sampling time, integral gain,
			 * previous controller output, and output limit.
			 */
			void init(T dt_, T Ki_, T u_k_1_, T u_max_);

			/**
			 * @brief Sets the integral controller parameters.
			 *
			 * @param[in] dt_ Controller sampling time in seconds.
			 * @param[in] Ki_ Integral gain.
			 * @param[in] u_k_1_ Previous controller output.
			 * @param[in] u_max_ Maximum allowed controller output.
			 *
			 * @details
			 * Updates the controller sampling time, integral gain,
			 * previous controller output, and maximum output limit.
			 */
			void set_param(T dt_, T Ki_, T u_k_1_, T u_max_);

			/**
			 * @brief Computes the integral control output.
			 *
			 * @param[in] x_0 Reference or desired value.
			 * @param[in] x Current or measured value.
			 *
			 * @return Integral controller output limited by the configured
			 * maximum output.
			 *
			 * @details
			 * Calculates the integral contribution using the current
			 * control error, sampling time, integral gain, and stored
			 * controller state.
			 */
			T update(T x_0, T x);

			/**
			 * @brief Resets the integral controller state.
			 *
			 * @details
			 * Resets the stored controller output to its initial state.
			 */
			void reset();

			/**
			 * @brief Merges an external controller output into the integral state.
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
			 * Sampling period of the discrete-time integral controller
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
		};


#include "i.tpp"
	}
}


#endif