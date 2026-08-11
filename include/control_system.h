/**
 * @file control_system.h
 * @brief Main public header for the Control System library.
 *
 * @details
 * This header provides access to the control-system algorithms available
 * in the Control System library.
 *
 * The library provides implementations for:
 *
 * - Proportional (P) control.
 * - Integral (I) control.
 * - Derivative (D) control.
 * - Derivative control with first-order low-pass filtering.
 * - Proportional-Integral (PI) control.
 * - Proportional-Derivative (PD) control.
 * - Proportional-Derivative control with first-order low-pass filtering.
 * - Proportional-Integral-Derivative (PID) control.
 * - PID control with first-order low-pass filtering.
 * - PID control with first-order low-pass filtering and feed-forward.
 * - PID control using setpoint weighting.
 * - PID control using gain scheduling.
 *
 * Including this header provides access to the public control-system
 * classes through a single include file.
 *
 * @author Abhinay Kumar
 * @version 1.0.0
 * @date 2026-08-11
 *
 * @copyright
 * Copyright (c) 2026 Abhinay Kumar
 */

#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

/**
 * @brief Proportional controller.
 */
#include "proportional/p.h"

/**
 * @brief Integral controller.
 */
#include "integral/i.h"

/**
 * @brief Derivative controller.
 */
#include "derivative/d.h"

/**
 * @brief Derivative controller with first-order low-pass filtering.
 */
#include "derivative/d_lpf_1.h"

/**
 * @brief Proportional-Integral controller.
 */
#include "pi/pi.h"

/**
 * @brief Proportional-Derivative controller.
 */
#include "pd/pd.h"

/**
 * @brief Proportional-Derivative controller with first-order low-pass filtering.
 */
#include "pd/pd_lpf_1.h"

/**
 * @brief Proportional-Integral-Derivative controller.
 */
#include "pid/pid.h"

/**
 * @brief PID controller with first-order low-pass filtering.
 */
#include "pid/pid_lpf_1.h"

/**
 * @brief PID controller with first-order low-pass filtering and feed-forward.
 */
#include "pid/pid_lpf_1_ff.h"

/**
 * @brief PID controller using setpoint weighting.
 */
#include "pid/pid_s.h"

/**
 * @brief PID controller with proportional-only control.
 */
#include "pid/pid_P.h"

/**
 * @brief PID controller using gain scheduling.
 */
#include "pid/pid_gs.h"

#endif