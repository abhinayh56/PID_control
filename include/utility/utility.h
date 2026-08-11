/**
 * @file utility.h
 * @brief Utility functions for the Control System library.
 *
 * @details
 * This header provides generic utility functions used by the control
 * system components.
 *
 * @author Abhinay Kumar
 * @version 1.0.0
 * @date 2026-08-11
 *
 * @copyright
 * Copyright (c) 2026 Abhinay Kumar
 */

/**
 * @brief Saturates a value within a specified range.
 *
 * @details
 * If @p x is less than the minimum limit, the minimum limit is returned.
 * If @p x is greater than the maximum limit, the maximum limit is returned.
 * Otherwise, the input value is returned unchanged.
 *
 * The function is implemented as a generic function and can be used with
 * different numeric types.
 *
 * @tparam T Numeric data type.
 *
 * @param[in] x Input value to be saturated.
 * @param[in] x_min Minimum allowed value.
 * @param[in] x_max Maximum allowed value.
 *
 * @return Saturated value within the range
 *         [@p x_min, @p x_max].
 *
 * @note
 * The caller should ensure that @p x_min is less than or equal to
 * @p x_max.
 *
 * @par Example
 * @code{.cpp}
 * double output = saturate(input, -10.0, 10.0);
 * @endcode
 */

#ifndef CONTROL_SYSTEM_PID_UTILITY_H
#define CONTROL_SYSTEM_PID_UTILITY_H

template <typename T>
inline constexpr T saturate(T x, T x_min, T x_max)
{
    if (x < x_min)
    {
        return x_min;
    }
    else if (x > x_max)
    {
        return x_max;
    }
    else
    {
        return x;
    }
}

#endif // CONTROL_SYSTEM_PID_UTILITY_H