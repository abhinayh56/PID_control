/**
 * @file version.h
 * @brief Version information for the Control System library.
 *
 * @details
 * This header defines the version information for the Control System
 * library.
 *
 * The library version follows the MAJOR.MINOR.PATCH format. The version
 * is available both as preprocessor macros and as C++ constants in the
 * @c CONTROL_SYSTEM namespace.
 *
 * The numeric version is encoded as:
 *
 * @code
 * MAJOR * 10000 + MINOR * 100 + PATCH
 * @endcode
 *
 * For example, version 1.2.3 is represented numerically as 10203.
 *
 * @author Abhinay Kumar
 * @version 1.0.0
 * @date 2026-08-11
 *
 * @copyright
 * Copyright (c) 2026 Abhinay Kumar
 */

#ifndef CONTROL_SYSTEM_VERSION_H
#define CONTROL_SYSTEM_VERSION_H


/**
 * @def CONTROL_SYSTEM_VERSION_MAJOR
 * @brief Major version number of the Control System library.
 *
 * @details
 * The major version is incremented when incompatible API or
 * behavioral changes are introduced.
 */
#define CONTROL_SYSTEM_VERSION_MAJOR 1


/**
 * @def CONTROL_SYSTEM_VERSION_MINOR
 * @brief Minor version number of the Control System library.
 *
 * @details
 * The minor version is incremented when backward-compatible
 * functionality is added.
 */
#define CONTROL_SYSTEM_VERSION_MINOR 0


/**
 * @def CONTROL_SYSTEM_VERSION_PATCH
 * @brief Patch version number of the Control System library.
 *
 * @details
 * The patch version is incremented for backward-compatible bug
 * fixes and maintenance changes.
 */
#define CONTROL_SYSTEM_VERSION_PATCH 0


/**
 * @def CONTROL_SYSTEM_VERSION
 * @brief Numeric representation of the Control System library version.
 *
 * @details
 * The version is encoded using the following format:
 *
 * @code
 * MAJOR * 10000 + MINOR * 100 + PATCH
 * @endcode
 *
 * The current version, 1.0.0, is represented as 10000.
 */
#define CONTROL_SYSTEM_VERSION (CONTROL_SYSTEM_VERSION_MAJOR * 10000 + CONTROL_SYSTEM_VERSION_MINOR * 100 + CONTROL_SYSTEM_VERSION_PATCH)


/**
 * @def STRINGIFY_HELPER
 * @brief Helper macro used to convert a preprocessor value to a string.
 *
 * @internal
 */
#define STRINGIFY_HELPER(x) #x


/**
 * @def STRINGIFY
 * @brief Converts a preprocessor value to a string literal.
 *
 * @internal
 */
#define STRINGIFY(x) STRINGIFY_HELPER(x)


/**
 * @def CONTROL_SYSTEM_VERSION_STRING
 * @brief String representation of the Control System library version.
 *
 * @details
 * The version string follows the MAJOR.MINOR.PATCH format.
 *
 * For the current version, this evaluates to @c "1.0.0".
 */
#define CONTROL_SYSTEM_VERSION_STRING STRINGIFY(CONTROL_SYSTEM_VERSION_MAJOR) "." STRINGIFY(CONTROL_SYSTEM_VERSION_MINOR) "." STRINGIFY(CONTROL_SYSTEM_VERSION_PATCH)


/**
 * @namespace CONTROL_SYSTEM
 * @brief Provides version information for the Control System library.
 *
 * @details
 * This namespace provides C++ constants corresponding to the version
 * information defined by the Control System library version macros.
 */
namespace CONTROL_SYSTEM
{
    /**
     * @brief Major version number.
     */
    inline constexpr int version_major = CONTROL_SYSTEM_VERSION_MAJOR;

    /**
     * @brief Minor version number.
     */
    inline constexpr int version_minor = CONTROL_SYSTEM_VERSION_MINOR;

    /**
     * @brief Patch version number.
     */
    inline constexpr int version_patch = CONTROL_SYSTEM_VERSION_PATCH;

    /**
     * @brief Numeric version representation.
     */
    inline constexpr int version = CONTROL_SYSTEM_VERSION;

    /**
     * @brief Version string in MAJOR.MINOR.PATCH format.
     */
    inline constexpr const char *version_string = CONTROL_SYSTEM_VERSION_STRING;
}


#endif