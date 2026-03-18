/**
 * @file fx25_gf256.h
 * @brief FX.25 Reed-Solomon GF(256) Forward Error Correction Library
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * This header defines the Galois Field GF(2^8) arithmetic operations required
 * for FX.25 Reed-Solomon forward error correction. FX.25 extends AX.25 with
 * FEC capabilities using RS codes over GF(256) with primitive polynomial
 * 0x11D (x^8 + x^4 + x^3 + x^2 + 1).
 *
 * @section Standards_References
 * - FX.25 Forward Error Correction Extension, Version 01.06
 *   https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 * - Reed-Solomon Error Correction Coding Standards
 * - AX.25 Link Access Protocol for Amateur Packet Radio, Version 2.2
 *
 * @section FX25_Protocol_Summary
 * FX.25 encapsulates standard AX.25 frames within Reed-Solomon coded blocks:
 * - Correlation Tag: 8-byte synchronization pattern identifying FEC mode
 * - Data Field: Complete AX.25 frame (32-239 bytes)
 * - Parity Field: 16, 32, or 64 RS parity bytes
 *
 * Supported RS configurations:
 * - RS(255,239): 16 parity bytes, corrects up to 8 byte errors
 * - RS(255,223): 32 parity bytes, corrects up to 16 byte errors
 * - RS(255,191): 64 parity bytes, corrects up to 32 byte errors
 *
 * @section GF256_Mathematics
 * GF(2^8) is a finite field with 256 elements where:
 * - Addition is bitwise XOR
 * - Multiplication uses logarithm tables with generator element α
 * - Primitive polynomial: p(x) = x^8 + x^4 + x^3 + x^2 + 1 = 0x11D
 * - Field elements are polynomials of degree < 8 with binary coefficients
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef FX25_GF256_H_
#define FX25_GF256_H_

#include <stdint.h>

/*============================================================================*/
/* Error Handling Configuration                                               */
/*============================================================================*/

/**
 * @defgroup ErrorHandling GF(256) Error Detection
 * @brief Runtime error detection for embedded systems
 *
 * When MCU_DEBUG is defined, runtime errors (e.g., division by zero)
 * set a global error flag that can be checked by the application.
 * In production builds, errors are silently ignored for performance.
 *
 * @section Error_Conditions
 * - Division by zero in gf_div()
 * - Invalid field element operations
 *
 * @section Usage
 * @code
 * #define MCU_DEBUG
 * #include "fx25_gf256.h"
 *
 * uint8_t result = gf_div(a, 0);  // Sets gf_error_flag
 * if (gf_error_flag) {
 *     // Handle error condition
 * }
 * @endcode
 */
#ifdef MCU_DEBUG
/**
 * @brief Global error flag for GF(256) operations
 *
 * Set to non-zero when arithmetic errors occur. Must be cleared
 * by application after handling.
 */
extern uint8_t gf_error_flag;

/**
 * @brief Macro to set the global error flag
 * @hideinitializer
 */
#define GF_SET_ERROR() do { gf_error_flag = 1; } while(0)
#else
/**
 * @brief No-op error macro for production builds
 * @hideinitializer
 */
#define GF_SET_ERROR() do { } while(0)
#endif

/*============================================================================*/
/* Galois Field Parameters                                                    */
/*============================================================================*/

/**
 * @defgroup GFParameters Galois Field GF(2^8) Constants
 * @brief Mathematical constants defining the finite field structure
 *
 * GF(2^8) is constructed using the primitive polynomial:
 * p(x) = x^8 + x^4 + x^3 + x^2 + 1
 *
 * In binary representation: 1 0001 1101 = 0x11D
 *
 * This polynomial is irreducible over GF(2) and generates
 * a field of 256 elements suitable for byte-oriented RS coding.
 */
#define GF_PRIMITIVE_POLY 0x11D /**< Primitive polynomial for GF(2^8) field generation */

/*============================================================================*/
/* Lookup Tables                                                              */
/*============================================================================*/

/**
 * @defgroup LookupTables GF(256) Exponential and Logarithm Tables
 * @brief Pre-computed lookup tables for fast field arithmetic
 *
 * These tables implement the discrete logarithm representation of GF(256):
 * - gf_exp[]: Anti-log table maps log values to field elements (α^n)
 * - gf_log[]: Log table maps field elements to discrete logs (log_α)
 *
 * The generator element α = 0x02 is a primitive element of the field.
 *
 * @section Table_Structure
 * - gf_exp has 512 entries to avoid modulo 255 operations during multiplication
 * - gf_log has 256 entries (0x00 is undefined, represented as 0)
 * - Multiplication: a * b = exp[(log[a] + log[b]) mod 255]
 * - Division: a / b = exp[(log[a] - log[b]) mod 255]
 *
 * @section Memory_Layout
 * Tables are declared as const and should be placed in ROM/flash
 * on microcontroller platforms to conserve RAM.
 */
extern const uint8_t gf_exp[512]; /**< Anti-logarithm table: α^n for n in [0,510] */
extern const uint8_t gf_log[256]; /**< Discrete logarithm table: log_α(x) for x in [0,255] */

/*============================================================================*/
/* GF(256) Arithmetic Operations                                              */
/*============================================================================*/

/**
 * @defgroup GFArithmetic Galois Field Arithmetic Operations
 * @brief Inline functions for fast GF(256) computations
 *
 * These operations implement the field arithmetic required for
 * Reed-Solomon encoding and decoding:
 * - Addition/Subtraction: Bitwise XOR (identical operations in GF(2^m))
 * - Multiplication: Log table lookup with modular addition
 * - Division: Log table lookup with modular subtraction
 * - Inversion: Exponentiation using Fermat's little theorem
 *
 * @section Mathematical_Foundations
 * In GF(2^8):
 * - Characteristic is 2, so a + a = 0 for all elements
 * - Additive inverse of a is a itself
 * - Multiplicative group has order 255 (non-zero elements)
 * - α^255 = 1 (Fermat's little theorem)
 */

/**
 * @brief Galois Field addition
 *
 * Adds two elements in GF(2^8). Addition is defined as the bitwise
 * XOR of the polynomial coefficients.
 *
 * @param[in] a First operand (field element)
 * @param[in] b Second operand (field element)
 * @return Sum a + b in GF(2^8)
 *
 * @section Properties
 * - Commutative: gf_add(a,b) = gf_add(b,a)
 * - Associative: gf_add(a,gf_add(b,c)) = gf_add(gf_add(a,b),c)
 * - Identity: gf_add(a,0) = a
 * - Inverse: gf_add(a,a) = 0 (self-inverse)
 *
 * @note In GF(2^m), addition and subtraction are identical operations
 */
static inline uint8_t gf_add(uint8_t a, uint8_t b) {
    return a ^ b;
}

/**
 * @brief Galois Field subtraction
 *
 * Subtracts two elements in GF(2^8). In fields of characteristic 2,
 * subtraction is identical to addition (XOR operation).
 *
 * @param[in] a Minuend (field element)
 * @param[in] b Subtrahend (field element)
 * @return Difference a - b in GF(2^8)
 *
 * @section Equivalence
 * gf_sub(a,b) = gf_add(a,b) = a ^ b
 *
 * @note This identity holds because -1 = 1 in GF(2)
 */
static inline uint8_t gf_sub(uint8_t a, uint8_t b) {
    return a ^ b;
}

/**
 * @brief Galois Field multiplication
 *
 * Multiplies two non-zero elements using logarithm tables.
 * The product is computed as: a × b = α^(log_α(a) + log_α(b))
 *
 * @param[in] a First multiplicand (field element)
 * @param[in] b Second multiplicand (field element)
 * @return Product a × b in GF(2^8), or 0 if either operand is 0
 *
 * @section Algorithm
 * 1. Handle zero operands (return 0)
 * 2. Lookup logarithms: la = log[a], lb = log[b]
 * 3. Add logarithms: sum = la + lb
 * 4. Modular reduction: if sum >= 255, sum -= 255
 * 5. Return anti-log: result = exp[sum]
 *
 * @section Complexity
 * - Time: O(1) - constant time with table lookups
 * - Space: Uses 768 bytes for combined tables
 *
 * @warning Behavior is undefined if a or b are not valid field elements
 */
static inline uint8_t gf_mul(uint8_t a, uint8_t b) {
    if (a == 0 || b == 0)
        return 0;

    /* Use 16-bit intermediate to prevent overflow before modulo */
    uint16_t log_sum = (uint16_t) gf_log[a] + (uint16_t) gf_log[b];

    /* Modulo 255 using conditional subtraction (avoids division) */
    if (log_sum >= 255) {
        log_sum -= 255;
    }

    return gf_exp[log_sum];
}

/**
 * @brief Galois Field division
 *
 * Divides two field elements using logarithm tables.
 * The quotient is computed as: a ÷ b = α^(log_α(a) - log_α(b))
 *
 * @param[in] a Dividend (field element)
 * @param[in] b Divisor (field element, must be non-zero)
 * @return Quotient a ÷ b in GF(2^8), or 0 if a is 0, or 0xFF on division by zero
 *
 * @section Error_Handling
 * - If b = 0: Sets GF error flag (if MCU_DEBUG defined) and returns 0xFF
 * - If a = 0: Returns 0 (valid result in GF)
 *
 * @section Algorithm
 * 1. Handle zero dividend (return 0)
 * 2. Check for zero divisor (set error, return 0xFF)
 * 3. Lookup logarithms: la = log[a], lb = log[b]
 * 4. Subtract logarithms: diff = la - lb
 * 5. Modular reduction: if diff < 0, diff += 255
 * 6. Return anti-log: result = exp[diff]
 *
 * @warning Division by zero triggers error handling and returns invalid value
 */
static inline uint8_t gf_div(uint8_t a, uint8_t b) {
    if (a == 0)
        return 0; /* Zero divided by anything is zero (valid in GF) */

    if (b == 0) {
        /* Division by zero is undefined - indicates algorithm error or corrupted data */
        GF_SET_ERROR();
        return 0xFF; /* Return invalid field element to make error visible */
    }

    /* Signed intermediate for handling negative values */
    int16_t log_diff = (int16_t) gf_log[a] - (int16_t) gf_log[b];

    /* Modulo 255 for negative values */
    if (log_diff < 0) {
        log_diff += 255;
    }

    return gf_exp[log_diff];
}

/**
 * @brief Galois Field multiplicative inverse
 *
 * Computes the multiplicative inverse of a field element using
 * Fermat's little theorem: a^(-1) = a^(254) = exp[255 - log[a]]
 *
 * @param[in] a Field element to invert (must be non-zero)
 * @return Multiplicative inverse a^(-1), or 0 if a is 0
 *
 * @section Mathematical_Basis
 * In GF(2^8), the multiplicative group has order 255.
 * By Fermat's little theorem: a^255 = 1 for all a ≠ 0
 * Therefore: a^(-1) = a^254 = α^(255 - log_α(a))
 *
 * @section Usage
 * Used in Reed-Solomon decoding for:
 * - Forney algorithm (error value computation)
 * - Syndrome polynomial inversion
 * - Chien search optimization
 *
 * @note The inverse of 0 is undefined; function returns 0
 */
static inline uint8_t gf_inverse(uint8_t a) {
    if (a == 0)
        return 0;
    return gf_exp[255 - gf_log[a]];
}

/**
 * @brief Galois Field exponentiation
 *
 * Computes a raised to the power b using repeated squaring.
 * Efficiently handles exponents up to 255 without 64-bit arithmetic.
 *
 * @param[in] a Base (field element)
 * @param[in] b Exponent (0-255, treated modulo 255 for non-zero base)
 * @return a^b in GF(2^8)
 *
 * @section Algorithm
 * Uses exponentiation by squaring (binary method):
 * - result = 1
 * - While b > 0:
 *   - If b is odd: result *= base
 *   - base *= base
 *   - b >>= 1
 *
 * @section Complexity
 * - Time: O(log b) multiplications
 * - Space: O(1) additional storage
 *
 * @section Applications
 * - RS encoder: Generator polynomial evaluation
 * - RS decoder: Syndrome computation, error evaluation
 * - Key expansion in cryptographic applications
 *
 * @note 0^0 is defined as 1 (consistent with mathematical convention)
 * @note For a ≠ 0, a^255 = 1 (order of multiplicative group)
 */
uint8_t gf_pow(uint8_t a, uint8_t b);

#endif /* FX25_GF256_H_ */
