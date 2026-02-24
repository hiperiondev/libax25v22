/**
 * @file fx25_rs.h
 * @brief FX.25 Reed-Solomon Forward Error Correction Header
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * This header defines the Reed-Solomon (RS) error correction implementation for
 * the FX.25 protocol extension to AX.25. FX.25 adds forward error correction
 * capabilities to standard AX.25 frames while maintaining full backward
 * compatibility with legacy equipment.
 *
 * @section Standards_References
 * - FX.25 Protocol Specification v01.06 (2006, Stensat Group)
 *   https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 * - AX.25 Link Access Protocol for Amateur Packet Radio, Version 2.2 (July 1998)
 *   https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * - Reed-Solomon Codes over GF(2^8) with primitive polynomial
 *   x^8 + x^7 + x^2 + x + 1
 *
 * @section FX25_Protocol_Description
 * FX.25 encapsulates standard AX.25 frames within Reed-Solomon codewords to
 * provide forward error correction (FEC) capability. The frame structure is:
 *
 * @verbatim
 * +------------------+------------------------+------------------+
 * |  Correlation Tag |  AX.25 Frame (bit-     |  RS Parity Bytes |
 * |    (8 bytes)     |   stuffed, with FCS)   | (16/32/64 bytes) |
 * +------------------+------------------------+------------------+
 * @endverbatim
 *
 * The correlation tag serves dual purposes: frame synchronization and RS code
 * parameter identification. Eleven predefined 64-bit tags exist, each specifying
 * different RS code configurations optimized for various AX.25 frame sizes.
 *
 * @section Reed_Solomon_Implementation
 * This implementation uses shortened Reed-Solomon codes over GF(2^8):
 * - Field: GF(2^8) with primitive polynomial 0x11D (x^8 + x^4 + x^3 + x^2 + 1)
 *   Note: 0x11D = x^8+x^4+x^3+x^2+1; the wrong form x^8+x^7+x^2+x+1 = 0x187.
 *   Verified against GF_PRIMITIVE_POLY in fx25_gf256.h and the gf_exp/gf_log
 *   lookup tables in fx25_gf256.c.
 * - Generator polynomial roots: consecutive powers of alpha starting at fcr
 * - Code structure: Systematic encoding with parity appended
 * - Error correction capability: t = nroots/2 symbol errors
 *
 * Supported code configurations:
 * - RS(255,239): 16 parity bytes, corrects up to 8 errors (FX25_TAG_01)
 * - RS(144,128): 16 parity bytes, corrects up to 8 errors (FX25_TAG_02)
 * - RS(80,64):   16 parity bytes, corrects up to 8 errors (FX25_TAG_03)
 * - RS(48,32):   16 parity bytes, corrects up to 8 errors (FX25_TAG_04)
 * - RS(255,223): 32 parity bytes, corrects up to 16 errors (FX25_TAG_05)
 * - RS(160,128): 32 parity bytes, corrects up to 16 errors (FX25_TAG_06)
 * - RS(96,64):   32 parity bytes, corrects up to 16 errors (FX25_TAG_07)
 * - RS(64,32):   32 parity bytes, corrects up to 16 errors (FX25_TAG_08)
 * - RS(255,191): 64 parity bytes, corrects up to 32 errors (FX25_TAG_09)
 * - RS(192,128): 64 parity bytes, corrects up to 32 errors (FX25_TAG_0A)
 * - RS(128,64):  64 parity bytes, corrects up to 32 errors (FX25_TAG_0B)
 *
 * @section Backward_Compatibility
 * FX.25 maintains backward compatibility through careful frame construction:
 * 1. The correlation tag is transmitted before the AX.25 frame
 * 2. Legacy AX.25 decoders ignore the tag (treating it as noise/preamble)
 * 3. The AX.25 frame including flags and FCS remains unmodified
 * 4. Parity bytes follow the AX.25 frame, ignored by legacy equipment
 *
 * @section Implementation_Notes
 * - Galois Field tables (gf_exp and gf_log from fx25_gf256.h) are pre-computed
 *   constants; no runtime initialization is required
 * - All operations are performed in GF(2^8) using lookup tables for efficiency
 * - Shortened codes are handled by virtual zero padding during encoding/decoding
 * - Systematic encoding preserves original data, appending parity check bytes
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef FX25_RS_H_
#define FX25_RS_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "fx25_gf256.h"

/**
 * @brief Reed-Solomon Code Parameters Structure
 *
 * Encapsulates all parameters necessary for Reed-Solomon encoding and decoding
 * operations. Supports both full-length and shortened RS codes.
 *
 * @section Parameter_Details
 * - n: Total codeword length in symbols (≤ 255 for GF(2^8))
 * - k: Information symbols (data length)
 * - t: Error correction capability in symbols (floor((n-k)/2))
 * - nroots: Number of parity symbols (n-k, must be even)
 * - fcr: First consecutive root of generator polynomial (typically 1)
 * - prim: Primitive element for generator polynomial roots (typically 1 for alpha)
 *
 * @section Shortened_Codes
 * For shortened RS codes (actual data < k), the implementation virtually pads
 * the data with (n - k - nroots) zero symbols at the beginning. These zeros are
 * not transmitted but affect the parity calculation.
 *
 * @note For FX.25, n is typically 255 (full field) while actual data length
 *       varies based on correlation tag selection
 */
typedef struct {
    uint8_t n; /**< Total codeword length in symbols (1-255) */
    uint8_t k; /**< Information symbols (data payload length) */
    uint8_t t; /**< Error correction capability: t = nroots/2 symbols */
    uint8_t nroots; /**< Number of parity symbols (n - k, must be even) */
    uint8_t fcr; /**< First consecutive root exponent for generator polynomial */
    uint8_t prim; /**< Primitive element for root generation (1 = alpha) */
} rs_params_t;

/**
 * @defgroup FX25Tags FX.25 Correlation Tag Identifiers
 * @brief Enumeration of FX.25 correlation tag values
 *
 * Each tag corresponds to a specific Reed-Solomon code configuration optimized
 * for different AX.25 frame sizes. The correlation tag is an 8-byte (64-bit)
 * pattern transmitted before the AX.25 frame for synchronization and mode
 * identification.
 *
 * @section Tag_Selection_Criteria
 * Tag selection depends on AX.25 frame size (including flags and FCS):
 * - Tags 01-04: 16 parity bytes (8-symbol correction), for smaller frames
 * - Tags 05-08: 32 parity bytes (16-symbol correction), for medium frames
 * - Tags 09-0B: 64 parity bytes (32-symbol correction), for larger frames
 *
 * @section Correlation_Tag_Values
 * The actual 64-bit correlation tag patterns (not shown here) are defined in
 * the implementation file. They are chosen for optimal autocorrelation
 * properties to minimize false synchronization.
 */
typedef enum {
    FX25_TAG_01 = 0x01, /**< RS(255,239): 239 data bytes, 16 parity bytes, 8-symbol correction */
    FX25_TAG_02 = 0x02, /**< RS(144,128): 128 data bytes, 16 parity bytes, 8-symbol correction */
    FX25_TAG_03 = 0x03, /**< RS(80,64): 64 data bytes, 16 parity bytes, 8-symbol correction */
    FX25_TAG_04 = 0x04, /**< RS(48,32): 32 data bytes, 16 parity bytes, 8-symbol correction */
    FX25_TAG_05 = 0x05, /**< RS(255,223): 223 data bytes, 32 parity bytes, 16-symbol correction */
    FX25_TAG_06 = 0x06, /**< RS(160,128): 128 data bytes, 32 parity bytes, 16-symbol correction */
    FX25_TAG_07 = 0x07, /**< RS(96,64): 64 data bytes, 32 parity bytes, 16-symbol correction */
    FX25_TAG_08 = 0x08, /**< RS(64,32): 32 data bytes, 32 parity bytes, 16-symbol correction */
    FX25_TAG_09 = 0x09, /**< RS(255,191): 191 data bytes, 64 parity bytes, 32-symbol correction */
    FX25_TAG_0A = 0x0A, /**< RS(192,128): 128 data bytes, 64 parity bytes, 32-symbol correction */
    FX25_TAG_0B = 0x0B, /**< RS(128,64): 64 data bytes, 64 parity bytes, 32-symbol correction */
} fx25_tag_id_t;

/**
 * @brief Retrieve RS parameters for specified FX.25 correlation tag
 *
 * Populates an rs_params_t structure with the Reed-Solomon code parameters
 * corresponding to the requested FX.25 tag identifier.
 *
 * @section Parameter_Mapping
 * Each tag maps to specific RS(n,k) parameters:
 * - Tag 01: n=255, k=239, nroots=16, t=8
 * - Tag 02: n=144, k=128, nroots=16, t=8
 * - Tag 03: n=80, k=64, nroots=16, t=8
 * - Tag 04: n=48, k=32, nroots=16, t=8
 * - Tag 05: n=255, k=223, nroots=32, t=16
 * - Tag 06: n=160, k=128, nroots=32, t=16
 * - Tag 07: n=96, k=64, nroots=32, t=16
 * - Tag 08: n=64, k=32, nroots=32, t=16
 * - Tag 09: n=255, k=191, nroots=64, t=32
 * - Tag 0A: n=192, k=128, nroots=64, t=32
 * - Tag 0B: n=128, k=64, nroots=64, t=32
 *
 * @param[in]  tag    FX.25 correlation tag identifier (fx25_tag_id_t)
 * @param[out] params Pointer to rs_params_t structure to populate
 * @return true if tag is valid and parameters populated, false otherwise
 *
 * @note All tags use fcr=1 (first consecutive root at alpha^1)
 * @note All tags use prim=1 (primitive element is alpha)
 * @note Caller must ensure params pointer is valid (non-NULL)
 */
bool fx25_get_rs_params(fx25_tag_id_t tag, rs_params_t *params);

/**
 * @brief Initialize Reed-Solomon parameters with custom parity length
 *
 * Configures an rs_params_t structure for a full RS(255,k) code with
 * specified number of parity bytes. Used for custom configurations or
 * when FX.25 standard tags are not applicable.
 *
 * @section Custom_Configuration
 * This function creates a standard RS code with:
 * - n = 255 (full field size)
 * - nroots = specified parity_bytes
 * - k = 255 - parity_bytes
 * - t = parity_bytes / 2
 * - fcr = 1 (standard first consecutive root)
 * - prim = 1 (standard primitive element)
 *
 * @param[out] params       Pointer to rs_params_t structure to initialize
 * @param[in]  parity_bytes Number of parity symbols (must be even, 2-254)
 *
 * @note parity_bytes must be even (RS codes require 2t parity for t errors)
 * @note Maximum parity_bytes is 254 (minimum k=1 information symbol)
 * @note For FX.25 compatibility, use standard tag values via fx25_get_rs_params()
 */
void rs_init_params(rs_params_t *params, uint8_t parity_bytes);

/**
 * @brief Encode data using Reed-Solomon error correction
 *
 * Generates parity check bytes for the provided data using systematic
 * Reed-Solomon encoding. The data remains unchanged; parity bytes are
 * calculated and stored separately.
 *
 * @section Encoding_Process
 * 1. Generate generator polynomial based on parameters (fcr, prim, nroots)
 * 2. For shortened codes: process virtual zero padding (not transmitted)
 * 3. Process actual data bytes through LFSR (Linear Feedback Shift Register)
 * 4. Reverse parity byte order for systematic codeword format
 *
 * @section Systematic_Encoding
 * This implementation uses systematic encoding where the original data
 * forms the high-order coefficients of the codeword polynomial, and parity
 * symbols are appended. The resulting codeword is:
 * C(x) = D(x) * x^{nroots} + P(x)
 * where D(x) is data polynomial and P(x) is parity polynomial.
 *
 * @param[in]  params Pointer to initialized RS parameters structure
 * @param[in]  data   Pointer to data buffer (params->k bytes)
 * @param[out] parity Pointer to parity buffer (params->nroots bytes)
 *                  to receive calculated parity symbols
 *
 * @note Input data length must match params->k
 * @note Parity buffer must be pre-allocated with params->nroots bytes
 * @note For shortened codes, data shorter than k is virtually zero-padded
 * @note Parity bytes must be appended to data to form complete codeword
 */
void rs_encode(const rs_params_t *params, const uint8_t *data, uint8_t *parity);

/**
 * @brief Decode and correct Reed-Solomon codeword
 *
 * Detects and corrects errors in a received RS codeword using the
 * Berlekamp-Massey algorithm for error locator polynomial determination,
 * followed by Chien search for error location and Forney algorithm for
 * error magnitude calculation.
 *
 * @section Decoding_Process
 * 1. Syndrome calculation: Compute 2t syndromes from received codeword
 * 2. Berlekamp-Massey: Find error locator polynomial Lambda(x)
 * 3. Error count determination: Degree of Lambda(x) indicates error count
 * 4. Chien search: Find error locations by evaluating Lambda(x)
 * 5. Forney algorithm: Calculate error magnitudes at located positions
 * 6. Error correction: Apply calculated corrections to received codeword
 *
 * @section Syndrome_Calculation
 * Syndromes S_i are computed by evaluating the received polynomial R(x)
 * at consecutive powers of alpha:
 * S_i = R(alpha^{fcr+i}) for i = 0 to nroots-1
 *
 * @section Error_Correction_Capability
 * The decoder can correct up to t = nroots/2 symbol errors. If more than
 * t errors are present, decoding fails (returns -1).
 *
 * @param[in,out] params   Pointer to RS parameters structure
 * @param[in,out] codeword Pointer to received codeword buffer
 *                         (params->k + params->nroots bytes)
 *                         Contains data + parity on input,
 *                         corrected data + parity on output
 * @return Number of corrected symbol errors (0 = no errors, -1 = uncorrectable)
 *
 * @note Codeword buffer is modified in-place with corrected values
 * @note Returns 0 immediately if all syndromes are zero (no errors detected)
 * @note Returns -1 if error count exceeds correction capability (t)
 * @note Returns -1 if error locator polynomial degree mismatch detected
 */
int rs_decode(const rs_params_t *params, uint8_t *codeword);

/**
 * @brief Generate Reed-Solomon generator polynomial
 *
 * Constructs the generator polynomial g(x) for the Reed-Solomon code
 * defined by the parameters. The generator is the product of minimal
 * polynomials for consecutive roots:
 *
 * g(x) = (x - alpha^{fcr}) * (x - alpha^{fcr+1}) * ... * (x - alpha^{fcr+nroots-1})
 *
 * @section Generator_Polynomial_Properties
 * - Degree: nroots (number of parity symbols)
 * - Roots: alpha^{fcr}, alpha^{fcr+1}, ..., alpha^{fcr+nroots-1}
 * - Coefficients: Elements of GF(2^8), stored in ascending order of degree
 *   (gen_poly[0] = constant term, gen_poly[nroots] = leading coefficient)
 *
 * @section Encoding_Usage
 * The generator polynomial is used in the LFSR encoding process to
 * compute the remainder of data(x) * x^{nroots} divided by g(x), which
 * forms the parity check bytes.
 *
 * @param[in]  params   Pointer to RS parameters structure
 * @param[out] gen_poly Pointer to generator polynomial buffer
 *                     (params->nroots + 1 bytes)
 *
 * @note Leading coefficient gen_poly[nroots] will always be 1 (monic polynomial)
 * @note Buffer must be pre-allocated with params->nroots + 1 bytes
 * @note Polynomial coefficients are in GF(2^8)
 */
void rs_generate_genpoly(const rs_params_t *params, uint8_t *gen_poly);

#endif /* FX25_RS_H_ */
