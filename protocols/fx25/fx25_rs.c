/*
 * Copyright 2026 Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * * Project Site: https://github.com/hiperiondev/libax25v22 *
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <string.h>

#include "fx25_rs.h"
#include "fx25_gf256.h"

#ifdef MCU_DEBUG
extern uint8_t gf_error_flag;
#endif

void rs_init_params(rs_params_t *params, uint8_t parity_bytes) {
    if (!params)
        return;

    // Safe zero of all fields before switch
    params->n = params->k = params->t = params->nroots = params->fcr = params->prim = 0;

    switch (parity_bytes) {
        case 16:
            params->n = 255;
            params->k = 239;
            params->t = 8;
            params->nroots = 16;
            params->fcr = 1;
            params->prim = 1;
        break;
        case 32:
            params->n = 255;
            params->k = 223;
            params->t = 16;
            params->nroots = 32;
            params->fcr = 1;
            params->prim = 1;
        break;
        case 64:
            params->n = 255;
            params->k = 191;
            params->t = 32;
            params->nroots = 64;
            params->fcr = 1;
            params->prim = 1;
        break;
        default: /* invalid - struct stays zeroed, caller can detect */
        break;
    }
}

// FX.25 tag to RS parameters mapping
bool fx25_get_rs_params(fx25_tag_id_t tag, rs_params_t *params) {
    if (!params)
        return false;

    switch (tag) {
        case FX25_TAG_01:  // RS(255,239)
            params->n = 255;
            params->k = 239;
            params->nroots = 16;
            params->t = 8;
            params->fcr = 1;
            params->prim = 1;
            return true;

        case FX25_TAG_02:  // RS(144,128) shortened
            params->n = 144;
            params->k = 128;
            params->nroots = 16;
            params->t = 8;
            params->fcr = 1;
            params->prim = 1;
            return true;

        case FX25_TAG_03:  // RS(80,64) shortened
            params->n = 80;
            params->k = 64;
            params->nroots = 16;
            params->t = 8;
            params->fcr = 1;
            params->prim = 1;
            return true;

        case FX25_TAG_04:  // RS(48,32) shortened
            params->n = 48;
            params->k = 32;
            params->nroots = 16;
            params->t = 8;
            params->fcr = 1;
            params->prim = 1;
            return true;

        case FX25_TAG_05:  // RS(255,223)
            params->n = 255;
            params->k = 223;
            params->nroots = 32;
            params->t = 16;
            params->fcr = 1;
            params->prim = 1;
            return true;

        case FX25_TAG_06:  // RS(160,128) shortened
            params->n = 160;
            params->k = 128;
            params->nroots = 32;
            params->t = 16;
            params->fcr = 1;
            params->prim = 1;
            return true;

        case FX25_TAG_07:  // RS(96,64) shortened
            params->n = 96;
            params->k = 64;
            params->nroots = 32;
            params->t = 16;
            params->fcr = 1;
            params->prim = 1;
            return true;

        case FX25_TAG_08:  // RS(64,32) shortened
            params->n = 64;
            params->k = 32;
            params->nroots = 32;
            params->t = 16;
            params->fcr = 1;
            params->prim = 1;
            return true;

        case FX25_TAG_09:  // RS(255,191)
            params->n = 255;
            params->k = 191;
            params->nroots = 64;
            params->t = 32;
            params->fcr = 1;
            params->prim = 1;
            return true;

        case FX25_TAG_0A:  // RS(192,128) shortened
            params->n = 192;
            params->k = 128;
            params->nroots = 64;
            params->t = 32;
            params->fcr = 1;
            params->prim = 1;
            return true;

        case FX25_TAG_0B:  // RS(128,64) shortened
            params->n = 128;
            params->k = 64;
            params->nroots = 64;
            params->t = 32;
            params->fcr = 1;
            params->prim = 1;
            return true;

        default:
            return false;
    }
}

// Generate generator polynomial: g(x) = (x-alpha^fcr)(x-alpha^(fcr+1))...(x-alpha^(fcr+nroots-1))
void rs_generate_genpoly(const rs_params_t *params, uint8_t *gen_poly) {
    int i, j;
    uint8_t root;

    if (!params || !gen_poly)
        return;

    // Initialize to g(x) = 1
    memset(gen_poly, 0, params->nroots + 1);
    gen_poly[0] = 1;

    // Multiply by each factor (x - alpha^(fcr+i))
    for (i = 0; i < params->nroots; i++) {
        // Get alpha^(fcr+i) directly from exponential table
        // The primitive element alpha is at gf_exp[1], alpha^2 at gf_exp[2], etc.
        root = gf_exp[params->fcr + i];

        // Multiply current polynomial by (x - root)
        // Working from high degree down to avoid temporary storage
        // New degree is i+1, so start from degree i and work down to 0
        for (j = i; j >= 0; j--) {
            // Coefficient of x^(j+1) in product = coefficient of x^j in current
            // Coefficient of x^j in product = coefficient of x^j * (-root)
            uint8_t coeff = gen_poly[j];
            gen_poly[j + 1] = gf_add(gen_poly[j + 1], coeff);  // Contribution to x^(j+1)
            gen_poly[j] = gf_mul(coeff, root);  // Multiply by root (which is -root in char 2)
        }
    }
}

// Reed-Solomon systematic encoding using LFSR approach
// This is optimized for microcontrollers (no dynamic allocation)
void rs_encode(const rs_params_t *params, const uint8_t *data, uint8_t *parity) {
    uint8_t gen_poly[65];  // Max nroots is 64
    uint8_t feedback;
    int i, j;

    // Generate generator polynomial
    rs_generate_genpoly(params, gen_poly);

    // Initialize parity to zero
    memset(parity, 0, params->nroots);

    // Systematic encoding: compute remainder of data(x) * x^nroots / g(x)
    // This is equivalent to LFSR with generator polynomial
    for (i = 0; i < params->k; i++) {
        feedback = gf_add(data[i], parity[params->nroots - 1]);

        if (feedback != 0) {
            // Shift and add feedback * gen_poly
            for (j = params->nroots - 1; j > 0; j--) {
                if (gen_poly[j] != 0) {
                    parity[j] = gf_add(parity[j - 1], gf_mul(gen_poly[j], feedback));
                } else {
                    parity[j] = parity[j - 1];
                }
            }
            parity[0] = gf_mul(gen_poly[0], feedback);
        } else {
            // Just shift
            for (j = params->nroots - 1; j > 0; j--) {
                parity[j] = parity[j - 1];
            }
            parity[0] = 0;
        }
    }
}

// Syndrome calculation for decoding
static void rs_calc_syndromes(const rs_params_t *params, const uint8_t *codeword, uint8_t *syndromes) {
    int i, j;
    uint8_t alpha_root;

    if (!params || !codeword || !syndromes)
        return;

    // Calculate syndromes: S_i = r(α^(fcr+i)) for i = 0 to nroots-1
    for (i = 0; i < params->nroots; i++) {
        syndromes[i] = 0;

        // Use exponential table directly to get α^(fcr+i)
        alpha_root = gf_exp[params->fcr + i];

        // Evaluate polynomial at α^(fcr+i) using Horner's method
        // Working backwards: codeword[n-1] + codeword[n-2]*α + ... + codeword[0]*x^(n-1)
        for (j = params->n - 1; j >= 0; j--) {
            syndromes[i] = gf_add(gf_mul(syndromes[i], alpha_root), codeword[j]);
        }
    }
}

// Berlekamp-Massey algorithm to find error locator polynomial
static int rs_find_error_locator(const uint8_t *syndromes, int nroots, uint8_t *lambda, uint8_t *omega) {
    uint8_t c[65];  // Connection polynomial
    uint8_t b[65];  // Previous connection polynomial
    uint8_t t[65];  // Temporary
    int i, j, l, m, n;
    uint8_t discr, d;
    int old_l;      // Old l value for proper bounds

    if (!syndromes || !lambda || !omega)
        return -1;

    // Initialize
    memset(lambda, 0, nroots + 1);
    lambda[0] = 1;
    memset(c, 0, nroots + 1);
    c[0] = 1;
    memset(b, 0, nroots + 1);
    b[0] = 1;
    memset(t, 0, sizeof(t));

    l = 0;  // Current length
    m = 1;  // Stored length

    for (n = 0; n < nroots; n++) {
        // Compute discrepancy
        discr = syndromes[n];
        for (i = 1; i <= l && i <= n; i++) {
            discr = gf_add(discr, gf_mul(lambda[i], syndromes[n - i]));
        }

        if (discr == 0) {
            m++;
        } else {
            // Save current lambda before modifying
            for (i = 0; i <= l; i++) {
                t[i] = lambda[i];
            }

            d = discr;
            old_l = l;

            // Update lambda: λ(x) = λ(x) - d * x^m * b(x)
            for (i = 0; i <= nroots - m; i++) {
                if (b[i] != 0) {
                    lambda[i + m] = gf_add(lambda[i + m], gf_mul(d, b[i]));
                }
            }

            if (2 * l <= n) {
                l = n + 1 - l;
                // Update b(x) = λ_old(x) / d
                for (i = 0; i <= nroots; i++) {
                    b[i] = (i <= old_l && t[i] != 0) ? gf_div(t[i], d) : 0;
                }
                m = 1;
            } else {
                m++;
            }
        }
    }

    // Compute error evaluator polynomial omega(x) = S(x) * λ(x) mod x^nroots
    memset(omega, 0, nroots);
    for (i = 0; i <= l; i++) {
        for (j = 0; j < nroots && i + j < nroots; j++) {
            omega[i + j] = gf_add(omega[i + j], gf_mul(lambda[i], syndromes[j]));
        }
    }

    return l;  // Number of errors
}

// Chien search to find error locations
static int rs_find_errors(const rs_params_t *params, const uint8_t *lambda, int nerrs, uint8_t *error_loc) {
    int i, j, count;
    uint8_t sum;
    // Use α^(-1) = α^254
    // params->prim is 1 (field element), not primitive element α=2
    // gf_inverse(1) = 1, which causes all evaluations to be at x=1
    uint8_t alpha_inv = gf_exp[254];  // α^(-1) = α^254

    count = 0;

    // Test all possible locations
    uint8_t alpha_eval = 1;  // α^0 = 1, evaluates to α^(-i) after i updates
    for (i = 0; i < params->n; i++) {
        // Evaluate lambda at α^(-i)
        sum = 0;
        uint8_t term = 1;  // (α^(-i))^0 = 1
        for (j = 0; j <= nerrs; j++) {
            if (lambda[j] != 0) {
                sum = gf_add(sum, gf_mul(lambda[j], term));
            }
            term = gf_mul(term, alpha_eval);  // (α^(-i))^j
        }

        if (sum == 0) {
            // Error found at position i
            if (count < nerrs) {
                error_loc[count] = i;
                count++;
            }
        }

        // Update for next position: α^(-(i+1)) = α^(-i) * α^(-1)
        alpha_eval = gf_mul(alpha_eval, alpha_inv);
    }

    return count;
}

// Forney algorithm to find error magnitudes
static void rs_find_error_values(const rs_params_t *params, const uint8_t *omega, const uint8_t *lambda, int nerrs, const uint8_t *error_loc,
        uint8_t *error_val) {
    int i, j;
    uint8_t alpha_power, alpha_inv;
    uint8_t num, denom;

    alpha_inv = gf_inverse(params->prim);

    for (i = 0; i < nerrs; i++) {
        // Calculate α^(-error_loc[i])
        alpha_power = gf_pow(alpha_inv, error_loc[i]);

        // Numerator: omega(α^(-j))
        num = 0;
        uint8_t alpha_k = 1;
        for (j = 0; j < params->nroots; j++) {
            if (omega[j] != 0) {
                num = gf_add(num, gf_mul(omega[j], alpha_k));
            }
            alpha_k = gf_mul(alpha_k, alpha_power);
        }

        // Denominator: lambda'(α^(-j)) - derivative of lambda
        denom = 0;
        alpha_k = 1;
        for (j = 1; j <= nerrs; j += 2) {  // Odd terms only (derivative)
            if (lambda[j] != 0) {
                denom = gf_add(denom, gf_mul(lambda[j], alpha_k));
            }
            if (j + 1 <= nerrs) {
                alpha_k = gf_mul(alpha_k, alpha_power);
            }
        }

        error_val[i] = gf_div(num, denom);
    }
}

// Complete Reed-Solomon decoder
int rs_decode(const rs_params_t *params, uint8_t *codeword) {
    uint8_t syndromes[64];  // Max nroots is 64
    uint8_t lambda[65];
    uint8_t omega[64];
    uint8_t error_loc[64];
    uint8_t error_val[64];
    int i, nerrs, count;
    bool all_zero;

#ifdef MCU_DEBUG
    // Clear error flag at start of decoding
    gf_error_flag = 0;
#endif

    // Calculate syndromes
    rs_calc_syndromes(params, codeword, syndromes);

    // Check if all syndromes are zero (no errors)
    all_zero = true;
    for (i = 0; i < params->nroots; i++) {
        if (syndromes[i] != 0) {
            all_zero = false;
            break;
        }
    }

    if (all_zero) {
        return 0;  // No errors
    }

    // Find error locator polynomial using Berlekamp-Massey
    nerrs = rs_find_error_locator(syndromes, params->nroots, lambda, omega);

    if (nerrs > params->t) {
        return -1;  // Too many errors, uncorrectable
    }

    // Find error locations using Chien search
    count = rs_find_errors(params, lambda, nerrs, error_loc);

    if (count != nerrs) {
        return -1;  // Could not find all error locations
    }

    // Find error magnitudes using Forney algorithm
    rs_find_error_values(params, omega, lambda, nerrs, error_loc, error_val);

#ifdef MCU_DEBUG
    // Check if division by zero occurred during error value calculation
    if (gf_error_flag) {
        return -2;  // Math error occurred during decoding
    }
#endif

    // Correct errors
    for (i = 0; i < nerrs; i++) {
        codeword[error_loc[i]] = gf_add(codeword[error_loc[i]], error_val[i]);
    }

    return nerrs;
}
