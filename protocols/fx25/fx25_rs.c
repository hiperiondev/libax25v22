/**
 * @file fx25_rs.c
 * @brief FX.25 Reed-Solomon Forward Error Correction Implementation
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 */

//#define RS_DEBUG
#include <string.h>
#include <stdio.h>

#include "fx25_rs.h"
#include "fx25_gf256.h"
#include "fx25.h"

#ifdef RS_DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

void rs_init_params(rs_params_t *params, uint8_t parity_bytes) {
    DEBUG_PRINT("[RS_DEBUG] rs_init_params: parity_bytes=%u\n", parity_bytes);

    params->nroots = parity_bytes;
    params->t = parity_bytes / 2;
    params->k = 255 - parity_bytes;
    params->n = 255;  // Full field size for polynomial
    params->fcr = 1;
    params->prim = 1;

    DEBUG_PRINT("[RS_DEBUG] RS params: n=%u, k=%u, t=%u, nroots=%u, fcr=%u, prim=%u\n", params->n, params->k, params->t, params->nroots, params->fcr,
            params->prim);
}

void rs_generate_genpoly(const rs_params_t *params, uint8_t *gen_poly) {
    DEBUG_PRINT("[RS_DEBUG] Generating generator polynomial for nroots=%u, fcr=%u\n", params->nroots, params->fcr);

    memset(gen_poly, 0, params->nroots + 1);
    gen_poly[0] = 1;

    for (uint8_t i = 0; i < params->nroots; i++) {
        uint8_t root = (params->fcr + i) % 255;
        uint8_t alpha_root = gf_exp[root];

        DEBUG_PRINT("[RS_DEBUG] Root %u: alpha^%u = 0x%02X\n", i, root, alpha_root);

        // Build g(x) = prod(x + alpha^(fcr+i)) so that g(alpha^(fcr+i)) = 0.
        // Multiply current poly by (x + alpha_root):
        //   new[j+1] = old[j] + alpha_root * old[j+1]
        //   new[0]   = alpha_root * old[0]
        // Process high-to-low so that old values are used before overwrite.
        for (int j = i; j >= 0; j--) {
            gen_poly[j + 1] = gf_add(gen_poly[j], gf_mul(alpha_root, gen_poly[j + 1]));
        }
        gen_poly[0] = gf_mul(alpha_root, gen_poly[0]);
    }

    DEBUG_PRINT("[RS_DEBUG] Generator polynomial coefficients: ");
    for (uint8_t i = 0; i <= params->nroots; i++) {
        DEBUG_PRINT("%02X ", gen_poly[i]);
    }DEBUG_PRINT("\n");
}

void rs_encode(const rs_params_t *params, const uint8_t *data, uint8_t *parity) {
    DEBUG_PRINT("\n[RS_DEBUG] === RS ENCODE START ===\n");
    DEBUG_PRINT("[RS_DEBUG] Data length (k): %u bytes\n", params->k);
    DEBUG_PRINT("[RS_DEBUG] Input data (%u bytes): ", params->k);
    for (int i = 0; i < params->k && i < 239; i++) {
        if ((i % 16 == 0) && (i > 0)) {
            DEBUG_PRINT("\n                          ");
        }DEBUG_PRINT("%02X ", data[i]);
    }
    DEBUG_PRINT("\n");

    uint8_t gen_poly[65];
    rs_generate_genpoly(params, gen_poly);

    // LFSR systematic encoder: parity[j] accumulates coefficient of x^j in r(x).
    // parity[0] = constant term (x^0), parity[nroots-1] = highest degree term.
    memset(parity, 0, params->nroots);

    for (int i = 0; i < params->k; i++) {
        uint8_t feedback = gf_add(parity[params->nroots - 1], data[i]);
        if (feedback != 0) {
            for (int j = params->nroots - 1; j > 0; j--) {
                parity[j] = gf_add(parity[j - 1], gf_mul(gen_poly[j], feedback));
            }
            parity[0] = gf_mul(gen_poly[0], feedback);
        } else {
            for (int j = params->nroots - 1; j > 0; j--)
                parity[j] = parity[j - 1];
            parity[0] = 0;
        }
    }

    DEBUG_PRINT("[RS_DEBUG] Final parity (%u bytes): ", params->nroots);
    for (int i = 0; i < params->nroots; i++) {
        if (i % 16 == 0 && i > 0) {
            DEBUG_PRINT("\n                          ");
        }DEBUG_PRINT("%02X ", parity[i]);
    }
    DEBUG_PRINT("\n[RS_DEBUG] === RS ENCODE END ===\n\n");

    // Reverse parity bytes: the Horner syndrome evaluator treats codeword[k+0] as
    // the x^(nroots-1) coefficient (highest degree parity), but the LFSR stores
    // parity[0] = x^0 coefficient (constant/lowest degree).  Reversing aligns
    // the two representations so that a clean codeword yields zero syndromes.
    for (int i = 0; i < params->nroots / 2; i++) {
        uint8_t tmp = parity[i];
        parity[i] = parity[params->nroots - 1 - i];
        parity[params->nroots - 1 - i] = tmp;
    }
}

int rs_decode(const rs_params_t *params, uint8_t *codeword) {
    DEBUG_PRINT("\n[RS_DEBUG] ========================================\n");
    DEBUG_PRINT("[RS_DEBUG] === RS DECODE START ===\n");
    DEBUG_PRINT("[RS_DEBUG] ========================================\n");
    DEBUG_PRINT("[RS_DEBUG] Codeword length (n): %u bytes\n", params->n);
    DEBUG_PRINT("[RS_DEBUG] Data length (k): %u bytes\n", params->k);
    DEBUG_PRINT("[RS_DEBUG] Parity length: %u bytes\n", params->nroots);
    DEBUG_PRINT("[RS_DEBUG] Error correction capability (t): %u symbols\n", params->t);

    uint8_t actual_len = params->k + params->nroots;
#ifdef RS_DEBUG
    uint8_t pad = params->n - actual_len;
#endif
    DEBUG_PRINT("[RS_DEBUG] Actual codeword length: %u bytes\n", actual_len);
    DEBUG_PRINT("[RS_DEBUG] Virtual zero padding: %u bytes\n", pad);
    DEBUG_PRINT("[RS_DEBUG] Received codeword (%u bytes): ", actual_len);
    for (int i = 0; i < actual_len; i++) {
        if (i % 16 == 0 && i > 0) {
            DEBUG_PRINT("\n                          ");DEBUG_PRINT("%02X ", codeword[i]);
        }
    }
    DEBUG_PRINT("\n");

    // Syndrome evaluation via Horner: S[r] = R(alpha^(fcr+r)).
    // Horner maps codeword[j] to polynomial coefficient of degree (actual_len-1-j),
    // so an error at array index j contributes e * alpha^((fcr+r)*(actual_len-1-j)).
    uint8_t S[64] = { 0 };
    for (int s_idx = 0; s_idx < params->nroots; s_idx++) {
        uint8_t root_exp = params->fcr + s_idx;
        uint8_t alpha_r = gf_exp[root_exp % 255u];  // alpha^(fcr+s_idx)
        uint8_t s = 0;
        for (int j = 0; j < actual_len; j++) {
            // Horner step: s = s * alpha_r + codeword[j]
            s = gf_add(gf_mul(s, alpha_r), codeword[j]);
        }
        S[s_idx] = s;
        if (s_idx < 8)
            DEBUG_PRINT("[RS_DEBUG] Syndrome[%d] at alpha^%u: 0x%02X\n", s_idx, root_exp, s);
    }

    int nonzero = 0;
    for (int i = 0; i < params->nroots; i++)
        if (S[i] != 0)
            nonzero++;
    DEBUG_PRINT("[RS_DEBUG] Syndromes: %d/%u non-zero\n", nonzero, params->nroots);

    if (nonzero == 0) {
        DEBUG_PRINT("[RS_DEBUG] No errors detected\n");
        DEBUG_PRINT("[RS_DEBUG] === RS DECODE END (Success - no errors) ===\n\n");
        return 0;
    }

    DEBUG_PRINT("[RS_DEBUG] Errors detected - starting Berlekamp-Massey\n");

    if (nonzero == 0)
        return 0;

    // Berlekamp-Massey
    uint8_t C[65] = { 0 }, B[65] = { 0 };
    C[0] = 1;
    B[0] = 1;
    int L = 0, m = 1;
    uint8_t b = 1;

    for (int n = 0; n < params->nroots; n++) {
        uint8_t d = 0;
        for (int i = 0; i <= L; i++)
            d = gf_add(d, gf_mul(C[i], S[n - i]));
        if (d == 0) {
            m++;
        } else if (2 * L <= n) {
            uint8_t temp[65];
            memcpy(temp, C, sizeof(temp));
            uint8_t q = gf_div(d, b);
            for (int i = 0; i < params->nroots - m + 1; i++)
                if (B[i])
                    C[i + m] = gf_add(C[i + m], gf_mul(q, B[i]));
            L = n + 1 - L;
            memcpy(B, temp, sizeof(B));
            b = d;
            m = 1;
        } else {
            uint8_t q = gf_div(d, b);
            for (int i = 0; i < params->nroots - m + 1; i++)
                if (B[i])
                    C[i + m] = gf_add(C[i + m], gf_mul(q, B[i]));
            m++;
        }
    }

    int nerrs = L;
    DEBUG_PRINT("[RS_DEBUG] Error locator degree (nerrs): %d\n", nerrs);

    if (nerrs > params->t)
        return -1;
    if (nerrs == 0)
        return 0;

    if (nerrs > params->t) {
        DEBUG_PRINT("[RS_DEBUG] === RS DECODE END (Failed: too many errors) ===\n\n");
        return -1;
    }
    if (nerrs == 0) {
        DEBUG_PRINT("[RS_DEBUG] === RS DECODE END (Success - no errors) ===\n\n");
        return 0;
    }

    // Normalise Lambda to monic
    uint8_t lead = C[nerrs];
    uint8_t inv = gf_inverse(lead);
    uint8_t Lambda[65] = { 0 };
    for (int i = 0; i <= nerrs; i++)
        Lambda[i] = gf_mul(C[i], inv);

    // Error evaluator polynomial: Omega(x) = S(x)*Lambda(x) mod x^(2t)
    uint8_t Omega[65] = { 0 };
    for (int i = 0; i < params->nroots; i++) {
        for (int j = 0; j <= nerrs; j++) {
            if (i - j >= 0)
                Omega[i] = gf_add(Omega[i], gf_mul(S[i - j], Lambda[j]));
        }
    }

    // Chien search: find positions where Lambda(alpha^(-j_prime)) = 0.
    // Due to Horner indexing, j_prime (Chien variable) maps to array index:
    //   array_index = actual_len - 1 - j_prime
    // start modified part
    uint8_t err_chien[64];  // Chien variable values (NOT direct array indices)
    int err_count = 0;
    DEBUG_PRINT("[RS_DEBUG] === Chien Search ===\n");
    for (int j = 0; j < actual_len && err_count < nerrs; j++) {
        uint8_t X_inv_c = gf_exp[(255u - (unsigned) j) % 255u];  // alpha^(-j)
        uint8_t eval = Lambda[0];
        uint8_t xp = X_inv_c;
        for (int i = 1; i <= nerrs; i++) {
            if (Lambda[i])
                eval = gf_add(eval, gf_mul(Lambda[i], xp));
            xp = gf_mul(xp, X_inv_c);
        }
        if (eval == 0)
            err_chien[err_count++] = (uint8_t) j;
    }

    if (err_count != nerrs) {
        DEBUG_PRINT("[RS_DEBUG] ERROR: Found %d locations but expected %d\n", err_count, nerrs);
        DEBUG_PRINT("[RS_DEBUG] === RS DECODE END (Failed: location mismatch) ===\n\n");
        return -1;
    }

    if (err_count != nerrs)
        return -1;

    // Forney algorithm: compute error magnitude at each located error.
    // X_inv = alpha^(-j_prime) where j_prime is the Chien variable.
    // The actual array index to correct is actual_len-1-j_prime.
    for (int i = 0; i < err_count; i++) {
        uint8_t j_prime = err_chien[i];  // Chien variable
        // Map Chien variable to codeword array index via Horner position convention:
        // Horner treats codeword[idx] as degree (actual_len-1-idx), so
        // a locator X = alpha^(j_prime) corresponds to array index actual_len-1-j_prime.
        uint8_t arr_idx = actual_len - 1 - j_prime;

        // Validate corrected array index is within codeword bounds
        if (arr_idx >= actual_len)
            return -1;

        uint8_t X_inv = gf_exp[(255u - j_prime) % 255u];  // alpha^(-j_prime)

        // Evaluate Omega at X_inv
        uint8_t omega_val = 0;
        uint8_t xp = 1;
        for (int j = 0; j < params->nroots; j++) {
            if (Omega[j])
                omega_val = gf_add(omega_val, gf_mul(Omega[j], xp));
            xp = gf_mul(xp, X_inv);
        }

        // Formal derivative Lambda'(X_inv): only odd-index terms in GF(2^m)
        uint8_t lambda_der = 0;
        uint8_t xp_der = 1;
        uint8_t X_inv2 = gf_mul(X_inv, X_inv);
        for (int j = 1; j <= nerrs; j += 2) {
            if (Lambda[j])
                lambda_der = gf_add(lambda_der, gf_mul(Lambda[j], xp_der));
            xp_der = gf_mul(xp_der, X_inv2);
        }

        if (lambda_der == 0)
            return -1;

        uint8_t mag = gf_div(omega_val, lambda_der);
        // Apply correction at the correct array index (not the Chien variable)
        codeword[arr_idx] = gf_add(codeword[arr_idx], mag);
    }

    DEBUG_PRINT("[RS_DEBUG] === RS DECODE END (Success - corrected %d errors) ===\n\n", err_count);
    return err_count;
}
