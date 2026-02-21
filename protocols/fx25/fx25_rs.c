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

// debug macro
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
        uint8_t alpha_root = fx25_gf_exp[root];

        DEBUG_PRINT("[RS_DEBUG] Root %u: alpha^%u = 0x%02X\n", i, root, alpha_root);

        for (int j = i; j >= 0; j--) {
            gen_poly[j + 1] = gf_add(gen_poly[j + 1], gf_mul(gen_poly[j], alpha_root));
        }
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
            DEBUG_PRINT("\n                          ");DEBUG_PRINT("%02X ", data[i]);
        }
    }
    DEBUG_PRINT("\n");

    uint8_t gen_poly[65];
    rs_generate_genpoly(params, gen_poly);

    uint8_t temp_parity[64];
    memset(temp_parity, 0, params->nroots);

    // For shortened codes: prepend virtual zeros
    uint8_t pad = params->n - (params->k + params->nroots);

    DEBUG_PRINT("[RS_DEBUG] Virtual zero padding: %u bytes\n", pad);
    DEBUG_PRINT("[RS_DEBUG] Starting LFSR division...\n");

    // Process virtual zero padding first
    for (int i = 0; i < pad; i++) {
        uint8_t feedback = temp_parity[params->nroots - 1];

        for (int j = params->nroots - 1; j > 0; j--) {
            temp_parity[j] = gf_add(temp_parity[j - 1], gf_mul(gen_poly[params->nroots - j], feedback));
        }
        temp_parity[0] = gf_mul(gen_poly[params->nroots], feedback);
    }

    DEBUG_PRINT("[RS_DEBUG] After padding, starting actual data...\n");

    // Now process actual data
    for (int i = 0; i < params->k; i++) {
        uint8_t feedback = gf_add(temp_parity[params->nroots - 1], data[i]);

        if (i < 5) {
            DEBUG_PRINT("[RS_DEBUG] Iteration %d: data=0x%02X, parity[%d]=0x%02X, feedback=0x%02X\n", i, data[i], params->nroots - 1,
                    temp_parity[params->nroots - 1], feedback);
        }

        for (int j = params->nroots - 1; j > 0; j--) {
            temp_parity[j] = gf_add(temp_parity[j - 1], gf_mul(gen_poly[params->nroots - j], feedback));
        }
        temp_parity[0] = gf_mul(gen_poly[params->nroots], feedback);

        if (i < 5 || i == params->k - 1) {
            DEBUG_PRINT("[RS_DEBUG] Parity after iteration %d: ", i);
            for (int j = 0; j < params->nroots && j < 16; j++) {
                DEBUG_PRINT("%02X ", temp_parity[j]);
            }
            if (params->nroots > 16) {
                DEBUG_PRINT("...");DEBUG_PRINT("\n");
            }
        }
    }

    // Reverse parity bytes for systematic codeword format
    // LFSR produces: temp_parity[0] = highest degree parity term
    // Systematic format needs: parity[0] = lowest degree parity term
    for (int i = 0; i < params->nroots; i++) {
        parity[i] = temp_parity[params->nroots - 1 - i];
    }

    DEBUG_PRINT("[RS_DEBUG] Final parity (%u bytes): ", params->nroots);
    for (int i = 0; i < params->nroots; i++) {
        if (i % 16 == 0 && i > 0) {
            DEBUG_PRINT("\n                          ");DEBUG_PRINT("%02X ", parity[i]);

        }
    }
    DEBUG_PRINT("\n[RS_DEBUG] === RS ENCODE END ===\n\n");
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

    // Syndromes
    uint8_t S[64] = { 0 };
    for (int s_idx = 0; s_idx < params->nroots; s_idx++) {
        uint8_t root_exp = params->fcr + s_idx;
        uint8_t s = 0;
        uint32_t init_pow = ((uint32_t) (actual_len - 1) * root_exp) % 255;
        uint8_t alpha_power = fx25_gf_exp[init_pow];
        for (int j = 0; j < actual_len; j++) {
            s = gf_add(s, gf_mul(codeword[j], alpha_power));
            alpha_power = gf_mul(alpha_power, fx25_gf_exp[(255 - root_exp) % 255]);
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

    if (nerrs > params->t) {
        DEBUG_PRINT("[RS_DEBUG] === RS DECODE END (Failed: too many errors) ===\n\n");
        return -1;
    }
    if (nerrs == 0) {
        DEBUG_PRINT("[RS_DEBUG] === RS DECODE END (Success - no errors) ===\n\n");
        return 0;
    }

    // Normalise to monic
    uint8_t lead = C[nerrs];
    uint8_t inv = gf_inverse(lead);
    uint8_t Lambda[65] = { 0 };
    for (int i = 0; i <= nerrs; i++)
        Lambda[i] = gf_mul(C[i], inv);

    // Omega(x) = S(x) * Lambda(x) mod x^{2t}
    uint8_t Omega[65] = { 0 };
    for (int i = 0; i < params->nroots; i++) {
        for (int j = 0; j <= nerrs; j++) {
            if (i - j >= 0)
                Omega[i] = gf_add(Omega[i], gf_mul(S[i - j], Lambda[j]));
        }
    }

    // Chien search
    uint8_t err_pos[64];
    int err_count = 0;
    DEBUG_PRINT("[RS_DEBUG] === Chien Search ===\n");
    for (int j = 0; j < actual_len && err_count < nerrs; j++) {
        int full_pos = actual_len - 1 - j; /* <-- THIS IS THE FIX */
        uint8_t X = (full_pos == 0) ? 1 : fx25_gf_exp[255 - full_pos];

        uint8_t eval = Lambda[0];
        uint8_t xp = X;
        for (int i = 1; i <= nerrs; i++) {
            if (Lambda[i])
                eval = gf_add(eval, gf_mul(Lambda[i], xp));
            xp = gf_mul(xp, X);
        }
        if (eval == 0)
            err_pos[err_count++] = (uint8_t) j;
    }

    if (err_count != nerrs) {
        DEBUG_PRINT("[RS_DEBUG] ERROR: Found %d locations but expected %d\n", err_count, nerrs);
        DEBUG_PRINT("[RS_DEBUG] === RS DECODE END (Failed: location mismatch) ===\n\n");
        return -1;
    }

    // Forney
    for (int i = 0; i < err_count; i++) {
        uint8_t pos = err_pos[i];
        int full_pos = actual_len - 1 - pos;
        uint8_t X = (full_pos == 0) ? 1 : fx25_gf_exp[255 - full_pos]; /* root of Lambda */

        uint8_t omega_val = 0;
        uint8_t xp = 1;
        for (int j = 0; j < params->nroots; j++) {
            if (Omega[j])
                omega_val = gf_add(omega_val, gf_mul(Omega[j], xp));
            xp = gf_mul(xp, X);
        }

        uint8_t lambda_der = 0;
        uint8_t xp_der = 1;
        uint8_t X2 = gf_mul(X, X);
        for (int j = 1; j <= nerrs; j += 2) {
            if (Lambda[j])
                lambda_der = gf_add(lambda_der, gf_mul(Lambda[j], xp_der));
            xp_der = gf_mul(xp_der, X2);
        }

        if (lambda_der == 0)
            return -1;

        uint8_t mag = gf_div(omega_val, lambda_der);
        codeword[pos] = gf_add(codeword[pos], mag);
    }

    DEBUG_PRINT("[RS_DEBUG] === RS DECODE END (Success - corrected %d errors) ===\n\n", err_count);
    return err_count;
}
