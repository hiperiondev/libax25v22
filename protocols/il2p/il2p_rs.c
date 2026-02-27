/* -----------------------------------------------------------------------
 * il2p_rs.c  —  Reed-Solomon encoder/decoder for IL2P
 *
 * Uses systematic (non-recursive) polynomial division for encoding.
 * Uses Peterson-Berlekamp-Massey + Chien search + Forney for decoding.
 * All arithmetic is in GF(2^8) via the gf_mul/gf_add inline functions.
 * No floating point. No 64-bit arithmetic.
 * ----------------------------------------------------------------------- */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "il2p_rs.h"
#include "fx25_gf256.h"

/*
 * RS Generator polynomial g(x) with first root = alpha^0:
 *   g(x) = prod(i=0 to nroots-1) (x - alpha^i)
 *
 * We precompute generator polynomials for nroots=2 and nroots=16.
 * For other nroots values, compute on the fly (not needed for IL2P).
 */

/* Compute generator polynomial coefficients for 'nroots' parity symbols.
 * Result: gen[0..nroots], gen[0] is the leading coefficient (= 1).
 * gen[nroots] is the constant term.
 */
static void rs_gen_poly(uint8_t nroots, uint8_t *gen) {
    gen[0] = 1u;
    for (uint8_t i = 0u; i < nroots; i++) {
        gen[i + 1u] = 1u;
        /* root = alpha^i = gf_exp[i] */
        uint8_t root = gf_exp[i];
        /* Multiply (gen so far) by (x - root) = (x + root) in GF(2) */
        for (uint8_t j = i; j > 0u; j--) {
            gen[j] = gf_add(gf_mul(gen[j], root), gen[j - 1u]);
        }
        gen[0] = gf_mul(gen[0], root);
    }
    /* Reverse so gen[0] is constant term, gen[nroots] is leading x^nroots */
    /* Actually keep as-is: gen[0]=leading coeff=1, gen[nroots]=constant */
}

bool il2p_rs_encode(uint8_t *block, uint8_t len, uint8_t nroots) {
    if (!block || nroots == 0u || len == 0u)
        return false;
    /* nroots + 1 coefficients for the generator polynomial */
    uint8_t gen[IL2P_RS_PAY_PARITY + 1u]; /* max size 17 bytes */
    if (nroots > IL2P_RS_PAY_PARITY)
        return false;

    rs_gen_poly(nroots, gen);

    /*
     * Systematic RS encoding by polynomial long division.
     * The codeword is: c(x) = x^nroots * m(x) + r(x)
     * where r(x) = -(x^nroots * m(x)) mod g(x)
     * In GF(2), -r = r, so we XOR in the remainder.
     *
     * Division is done using a feedback shift register of length nroots.
     */
    uint8_t *parity = &block[len];
    memset(parity, 0, nroots);

    for (uint8_t i = 0u; i < len; i++) {
        uint8_t feedback = gf_add(block[i], parity[0]);
        if (feedback != 0u) {
            for (uint8_t j = 0u; j < nroots - 1u; j++) {
                parity[j] = gf_add(parity[j + 1u], gf_mul(feedback, gen[nroots - 1u - j]));
            }
            parity[nroots - 1u] = gf_mul(feedback, gen[0]);
        } else {
            /* Shift without feedback */
            for (uint8_t j = 0u; j < nroots - 1u; j++) {
                parity[j] = parity[j + 1u];
            }
            parity[nroots - 1u] = 0u;
        }
    }
    return true;
}

/*
 * RS Decoder — Berlekamp-Massey algorithm.
 * Returns number of corrections made, or -1 on uncorrectable error.
 *
 * This implementation handles the shortened RS code by treating the
 * missing leading zeros as padding (they are not transmitted).
 */
int8_t il2p_rs_decode(uint8_t *block, uint8_t len, uint8_t nroots) {
    if (!block || nroots == 0u || len == 0u)
        return -1;
    if (nroots > IL2P_RS_PAY_PARITY)
        return -1;

    uint8_t nn = (uint8_t) (len + nroots); /* Total codeword length (shortened) */
    uint8_t t = (uint8_t) (nroots / 2u); /* Max errors correctable */

    /* Step 1: Compute syndromes S[0..nroots-1] */
    /* S[i] = codeword evaluated at alpha^i */
    uint8_t syndromes[IL2P_RS_PAY_PARITY];
    bool all_zero = true;
    for (uint8_t i = 0u; i < nroots; i++) {
        uint8_t root = gf_exp[i]; /* alpha^i */
        uint8_t s = 0u;
        for (uint8_t j = 0u; j < nn; j++) {
            s = gf_add(gf_mul(s, root), block[j]);
        }
        syndromes[i] = s;
        if (s != 0u)
            all_zero = false;
    }
    if (all_zero)
        return 0; /* No errors */

    /* Step 2: Berlekamp-Massey to find error locator polynomial */
    uint8_t C[IL2P_RS_PAY_PARITY + 1u]; /* Error locator sigma */
    uint8_t B[IL2P_RS_PAY_PARITY + 1u]; /* Previous locator */
    memset(C, 0, sizeof(C));
    C[0] = 1u;
    memset(B, 0, sizeof(B));
    B[0] = 1u;
    uint8_t L = 0u, x = 1u;
    uint8_t delta;

    for (uint8_t n_iter = 0u; n_iter < nroots; n_iter++) {
        /* Compute discrepancy delta */
        delta = syndromes[n_iter];
        for (uint8_t i = 1u; i <= L; i++) {
            delta = gf_add(delta, gf_mul(C[i], syndromes[n_iter - i]));
        }
        /* Shift B left by one (multiply by x) */
        for (uint8_t i = IL2P_RS_PAY_PARITY; i > 0u; i--)
            B[i] = B[i - 1u];
        B[0] = 0u;
        x++;
        if (delta == 0u)
            continue;

        uint8_t T[IL2P_RS_PAY_PARITY + 1u];
        memcpy(T, C, sizeof(T));
        for (uint8_t i = 0u; i <= IL2P_RS_PAY_PARITY; i++) {
            C[i] = gf_add(C[i], gf_mul(delta, B[i]));
        }
        if (2u * L <= n_iter) {
            L = (uint8_t) (n_iter + 1u - L);
            memcpy(B, T, sizeof(B));
            /* B = T / delta: each coeff /= delta */
            uint8_t inv_d = gf_div(1u, delta);
            for (uint8_t i = 0u; i <= IL2P_RS_PAY_PARITY; i++)
                B[i] = gf_mul(B[i], inv_d);
            x = 1u;
        }
    }

    if (L > t)
        return -1; /* Too many errors */

    /* Step 3: Chien search — find roots of error locator polynomial */
    uint8_t err_pos[IL2P_RS_PAY_PARITY / 2u];
    uint8_t err_cnt = 0u;

    for (uint8_t i = 0u; i < nn; i++) {
        /* Evaluate sigma at alpha^(255-i) */
        uint8_t xi = gf_exp[255u - i];
        uint8_t val = 0u;
        uint8_t xpow = 1u;
        for (uint8_t j = 0u; j <= L; j++) {
            val = gf_add(val, gf_mul(C[j], xpow));
            xpow = gf_mul(xpow, xi);
        }
        if (val == 0u) {
            if (err_cnt >= t)
                return -1; /* More roots than capacity */
            err_pos[err_cnt++] = i;
        }
    }

    if (err_cnt != L)
        return -1; /* Inconsistent */

    /* Step 4: Forney algorithm — compute error magnitudes */
    for (uint8_t i = 0u; i < err_cnt; i++) {
        uint8_t pos = err_pos[i];
        // Syndrome Horner uses high-degree-first convention: block[j] is coefficient of
        // x^(nn-1-j).  The error locator root for a physical error at index p is
        // X_k = alpha^(nn-1-p), so Chien index pos = nn-1-p, i.e. p = nn-1-pos.
        // Both omega and sigma' must be evaluated at X_k^(-1) = alpha^(255-pos).

        uint8_t xi_inv = gf_exp[255u - pos];  // X_k^(-1) = alpha^(255-pos)
        uint8_t xi = gf_exp[pos];         // X_k      = alpha^pos

        // Formal derivative of sigma at X_k^(-1): only odd-degree terms survive in GF(2)
        uint8_t sigma_deriv = 0u;
        {
            uint8_t xi_inv_pow = 1u;  // xi_inv^0 for j-1=0 at j=1
            for (uint8_t j = 1u; j <= L; j += 2u) {
                sigma_deriv = gf_add(sigma_deriv, gf_mul(C[j], xi_inv_pow));
                // advance xi_inv_pow by xi_inv^2 for next odd j (j -> j+2, exponent -> j+1)
                xi_inv_pow = gf_mul(xi_inv_pow, gf_mul(xi_inv, xi_inv));
            }
        }
        if (sigma_deriv == 0u)
            return -1;

        // Compute error evaluator omega polynomial and evaluate at X_k^(-1).
        // omega[j] = sum_{k=0}^{min(j,L)} C[k] * syndromes[j-k]   (omega = S*sigma mod z^nroots)
        // omega_val = sum_{j=0}^{nroots-1} omega[j] * xi_inv^j
        uint8_t omega_val = 0u;
        {
            uint8_t xi_inv_pow = 1u;  // xi_inv^0
            for (uint8_t j = 0u; j < nroots; j++) {
                uint8_t omega_j = 0u;
                for (uint8_t k = 0u; k <= j && k <= L; k++) {
                    omega_j = gf_add(omega_j, gf_mul(C[k], syndromes[j - k]));
                }
                omega_val = gf_add(omega_val, gf_mul(omega_j, xi_inv_pow));
                xi_inv_pow = gf_mul(xi_inv_pow, xi_inv);
            }
        }

        // Forney formula: magnitude = X_k * omega(X_k^(-1)) / sigma'(X_k^(-1))
        uint8_t magnitude = gf_mul(xi, gf_div(omega_val, sigma_deriv));

        // Apply correction at physical position nn-1-pos (high-degree-first convention)
        if (pos < nn) {
            block[(uint8_t) (nn - 1u - pos)] = gf_add(block[(uint8_t) (nn - 1u - pos)], magnitude);
        }
    }

    /* Verify all syndromes are now zero */
    for (uint8_t i = 0u; i < nroots; i++) {
        uint8_t root = gf_exp[i];
        uint8_t s = 0u;
        for (uint8_t j = 0u; j < nn; j++) {
            s = gf_add(gf_mul(s, root), block[j]);
        }
        if (s != 0u)
            return -1;
    }

    return (int8_t) err_cnt;
}
