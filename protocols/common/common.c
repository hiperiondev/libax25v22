/**
 * @file common.c
 * @brief AX.25 v2.2 Protocol Library - Common Utilities and Definitions
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "common.h"
#include "hal.h"

/**
 * Custom strdup implementation for C99.
 * @param s String to duplicate
 * @return Pointer to duplicated string or NULL on failure
 */
char* my_strdup(const char *s) {
    if (!s)
        return NULL;
    size_t len = strlen(s);
    // Sanity check for microcontroller
    if (len > 1024)
        return NULL;
    char *dup = (char *)hal_mem_alloc((uint16_t)(len + 1));
    if (dup)
        memcpy(dup, s, len + 1);
    return dup;
}

void trim_trailing_spaces(char *str) {
    size_t len = strlen(str);
    while (len > 0 && str[len - 1] == ' ') {
        str[--len] = '\0';
    }
}

// Custom strnlen replacement for portability
size_t my_strnlen(const char *s, size_t maxlen) {
    if (!s) {
        return 0;
    }
    size_t len = 0;
    while (len < maxlen && s[len] != '\0') {
        len++;
    }
    return len;
}

