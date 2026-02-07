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
 *
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>
#include <stdbool.h>

// Define maximum expected frame size.
// This value can be adjusted based on system requirements
// Typical AX.25 frames are under 256 bytes
#define MAX_FRAME_SIZE 256

// Per AX.25 v2.2 Section 3.9: minimum 136 bits (17 bytes)
// Minimum: Dest(7) + Source(7) + Control(1) + FCS(2) = 17 bytes
#define AX25_MIN_FRAME_SIZE 17

uint16_t CRC(unsigned char *frame, size_t len);
void trim_trailing_spaces(char *str);
size_t my_strnlen(const char *s, size_t maxlen);
char* my_strdup(const char *s);

#endif /* COMMON_H_ */
