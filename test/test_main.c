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

#include <stdio.h>

#include "test_ax25.h"
#include "test_hdlc.h"
#include "test_ax25_mgmt.h"
#include "test_ax25_state_machine.h"
#include "test_fx25.h"
#include "test_ax25_segmenter.h"
#include "test_ax25_timers.h"
#include "test_ax25_srej.h"
#include "test_ax25_fullduplex.h"
#include "test_ax25_mux.h"

int errors = 0;

int main() {
    errors += test_hdlc_main();
    errors += test_ax25_main();
    errors += test_ax25_mgmt_main();
    errors += test_ax25_state_machine_main();
    errors += test_fx25_main();
    errors += test_ax25_segmenter_main();
    errors += test_ax25_timers_main();
    errors += test_ax25_srej_main();
    errors += test_ax25_fullduplex_main();
    errors += test_ax25_mux_main();

    printf("\n----------------------------------------------------------------------------------\n");
    printf("All tests Completed. %s\n", errors == 0 ? "All tests passed" : "Some tests failed");
    printf("----------------------------------------------------------------------------------\n\n");
}
