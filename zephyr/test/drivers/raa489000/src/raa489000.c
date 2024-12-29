/* Copyright 2025 The ChromiumOS Authors
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "driver/tcpm/raa489000.h"
#include "driver/tcpm/tcpci.h"
#include "emul/tcpc/emul_raa489000.h"
#include "emul/tcpc/emul_tcpci.h"
#include "test/drivers/test_state.h"
#include "usb_pd_tcpm.h"

#include <zephyr/drivers/emul.h>
#include <zephyr/fff.h>
#include <zephyr/ztest.h>

#define RAA489000_PORT 0

ZTEST(tcpc_raa489000, test_check_vendor)
{
	int v;

	zassert_ok(tcpc_read16(RAA489000_PORT, TCPC_REG_VENDOR_ID, &v));
	zassert_equal(v, 0x45b);

	tcpm_dump_registers(RAA489000_PORT);
}

ZTEST_SUITE(tcpc_raa489000, drivers_predicate_post_main, NULL, NULL, NULL,
	    NULL);
