/* Copyright 2026 The Chromium OS Authors
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "common.h"
#include "ec_commands.h"
#include "host_command.h"
#include "tablet_mode.h"

/*
 * Tablet mode boolean:
 *   0 = notebook/clamshell
 *   1 = tablet
 *
 * Boards should set this based on sensors (e.g. lid angle / switches).
 */
static int tablet_mode;

int tablet_get_mode(void)
{
	return tablet_mode;
}

void tablet_set_mode(int mode)
{
	int new_mode = !!mode;

	if (tablet_mode == new_mode)
		return;

	tablet_mode = new_mode;

	host_set_single_event(EC_HOST_EVENT_MODE_CHANGE);
}

