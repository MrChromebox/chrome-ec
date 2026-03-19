#ifndef __CROS_EC_TABLET_MODE_H
#define __CROS_EC_TABLET_MODE_H

/*
 * Tablet mode is a high-level boolean for "tablet" vs "not tablet".
 *
 * Firmware is responsible for setting it based on board-specific sensors
 * (e.g. lid angle, tablet switch GPIO, GMR, etc).
 */
int tablet_get_mode(void);
void tablet_set_mode(int mode);

#endif /* __CROS_EC_TABLET_MODE_H */

