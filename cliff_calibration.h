#ifndef CLIFF_CALIBRATION_H_
#define CLIFF_CALIBRATION_H_

#include "open_interface.h"

/**
 * Capture a baseline reading of the floor by averaging the front-left and
 * front-right cliff signals over `samples` reads (~20 ms apart). Stores the
 * result in the module-static baseline so cliff_is_white() can reference it.
 *
 * Call this AFTER oi_init(), with the bot sitting still on bare floor.
 */
void cliff_baseline_capture(oi_t *sd, int samples);

/**
 * Returns the most recently captured baseline values. Useful for debug prints.
 */
int cliff_baseline_fl(void);
int cliff_baseline_fr(void);

/**
 * Print live cliff sensor readings to UART for `samples` iterations,
 * roughly 100 ms apart. Lists FL / FR / SL / SR + whether the front sensors
 * exceed the configured trip thresholds. Used by the 'c' diagnostic command.
 */
void cliff_calibration_print(oi_t *sd, int samples);

#endif /* CLIFF_CALIBRATION_H_ */
