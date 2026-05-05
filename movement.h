#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"

/**
 * Move the CyBot forward by distance_cm centimeters at the given speed.
 * Checks bump sensors every update cycle.
 * Returns 1 if a bump sensor was triggered (movement stopped early), 0 otherwise.
 */
int move_forward(oi_t *sensor_data, int speed, double distance_cm);

/**
 * Move the CyBot backward by distance_cm centimeters at the given speed.
 */
void move_backward(oi_t *sensor_data, int speed, double distance_cm);

/**
 * Turn the CyBot right (clockwise) by the given degrees at the given speed.
 */
void turn_right(oi_t *sensor_data, int speed, double degrees);

/**
 * Turn the CyBot left (counter-clockwise) by the given degrees at the given speed.
 */
void turn_left(oi_t *sensor_data, int speed, double degrees);

#endif /* MOVEMENT_H_ */
