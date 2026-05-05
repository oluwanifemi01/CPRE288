#include "movement.h"
#include "open_interface.h"

int move_forward(oi_t *sensor_data, int speed, double distance_cm) {
    double distance_mm = distance_cm * 10.0;
    double sum = 0;

    oi_setWheels(speed, speed);

    while (sum < distance_mm) {
        oi_update(sensor_data);
        sum += sensor_data->distance;

        // Check bump sensors every cycle
        if (sensor_data->bumpLeft || sensor_data->bumpRight) {
            oi_setWheels(0, 0);
            return 1;  // Bump detected
        }
    }

    oi_setWheels(0, 0);
    return 0;  // No bump
}

void move_backward(oi_t *sensor_data, int speed, double distance_cm) {
    double distance_mm = distance_cm * 10.0;
    double sum = 0;

    oi_setWheels(-speed, -speed);

    while (sum < distance_mm) {
        oi_update(sensor_data);
        sum -= sensor_data->distance;  // distance is negative when moving backward
    }

    oi_setWheels(0, 0);
}

void turn_right(oi_t *sensor_data, int speed, double degrees) {
    double angle_sum = 0;

    oi_setWheels(-speed, speed);  // Right wheel back, left wheel forward

    while (angle_sum > -degrees) {
        oi_update(sensor_data);
        angle_sum += sensor_data->angle;  // angle is negative for clockwise
    }

    oi_setWheels(0, 0);
}

void turn_left(oi_t *sensor_data, int speed, double degrees) {
    double angle_sum = 0;

    oi_setWheels(speed, -speed);  // Right wheel forward, left wheel back

    while (angle_sum < degrees) {
        oi_update(sensor_data);
        angle_sum += sensor_data->angle;  // angle is positive for counter-clockwise
    }

    oi_setWheels(0, 0);
}
