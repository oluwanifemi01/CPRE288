/*
 * roam_mission_manual.c
 *
 * Driver-assisted field-roam mode. The bot never drives autonomously here:
 * every movement comes from a user command. Full scans reuse the main object
 * detection pipeline so the existing Python GUI can draw objects/clusters.
 */

#include "roam_mission_manual.h"

#include "Timer.h"
#include "cyBot_Scan.h"
#include "movement.h"
#include "open_interface.h"
#include "uart.h"
#include "mission_types.h"
#include "imu.h"

#include <stdbool.h>
#include <stdio.h>

// Shared state and scan/detection pipeline from main.c
extern oi_t *sensor_data;
extern cyBOT_Scan_t scan_data;
extern bool autonomous_mode;
extern bool mission_aborted;
extern bool imu_ready;
extern bool mission_abort_requested(void);
extern void mission_abort_clear(void);

extern void perform_full_scan(void);
extern void detect_objects_from_scan(void);
extern void calculate_object_widths(void);
extern void classify_objects(void);
extern void build_clusters(void);
extern void display_object_info(void);
extern void display_cluster_info(void);

// Manual mode tuning
#define MANUAL_FORWARD_SPEED     80
#define MANUAL_BACKWARD_SPEED    80
#define MANUAL_TURN_SPEED        25
#define MANUAL_STEP_CM           5.0
#define MANUAL_TURN_DEG          15.0

// Keep these aligned with main.c/cliff_calibration.c.
#define MANUAL_CLIFF_WHITE_THRESHOLD 2900
#define MANUAL_CLIFF_BLACK_THRESHOLD 200
#define MANUAL_CLIFF_DEBOUNCE_MS     8

#define TAPE_LEFT_WHITE   0x01
#define TAPE_RIGHT_WHITE  0x02
#define TAPE_LEFT_BLACK   0x04
#define TAPE_RIGHT_BLACK  0x08

static int manual_tape_state_once(void) {
    int fl = sensor_data->cliffFrontLeftSignal;
    int fr = sensor_data->cliffFrontRightSignal;
    int state = 0;

    if (fl > MANUAL_CLIFF_WHITE_THRESHOLD) state |= TAPE_LEFT_WHITE;
    if (fr > MANUAL_CLIFF_WHITE_THRESHOLD) state |= TAPE_RIGHT_WHITE;
    if (fl < MANUAL_CLIFF_BLACK_THRESHOLD) state |= TAPE_LEFT_BLACK;
    if (fr < MANUAL_CLIFF_BLACK_THRESHOLD) state |= TAPE_RIGHT_BLACK;

    return state;
}

static int manual_tape_state(void) {
    int first;

    oi_update(sensor_data);
    first = manual_tape_state_once();
    if (first == 0) {
        return 0;
    }

    timer_waitMillis(MANUAL_CLIFF_DEBOUNCE_MS);
    oi_update(sensor_data);
    return manual_tape_state_once();
}

static const char *manual_tape_type(int state) {
    bool white = (state & (TAPE_LEFT_WHITE | TAPE_RIGHT_WHITE)) != 0;
    bool black = (state & (TAPE_LEFT_BLACK | TAPE_RIGHT_BLACK)) != 0;

    if (white && black) return "BLACK/WHITE";
    if (white) return "WHITE";
    if (black) return "BLACK";
    return "UNKNOWN";
}

static void manual_report_tape_stop(int state) {
    bool left = (state & (TAPE_LEFT_WHITE | TAPE_LEFT_BLACK)) != 0;
    bool right = (state & (TAPE_RIGHT_WHITE | TAPE_RIGHT_BLACK)) != 0;
    char msg[160];

    if (left && right) {
        sprintf(msg,
                ">>> TAPE WARNING: BOTH front sensors on %s tape. STOP. Back up, then turn around.\r\n",
                manual_tape_type(state));
    } else if (left) {
        sprintf(msg,
                ">>> TAPE WARNING: FRONT-LEFT on %s tape. STOP. Back up, then steer RIGHT.\r\n",
                manual_tape_type(state));
    } else if (right) {
        sprintf(msg,
                ">>> TAPE WARNING: FRONT-RIGHT on %s tape. STOP. Back up, then steer LEFT.\r\n",
                manual_tape_type(state));
    } else {
        sprintf(msg, ">>> TAPE WARNING: boundary detected. STOP.\r\n");
    }

    uart_sendStr(msg);
}

static void manual_scan_field(void) {
    uart_sendStr("\r\n>>> Manual field scan for GUI display...\r\n");
    mission_abort_clear();
    perform_full_scan();
    if (mission_abort_requested()) return;
    detect_objects_from_scan();
    calculate_object_widths();
    classify_objects();
    build_clusters();
    display_object_info();
    display_cluster_info();
}

static void manual_print_distance_ahead(void) {
    char msg[64];

    cyBOT_Scan(90, &scan_data);
    sprintf(msg, "Distance ahead: %.1f cm\r\n", scan_data.sound_dist);
    uart_sendStr(msg);
}

static void manual_print_imu(void) {
    float heading;
    float roll;
    float pitch;
    calib_t calib;
    uint8_t status;
    char msg[140];

    if (!imu_ready) {
        uart_sendStr(">>> IMU heading: unavailable (BNO055 not detected)\r\n");
        return;
    }

    if (!imu_getEulerDeg(&heading, &roll, &pitch)) {
        uart_sendStr(">>> IMU WARNING: heading read failed\r\n");
        return;
    }

    calib = imu_getCalibration();
    status = imu_getStatus();
    sprintf(msg,
            "IMU heading: %.1f deg | roll %.1f | pitch %.1f | calib S%d G%d A%d M%d | status %u\r\n",
            heading, roll, pitch,
            calib.sys, calib.gyr, calib.acc, calib.mag,
            status);
    uart_sendStr(msg);
}

static bool manual_drive_forward_guarded(void) {
    double target_mm = MANUAL_STEP_CM * 10.0;
    double traveled_mm = 0.0;
    int state;

    state = manual_tape_state();
    if (state != 0) {
        manual_report_tape_stop(state);
        return false;
    }

    uart_sendStr(">>> Manual forward guarded by tape sensors\r\n");
    oi_setWheels(MANUAL_FORWARD_SPEED, MANUAL_FORWARD_SPEED);

    while (traveled_mm < target_mm) {
        if (mission_abort_requested()) {
            oi_setWheels(0, 0);
            return false;
        }

        oi_update(sensor_data);
        traveled_mm += sensor_data->distance;

        if (sensor_data->bumpLeft || sensor_data->bumpRight) {
            oi_setWheels(0, 0);
            uart_sendStr(">>> Bump detected. STOP. Back up or scan before continuing.\r\n");
            return false;
        }

        state = manual_tape_state_once();
        if (state != 0) {
            oi_setWheels(0, 0);
            state = manual_tape_state();
            if (state != 0) {
                manual_report_tape_stop(state);
                return false;
            }

            oi_setWheels(MANUAL_FORWARD_SPEED, MANUAL_FORWARD_SPEED);
        }
    }

    oi_setWheels(0, 0);
    return true;
}

static void manual_print_help(void) {
    uart_sendStr("\r\n============================================\r\n");
    uart_sendStr("  Manual Field Roam Mode\r\n");
    uart_sendStr("============================================\r\n");
    uart_sendStr("No autonomous movement in this mode.\r\n");
    uart_sendStr("Commands:\r\n");
    uart_sendStr("  w - guarded forward 5 cm\r\n");
    uart_sendStr("  s - backward 5 cm\r\n");
    uart_sendStr("  a - turn left 15 deg\r\n");
    uart_sendStr("  d - turn right 15 deg\r\n");
    uart_sendStr("  m - scan field and update GUI radar/object list\r\n");
    uart_sendStr("  h - print IMU heading / calibration status\r\n");
    uart_sendStr("  q - exit manual field roam\r\n");
    uart_sendStr("Tape guard: forward stops on black/white tape and reports LEFT/RIGHT.\r\n");
    uart_sendStr("============================================\r\n\r\n");
}

void run_roam_mission_manual(void) {
    char command;
    bool done = false;
    bool previous_autonomous_mode = autonomous_mode;

    autonomous_mode = false;
    mission_abort_clear();
    uart_sendStr("\r\n>>> Mode: MANUAL\r\n");
    manual_print_help();

    // Initial scan gives the driver an immediate map in the GUI.
    manual_scan_field();
    manual_print_imu();

    while (!done) {
        uart_sendStr("ManualRoam> ");
        command = uart_receive();
        uart_sendChar(command);
        uart_sendStr("\r\n");

        switch (command) {
            case 'w': case 'W':
                manual_drive_forward_guarded();
                manual_print_distance_ahead();
                manual_print_imu();
                break;

            case 's': case 'S':
                uart_sendStr(">>> Manual backward\r\n");
                move_backward(sensor_data, MANUAL_BACKWARD_SPEED, MANUAL_STEP_CM);
                manual_print_distance_ahead();
                manual_print_imu();
                break;

            case 'a': case 'A':
                uart_sendStr(">>> Manual turn left\r\n");
                turn_left(sensor_data, MANUAL_TURN_SPEED, MANUAL_TURN_DEG);
                manual_print_distance_ahead();
                manual_print_imu();
                break;

            case 'd': case 'D':
                uart_sendStr(">>> Manual turn right\r\n");
                turn_right(sensor_data, MANUAL_TURN_SPEED, MANUAL_TURN_DEG);
                manual_print_distance_ahead();
                manual_print_imu();
                break;

            case 'm': case 'M':
                manual_scan_field();
                manual_print_imu();
                break;

            case 'h': case 'H':
                manual_print_imu();
                break;

            case 'q': case 'Q':
                oi_setWheels(0, 0);
                done = true;
                break;

            default:
                uart_sendStr(">>> Unknown manual command. Use w/a/s/d, m, h, or q.\r\n");
                break;
        }

        if (mission_aborted) {
            done = true;
        }
    }

    autonomous_mode = previous_autonomous_mode;
    mission_abort_clear();
    uart_sendStr("\r\n>>> Manual field roam exited.\r\n");
    uart_sendStr(autonomous_mode ? ">>> Mode: AUTONOMOUS\r\n" : ">>> Mode: MANUAL\r\n");
}
