/*
 * roam_mission.c
 *
 * Roam-and-detonate behaviour. Drives forward continuously, using the IR
 * trip-wire (continuous ADC reads of the front IR sensor) as a fast,
 * cheap "something is in my path" trigger. PING is used only for
 * confirmation when the trip-wire fires, and a full 180° scan is only
 * run when an object actually warrants it.
 *
 * Hierarchy of reactions, fastest first:
 *   1. Cliff/boundary line       (cliff sensors, ~ms)        → escape
 *   2. Bumper                    (digital, instant)          → back off
 *   3. Forward PING <= SCAN      (~50 ms)                    → stop, scan
 *   4. IR trip-wire < NEAR       (~5 ms ADC read)            → stop, confirm
 *   5. IR trip-wire < SLOW       (~5 ms ADC read)            → small course correct
 *   6. Periodic distance budget  (every PERIODIC_SCAN_CM)    → checkpoint scan
 */

#include "roam_mission.h"

#include "Timer.h"
#include "lcd.h"
#include "cyBot_Scan.h"
#include "movement.h"
#include "uart.h"
#include "open_interface.h"
#include "ir_tripwire.h"
#include "mission_types.h"

#include <stdio.h>
#include <stdbool.h>

// ─── Externs from main.c ─────────────────────────────────────────────────────
extern oi_t        *sensor_data;
extern cyBOT_Scan_t scan_data;

extern Object  detected_objects[];
extern int     object_count;
extern Cluster clusters[];
extern int     cluster_count;
extern bool    complete;

extern void perform_full_scan(void);
extern void detect_objects_from_scan(void);
extern void calculate_object_widths(void);
extern void classify_objects(void);
extern void build_clusters(void);
extern int  find_largest_cluster(void);
extern void navigate_to_cluster(int cluster_index);
extern void avoid_obstacle(void);
extern void display_object_info(void);
extern void display_cluster_info(void);
extern bool mission_abort_requested(void);
extern bool hazard_detected(void);
extern void escape_from_hazard(void);

// ─── Roam config ────────────────────────────────────────────────────────────
#define ROAM_FORWARD_SPEED      100
#define ROAM_TURN_SPEED         25
#define ROAM_STEP_CM            5      // forward step between trip-wire checks
#define ROAM_PERIODIC_SCAN_CM   300    // forced scan every ~3 m even if clear
#define ROAM_SCAN_TRIGGER_CM    65.0   // stop and scan before contact distance
#define ROAM_MIN_VALID_PING_CM  5.0    // ignore ultrasonic readings in blind zone

// ─── Helpers ────────────────────────────────────────────────────────────────

// Confirm an IR trip-wire fire with a PING reading. Cheap (one PING ≈ 50 ms).
// Returns the confirmed PING distance, or 999 if PING disagrees with IR.
static double confirm_with_ping(void) {
    cyBOT_Scan(90, &scan_data);
    return scan_data.sound_dist;
}

// Reactive forward roam loop driven by the IR trip-wire.
//   Returns true  → something ahead, caller should run a full scan
//   Returns false → hazard handled internally, caller should keep roaming
static bool roam_until_event(void) {
    uart_sendStr(">>> Roaming (IR trip-wire active)...\r\n");

    // Park servo at 90° so all subsequent IR reads measure the front.
    ir_tripwire_park();

    int    steps      = 0;
    double dist_total = 0.0;

    while (dist_total < (double)ROAM_PERIODIC_SCAN_CM) {
        if (mission_abort_requested()) {
            return false;
        }

        oi_update(sensor_data);

        // 1. Cliff / boundary line (highest priority — fastest sensor)
        if (hazard_detected()) {
            uart_sendStr(">>> Line hazard during roam — escaping.\r\n");
            oi_setWheels(0, 0);
            escape_from_hazard();
            return false;
        }

        // 2. Bumper (already in contact — last-resort safety)
        if (sensor_data->bumpLeft || sensor_data->bumpRight) {
            uart_sendStr(">>> Bump! Object directly ahead.\r\n");
            oi_setWheels(0, 0);
            move_backward(sensor_data, ROAM_FORWARD_SPEED, 4);
            return true;
        }

        // 3. Proactive PING gate — stop and scan before we can bump.
        double ping_ahead = confirm_with_ping();
        if (ping_ahead > ROAM_MIN_VALID_PING_CM && ping_ahead <= ROAM_SCAN_TRIGGER_CM) {
            char msg[96];
            sprintf(msg, ">>> Object %.1f cm ahead — stopping to scan.\r\n",
                    ping_ahead);
            uart_sendStr(msg);
            oi_setWheels(0, 0);
            return true;
        }

        // 4. IR trip-wire — fast (~5 ms) front proximity check
        double ir_cm = ir_tripwire_read_cm();

        if (ir_cm < TRIPWIRE_NEAR_CM) {
            // IR says something is close. Stop wheels and confirm with PING
            // before committing to a scan (IR is noisy; PING is the tiebreaker).
            oi_setWheels(0, 0);
            double ping_cm = confirm_with_ping();

            char msg[96];
            sprintf(msg, ">>> IR trip-wire: %.1f cm  PING confirms: %.1f cm\r\n",
                    ir_cm, ping_cm);
            uart_sendStr(msg);

            if (ping_cm > ROAM_MIN_VALID_PING_CM && ping_cm <= ROAM_SCAN_TRIGGER_CM) {
                // Real object — caller should run a full scan
                return true;
            }
            // PING disagreed (IR false positive). Carry on after a small turn
            // so we look at slightly different geometry next loop.
            uart_sendStr(">>> PING disagrees — IR likely a glint. Course correcting.\r\n");
            turn_right(sensor_data, ROAM_TURN_SPEED, 15);
            continue;
        }

        if (ir_cm < TRIPWIRE_SLOW_CM) {
            cyBOT_Scan(60, &scan_data);
            double right_clearance = scan_data.sound_dist;
            cyBOT_Scan(120, &scan_data);
            double left_clearance = scan_data.sound_dist;

            if (right_clearance > left_clearance) {
                turn_right(sensor_data, ROAM_TURN_SPEED, 10);
            } else {
                turn_left(sensor_data, ROAM_TURN_SPEED, 10);
            }
            ir_tripwire_park();
            continue;
        }

        // 5. Step forward
        int bumped = move_forward(sensor_data, ROAM_FORWARD_SPEED, ROAM_STEP_CM);
        if (bumped) {
            uart_sendStr(">>> Bump during roam step — stopping to scan.\r\n");
            move_backward(sensor_data, ROAM_FORWARD_SPEED, 4);
            return true;
        }
        dist_total += (double)ROAM_STEP_CM;
        steps++;

        if (mission_abort_requested()) {
            return false;
        }

        // 6. Post-step hazard re-check (line could appear mid-step)
        if (hazard_detected()) {
            uart_sendStr(">>> Line hazard mid-step — escaping.\r\n");
            oi_setWheels(0, 0);
            escape_from_hazard();
            return false;
        }
    }

    uart_sendStr(">>> Periodic scan checkpoint reached.\r\n");
    return true;
}

// ─── Public entry point ─────────────────────────────────────────────────────

void run_roam_mission(void) {
    uart_sendStr("\r\n============================================\r\n");
    uart_sendStr("  Roam-and-Detonate Mission\r\n");
    uart_sendStr("============================================\r\n");

    complete = false;

    while (!complete) {
        if (mission_abort_requested()) {
            break;
        }

        // 1. Drive forward, react to bumps/cliffs/IR-trip-wire in real time
        bool object_ahead = roam_until_event();

        if (!object_ahead) {
            // Hazard handled internally; loop and keep roaming
            continue;
        }

        // 2. Something blocked our path — pause and run a full 180° scan
        uart_sendStr("\r\n>>> Object ahead — performing full scan...\r\n");
        perform_full_scan();
        if (mission_abort_requested()) {
            break;
        }
        detect_objects_from_scan();
        calculate_object_widths();
        classify_objects();
        build_clusters();
        display_object_info();
        display_cluster_info();

        // 3. If we found a thin-pillar cluster, go investigate
        if (cluster_count > 0) {
            int idx = find_largest_cluster();
            if (idx != -1) {
                navigate_to_cluster(idx);
                if (mission_abort_requested()) {
                    break;
                }
                continue;
            }
        }

        // 4. No safe cluster — bypass the obstacle and keep roaming
        uart_sendStr(">>> No cluster — bypassing obstacle.\r\n");
        avoid_obstacle();
    }

    if (mission_abort_requested()) {
        uart_sendStr("\r\n>>> Roam mission ABORTED. Stopping.\r\n");
    } else {
        uart_sendStr("\r\n>>> Roam mission COMPLETE. Stopping.\r\n");
    }
}
