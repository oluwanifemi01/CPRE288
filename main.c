
/**
 * Final Lab: CyBot Exploration Mission - Cluster Detection
 * Mission: Find the largest cluster of thin pillars (< 60 cm wide, min 3 pillars),
 *          navigate to within 0.5 m, and play a sound but NOT if a large pillar
 *          (>= 60 cm wide) is present within range.
 *
 * @author SH - 4
 * @date 2026
 */

#include "Timer.h"
#include "lcd.h"
#include "cyBot_Scan.h"
#include "movement.h"
#include "uart.h"
#include "open_interface.h"
#include "adc.h"
#include "ping.h"
#include "cliff_calibration.h"
#include "mission_types.h"
#include "roam_mission.h"
#include "roam_mission_manual.h"
#include "imu.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

//  Configuration Constants 
#define SCAN_RESOLUTION         2       // Degrees per scan step
#define IR_SAMPLES              5       // IR readings to average per angle
#define MAX_OBJECTS             20      // Maximum individual objects to track
#define MAX_CLUSTERS            10      // Maximum clusters to track
#define TURN_SPEED              25      // Speed for turning
#define FORWARD_SPEED           100     // Speed for forward movement
#define TARGET_DISTANCE         50      // Stop distance from cluster (cm) - 0.5 m
#define OBSTACLE_DISTANCE       40      // Distance to trigger obstacle avoidance (cm)
#define BYPASS_FORWARD_CLEARANCE 65.0   // Must be clear before bypass moves forward
#define MIN_BYPASS_CLEARANCE    45.0    // Side clearance needed before bypassing
#define AVOID_SCAN_START_DEG    15      // Avoid extreme sideways servo angles
#define AVOID_SCAN_END_DEG      165
#define AVOID_SCAN_STEP_DEG     15      // Best-path sweep resolution
#define AVOID_FORWARD_BIAS      0.15    // Prefer near-forward paths when similar
#define NUM_SCAN_ANGLES         ((180 / SCAN_RESOLUTION) + 1)

//  Object Classification Thresholds 
#define IR_OBJECT_RAW_THRESHOLD 80      // Raw IR above background to detect object
#define MIN_OBJECT_ANGLE        4       // Minimum angular span to count as object
#define THIN_PILLAR_MAX_WIDTH   60.0    // Max width (cm) for a small fish
#define MIN_CLUSTER_SIZE        3       // Minimum pillars to qualify as a cluster

//  Cluster Gap Tolerance
// Adjacent thin pillars separated by <= this many degrees are treated as one cluster
#define CLUSTER_GAP_DEG         15

//  Large fish / diver safety guard
#define LARGE_FISH_BLAST_RADIUS_CM 60.0 // Reject clusters this close to large fish
#define MAX_VALID_PING_CM          250.0

//  Cliff Sensor Thresholds
//
// Hardware calibration from floor readings:
//   bare floor: ~2735-2795
//   white tape: trip above 2900
//   black tape / hole: trip below 200
//
// Use the 'c' command to view live values and per-sensor delta from baseline.
#define CLIFF_DETECT_WHITE      1       // 1 = check white lines, 0 = ignore
#define CLIFF_WHITE_THRESHOLD   2900    // signal > this          ⇒ white tape
#define CLIFF_BLACK_THRESHOLD   200     // signal < this          ⇒ black tape
#define CLIFF_DEBOUNCE_MS       8       // trip must persist this long
#define CLIFF_BASELINE_SAMPLES  30      // floor samples taken at startup

//  Sound Configuration (Open Interface Song)
// Song 0: three-note chime played only after detonation is confirmed
#define SOUND_SONG_NUM          0

//  Structs — Object & Cluster definitions live in mission_types.h.

//  Globals 

Object   detected_objects[MAX_OBJECTS];
int      object_count = 0;

Cluster  clusters[MAX_CLUSTERS];
int      cluster_count = 0;

bool         autonomous_mode = true;
cyBOT_Scan_t scan_data;
oi_t        *sensor_data;
bool complete = false;
bool mission_aborted = false;
bool imu_ready = false;

int   ir_raw_values[NUM_SCAN_ANGLES];
float ping_distances[NUM_SCAN_ANGLES];
float ir_distances[NUM_SCAN_ANGLES];

//  Function Prototypes 

void perform_full_scan(void);
void detect_objects_from_scan(void);
void calculate_object_widths(void);
void classify_objects(void);
void build_clusters(void);
int  find_largest_cluster(void);
bool large_pillar_nearby(double target_angle, double target_distance);
void play_arrival_sound(void);
void navigate_to_cluster(int cluster_index);
void avoid_obstacle(void);
void manual_control(char command);
void print_imu_status(void);
void display_object_info(void);
void display_cluster_info(void);
double calculate_linear_width(int angle_diff, double distance);
double calculate_polar_separation(double dist_a, double angle_a,
                                  double dist_b, double angle_b);
void mission_abort_clear(void);
void mission_abort_signal(void);
bool mission_abort_requested(void);
bool hazard_detected(void);
void escape_from_hazard(void);
double ir_raw_to_cm(int raw_adc);
static inline bool _front_cliff_tripped(void);


//  Main 

int main(void) {
    timer_init();
    lcd_init();
    uart_init();
    adc_init();
    ping_init();
    cyBOT_init_Scan(0b0111);

    uart_sendStr("\r\n>>> Initializing BNO055 IMU...\r\n");
    imu_ready = imu_configure(IMU);
    if (imu_ready) {
        uart_sendStr(">>> IMU ready: BNO055 in relative IMU fusion mode.\r\n");
    } else {
        uart_sendStr(">>> IMU WARNING: BNO055 not detected. Manual roam will continue without heading.\r\n");
    }

    // *** Replace with your bot's calibration values ***
    right_calibration_value = 274750;
    left_calibration_value  = 1303750;

    sensor_data = oi_alloc();
    oi_init(sensor_data);

    // Capture floor baseline for cliff white-line detection.
    // The bot must be sitting still on bare lab floor when this runs.
    uart_sendStr("\r\n>>> Calibrating floor baseline (place on bare floor)...\r\n");
    cliff_baseline_capture(sensor_data, CLIFF_BASELINE_SAMPLES);

    // Load detonation confirmation chime into song slot 0: C5, E5, G5.
    // Note numbers: C5=72, E5=76, G5=79; duration units = 64ths of a second
    uint8_t notes[3]    = {72, 76, 79};
    uint8_t durations[3] = {32, 32, 48};  // ~0.5 s, 0.5 s, 0.75 s
    oi_loadSong(SOUND_SONG_NUM, 3, notes, durations);

    uart_sendStr("\r\n");
    uart_sendStr("============================================\r\n");
    uart_sendStr("  CyBot: Thin-Pillar Cluster Mission\r\n");
    uart_sendStr("  Target: Largest cluster of thin pillars\r\n");
    uart_sendStr("  (>= 3 pillars, each < 60 cm wide)\r\n");
    uart_sendStr("============================================\r\n");
    uart_sendStr("Commands (AUTONOMOUS mode - default):\r\n");
    uart_sendStr("  's' - Start autonomous mission\r\n");
    uart_sendStr("  'g' - Go to largest thin-pillar cluster\r\n");
    uart_sendStr("  't' - Toggle autonomous/manual mode\r\n");
    uart_sendStr("Commands (MANUAL mode):\r\n");
    uart_sendStr("  'w' - forward   's' - backward\r\n");
    uart_sendStr("  'a' - turn left 'd' - turn right\r\n");
    uart_sendStr("  'm' - manual scan\r\n");
    uart_sendStr("  't' - toggle back to autonomous\r\n");
    uart_sendStr("Diagnostics:\r\n");
    uart_sendStr("  'c' - cliff sensor live readout\r\n");
    uart_sendStr("  'b' - re-capture floor baseline\r\n");
    uart_sendStr("  'q' - ABORT active mission / stop wheels\r\n");
    uart_sendStr("Mission Modes (direct commands, work from Ready prompt):\r\n");
    uart_sendStr("  's' - roam-and-detonate (IR trip-wire)\r\n");
    uart_sendStr("  'r' - manual field roam (driver-controlled + GUI scan)\r\n");
    uart_sendStr("============================================\r\n\r\n");

    char command;
    int  target_cluster_index;

    while (1) {
        uart_sendStr("Ready> ");
        command = uart_receive();
        uart_sendChar(command);
        uart_sendStr("\r\n");

        switch (command) {

            case 'r': case 'R':
                // Manual field roam has its own direct entry point. It does
                // not depend on the autonomous/manual toggle because the
                // mode owns its own command loop until the driver exits.
                mission_abort_clear();
                uart_sendStr("\r\n>>> Starting manual field roam...\r\n");
                run_roam_mission_manual();
                break;

            case 's': case 'S':
                if (!autonomous_mode) {
                    // In manual mode, 's' is the backward command
                    manual_control(command);
                    break;
                }
                // Roam-and-detonate mission lives in roam_mission.c. It uses
                // the IR trip-wire (continuous fast ADC reads) for forward
                // collision sensing, with PING confirmation only when the
                // trip-wire fires.
                mission_abort_clear();
                run_roam_mission();
                break;

            case 'g': case 'G':
                if (object_count == 0) {
                    uart_sendStr("\r\n>>> ERROR: No objects detected. Run scan first.\r\n");
                } else if (cluster_count == 0) {
                    uart_sendStr("\r\n>>> No valid thin-pillar clusters found (need >= 3 thin pillars).\r\n");
                } else {
                    target_cluster_index = find_largest_cluster();
                    if (target_cluster_index == -1) {
                        uart_sendStr("\r\n>>> No safe cluster to navigate to.\r\n");
                    } else {
                        uart_sendStr("\r\n>>> Navigating to largest thin-pillar cluster...\r\n");
                        mission_abort_clear();
                        navigate_to_cluster(target_cluster_index);
                    }
                }
                break;

            case 't': case 'T':
                autonomous_mode = !autonomous_mode;
                uart_sendStr(autonomous_mode ? "\r\n>>> Mode: AUTONOMOUS\r\n"
                                             : "\r\n>>> Mode: MANUAL\r\nUse w/a/s/d, 'm' for scan\r\n");
                break;

            case 'c': case 'C':
                // Cliff sensor calibration — implementation lives in
                // cliff_calibration.c. Prints live FL/FR/SL/SR + delta from
                // floor baseline + which threshold (if any) is being tripped.
                cliff_calibration_print(sensor_data, 20);
                break;

            case 'b': case 'B':
                // Re-capture the floor baseline. Useful if the bot was moved
                // to a different surface mid-session.
                uart_sendStr("\r\n>>> Re-capturing floor baseline (hold still)...\r\n");
                cliff_baseline_capture(sensor_data, CLIFF_BASELINE_SAMPLES);
                break;

            case 'h': case 'H':
                print_imu_status();
                break;

            case 'q': case 'Q':
                mission_abort_signal();
                break;

            case 'w': case 'a': case 'd': case 'm':
            case 'W': case 'A': case 'D': case 'M':
                if (!autonomous_mode) {
                    manual_control(command);
                } else {
                    uart_sendStr("\r\n>>> Switch to manual mode first (press 't')\r\n");
                }
                break;

            default:
                uart_sendStr(">>> Unknown command\r\n");
                break;
        }
    }

    oi_free(sensor_data);
    return 0;
}


//  Scanning 

/**
 * Sweep 0 - 180, averaging IR_SAMPLES readings at each step.
 * Stores results in ir_raw_values[] and ping_distances[].
 */
void perform_full_scan(void) {
    uart_sendStr("Scanning 0-180 degrees...\r\n");
    uart_sendStr("Angle  |  IR Raw (avg)  |  PING (cm)  |  IR (cm)\r\n");
    uart_sendStr("-------|----------------|-------------|----------\r\n");

    int angle, index = 0;
    for (angle = 0; angle <= 180; angle += SCAN_RESOLUTION) {
        if (mission_abort_requested()) {
            object_count = 0;
            cluster_count = 0;
            return;
        }

        long ir_sum = 0;
        double ping_sum = 0.0;
        int i;
        for (i = 0; i < IR_SAMPLES; i++) {
            cyBOT_Scan(angle, &scan_data);
            ir_sum += scan_data.IR_raw_val;
            ping_sum += scan_data.sound_dist;
            timer_waitMillis(5);
        }
        ir_raw_values[index]   = (int)(ir_sum / IR_SAMPLES);
        ping_distances[index]  = ping_sum / IR_SAMPLES;
        ir_distances[index]    = ir_raw_to_cm(ir_raw_values[index]);

        if (angle % 10 == 0) {
            char line[100];
            sprintf(line, " %3d   |    %5d       |  %6.1f  |  %6.1f\r\n",
                    angle, ir_raw_values[index], ping_distances[index], ir_distances[index]);
            uart_sendStr(line);
        }
        index++;
    }
    uart_sendStr("Scan complete!\r\n\r\n");
}


//  Detection 

/**
 * Detect objects as contiguous IR readings above a dynamic background.
 * This is less sensitive to room lighting and sensor drift than fixed edges.
 */
void detect_objects_from_scan(void) {
    object_count = 0;

    uart_sendStr("Detecting objects from scan data...\r\n");

    int sorted_vals[NUM_SCAN_ANGLES];
    int i, j, key;
    for (i = 0; i < NUM_SCAN_ANGLES; i++) {
        sorted_vals[i] = ir_raw_values[i];
    }

    for (i = 1; i < NUM_SCAN_ANGLES; i++) {
        key = sorted_vals[i];
        j = i - 1;
        while (j >= 0 && sorted_vals[j] > key) {
            sorted_vals[j + 1] = sorted_vals[j];
            j--;
        }
        sorted_vals[j + 1] = key;
    }

    int background = sorted_vals[NUM_SCAN_ANGLES / 4];
    int threshold = background + IR_OBJECT_RAW_THRESHOLD;
    char msg[80];
    sprintf(msg, "Background IR: %d, Threshold: %d\r\n", background, threshold);
    uart_sendStr(msg);

    bool in_object = false;
    int start_angle = 0;

    for (i = 0; i < NUM_SCAN_ANGLES; i++) {
        if (!in_object && ir_raw_values[i] > threshold) {
            in_object = true;
            start_angle = i * SCAN_RESOLUTION;
        } else if (in_object && ir_raw_values[i] <= threshold) {
            int end_angle = (i - 1) * SCAN_RESOLUTION;

            if (end_angle - start_angle >= MIN_OBJECT_ANGLE &&
                object_count < MAX_OBJECTS) {
                detected_objects[object_count].start_angle = start_angle;
                detected_objects[object_count].end_angle   = end_angle;
                detected_objects[object_count].mid_angle   = (start_angle + end_angle) / 2;
                detected_objects[object_count].distance    = 0.0;
                detected_objects[object_count].width       = 0.0;
                detected_objects[object_count].is_thin     = false;
                object_count++;
            }
            in_object = false;
        }
    }

    // Handle object that extends to 180 degrees
    if (in_object && object_count < MAX_OBJECTS) {
        int end_angle = 180;
        if (end_angle - start_angle >= MIN_OBJECT_ANGLE) {
            detected_objects[object_count].start_angle = start_angle;
            detected_objects[object_count].end_angle   = end_angle;
            detected_objects[object_count].mid_angle   = (start_angle + end_angle) / 2;
            detected_objects[object_count].distance    = 0.0;
            detected_objects[object_count].width       = 0.0;
            detected_objects[object_count].is_thin     = false;
            object_count++;
        }
    }

    sprintf(msg, "Found %d object(s)\r\n\r\n", object_count);
    uart_sendStr(msg);
}

/**
 * Point the PING sensor at each object's midpoint to measure distance,
 * then compute linear width via trigonometry.
 */
void calculate_object_widths(void) {
    uart_sendStr("Calculating object widths...\r\n");

    int i;
    for (i = 0; i < object_count; i++) {
        double readings[3];
        int j, k;

        for (j = 0; j < 3; j++) {
            cyBOT_Scan(detected_objects[i].mid_angle, &scan_data);
            readings[j] = scan_data.sound_dist;
            timer_waitMillis(40);
        }

        for (j = 0; j < 2; j++) {
            for (k = 0; k < 2 - j; k++) {
                if (readings[k] > readings[k + 1]) {
                    double tmp = readings[k];
                    readings[k] = readings[k + 1];
                    readings[k + 1] = tmp;
                }
            }
        }
      double spread = readings[2] - readings[0]; // rejects outliers 6:51pm
      double chosen;
      if (spread > readings[1] * 0.20) {
        double d_low  = readings[1] - readings[0];
        double d_high = readings[2] - readings[1];
        if (d_low < d_high) {
          chosen = readings[0];
        } else {
          chosen = readings[2];
        }
      } else {
        chosen = readings[1];
      }
      detected_objects[i].distance = chosen;

        detected_objects[i].distance = readings[1];
        if (detected_objects[i].distance <= 1.0 ||
            detected_objects[i].distance > MAX_VALID_PING_CM) {
            int mid_index = detected_objects[i].mid_angle / SCAN_RESOLUTION;
            if (mid_index >= 0 && mid_index < NUM_SCAN_ANGLES) {
                detected_objects[i].distance = ping_distances[mid_index];
            }
        }

        int angle_diff = detected_objects[i].end_angle - detected_objects[i].start_angle;
        detected_objects[i].width    = calculate_linear_width(angle_diff, detected_objects[i].distance);
    }

    uart_sendStr("Width calculation complete!\r\n\r\n");
}

/**
 * width = 2 * distance * tan(angle_diff / 2)
 */
double calculate_linear_width(int angle_diff, double distance) {
    double angle_rad = (angle_diff * M_PI) / 180.0;
    return 2.0 * distance * tan(angle_rad / 2.0);
}

double calculate_polar_separation(double dist_a, double angle_a,
                                  double dist_b, double angle_b) {
    double delta_rad = ((angle_a - angle_b) * M_PI) / 180.0;
    double d2 = dist_a * dist_a + dist_b * dist_b -
                2.0 * dist_a * dist_b * cos(delta_rad);
    if (d2 < 0.0) d2 = 0.0;
    return sqrt(d2);
}

void mission_abort_clear(void) {
    mission_aborted = false;
    complete = false;
}

void mission_abort_signal(void) {
    mission_aborted = true;
    complete = true;
    oi_setWheels(0, 0);
    uart_sendStr("\r\n>>> ABORT requested. Motors stopped. Returning to Ready prompt.\r\n");
    lcd_printf("ABORTED");
}

bool mission_abort_requested(void) {
    char command;

    if (mission_aborted) {
        return true;
    }

    if (uart_receive_nonblocking(&command)) {
        if (command == 'q' || command == 'Q') {
            mission_abort_signal();
            return true;
        }
    }

    return false;
}


//  Classification 

/**
 * Tag each object as thin (< THIN_PILLAR_MAX_WIDTH cm) or large.
 */
void classify_objects(void) {
    uart_sendStr("Classifying objects...\r\n");
    int i;
    for (i = 0; i < object_count; i++) {
        detected_objects[i].is_thin = (detected_objects[i].width < THIN_PILLAR_MAX_WIDTH);
    }
    uart_sendStr("Classification complete!\r\n\r\n");
}


//  Clustering 

/**
 * Group adjacent thin pillars into clusters.
 * Two thin pillars belong to the same cluster if the angular gap between
 * one's end_angle and the next's start_angle is <= CLUSTER_GAP_DEG.
 * Only clusters with >= MIN_CLUSTER_SIZE pillars are kept.
 */
void build_clusters(void) {
    cluster_count = 0;

    uart_sendStr("Building clusters from thin pillars...\r\n");

    // Work through objects in angular order (they are already sorted since
    // detect_objects_from_scan processes angles 0 -> 180).
    int i;
    Cluster current;
    current.count = 0;

    for (i = 0; i < object_count; i++) {
        if (!detected_objects[i].is_thin) continue {
        }// skip large pillars

        if (current.count == 0) {
            // Start a new cluster with this pillar
            if (current.count < MAX_OBJECTS) {
                current.pillar_indices[current.count++] = i;
            }
        } else {
            // Check gap from last pillar in current cluster to this pillar
            int last_idx = current.pillar_indices[current.count - 1];
            int gap = detected_objects[i].start_angle - detected_objects[last_idx].end_angle;

            if (gap <= CLUSTER_GAP_DEG) {
                // Same cluster
                if (current.count < MAX_OBJECTS) {
                    current.pillar_indices[current.count++] = i;
                }
            } else {
                // Save completed cluster if large enough
                if (current.count >= MIN_CLUSTER_SIZE && cluster_count < MAX_CLUSTERS) {
                    clusters[cluster_count++] = current;
                }
                // Start fresh cluster
                current.count = 0;
                current.pillar_indices[current.count++] = i;
            }
        }
    }

    // Don't forget the last in-progress cluster
    if (current.count >= MIN_CLUSTER_SIZE && cluster_count < MAX_CLUSTERS) {
        clusters[cluster_count++] = current;
    }

    // Compute centroid angle and average distance for each cluster
    int c;
    for (c = 0; c < cluster_count; c++) {
        int    angle_sum = 0;
        double dist_sum  = 0.0;
        int    j;
        for (j = 0; j < clusters[c].count; j++) {
            int idx = clusters[c].pillar_indices[j];
            angle_sum += detected_objects[idx].mid_angle;
            dist_sum  += detected_objects[idx].distance;
        }
        clusters[c].mid_angle    = angle_sum / clusters[c].count;
        clusters[c].avg_distance = dist_sum  / clusters[c].count;
    }

    char msg[60];
    sprintf(msg, "Found %d valid cluster(s) (>= %d thin pillars)\r\n\r\n",
            cluster_count, MIN_CLUSTER_SIZE);
    uart_sendStr(msg);
}

/**
 * Return index of the cluster with the most thin pillars.
 */
int find_largest_cluster(void) {
    int best_idx = -1;
    int best_count = 0;

    int i;
    for (i = 0; i < cluster_count; i++) {

        if (large_pillar_nearby(clusters[i].mid_angle, clusters[i].avg_distance))
            continue; // reject cluster

        if (clusters[i].count > best_count) {
            best_count = clusters[i].count;
            best_idx = i;
        }
    }

    if (best_idx == -1) {
        uart_sendStr(">>> No safe clusters (large pillar nearby all).\r\n");
        return -1;
    }

    char msg[100];
    sprintf(msg, "Largest VALID cluster: #%d (%d pillars)\r\n",
            best_idx + 1, clusters[best_idx].count);
    uart_sendStr(msg);

    return best_idx;
}


//  Sound Guard 

/**
 * Return true if any classified large pillar (a "diver") sits within the
 * blast radius of the target cluster in the most recent scan.
 */
bool large_pillar_nearby(double target_angle, double target_distance)
{
    int i;

    for (i = 0; i < object_count; i++)
    {
        if (!detected_objects[i].is_thin)
        {
            double diver_distance = detected_objects[i].distance;

            if (diver_distance <= 1.0 || diver_distance > MAX_VALID_PING_CM ||
                target_distance <= 1.0 || target_distance > MAX_VALID_PING_CM)
            {
                continue;
            }

            double separation = calculate_polar_separation(
                target_distance, target_angle,
                diver_distance, (double)detected_objects[i].mid_angle);

            if (separation <= LARGE_FISH_BLAST_RADIUS_CM)
            {
                char msg[100];
                sprintf(msg, ">>> Large fish %.1f cm from cluster. Rejecting.\r\n",
                        separation);
                uart_sendStr(msg);
                return true;
            }
        }
    }
    return false;
}


//  Sound 

/**
 * Play the pre-loaded detonation confirmation sound (song slot SOUND_SONG_NUM).
 */
void play_arrival_sound(void) {
    uart_sendStr(">>> Playing detonation sound!\r\n");
    oi_play_song(SOUND_SONG_NUM);
    // Wait long enough for the three notes to finish (~1.75 s)
    timer_waitMillis(1800);
}


//  Navigation 

/**
 * Turn to face the cluster centroid, drive to within TARGET_DISTANCE cm,
 * then ask the user before playing the detonation sound.
 */
void navigate_to_cluster(int cluster_index) {
    if (cluster_index < 0 || cluster_index >= cluster_count) {
        uart_sendStr(">>> ERROR: Invalid cluster index, aborting navigation.\r\n");
        return;
    }

    if (mission_abort_requested()) return;

    Cluster target = clusters[cluster_index];
    char msg[100];

    uart_sendStr(">>> Navigation started\r\n");

    // Step 1: Turn to face cluster centroid
    sprintf(msg, "Turning to face cluster centroid at %d degrees...\r\n", target.mid_angle);
    uart_sendStr(msg);

    int turn_angle = target.mid_angle - 90; // 90 = straight ahead
    if (turn_angle > 0) {
        turn_left(sensor_data, TURN_SPEED, turn_angle);
    } else if (turn_angle < 0) {
        turn_right(sensor_data, TURN_SPEED, -turn_angle);
    }
    if (mission_abort_requested()) return;

    // Step 2: Drive forward until within TARGET_DISTANCE cm
    sprintf(msg, "Approaching cluster (target stop distance: %d cm)...\r\n", TARGET_DISTANCE);
    uart_sendStr(msg);

    double distance_to_travel = target.avg_distance - TARGET_DISTANCE;
    double distance_traveled  = 0.0;

    while (distance_traveled < distance_to_travel) {
        if (mission_abort_requested()) return;

        oi_update(sensor_data);

        // Check for field boundary or hole lines BEFORE moving
        if (hazard_detected()) {
            uart_sendStr(">>> Line hazard detected! Escaping...\r\n");
            oi_setWheels(0, 0);
            escape_from_hazard();
            return;
        }

        bool bump_detected = sensor_data->bumpLeft || sensor_data->bumpRight;

        cyBOT_Scan(90, &scan_data);
        double dist_ahead = scan_data.sound_dist;

        if (dist_ahead <= TARGET_DISTANCE) {
            uart_sendStr(">>> Within target distance, stopping.\r\n");
            break;
        }

        if (dist_ahead < OBSTACLE_DISTANCE || bump_detected) {
            if (bump_detected) {
                uart_sendStr(">>> Bump! Initiating bypass...\r\n");
            } else {
                uart_sendStr(">>> Obstacle ahead! Initiating bypass...\r\n");
            }

            avoid_obstacle();
            if (mission_abort_requested()) return;

            // Re-scan and re-acquire after bypass
            uart_sendStr(">>> Re-acquiring largest cluster...\r\n");
            perform_full_scan();
            if (mission_abort_requested()) return;
            detect_objects_from_scan();
            calculate_object_widths();
            classify_objects();
            build_clusters();

            if (cluster_count > 0) {
                int new_idx = find_largest_cluster();
                if (new_idx == -1) {
                    uart_sendStr(">>> No safe clusters after bypass. Returning to roam loop.\r\n");
                    // Caller (run_roam_mission) will resume roaming
                    return;
                }
                target = clusters[new_idx];

                turn_angle = target.mid_angle - 90;
                if (turn_angle > 0)       turn_left(sensor_data, TURN_SPEED, turn_angle);
                else if (turn_angle < 0)  turn_right(sensor_data, TURN_SPEED, -turn_angle);

                distance_to_travel = target.avg_distance - TARGET_DISTANCE;
                distance_traveled  = 0.0;
                uart_sendStr(">>> Target re-locked. Resuming...\r\n");
            } else {
                uart_sendStr(">>> ERROR: Lost cluster after bypass.\r\n");
                return;
            }

        } else {
            // 5 cm steps so hazard check triggers before we cross a full line width
            move_forward(sensor_data, FORWARD_SPEED, 5);
            distance_traveled += 5.0;

            // Check for hazard immediately after each step
            if (hazard_detected()) {
                uart_sendStr(">>> Line hazard mid-move! Escaping...\r\n");
                oi_setWheels(0, 0);
                escape_from_hazard();
                return;
            }
        }
    }

    uart_sendStr(">>> Arrived at cluster!\r\n");

    perform_full_scan();
    if (mission_abort_requested()) return;
    detect_objects_from_scan();
    calculate_object_widths();
    classify_objects();
    build_clusters();

    cyBOT_Scan(90, &scan_data);
    sprintf(msg, "Final distance to cluster: %.1f cm\r\n", scan_data.sound_dist);
    uart_sendStr(msg);

    // Re-acquire cluster after arrival rescan; check for divers using
    // the freshly-detected cluster centroid (or 90 deg if none).
    int    arrival_idx      = find_largest_cluster();
    double check_angle      = (arrival_idx >= 0) ? (double)clusters[arrival_idx].mid_angle : 90.0;
    double check_distance   = (arrival_idx >= 0) ? clusters[arrival_idx].avg_distance : target.avg_distance;
    int    fish_count       = (arrival_idx >= 0) ? clusters[arrival_idx].count : target.count;

    if (arrival_idx >= 0 && !large_pillar_nearby(check_angle, check_distance)) {

        uart_sendStr("\r\n============================================\r\n");
        sprintf(msg, "CLUSTER FOUND: %d invasive fish (thin pillars)\r\n", fish_count);
        uart_sendStr(msg);
        uart_sendStr("No divers (large pillars) in blast radius.\r\n");
        uart_sendStr("Detonate? (y/n): ");

        char response = uart_receive();
        uart_sendChar(response);
        uart_sendStr("\r\n");

        if (response == 'y' || response == 'Y') {
            uart_sendStr(">>> DETONATION CONFIRMED! Area cleared.\r\n");
            play_arrival_sound();
            lcd_printf("DETONATED!");
            complete = true;
        } else if (response == 'q' || response == 'Q') {
            mission_abort_signal();
        } else {
            // User declined: back away just enough to clear the cluster, then
            // pivot so the next scan picks up a different region. Small backup
            // (10 cm) avoids hitting other pillars sitting behind us.
            uart_sendStr(">>> Detonation cancelled. Backing away to resume search...\r\n");
            if (!hazard_detected()) {
                move_backward(sensor_data, FORWARD_SPEED, 10);
            }
            turn_right(sensor_data, TURN_SPEED, 120);
        }
    } else {
        uart_sendStr(">>> WARNING: Diver (large pillar) detected nearby!\r\n");
        uart_sendStr(">>> Cannot detonate safely. Backing away...\r\n");
        if (!hazard_detected()) {
            move_backward(sensor_data, FORWARD_SPEED, 10);
        }
        turn_right(sensor_data, TURN_SPEED, 120);
    }

    uart_sendStr("\r\n");
}


//  Obstacle Avoidance 

// Gentle obstacle avoidance.
//   - If we got bumped, back off by just 4 cm (enough to break contact, but
//     small enough that we won't slam into anything sitting behind us).
//   - Sweep PING from 15 to 165 deg in 15 deg increments.
//   - Pick the open angle with the best clearance, with a small bias toward
//     90 deg so similar paths do not cause unnecessary hard turns.
//   - Turn toward that angle, then verify the new forward path before inching
//     forward a short distance.
void avoid_obstacle(void) {
    if (mission_abort_requested()) return;

    oi_update(sensor_data);
    bool was_bumped = sensor_data->bumpLeft || sensor_data->bumpRight;

    if (was_bumped) {
        // Just enough to release the contact. Larger backups risk hitting
        // pillars right behind the bot.
        move_backward(sensor_data, FORWARD_SPEED, 4);
        if (hazard_detected()) { escape_from_hazard(); return; }
        if (mission_abort_requested()) return;
    }

    int best_angle = 90;
    double best_clearance = 0.0;
    double best_score = -9999.0;
    bool found_clear_path = false;

    int angle;
    for (angle = AVOID_SCAN_START_DEG; angle <= AVOID_SCAN_END_DEG; angle += AVOID_SCAN_STEP_DEG) {
        if (mission_abort_requested()) return;

        cyBOT_Scan(angle, &scan_data);
        double clearance = scan_data.sound_dist;

        if (clearance > MIN_BYPASS_CLEARANCE) {
            int offset = angle - 90;
            if (offset < 0) offset = -offset;

            double score = clearance - ((double)offset * AVOID_FORWARD_BIAS);
            if (!found_clear_path || score > best_score) {
                found_clear_path = true;
                best_score = score;
                best_clearance = clearance;
                best_angle = angle;
            }
        }
    }

    if (!found_clear_path) {
        uart_sendStr("No bypass clearance found — pivoting in place.\r\n");
        turn_right(sensor_data, TURN_SPEED, 60);
        return;
    }

    char msg[96];
    sprintf(msg, "Best bypass angle: %d deg (PING %.1f cm)\r\n",
            best_angle, best_clearance);
    uart_sendStr(msg);

    int turn_angle = best_angle - 90;
    if (turn_angle > 0) {
        turn_left(sensor_data, TURN_SPEED, turn_angle);
    } else if (turn_angle < 0) {
        turn_right(sensor_data, TURN_SPEED, -turn_angle);
    }

    // Verify forward path is clear after the pivot before committing.
    cyBOT_Scan(90, &scan_data);
    if (scan_data.sound_dist > BYPASS_FORWARD_CLEARANCE && !hazard_detected()) {
        int bumped = move_forward(sensor_data, FORWARD_SPEED, 6);
        if (bumped) {
            move_backward(sensor_data, FORWARD_SPEED, 4);
        }
    }
    if (hazard_detected()) { escape_from_hazard(); return; }
    if (mission_abort_requested()) return;
}


//  Display 

/**
 * Print the raw object list with width and thin/large classification.
 */
void display_object_info(void) {
    uart_sendStr("\r\n--- DETECTED OBJECTS ---\r\n");
    uart_sendStr("  #  | Start | End  | Mid  | PING (cm) | IR (cm) | Width (cm) | Class\r\n");
    uart_sendStr("-----|-------|------|------|-----------|---------|------------|-------\r\n");

    int i;
    for (i = 0; i < object_count; i++) {
        int mid_idx = detected_objects[i].mid_angle / SCAN_RESOLUTION;
        double ir_dist = (mid_idx < NUM_SCAN_ANGLES) ? ir_distances[mid_idx] : 0.0;
        char line[140];
        sprintf(line, "  %2d |  %3d  | %3d  | %3d  |  %6.1f   | %6.1f  |  %6.1f    | %s\r\n",
                i + 1,
                detected_objects[i].start_angle,
                detected_objects[i].end_angle,
                detected_objects[i].mid_angle,
                detected_objects[i].distance,
                ir_dist,
                detected_objects[i].width,
                detected_objects[i].is_thin ? "THIN (fish)" : "LARGE (diver)");
        uart_sendStr(line);
    }
    uart_sendStr("-----|-------|------|------|-----------|---------|------------|-------\r\n\r\n");
}

/**
 * Print the cluster summary table.
 */
void display_cluster_info(void) {
    if (cluster_count == 0) {
        uart_sendStr(">>> No valid thin-pillar clusters (need >= 3 adjacent thin pillars).\r\n\r\n");
        return;
    }

    uart_sendStr("--- THIN-PILLAR CLUSTERS ---\r\n");
    uart_sendStr("  #  | Pillars | Centroid (deg) | Avg Dist (cm)\r\n");
    uart_sendStr("-----|---------|----------------|---------------\r\n");

    int c;
    for (c = 0; c < cluster_count; c++) {
        char line[100];
        sprintf(line, "  %2d |    %2d   |      %3d       |    %6.1f\r\n",
                c + 1,
                clusters[c].count,
                clusters[c].mid_angle,
                clusters[c].avg_distance);
        uart_sendStr(line);
    }
    uart_sendStr("-----|---------|----------------|---------------\r\n\r\n");

    int best = find_largest_cluster();
    if (best >= 0) {
        char msg[80];
        sprintf(msg, ">>> TARGET: Cluster #%d has the most thin pillars (%d)\r\n\r\n",
                best + 1, clusters[best].count);
        uart_sendStr(msg);
    }
}


//  Manual Control 

void print_imu_status(void) {
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

void manual_control(char command) {
    switch (command) {
        case 'w': case 'W':
            uart_sendStr(">>> Moving forward\r\n");
            move_forward(sensor_data, FORWARD_SPEED, 10);
            break;
        case 's': case 'S':
            uart_sendStr(">>> Moving backward\r\n");
            move_backward(sensor_data, FORWARD_SPEED, 10);
            break;
        case 'a': case 'A':
            uart_sendStr(">>> Turning left\r\n");
            turn_left(sensor_data, TURN_SPEED, 15);
            break;
        case 'd': case 'D':
            uart_sendStr(">>> Turning right\r\n");
            turn_right(sensor_data, TURN_SPEED, 15);
            break;
        case 'm': case 'M':
            uart_sendStr(">>> Manual scan\r\n");
            perform_full_scan();
            detect_objects_from_scan();
            calculate_object_widths();
            classify_objects();
            build_clusters();
            display_object_info();
            display_cluster_info();
            break;
        case 'h': case 'H':
            print_imu_status();
            break;
    }

    cyBOT_Scan(90, &scan_data);
    char msg[50];
    sprintf(msg, "Distance ahead: %.1f cm\r\n", scan_data.sound_dist);
    uart_sendStr(msg);
}

// (Roam logic moved to roam_mission.c — invoked via run_roam_mission())

// LINE / HAZARD DETECTION
// iRobot cliff IR sensors: HIGH value = bright/white surface, LOW value = dark/black surface.
//
// IMPORTANT: only the FRONT cliff sensors (cliffFrontLeftSignal / cliffFrontRightSignal)
// are used. The side sensors (cliffLeftSignal / cliffRightSignal) sit further back and
// drift outside the typical "floor" range on many lab tiles, causing constant false
// hazard triggers as soon as the bot is set down.
//
// A single noisy reading is also ignored: the trip condition must persist for
// CLIFF_DEBOUNCE_MS milliseconds before we declare a hazard.
//
// Use the 'c' command on the UART menu to print live cliff values for calibration.

static inline bool _front_cliff_tripped(void) {
    int fl = sensor_data->cliffFrontLeftSignal;
    int fr = sensor_data->cliffFrontRightSignal;

    bool black = (fl < CLIFF_BLACK_THRESHOLD || fr < CLIFF_BLACK_THRESHOLD);
#if CLIFF_DETECT_WHITE
    bool white  = (fl > CLIFF_WHITE_THRESHOLD ||
                   fr > CLIFF_WHITE_THRESHOLD);
    return black || white;
#else
    return black;
#endif
}

bool hazard_detected(void) {
    oi_update(sensor_data);
    if (!_front_cliff_tripped()) return false;

    // Debounce: require the trip to persist for CLIFF_DEBOUNCE_MS
    timer_waitMillis(CLIFF_DEBOUNCE_MS);
    oi_update(sensor_data);
    return _front_cliff_tripped();
}

// Escape: small backup just to clear the line, then turn parallel-ish to it
// (90 deg) so the bot resumes driving along the boundary instead of
// driving straight back into it. Smaller backup distance helps avoid hitting
// objects behind the bot during avoidance.
void escape_from_hazard(void) {
    // Read current values (hazard_detected already called oi_update)
    int fl = sensor_data->cliffFrontLeftSignal;
    int fr = sensor_data->cliffFrontRightSignal;

#if CLIFF_DETECT_WHITE
    bool left_triggered  = (fl > CLIFF_WHITE_THRESHOLD ||
                            fl < CLIFF_BLACK_THRESHOLD);
    bool right_triggered = (fr > CLIFF_WHITE_THRESHOLD ||
                            fr < CLIFF_BLACK_THRESHOLD);
#else
    bool left_triggered  = (fl < CLIFF_BLACK_THRESHOLD);
    bool right_triggered = (fr < CLIFF_BLACK_THRESHOLD);
#endif

    // Small backup — only enough to fully clear the front sensor off the line.
    // Larger backups risk hitting objects behind the bot.
    move_backward(sensor_data, FORWARD_SPEED, 8);
     oi_update(sensor_data); // to check if we run into another hazard // 6:08pm //pushed to the CCS project
    if (left_triggered && !right_triggered) {
        // Hazard on the left — turn right ~90° to run parallel to the line
        turn_right(sensor_data, TURN_SPEED, 90);
    } else if (right_triggered && !left_triggered) {
        // Hazard on the right — turn left ~90° to run parallel to the line
        turn_left(sensor_data, TURN_SPEED, 90);
    } else {
        // Both sides or front-only — pivot away in a hopefully-clear direction
        turn_right(sensor_data, TURN_SPEED, 135);
    }
}

double ir_raw_to_cm(int raw_adc) {
    if (raw_adc < 50) return 999.0;
    return 8668330.42 * pow((double)raw_adc, -1.73845);
}
