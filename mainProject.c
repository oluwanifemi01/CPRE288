
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
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

//  Configuration Constants 
#define SCAN_RESOLUTION         1       // Degrees per scan step
#define IR_SAMPLES              5       // IR readings to average per angle
#define MAX_OBJECTS             20      // Maximum individual objects to track
#define MAX_CLUSTERS            10      // Maximum clusters to track
#define TURN_SPEED              25      // Speed for turning
#define FORWARD_SPEED           100     // Speed for forward movement
#define TARGET_DISTANCE         50      // Stop distance from cluster (cm) - 0.5 m
#define OBSTACLE_DISTANCE       25      // Distance to trigger obstacle avoidance (cm)
#define NUM_SCAN_ANGLES         ((180 / SCAN_RESOLUTION) + 1)

//  Object Classification Thresholds 
#define IR_EDGE_THRESHOLD      200   // lower than before (more sensitive)
#define PING_OBJECT_MAX_DIST   100   // cm (anything closer is "real")
#define PING_CONFIRM_SAMPLES   2     // number of confirmations
#define MIN_OBJECT_ANGLE       4
#define THIN_PILLAR_MAX_WIDTH   15.0    // Max width (cm) for a "thin" pillar
#define MIN_CLUSTER_SIZE        3       // Minimum pillars to qualify as a cluster

//  Cluster Gap Tolerance 
// Adjacent thin pillars separated by <= this many degrees are treated as one cluster
#define CLUSTER_GAP_DEG         30

//  Sound Configuration (Open Interface Song) 
// Song 0: three-note ascending chime played when target is reached
#define SOUND_SONG_NUM          0

//  Structs 

typedef struct {
    int    start_angle;
    int    end_angle;
    int    mid_angle;
    double distance;
    double width;
    bool   is_thin;     // true if width < THIN_PILLAR_MAX_WIDTH
} Object;

typedef struct {
    int    pillar_indices[MAX_OBJECTS]; // indices into detected_objects[]
    int    count;                       // number of thin pillars in this cluster
    int    mid_angle;                   // centroid angle of the cluster
    double avg_distance;                // average distance to pillars in cluster
} Cluster;

//  Globals 

Object   detected_objects[MAX_OBJECTS];
int      object_count = 0;

Cluster  clusters[MAX_CLUSTERS];
int      cluster_count = 0;

bool         autonomous_mode = true;
cyBOT_Scan_t scan_data;
oi_t        *sensor_data;
bool complete = false;

int   ir_raw_values[NUM_SCAN_ANGLES];
float ping_distances[NUM_SCAN_ANGLES];

//  Function Prototypes 

void perform_full_scan(void);
void detect_objects_from_scan(void);
void calculate_object_widths(void);
void classify_objects(void);
void build_clusters(void);
int  find_largest_cluster(void);
bool large_pillar_nearby(double center_dist_cm);
void play_arrival_sound(void);
void navigate_to_cluster(int cluster_index);
void avoid_obstacle(void);
void manual_control(char command);
void display_object_info(void);
void display_cluster_info(void);
double calculate_linear_width(int angle_diff, double distance);
void roam(void);
bool boundary_detected(void);


//  Main 

int main(void) {
    timer_init();
    lcd_init();
    uart_init();
    cyBOT_init_Scan(0b0111);

    // *** Replace with your bot's calibration values ***
    right_calibration_value = 274750;
    left_calibration_value  = 1303750;

    sensor_data = oi_alloc();
    oi_init(sensor_data);

    // Load arrival chime into song slot 0: C5, E5, G5 (quarter notes)
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
    uart_sendStr("Commands:\r\n");
    uart_sendStr("  's' - Start autonomous mission\r\n");
    uart_sendStr("  'g' - Go to largest thin-pillar cluster\r\n");
    uart_sendStr("  't' - Toggle autonomous/manual mode\r\n");
    uart_sendStr("  'w/a/s/d' - Manual movement (manual mode)\r\n");
    uart_sendStr("  'm' - Manual scan (manual mode)\r\n");
    uart_sendStr("============================================\r\n\r\n");

    char command;
    int  target_cluster_index;

    while (1) {
        uart_sendStr("Ready> ");
        command = uart_receive();
        uart_sendChar(command);
        uart_sendStr("\r\n");

        switch (command) {

            case 's': case 'S':
                
                uart_sendStr("\r\n>>> Starting autonomous mission...\r\n");

                while (!complete) {
                    perform_full_scan();
                    detect_objects_from_scan();
                    calculate_object_widths();
                    classify_objects();
                    build_clusters();

                    display_object_info();
                    display_cluster_info();

                    if (object_count == 0 || cluster_count == 0)
                    {
                        uart_sendStr("\r\n>>> No valid clusters. Roaming...\r\n");
                        roam();
                        continue;
                    }

                    target_cluster_index = find_largest_cluster();

                    if (target_cluster_index == -1) {
                        roam();
                        continue;
                    }

                    navigate_to_cluster(target_cluster_index);
                }

                uart_sendStr("\r\n>>> Mission COMPLETE. Stopping.\r\n");
                break;

            case 'g': case 'G':
                if (object_count == 0) {
                    uart_sendStr("\r\n>>> ERROR: No objects detected. Run scan first.\r\n");
                } else if (cluster_count == 0) {
                    uart_sendStr("\r\n>>> No valid thin-pillar clusters found (need >= 3 thin pillars).\r\n");
                } else {
                    target_cluster_index = find_largest_cluster();
                    uart_sendStr("\r\n>>> Navigating to largest thin-pillar cluster...\r\n");
                    navigate_to_cluster(target_cluster_index);
                }
                break;

            case 't': case 'T':
                autonomous_mode = !autonomous_mode;
                uart_sendStr(autonomous_mode ? "\r\n>>> Mode: AUTONOMOUS\r\n"
                                             : "\r\n>>> Mode: MANUAL\r\nUse w/a/s/d, 'm' for scan\r\n");
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

    int index = 0;
    int angle;
    for ( angle = 0; angle <= 180; angle += SCAN_RESOLUTION) {

        long ir_sum = 0;
        double ping_sum = 0;
        int i;
        for (i = 0; i < IR_SAMPLES; i++) {
            cyBOT_Scan(angle, &scan_data);

            ir_sum   += scan_data.IR_raw_val;
            ping_sum += scan_data.sound_dist;

            timer_waitMillis(5); // allow sensor settle
        }

        ir_raw_values[index]  = ir_sum / IR_SAMPLES;
        ping_distances[index] = ping_sum / IR_SAMPLES;

        // Clamp bad PING values
        if (ping_distances[index] < 5)   ping_distances[index] = 5;
        if (ping_distances[index] > 200) ping_distances[index] = 200;

        index++;
    }

    uart_sendStr("Scan complete!\r\n\r\n");
}


//  Detection 

/**
 * Detect object edges from stored IR data.
 * A positive jump > IR_OBJECT_RAW_THRESHOLD marks a leading edge;
 * a negative jump marks a trailing edge.
 */
void detect_objects_from_scan(void) {
    object_count = 0;

    bool in_object = false;
    int start_index = 0;

    uart_sendStr("Detecting objects (FIXED IR + PING)...\r\n");
    int i;
    for ( i = 1; i < NUM_SCAN_ANGLES; i++) {

        int ir_diff = ir_raw_values[i] - ir_raw_values[i - 1];

        // --- ENTER OBJECT (IR ONLY) ---
        if (!in_object && ir_diff > IR_EDGE_THRESHOLD) {
            in_object = true;
            start_index = i;
        }

        // --- EXIT OBJECT (IR ONLY) ---
        else if (in_object && ir_diff < -IR_EDGE_THRESHOLD) {

            int end_index = i - 1;

            // --- NOW validate using PING over region ---
            int valid_count = 0;
            int total = 0;
            int j;
            for (j = start_index; j <= end_index; j++) {
                if (ping_distances[j] < PING_OBJECT_MAX_DIST) {
                    valid_count++;
                }
                total++;
            }

            // Require at least 30% of points to be "close"
            if (total > 0 && (valid_count > total * 0.3)) {

                int start_angle = start_index * SCAN_RESOLUTION;
                int end_angle   = end_index   * SCAN_RESOLUTION;

                if ((end_angle - start_angle) >= MIN_OBJECT_ANGLE &&
                    object_count < MAX_OBJECTS)
                {
                    detected_objects[object_count].start_angle = start_angle;
                    detected_objects[object_count].end_angle   = end_angle;
                    detected_objects[object_count].mid_angle   =
                        (start_angle + end_angle) / 2;
                    detected_objects[object_count].is_thin = false;

                    object_count++;
                }
            }

            in_object = false;
        }
    }

    // Handle object reaching 180°
    if (in_object && object_count < MAX_OBJECTS) {
        int start_angle = start_index * SCAN_RESOLUTION;
        int end_angle = 180;

        detected_objects[object_count].start_angle = start_angle;
        detected_objects[object_count].end_angle   = end_angle;
        detected_objects[object_count].mid_angle   =
            (start_angle + end_angle) / 2;
        detected_objects[object_count].is_thin = false;

        object_count++;
    }

    char msg[50];
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

        int mid   = detected_objects[i].mid_angle;
        int left  = detected_objects[i].start_angle;
        int right = detected_objects[i].end_angle;

        double mid_dist = 0;
        double left_dist = 0;
        double right_dist = 0;
        int j;

        // --- MIDPOINT DISTANCE (averaged) ---
        for (j = 0; j < 3; j++) {
            cyBOT_Scan(mid, &scan_data);
            timer_waitMillis(40);
            mid_dist += scan_data.sound_dist;
        }
        mid_dist /= 3.0;

        // Clamp
        if (mid_dist < 5)   mid_dist = 5;
        if (mid_dist > 200) mid_dist = 200;

        // --- EDGE DISTANCES ---
        cyBOT_Scan(left, &scan_data);
        left_dist = scan_data.sound_dist;

        cyBOT_Scan(right, &scan_data);
        right_dist = scan_data.sound_dist;

        // Clamp edges
        if (left_dist < 5)   left_dist = mid_dist;
        if (right_dist < 5)  right_dist = mid_dist;

        // --- FINAL DISTANCE ---
        double avg_dist = (mid_dist + left_dist + right_dist) / 3.0;

        detected_objects[i].distance = avg_dist;

        // --- WIDTH CALC ---
        int angle_diff = right - left;

        detected_objects[i].width =
            calculate_linear_width(angle_diff, avg_dist);
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
    char msg[120];

    for (i = 0; i < object_count; i++) {
        if (!detected_objects[i].is_thin) continue; // skip large pillars

        if (current.count == 0) {
            // Start a new cluster with this pillar
            current.pillar_indices[current.count++] = i;
        } else {
            // Check gap from last pillar in current cluster to this pillar
            int last_idx = current.pillar_indices[current.count - 1];
            int gap = detected_objects[i].start_angle - detected_objects[last_idx].end_angle;

            if (gap <= CLUSTER_GAP_DEG) {
                // Same cluster
                current.pillar_indices[current.count++] = i;
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

    char msg[100];
    int i;
    for (i = 0; i < cluster_count; i++) {

        if (large_pillar_nearby(clusters[i].mid_angle))
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

    sprintf(msg, "Largest VALID cluster: #%d (%d pillars)\r\n",
            best_idx + 1, clusters[best_idx].count);
    uart_sendStr(msg);

    return best_idx;
}


//  Sound Guard 

/**
 * Scan from 60 - 120 (frontal cone) and return true if any large pillar
 * (>= THIN_PILLAR_MAX_WIDTH cm wide) is detected within TARGET_DISTANCE + 20 cm.
 * Used to suppress the sound when a large obstacle is present.
 */
bool large_pillar_nearby(double target_angle)
{
    int i;

    for (i = 0; i < object_count; i++)
    {
        if (!detected_objects[i].is_thin)
        {
            int diff = abs(detected_objects[i].mid_angle - target_angle);

            if (diff < 20) // adjustable
            {
                uart_sendStr(">>> Large pillar near cluster. Rejecting.\r\n");
                return true;
            }
        }
    }
    return false;
}


//  Sound 

/**
 * Play the pre-loaded arrival chime (song slot SOUND_SONG_NUM).
 */
void play_arrival_sound(void) {
    uart_sendStr(">>> Playing arrival chime!\r\n");
    oi_play_song(SOUND_SONG_NUM);
    // Wait long enough for the three notes to finish (~1.75 s)
    timer_waitMillis(1800);
}


//  Navigation 

/**
 * Turn to face the cluster centroid, drive to within TARGET_DISTANCE cm,
 * then play the arrival sound unless a large pillar is nearby.
 */
void navigate_to_cluster(int cluster_index) {
    Cluster target = clusters[cluster_index];
    char msg[100];

    uart_sendStr(">>> Navigation started\r\n");

    // Step 1: Turn to face cluster centroid
   // uart_printf("Turning to face cluster centroid at %d degrees...\r\n", target.mid_angle);

    int turn_angle = target.mid_angle - 90; // 90 = straight ahead
    if (turn_angle > 0) {
        turn_left(sensor_data, TURN_SPEED, turn_angle);
    } else if (turn_angle < 0) {
        turn_right(sensor_data, TURN_SPEED, -turn_angle);
    }

    // Step 2: Drive forward until within TARGET_DISTANCE cm
    sprintf(msg,"Approaching cluster (target stop distance: %d cm)...\r\n", TARGET_DISTANCE);

    double distance_to_travel = target.avg_distance - TARGET_DISTANCE;
    double distance_traveled  = 0.0;

    while (distance_traveled < distance_to_travel) {
        oi_update(sensor_data);
        if (boundary_detected()) {
            uart_sendStr(">>> Boundary detected! Escaping...\r\n");
            oi_setWheels(0,0);
            move_backward(sensor_data, FORWARD_SPEED, 100);
            turn_left(sensor_data, TURN_SPEED, 90);
            return;
        }

        if (hole_detected()) {
            uart_sendStr(">>> Hole during roam!\r\n");
            oi_setWheels(0,0);
            move_backward(sensor_data, FORWARD_SPEED, 50);
            turn_left(sensor_data, TURN_SPEED, 90);
            move_forward(sensor_data, FORWARD_SPEED, 100);
            turn_right(sensor_data, TURN_SPEED, 90);
            return;
        }


        bool bump_detected_left = sensor_data->bumpLeft;
        bool bump_detected_right = sensor_data->bumpRight;

        cyBOT_Scan(90, &scan_data);
        double dist_ahead = scan_data.sound_dist;

        if (dist_ahead <= TARGET_DISTANCE) {
            // Close enough stop loop early
            uart_sendStr(">>> Within target distance, stopping.\r\n");
            break;
        }

        if (bump_detected_left) {
            uart_sendStr(">>> Left Bump detected! Initiating bypass...\r\n");
            oi_setWheels(0, 0);
            move_backward(sensor_data, FORWARD_SPEED, 100);
            turn_right(sensor_data, TURN_SPEED, 90);
            move_forward(sensor_data, FORWARD_SPEED, 100);
            turn_left(sensor_data, TURN_SPEED, 90);
            return;
        }

        else if (bump_detected_right) {
            uart_sendStr(">>> Right bump detected! Initiating bypass...\r\n");
            oi_setWheels(0, 0);
            move_backward(sensor_data, FORWARD_SPEED, 100);
            turn_left(sensor_data, TURN_SPEED, 90);
            move_forward(sensor_data, FORWARD_SPEED, 100);
            turn_right(sensor_data, TURN_SPEED, 90);
            return;
        }

        if (dist_ahead < OBSTACLE_DISTANCE || boundary_detected()) {
            

            if (boundary_detected()) {
                uart_sendStr(">>> Boundary! Initiating bypass...\r\n");
                oi_setWheels(0, 0);
                move_backward(sensor_data, FORWARD_SPEED, 100);
                turn_left(sensor_data,TURN_SPEED, 90);
                return;
            }
            else {
                uart_sendStr(">>> Obstacle ahead! Initiating bypass...\r\n");
            }

            avoid_obstacle();

            // Re-scan and re-acquire
            uart_sendStr(">>> Re-acquiring largest cluster...\r\n");
            perform_full_scan();
            detect_objects_from_scan();
            calculate_object_widths();
            classify_objects();
            build_clusters();

            if (cluster_count > 0) {
                int new_idx = find_largest_cluster();
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
            move_forward(sensor_data, FORWARD_SPEED, 10);
            distance_traveled += 10.0;
        }
    }

    uart_sendStr(">>> Arrived at cluster!\r\n");
    cyBOT_Scan(90, &scan_data);
   // uart_sendStr("Final distance to cluster: %.1f cm\r\n", scan_data.sound_dist);

    // Step 3: Play sound - only if no large pillar is within range
    if (!large_pillar_nearby(scan_data.sound_dist)) {
        play_arrival_sound();
        complete = true;
    }

    uart_sendStr(">>> Mission complete!\r\n\r\n");
}


//  Obstacle Avoidance 

void avoid_obstacle(void) {
    move_backward(sensor_data, FORWARD_SPEED, 15);

    cyBOT_Scan(45, &scan_data);
    double right_clearance = scan_data.sound_dist;

    cyBOT_Scan(135, &scan_data);
    double left_clearance = scan_data.sound_dist;

    if (left_clearance > right_clearance) {
        uart_sendStr("Bypassing to the left...\r\n");
        turn_left(sensor_data, TURN_SPEED, 90);
        move_forward(sensor_data, FORWARD_SPEED, 30);
        turn_right(sensor_data, TURN_SPEED, 90);
    } else {
        uart_sendStr("Bypassing to the right...\r\n");
        turn_right(sensor_data, TURN_SPEED, 90);
        move_forward(sensor_data, FORWARD_SPEED, 30);
        turn_left(sensor_data, TURN_SPEED, 90);
    }
}


//  Display 

/**
 * Print the raw object list with width and thin/large classification.
 */
void display_object_info(void) {
    uart_sendStr("\r\n--- DETECTED OBJECTS ---\r\n");
    uart_sendStr("  #  | Start | End  | Mid  | Dist (cm) | Width (cm) | Class\r\n");
    uart_sendStr("-----|-------|------|------|-----------|------------|-------\r\n");

    int i;
    char line[150];
    for (i = 0; i < object_count; i++) {
        sprintf(line, "  %2d |  %3d  | %3d  | %3d  |  %6.1f   |  %6.1f    | %s\r\n",
                i + 1,
                detected_objects[i].start_angle,
                detected_objects[i].end_angle,
                detected_objects[i].mid_angle,
                detected_objects[i].distance,
                detected_objects[i].width,
                detected_objects[i].is_thin ? "THIN" : "LARGE");
        uart_sendStr(line);
    }
    uart_sendStr("-----|-------|------|------|-----------|------------|-------\r\n\r\n");
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
    char line[120];
    for (c = 0; c < cluster_count; c++) {
        sprintf(line, "  %2d |    %2d   |      %3d       |    %6.1f\r\n",
                c + 1,
                clusters[c].count,
                clusters[c].mid_angle,
                clusters[c].avg_distance);
        uart_sendStr(line);
    }
    uart_sendStr("-----|---------|----------------|---------------\r\n\r\n");

    int best = find_largest_cluster();
   // uart_sendStr(">>> TARGET: Cluster #%d has the most thin pillars (%d)\r\n\r\n",
          //  best + 1, clusters[best].count);
}


//  Manual Control 

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
    }

    cyBOT_Scan(90, &scan_data);
    //uart_sendStr("Distance ahead: %.1f cm\r\n", scan_data.sound_dist);
}

// ROAM
void roam(void) {
    uart_sendStr(">>> Roaming...\r\n");

    int i;
    for (i = 0; i < 5; i++) {
        if (boundary_detected()) {
            uart_sendStr(">>> Boundary during roam!\r\n");
            move_backward(sensor_data, FORWARD_SPEED, 50);
            turn_left(sensor_data, TURN_SPEED, 90);
            return;
        }

        if (hole_detected()) {
            uart_sendStr(">>> Hole during roam!\r\n");
            move_backward(sensor_data, FORWARD_SPEED, 50);
            turn_left(sensor_data, TURN_SPEED, 90);
            return;
        }


        move_forward(sensor_data, FORWARD_SPEED, 10);
    }

    int angle = rand() % 90 + 45;
    turn_left(sensor_data, TURN_SPEED, angle);
}

#define CLIFF_SENSOR_THRESHOLD_BLACK 250
#define CLIFF_SENSOR_THRESHOLD_WHITE 2700

// BOUNDARY 
bool boundary_detected(void) {
    oi_update(sensor_data);

    return (
        sensor_data->cliffFrontLeftSignal > CLIFF_SENSOR_THRESHOLD_WHITE ||
        sensor_data->cliffFrontRightSignal > CLIFF_SENSOR_THRESHOLD_WHITE ||
        sensor_data->cliffLeftSignal > CLIFF_SENSOR_THRESHOLD_WHITE ||
        sensor_data->cliffRightSignal > CLIFF_SENSOR_THRESHOLD_WHITE
    );
}

bool hole_detected(void) {
    oi_update(sensor_data);

    return (sensor_data->cliffFrontLeftSignal < CLIFF_SENSOR_THRESHOLD_BLACK ||
        sensor_data->cliffFrontRightSignal < CLIFF_SENSOR_THRESHOLD_BLACK ||
        sensor_data->cliffLeftSignal < CLIFF_SENSOR_THRESHOLD_BLACK ||
        sensor_data->cliffRightSignal < CLIFF_SENSOR_THRESHOLD_BLACK
    );
}
