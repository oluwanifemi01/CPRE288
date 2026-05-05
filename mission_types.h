#ifndef MISSION_TYPES_H_
#define MISSION_TYPES_H_

#include <stdbool.h>

/* Shared mission-data structs used by main.c and patrol_mission.c.
 * Keep this header tiny — it exists only so the two translation units
 * agree on the in-memory layout of the global object/cluster arrays. */

typedef struct {
    int    start_angle;
    int    end_angle;
    int    mid_angle;
    double distance;
    double width;
    bool   is_thin;     /* true if width < THIN_PILLAR_MAX_WIDTH */
} Object;

typedef struct {
    int    pillar_indices[20]; /* MAX_OBJECTS — keep in sync with main.c */
    int    count;
    int    mid_angle;
    double avg_distance;
} Cluster;

#endif /* MISSION_TYPES_H_ */
