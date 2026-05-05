#ifndef ROAM_MISSION_H_
#define ROAM_MISSION_H_

/*
 * Roam-and-detonate mission.
 *
 * Drives forward continuously using a fast IR trip-wire as the front
 * collision sensor. When the trip-wire fires, confirms with PING and
 * runs a full 180° scan — if a thin-pillar cluster is found, navigates
 * to it and prompts the user to detonate.
 *
 * Replaces the old scan-every-iteration behaviour from main.c's case 's'.
 */
void run_roam_mission(void);

#endif /* ROAM_MISSION_H_ */
