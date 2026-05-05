#ifndef ROAM_MISSION_MANUAL_H_
#define ROAM_MISSION_MANUAL_H_

/*
 * Manual field-roam mode.
 *
 * No autonomous navigation is performed. The driver controls movement with
 * w/a/s/d, can scan the field with m, and forward movement is blocked by the
 * front cliff sensors when black/white tape is detected.
 */
void run_roam_mission_manual(void);

#endif /* ROAM_MISSION_MANUAL_H_ */
