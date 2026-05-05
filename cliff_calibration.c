/*
 * cliff_calibration.c
 *
 * Floor-relative cliff baseline capture + UART diagnostic tool.
 *
 * Floor baseline capture + UART diagnostic tool for cliff sensors.
 * Current hardware calibration trips white tape above 2900 and black tape
 * below 200. Baseline values are still printed so floor drift is visible.
 */

#include "cliff_calibration.h"
#include "Timer.h"
#include "uart.h"
#include <stdio.h>
#include <stdbool.h>

// Module-static baseline values (filled by cliff_baseline_capture)
static int s_baseline_fl = 0;
static int s_baseline_fr = 0;

// These match main.c's #defines; kept private so calibration is self-contained.
#define LOCAL_WHITE_THRESHOLD   2900 // signal > this  ⇒ white tape
#define LOCAL_BLACK_THRESHOLD   200  // signal < this  ⇒ black tape

void cliff_baseline_capture(oi_t *sd, int samples) {
    if (sd == NULL || samples <= 0) return;

    long sum_fl = 0;
    long sum_fr = 0;
    int  n;
    for (n = 0; n < samples; n++) {
        oi_update(sd);
        sum_fl += sd->cliffFrontLeftSignal;
        sum_fr += sd->cliffFrontRightSignal;
        timer_waitMillis(20);
    }

    s_baseline_fl = (int)(sum_fl / samples);
    s_baseline_fr = (int)(sum_fr / samples);

    char msg[96];
    sprintf(msg, ">>> Floor baseline captured: FL=%d  FR=%d\r\n",
            s_baseline_fl, s_baseline_fr);
    uart_sendStr(msg);
}

int cliff_baseline_fl(void) { return s_baseline_fl; }
int cliff_baseline_fr(void) { return s_baseline_fr; }

void cliff_calibration_print(oi_t *sd, int samples) {
    if (sd == NULL || samples <= 0) return;

    uart_sendStr("\r\n>>> Cliff sensor live readout\r\n");
    uart_sendStr("    FL    FR    SL    SR    DELTA_FL DELTA_FR  HAZARD?\r\n");

    int n;
    for (n = 0; n < samples; n++) {
        oi_update(sd);

        int fl = sd->cliffFrontLeftSignal;
        int fr = sd->cliffFrontRightSignal;
        int sl = sd->cliffLeftSignal;
        int sr = sd->cliffRightSignal;

        int dfl = fl - s_baseline_fl;
        int dfr = fr - s_baseline_fr;

        bool white = (fl > LOCAL_WHITE_THRESHOLD || fr > LOCAL_WHITE_THRESHOLD);
        bool black = (fl < LOCAL_BLACK_THRESHOLD || fr < LOCAL_BLACK_THRESHOLD);
        bool hz    = white || black;

        char tag[12];
        if (white)      sprintf(tag, "WHITE");
        else if (black) sprintf(tag, "BLACK");
        else            sprintf(tag, "no");

        char msg[120];
        sprintf(msg, "  %5d %5d %5d %5d   %+6d %+6d   %s\r\n",
                fl, fr, sl, sr, dfl, dfr, tag);
        uart_sendStr(msg);

        timer_waitMillis(100);
    }

    char tmsg[128];
    sprintf(tmsg, ">>> Baseline: FL=%d FR=%d   Current thresholds: WHITE > %d, BLACK < %d\r\n",
            s_baseline_fl, s_baseline_fr,
            LOCAL_WHITE_THRESHOLD, LOCAL_BLACK_THRESHOLD);
    uart_sendStr(tmsg);
}
