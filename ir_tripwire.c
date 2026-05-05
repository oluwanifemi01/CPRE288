/*
 * ir_tripwire.c
 *
 * Implementation: fast continuous IR proximity check using the on-board
 * Sharp IR sensor and the project's calibration formula.
 *
 *     cm = 8668330.42 * raw^-1.73845
 *
 * The function is intentionally trivial — it averages a few ADC samples
 * and converts. Caller owns the servo position.
 */

#include "ir_tripwire.h"

#include "adc.h"
#include "cyBot_Scan.h"
#include <math.h>

extern cyBOT_Scan_t scan_data;   // shared with main.c

void ir_tripwire_park(void) {
    // One scan call to drive the servo to 90°. The PING value returned is
    // discarded; we just want the servo positioned.
    cyBOT_Scan(90, &scan_data);
}

double ir_tripwire_read_cm(void) {
    long sum = 0;
    int  i;
    for (i = 0; i < TRIPWIRE_SAMPLES; i++) {
        sum += adc_read();
    }
    int avg_raw = (int)(sum / TRIPWIRE_SAMPLES);

    if (avg_raw < 50) return 999.0;  // nothing in range
    return 8668330.42 * pow((double)avg_raw, -1.73845);
}

bool ir_tripwire_tripped(double threshold_cm) {
    double cm = ir_tripwire_read_cm();
    return (cm > 1.0 && cm < threshold_cm);
}
