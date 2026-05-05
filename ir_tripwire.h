#ifndef IR_TRIPWIRE_H_
#define IR_TRIPWIRE_H_

#include <stdbool.h>

/*
 * IR trip-wire — fast continuous obstacle proximity check.
 *
 * Uses the on-board Sharp IR sensor (mounted on the scanning servo). Samples
 * the ADC directly and converts via the calibration formula
 * cm = 8668330.42 * raw^-1.73845.
 *
 * The caller is responsible for parking the servo at 90° (straight ahead)
 * before relying on these readings. Use ir_tripwire_park() to do that.
 *
 * Below ~10 cm the IR response is non-monotonic, so very close objects can
 * read as a moderate value. Treat any reading < TRIPWIRE_NEAR_CM as "stop now"
 * regardless of exact value, then confirm with a PING read before deciding
 * what to do next.
 */

#define TRIPWIRE_NEAR_CM        20.0   // anything closer than this = stop
#define TRIPWIRE_SLOW_CM        35.0   // soft warning — slow / course correct
#define TRIPWIRE_SAMPLES        3      // averages this many ADC reads per check

/**
 * Park the scanning servo at 90° (straight ahead) so subsequent IR reads
 * measure the forward direction. Call once before entering a continuous
 * sampling loop.
 */
void ir_tripwire_park(void);

/**
 * Take a fresh IR reading (averaged over TRIPWIRE_SAMPLES samples) and
 * return the converted distance in cm. Returns 999.0 if no object detected.
 * Fast (~5–15 ms per call). Servo position unchanged.
 */
double ir_tripwire_read_cm(void);

/**
 * Convenience: returns true if the IR trip-wire reads closer than `threshold_cm`.
 */
bool ir_tripwire_tripped(double threshold_cm);

#endif /* IR_TRIPWIRE_H_ */
