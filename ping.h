/**

- Driver for ping sensor
- @file ping.h
- @author Oluwanifemi Oloyede
  */
  #ifndef PING_H_
  #define PING_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

// Shared enum - defined here so both ping.c and lab9.c can use LOW/HIGH/DONE
typedef enum { LOW, HIGH, DONE } PingState;

// Shared variables - defined in ping.c, declared here for other files
extern volatile PingState g_state;
extern volatile uint32_t  g_start_time;
extern volatile uint32_t  g_end_time;
extern volatile int       g_overflow;        // 1 if last reading overflowed
extern volatile int       g_overflow_count;  // running total of overflows

void ping_init(void);
void ping_trigger(void);
void TIMER3B_Handler(void);
float ping_getDistance(void);

#endif /* PING_H_ */
