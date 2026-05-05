/**

- Driver for ping sensor
- @file ping.c
- @author Oluwanifemi Oloyede
  */

#include "ping.h"
#include "Timer.h"

// Global shared variables - extern declared in ping.h
 volatile PingState g_state        = LOW;
 volatile uint32_t  g_start_time   = 0;
 volatile uint32_t  g_end_time     = 0;
 volatile int       g_overflow     = 0;
 volatile int       g_overflow_count = 0;

void ping_init(void) {
// 1. Enable peripheral clocks
SYSCTL_RCGCGPIO_R  |= 0x02;  // Port B
SYSCTL_RCGCTIMER_R |= 0x08;  // Timer 3
while ((SYSCTL_PRGPIO_R & 0x02) == 0); // Wait for Port B ready
while ((SYSCTL_PRTIMER_R & 0x08) == 0); // Wait for Timer 3 ready



// 2. Configure PB3 as GPIO digital output initially (for trigger pulse)
GPIO_PORTB_AFSEL_R &= ~0x08; // No alternate function yet
GPIO_PORTB_DIR_R   |=  0x08; // PB3 = output
GPIO_PORTB_DEN_R   |=  0x08; // PB3 digital enable
GPIO_PORTB_DATA_R  &= ~0x08; // PB3 start low

// Pre-load PMC3 = 7 (T3CCP1) into PCTL - takes effect when AFSEL is set
GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF0FFF) | 0x00007000;

// 3. Configure Timer 3B in input edge-time mode
TIMER3_CTL_R  &= ~(1 << 8);  // Disable Timer B before configuring
TIMER3_CFG_R   = 0x04;       // 16-bit split-timer mode
TIMER3_TBMR_R  = 0x07;       // Capture mode, edge-time, count-DOWN
TIMER3_CTL_R  |= (0x3 << 10);// TBEVENT = 11b = both edges
TIMER3_TBILR_R = 0xFFFF;     // Full 16-bit load value
TIMER3_TBPR_R  = 0xFF;       // 8-bit prescaler -> 24-bit counter total

// 4. Set up interrupts
TIMER3_ICR_R = (1 << 10);    // Clear any stale interrupt
TIMER3_IMR_R |= (1 << 10);   // Enable Capture B Event interrupt (CBEIM)

IntRegister(INT_TIMER3B, TIMER3B_Handler);
IntEnable(INT_TIMER3B);
IntMasterEnable();

// 5. Start timer
TIMER3_CTL_R |= (1 << 8);    // Enable Timer B


}

void ping_trigger(void) {
g_state = LOW;


// Step 1: Disable timer and interrupt before touching the pin
TIMER3_CTL_R &= ~(1 << 8);   // Disable Timer B (TBEN)
TIMER3_IMR_R &= ~(1 << 10);  // Mask Capture B interrupt (CBEIM)

// Step 2: Switch PB3 to plain GPIO output
GPIO_PORTB_AFSEL_R &= ~0x08; // Disconnect timer from pin
GPIO_PORTB_DIR_R   |=  0x08; // PB3 = output

// Step 3: Send trigger pulse - low->high (5us min)->low
GPIO_PORTB_DATA_R &= ~0x08;  // PB3 = 0
GPIO_PORTB_DATA_R |=  0x08;  // PB3 = 1
timer_waitMicros(5);          // Hold high >= 5 us
GPIO_PORTB_DATA_R &= ~0x08;  // PB3 = 0

// Step 4: Switch PB3 back to Timer 3B CCP alternate function
GPIO_PORTB_DIR_R   &= ~0x08;
GPIO_PORTB_AFSEL_R |=  0x08; // Re-enable alternate function (T3CCP1)

// Step 5: Clear any spurious capture flag caused by the GPIO toggling
//         AND by the brief float during the AFSEL transition.
//         Must be done AFTER AFSEL is re-enabled, otherwise the
//         transition glitch latches a stale capture event.
TIMER3_ICR_R = (1 << 10);

// Step 6: Re-enable interrupt then timer
TIMER3_IMR_R |= (1 << 10);   // Unmask Capture B interrupt
TIMER3_CTL_R |= (1 << 8);    // Enable Timer B

}

void TIMER3B_Handler(void) {
// Confirm interrupt source is Capture B Event (CBEMIS = bit 10 in MIS)
if (TIMER3_MIS_R & (1 << 10)) {
TIMER3_ICR_R = (1 << 10);  // Clear interrupt immediately

    /* 24-bit countdown value: 8-bit prescaler snapshot (TBPS) + 16-bit timer (TBR) */
    uint32_t captured =
        ((TIMER3_TBPS_R & 0xFFU) << 16) | (TIMER3_TBR_R & 0xFFFFU);

    if (g_state == LOW) {
        g_start_time = captured;  // Rising edge = start
        g_state = HIGH;
    } else if (g_state == HIGH) {
        g_end_time = captured;    // Falling edge = end
        g_state = DONE;
    }
}

}

float ping_getDistance(void) {
ping_trigger();
while (g_state != DONE) {} // Wait for both edges

// Compute pulse width (count-down: start > end normally)
int64_t pulse_cycles;
if (g_start_time >= g_end_time) {
    pulse_cycles = (int64_t)(g_start_time - g_end_time);
    g_overflow = 0;
} else {
    // Timer wrapped from 0x000000 back to 0xFFFFFF between edges
    pulse_cycles = (int64_t)(g_start_time + (0xFFFFFF - g_end_time) + 1);
    g_overflow = 1;
    g_overflow_count++;
}

// Convert to cm: (cycles / 16MHz) * 34300 cm/s / 2 (round trip)
float dist_cm = ((float)pulse_cycles / 16000000.0f * 34300.0f) / 2.0f;
return dist_cm;

}
