#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdbool.h>
#include "REF_tm4c123gh6pm.h" // Given in the reference files [cite: 55]

// Initializes the ADC module and GPIO Port B, Pin 4
void adc_init(void);

// Triggers a single conversion and returns the 12-bit raw result
uint16_t adc_read(void);

#endif /* ADC_H_ */
