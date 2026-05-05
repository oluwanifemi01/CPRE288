#include "adc.h"

void adc_init(void) {
    // 1. Enable ADC and GPIO Clocks
    SYSCTL_RCGCADC_R |= 0x0001;   // Enable clock for ADC0 [cite: 213]
    SYSCTL_RCGCGPIO_R |= 0x02;    // Enable clock for GPIO Port B (for PB4) [cite: 83]
    
    // Wait for the peripherals to be ready
    while((SYSCTL_PRGPIO_R & 0x02) == 0) {};
    while((SYSCTL_PRADC_R & 0x0001) == 0) {}; // Check ADC Peripheral Ready [cite: 213]

    // 2. Configure PB4 as an analog input
    GPIO_PORTB_DIR_R &= ~0x10;    // Set PB4 as input
    GPIO_PORTB_AFSEL_R |= 0x10;   // Enable alternate function on PB4
    GPIO_PORTB_DEN_R &= ~0x10;    // Disable digital I/O on PB4
    GPIO_PORTB_AMSEL_R |= 0x10;   // Enable analog functionality on PB4

    // 3. Configure ADC0 Sample Sequencer 1 (SS1)
    ADC0_ACTSS_R &= ~0x0002;      // Disable SS1 during configuration [cite: 204]
    ADC0_EMUX_R &= ~0x00F0;       // Configure SS1 for software trigger [cite: 208]
    
    // Set SS1 multiplexer to read from AIN10 [cite: 83, 210]
    ADC0_SSMUX1_R = (ADC0_SSMUX1_R & 0xFFFFFFF0) | 10; 
    
    // Configure sample control: set END0 and IE0 (Interrupt Enable) for the first sample
    ADC0_SSCTL1_R = 0x0006;       // [cite: 210]
    
    // Re-enable SS1 after configuration is complete
    ADC0_ACTSS_R |= 0x0002;       // [cite: 204]
}

uint16_t adc_read(void) {
    uint16_t result;
    
    // 1. Initiate the sample sequence
    ADC0_PSSI_R = 0x0002;         // Start conversion on SS1 [cite: 209]
    
    // 2. Wait for the conversion to complete
    while((ADC0_RIS_R & 0x02) == 0){}; // Poll the Raw Interrupt Status flag [cite: 205]
    
    // 3. Read the 12-bit digital value from the FIFO (0 to 4095)
    result = ADC0_SSFIFO1_R & 0xFFF;   // [cite: 29, 211]
    
    // 4. Acknowledge and clear the interrupt flag so the next read works
    ADC0_ISC_R = 0x0002;          // [cite: 207]
    
    return result;
}