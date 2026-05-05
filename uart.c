/*
 *   uart.c
 *
 *   UART1 initialization and communication functions for CyBot <-> PuTTY
 *   Serial parameters: Baud = 115200, 8 data bits, 1 stop bit,
 *   no parity, no flow control, FIFOs disabled on UART1
 *
 *   Port B pins used:
 *     PB0 = UART1 Rx
 *     PB1 = UART1 Tx
 */

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "uart.h"

void uart_init(void) {

    // 1. Enable clock to GPIO Port B
    SYSCTL_RCGCGPIO_R |= 0x02;         // Bit 1 = Port B

    // 2. Enable clock to UART1
    SYSCTL_RCGCUART_R |= 0x02;         // Bit 1 = UART1

    // 3. Wait for peripherals to be ready
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {};
    while ((SYSCTL_PRUART_R & 0x02) == 0) {};

    // 4. Enable alternate functions on PB0 (Rx) and PB1 (Tx)
    GPIO_PORTB_AFSEL_R |= 0x03;        // Bits 1:0

    // 5. Enable digital functionality on PB0 and PB1
    GPIO_PORTB_DEN_R |= 0x03;          // Bits 1:0

    // 6. Configure Port Control register to select UART1 (PMC = 1) for PB0 and PB1
    //    Each pin uses 4 bits in PCTL; PB0 = bits[3:0], PB1 = bits[7:4]
    GPIO_PORTB_PCTL_R &= 0xFFFFFF00;   // Clear fields for PB0 and PB1
    GPIO_PORTB_PCTL_R |= 0x00000011;   // Set PMC0=1 and PMC1=1 (UART1)

    // ---- UART1 Device Configuration ----

    // 7. Disable UART1 while configuring
    UART1_CTL_R &= ~0x01;              // Clear UARTEN bit

    // 8. Calculate baud rate divisor
    //    BRD = SysClk / (ClkDiv * BaudRate) = 16,000,000 / (16 * 115200) = 8.68055...
    //    IBRD = integer part = 8
    //    FBRD = integer(0.68055... * 64 + 0.5) = integer(44.1) = 44
    uint16_t iBRD = 8;
    uint16_t fBRD = 44;

    UART1_IBRD_R = iBRD;
    UART1_FBRD_R = fBRD;

    // 9. Set frame format: 8 data bits, 1 stop bit, no parity, no FIFO
    //    WLEN=0x3 (8-bit), FEN=0 (FIFO disabled), STP2=0, PEN=0, EPS=0
    //    LCRH value: 0b01100000 = 0x60
    UART1_LCRH_R = 0x60;

    // 10. Use system clock as UART clock source
    UART1_CC_R = 0x0;

    // 11. Re-enable UART1 with RX and TX enabled
    //     Bit 0: UARTEN, Bit 8: TXE, Bit 9: RXE
    UART1_CTL_R = 0x301;               // Enable UART, TX, and RX
}


void uart_sendChar(char data) {
    // Wait until the transmit FIFO/buffer is not full (TXFF bit = 0)
    while ((UART1_FR_R & 0x20) != 0) {};
    UART1_DR_R = data;
}


char uart_receive(void) {
    // Wait until data is available (RXFE bit = 0 means receive FIFO not empty)
    while ((UART1_FR_R & 0x10) != 0) {};
    return (char)(UART1_DR_R & 0xFF);
}


void uart_sendStr(const char *data) {
    while (*data != '\0') {
        uart_sendChar(*data);
        data++;
    }
}

int uart_receive_nonblocking(char *data) {
    // Check if RXFE (Receive FIFO Empty) bit is 0
    if ((UART1_FR_R & 0x10) == 0) {
        *data = (char)(UART1_DR_R & 0xFF);
        return 1; // Data received
    }
    return 0; // No data available right now
}