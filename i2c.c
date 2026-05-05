/**
 * @file i2c.c
 *
 * Library for I2C on TM4C123GH6PM Microcontroller
 *
 * @author Braedon Giblin <bgiblin@iastate.edu>
 */

#include "i2c.h"
#include <stdlib.h>

void i2c_init() {
    // Enable clock to I2C
    SYSCTL_RCGCI2C_R |= 0X2;
    while (!(SYSCTL_PRI2C_R & 0x2));

    // Enable clock to GPIO Port A
    SYSCTL_RCGCGPIO_R |= 0X1;
    while (!(SYSCTL_PRGPIO_R & 0x1));

    // Config SCL and SDA as digital i2c with SCL set as open drain
    GPIO_PORTA_DEN_R |= 0xC0;
    GPIO_PORTA_AFSEL_R |= 0xC0;
    GPIO_PORTA_ODR_R  |= 0x80;
    GPIO_PORTA_PCTL_R |= 0x33000000;

    // Enable as master
    I2C1_MCR_R |= 0x10;

    // Set clock speed to 100 kHz for the current 16 MHz lab clock.
    // If the system clock is changed to 80 MHz, this must become 39.
    I2C1_MTPR_R  = 7;
}

uint8_t i2c_requestByte(uint8_t addr, uint8_t request) {
    // Write slave address to address register
    I2C1_MSA_R = addr << 1;

    // Write byte to send to data register
    I2C1_MDR_R = request;

    // send START and EXECUTE
    I2C1_MCS_R = 0x3;

    // Wait while busy
    while(I2C1_MCS_R & 0x1);

    // Check for error
    if (I2C1_MCS_R & 0x02) {
        return 0;   // Return 0 if error
    } else {
        return 1;   // Return 1 if not error (for 1 byte sent)
    }
}

size_t i2c_sendByte(uint8_t addr, uint8_t data) {
    // Write slave address to address register
    I2C1_MSA_R = addr << 1;

    // Write byte to send to data register
    I2C1_MDR_R = data;

    // send START, STOP, and EXECUTE
    I2C1_MCS_R = 0x7;

    // Wait while busy
    while(I2C1_MCS_R & 0x1);

    // Check for error
    if (I2C1_MCS_R & 0x02) {
        return 0;   // Return 0 if error
    } else {
        return 1;   // Return 1 if not error (for 1 byte sent)
    }
}

size_t i2c_sendBytes(uint8_t addr, uint8_t* data, size_t dataLen) {
    if (data == NULL || dataLen == 0) {
        return 0;
    }
    if (dataLen == 1) {
        return i2c_sendByte(addr, data[0]);
    }

    size_t dataSent = 0;

    // Write Slave address to address register
    I2C1_MSA_R = addr << 1;

    // write first data byte with a START and EXECUTE
    I2C1_MDR_R = data[dataSent++];
    I2C1_MCS_R = 0x3;
    while(I2C1_MCS_R & 0x1);
    if (I2C1_MCS_R & 0x02) {
        return dataSent - 1;
    }
    while (dataSent < dataLen - 1) {
        // write next data byte with a EXECUTE
        I2C1_MDR_R = data[dataSent++];
        I2C1_MCS_R = 0x1;
        while(I2C1_MCS_R & 0x1);
        if (I2C1_MCS_R & 0x02) {
            return dataSent - 1;
        }
    }
    // write last data byte with a STOP, EXECUTE
    I2C1_MDR_R = data[dataSent++];
    I2C1_MCS_R = 0x5;
    while(I2C1_MCS_R & 0x1);
    if (I2C1_MCS_R & 0x02) {
        return dataSent - 1;
    }
    return dataSent;
}

uint8_t i2c_recByte(uint8_t addr){
    I2C1_MSA_R = (addr << 1) | 0b1;

    I2C1_MCS_R = 0x7;

    while(I2C1_MCS_R & 0x1);

    if (I2C1_MCS_R & 0x02) {
        return 0;
    } else {
        return (uint8_t) I2C1_MDR_R;
    }

}

uint8_t* i2c_recBytes(uint8_t addr, size_t dataLen) {
    if (dataLen == 0) {
        return NULL;
    }
    if (dataLen == 1) {
        uint8_t* recData = calloc(1, sizeof(uint8_t));
        if (recData != NULL) {
            recData[0] = i2c_recByte(addr);
        }
        return recData;
    }

    uint8_t* recData = calloc(dataLen, sizeof(uint8_t));
    if (recData == NULL) {
        return NULL;
    }
    int dataRecIndex = 0;

    // Write Slave address to address register
    I2C1_MSA_R = (addr << 1) | 0b1;

    // read first data byte with a START, ACK, and EXECUTE
    I2C1_MCS_R = 0xB;
    while(I2C1_MCS_R & 0x1);
    if (I2C1_MCS_R & 0x02) {
        free(recData);
        return NULL;
    }
    while (dataRecIndex < (int)dataLen - 2) {
        // Store middle bytes and ACK each following read.
        recData[dataRecIndex++] = I2C1_MDR_R;
        I2C1_MCS_R = 0x9;
        while(I2C1_MCS_R & 0x1);
        if (I2C1_MCS_R & 0x02) {
            free(recData);
            return NULL;
        }
    }

    // Store the next-to-last byte, then NACK+STOP the final byte.
    recData[dataRecIndex++] = I2C1_MDR_R;
    I2C1_MCS_R = 0x5;
    while(I2C1_MCS_R & 0x1);
    if (I2C1_MCS_R & 0x02) {
        free(recData);
        return NULL;
    }
    recData[dataRecIndex++] = I2C1_MDR_R;
    return recData;
}


