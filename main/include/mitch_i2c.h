#ifndef MITCH_I2C_H
#define MITCH_I2C_H
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef unsigned long TickType_t;
#define portMAX_DELAY ( TickType_t ) ULONG_MAX

typedef enum {
    MI2C_OK,
    MI2C_TIMEOUT,
    MI2C_ERROR
} mi2c_status_t;

void mi2c_dump_buffers();
void mi2c_slave_setup();
void mi2c_clear_fifos();
void mi2c_clear_tx_fifo();
void mi2c_clear_rx_fifo();
mi2c_status_t mi2c_slave_polling_read(uint8_t *data, size_t size, TickType_t ticks_to_wait);
mi2c_status_t mi2c_slave_polling_write(const uint8_t *data, size_t size, TickType_t ticks_to_wait);

#endif