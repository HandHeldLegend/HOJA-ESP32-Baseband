#include "mitch_i2c.h"
#include "driver/i2c.h"
#include <stdint.h>
#include "hoja_includes.h"
#include "hal/i2c_hal.h"
#include "esp_private/periph_ctrl.h"
#include "soc/i2c_reg.h"
#include "soc/i2c_struct.h"

#define I2C_MAX_ALLOWED_SIZE 128
#define MI2C_ENTER_CRITICAL(mux) portENTER_CRITICAL(mux)
#define MI2C_EXIT_CRITICAL(mux) portEXIT_CRITICAL(mux)

#define MI2C_RX_THRESHOLD 16
#define MI2C_TX_THRESHOLD 32

#define I2C_SLAVE_SCL_IO 20 /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 19 /*!< gpio number for i2c slave data */
#define I2C_SLAVE_PORT (0)  /*!< I2C port number for slave dev */
#define ESP_SLAVE_ADDR 0x76 /*!< ESP32 slave address, you can set any 7bit value */
#define I2C_RINGBUFFER_SIZE 16
i2c_dev_t *i2c_dev = &I2C0;
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

#define I2C_RAM_BASE (0x3ff53000 + 0x100)

intr_handle_t intr_handle; /*!< I2C interrupt handle*/

volatile uint8_t _rx_buffer[I2C_MAX_ALLOWED_SIZE] = {0};
// Request size sets how large the buffer
// should be when we read data
volatile uint32_t _rx_request_size = 32;
volatile uint32_t _rx_buffer_idx = 0;
volatile uint32_t _rx_buffer_size = 0;
volatile bool _rx_done = false;

volatile uint8_t _tx_buffer[I2C_MAX_ALLOWED_SIZE] = {0};
volatile uint32_t _tx_buffer_idx = 0;
volatile uint32_t _tx_buffer_size = 0;
volatile bool _tx_done = false;

volatile uint8_t _rx_ring_buffer[I2C_RINGBUFFER_SIZE][I2C_MAX_ALLOWED_SIZE];

typedef struct
{
    size_t size;
    size_t head;
    size_t tail;
    size_t count;
} RingBuffer;

volatile RingBuffer _rx_ringbuffer_ = {.size = I2C_RINGBUFFER_SIZE};

// Add data to the ring buffer
bool mi2c_ringbuffer_set(uint8_t *data, uint32_t size)
{
    if (_rx_ringbuffer_.count == _rx_ringbuffer_.size)
    {
        // Buffer is full
        return false;
    }
    memcpy(&(_rx_ring_buffer[_rx_ringbuffer_.head]), data, size);
    _rx_ringbuffer_.head = (_rx_ringbuffer_.head + 1) % _rx_ringbuffer_.size;
    _rx_ringbuffer_.count++;
    return true;
}

// Get data from the ring buffer
uint8_t *mi2c_ringbuffer_get()
{
    if (_rx_ringbuffer_.count == 0)
    {
        // Buffer is empty
        return NULL;
    }
    i2cinput_status_s *data = &(_rx_ring_buffer[_rx_ringbuffer_.tail]);
    _rx_ringbuffer_.tail = (_rx_ringbuffer_.tail + 1) % _rx_ringbuffer_.size;
    _rx_ringbuffer_.count--;
    return data;
}

static void IRAM_ATTR mi2c_isr_handler_default(void *arg)
{
    uint32_t int_mask;
    i2c_ll_get_intr_mask(&I2C0, &int_mask);

    i2c_intr_event_t evt_type = I2C_INTR_EVENT_ERR;
    i2c_ll_slave_get_event(&I2C0, &evt_type);

    // Handle putting data into our RX buffer
    // Intentionally not using 
    if ( evt_type == I2C_INTR_EVENT_TRANS_DONE )
    {
        uint32_t rx_fifo_cnt = 0;
        i2c_ll_get_rxfifo_cnt(&I2C0, &rx_fifo_cnt);
        
        if(rx_fifo_cnt>0)
        {
            i2c_ll_read_rxfifo(&I2C0, &(_rx_buffer[0]), rx_fifo_cnt);
        }
        if(rx_fifo_cnt >= _rx_request_size)
        {
            mi2c_ringbuffer_set(_rx_buffer, _rx_request_size);
        }
        
        i2c_ll_clear_intr_mask(&I2C0, int_mask);
    }
    else if (evt_type == I2C_INTR_EVENT_TXFIFO_EMPTY)
    {
        //uint32_t tx_fifo_rem;
        //i2c_ll_get_txfifo_len(&I2C0, &tx_fifo_rem);

        // We disable unused parts from the default handler.
        // We are only sending 16 bytes.
        // if (data)
        //{
        //    i2c_ll_write_txfifo(i2c_context[i2c_num].hal.dev, data, size);
        //    vRingbufferReturnItemFromISR(p_i2c->tx_ring_buf, data, &HPTaskAwoken);
        //}
        // else
        //{
        i2c_ll_slave_disable_tx_it(&I2C0);
        _tx_done = true;
        //}
        i2c_ll_clear_intr_mask(&I2C0, int_mask);
    }
}

typedef union
{
    struct
    {
        uint32_t data : 8; /*The register represent the byte  data read from rx_fifo when use apb fifo access*/
        uint32_t reserved : 24;
    };
    uint32_t val;
} fifo_data;

void mi2c_slave_setup()
{
    static i2c_hal_context_t context = {0};
    context.dev = &I2C0;
    i2c_config_t c = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.maximum_speed = 400 * 1000,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP_SLAVE_ADDR};

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &c));

    i2c_set_pin(I2C_NUM_0, c.sda_io_num, c.scl_io_num,
                c.sda_pullup_en, c.scl_pullup_en, c.mode);

    periph_module_enable(i2c_periph_signal[I2C_NUM_0].module);
    i2c_hal_init(&context, I2C_NUM_0);

    i2c_ll_disable_intr_mask(&I2C0, I2C_LL_INTR_MASK);
    i2c_ll_clear_intr_mask(&I2C0, I2C_LL_INTR_MASK);

    i2c_ll_slave_init(&I2C0);
    // MSB
    i2c_ll_set_data_mode(&I2C0, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);

    i2c_ll_txfifo_rst(&I2C0);
    i2c_ll_rxfifo_rst(&I2C0);

    i2c_ll_set_slave_addr(&I2C0, c.slave.slave_addr, c.slave.addr_10bit_en);

    i2c_ll_set_rxfifo_full_thr(&I2C0, 32);
    i2c_ll_set_txfifo_empty_thr(&I2C0, 0);

    i2c_ll_master_set_filter(&I2C0, 2);

    i2c_ll_set_sda_timing(&I2C0, 10, 10);
    i2c_ll_set_tout(&I2C0, 32000);

    esp_intr_alloc(i2c_periph_signal[I2C_NUM_0].irq, 0,
                   mi2c_isr_handler_default, NULL,
                   &intr_handle);

    I2C0.int_ena.val |= I2C_TRANS_COMPLETE_INT_ENA_M;
    I2C0.int_ena.val |= I2C_TXFIFO_EMPTY_INT_ENA_M;
}

void mi2c_clear_rx_fifo()
{

    _rx_buffer_idx = 0;
    // Reset RX FIFO
    // i2c_dev->fifo_conf.rx_fifo_rst = 1;
    // Clear reset bits after resetting
    // i2c_dev->fifo_conf.rx_fifo_rst = 0;
}

void mi2c_clear_tx_fifo()
{
    memset(_tx_buffer, 0, sizeof(_rx_buffer));
    _tx_buffer_idx = 0;
    // Reset TX FIFO
    i2c_dev->fifo_conf.tx_fifo_rst = 1;

    // Clear reset bits after resetting
    i2c_dev->fifo_conf.tx_fifo_rst = 0;
}

void mi2c_clear_fifos()
{

    MI2C_ENTER_CRITICAL((portMUX_TYPE *)&spinlock);
    uint32_t tmp = I2C0.fifo_conf.val;
    typeof(I2C0.fifo_conf) temp_reg;
    temp_reg.rx_fifo_rst = 1;
    temp_reg.tx_fifo_rst = 1;
    I2C0.fifo_conf.val = temp_reg.val;

    temp_reg.rx_fifo_rst = 0;
    temp_reg.tx_fifo_rst = 0;
    I2C0.fifo_conf.val = temp_reg.val;
    MI2C_EXIT_CRITICAL((portMUX_TYPE *)&spinlock);
}

mi2c_status_t mi2c_slave_polling_read(uint8_t *data, size_t size, uint32_t ms_wait)
{
    uint8_t *out = NULL;

    out = mi2c_ringbuffer_get();

    if (out != NULL)
    {
        memset(data, 0, size);
        // Got data ok
        memcpy(data, out, size);
        return MI2C_OK;
    }
    else
    {
        return MI2C_TIMEOUT;
    }
    return MI2C_TIMEOUT;
}

mi2c_status_t mi2c_slave_polling_write(const uint8_t *data, size_t size, TickType_t ticks_to_wait)
{
    // Copy data into our buffer
    memcpy(_tx_buffer, data, size);
    // Reset TX flag
    _tx_done = false;

    // Set our initial expiration. We may update this in this function!
    TickType_t ticks_current = xTaskGetTickCount();
    TickType_t ticks_end = ticks_current + ticks_to_wait;

    i2c_ll_write_txfifo(&I2C0, _tx_buffer, size);
    i2c_ll_slave_enable_tx_it(&I2C0);
    
    while ((!_tx_done) && (ticks_current <= ticks_end))
    {
        // Update our timer
        ticks_current = xTaskGetTickCount();
        portYIELD();
    }

    if (_tx_done)
    {
        //printf("TX OK\n");
        return MI2C_OK;
    }
    else
    {
        return MI2C_TIMEOUT;
    }
    return MI2C_TIMEOUT;
}
