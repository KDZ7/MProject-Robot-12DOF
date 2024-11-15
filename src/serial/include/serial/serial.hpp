#ifndef SERIAL__SERIAL_HPP_
#define SERIAL__SERIAL_HPP_

#include <cstdint>

typedef enum
{
    BAUDRATE_9600 = 9600,
    BAUDRATE_19200 = 19200,
    BAUDRATE_38400 = 38400,
    BAUDRATE_57600 = 57600,
    BAUDRATE_115200 = 115200,
    BAUDRATE_230400 = 230400,
    BAUDRATE_460800 = 460800,
    BAUDRATE_500000 = 500000,
    BAUDRATE_576000 = 576000,
    BAUDRATE_921600 = 921600,
    BAUDRATE_1000000 = 1000000
} baudrate_t;

typedef enum
{
    DATABIT_5 = 5,
    DATABIT_6 = 6,
    DATABIT_7 = 7,
    DATABIT_8 = 8
} databit_t;

typedef enum
{
    STOPBIT_1 = 1,
    STOPBIT_2 = 2
} stopbit_t;

typedef enum
{
    PARITY_NONE = 0,
    PARITY_ODD = 1,
    PARITY_EVEN = 2
} parity_t;

class serial
{
public:
    serial(const char *device,
           baudrate_t baudrate,
           databit_t databit,
           stopbit_t stopbit,
           parity_t parity,
           bool enable_RTS_CTS = false,
           uint8_t max_timeout_read_ds = 10,
           uint8_t min_buf_read = 0,
           bool wait_buf_read = false);
    ~serial();

    bool init();
    ssize_t write(const uint8_t *data, uint32_t size);
    ssize_t read(uint8_t *data, uint32_t size);

private:
    const char *device;
    baudrate_t baudrate;
    databit_t databit;
    stopbit_t stopbit;
    parity_t parity;
    bool enable_RTS_CTS;
    uint8_t max_timeout_read_ds;
    uint8_t min_buf_read;
    bool wait_buf_read;
    int fd;
};

#endif // SERIAL__SERIAL_HPP_
