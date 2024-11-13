#ifndef SERIAL__SERIAL_HPP_
#define SERIAL__SERIAL_HPP_

#include <stdint.h>

class serial
{
public:
    serial(const char *device,
           uint32_t baudrate,
           uint8_t databits,
           uint8_t stopbits,
           uint8_t parity,
           uint8_t min_buf_read = 0,
           uint8_t max_timeout_read_ds = 10,
           uint32_t attempts_read = 1000);
    ~serial();
    bool init();
    bool write(const uint8_t *data, uint32_t size);
    bool read(uint8_t *data, uint32_t size);
    bool close();

private:
    const char *device;
    uint32_t baudrate;
    uint8_t databits;
    uint8_t stopbits;
    uint8_t parity;
    uint8_t min_buf_read;
    uint8_t max_timeout_read_ds;
    uint32_t attempts_read;
    int fd;
};

#endif // SERIAL__SERIAL_HPP_