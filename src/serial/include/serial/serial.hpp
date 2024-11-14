#ifndef SERIAL__SERIAL_HPP_
#define SERIAL__SERIAL_HPP_

#include <cstdint>

class serial
{
public:
    serial(const char *device,
           uint32_t baudrate,
           uint8_t databits,
           uint8_t stopbits,
           uint8_t parity,
           uint8_t min_buf_read = 0,
           uint8_t max_timeout_read_ds = 10);
    ~serial();

    bool init();
    bool write(const uint8_t *data, uint32_t size);
    bool read(uint8_t *data, uint32_t size);

private:
    const char *device_;
    uint32_t baudrate_;
    uint8_t databits_;
    uint8_t stopbits_;
    uint8_t parity_;
    uint8_t min_buf_read_;
    uint8_t max_timeout_read_ds_;
    int fd_;
};

#endif // SERIAL__SERIAL_HPP_
