#include <termios.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "serial/serial.hpp"

#if __DEBUG_SERIAL
#define __debug_print(...) std::cout << __VA_ARGS__ << std::endl
#define __debug_print_error(...) std::cerr << __VA_ARGS__ << std::endl
#else
#define __debug_print(...)
#define __debug_print_error(...)
#endif

serial::serial(const char *device,
               uint32_t baudrate,
               uint8_t databits,
               uint8_t stopbits,
               uint8_t parity,
               uint8_t min_buf_read,
               uint8_t max_timeout_read_ds,
               uint32_t attempts_read)
    : device(device),
      baudrate(baudrate),
      databits(databits),
      stopbits(stopbits),
      parity(parity),
      min_buf_read(min_buf_read),
      max_timeout_read_ds(max_timeout_read_ds),
      attempts_read(attempts_read),
      fd(-1) {}

serial::~serial()
{
    if (this->fd != -1)
    {
        ::close(this->fd);
        this->fd = -1;
    }
}

bool serial::init()
{
    this->fd = ::open(this->device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (this->fd == -1)
    {
        __debug_print_error("Failed to open serial device: " << this->device);
        return false;
    }

    struct termios tios;
    if (tcgetattr(this->fd, &tios) != 0)
    {
        __debug_print_error("Failed to get serial device attributes");
        ::close(this->fd);
        return false;
    }
    tios.c_ispeed = this->baudrate;
    tios.c_ospeed = this->baudrate;
    tios.c_cflag &= ~CSIZE;
    switch (this->databits)
    {
    case 5:
        tios.c_cflag |= CS5;
        break;
    case 6:
        tios.c_cflag |= CS6;
        break;
    case 7:
        tios.c_cflag |= CS7;
        break;
    case 8:
        tios.c_cflag |= CS8;
        break;
    default:
        __debug_print_error("Invalid databits: " << this->databits);
        ::close(this->fd);
        return false;
    }
    switch (this->stopbits)
    {
    case 1:
        tios.c_cflag &= ~CSTOPB;
        break;
    case 2:
        tios.c_cflag |= CSTOPB;
        break;
    default:
        __debug_print_error("Invalid stopbits: " << this->stopbits);
        ::close(this->fd);
        return false;
    }
    switch (this->parity)
    {
    case 0: // Disable parity
        tios.c_cflag &= ~PARENB;
        break;
    case 1: // Enable Parity (Odd)
        tios.c_cflag |= PARENB;
        tios.c_cflag |= PARODD;
        break;
    case 2: // Enable Parity (Even)
        tios.c_cflag |= PARENB;
        tios.c_cflag &= ~PARODD;
        break;
    default:
        __debug_print_error("Invalid parity: " << this->parity);
        ::close(this->fd);
        return false;
    }

    tios.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tios.c_oflag &= ~(OPOST | ONLCR);
    tios.c_cflag |= (CLOCAL | CREAD);
    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tios.c_cc[VMIN] = min_buf_read;
    tios.c_cc[VTIME] = max_timeout_read_ds;

    if (tcsetattr(this->fd, TCSANOW, &tios) != 0)
    {
        __debug_print_error("Failed to set serial device attributes");
        ::close(this->fd);
        return false;
    }

    return true;
}

bool serial::write(const uint8_t *data, uint32_t size)
{
    if (this->fd == -1)
    {
        __debug_print_error("Serial device not initialized");
        return false;
    }

    if (::write(this->fd, data, size) != size)
    {
        __debug_print_error("Failed to write data to the serial device: " << this->device);
        return false;
    }

    return true;
}

bool serial::read(uint8_t *data, uint32_t size)
{
    if (this->fd == -1)
    {
        __debug_print_error("Serial device not initialized");
        return false;
    }
    ssize_t tot_bufs_read = 0;
    uint32_t attempts = 0;
    while (tot_bufs_read < size && attempts < this->attempts_read)
    {
        ssize_t bufs_read = ::read(this->fd, data + tot_bufs_read, size - tot_bufs_read);
        if (bufs_read == -1)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                attempts++;
                __debug_print("Device: " << this->device << " is busy, retrying... (" << attempts << ")");
                continue;
            }
            __debug_print_error("Failed to read data to the serial device: " << this->device);
            return false;
        }
        else if (bufs_read == 0)
        {
            attempts++;
            __debug_print("No incoming data to the serial device: " << this->device << ", retrying... (" << attempts << ")");
            continue;
        }
        else
            tot_bufs_read += bufs_read;
    }
    if (tot_bufs_read == 0)
        __debug_print("No data received to the serial device: " << this->device);
    return true;
}
