#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <termios.h>
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
               uint8_t max_timeout_read_ds) : device_(device),
                                              baudrate_(baudrate),
                                              databits_(databits),
                                              stopbits_(stopbits),
                                              parity_(parity),
                                              min_buf_read_(min_buf_read),
                                              max_timeout_read_ds_(max_timeout_read_ds),
                                              fd_(-1) {}

serial::~serial()
{
    if (fd_ != -1)
        ::close(fd_);
}

bool serial::init()
{
    fd_ = ::open(device_, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1)
    {
        __debug_print_error("Failed to open serial device: " << device_);
        return false;
    }

    struct termios tios;
    if (tcgetattr(fd_, &tios) != 0)
    {
        __debug_print_error("Failed to get serial device attributes");
        ::close(fd_);
        return false;
    }

    tios.c_ispeed = baudrate_;
    tios.c_ospeed = baudrate_;

    tios.c_cflag &= ~CSIZE;

    switch (databits_)
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
        __debug_print_error("Invalid databits: " << databits_);
        ::close(fd_);
        return false;
    }

    switch (stopbits_)
    {
    case 1:
        tios.c_cflag &= ~CSTOPB;
        break;
    case 2:
        tios.c_cflag |= CSTOPB;
        break;
    default:
        __debug_print_error("Invalid stopbits: " << stopbits_);
        ::close(fd_);
        return false;
    }

    switch (parity_)
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
        __debug_print_error("Invalid parity: " << parity_);
        ::close(fd_);
        return false;
    }

    tios.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tios.c_oflag &= ~(OPOST | ONLCR);
    tios.c_cflag |= (CLOCAL | CREAD);
    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tios.c_cc[VMIN] = min_buf_read_;
    tios.c_cc[VTIME] = max_timeout_read_ds_;

    if (tcsetattr(fd_, TCSANOW, &tios) != 0)
    {
        __debug_print_error("Failed to set serial device attributes");
        ::close(fd_);
        return false;
    }

    return true;
}

bool serial::write(const uint8_t *data, uint32_t size)
{
    if (fd_ == -1)
    {
        __debug_print_error("Serial device not initialized");
        return false;
    }

    if (::write(fd_, data, size) != size)
    {
        __debug_print_error("Failed to write data to the serial device: " << device_);
        return false;
    }

    return true;
}

bool serial::read(uint8_t *data, uint32_t size)
{
    if (fd_ == -1)
    {
        __debug_print_error("serial device not initialized");
        return false;
    }

    ssize_t bytes_read = ::read(fd_, data, size);
    if (bytes_read < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return false;

        __debug_print_error("Failed to read from serial device");
        return false;
    }

    return true;
}
