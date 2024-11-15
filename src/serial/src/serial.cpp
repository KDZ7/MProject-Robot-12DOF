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
               baudrate_t baudrate,
               databit_t databit,
               stopbit_t stopbit,
               parity_t parity,
               bool enable_RTS_CTS,
               uint8_t max_timeout_read_ds,
               uint8_t min_buf_read,
               bool wait_buf_read) : device(device),
                                     baudrate(baudrate),
                                     databit(databit),
                                     stopbit(stopbit),
                                     parity(parity),
                                     enable_RTS_CTS(enable_RTS_CTS),
                                     max_timeout_read_ds(max_timeout_read_ds),
                                     min_buf_read(min_buf_read),
                                     wait_buf_read(wait_buf_read),
                                     fd(-1) {}

serial::~serial()
{
    if (fd != -1)
        ::close(fd);
}

bool serial::init()
{

    switch (wait_buf_read)
    {
    case true:
        fd = ::open(device, O_RDWR | O_NOCTTY);
        break;
    case false:
        fd = ::open(device, O_RDWR | O_NOCTTY | O_NDELAY);
        break;
    }

    if (fd == -1)
    {
        __debug_print_error("Failed to open serial device: " << device);
        return false;
    }

    struct termios tios;
    if (tcgetattr(fd, &tios) != 0)
    {
        __debug_print_error("Failed to get serial device attributes");
        ::close(fd);
        return false;
    }

    tios.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tios.c_oflag &= ~(OPOST | ONLCR);

    tios.c_cflag &= ~CSIZE;
    switch (databit)
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
        __debug_print_error("Invalid databit: " << databit);
        ::close(fd);
        return false;
    }

    switch (stopbit)
    {
    case 1:
        tios.c_cflag &= ~CSTOPB;
        break;
    case 2:
        tios.c_cflag |= CSTOPB;
        break;
    default:
        __debug_print_error("Invalid stopbit: " << stopbit);
        ::close(fd);
        return false;
    }

    switch (parity)
    {
    case 0: // Disable Parity
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
        __debug_print_error("Invalid parity: " << parity);
        ::close(fd);
        return false;
    }
    switch (enable_RTS_CTS)
    {
    case true:
        tios.c_cflag |= CRTSCTS;
        break;
    case false:
        tios.c_cflag &= ~CRTSCTS;
        break;
    }
    tios.c_cflag |= (CLOCAL | CREAD);

    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

    tios.c_cc[VTIME] = max_timeout_read_ds;
    tios.c_cc[VMIN] = min_buf_read;

    switch (baudrate)
    {
    case 9600:
        cfsetispeed(&tios, B9600);
        cfsetospeed(&tios, B9600);
        break;
    case 19200:
        cfsetispeed(&tios, B19200);
        cfsetospeed(&tios, B19200);
        break;
    case 38400:
        cfsetispeed(&tios, B38400);
        cfsetospeed(&tios, B38400);
        break;
    case 57600:
        cfsetispeed(&tios, B57600);
        cfsetospeed(&tios, B57600);
        break;
    case 115200:
        cfsetispeed(&tios, B115200);
        cfsetospeed(&tios, B115200);
        break;
    case 230400:
        cfsetispeed(&tios, B230400);
        cfsetospeed(&tios, B230400);
        break;
    case 460800:
        cfsetispeed(&tios, B460800);
        cfsetospeed(&tios, B460800);
        break;
    case 500000:
        cfsetispeed(&tios, B500000);
        cfsetospeed(&tios, B500000);
        break;
    case 576000:
        cfsetispeed(&tios, B576000);
        cfsetospeed(&tios, B576000);
        break;
    case 921600:
        cfsetispeed(&tios, B921600);
        cfsetospeed(&tios, B921600);
        break;
    case 1000000:
        cfsetispeed(&tios, B1000000);
        cfsetospeed(&tios, B1000000);
        break;
    default:
        __debug_print_error("Invalid baudrate: " << baudrate);
        ::close(fd);
        return false;
    }

    if (tcsetattr(fd, TCSANOW, &tios) != 0)
    {
        __debug_print_error("Failed to set serial device attributes");
        ::close(fd);
        return false;
    }

    return true;
}

ssize_t serial::write(const uint8_t *data, uint32_t size)
{
    if (fd == -1)
    {
        __debug_print_error("Serial device not initialized");
        return -1;
    }

    ssize_t bytes_written = ::write(fd, data, size);
    if (bytes_written != size)
    {
        __debug_print_error("Failed to write data to the serial device: " << device);
        return -1;
    }

    return bytes_written;
}

ssize_t serial::read(uint8_t *data, uint32_t size)
{
    if (fd == -1)
    {
        __debug_print_error("serial device not initialized");
        return -1;
    }

    ssize_t bytes_read = ::read(fd, data, size);
    if (bytes_read < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return -1;

        __debug_print_error("Failed to read from serial device");
        return -1;
    }

    return bytes_read;
}
