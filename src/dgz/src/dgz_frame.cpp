#include <iostream>
#include <cstring>
#include "dgz/dgz_frame.hpp"

#define __DEBUG_DGZ_FRAME 1

#if __DEBUG_DGZ_FRAME
#define __debug_print(...) std::cout << __VA_ARGS__ << std::endl
#define __debug_print_error(...) std::cerr << __VA_ARGS__ << std::endl
#else
#define __debug_print(...)
#define __debug_print_error(...)
#endif

dgz_frame::dgz_frame(uint8_t type,
                     uint8_t firstAddress,
                     uint8_t *data,
                     uint8_t size_data)
    : length(size_data + 8),
      type(type),
      firstAddress(firstAddress)
{
    data = new uint8_t[size_data];
    memcpy(this->data, data, size_data);
    this->checksum = this->CChecksum();
}

dgz_frame::~dgz_frame()
{
    delete[] data;
}

uint8_t dgz_frame::CChecksum()
{
    uint8_t sum = 0;
    sum += length;
    sum += type;
    sum += firstAddress;
    for (uint8_t i = 0; i < length - 8; i++)
        sum += data[i];
    return (uint8_t)(~(sum & 0xFF));
}

void dgz_frame::toBufs(uint8_t *bufs)
{
    bufs[0] = header[0];
    bufs[1] = header[1];
    bufs[2] = length;
    bufs[3] = type;
    bufs[4] = firstAddress;
    memcpy(bufs + 5, data, length - 8);
    bufs[length - 3] = checksum;
    bufs[length - 2] = tail[0];
    bufs[length - 1] = tail[1];
}

uint8_t dgz_frame::Length()
{
    return length;
}