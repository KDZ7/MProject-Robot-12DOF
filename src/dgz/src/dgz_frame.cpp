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
    : Length(size_data + 8),
      Type(type),
      FirstAddress(firstAddress)
{
    Data = new uint8_t[size_data];
    memcpy(Data, data, size_data);
    this->Checksum = this->CChecksum();
}

dgz_frame::~dgz_frame()
{
    delete[] Data;
}

uint8_t dgz_frame::CChecksum()
{
    uint8_t sum = 0;
    sum += Length;
    sum += Type;
    sum += FirstAddress;
    for (uint8_t i = 0; i < Length - 8; i++)
        sum += Data[i];

    return ~(sum & 0xFF);
}

void dgz_frame::toBufs(uint8_t *bufs)
{
    bufs[0] = Header[0];
    bufs[1] = Header[1];
    bufs[2] = Length;
    bufs[3] = Type;
    bufs[4] = FirstAddress;
    memcpy(bufs + 5, Data, Length - 8);
    bufs[Length - 3] = Checksum;
    bufs[Length - 2] = Tail[0];
    bufs[Length - 1] = Tail[1];
}