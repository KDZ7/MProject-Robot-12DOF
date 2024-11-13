#ifndef DGZ__FRAME_HPP_
#define DGZ__FRAME_HPP_

#include <stdint.h>

class dgz_frame
{
public:
    uint8_t Header[2] = {0x55, 0x00};
    uint8_t Length;
    uint8_t Type;
    uint8_t FirstAddress;
    uint8_t *Data;
    uint8_t Checksum;
    uint8_t Tail[2] = {0x00, 0xAA};

    dgz_frame(uint8_t type,
              uint8_t firstAddress,
              uint8_t *data,
              uint8_t size_data);
    ~dgz_frame();
    uint8_t CChecksum();
    void toBufs(uint8_t *bufs);
};
#endif // DGZ__FRAME_HPP_