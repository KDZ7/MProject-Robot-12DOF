#ifndef DGZ__FRAME_HPP_
#define DGZ__FRAME_HPP_

#include <memory>
#include <stdint.h>

class dgz_frame
{
public:
    dgz_frame(uint8_t type,
              uint8_t firstAddress,
              uint8_t *data,
              uint8_t size_data);

private:
    uint8_t header[2] = {0x55, 0x00};
    uint8_t length;
    uint8_t type;
    uint8_t firstAddress;
    std::unique_ptr<uint8_t[]> data;
    uint8_t checksum;
    uint8_t tail[2] = {0x00, 0xAA};
    uint8_t CChecksum();

public:
    void toBufs(uint8_t *bufs);
    uint8_t Length();
};
#endif // DGZ__FRAME_HPP_