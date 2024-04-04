#ifndef MESSAGE_H_
#define MESSAGE_H_

#include <cstdint>
#include <array>

struct MessagePacket
{
    char head;
    uint32_t number;
    int16_t action[4];
    uint16_t button;
    uint16_t reserve;
    uint32_t crc32;
    char tail;
};

bool get_message_packet(MessagePacket& packet, const std::array<uint8_t,22> &buffer);


#endif /* MESSAGE_H_ */