#ifndef MESSAGE_H_
#define MESSAGE_H_

#include <cstdint>
#include <array>
#include "logger/logger.h"

namespace Message
{
    struct MessagePacket
    {
        char head;
        uint32_t number;
        std::array<int16_t, 4> action;
        uint16_t button;
        uint16_t reserve;
        uint32_t crc32;
        char tail;
    };
    inline LoggerImpl logger("Message");
    bool get_message_packet(MessagePacket &packet, const std::array<uint8_t, 22> &buffer);
}

#endif /* MESSAGE_H_ */