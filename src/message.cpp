#include "message/message.h"

#include <iostream>

#include <spdlog/spdlog.h>
#include <boost/crc.hpp>

bool get_message_packet(MessagePacket& packet, const std::array<uint8_t,22> &buffer) 
{
    if (buffer[0] != 0x2B || buffer[21] != 0x2A) 
    {
        spdlog::error("Error SOF {} or EOF {}", buffer[0], buffer[21]);
        return false;
    }
    packet.head = buffer[0];
    packet.number = (buffer[1]) | (buffer[2] << 8) | (buffer[3] << 16) | (buffer[4] << 24);
    for (int i = 0; i < 4; i++) 
    {
        packet.action[i] = (buffer[5 + i * 2]) | (buffer[6 + i * 2] << 8);
    }
    packet.button = (buffer[13]) | (buffer[14] << 8);
    packet.reserve = (buffer[15]) | (buffer[16] << 8);
    packet.crc32 = (buffer[17]) | (buffer[18] << 8) | (buffer[19] << 16) | (buffer[20] << 24);
    packet.tail = buffer[21];

    // spdlog::info("MessagePacket: head: {}, number: {}, action: {}, button: {}, reserve: {}, crc32: {}, tail: {}", 
    //     packet.head, packet.number, packet.action[0], packet.button, packet.reserve, packet.crc32, packet.tail);
    
    boost::crc_32_type crc32;
    crc32.process_bytes(buffer.data(), 17);
    if (crc32.checksum() != packet.crc32) 
    {
        spdlog::error("CRC32 error");
        return false;
    }
    return true;
}
    