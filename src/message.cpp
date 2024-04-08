#include "message/message.h"

#include <iostream>
#include <boost/crc.hpp>

#ifndef ROS2_ENVIROMENT
#include <spdlog/spdlog.h>
#endif

static std::array<int16_t, 4> last_action;

double low_pass_filter(double current, double target, double a)
{
    return current + (target - current) * a;
}

bool get_message_packet(MessagePacket &packet, const std::array<uint8_t, 22> &buffer)
{
    if (buffer[0] != 0x2B || buffer[21] != 0x2A)
    {
#ifdef SPDLOG_H
        spdlog::error("Error SOF {} or EOF {}", buffer[0], buffer[21]);
#endif
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

    boost::crc_32_type crc32;
    crc32.process_bytes(buffer.data(), 17);
    if (crc32.checksum() != packet.crc32)
    {
#ifdef SPDLOG_H
        spdlog::error("CRC32 error");
#endif
        return false;
    }

    for (int i = 0; i < 4; i++)
        last_action[i] = packet.action[i] = low_pass_filter(last_action[i], packet.action[i], 0.4);

    return true;
}
