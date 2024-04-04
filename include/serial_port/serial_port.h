#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <boost/asio.hpp>
#include <array>
#include <string>

#include "message/message.h"

class SerialPortClient
{
private:
    boost::asio::io_context &context;
    boost::asio::serial_port serial;
    std::string read_buffer_;
    std::function<void(const MessagePacket &)> on_receive;

    void read_start();
    void read_packet();

public:
    SerialPortClient(boost::asio::io_context &context, const std::string &port, std::function<void(const MessagePacket &)> on_receive);
    void run();
};

#endif /* SERIAL_PORT_H_ */
