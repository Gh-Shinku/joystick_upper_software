#ifndef TCP_SOCKET_H_
#define TCP_SOCKET_H_

#include <boost/asio.hpp>
#include <string>
#include <array>
#include "message/message.h"


class TCPSocketClient {
private:
    boost::asio::io_context &context;
    boost::asio::ip::tcp::socket socket;
    std::array<uint8_t, 22> buffer;
    std::function<void(const MessagePacket&)> on_receive;

    void start_communication();
    void send_sync_byte();

public:
    TCPSocketClient(boost::asio::io_context &context, std::function<void(const MessagePacket &)> on_receive = nullptr);
    ~TCPSocketClient();
    void connect(const std::string& host, const std::string& port);
    void run();
};

#endif /* TCP_SOCKET_H_ */
