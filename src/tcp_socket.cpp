#include "tcp_socket/tcp_socket.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace boost::asio;
using ip::tcp;
using std::cout;
using std::endl;

TCPSocketClient::TCPSocketClient(boost::asio::io_context &context_arg, std::function<void(const MessagePacket&)> on_receive):
    context(context_arg), 
    socket(context), 
    on_receive(on_receive)
{

}

TCPSocketClient::~TCPSocketClient()
{
    socket.close();
}

void TCPSocketClient::connect(const std::string& host, const std::string& port) {
    tcp::resolver resolver(context);
    auto endpoints = resolver.resolve(host, port);
    boost::asio::connect(socket, endpoints);
    start_communication();
}

void TCPSocketClient::start_communication() {
    send_sync_byte();
    // 注意：实际项目中，你可能需要循环或在更复杂的逻辑中处理接收和发送
}

void TCPSocketClient::send_sync_byte() {
    // 发送一个字节给服务器以请求数据
    std::array<char, 1> sync_byte = {0x01};
    boost::asio::write(socket, boost::asio::buffer(sync_byte));

    // 然后尝试读取数据
    socket.async_read_some(boost::asio::buffer(buffer),
        [this](boost::system::error_code ec, std::size_t length) {
            if (!ec) {
                MessagePacket message_packet;
                if (get_message_packet(message_packet, buffer))
                {
                    if (on_receive)
                    {
                        on_receive(message_packet);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                send_sync_byte();
            }
        });
}

void TCPSocketClient::run()
{
    context.run();
}
