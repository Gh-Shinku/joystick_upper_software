#include "tcp_socket/tcp_socket.h"

#include <iostream>
#include <chrono>
#include <thread>

using namespace boost::asio;
using ip::tcp;
using std::cout;
using std::endl;

TCPSocketClient::TCPSocketClient(boost::asio::io_context &context_arg, std::function<void(const Message::MessagePacket &)> on_receive) : context(context_arg),
                                                                                                                                         socket(context),
                                                                                                                                         on_receive(on_receive),
                                                                                                                                         logger("TCPSocketClient")
{
}

TCPSocketClient::~TCPSocketClient()
{
    socket.close();
}

void TCPSocketClient::connect(const std::string &host, const std::string &port)
{
    this->host = host, this->port = port;
    tcp::resolver resolver(context);
    auto endpoints = resolver.resolve(host, port);
    timer = std::make_unique<boost::asio::steady_timer>(context);
    timer->expires_after(std::chrono::seconds(1));
    timer->async_wait([this](boost::system::error_code ec)
                      {
                         if (ec != boost::asio::error::operation_aborted)
                         {
                             socket.close();
                             logger.warn("Connection timeout.");
                         } });
    boost::asio::async_connect(socket, endpoints, [this](boost::system::error_code ec, tcp::endpoint)
                               {
                                    if (!ec)
                                    {
                                        logger.info("Connected to server.");
                                        timer->cancel();
                                        is_connected = true;
                                        start_communication();
                                    } });
}

void TCPSocketClient::start_communication()
{
    send_sync_byte();
}

void TCPSocketClient::send_sync_byte()
{
    std::array<char, 1> sync_byte = {0x01};
    boost::asio::write(socket, boost::asio::buffer(sync_byte));

    socket.async_read_some(boost::asio::buffer(buffer),
                           [this](boost::system::error_code ec, std::size_t length)
                           {
                               if (!ec)
                               {
                                   Message::MessagePacket message_packet;
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

bool TCPSocketClient::is_connected_to_server() const
{
    return is_connected;
}
