#include <boost/asio.hpp>
#include <iostream>
#include <array>

using boost::asio::ip::tcp;
namespace asio = boost::asio;

int main() {
    try {
        asio::io_context io_context;

        // 解析服务器地址和端口
        tcp::resolver resolver(io_context);
        auto endpoints = resolver.resolve("192.168.4.1", "3456");

        // 创建并连接socket
        tcp::socket socket(io_context);
        asio::connect(socket, endpoints);

        // 发送一个字节的数据
        std::array<char, 1> send_buf = {'1'};
        asio::write(socket, asio::buffer(send_buf));

        // 接收22个字节的数据
        std::array<char, 22> recv_buf;
        size_t len = asio::read(socket, asio::buffer(recv_buf));

        std::cout << "Received " << len << " bytes." << std::endl;
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}