#include "tcp_socket/tcp_socket.h"
#include "serial_port/serial_port.h"
#include <spdlog/spdlog.h>

int main() {
    try {
        boost::asio::io_context io_context;
        TCPSocketClient tcpClient(io_context);
        tcpClient.connect("192.168.4.1", "3456");

        SerialPortClient serialClient(io_context,"COM6");
        io_context.run();
    }
    catch (std::exception& e) {
        spdlog::error("Exception: {}", e.what());
    }

    return 0;
}
