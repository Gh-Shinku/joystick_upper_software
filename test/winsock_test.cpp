#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <array>

#include "message/message.h"

#pragma comment(lib, "Ws2_32.lib")

int main() {
    WSADATA wsaData;
    SOCKET sock = INVALID_SOCKET;
    struct sockaddr_in server;

    // 初始化Winsock
    if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed." << std::endl;
        return 1;
    }

    // 创建socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Socket creation failed." << std::endl;
        WSACleanup();
        return 1;
    }

    // 设置服务器地址
    server.sin_addr.s_addr = inet_addr("192.168.4.1");
    server.sin_family = AF_INET;
    server.sin_port = htons(3456);

    // 连接到服务器
    if (connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0) {
        std::cerr << "Connect failed." << std::endl;
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    std::cout << "Connected to the server." << std::endl;

    // 发送一个字节数据
    char send_data = '1'; // 示例：发送字符'1'
    if (send(sock, &send_data, 1, 0) < 0) {
        std::cerr << "Send failed." << std::endl;
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    std::cout << "Data sent." << std::endl;

    // 接收22个字节的数据
    std::array<uint8_t,22> recv_data;
    int received = recv(sock,(char *)recv_data.data(), 22, 0);
    if (received < 0) {
        std::cerr << "Receive failed." << std::endl;
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    MessagePacket msg;
    get_message_packet(msg,recv_data);
    std::cout << "Received " << received << " bytes." << std::endl;

    // 关闭socket和清理Winsock
    closesocket(sock);
    WSACleanup();

    return 0;
}