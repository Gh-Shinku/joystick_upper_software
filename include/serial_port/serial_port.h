#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <boost/asio.hpp>
#include <array>
#include <string>

class SerialPortClient {
private:
    boost::asio::io_context &context;
    boost::asio::serial_port serial;
    std::string read_buffer_;

    void read_start();
    void read_packet();

public:
    SerialPortClient(boost::asio::io_context &context,const std::string& port);
    void run();
};

#endif /* SERIAL_PORT_H_ */
