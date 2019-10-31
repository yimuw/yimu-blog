#include "tcp_server.h"


using namespace comms;

int main(void)
{
    TcpServerConfig tcp_config{"3490", "192.168.0.138"};

    constexpr size_t MESSAGE_SIZE_BYTE = 20;
    char test_data[MESSAGE_SIZE_BYTE];
    TcpServer<MESSAGE_SIZE_BYTE> tcp_server(tcp_config);

    tcp_server.initailize();

    for(int i = 0; i < 1000; ++i)
    {
        sleep(1);

        sprintf (test_data, "%d", i);
        std::cout << "publish: " << test_data << std::endl;
        tcp_server.publish(test_data);
    }

}
