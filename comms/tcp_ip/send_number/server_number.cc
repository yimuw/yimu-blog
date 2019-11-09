#include "../tcp_server.h"
#include "message_types.h"

using namespace comms;

int main(void)
{
    control::set_gracefully_exit();

    TcpConfig tcp_config{"3491", "AI_PASSIVE"};

    TcpServer<int32_t> tcp_server(tcp_config);

    if(tcp_server.initailize() == false)
    {
        return 0;
    }

    for(int32_t i = 0; i < 1000; ++i)
    {
        usleep(200 * 1e3);

        std::cout << "publish int: " << i << std::endl; 
        tcp_server.send_to_peer(i);

        int32_t received_value = -1;
        while(tcp_server.recv_from_peer(received_value))
        {
            std::cout << "received: " << received_value << std::endl;
        }

        if(control::program_exit() == true)
        {
            break;
        }
    }

}
