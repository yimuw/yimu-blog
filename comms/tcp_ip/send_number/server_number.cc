#include "../tcp_server.h"
#include "message_types.h"

using namespace comms;

int main(int argc, char const* argv[])
{
    print_current_ip();

    control::set_gracefully_exit();
    
    const Arguments args = tcp_ip_argparse(argc, argv);
    TcpConfig tcp_config{ args.port, args.ip };

    TcpServer<double, int32_t> tcp_server(tcp_config);

    if (tcp_server.initialize() == false) {
        return 0;
    }

    for (double i = 0; i < 1000; i += 0.5) {
        usleep(200 * 1e3);

        std::cout << "publish int: " << i << std::endl;
        tcp_server.send_to_peer(i);

        int32_t received_value = -1;
        while (tcp_server.recv_from_peer(received_value)) {
            std::cout << "received: " << received_value << std::endl;
        }

        if (control::program_exit() == true) {
            break;
        }
    }
}
