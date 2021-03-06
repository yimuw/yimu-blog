#include "../tcp_client.h"
#include "message_types.h"

using namespace comms;

int main(int argc, char const* argv[])
{
    control::set_gracefully_exit();

    const Arguments args = tcp_ip_argparse(argc, argv);
    TcpConfig tcp_config{ args.port, args.ip };

    TcpClient<int32_t, double> tcp_client(tcp_config);

    if (tcp_client.initialize() == false) {
        return 0;
    }

    for (int32_t i = 1000; i < 2000; ++i) {
        usleep(1500 * 1e3);

        std::cout << "publish int: " << i << std::endl;
        tcp_client.send_to_peer(i);

        double received_value = -1;
        while (tcp_client.recv_from_peer(received_value)) {
            std::cout << "received: " << received_value << std::endl;
        }

        if (control::program_exit() == true) {
            break;
        }
    }
}
