#include "../tcp_client.h"
#include "messages_types.h"

using namespace comms;

void get_keybroad_input()
{
    // detect enter
    while (getchar() != '\n')
        ;
    std::cout << "get keybroad enter" << std::endl;
}

int main(void)
{
    control::set_gracefully_exit();

    TcpConfig tcp_config{ "3491", "yimu-mate" };
    TcpClient<message::VideoControl, message::Frame> tcp_client(tcp_config);
    if (tcp_client.initialize() == false) {
        return 0;
    }

    // TODO: capture tcp_client is not safe.
    // TODO: getchar block control::program_exit()
    std::thread keybroad_input_thread(
        [&tcp_client]() {
            while (control::program_exit() == false) {
                get_keybroad_input();
                message::VideoControl control;
                control.control = message::VideoControl::ControlType::change_status;
                tcp_client.send_to_peer(control);
            }
        });

    while (control::program_exit() == false) {
        message::Frame frame;
        if (tcp_client.recv_from_peer(frame)) {
            std::cout << "recv image id: " << frame.frame_id << std::endl;
            cv::imshow("image from tcp", frame.image);
            cv::waitKey(1);
        }

        usleep(1000);
    }

    keybroad_input_thread.join();
}
