#include "../tcp_server.h"
#include "messages_types.h"

using namespace comms;

void video_play_control(TcpServer<message::Frame, message::VideoControl>& tcp_server)
{
    // stop playing when recv a control message
    message::VideoControl control_message;
    if (tcp_server.recv_from_peer(control_message)) {
        // block here until next control message arrive
        while (tcp_server.recv_from_peer(control_message) == false
            && control::program_exit() == false) {
            usleep(1000);
        }
    }
}

int main(void)
{
    control::set_gracefully_exit();

    TcpConfig tcp_config{ "3491", "AI_PASSIVE" };
    TcpServer<message::Frame, message::VideoControl> tcp_server(tcp_config);
    tcp_server.initialize();

    ImageIO image_io("/home/yimu/Desktop/yimu-blog/data/image_seqence_basketball");

    int count = 0;
    while (image_io.has_more()) {
        usleep(50 * 1000);
        message::Frame frame;
        frame.frame_id = count++;
        frame.image = image_io.read_next();

        tcp_server.send_to_peer(frame);

        video_play_control(tcp_server);

        if (control::program_exit() == true) {
            break;
        }
    }

    control::quit = true;
    exit(0);
}
