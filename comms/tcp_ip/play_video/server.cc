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

// play_video_server --ip AI_PASSIVE --port 3491 --image-dir /home/yimu/Desktop/yimu-blog/data/image_seqence_basketball
int main(int argc, char const* argv[])
{
    print_current_ip();

    control::set_gracefully_exit();
    
    const Arguments args = tcp_ip_image_server_argparse(argc, argv);
    TcpConfig tcp_config{ args.port, args.ip };
    
    TcpServer<message::Frame, message::VideoControl> tcp_server(tcp_config);
    if(!tcp_server.initialize())
    {
        return 0;
    }

    ImageIO image_io(args.image_dir);

    int count = 0;
    while (image_io.has_more()) {
        usleep(20 * 1000);
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
    return 0;
}
