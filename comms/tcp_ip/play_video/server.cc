#include "../tcp_server.h"
#include "messages_types.h"


using namespace comms;


int main(void)
{
    control::set_gracefully_exit();

    TcpConfig tcp_config{"3491", "AI_PASSIVE"};
    TcpServer<message::Frame, message::VideoControl> tcp_server(tcp_config);
    tcp_server.initailize();

    ImageIO image_io("/home/yimu/Desktop/yimu-blog/data/image_seqence_basketball");

    int count = 0;
    while(image_io.has_more())
    {
        usleep(20 * 1000);
        message::Frame frame;
        frame.frame_id = count++;
        frame.image = image_io.read_next();

        tcp_server.send_to_peer(frame);

        if(control::program_exit() == true)
        {
            break;
        }
    }

    control::quit = true;
    exit(0);
}
