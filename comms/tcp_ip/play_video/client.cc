#include "../tcp_client.h"
#include "messages_types.h"

using namespace comms;

int main(void)
{
    control::set_gracefully_exit();

    TcpConfig tcp_config{"3491", "yimu-mate"};
    TcpClient<message::VideoControl, message::Frame> tcp_client(tcp_config);
    tcp_client.initailize();

    while(control::program_exit() == false)
    {
        message::Frame frame;
        if(tcp_client.recv_from_peer(frame))
        {
            std::cout << "recv image id: " << frame.frame_id << std::endl;
            cv::imshow("image from tcp", frame.image);
            cv::waitKey(1);
        }
        
        usleep(1000);
    }

    control::quit = true;
    exit(0);
}
