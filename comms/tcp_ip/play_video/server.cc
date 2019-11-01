#include "../tcp_server.h"
#include "image_utils.h"

using namespace comms;

constexpr size_t IMAGE_SIZE_BYTE = 995340;

int main(void)
{
    control::gracefully_exit();

    TcpServerConfig tcp_config{"3490", "192.168.0.138"};
    TcpServer<IMAGE_SIZE_BYTE> tcp_server(tcp_config);

    tcp_server.initailize();

    ImageIO image_io("/home/yimu/Desktop/yimu-blog/data/image_seqence_basketball");
    while(image_io.has_more())
    {
        usleep(100 * 1e3);
        const auto image = image_io.read_next();
        std::vector<char> serialized_image = serialize_cvmat(image);

        assert(serialized_image.size() == IMAGE_SIZE_BYTE && "queue size mismatch!");

        tcp_server.publish(&serialized_image[0]);

        if(control::problem_exit() == true)
        {
            break;
        }
    }

    control::quit = true;
}
