#include "../tcp_server.h"
#include "image_utils.h"

using namespace comms;

constexpr size_t IMAGE_SIZE_BYTE = 995340;

// TODO: trait for each type. e.g. <type>::size(), <type>::serialize()...
struct ImagePublisher
{
    using image_server = TcpServer<IMAGE_SIZE_BYTE>;

    ImagePublisher()
    {
        TcpServerConfig tcp_config{"3490", "172.31.99.184"};
        tcp_server_ptr = std::make_shared<image_server>(tcp_config);

        tcp_server_ptr->initailize();
    }

    bool publish(const cv::Mat &image)
    {
        std::vector<char> serialized_image = serialize_cvmat(image);
        assert(serialized_image.size() == IMAGE_SIZE_BYTE && "queue size mismatch!");
        const bool status = tcp_server_ptr->publish(&serialized_image[0]);
        return status;
    }

    std::shared_ptr<image_server> tcp_server_ptr;
};

int main(void)
{
    control::set_gracefully_exit();

    ImagePublisher image_publisher;

    ImageIO image_io("/home/yimu/Desktop/yimu-blog/data/image_seqence_basketball");
    while(image_io.has_more())
    {
        usleep(20 * 1000);
        const auto image = image_io.read_next();
        
        image_publisher.publish(image);

        if(control::program_exit() == true)
        {
            break;
        }
    }

    control::quit = true;
    exit(0);
}
