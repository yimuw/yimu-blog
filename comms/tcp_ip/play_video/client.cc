#include "../tcp_client.h"
#include "image_utils.h"

using namespace comms;

constexpr size_t IMAGE_SIZE_BYTE = 995340;
const auto ip = "192.168.1.9";
const auto port = "3490";

struct ImageReceiver
{
    using image_client = TcpClient<IMAGE_SIZE_BYTE>;

    ImageReceiver()
    {
        TcpClientConfig tcp_config{port, ip};
        tcp_client_ptr = std::make_shared<image_client>(tcp_config);

        tcp_client_ptr->initailize();
    }

    bool has_data()
    {
        return tcp_client_ptr->has_data();
    }

    cv::Mat get_mat()
    {
        std::vector<char> serialized_image(IMAGE_SIZE_BYTE);
		tcp_client_ptr->read(&serialized_image[0]);
		cv::Mat image = deserialize_cvmat(&serialized_image[0]);
        return image;
    }

    void receive_and_display_images()
    {
        cv::namedWindow("image from tcp", cv::WINDOW_AUTOSIZE );
        while(control::program_exit() == false)
        {
            if(has_data())
            {
                cv::Mat image = get_mat();
                
                cv::imshow("image from tcp", image);
                cv::waitKey(1);
            }

            if(control::program_exit() == true)
            {
                break;
            }

            // polling
            usleep(50);
        }
    }

    std::shared_ptr<image_client> tcp_client_ptr;
};


int main(void)
{
    control::set_gracefully_exit();

    ImageReceiver image_receiver;

    std::thread receive_and_display_images([&image_receiver]()
    {
        image_receiver.receive_and_display_images();
    });

    if(receive_and_display_images.joinable())
    {
        receive_and_display_images.join(); 
    }
    
    control::quit = true;
    exit(0);
}
