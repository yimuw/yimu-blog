#include "../tcp_client.h"
#include "image_utils.h"

using namespace comms;

constexpr size_t IMAGE_SIZE_BYTE = 995340;

struct ImageReceiver
{
    using image_client = TcpClient<IMAGE_SIZE_BYTE>;

    ImageReceiver()
    {
        TcpClientConfig tcp_config{"3490", "172.31.99.184"};
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

    std::shared_ptr<image_client> tcp_client_ptr;
};

int main(void)
{
    control::set_gracefully_exit();

    ImageReceiver image_receiver;

    while(true)
    {
        cv::namedWindow("image from tcp", cv::WINDOW_AUTOSIZE );

		if(image_receiver.has_data())
		{
			cv::Mat image = image_receiver.get_mat();
			
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

    control::quit = true;
    exit(0);
}
