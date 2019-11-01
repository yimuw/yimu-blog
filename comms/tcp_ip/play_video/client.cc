#include "../tcp_client.h"
#include "image_utils.h"

using namespace comms;

constexpr size_t IMAGE_SIZE_BYTE = 995340;

int main(void)
{
    control::gracefully_exit();

    TcpClientConfig tcp_config{"3490", "192.168.0.138"};
    TcpClient<IMAGE_SIZE_BYTE> tcp_client(tcp_config);

    tcp_client.initailize();

    while(true)
    {
		if(tcp_client.has_data())
		{
			std::vector<char> serialized_image(IMAGE_SIZE_BYTE);
			tcp_client.read(&serialized_image[0]);

			cv::Mat image = deserialize_cvmat(&serialized_image[0]);
			cv::imshow("image from tcp", image);
		}

		if(control::problem_exit() == true)
        {
            break;
        }
    }

    control::quit = true;
}
