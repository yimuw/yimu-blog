#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "lucas_kanada_algorithm.h"


struct ImageIO
{
    ImageIO(const std::string _image_dir_path)
        : image_dir_path(_image_dir_path)
    {
    }

    cv::Mat read_next()
    {
        char filename[9];
        sprintf(filename, "%04zu.jpg", idx);
        const std::string path = image_dir_path + "/" + std::string(filename);
        std::cout << "loading from: " << path << std::endl;

        cv::Mat image = cv::imread(path);
        // To gray
        cv::cvtColor(image, image, CV_BGR2GRAY);
        // To float. Not nessesary
        image.convertTo(image, CV_32F, 1/255.0); 
        
        if(false)
        {
            imshow( "write process", image );
            cv::waitKey(1);
        }

        ++idx;
        return image;
    }

    bool has_more() const
    {
        return idx < 700;
    }

    std::string image_dir_path = "";
    size_t idx = 200;
};


int main(int argc, char *argv[])
{
    const std::string image_dir_path = "/home/yimu/Desktop/yimu-blog/data/image_seqence_basketball";
    ImageIO imageio(image_dir_path);

    LucasKanadaTracker lk_tracker;

    while(imageio.has_more())
    {
        const auto image = imageio.read_next();
        lk_tracker.track(image);
        lk_tracker.show_features("features", -1);
    }

    return 0;
}