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
        return idx <= 725;
    }

    std::string image_dir_path = "";
    size_t idx = 1;
};


struct DebugImages
{
    cv::Mat read_next()
    {
        cv::Mat image = cv::Mat::zeros(1000, 1000, CV_32F);

        // select a region of interest
        cv::Mat pRoi = image(cv::Rect(rect_pos.x + idx, rect_pos.y, 20, 20));

        // set roi to some rgb colour   
        pRoi.setTo(0.5);
        // more region with gredient.
        cv::GaussianBlur( image, image, cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT );
        
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
        return idx <= 725;
    }

    size_t idx = 0; 
    cv::Point2f rect_pos = {160, 130};
};


int main(int argc, char *argv[])
{
    const std::string image_dir_path = "/home/yimu/Desktop/yimu-blog/data/image_seqence_basketball";
    ImageIO imageio(image_dir_path);
    
    // DebugImages imageio;

    LucasKanadaTracker lk_tracker;

    while(imageio.has_more())
    {
        const auto image = imageio.read_next();

        constexpr int IMAGE_DT_MS = 1;
        lk_tracker.track(image);
        
        // lk_tracker.show_features("features", IMAGE_DT_MS);
        lk_tracker.show_features_with_covariance("features", IMAGE_DT_MS);
    }

    return 0;
}