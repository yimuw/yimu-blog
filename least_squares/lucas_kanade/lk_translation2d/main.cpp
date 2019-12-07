#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "lucas_kanada_algorithm.h"

struct ImageIO {
    ImageIO(const std::string _image_dir_path)
        : image_dir_path(_image_dir_path)
    {
    }

    bool file_exists(const std::string& name)
    {
        std::ifstream f(name.c_str());
        return f.good();
    }

    cv::Mat read_next()
    {
        char filename[9];
        sprintf(filename, "%04zu.jpg", idx);
        const std::string path = image_dir_path + "/" + std::string(filename);

        std::cout << "loading from: " << path << std::endl;
        if (file_exists(path) == false) {
            throw std::runtime_error("image not exist");
        }

        cv::Mat image = cv::imread(path);
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

struct DebugImages {
    cv::Mat read_next()
    {
        cv::Mat image = cv::Mat::zeros(1000, 1000, CV_32F);

        // select a region of interest
        cv::Mat pRoi = image(cv::Rect(rect_pos.x + idx, rect_pos.y, 20, 20));

        // set roi to some rgb colour
        pRoi.setTo(0.5);
        // more region with gradient.
        cv::GaussianBlur(image, image, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);

        if (false) {
            imshow("write process", image);
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
    cv::Point2f rect_pos = { 160, 130 };
};

bool read_command_line(
    int argc,
    char* argv[],
    std::string& image_dir_path,
    bool& show_covariance)
{
    if (argc < 2) {
        throw std::runtime_error("please input the path to images folder");
    }
    // "/home/yimu/Desktop/yimu-blog/data/image_seqence_basketball";
    image_dir_path = argv[1];

    show_covariance = false;
    if (argc == 3) {
        show_covariance = (std::string(argv[2]) == std::string("show_cov"));
    }
    return true;
}

int main(int argc, char* argv[])
{
    std::string image_dir_path;
    bool show_covariance;
    read_command_line(argc, argv, image_dir_path, show_covariance);

    ImageIO imageio(image_dir_path);
    LucasKanadaTracker lk_tracker;

    // WARNING: hard-code size & path
    const std::string video_save_path = show_covariance ? "lk_cov.avi" : "lk.avi";
    cv::VideoWriter video;
    // hack to get image size
    {
        ImageIO image_size(image_dir_path);
        cv::Mat test = image_size.read_next();
        video = cv::VideoWriter(video_save_path, CV_FOURCC('M', 'J', 'P', 'G'), 24, cv::Size(576, 432));
    }

    while (imageio.has_more()) {
        const auto image = imageio.read_next();

        lk_tracker.track(image);

        cv::Mat frame_with_tracks;
        if (show_covariance) {
            frame_with_tracks = lk_tracker.show_features_with_covariance("features");
        } else {
            frame_with_tracks = lk_tracker.show_features("features");
        }
        video.write(frame_with_tracks);
    }
    video.release();
    std::cout << "video saved at " << video_save_path << std::endl;

    return 0;
}