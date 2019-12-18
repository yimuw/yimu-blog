#include <iostream>

#include "../comms_utils.h"
#include "../serialization.h"
#include <opencv2/opencv.hpp>

struct ImageIO {
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
        image.convertTo(image, CV_32F, 1 / 255.0);

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

    std::string image_dir_path = "";
    size_t idx = 1;
};

// https://stackoverflow.com/questions/4170745/serializing-opencv-mat-vec3f/21444792#21444792
// doesn't consider platfrom.
std::vector<char> serialize_cvmat(const cv::Mat& mat)
{
    std::vector<char> serialized_data;
    // 4 int32_t
    serialized_data.resize(4 * 3);

    int cols, rows, type;
    bool continuous;

    cols = mat.cols;
    rows = mat.rows;
    type = mat.type();
    continuous = mat.isContinuous();

    assert(continuous == true);

    *reinterpret_cast<uint32_t*>(&serialized_data.at(0)) = cols;
    *reinterpret_cast<uint32_t*>(&serialized_data.at(4)) = rows;
    *reinterpret_cast<uint32_t*>(&serialized_data.at(8)) = type;
    // mat.create(rows, cols, type);

    const size_t data_size = rows * cols * mat.elemSize();
    serialized_data.insert(serialized_data.end(), mat.ptr(), mat.ptr() + data_size);

    return serialized_data;
}

cv::Mat deserialize_cvmat(const char* const serialized_data)
{
    int cols, rows, type;
    cols = *reinterpret_cast<const uint32_t*>(&serialized_data[0]);
    rows = *reinterpret_cast<const uint32_t*>(&serialized_data[4]);
    type = *reinterpret_cast<const uint32_t*>(&serialized_data[8]);

    cv::Mat res_mat;
    res_mat.create(rows, cols, type);

    const size_t data_size = rows * cols * res_mat.elemSize();

    auto imdata_start = serialized_data + 4 * 3;
    auto imdata_end = serialized_data + 4 * 3 + data_size;

    std::copy(imdata_start, imdata_end, res_mat.ptr());

    return res_mat;
}

namespace comms {
namespace message {
    /////////////////////////////////////////////////////////////
    struct Frame {
        int32_t frame_id;
        cv::Mat image;
    };

    // Assuming fixed size image
    constexpr uint32_t TEST_IMAGE_SIZE = 995340;

    template <> // primary template
    constexpr uint32_t size_of_message<Frame>()
    {
        return sizeof(int32_t) + TEST_IMAGE_SIZE;
    }

    template <>
    void serialize<Frame>(const Frame& obj, char* const buffer)
    {
        // assume same platform
        // 1. frame_id
        memcpy(buffer, &obj.frame_id, sizeof(obj.frame_id));
        // 2. cv::mat
        // TODO: not efficient
        const std::vector<char> mat_serialized = serialize_cvmat(obj.image);
        assert(mat_serialized.size() == TEST_IMAGE_SIZE);
        memcpy(buffer + sizeof(obj.frame_id), &mat_serialized[0], sizeof(char) * mat_serialized.size());
    }

    template <>
    void deserialize<Frame>(char const* const buffer, Frame& obj)
    {
        // assume same platform
        // 1. frame_id
        memcpy(&obj.frame_id, buffer, sizeof(obj.frame_id));
        // 2. cv::mat
        obj.image = deserialize_cvmat(buffer + sizeof(obj.frame_id));
    }
    /////////////////////////////////////////////////////////////
    struct VideoControl {
        enum class ControlType : int32_t {
            none = 0,
            // pause if video is playing, resume if video is paused
            change_status
        };

        ControlType control = ControlType::none;
    };

    template <> // primary template
    constexpr uint32_t size_of_message<VideoControl>()
    {
        return sizeof(VideoControl);
    }

    template <>
    void serialize<VideoControl>(const VideoControl& obj, char* const buffer)
    {
        // assume same platform
        memcpy(buffer, &obj, sizeof(VideoControl));
    }

    template <>
    void deserialize<VideoControl>(char const* const buffer, VideoControl& obj)
    {
        // assume same platform
        memcpy(&obj, buffer, sizeof(VideoControl));
    }
}
}
