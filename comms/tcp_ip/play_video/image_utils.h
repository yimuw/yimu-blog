#include <iostream>

#include <opencv2/opencv.hpp>


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

// https://stackoverflow.com/questions/4170745/serializing-opencv-mat-vec3f/21444792#21444792
// Dosen't consider platfrom.
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

    *reinterpret_cast<uint32_t *>(&serialized_data.at(0)) = cols;
    *reinterpret_cast<uint32_t *>(&serialized_data.at(4)) = rows;
    *reinterpret_cast<uint32_t *>(&serialized_data.at(8)) = type;
    // mat.create(rows, cols, type);

    const size_t data_size = rows * cols * mat.elemSize();
    serialized_data.insert(serialized_data.end(), mat.ptr(), mat.ptr() + data_size);

	return serialized_data; 
}


cv::Mat deserialize_cvmat(const char * const serialized_data)
{
    int cols, rows, type;
    cols = *reinterpret_cast<const uint32_t *>(&serialized_data[0]); 
    rows = *reinterpret_cast<const uint32_t *>(&serialized_data[4]);
    type = *reinterpret_cast<const uint32_t *>(&serialized_data[8]);

    cv::Mat res_mat;
    res_mat.create(rows, cols, type);

    const size_t data_size = rows * cols * res_mat.elemSize();

    auto imdata_start = serialized_data + 4 * 3;
    auto imdata_end = serialized_data + 4 * 3 + data_size;

    std::copy(imdata_start, imdata_end, res_mat.ptr());

	return res_mat; 
}

struct VideoPlayControl
{
    enum class ControlType : int32_t
    {
        none = 0,
        pause ,
        play
    };

    ControlType control = ControlType::none;
};