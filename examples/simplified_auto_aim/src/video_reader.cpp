#include "video_reader.hpp"


VideoReader::VideoReader(const std::string& path)
{
    cap.open(path);
}


bool VideoReader::isOpened()
{
    return cap.isOpened();
}


cv::Mat VideoReader::read()
{
    cv::Mat frame;

    cap.read(frame);

    return frame;
}

double VideoReader::getFPS() const
{
    return cap.get(
        cv::CAP_PROP_FPS
    );
}