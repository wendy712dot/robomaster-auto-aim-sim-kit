#ifndef VIDEO_READER_HPP
#define VIDEO_READER_HPP

#include <opencv2/opencv.hpp>
#include <string>


class VideoReader
{

public:

    VideoReader(const std::string& path);

    bool isOpened();

    cv::Mat read();

    double getFPS() const;


private:

    cv::VideoCapture cap;

};


#endif