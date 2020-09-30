//
// Created by rm on 20-9-16.
//
#include "Videotape.h"

SaveFrame::SaveFrame(std::string FilePath, cv::Size sWH, float frameRate)
{
    fileName = getFileName() + ".avi";
    capsaveVideo.open(FilePath + fileName,
            cv::VideoWriter::fourcc('M','P','4','2'),
            frameRate,
            sWH);
}


SaveFrame::~SaveFrame()
{
    std::cout<< "111" << std::endl;
    capsaveVideo.release();
}

bool SaveFrame::saveFrame(cv::Mat frame)
{
    capsaveVideo.write(frame);
    return true;
}

bool SaveFrame::savepicture(cv::Mat src)
{
    std::string i;
    std::string label = "Six_";
    for (int j = 0; j <200 ; j++)
    {
        //std::cout << j << std::endl;
        i = std::to_string(j);
        std::string FilePath = "/home/rm/caffe/examples/mydatabase/number_rain/";
        fileName = label + i + ".jpg";
        cv::imwrite(FilePath + fileName, src);
    }
    return true;
}

void  SaveFrame::getPictureName(std::string &a)
{
    std::string PictureName;
    for (int j = 0; j <200 ; ++j)
    {
        a = std::to_string(j);
    }
}



std::string SaveFrame::getFileName()
{
    time_t currentTime;
    struct tm*timePointer;
    time(&currentTime);
    timePointer = localtime(&currentTime);

    return std::to_string(timePointer->tm_year + 1900) + "-" +
            std::to_string(timePointer->tm_mon + 1) + "-" +
            std::to_string(timePointer->tm_mday) + "-" +
            std::to_string(timePointer->tm_hour) + ":" +
            std::to_string(timePointer->tm_min) + ":" +
            std::to_string(timePointer->tm_sec);
}