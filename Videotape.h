//
// Created by rm on 20-9-16.
//

#ifndef AUTOMATIC_AIMING_SYSTEM_VIDEOTAPE_H
#define AUTOMATIC_AIMING_SYSTEM_VIDEOTAPE_H


#include <iostream>
#include <assert.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <fstream>
#include <thread>
#include <mutex>
#include <map>
#include <unistd.h>


class SaveFrame
{
public:
    SaveFrame(std::string FilePath = "/home/rm/桌面/Automatic aiming system/Video/" ,
            cv::Size sWH = cv::Size(1280, 1024),
            float frameRate = 211
            );

    ~SaveFrame();


    bool saveFrame(cv::Mat frame);

public:

    bool savepicture(cv::Mat src);

    std::string getFileName();

    void getPictureName(std::string& a );

    std::string fileName = "";


public:

    cv::VideoWriter capsaveVideo;
};



#endif //AUTOMATIC_AIMING_SYSTEM_VIDEOTAPE_H
