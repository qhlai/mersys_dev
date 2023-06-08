#pragma once
// #ifndef VALUE_REDEFINE
#include <iostream>
#include <stdio.h>
#include <stdint.h>

#include <opencv2/opencv.hpp>
namespace read_parm {
template<typename T> inline static T GetValFromYaml(std::string path, std::string paramname)
{
    cv::FileStorage fSettings(path, cv::FileStorage::READ);

    if(!fSettings.isOpened())
    {
       std::cerr << "Failed to open config file at: " << path << std::endl;
       exit(-1);
    }

    const double val = fSettings[paramname];
    return (T)val;
}

inline static std::string GetStringFromYaml(std::string path, std::string paramname)
{
    cv::FileStorage fSettings(path, cv::FileStorage::READ);

    if(!fSettings.isOpened())
    {
       std::cerr << "Failed to open config file at: " << path << std::endl;
       exit(-1);
    }

    const std::string val = fSettings[paramname];
    return val;
}
}