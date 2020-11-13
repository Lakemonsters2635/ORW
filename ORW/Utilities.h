#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

int PostProcess();
void DoHistograms(const cv::Mat& image);

