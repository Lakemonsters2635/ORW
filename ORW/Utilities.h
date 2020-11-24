#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include "CSettings.h"


class texture;
struct glfw_state;
class window;

void DisplayFrame(rs2::frame& frame, const char* Title, texture& frame_texture);

void DisplayTriangle2();
void DisplayTriangle3(float xSize, float ySize, float z);

extern CSettings g_settings;
