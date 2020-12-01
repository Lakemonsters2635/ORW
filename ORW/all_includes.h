#pragma once

//#define _CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>

#include <Windows.h>

#include <gl/glew.h>
#include <GLFW/glfw3.h>

#include "vendor/imgui/imgui_impl_glfw.h"
#include "vendor/imgui/imgui_impl_opengl3.h"

#include "vendor/implot/implot.h"
#include "vendor/implot/implot_internal.h"

#include "vendor/RealSense/rs_example.h"

#include <opencv2/opencv.hpp>
//#include <opencv2/imgproc/types_c.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
