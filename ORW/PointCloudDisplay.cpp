#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include "vendor/imgui/imgui_impl_glfw.h"
#include "vendor/imgui/imgui_impl_opengl3.h"
#include "vendor/RealSense/rs_example.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include "PointCloudDisplay.h"

PointCloudDisplay::PointCloudDisplay(const char* szTitle)
	: win(640, 480, szTitle)
{
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(win, app_state);
}


void PointCloudDisplay::DisplayPointCloud(rs2::points& points)
{
	auto oldCTX = glfwGetCurrentContext();
	glfwMakeContextCurrent((GLFWwindow*)win);
	glClear(GL_COLOR_BUFFER_BIT);

	draw_pointcloud(1280, 1024, app_state, points);
	/* Swap front and back buffers */
	glfwSwapBuffers((GLFWwindow*)win);

	/* Poll for and process events */
	glfwPollEvents();

	glfwMakeContextCurrent(oldCTX);
}

