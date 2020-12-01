#pragma once

#include <librealsense2/rs.h>
#include "vendor/RealSense/rs_example.h"
#include "PCLUtils.h"
#include "CRANSAC.h"

void draw_pointcloud(float width, float height, glfw_state app_state, const std::vector <feature_ptr>& features);

struct byte3
{
	unsigned char r, g, b;
};

extern byte3 LayerColors[N_COLORS];


class PointCloudDisplay
{
public:
	PointCloudDisplay(const char* szTitle);

	void DisplayPointCloud(rs2::points& points);
	void DisplayPointCloud(const std::vector <feature_ptr>& features);

	void draw_pointcloud(const std::vector <feature_ptr>& features);

private:
	window win;
	glfw_state app_state;			// Object to manage view state


};

