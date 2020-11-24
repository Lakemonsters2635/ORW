#pragma once

#include <librealsense2/rs.h>

class PointCloudDisplay
{
public:
	PointCloudDisplay(const char* szTitle);

	void DisplayPointCloud(rs2::points& points);


private:
	window win;
	glfw_state app_state;			// Object to manage view state
};

