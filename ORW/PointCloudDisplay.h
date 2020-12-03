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


class CPointCloudDisplay
{
public:
	CPointCloudDisplay(GLFWwindow* win, const char* szTitle);

	void RenderUI();

	void DisplayPointCloud(rs2::points& points);
	void DisplayPointCloud(const std::vector <feature_ptr>& features);

	void draw_pointcloud(const std::vector <feature_ptr>& features);

private:
	std::string			m_Title;
	std::string			m_Child;

	ImVec2				m_Pos;			// Position at which to draw cloud
	ImVec2				m_Size;			// Size window

	GLFWwindow*			m_win;
	glfw_state			m_AppState;		// Object to manage view state

	ImVec2				m_LastPos;		// For calculating mouse movements
};

