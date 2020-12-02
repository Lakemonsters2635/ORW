#include "all_includes.h"

#include "PointCloudDisplay.h"

byte3 LayerColors[N_COLORS] = {
	//{ 255, 255, 255 },
	{ 240, 41, 12 },
	{ 240, 139, 12 },
	{ 240, 233, 12 },
	{ 19, 240, 12 },
	{ 12, 240, 218 },
	{ 12, 157, 240 },
	{ 12, 70, 240 },
	{ 99, 12, 240 },
	{ 193, 12, 240 },
	{ 240, 12, 208 }
};

void CPointCloudDisplay::draw_pointcloud(const std::vector <feature_ptr>& features)
{
	if (m_Pos.x < 0 || m_Pos.y < 0)
		return;

	int h;
	int w;
	glfwGetWindowSize(m_win, &w, &h);

	glViewport(m_Pos.x, h - m_Pos.y - m_Size.y, m_Size.x, m_Size.y);				// may by inverted.....

	glClearColor(0.0, 0.0, 0.0, 1);
	glScissor(m_Pos.x, h - m_Pos.y - m_Size.y, m_Size.x, m_Size.y);
	glEnable(GL_SCISSOR_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_SCISSOR_TEST);

	::draw_pointcloud(m_Size.x, m_Size.y, m_AppState, features);
}

// Width/Height used for aspect ratio.  width used for text size (not using...)

void draw_pointcloud(float width, float height, glfw_state app_state, const std::vector <feature_ptr>& features)
{
	// OpenGL commands that prep screen for the pointcloud
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	//	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	//glClearColor(0.0, 0.0, 0.0, 1);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, width / height, 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

	/*********************  BUGBUG  ********************  Pan and Z offset not yet implemented   */

	glTranslatef(-0.5f, -0.5f, +0.5f /*+ app_state.offset_z * 0.05f*/);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(+0.5f /*+ app_state.panx * 0.05f*/, +0.5f /*+ app_state.pany * 0.05f*/, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_TEXTURE_2D);

	int color = 0;

	for (auto&& f : features)
	{
		//if (app_state.coef_sel != -1 && color == app_state.coef_sel)
		//{
		//	if (f->m_pCoefficients)
		//	{
		//		std::cerr << "Model Coefficients: " << *f->m_pCoefficients << std::endl;
		//		double Nx = f->m_pCoefficients->values[0];
		//		double Nz = f->m_pCoefficients->values[2];
		//		float theta = acos(Nz / sqrt(Nx * Nx + Nz * Nz));
		//		std::cerr << "Error Angle: " << theta * 180 / 3.1415926 << " degrees" << std::endl;
		//	}

		//	app_state.coef_sel = -1;
		//}

		auto pc = f->cloud;
		//if (app_state.mask & (1LL << color))
		//{
			//		auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];
			auto c = LayerColors[(color) % (N_COLORS)];

			glBegin(GL_POINTS);
			glColor3f(c.r / 255.0, c.g / 255.0, c.b / 255.0);

			/* this segment actually prints the pointcloud */
			for (int i = 0; i < pc->points.size(); i++)
			{
				auto&& p = pc->points[i];
				if (p.z)
				{
					// upload the point and texture coordinates only for points we have depth data for
					glVertex3f(p.x, p.y, p.z);
				}
			}

			glEnd();
		//}
		color++;
	}

	// OpenGL cleanup
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}




CPointCloudDisplay::CPointCloudDisplay(GLFWwindow* win, const char* szTitle)
	//: win(640, 480, szTitle)
	: m_AppState(0, 0)
	, m_win(win)
{
	// register callbacks to allow manipulation of the pointcloud
	//register_glfw_callbacks(win, app_state);
	m_Title = szTitle;
	m_Child = szTitle;
	m_Child.append(" cloud");
}

void CPointCloudDisplay::RenderUI()
{
	if (ImGui::Begin("RANSAC Output"))
	{
		// Using a Child allow to fill all the space of the window.
		// It also alows customization
		ImGui::BeginChild(m_Child.c_str());

		// Get the size of the child (i.e. the whole draw size of the windows).
		ImGuiWindow* window = ImGui::GetCurrentWindowRead();
		m_Pos = window->Pos;
		m_Size = window->Size;

		ImGui::EndChild();


		ImGui::End();
	}
	else
	{
		m_Pos = { -1.0f, -1.0f };
		m_Size = { -1.0f, -1.0f };
	}
}


void CPointCloudDisplay::DisplayPointCloud(rs2::points& points)
{
	//auto oldCTX = glfwGetCurrentContext();
	//glfwMakeContextCurrent((GLFWwindow*)win);
	//glClear(GL_COLOR_BUFFER_BIT);

	//::draw_pointcloud(1280, 1024, app_state, points);
	///* Swap front and back buffers */
	//glfwSwapBuffers((GLFWwindow*)win);

	///* Poll for and process events */
	//glfwPollEvents();

	//glfwMakeContextCurrent(oldCTX);
}



void CPointCloudDisplay::DisplayPointCloud(const std::vector <feature_ptr>& features)
{
	//auto oldCTX = glfwGetCurrentContext();
	//glfwMakeContextCurrent((GLFWwindow*)win);
	//glClear(GL_COLOR_BUFFER_BIT);

	//draw_pointcloud(features);
	///* Swap front and back buffers */
	//glfwSwapBuffers((GLFWwindow*)win);

	///* Poll for and process events */
	//glfwPollEvents();

	//glfwMakeContextCurrent(oldCTX);
}

