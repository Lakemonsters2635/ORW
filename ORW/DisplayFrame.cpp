// CDisplayFrame

#include "all_includes.h"

#include "DisplayFrame.h"

void CDisplayFrame::FrameToTexture(rs2::frame& frame)
{
	auto vf = frame.as<rs2::video_frame>();
	auto width = vf.get_width();
	auto height = vf.get_height();

	m_FrameTexture.render(vf, { 0, 0, (float)width, (float)height });

	// Intel's example.hpp code doesn't clean up after itself!

	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
}


// **********  TODO  **************************
// We call texture::render on the colorized depth frame here.  We also 
// call texture::upload (which calls render) in updata data.  Really
// should only do this once.

void CDisplayFrame::RenderUI()
{
	ImGui::Begin(m_Title.c_str());
	{
		// Using a Child allow to fill all the space of the window.
		// It also alows customization
		ImGui::BeginChild(m_Child.c_str());

		// Get the size of the child (i.e. the whole draw size of the windows).
		ImGuiWindow* window = ImGui::GetCurrentWindowRead();
		ImVec2 wsize = ImGui::GetWindowSize();
		ImGui::Image((ImTextureID)m_FrameTexture.get_gl_handle(), wsize);
		ImGui::EndChild();
		ImGuiIO& io = ImGui::GetIO(); (void)io;
		m_MousePos.x = (io.MousePos.x - window->Pos.x) / wsize.x;
		m_MousePos.y = (io.MousePos.y - window->Pos.y) / wsize.y;

		if (ImGui::IsMousePosValid() && m_MousePos.x >= 0 && m_MousePos.x < 1.0 && m_MousePos.y >= 0 && m_MousePos.y < 1.0)
		{
			//ImGui::Text("Mouse pos: (%g, %g)", m_MousePos.x, m_MousePos.y);
			if (ImGui::IsMouseDown(0))
			{
				m_LastClick = { -1.0f, -1.0f };
			}

			if (ImGui::IsMouseReleased(0))
			{
				m_LastClick = m_MousePos;
			}
		}
		//else
		//	ImGui::Text("Mouse pos: <INVALID>");
	}
	ImGui::End();
}


bool CDisplayFrame::IsClicked(float& x, float& y)
{
	if (m_LastClick.x < 0.0f || m_LastClick.y < 0.0f)
		return false;

	x = m_LastClick.x; y = m_LastClick.y;

	m_LastClick = { -1.0f, -1.0f };

	return true;
}



#if 0
void DisplayPointCloud(glfw_state& state, rs2::points& points, const char* Title)
{
	// Use the RealSense example code to render the pointcloud.  But we'll make it
	// render it to a frambuffer so we can then show it inside an ImGui window.

	static int nTotalFrames[2] = { 0, 0 };
	static int nZeroFrames[2] = { 0, 0 };

	std::string child(Title);
	int which = 1;
	
	if (child == "Depth Point Cloud")
		which = 0;

	nTotalFrames[which]++;
	if (!points)
		nZeroFrames[which]++;

	ImGui::Begin("Rate");
	ImGui::SetCursorPos({ 10, 35 });
	ImGui::Text("Depth: %d %d", nZeroFrames[0], nTotalFrames[0]);
	ImGui::SetCursorPos({ 10, 70 });
	ImGui::Text("Filtered: %d %d", nZeroFrames[1], nTotalFrames[1]);
	ImGui::End();

//#define STATIC_FRAMEBUFFER

#ifdef STATIC_FRAMEBUFFER
	static unsigned int gl_framebuffer = 0;
	static unsigned int gl_texture;

	if (!gl_framebuffer)
	{
		glGenFramebuffers(1, &gl_framebuffer);
		glBindFramebuffer(GL_FRAMEBUFFER, gl_framebuffer);

		glGenTextures(1, &gl_texture);
		glBindTexture(GL_TEXTURE_2D, gl_texture);

		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1280, 720, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gl_texture, 0);
	}
	//else
	//	glBindFramebuffer(GL_FRAMEBUFFER, gl_framebuffer);
#else
	unsigned int gl_framebuffer = 0;
	unsigned int gl_texture;

	glGenFramebuffers(1, &gl_framebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, gl_framebuffer);

	glGenTextures(1, &gl_texture);
	glBindTexture(GL_TEXTURE_2D, gl_texture);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1280, 720, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gl_texture, 0);
#endif

	assert(GL_FRAMEBUFFER_COMPLETE == glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER));

	glClear(GL_COLOR_BUFFER_BIT);


	// Draw the pointcloud
	//dpc(1280, 1024, state, points);

	//if (points)
	//	dpc(1280, 1024, state, points);
	//else
		DisplayTriangle3(0.5, !which ? -0.25 : 0.5, 1.0);

	// Unbind the framebuffer.  Otherwise, nothing else will show on the screen!
	glBindFramebuffer(GL_FRAMEBUFFER, 0);


	ImGui::Begin(Title);
	{
		child.append(" Stream");

		// Using a Child allow to fill all the space of the window.
		// It also alows customization
		ImGui::BeginChild(child.c_str());
		// Get the size of the child (i.e. the whole draw size of the windows).
			// Get the size of the child (i.e. the whole draw size of the windows).
		ImVec2 wsize = ImGui::GetWindowSize();
		ImGui::Image((ImTextureID)gl_texture, wsize);
		ImGui::EndChild();
	}
	ImGui::End();

#ifndef STATIC_FRAMEBUFFER
	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
	glDeleteTextures(1, &gl_texture);

	glDeleteFramebuffers(1, &gl_framebuffer);
#endif
}
#endif


void DisplayTriangle2()
{
	glClear(GL_COLOR_BUFFER_BIT);

	glBegin(GL_TRIANGLES);
	glVertex2f(-0.5f, -0.5f);
	glVertex2f(0.0f, 0.5f);
	glVertex2f(0.5f, -0.5f);
	glEnd();

}

void DisplayTriangle3(float xSize, float ySize, float z)
{
	glClear(GL_COLOR_BUFFER_BIT);

	glBegin(GL_TRIANGLES);
	glVertex3f(-xSize, -ySize, z);
	glVertex3f(0.0f, ySize, z);
	glVertex3f(xSize, -ySize, z);
	glEnd();

}