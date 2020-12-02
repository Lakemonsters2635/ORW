#pragma once

class CDisplayFrame
{
public:
	CDisplayFrame(const char* Title, bool bHasPicker = false)
	: m_bHasPicker(bHasPicker)
	, m_Title(Title)
	, m_LastClick({ -1.0f, -1.0f })
	{
		m_Child = m_Title;
		m_Child.append(" Stream");
	}

	void FrameToTexture(rs2::frame& frame);
	bool IsClicked(float& x, float& y);

	void RenderUI();

protected:
	// Makes sure the contained OpenGL
	// texture is deleted (by ~texture()) to avoid memory leaks.
	// IMPORTANT: *I* added the destructor to texture.  If you update
	// rs_example.* (from Intel's example.hpp), you may need to put
	// back the destructor.

	texture			m_FrameTexture;
	std::string		m_Title;
	std::string		m_Child;
	bool			m_bHasPicker;

	ImVec2			m_MousePos;
	ImVec2			m_LastClick;

};

// For debugging.

void DisplayTriangle2();
void DisplayTriangle3(float xSize, float ySize, float z);

