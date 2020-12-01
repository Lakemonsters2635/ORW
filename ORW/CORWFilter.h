#pragma once
#include "librealsense2/rs.hpp"
#include <string>
#include <assert.h>

class CORWFilter
{
public:
	typedef enum cof_option {
		COF_OPTION_HSVMASK,
		COF_OPTION_RGBMASK,
	} cof_option;

	CORWFilter() {}
	void SetName(const std::string& name) { m_strName = name;  }

	virtual void Process(rs2::frame& color_frame, rs2::frame& depth_frame);
	virtual void SetOption(cof_option opt, void* value);

protected:
	std::string m_strName;
};

