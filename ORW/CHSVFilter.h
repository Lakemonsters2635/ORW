#pragma once
#include <librealsense2/hpp/rs_processing.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

#include "CORWFilter.h"

class CHSVFilter : public CORWFilter
{
public:
	CHSVFilter()
	: m_hsvMask(nullptr)
	, m_rgbMask(nullptr)
	{}

	virtual void Process(rs2::frame& color_frame, rs2::frame& depth_frame);
	virtual void SetOption(cof_option opt, void* value);

protected:
	cv::Mat*		m_hsvMask;
	cv::Mat*		m_rgbMask;

};

