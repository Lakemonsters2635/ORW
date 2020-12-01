#include "all_includes.h"
#include "CHSVFilter.h"

void CHSVFilter::Process(rs2::frame& color_frame, rs2::frame& depth_frame)
{
	auto mw = m_hsvMask->cols;
	auto mh = m_hsvMask->rows;

	if (mw == 0 || mh == 0)
		return;

	auto cw = color_frame.as<rs2::video_frame>().get_width();
	auto ch = color_frame.as<rs2::video_frame>().get_height();
	auto dw = depth_frame.as<rs2::depth_frame>().get_width();
	auto dh = depth_frame.as<rs2::depth_frame>().get_height();

	assert(mw == cw && mh == ch);			// size of mask must match color_frame
	assert(((dw * ch) / (dh * cw)) == 1);	// Depth and color must have the same aspect ratio

	// Filter the color_frame with the existing mask

	cv::Mat image(cv::Size(cw, ch), CV_8UC3, (void*)color_frame.as<rs2::video_frame>().get_data(), cv::Mat::AUTO_STEP);

	cv::Mat dst;
	cv::bitwise_and(image, image, dst, *m_hsvMask);
	memcpy(image.data, dst.data, dst.step[0] * dst.rows);

	// Now decimate the mask to the depth_frame size

	cv::Mat dMask = *m_hsvMask;

	if (dh != ch)
	{
		// specify fx and fy and let the function compute the destination image size.
		resize(*m_hsvMask, dMask, cv::Size(), 1.0 * dw / cw, 1.0 * dh / ch, cv::INTER_AREA);
	}

	mw = dMask.cols;
	mh = dMask.rows;

	// Filter the depth_frame with the existing mask

	cv::Mat depth(cv::Size(dw, dh), CV_16U, (void*)depth_frame.as<rs2::depth_frame>().get_data(), cv::Mat::AUTO_STEP);

	cv::Mat ddst;
	cv::bitwise_and(depth, depth, ddst, dMask);
	memcpy(depth.data, ddst.data, ddst.step[0] * ddst.rows);


	int i = 0;
}

void CHSVFilter::SetOption(cof_option opt, void* value)
{
	switch (opt)
	{
	case COF_OPTION_HSVMASK:
		m_hsvMask = (cv::Mat*)value;
		break;

	case COF_OPTION_RGBMASK:
		m_rgbMask = (cv::Mat*)value;
		break;

	}
}
