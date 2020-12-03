#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

#include "CORWFilter.h"

#define RGB_HIST

class CColorChannelHistogramPlot
{
public:
	CColorChannelHistogramPlot(const char* name, float* data, size_t count = 256);
	auto GetMin() { return m_nMin; }
	auto GetMax() { return m_nMax; }

	void Reset() { m_nMin = 0; m_nMax = 0; }
	void RegisterSettings();
	void ImportSettings();


	void Draw();                    // Draw the histogram using ImPlot/ImGui

protected:
	static const int c_Ymaxes = 20;	// How many ymax values to average

	const char* m_pszName;          // Channel name
	float* m_pData;                 // Channel data - m_Count elements
	float m_LastYmaxes[c_Ymaxes];	// Save last n ymax values for averaging
	int m_iCurrentYmax;				// Index into LastYmaxes buffer
	size_t m_Count;                 // # of data elements
	union
	{
		int m_Vals[2];
		struct { int m_nMin, m_nMax; };
	};
	//int m_nMin;                     // Min selected value
	//int m_nMax;                     // Max selected value
	static double m_dPos[256];         // X-Axis values

	std::string m_strLabel;         // Chart label
	std::string m_strMinLabel;      // Xmin label
	std::string m_strMaxLabel;      // Xmax label
};


class CHistograms : public CORWFilter
{
public:
	CHistograms()
		: h_hist(180, 1, CV_32F)
		, s_hist(256, 1, CV_32F)
		, v_hist(256, 1, CV_32F)
		, cchpH("H", (float*)h_hist.ptr(), 180)
		, cchpS("S", (float*)s_hist.ptr())
		, cchpV("V", (float*)v_hist.ptr())
#ifdef RGB_HIST
		, r_hist(256, 1, CV_32F)
		, g_hist(256, 1, CV_32F)
		, b_hist(256, 1, CV_32F)
		, cchpR("R", (float*)r_hist.ptr())
		, cchpG("G", (float*)g_hist.ptr())
		, cchpB("B", (float*)b_hist.ptr())
#endif
	{}

	virtual void Process(rs2::frame& color_frame, rs2::frame& depth_frame);
	virtual void SetOption(cof_option opt, void* value);

	void RenderUI();
	void CalcHistogramMasks(const cv::Mat& rgbImage);

	cv::Mat& GetMaskHSV() { return m_maskHSV; }
#ifdef RGB_HIST
	cv::Mat& GetMaskRGB() { return m_maskRGB; }
#endif

	void RegisterSettings();
	void ImportSettings();
	void Reset();

protected:
	cv::Mat m_maskHSV;

	cv::Mat h_hist, s_hist, v_hist;

	CColorChannelHistogramPlot cchpH;
	CColorChannelHistogramPlot cchpS;
	CColorChannelHistogramPlot cchpV;

#ifdef RGB_HIST
	cv::Mat m_maskRGB;

	cv::Mat r_hist, g_hist, b_hist;

	CColorChannelHistogramPlot cchpR;
	CColorChannelHistogramPlot cchpG;
	CColorChannelHistogramPlot cchpB;

#endif

};