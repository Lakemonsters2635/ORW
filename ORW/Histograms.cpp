
#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include "vendor/imgui/imgui_impl_glfw.h"
#include "vendor/imgui/imgui_impl_opengl3.h"
//#include "vendor/RealSense/example.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include "vendor/implot/implot.h"
#include "vendor/implot/implot_internal.h"

#include "Utilities.h"

#include <iostream>

namespace MyPlotFunctions
{
	template <typename T>
	void PlotHist(const char* label_id, const T* values, int count, int& Hmin, int& Hmax) {
		int width = 0;
		T maxValue = 0;
		if (ImPlot::BeginItem(label_id, ImPlotCol_Fill)) {
			if (ImPlot::FitThisFrame()) {
				for (int i = 0; i < count; ++i) {
					ImPlotPoint p = ImPlotPoint(i, values[i]);
					ImPlot::FitPoint(p);
				}
			}
			const ImPlotNextItemData& s = ImPlot::GetItemData();
			ImDrawList& DrawList = *ImPlot::GetPlotDrawList();
			ImU32 col_marker = ImGui::GetColorU32(s.Colors[ImPlotCol_MarkerFill]);
			ImU32 col_fill = ImGui::GetColorU32(s.Colors[ImPlotCol_Fill]);
			for (int i = 0; i < count; ++i) {
				ImPlotPoint p = ImPlotPoint(i, values[i]);
				if (values[i] > maxValue)
					maxValue = values[i];
				if (p.y == 0)
					continue;
				ImVec2 a = ImPlot::PlotToPixels(p.x, p.y);
				ImVec2 b = ImPlot::PlotToPixels(p.x, 0);
				DrawList.AddLine(a, b, col_fill);
			}
			DrawList.AddLine(ImPlot::PlotToPixels(Hmin, 0), ImPlot::PlotToPixels(Hmin, maxValue), col_marker);
			DrawList.AddLine(ImPlot::PlotToPixels(Hmax, 0), ImPlot::PlotToPixels(Hmax, maxValue), col_marker);
			ImPlot::EndItem();
		}
	}

}

// class CColorChannelHistogramPlot
//
//  Plots one channel of a color histogram.  Data is always an array of 256 longs.

class CColorChannelHistogramPlot
{
public:
	CColorChannelHistogramPlot(const char* name, float* data, size_t count = 256)
		: m_pszName(name)
		, m_pData(data)
		, m_Count(count)
		, m_nMin(0)
		, m_nMax((int)count)
		, m_strMinLabel("##")
		, m_strMaxLabel("##")
		, m_iCurrentYmax(0)
	{
		m_strLabel = name;
		m_strLabel.append(" Histogram");

		m_strMinLabel.append(name);
		m_strMinLabel.append("min");

		m_strMaxLabel.append(name);
		m_strMaxLabel.append("max");

		if (m_dPos[0] != 0) {
			for (int i = 0; i < 256; i++) {
				m_dPos[i] = i;
			}
		}

		for (auto& val : m_LastYmaxes)
			val = 0;
	}

	auto GetMin() { return m_nMin; }
	auto GetMax() { return m_nMax; }

	void Draw()                     // Draw the histogram using ImPlot/ImGui
	{
		float ymax = 0;
		for (int i = 0; i < m_Count; i++)
			if (m_pData[i] > ymax)
				ymax = m_pData[i];

		m_LastYmaxes[m_iCurrentYmax++] = ymax;
		m_iCurrentYmax %= c_Ymaxes;

		ymax = 0;
		for (auto& y : m_LastYmaxes)
			ymax += y;

		ImPlot::SetNextPlotLimits(0.0, (double)m_Count, 0, ymax / c_Ymaxes, ImGuiCond_Always);
		ImPlot::SetNextPlotTicksX(m_dPos, 1);
		ImPlot::SetNextMarkerStyle(-1, -1, ImColor(255, 255, 0));

		if (ImPlot::BeginPlot(m_strLabel.c_str(), "", "", ImVec2(-1, 0), 0, 0, 0))
		{
			MyPlotFunctions::PlotHist("", m_pData, (int)m_Count, m_nMin, m_nMax);
			ImPlot::EndPlot();
		}
		ImGui::SliderInt(m_strMinLabel.c_str(), &m_nMin, 0, (int)m_Count - 1);
		ImGui::SliderInt(m_strMaxLabel.c_str(), &m_nMax, 0, (int)m_Count - 1);

	}

protected:
	static const int c_Ymaxes = 20;	// How many ymax values to average

	const char* m_pszName;          // Channel name
	float* m_pData;                 // Channel data - m_Count elements
	float m_LastYmaxes[c_Ymaxes];	// Save last n ymax values for averaging
	int m_iCurrentYmax;				// Index into LastYmaxes buffer
	size_t m_Count;                 // # of data elements
	int m_nMin;                     // Min selected value
	int m_nMax;                     // Max selected value
	static double m_dPos[256];         // X-Axis values

	std::string m_strLabel;         // Chart label
	std::string m_strMinLabel;      // Xmin label
	std::string m_strMaxLabel;      // Xmax label
};

double CColorChannelHistogramPlot::m_dPos[256] = { 42 };               // Any non-zero value in element 0 means we need to init 

void DoHistograms(const cv::Mat& image)
{
	cv::Mat hsvImage;
	cv::cvtColor(image, hsvImage, cv::COLOR_RGB2HSV);

	std::vector<cv::Mat> hsvPlanes;
	cv::split(hsvImage, hsvPlanes);

	int histSizeSV = 256;
	int histSizeH = 180;

	float rangeSV[] = { 0, 256 };
	float rangeH[] = { 0, 180 };

	const float* rangesSV = { rangeSV };
	const float* rangesH = { rangeH };

	bool uniform = true; bool accumulate = false;

	static cv::Mat h_hist, s_hist, v_hist;

	const int zero = 0;

	cv::calcHist(&hsvPlanes[0], 1, &zero, cv::Mat(), h_hist, 1, &histSizeH, &rangesH, uniform, accumulate);
	cv::calcHist(&hsvPlanes[1], 1, &zero, cv::Mat(), s_hist, 1, &histSizeSV, &rangesSV, uniform, accumulate);
	cv::calcHist(&hsvPlanes[2], 1, &zero, cv::Mat(), v_hist, 1, &histSizeSV, &rangesSV, uniform, accumulate);

	s_hist.row(255) = 0;			// Not sure why I need this.  There's a spike in the last value....

	static CColorChannelHistogramPlot cchpH("H", (float*)h_hist.ptr(), 180);
	static CColorChannelHistogramPlot cchpS("S", (float*)s_hist.ptr());
	static CColorChannelHistogramPlot cchpV("V", (float*)v_hist.ptr());

	ImGui::Begin("HSV Histograms");
	{
		cchpH.Draw();
		cchpS.Draw();
		cchpV.Draw();
		ImGui::End();
	}

#ifdef RGB_HIST
	std::vector<cv::Mat> rgbPlanes;
	cv::split(image, rgbPlanes);

	int histSizeRGB = 256;
	float rangeRGB[] = { 0, 256 };
	const float* rangesRGB = { rangeRGB };
	static cv::Mat r_hist, g_hist, b_hist;

	cv::calcHist(&rgbPlanes[0], 1, &zero, cv::Mat(), r_hist, 1, &histSizeRGB, &rangesRGB, uniform, accumulate);
	cv::calcHist(&rgbPlanes[1], 1, &zero, cv::Mat(), g_hist, 1, &histSizeRGB, &rangesRGB, uniform, accumulate);
	cv::calcHist(&rgbPlanes[2], 1, &zero, cv::Mat(), b_hist, 1, &histSizeRGB, &rangesRGB, uniform, accumulate);

	static CColorChannelHistogramPlot cchpR("R", (float*)r_hist.ptr());
	static CColorChannelHistogramPlot cchpG("G", (float*)g_hist.ptr());
	static CColorChannelHistogramPlot cchpB("B", (float*)b_hist.ptr());

	ImGui::Begin("RGB Histograms");
	{
		cchpR.Draw();
		cchpG.Draw();
		cchpB.Draw();
		ImGui::End();
	}
#endif
	// Filter the color image by the HSV (and possibly RGB) values

	{
		cv::Mat mask;

		if (cchpH.GetMax() < cchpH.GetMin())
		{
			// Special handling for Hmax < Hmin because H coordinate wraps around
			//
			// We need two masks.  One for the H channel and another for the S and V channels.
			// We then bitwise AND the the two masks for the final mask
			//
			// To compute the Hmask, we swap the max and min in the call to inRange and then invert the result

			cv::Mat maskH, maskSV;

			cv::inRange(hsvImage, cv::Scalar(0, cchpS.GetMin(), cchpV.GetMin()), cv::Scalar(180, cchpS.GetMax(), cchpV.GetMax()), maskSV);

			cv::inRange(hsvImage, cv::Scalar(cchpH.GetMax(), 0, 0), cv::Scalar(cchpH.GetMin(), 256, 256), maskH);
			cv::bitwise_not(maskH, maskH);
			cv::bitwise_and(maskH, maskSV, mask);

		}
		else
		{
			// Normal Handling for Hmax >= Hmin

			cv::inRange(hsvImage, cv::Scalar(cchpH.GetMin(), cchpS.GetMin(), cchpV.GetMin()), cv::Scalar(cchpH.GetMax(), cchpS.GetMax(), cchpV.GetMax()), mask);
		}

		cv::Mat dst;
		cv::bitwise_and(image, image, dst, mask);
		memcpy(image.data, dst.data, dst.step[0] * dst.rows);
	}
}