// ORW.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include "vendor/imgui/imgui_impl_glfw.h"
#include "vendor/imgui/imgui_impl_opengl3.h"
#include "vendor/RealSense/example.hpp"
#include "vendor/implot/implot_demo.cpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

#include "Utilities.h"

#include <iostream>

#include "Post Processing.h"

//#define	RGB_HIST			// Define this if you want RGB histograms to be calculated

CPostProcessing pp;



static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

/*
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
		, m_nMax((int) count)
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
*/

int main()
{
	// Init the RealSense library and camera

	// Create a pipeline to easily configure and start the camera
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR);
	pipe.start(cfg);

	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;
	texture depth_image, color_image;     // Helpers for rendering images

	// Setup window
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
		return 1;

	// Create and initialize GUI related objects
	window app(1280, 720, "RealSense Align Example"); // Simple window handling
 //   glfwMaximizeWindow(app);

	// Initialize OpenGL loader
	bool err = glewInit() != GLEW_OK;
	if (err)
	{
		fprintf(stderr, "Failed to initialize OpenGL loader!\n");
		return 1;
	}

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImPlot::CreateContext();

	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(app, true);
	ImGui_ImplOpenGL3_Init(/*glsl_version*/);

	bool show_demo_window = true;
	bool show_another_window = false;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);


	// Main loop
	while (!glfwWindowShouldClose(app))
	{
		rs2::frameset data = pipe.wait_for_frames();    // Wait for next set of frames from the camera
		auto depth = data.get_depth_frame();
		auto color = data.get_color_frame();
		auto colorized_depth = color_map.colorize(depth);

		// Poll and handle events (inputs, window resize, etc.)
		// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
		// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
		// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
		// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
		glfwPollEvents();

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		static const int flags = ImGuiWindowFlags_NoCollapse
			| ImGuiWindowFlags_NoScrollbar
			| ImGuiWindowFlags_NoSavedSettings
			| ImGuiWindowFlags_NoTitleBar
			//| ImGuiWindowFlags_NoResize
			| ImGuiWindowFlags_NoMove;

		//ImPlot::ShowDemoWindow();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		auto width = color.get_width();
		auto height = color.get_height();
		uint8_t* pixels = (uint8_t*)color.get_data();

		auto widthDepth = depth.get_width();
		auto heightDepth = depth.get_height();

		cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
		DoHistograms(image);

/*
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
			//cv::Mat src = image.clone();
			cv::inRange(hsvImage, cv::Scalar(cchpH.GetMin(), cchpS.GetMin(), cchpV.GetMin()), cv::Scalar(cchpH.GetMax(), cchpS.GetMax(), cchpV.GetMax()), mask);
			auto t = mask.type();
			//cv::bitwise_not(mask, mask);
			auto x = color.get_data();
			cv::Mat dst;
			cv::bitwise_and(image, image, dst, mask);
			memcpy(image.data, dst.data, dst.step[0] * dst.rows);
			int i = 0;
		}
*/
		pp.RenderUI(1280, 720);

		ImGui::Begin("Color");
		{
			color_image.render(color, { 0, 0, 200.0f*(float)width/(float)height, 200 });

			// Using a Child allow to fill all the space of the window.
			// It also alows customization
			ImGui::BeginChild("Color Stream");
			// Get the size of the child (i.e. the whole draw size of the windows).
			ImVec2 wsize = ImGui::GetWindowSize();
			ImGui::Image((ImTextureID)color_image.get_gl_handle(), wsize);
			ImGui::EndChild();
		}
		ImGui::End();

		ImGui::Begin("Depth");
		{
			depth_image.render(colorized_depth, { 0, 0, 200.0f * (float)widthDepth / (float)heightDepth, 200 });

			// Using a Child allow to fill all the space of the window.
			// It also alows customization
			ImGui::BeginChild("Depth Stream");
			// Get the size of the child (i.e. the whole draw size of the windows).
			ImVec2 wsize = ImGui::GetWindowSize();
			ImGui::Image((ImTextureID)depth_image.get_gl_handle(), wsize);
			ImGui::EndChild();
		}
		ImGui::End();

		// Rendering
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(app, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(app);
	}

	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(app);
	glfwTerminate();

	return 0;
}
