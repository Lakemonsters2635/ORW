// ORW.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
//#define _CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>


#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include "vendor/imgui/imgui_impl_glfw.h"
#include "vendor/imgui/imgui_impl_opengl3.h"
#include "vendor/RealSense/rs_example.h"
#include "vendor/implot/implot_demo.cpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

#include "PointCloudDisplay.h"
#include "CRANSAC.h"
#include "CSettings.h"
#include "Histograms.h"

//#include <pcl/ModelCoefficients.h>

#include "Utilities.h"

#include <iostream>
#include <chrono>

#include "Post Processing.h"

//#define	RGB_HIST			// Define this if you want RGB histograms to be calculated

static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

// class to calculate frame rate.
// Can average over multiple frames

class FrameRate
{
public:

	FrameRate(int nFrames = 10)
		: m_nFrames(nFrames)
	{
		frames = new std::chrono::duration<double>[m_nFrames];
	}

	~FrameRate()
	{
		if (frames)
			delete frames;
	}

	void NextFrame()
	{
		auto t = std::chrono::high_resolution_clock::now();
		auto delta = t - m_prev;
		m_prev = t;

		frames[m_nCurrFrame++] = delta;
		m_nCurrFrame %= m_nFrames;
	}

	auto GetFrameRate()
	{
		auto delta = frames[0];
		for (int i = 1; i < m_nFrames; i++)
			delta += frames[i];

		delta /= m_nFrames;

		return 1 / delta.count();
	}

private:
	int m_nFrames;
	std::chrono::duration<double>* frames = nullptr;
	int m_nCurrFrame = 0;
	std::chrono::high_resolution_clock::time_point m_prev = std::chrono::high_resolution_clock::now();
};

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
	window app(1792, 1008, "RealSense Align Example"); // Simple window handling
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

	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

	CPostProcessing pp;
	pp.StartProcessing(pipe);

	PointCloudDisplay pc("Point Cloud"), fpc("Filtered Point Cloud");
	FrameRate fr;
	CRANSAC ransac;
	CHistograms hist;

	int nFrames = 0;

	// Main loop
	while (!glfwWindowShouldClose(app)/* && nFrames++ < 100*/)
	{
		fr.NextFrame();
		auto rate = fr.GetFrameRate();

		glfwMakeContextCurrent((GLFWwindow * )app);

		pp.UpdateData();

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

		//ImGui::ShowDemoWindow();

		//ImPlot::ShowDemoWindow();

		if (ImGui::BeginMainMenuBar())
		{
			if (ImGui::BeginMenu("File"))
			{
				ImGui::MenuItem("(demo menu)", NULL, false, false);
				if (ImGui::MenuItem("New"))
				{
					g_settings.New();
					pp.Reset();
					hist.Reset();

					pp.RegisterSettings();
					hist.RegisterSettings();
				}
				if (ImGui::MenuItem("Open", "Ctrl+O"))
				{
					g_settings.Load("");

					pp.Reset();
					hist.Reset();

					pp.ImportSettings();
					hist.ImportSettings();
				}
				if (ImGui::MenuItem("Save", "Ctrl+S"))
				{
					g_settings.Save();
					//pp.SaveSettings();
				}
				if (ImGui::BeginMenu("Open Recent"))
				{
					ImGui::MenuItem("fish_hat.c");
					ImGui::MenuItem("fish_hat.inl");
					ImGui::MenuItem("fish_hat.h");
					//if (ImGui::BeginMenu("More.."))
					//{
					//	ImGui::MenuItem("Hello");
					//	ImGui::MenuItem("Sailor");
					//}
					ImGui::EndMenu();
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Edit"))
			{
				if (ImGui::MenuItem("Undo", "CTRL+Z")) {}
				if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {}  // Disabled item
				ImGui::Separator();
				if (ImGui::MenuItem("Cut", "CTRL+X")) {}
				if (ImGui::MenuItem("Copy", "CTRL+C")) {}
				if (ImGui::MenuItem("Paste", "CTRL+V")) {}
				ImGui::EndMenu();
			}
			ImGui::EndMainMenuBar();
		}

		ImGui::Begin("Stats");
		ImGui::Text("Frame Rate: %.1f fps", rate);
		ImGui::End();

		// If we have a new frame, process it and update the UI

		// Each call to DisplayFrame needs a unique texture.
		// The scope of the RealSense texture needs to extend to past the
		// call to ImGui::Render(), but then needs to end before
		// starting another frame.  That makes sure the contained OpenGL
		// texture is deleted (by ~texture()) to avoid memory leaks.
		// IMPORTANT: *I* added the destructor to texture.  If you update
		// rs_example.* (from Intel's example.hpp), you may need to put
		// back the destructor.

		texture color_texture;
		texture depth_texture;
		texture filtered_texture;

		if (pp.color)
		{
			// Create OpenCV matrix of size (w,h) from the colorized depth data
			auto width = pp.color.as<rs2::video_frame>().get_width();
			auto height = pp.color.as<rs2::video_frame>().get_height();
			uint8_t* pixels = (uint8_t*)pp.color.get_data();

			auto widthDepth = pp.colored_depth.as<rs2::video_frame>().get_width();
			auto heightDepth = pp.colored_depth.as<rs2::video_frame>().get_height();

			cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)pp.color.get_data(), cv::Mat::AUTO_STEP);
			hist.DoHistograms(image);

			DisplayFrame(pp.color, "Color", color_texture);
			DisplayFrame(pp.colored_depth, "Depth", depth_texture);
			DisplayFrame(pp.colored_filtered, "Filtered Depth", filtered_texture);
		}

		pp.RenderUI();
		ransac.RenderUI();

		// Rendering
		ImGui::Render();

		int display_w, display_h;
		glfwGetFramebufferSize(app, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(app);

		pc.DisplayPointCloud(pp.original_points);
		fpc.DisplayPointCloud(pp.filtered_points);

	}

	pp.Stop();

	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(app);
	glfwTerminate();

//	_CrtDumpMemoryLeaks();
	return 0;
}
