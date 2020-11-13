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
