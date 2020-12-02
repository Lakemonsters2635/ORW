// ORW.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "all_includes.h"

#include "vendor/implot/implot_demo.cpp"

#include "PointCloudDisplay.h"
#include "CRANSAC.h"
#include "CSettings.h"
#include "Histograms.h"
#include "CHSVFilter.h"
#include "CORWFilter.h"
#include "CFileDlg.h"
#include "DisplayFrame.h"

//#include <pcl/ModelCoefficients.h>

#include "PCLUtils.h"

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

int main(int argc, char** argv)
{
	std::string strBagFile, strOrwFile;

	while (--argc)
	{
		std::string test(argv[1]);
		std::for_each(test.begin(), test.end(), [](char& c) {
			c = ::toupper(c);
		});
		if (test.find(".BAG") != std::string::npos)
			strBagFile = argv[1];
		if (test.find(".ORW") != std::string::npos)
			strOrwFile = argv[1];
		argv++;
	}

	// Init the RealSense library and camera

	rs2::context ctx;            // Create librealsense context for managing devices
	auto devices = ctx.query_devices();
	std::string bagFilename;

	std::vector<rs2::device> depthCameras;

	for (auto&& dev : devices)
	{
		std::string strCameraName(dev.get_info(RS2_CAMERA_INFO_NAME));
		std::string strCameraSerialNumber(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

		depthCameras.push_back(dev);
	}

	int nCurrentCamera = 0;										// Always start w/ 1st camera found

	// Create a pipeline to easily configure and start the camera
	auto pipe = std::make_shared<rs2::pipeline>();
	//rs2::pipeline pipe(ctx);
	rs2::config cfg;

	if (!strBagFile.empty())
	{
		cfg.enable_device_from_file(strBagFile);
	}
	else if (depthCameras.size() == 0)								// Read from file
	{
		CFileDlg dlg("RealSense Recorded Data\0*.bag\0", "bag");
		strBagFile = dlg.GetOpenFileName();
		if (strBagFile.empty())
		{
			std::cerr << "No cameras and no input file\n";
		}
		cfg.enable_device_from_file(strBagFile);
	}
	else
	{
		cfg.enable_device(depthCameras[nCurrentCamera].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	}

	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR);
	pipe->start(cfg);

	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;
	texture depth_image, color_image;     // Helpers for rendering images

	// Setup window
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
		return 1;

	// Create and initialize GUI related objects
	window app(1792, 1008, "Lake Monsters Object Recognition Workbench"); // Simple window handling
    glfwMaximizeWindow(app);

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
	FrameRate fr;
	CRANSAC ransac;

	// Filter that calculates histograms

	CHistograms hist;
	pp.AddFilter("Hist", hist);

	// Filter that applies HSV mask

	CHSVFilter hsvFilter;
	pp.AddFilter("HSV", hsvFilter);
	hsvFilter.SetOption(CORWFilter::COF_OPTION_HSVMASK, &hist.GetMaskHSV());

	pp.StartProcessing(*pipe);

	//PointCloudDisplay pc("Point Cloud"), fpc("Filtered Point Cloud"), featurePC("Features Detected");
	CPointCloudDisplay pcdRANSAC(app, "RANSAC Cloud");			// Used to display RANSAC output

	if (!strOrwFile.empty())
	{
		g_settings.Load(strOrwFile);
		pp.Reset();
		hist.Reset();
		ransac.Reset();

		pp.ImportSettings();
		hist.ImportSettings();
		ransac.ImportSettings();
	}

	int nFrames = 0;

	CDisplayFrame colorFrame("Color", true);			// This one has a color picker
	CDisplayFrame depthFrame("Depth");
	CDisplayFrame filteredFrame("Filtered Depth");

	// Main loop
	while (!glfwWindowShouldClose(app)/* && nFrames++ < 100*/)
	{
		glfwMakeContextCurrent((GLFWwindow * )app);

//		if (!ransac.IsBusy())							// Needs to accomodate *all* analysis modules
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
				//ImGui::MenuItem("(demo menu)", NULL, false, false);
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
					ransac.Reset();

					pp.ImportSettings();
					hist.ImportSettings();
					ransac.ImportSettings();
				}
				if (ImGui::MenuItem("Save", "Ctrl+S"))
				{
					g_settings.Save();
					//pp.SaveSettings();
				}
				if (ImGui::MenuItem("Save As...", ""))
				{
					g_settings.SaveAs();
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
			if (ImGui::BeginMenu("Sources"))
			{
				for (auto i=0; i< depthCameras.size(); i++)
				{
					auto cam = depthCameras[i];
					if (ImGui::MenuItem(cam.get_info(RS2_CAMERA_INFO_NAME), "", nCurrentCamera == i))
					{
						pp.StopProcessing();

						nCurrentCamera = i;

						pipe->stop(); // Stop streaming with default configuration
						pipe = std::make_shared<rs2::pipeline>();
						rs2::config cfg;
						cfg.enable_device(depthCameras[nCurrentCamera].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
						cfg.enable_stream(RS2_STREAM_DEPTH);
						cfg.enable_stream(RS2_STREAM_COLOR);
						pipe->start(cfg);
						pp.StartProcessing(*pipe);

						//pipe->start(cfg); //File will be opened in read mode at this point
						//device = pipe->get_active_profile().get_device();

					}
				}
				if (ImGui::MenuItem("Read from File", nCurrentCamera == -1 ? bagFilename.c_str() : "", nCurrentCamera == -1))
				{
					char szDir[MAX_PATH + 1];
					::GetCurrentDirectoryA(MAX_PATH + 1, szDir);

					OPENFILENAMEA ofn;                      // common dialog box structure
					char szFile[MAX_PATH + 1] = { 0 };

					// Initialize OPENFILENAME
					ZeroMemory(&ofn, sizeof(ofn));
					ofn.lStructSize = sizeof(ofn);
					ofn.hwndOwner = NULL;
					ofn.lpstrFile = szFile;
					ofn.nMaxFile = MAX_PATH + 1;
					ofn.lpstrFilter = "RealSense Recorded Data\0*.bag\0";
					ofn.nFilterIndex = 1;
					ofn.lpstrFileTitle = NULL;
					ofn.nMaxFileTitle = 0;
					ofn.lpstrInitialDir = NULL;
					ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
					ofn.lpstrDefExt = "bag";
					ofn.lpstrInitialDir = szDir;

					if (GetOpenFileNameA(&ofn))
					{
						pp.StopProcessing();

						nCurrentCamera = -1;
						bagFilename = ofn.lpstrFile;

						pipe->stop(); // Stop streaming with default configuration
						pipe = std::make_shared<rs2::pipeline>();
						rs2::config cfg;
						cfg.enable_device_from_file(ofn.lpstrFile);
						pipe->start(cfg);
						pp.StartProcessing(*pipe);
					}
				}
				ImGui::EndMenu();
			}
			if (ImGui::MenuItem(pp.IsPaused() ? "Resume" : "Pause"))
			{
				if (pp.IsPaused())
					pp.Continue();
				else
					pp.Pause();
			}
			ImGui::EndMainMenuBar();
		}

		// If we have a new frame, process it and update the UI

		if (pp.color)
		{
			colorFrame.FrameToTexture(pp.color);
			depthFrame.FrameToTexture(pp.colored_depth);
			filteredFrame.FrameToTexture(pp.colored_filtered);
		}

		if (ransac.FindFeatures(pp.filtered_points))
		{
			fr.NextFrame();
		}

		float x, y;
		if (colorFrame.IsClicked(x, y))
			std::cerr << "Clicked at " << x << ", " << y << std::endl;

		auto rate = fr.GetFrameRate();
		ImGui::Begin("Stats");
		ImGui::Text("Frame Rate: %.1f fps", rate);
		ImGui::End();

		colorFrame.RenderUI();
		depthFrame.RenderUI();
		filteredFrame.RenderUI();

		pp.RenderUI();
		ransac.RenderUI();
		hist.RenderUI();

		pcdRANSAC.RenderUI();		// This just renders an empty box.  We'll scribble on it later...

		// Rendering
		ImGui::Render();

		int display_w, display_h;
		glfwGetFramebufferSize(app, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


////		glClear(GL_COLOR_BUFFER_BIT);
//
//		glBegin(GL_TRIANGLES);
//		glVertex2f(-0.25f, -0.25f);
//		glVertex2f(0.0f, 0.25f);
//		glVertex2f(0.25f, -0.25f);
//		glEnd();
//
//
		//int h;
		//int w;
		//glfwGetWindowSize(app, &w, &h);
		//glViewport(2 * w / 3, 0, w / 3, h / 3);

		//glClearColor(0.0, 0.0, 0.0, 1);
		//glScissor(2 * w / 3, 0, w / 3, h / 3);
		//glEnable(GL_SCISSOR_TEST);
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//glDisable(GL_SCISSOR_TEST);

		//glfw_state original_view_orientation{ 0, 0 };
		//draw_pointcloud(w / 3.0, h / 3.0, original_view_orientation, pp.filtered_points);

		ransac.Lock();
		pcdRANSAC.draw_pointcloud(ransac.GetLayers());
		//draw_pointcloud(w, h, original_view_orientation, ransac.GetLayers());
		ransac.Unlock();

		glfwSwapBuffers(app);


		//pc.DisplayPointCloud(pp.original_points);
		//fpc.DisplayPointCloud(pp.filtered_points);
		//featurePC.DisplayPointCloud(ransac.GetLayers());
	}

	pp.StopProcessing();
	ransac.Stop();

	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(app);
	glfwTerminate();

#ifdef _CRTDBG_MAP_ALLOC
	_CrtDumpMemoryLeaks();
#endif

	return 0;
}
