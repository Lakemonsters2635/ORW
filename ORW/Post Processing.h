#pragma once

// This is copied from example.hpp because we cannot include it in 2 places.

struct float3;
class texture;

extern const std::array<rs2_option, 6> possible_filter_options;

/**
Helper class for controlling the filter's GUI element
*/
struct filter_slider_ui
{
    std::string name;
    std::string label;
    std::string description;
    bool is_int;
    float value;
    rs2::option_range range;

    bool render(const float3& location, bool enabled);
    static bool is_all_integers(const rs2::option_range& range);
};

/**
Class to encapsulate a filter alongside its options
*/
class filter_options
{
public:
    filter_options(const std::string name, rs2::filter& filter);
    filter_options(filter_options&& other);
    std::string filter_name;                                   //Friendly name of the filter
    rs2::filter& filter;                                       //The filter in use
    std::map<rs2_option, filter_slider_ui> supported_options;  //maps from an option supported by the filter, to the corresponding slider
    std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not
};

// Helper functions for rendering the UI
// Helper function for getting data from the queues and updating the view
void update_data(rs2::frame_queue& data, rs2::frame& depth, rs2::points& points, rs2::pointcloud& pc, texture& depth_texture, rs2::colorizer& color_map);
void update_data(rs2::frame_queue& data, rs2::frame& color);

class CPostProcessing
{
private:

    //struct Defaults
    //{
    //    int     dec_Magnitude;
    //    float   thr_MinDIstance;
    //    float   thr_MaxDistance;
    //    float   spat_SmoothAlpha;
    //    int     spat_Magnitude;
    //    int     spat_SmoothDelta;
    //    int     spat_HolesFill;
    //    int     temp_HolesFill;
    //    float   temp_SmoothAlpha;
    //    int     temp_SmoothDelta;

    //};

public:
    CPostProcessing();

    void render_ui(/*std::vector<filter_options>& filters*/);

    void RenderUI()
    {
        // Render the GUI
        render_ui(/*filters*/);
    }

    void StartProcessing(rs2::pipeline& the_pipe);

    void UpdateData();
    void RegisterSettings();
    void ImportSettings();
    void Reset();

    void Stop()
    {
        stopped = true;
    }

protected:
    struct Default
    {
        filter_options& fo;
        rs2_option opt;
        bool is_int;
        float value;
    };

    std::map<std::string, Default> filterDefaults;

    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

                                        // Declare disparity transform from depth to disparity and vice versa
    const std::string disparity_filter_name;
    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;

    // Initialize a vector that holds filters and their options
    std::vector<filter_options> filters;


    // Declaring two concurrent queues that will be used to enqueue and dequeue frames from different threads
    rs2::frame_queue original_data;
    rs2::frame_queue filtered_data;
    rs2::frame_queue color_data;        // The color stream

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Atomic boolean to allow thread safe way to stop the thread
    std::atomic_bool stopped;

    // The processing thread
    std::thread *processing_thread;

    // The pipeline
    rs2::pipeline* ppipe;

    // Declare pointcloud objects, for calculating pointclouds and texture mappings
    rs2::pointcloud original_pc;
    rs2::pointcloud filtered_pc;

public:
    // These hold the most recent frames
    rs2::frame color;
    rs2::frame colored_depth;
    rs2::frame colored_filtered;

    glfw_state depth_state{};
    glfw_state filtered_state{};

    // Declare objects that will hold the calculated pointclouds and colored frames
   // We save the last set of data to minimize flickering of the view
    rs2::points original_points;
    rs2::points filtered_points;

};
