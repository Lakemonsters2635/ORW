#pragma once

// This is copied from example.hpp because we cannot include it in 2 places.

struct float3/* {
    float x, y, z;
    float3 operator*(float t)
    {
        return { x * t, y * t, z * t };
    }

    float3 operator-(float t)
    {
        return { x - t, y - t, z - t };
    }

    void operator*=(float t)
    {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    void operator=(float3 other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void add(float t1, float t2, float t3)
    {
        x += t1;
        y += t2;
        z += t3;
    }
}*/;

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
void render_ui(float w, float h, std::vector<filter_options>& filters);
// Helper function for getting data from the queues and updating the view
//void update_data(rs2::frame_queue& data, rs2::frame& depth, rs2::points& points, rs2::pointcloud& pc, glfw_state& view, rs2::colorizer& color_map);

class CPostProcessing
{
public:
    CPostProcessing()
        : disparity_filter_name("Disparity")
        , depth_to_disparity(true)
        , disparity_to_depth(false)
        , stopped(false)
    {
        // The following order of emplacement will dictate the orders in which filters are applied
        filters.emplace_back("Decimate", dec_filter);
        filters.emplace_back("Threshold", thr_filter);
        filters.emplace_back(disparity_filter_name, depth_to_disparity);
        filters.emplace_back("Spatial", spat_filter);
        filters.emplace_back("Temporal", temp_filter);
    }

    void RenderUI(float w, float h)
    {
        // Render the GUI
        render_ui(w, h, filters);
    }

protected:
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

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Atomic boolean to allow thread safe way to stop the thread
    std::atomic_bool stopped;

};
