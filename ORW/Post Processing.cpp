// Post processing

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "all_includes.h"
//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include "Utilities.h"

#include <map>
#include <string>
#include <thread>
#include <atomic>

#include "Post Processing.h"

const std::array<rs2_option, 6> possible_filter_options = {
    RS2_OPTION_FILTER_MAGNITUDE,
    RS2_OPTION_FILTER_SMOOTH_ALPHA,
    RS2_OPTION_MIN_DISTANCE,
    RS2_OPTION_MAX_DISTANCE,
    RS2_OPTION_FILTER_SMOOTH_DELTA,
    RS2_OPTION_HOLES_FILL
};


void update_data(rs2::frame_queue& data, rs2::frame& colorized_depth, rs2::points& points, rs2::pointcloud& pc, texture& depth_texture, rs2::colorizer& color_map)
{
    rs2::frame f;
    if (data.poll_for_frame(&f))                    // Try to take the depth and points from the queue
    {
        points = pc.calculate(f);                   // Generate pointcloud from the depth data
        colorized_depth = color_map.process(f);     // Colorize the depth frame with a color map
        pc.map_to(colorized_depth);                 // Map the colored depth to the point cloud
        depth_texture.upload(colorized_depth);      // Upload the colorized depth to a texture
    }
}

void update_data(rs2::frame_queue& data, rs2::frame& color)
{
   if (!data.poll_for_frame(&color))
       color = rs2::frame{ nullptr };
}

CPostProcessing::CPostProcessing()
    : disparity_filter_name("Disparity")
    , depth_to_disparity(true)
    , disparity_to_depth(false)
    , stopped(false)
    , processing_thread(nullptr)
{
    m_pAlign = new rs2::align(m_AlignMode);

    // The following order of emplacement will dictate the orders in which filters are applied
    filters.emplace_back("Decimate", dec_filter);
    filters.emplace_back("Threshold", thr_filter);
    filters.emplace_back(disparity_filter_name, depth_to_disparity);
    filters.emplace_back("Spatial", spat_filter);
    filters.emplace_back("Temporal", temp_filter);

    // Get the default values for the filters.
    // If the above set of filters changes, the following
    // code is adaptive and will not need changing.

    for (rs2_option opt : possible_filter_options)
    {
        for (auto& fo : filters)
        {
            auto flt = fo.filter;

            if (flt.supports(opt))
            {
                rs2::option_range range = flt.get_option_range(opt);
                std::string key = fo.filter_name + "_" + flt.get_option_name(opt);
                Default def = { fo, opt, filter_slider_ui::is_all_integers(range), flt.get_option(opt) };
                filterDefaults.emplace(key, def);
                int i = 0;
            }
        }
    }
}


void CPostProcessing::render_ui(/*std::vector<filter_options>& filters*/)
{
    // Flags for displaying ImGui window
    static const int flags = ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoScrollbar
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoMove;

    ImGui::Begin("Post Processing"/*, nullptr, flags*/);

    // Using ImGui library to provide slide controllers for adjusting the filter options
    const float offset_x = 0;
    const int offset_from_checkbox = 105;
    float offset_y = 45;
    float elements_margin = 35;

    ImGui::SetCursorPos({ offset_x, offset_y });

    bool bAlignChanged = ImGui::RadioButton("Align to Depth", (int*) &m_AlignMode, (int) RS2_STREAM_DEPTH); ImGui::SameLine();
    bAlignChanged |= ImGui::RadioButton("Align to Color", (int*) &m_AlignMode, (int) RS2_STREAM_COLOR);
    if (bAlignChanged)
    {
        g_settings.SetValue("PostProcessing", "Alignment", (int) m_AlignMode);
        UpdateAlign();
    }

    offset_y += elements_margin;

    for (auto& filter : filters)
    {
        // Draw a checkbox per filter to toggle if it should be applied
        ImGui::SetCursorPos({ offset_x, offset_y });
        ImGui::PushStyleColor(ImGuiCol_CheckMark, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
        bool tmp_value = filter.is_enabled;
        if (ImGui::Checkbox(filter.filter_name.c_str(), &tmp_value))
            g_settings.SetValue("PostProcessing", filter.filter_name + "_Enabled", tmp_value);
        filter.is_enabled = tmp_value;
        ImGui::PopStyleColor();

        if (filter.supported_options.size() == 0)
        {
            offset_y += elements_margin;
        }
        // Draw a slider for each of the filter's options
        for (auto& option_slider_pair : filter.supported_options)
        {
            filter_slider_ui& slider = option_slider_pair.second;
            if (slider.render({ offset_x + offset_from_checkbox, offset_y, 240 }, filter.is_enabled))
            {
                filter.filter.set_option(option_slider_pair.first, slider.value);
                if (slider.is_int)
                    g_settings.SetValue("PostProcessing", slider.name, (int) slider.value);
                else
                    g_settings.SetValue("PostProcessing", slider.name, slider.value);
            }
            offset_y += elements_margin;
        }
    }

    ImGui::End();
    //ImGui::Render();
}

bool filter_slider_ui::render(const float3& location, bool enabled)
{
    bool value_changed = false;
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12);
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, { 20 / 255.f, 150 / 255.f, 70 / 255.f, 1 });
    ImGui::GetStyle().GrabRounding = 12;
    if (!enabled)
    {
        ImGui::PushStyleColor(ImGuiCol_SliderGrab, { 0,0,0,0 });
        ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, { 0,0,0,0 });
        ImGui::PushStyleColor(ImGuiCol_Text, { 0.6f, 0.6f, 0.6f, 1 });
    }

    ImGui::PushItemWidth(location.z);
    ImGui::SetCursorPos({ location.x, location.y + 3 });
    ImGui::TextUnformatted(label.c_str());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", description.c_str());

    ImGui::SetCursorPos({ location.x + 170, location.y });

    if (is_int)
    {
        int value_as_int = static_cast<int>(value);
        value_changed = ImGui::SliderInt(("##" + name).c_str(), &value_as_int, static_cast<int>(range.min), static_cast<int>(range.max), "%.0f");
        value = static_cast<float>(value_as_int);
    }
    else
    {
        value_changed = ImGui::SliderFloat(("##" + name).c_str(), &value, range.min, range.max, "%.3f", 1.0f);
    }

    ImGui::PopItemWidth();

    if (!enabled)
    {
        ImGui::PopStyleColor(3);
    }
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(2);
    return value_changed;
}

/**
  Helper function for deciding on int ot float slider
*/
bool filter_slider_ui::is_all_integers(const rs2::option_range& range)
{
    const auto is_integer = [](float f)
    {
        return (fabs(fmod(f, 1)) < std::numeric_limits<float>::min());
    };

    return is_integer(range.min) && is_integer(range.max) &&
        is_integer(range.def) && is_integer(range.step);
}

/**
Constructor for filter_options, takes a name and a filter.
*/
filter_options::filter_options(const std::string name, rs2::filter& flt) :
    filter_name(name),
    filter(flt),
    is_enabled(true)
{
    //Go over each filter option and create a slider for it
    for (rs2_option opt : possible_filter_options)
    {
        if (flt.supports(opt))
        {
            rs2::option_range range = flt.get_option_range(opt);
            supported_options[opt].range = range;
            supported_options[opt].value = range.def;
            supported_options[opt].is_int = filter_slider_ui::is_all_integers(range);
            supported_options[opt].description = flt.get_option_description(opt);
            std::string opt_name = flt.get_option_name(opt);
            supported_options[opt].name = name + "_" + opt_name;
            std::string prefix = "Filter ";
            supported_options[opt].label = opt_name;
        }
    }
}

filter_options::filter_options(filter_options&& other) :
    filter_name(std::move(other.filter_name)),
    filter(other.filter),
    supported_options(std::move(other.supported_options)),
    is_enabled(other.is_enabled.load())
{
}

void CPostProcessing::StopProcessing()
{
    stopped = true;
    processing_thread->join();              // wait for processing thread to stop
    stopped = false;
}

void CPostProcessing::StartProcessing(rs2::pipeline& the_pipe)
{
    ppipe = &the_pipe;

    processing_thread = new std::thread([&]() {
        while (!stopped) //While application is running
        {
            PauseLock.lock();                               // If already set, pipeline is locked.

            rs2::frameset data = ppipe->wait_for_frames(); // Wait for next set of frames from the camera

            AlignLock.lock();
            data = m_pAlign->process(data);
            AlignLock.unlock();

            rs2::frame depth_frame = data.get_depth_frame(); //Take the depth frame from the frameset
            rs2::frame color_frame = data.get_color_frame();

            if (!depth_frame) // Should not happen but if the pipeline is configured differently
            {
                PauseLock.unlock();
                return;       //  it might not provide depth and we don't want to crash
            }

            rs2::frame filtered = depth_frame; // Does not copy the frame, only adds a reference

            /* Apply filters.
            The implemented flow of the filters pipeline is in the following order:
            1. apply decimation filter
            2. apply threshold filter
            3. transform the scene into disparity domain
            4. apply spatial filter
            5. apply temporal filter
            6. revert the results back (if step Disparity filter was applied
            to depth domain (each post processing block is optional and can be applied independantly).
            */
            bool revert_disparity = false;
            for (auto&& filter : filters)
            {
                if (filter.is_enabled)
                {
                    filtered = filter.filter.process(filtered);
                    if (filter.filter_name == disparity_filter_name)
                    {
                        revert_disparity = true;
                    }
                }
            }
            if (revert_disparity)
            {
                filtered = disparity_to_depth.process(filtered);
            }

            for (auto& myfilter : myFilters)
            {
                myfilter->Process(color_frame, filtered);
            }

            // Push filtered & original data to their respective queues
            // Note, pushing to two different queues might cause the application to display
            //  original and filtered pointclouds from different depth frames
            //  To make sure they are synchronized you need to push them together or add some
            //  synchronization mechanisms
            filtered_data.enqueue(filtered);
            original_data.enqueue(depth_frame);
            color_data.enqueue(color_frame);

            PauseLock.unlock();
        }

    });

}

void CPostProcessing::UpdateData()
{
    update_data(original_data, colored_depth, original_points, original_pc, depth_state.tex, color_map);
    update_data(filtered_data, colored_filtered, filtered_points, filtered_pc, filtered_state.tex, color_map);
    update_data(color_data, color);
}

void CPostProcessing::UploadTextures()
{
}

void CPostProcessing::RegisterSettings()
{
    g_settings.SetValue("PostProcessing", "Alignment", (int)m_AlignMode);

    for (auto& filter : filters)
    {
        auto& name = filter.filter_name;
        g_settings.SetValue("PostProcessing", filter.filter_name + "_Enabled", filter.is_enabled);

        for (auto& option_slider_pair : filter.supported_options)
        {
            filter_slider_ui& slider = option_slider_pair.second;
            if (slider.is_int)
                g_settings.SetValue("PostProcessing", slider.name, (int) slider.value);
            else
                g_settings.SetValue("PostProcessing", slider.name, slider.value);
        }
    }
}

void CPostProcessing::ImportSettings()
{
    m_AlignMode = (rs2_stream)g_settings.GetValue("PostProcessing", "Alignment", (int)RS2_STREAM_DEPTH);
    UpdateAlign();

    for (auto& filter : filters)
    {
        auto& name = filter.filter_name;
        filter.is_enabled = g_settings.GetValue("PostProcessing", filter.filter_name + "_Enabled", true);
        
        for (auto& option_slider_pair : filter.supported_options)
        {
            filter_slider_ui& slider = option_slider_pair.second;
            slider.is_int;
            float value = slider.value;
            auto& option_name = slider.name;
            slider.value = g_settings.GetValue("PostProcessing", slider.name, 0.0f);
        }
    }
}

void CPostProcessing::Reset()
{
    m_AlignMode = RS2_STREAM_DEPTH;
    UpdateAlign();

    // Loop over filterDefaults structure and use it to reset the filters

    for (auto& it : filterDefaults)
    {
        std::string key = it.first;
        Default& def = it.second;

        def.fo.filter.set_option(def.opt, def.value);                   // Set option in filter
        def.fo.supported_options[def.opt].value = def.value;            // And also in the UI
    }

    for (auto& filter : filters)
    {
        filter.is_enabled = true;                                       // Enable all filters
    }
}

void CPostProcessing::UpdateAlign()
{
    AlignLock.lock();
    if (m_pAlign != nullptr)
        delete m_pAlign;
    m_pAlign = new rs2::align(m_AlignMode);
    AlignLock.unlock();
}
