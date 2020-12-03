// This file is derived from the RealSense example.hpp.  I had to split it so that the classes
// could be using in multiple source files.

// Note: the original code should be kept in the same directory as this file and rs_example.cpp, for 2 reasons:
//
//  1. You can reference the original licensing info
//  2. When Intel updates this fle in subsequent SDK releases, you will have a reference to compare and changes they've made


// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>

#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <map>
#include <functional>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const size_t IMU_FRAME_WIDTH = 1280;
const size_t IMU_FRAME_HEIGHT = 720;
//////////////////////////////
// Basic Data Types         //
//////////////////////////////

struct float3 {
    float x, y, z;
    float3 operator*(float t);

    float3 operator-(float t);

    void operator*=(float t);

    void operator=(float3 other);

    void add(float t1, float t2, float t3);
};

struct float2 { float x, y; };

struct rect
{
    float x, y;
    float w, h;

    // Create new rect within original boundaries with give aspect ration
    rect adjust_ratio(float2 size) const;
};

//////////////////////////////
// Simple font loading code //
//////////////////////////////

#include <stb_easy_font.h>

inline void draw_text(int x, int y, const char* text);

void set_viewport(const rect& r);

class imu_renderer
{
public:
    void render(const rs2::motion_frame& frame, const rect& r);

    GLuint get_gl_handle();

private:
    GLuint _gl_handle = 0;

    void draw_motion(const rs2::motion_frame& f, const rect& r);

    //IMU drawing helper functions
    void multiply_vector_by_matrix(GLfloat vec[], GLfloat mat[], GLfloat* result);

    float2 xyz_to_xy(float x, float y, float z, GLfloat model[], GLfloat proj[], float vec_norm);

    void print_text_in_3d(float x, float y, float z, const char* text, bool center_text, GLfloat model[], GLfloat proj[], float vec_norm);

    static void  draw_axes(float axis_size = 1.f, float axisWidth = 4.f);

    // intensity is grey intensity
    static void draw_circle(float xx, float xy, float xz, float yx, float yy, float yz, float radius = 1.1, float3 center = { 0.0, 0.0, 0.0 }, float intensity = 0.5f);

};

class pose_renderer
{
public:
    void render(const rs2::pose_frame& frame, const rect& r);
    GLuint get_gl_handle();

private:
    mutable GLuint _gl_handle = 0;

    // Provide textual representation only
    void draw_pose(const rs2::pose_frame& f, const rect& r);
};

/// \brief Print flat 2D text over openGl window
struct text_renderer
{
    // Provide textual representation only
    void put_text(const std::string& msg, float norm_x_pos, float norm_y_pos, const rect& r);
};

////////////////////////
// Image display code //
////////////////////////
/// \brief The texture class
class texture
{
public:
    ~texture();

    void upload(const rs2::video_frame& frame);

    void show(const rect& r, float alpha = 1.f) const;

    GLuint get_gl_handle();

    void render(const rs2::frame& frame, const rect& rect, float alpha = 1.f);

private:
    GLuint          _gl_handle = 0;
    rs2_stream      _stream_type = RS2_STREAM_ANY;
    int             _stream_index{};
    imu_renderer    _imu_render;
    pose_renderer   _pose_render;
};

class window
{
public:
    std::function<void(bool)>           on_left_mouse = [](bool) {};
    std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
    std::function<void(double, double)> on_mouse_move = [](double, double) {};
    std::function<void(int)>            on_key_release = [](int) {};

    window(int width, int height, const char* title);

    ~window();

    void close();

    float width() const;
    float height() const;

    operator bool();

    void show(rs2::frame frame);

    void show(const rs2::frame& frame, const rect& rect);

    void show(const std::map<int, rs2::frame> frames);

    operator GLFWwindow* ();

private:
    GLFWwindow* win;
    std::map<int, texture> _textures;
    std::map<int, imu_renderer> _imus;
    std::map<int, pose_renderer> _poses;
    text_renderer   _main_win;
    int _width, _height;

    void render_video_frame(const rs2::video_frame& f, const rect& r);

    void render_motion_frame(const rs2::motion_frame& f, const rect& r);

    void render_pose_frame(const rs2::pose_frame& f, const rect& r);

    void render_frameset(const rs2::frameset& frames, const rect& r);

    bool can_render(const rs2::frame& f) const;

    rect calc_grid(rect r, size_t streams);

    std::vector<rect> calc_grid(rect r, std::vector<rs2::frame>& frames);
};

// Struct to get keys pressed on window
struct window_key_listener {
    int last_key = GLFW_KEY_UNKNOWN;

    window_key_listener(window& win);

    void on_key_release(int key);

    int get_key();
};

// Struct for managing rotation of pointcloud view
struct glfw_state {
    glfw_state(float yaw = 0, float pitch = 0, float panx = 0.0, float pany = 0.0, float offset_z = 0.0);
    void Reset() { yaw = 0; pitch = 0; panx = 0; pany = 0; offset_x = 0.f; offset_y = 0.f; offset_z = 0; }
    double yaw;
    double pitch;
    double panx;
    double pany;
    double last_x;
    double last_y;
    bool ml;
    float offset_x;
    float offset_y;
    float offset_z;
    texture tex;
};

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(float width, float height, glfw_state& app_state, rs2::points& points);

void quat2mat(rs2_quaternion& q, GLfloat H[16]);  // to column-major matrix

// Handles all the OpenGL calls needed to display the point cloud w.r.t. static reference frame
void draw_pointcloud_wrt_world(float width, float height, glfw_state& app_state, rs2::points& points, rs2_pose& pose, float H_t265_d400[16], std::vector<rs2_vector>& trajectory);

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, glfw_state& app_state);
