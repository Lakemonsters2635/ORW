#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include "CSettings.h"


class texture;
struct glfw_state;
class window;

#define INCHES_PER_METER	39.37
#define	DEGREES_PER_RADIAN	(180.0/3.14159265358)

void FrameToTexture(rs2::frame& frame, texture& frame_texture);
void DisplayTexture(const char* Title, texture& frame_texture);

void DisplayTriangle2();
void DisplayTriangle3(float xSize, float ySize, float z);


extern CSettings g_settings;

#define XPRINT(x)	std::cerr << x
//#define XPRINT(x)	

// Because the RealSense library cannot (yet) use C++ 20 compiler, and std::semaphore is a C++ 20 feature

#include <mutex>
#include <condition_variable>

class Semaphore {
public:
    Semaphore(int count_ = 0)
        : count(count_)
    {
    }

    inline void notify() {
        std::unique_lock<std::mutex> lock(mtx);
        count++;
        //notify the waiting thread
        cv.notify_one();
    }
    inline void wait() {
        std::unique_lock<std::mutex> lock(mtx);
        while (count == 0) {
            //wait on the mutex until notify is called
            cv.wait(lock);
        }
        count--;
    }
private:
    std::mutex mtx;
    std::condition_variable cv;
    int count;
};

