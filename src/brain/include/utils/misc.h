#pragma once

#include "chrono"
#include <string>
#include <random>
#include <sstream>

// Calculate how many milliseconds have passed since start_time
inline int msecsSince(std::chrono::high_resolution_clock::time_point start_time)
{
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}