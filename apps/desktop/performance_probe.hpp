#pragma once
#include <chrono>

// Opt-in CPU measurements for --benchmark-kart; no clock reads in ordinary runs.
struct DesktopPerformance {
    bool enabled{};
    double advance_ms{}, publish_ms{}, geometry_ms{};
    int advances{}, publishes{}, geometries{};
};
inline thread_local DesktopPerformance desktop_performance;
class PerformanceScope {
public:
    PerformanceScope(double& total, int& calls) : total_(total), calls_(calls), enabled_(desktop_performance.enabled) {
        if (enabled_) start_=std::chrono::steady_clock::now();
    }
    ~PerformanceScope() {
        if (enabled_) {
            total_+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start_).count();
            ++calls_;
        }
    }
private:
    double& total_;
    int& calls_;
    bool enabled_;
    std::chrono::steady_clock::time_point start_;
};
