#include "sample_loop.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace race_sound {
namespace {
constexpr double pi=std::numbers::pi;
float interpolate(const std::vector<float>& data, double phase) {
    const auto i=static_cast<std::size_t>(phase)%data.size();
    const double fraction=phase-std::floor(phase);
    return static_cast<float>(data[i]+fraction*(data[(i+1)%data.size()]-data[i]));
}
}
SampleLoop::SampleLoop(std::vector<float> pcm,int sample_rate,double target_rms):sample_rate_(sample_rate) {
    if(sample_rate<22050 || sample_rate>192000 || pcm.size()<std::size_t(sample_rate/5) ||
       pcm.size()>std::size_t(sample_rate)*30 || !std::isfinite(target_rms) || target_rms<=0 || target_rms>0.5)
        throw std::invalid_argument("Invalid engine sample dimensions");
    double mean=0;
    for(float s:pcm) { if(!std::isfinite(s) || std::abs(s)>1) throw std::invalid_argument("Invalid PCM sample"); mean+=s; }
    mean/=pcm.size();
    for(auto& s:pcm) s-=static_cast<float>(mean);
    // Overlap the tail and head over 60 ms, preserving their continuous texture.
    // Start at the end of that head: after the blended tail it joins sample[fade].
    const auto fade=std::size_t(sample_rate*0.060), length=(pcm.size()-fade)/16*16;
    pcm.resize(length+fade);
    std::vector<float> loop(length);
    for(std::size_t i=0;i<length;++i) {
        loop[i]=pcm[i+fade];
        if(i>=length-fade) {
            const double t=double(i-(length-fade))/fade;
            loop[i]=static_cast<float>(loop[i]*std::cos(t*pi/2)+pcm[i-(length-fade)]*std::sin(t*pi/2));
        }
    }
    // A multiple of 16 gives all filtered levels exactly the same period.
    double power=0,peak=0;
    for(float s:loop) {power+=s*s;peak=std::max(peak,std::abs(double(s)));}
    if(power<1e-10) throw std::invalid_argument("Silent engine sample");
    const double gain=std::min(target_rms/std::sqrt(power/loop.size()),0.92/peak);
    for(auto& s:loop) s=static_cast<float>(s*gain);
    levels_.push_back(std::move(loop));
    // Circular 63-tap windowed-sinc half-band downsampling, off the real-time path.
    std::vector<double> kernel(63); double total=0;
    for(int j=-31;j<=31;++j) {
        const double sinc=j==0 ? 0.44 : std::sin(2*pi*0.22*j)/(pi*j);
        kernel[j+31]=sinc*(0.42+0.5*std::cos(pi*j/31)+0.08*std::cos(2*pi*j/31));
        total+=kernel[j+31];
    }
    for(auto& w:kernel) w/=total;
    for(int level=1;level<=4;++level) {
        const auto& source=levels_.back(); const auto n=static_cast<long long>(source.size());
        std::vector<float> half(source.size()/2);
        for(std::size_t i=0;i<half.size();++i) {
            double sample=0;
            for(int j=-31;j<=31;++j) sample+=kernel[j+31]*source[(static_cast<long long>(i*2)+j+n)%n];
            half[i]=static_cast<float>(sample);
        }
        levels_.push_back(std::move(half));
    }
}
float SampleLoop::read(double phase,double rate) const {
    // Begin fading to the lower-bandwidth level before its Nyquist limit is reached.
    const double level=std::clamp(1+std::log2(std::max(0.5,rate)),0.0,4.0);
    const int lo=static_cast<int>(level), hi=std::min(lo+1,4);
    const double a=interpolate(levels_[lo],phase*levels_[lo].size()/size());
    const double b=interpolate(levels_[hi],phase*levels_[hi].size()/size());
    return static_cast<float>(a+(b-a)*(level-lo));
}
float SamplePlayer::next(const SampleLoop& loop,double rate) {
    if(!std::isfinite(rate) || rate<=0 || rate>16) return 0;
    const auto sample=loop.read(phase_,rate);
    phase_+=rate;
    if(phase_>=loop.size()) phase_=std::fmod(phase_,double(loop.size()));
    return sample;
}
}
