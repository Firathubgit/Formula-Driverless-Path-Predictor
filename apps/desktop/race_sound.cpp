#include "race_sound.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace race_sound {
namespace {
constexpr double tau=2*std::numbers::pi;
constexpr std::array<double,7> ratios{0.29,0.385,0.50,0.62,0.75,0.90,1.06};
double bounded(double x,double lo,double hi) { return std::isfinite(x) ? std::clamp(x,lo,hi) : lo; }
double approach(double value,double target,double factor) { return value+(target-value)*factor; }
}
Synth::Synth(int sample_rate,std::shared_ptr<const Bank> bank):sample_rate_(sample_rate),bank_(std::move(bank)) {
    if(sample_rate<22050 || sample_rate>192000) throw std::invalid_argument("Unsupported audio sample rate");
}
void Synth::mix(Mix value) {
    value.volume=bounded(value.volume,0,1); mix_=value;
    if(!value.enabled || !value.cues || value.volume==0) for(auto& voice:voices_) voice.age=10;
}
void Synth::follow(Frame value) {
    value.time=bounded(value.time,0,1e9); value.speed=bounded(value.speed,0,200);
    value.throttle=bounded(value.throttle,0,1); value.brake=bounded(value.brake,0,1);
    value.slip=bounded(value.slip,0,1); value.top_speed=bounded(value.top_speed,3,150);
    const bool reset=!seen_ || value.eligible!=frame_.eligible || value.time<frame_.time;
    const double dt=reset ? 0 : std::clamp(value.time-frame_.time,0.0,1.0);
    if(reset) {
        gear_=1; cooldown_=0; shift_cut_=blip_=0;
        for(auto& voice:voices_) voice.age=10;
    }
    if(value.eligible) {
        if(!reset && value.laps==frame_.laps+1 && value.previous_lap>0) {
            const bool best=frame_.best_lap>0 && value.previous_lap<frame_.best_lap-1e-6;
            if(best) ++best_chimes_; else ++lap_chimes_;
            cue(best ? Cue::best : Cue::lap);
        } else if(!reset && value.running!=frame_.running) cue(value.running ? Cue::start : Cue::pause);
    }
    frame_=value; seen_=true;
    if(!value.eligible || !value.running) { target_scrub_=0; return; }
    cooldown_=std::max(0.0,cooldown_-dt);
    const auto rpm_for=[&](int g) { return 16000*value.speed/(value.top_speed*ratios[g-1]); };
    if(value.speed<0.5) { gear_=1; cooldown_=0; }
    else if(cooldown_==0) {
        if(gear_<7 && rpm_for(gear_)>15800 && value.brake<0.15 && value.throttle>0.12) {
            ++gear_; ++upshifts_; cooldown_=0.22; shift_cut_=0.055;
        } else if(gear_>1 && rpm_for(gear_-1)<10500) {
            --gear_; ++downshifts_; cooldown_=0.22; blip_=0.10;
        }
    }
    target_rpm_=std::clamp(std::max(4200+2200*value.throttle*(1-value.brake),rpm_for(gear_)),4200.0,17800.0);
    // No fake sliding noise from the steering stick. The caller supplies measured plant slip.
    target_scrub_=value.slip*std::clamp((value.speed-1)/4,0.0,1.0);
}
void Synth::note(double hz,double duration,double delay,double gain) {
    auto at=std::max_element(voices_.begin(),voices_.end(),[](const Voice& a,const Voice& b){return a.age<b.age;});
    *at={0,hz,duration,delay,gain};
}
void Synth::cue(Cue value) {
    if(!mix_.enabled || !mix_.cues || mix_.volume==0) return;
    // A lap notification takes priority over little UI ticks; bounded polyphony prevents spam.
    if(value==Cue::lap || value==Cue::best) for(auto& voice:voices_) voice.age=10;
    if(value==Cue::tap) { if(clock_-last_tap_<0.12) return; last_tap_=clock_; note(880,0.055,0,0.09); }
    if(value==Cue::start) { note(523.25,0.10,0,0.07); note(783.99,0.15,0.075,0.09); }
    if(value==Cue::pause) note(392,0.09,0,0.07);
    if(value==Cue::lap) { note(1046.5,0.5,0,0.23); note(1568,0.32,0.05,0.10); }
    if(value==Cue::best) { note(1046.5,0.34,0,0.20); note(1318.51,0.38,0.13,0.20); note(1568,0.55,0.26,0.24); }
}
double Synth::noise() {
    random_^=random_<<13; random_^=random_>>17; random_^=random_<<5;
    return static_cast<double>(random_)/2147483648.0-1;
}
void Synth::render(std::span<float> mono) {
    const double dt=1.0/sample_rate_;
    const double smooth=1-std::exp(-dt/0.025);
    for(auto& out:mono) {
        clock_+=dt;
        const bool driving=frame_.eligible && frame_.running && mix_.engine;
        gain_=approach(gain_,mix_.enabled ? mix_.volume*mix_.volume : 0,smooth);
        engine_gain_=approach(engine_gain_,driving ? 1 : 0,smooth);
        cue_gain_=approach(cue_gain_,mix_.cues ? 1 : 0,smooth);
        // A short ignition cut, followed by a crisp restoration of the recorded exhaust.
        const double shift=shift_cut_>0.020 ? 0.12 : (shift_cut_>0 ? 0.12+0.88*(1-shift_cut_/0.020) : 1);
        const double blip=std::max(0.0,blip_/0.10);
        load_=approach(load_,std::max(frame_.throttle*(1-frame_.brake),0.85*blip),smooth);
        rpm_=approach(rpm_,target_rpm_+1500*blip,1-std::exp(-dt/(shift_cut_>0 ? 0.016 : 0.050)));
        const double n=noise(); air_=approach(air_,n,0.12); rumble_=approach(rumble_,n,0.015);
        // Speed Dreams' Formula V10 sample carries the combustion/exhaust texture.
        // Its ~580 Hz firing fundamental gives a chosen audio calibration of 6960 RPM.
        // Both load colours share a cursor, so the transition cannot beat or restart a loop.
        const double exhaust=bank_ ? engine_player_.next(bank_->engine,rpm_/6960*bank_->engine.sample_rate()/sample_rate_) : 0;
        const double filter=1-std::exp(-tau*1800/sample_rate_);
        exhaust_low_=approach(exhaust_low_,exhaust,filter);
        exhaust_low2_=approach(exhaust_low2_,exhaust_low_,filter);
        limiter_=approach(limiter_,rpm_>17500 && frame_.throttle>0.5 && std::fmod(clock_*42,1.0)<0.45 ? 0.18 : 1,
                          1-std::exp(-dt/0.002));
        shift_gain_=approach(shift_gain_,shift,1-std::exp(-dt/0.002));
        const double engine=shift_gain_*limiter_*(0.28*exhaust_low2_*(1-load_)+0.95*exhaust*load_);
        scrub_=approach(scrub_,target_scrub_,smooth);
        const double speed=std::clamp(frame_.speed/frame_.top_speed,0.0,1.4);
        const double tire_sample=bank_ ? tire_player_.next(bank_->tires,(0.82+0.20*speed)*bank_->tires.sample_rate()/sample_rate_) : 0;
        const double tires=scrub_*0.48*tire_sample;
        const double road=0.045*speed*rumble_+0.015*speed*speed*air_;
        double cues=0; bool chiming=false;
        for(auto& voice:voices_) {
            voice.age+=dt;
            const double t=voice.age-voice.delay;
            if(t<0 || t>=voice.duration) continue;
            chiming=chiming || voice.gain>=0.20;
            const double envelope=std::min(1.0,t/0.006)*std::exp(-5*t/voice.duration)*std::min(1.0,(voice.duration-t)/0.015);
            cues+=voice.gain*envelope*(std::sin(tau*voice.frequency*t)+0.22*std::sin(tau*voice.frequency*2.003*t));
        }
        duck_=approach(duck_,chiming ? 0.45 : 1,1-std::exp(-dt/(chiming ? 0.012 : 0.20)));
        const double mixed=gain_*(engine_gain_*duck_*(engine+tires+road)+cue_gain_*cues);
        // DC removal and soft peak limiting. Smoothed gain avoids clicks on pause, mute and slider changes.
        const double dc=mixed-dc_in_+0.995*dc_out_; dc_in_=mixed; dc_out_=dc;
        out=static_cast<float>(0.90*std::tanh(dc/0.90));
        shift_cut_=std::max(0.0,shift_cut_-dt); blip_=std::max(0.0,blip_-dt);
    }
}
}
