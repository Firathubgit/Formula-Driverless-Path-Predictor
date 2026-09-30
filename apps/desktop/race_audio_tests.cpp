#include "race_audio.hpp"
#include "race_samples.hpp"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QFile>
#include <QDataStream>
#include <QDir>
#include <QTimer>
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace race_sound;
namespace {
std::vector<float> render(Synth& synth,double seconds) {
    std::vector<float> samples(static_cast<std::size_t>(48000*seconds)); synth.render(samples); return samples;
}
double rms(const std::vector<float>& samples) {
    double sum=0; for(double s:samples) sum+=s*s; return std::sqrt(sum/samples.size());
}
void wav(const QString& path,const std::vector<float>& samples) {
    QFile file(path); if(!file.open(QIODevice::WriteOnly)) throw std::runtime_error("Cannot write preview WAV");
    QDataStream out(&file); out.setByteOrder(QDataStream::LittleEndian);
    out.writeRawData("RIFF",4); out<<quint32(36+samples.size()*2); out.writeRawData("WAVEfmt ",8);
    out<<quint32(16)<<quint16(1)<<quint16(1)<<quint32(48000)<<quint32(96000)<<quint16(2)<<quint16(16);
    out.writeRawData("data",4); out<<quint32(samples.size()*2);
    for(float s:samples) out<<qint16(std::clamp(s,-1.0f,1.0f)*32767);
}
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv); QTemporaryDir temporary;
    const auto args=app.arguments();
    // Opt-in integration probe of the real default device. Normal CTest is silent and uses isolated preferences.
    const int device=args.indexOf("--device-check");
    if(device>=0 && device+1<args.size()) {
        RaceAudio sound(true,temporary.filePath("device.ini")); sound.setVolume(25);
        Frame frame{true,true,0,0,0.6,0,0,16};
        QTimer clock; clock.setInterval(20);
        QObject::connect(&clock,&QTimer::timeout,&app,[&] { frame.time+=0.02; frame.speed=std::min(13.0,frame.time*3); sound.follow(frame); });
        qint64 processed=0; int lastError=-1,buffer=0;
        QObject::connect(&sound,&RaceAudio::deviceProgress,&app,[&](qint64 us,int error,int bytes){processed=us;lastError=error;buffer=bytes;});
        clock.start();
        QTimer::singleShot(2500,&app,[&]{sound.previewLap();});
        QTimer::singleShot(4400,&app,[&]{
            const bool passed=processed>2000000 && lastError==0;
            QFile file(args[device+1]);
            if(file.open(QIODevice::WriteOnly)) file.write(QJsonDocument(QJsonObject{{"passed",passed},{"processed_us",double(processed)},
                {"error",lastError},{"buffer_bytes",buffer},{"status",sound.status()}}).toJson());
            std::cout<<"Device: "<<sound.status().toStdString()<<", processed="<<processed<<", error="<<lastError<<", buffer="<<buffer<<'\n';
            app.exit(passed?0:2);
        });
        return app.exec();
    }
    int checks=0;
    const auto check=[&](bool okay,const char* message){++checks;if(!okay)throw std::runtime_error(message);};
    try {
        const auto bank=recorded_bank();
        check(bank->engine.sample_rate()==44100 && bank->engine.size()>110000 && bank->tires.sample_rate()==96000,
              "real Formula V10 and tire recordings are embedded and decoded");
        for(const auto& corrupt: {QByteArray("not a WAV"),QByteArray("RIFF\xff\xff\xff\xffWAVE",12)}) {
            bool rejected=false; try {decode_wave(corrupt);} catch(const std::invalid_argument&) {rejected=true;}
            check(rejected,"invalid/truncated asset is rejected before streaming");
        }
        QFile source(":/audio/f1v10.wav"); source.open(QIODevice::ReadOnly); auto damaged=source.readAll();
        damaged[20]=char(3); bool rejected=false;
        try {decode_wave(damaged);} catch(const std::invalid_argument&) {rejected=true;}
        check(rejected,"non-PCM data cannot be interpreted as engine samples");
        // A high source partial would alias to 16 kHz at 2x without rate-aware filtering.
        std::vector<float> probe(48000);
        constexpr double tau=6.283185307179586;
        for(int i=0;i<48000;++i) probe[i]=float(0.4*std::sin(tau*800*i/48000)+0.4*std::sin(tau*16000*i/48000));
        SampleLoop loop(std::move(probe),48000); SamplePlayer player;
        double low_s=0,low_c=0,alias_s=0,alias_c=0;
        for(int i=0;i<48000;++i) {
            const double s=player.next(loop,2);
            low_s+=s*std::sin(tau*1600*i/48000); low_c+=s*std::cos(tau*1600*i/48000);
            alias_s+=s*std::sin(tau*16000*i/48000); alias_c+=s*std::cos(tau*16000*i/48000);
        }
        check(std::hypot(low_s,low_c)>100 && std::hypot(alias_s,alias_c)<std::hypot(low_s,low_c)*0.01,
              "pitched playback retains the engine partial and suppresses folded high frequencies");
        check(player.next(loop,std::numeric_limits<double>::quiet_NaN())==0 && player.next(loop,0)==0,
              "bad playback rates cannot corrupt the loop cursor");
        Synth silent;
        check(rms(render(silent,0.1))==0,"unstarted sound is silent");
        Frame frame{true,true,0,0,1,0,0,16};
        Synth synth(48000,bank); synth.mix({true,true,true,1}); synth.follow(frame);
        render(synth,0.3);
        check(synth.gear()==1 && synth.rpm()>6000 && synth.rpm()<7000,"stationary gas revs in first without shifting");
        int before=1; bool drop=false;
        std::vector<float> demo;
        for(int i=1;i<=1000;++i) {
            frame.time=i*0.01; frame.speed=std::min(16.0,frame.time*1.8);
            const double previous=synth.target_rpm(); synth.follow(frame);
            if(synth.gear()>before) { drop=drop || synth.target_rpm()<previous; before=synth.gear(); }
            const auto block=render(synth,0.01); demo.insert(demo.end(),block.begin(),block.end());
        }
        check(synth.gear()==7 && synth.upshifts()==6 && drop,"acceleration climbs seven gears with RPM drops");
        const int shifts=synth.upshifts()+synth.downshifts();
        for(int i=0;i<100;++i) {frame.time+=0.01;frame.speed=16+0.02*std::sin(i);synth.follow(frame);render(synth,0.01);}
        check(synth.upshifts()+synth.downshifts()==shifts,"steady speed does not hunt gears");
        frame.throttle=0; frame.brake=0.8;
        for(int i=0;i<600;++i) {
            frame.time+=0.01; frame.speed=std::max(0.0,16-i*0.03); synth.follow(frame);
            const auto block=render(synth,0.01); demo.insert(demo.end(),block.begin(),block.end());
        }
        check(synth.gear()==1 && synth.downshifts()==6,"braking steps down through gears");
        frame.speed=7; frame.brake=0; frame.throttle=1; frame.time+=1; synth.follow(frame); render(synth,0.3);
        const auto on=rms(render(synth,0.3)); frame.throttle=0; frame.time+=0.01; synth.follow(frame); render(synth,0.3);
        check(on>rms(render(synth,0.3))*1.5,"gas has a stronger loaded tone than coast at the same speed");
        frame.slip=1; frame.time+=0.01; synth.follow(frame); check(synth.scrub()>0.9,"sliding at speed raises tire scrub");
        frame.slip=0; synth.follow(frame); check(synth.scrub()==0,"no slip gives no squeal even at speed");
        frame.slip=1; frame.speed=0; synth.follow(frame); check(synth.scrub()==0,"stationary brakes cannot squeal");
        frame.laps=1; frame.previous_lap=52; frame.best_lap=52; synth.follow(frame);
        for(int i=0;i<10;++i) synth.follow(frame);
        check(synth.lap_chimes()==1 && synth.best_chimes()==0,"first complete lap plings only once");
        auto block=render(synth,1); demo.insert(demo.end(),block.begin(),block.end());
        frame.laps=2; frame.previous_lap=50; frame.best_lap=50; synth.follow(frame);
        check(synth.best_chimes()==1,"improved full lap gets best-lap cue");
        block=render(synth,1); demo.insert(demo.end(),block.begin(),block.end());
        frame.laps=3; frame.previous_lap=53; synth.follow(frame);
        check(synth.lap_chimes()==2 && synth.best_chimes()==1,"slower full lap gets ordinary cue");
        frame.time=0; synth.follow(frame); synth.follow(frame);
        check(synth.lap_chimes()==2 && synth.gear()==1,"restart with retained lap history never replays a completion");
        frame.running=false; synth.follow(frame); render(synth,1);
        check(rms(render(synth,0.1))<1e-10,"pause fades to silence");
        frame.eligible=false; frame.running=true; frame.laps=4; synth.follow(frame); render(synth,1);
        check(rms(render(synth,0.1))<1e-10 && synth.lap_chimes()==2,"showroom/replay/AI stay silent and do not chime");
        frame.eligible=true; synth.follow(frame); synth.mix({false,true,true,1}); synth.cue(Cue::lap); render(synth,1);
        check(rms(render(synth,0.1))<1e-10,"master mute silences engine and events");
        synth.mix({true,false,true,1}); synth.cue(Cue::lap);
        check(rms(render(synth,0.2))>0.01,"lap cues can play with engine off");
        synth.mix({true,false,false,1}); synth.cue(Cue::best); render(synth,1);
        check(rms(render(synth,0.1))<1e-10,"cue toggle suppresses queued cues");
        synth.mix({true,true,true,0}); render(synth,1); synth.cue(Cue::lap);
        check(rms(render(synth,0.1))<1e-10,"zero volume silences all channels");
        synth.mix({true,true,true,1}); frame.speed=std::numeric_limits<double>::quiet_NaN();
        frame.throttle=std::numeric_limits<double>::infinity(); frame.slip=-2; synth.follow(frame);
        auto invalid=render(synth,0.2);
        check(std::all_of(invalid.begin(),invalid.end(),[](float s){return std::isfinite(s)&&std::abs(s)<=0.9f;}),"bad input remains finite and bounded");
        check(std::all_of(demo.begin(),demo.end(),[](float s){return std::isfinite(s)&&std::abs(s)<0.9f;}),"whole demo is finite with limiter headroom");
        double sum=0,peak=0;for(double s:demo){sum+=s;peak=std::max(peak,std::abs(s));}
        check(std::abs(sum/demo.size())<0.001,"engine waveform has negligible DC");
        // Chunk boundaries do not change the waveform or clock (no phase restarts at render frames).
        Synth a(48000,bank),b(48000,bank); Frame steady{true,true,0,5,0.7,0,0,16}; a.follow(steady); b.follow(steady);
        auto whole=render(a,3); std::vector<float> split;
        for(int i=0;i<300;++i){auto piece=render(b,0.01);split.insert(split.end(),piece.begin(),piece.end());}
        check(whole==split,"synthesis is identical across output buffer chunking");
        for(int rate:{22050,44100,96000,192000}) {
            Synth alternate(rate,bank); alternate.follow({true,true,0,20,1,0,1,16});
            std::vector<float> samples(rate); alternate.render(samples);
            check(rms(samples)>0.005 && std::all_of(samples.begin(),samples.end(),[](float s){return std::isfinite(s)&&std::abs(s)<0.9f;}),
                  "recordings remain audible and bounded at supported device rates");
        }
        const auto path=temporary.filePath("sound.ini");
        {
            RaceAudio audio(false,path); check(audio.enabled() && audio.volume()==45,"moderate initial volume");
            audio.setEnabled(false); audio.setVolume(63); audio.setEngine(false); audio.setCues(false);
            audio.setVolume(101);check(audio.volume()==63,"invalid volume rejected");
        }
        {
            RaceAudio audio(false,path); check(!audio.enabled() && audio.volume()==63 && !audio.engine() && !audio.cues(),"all sound preferences persist locally");
            audio.follow(steady); check(audio.driving(),"live human drive accepted"); steady.eligible=false;
            audio.follow(steady); check(!audio.driving(),"presentation exclusion accepted");
        }
        const int output=args.indexOf("--demo");
        if(output>=0 && output+1<args.size()) wav(args[output+1],demo);
        std::cout<<checks<<" audio checks passed; demo peak="<<peak<<", RMS="<<rms(demo)<<", mean="<<sum/demo.size()<<'\n';
    } catch(const std::exception& e) { std::cerr<<"Audio check "<<checks<<": "<<e.what()<<'\n';return 1; }
    return 0;
}
