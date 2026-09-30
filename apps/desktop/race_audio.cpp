#include "race_audio.hpp"
#include "race_samples.hpp"
#include <QAudioSink>
#include <QAudioDevice>
#include <QMediaDevices>
#include <QStandardPaths>
#include <QDir>
#include <QFileInfo>
#include <QTimer>
#include <QtEndian>
#include <algorithm>
#include <bit>
#include <vector>

// A dedicated output thread keeps the audio stream fed during an expensive planner/render/UI operation.
// All synthesis and device objects belong to it; only immutable frame/mix copies cross the queued seam.
class AudioWorker : public QObject {
public:
    explicit AudioWorker(RaceAudio* owner):owner_(owner) {}
    void start() {
        devices_=new QMediaDevices(this);
        connect(devices_,&QMediaDevices::audioOutputsChanged,this,[this]{open();});
        timer_=new QTimer(this); timer_->setTimerType(Qt::PreciseTimer); timer_->setInterval(8);
        connect(timer_,&QTimer::timeout,this,[this]{pump();});
        open(); timer_->start();
    }
    void stop() { if(timer_) timer_->stop(); if(sink_) sink_->reset(); sink_.reset(); output_=nullptr; }
    void follow(race_sound::Frame value) { frame_=value; if(synth_) synth_->follow(value); }
    void mix(race_sound::Mix value) { mix_=value; if(synth_) synth_->mix(value); }
    void cue(race_sound::Cue value) { if(synth_) synth_->cue(value); }
private:
    void status(QString text) {
        QMetaObject::invokeMethod(owner_,[owner=owner_,text]{
            if(owner->status_!=text) { owner->status_=text; emit owner->changed(); }
        },Qt::QueuedConnection);
    }
    void open() {
        if(sink_) sink_->reset(); sink_.reset(); output_=nullptr; pending_.clear(); offset_=0;
        const auto device=QMediaDevices::defaultAudioOutput();
        if(device.isNull()) { status("No audio output. Connect headphones or speakers."); return; }
        QAudioFormat format;
        format.setSampleFormat(QAudioFormat::Int16);
        bool supported=false;
        for(int rate:{48000,44100}) {
            for(int channels:{2,1}) {
                format.setSampleRate(rate); format.setChannelCount(channels);
                if(device.isFormatSupported(format)) { supported=true; break; }
            }
            if(supported) break;
        }
        if(!supported) format=device.preferredFormat();
        if(!format.isValid() || format.sampleRate()<22050 || format.sampleRate()>192000 ||
           format.channelCount()>8 || !device.isFormatSupported(format)) {
            status("This audio output does not support the engine format."); return;
        }
        channels_=format.channelCount(); sample_format_=format.sampleFormat(); sample_bytes_=format.bytesPerSample();
        samples_.resize(format.sampleRate()/100);
        try { synth_=std::make_unique<race_sound::Synth>(format.sampleRate(),race_sound::recorded_bank()); }
        catch(const std::exception&) { status("Engine recordings could not be loaded. Rebuild the audio assets."); return; }
        synth_->mix(mix_); synth_->follow(frame_);
        sink_=std::make_unique<QAudioSink>(device,format);
        sink_->setBufferSize(format.bytesForDuration(50000)); // request 50 ms, instead of the large default buffer.
        connect(sink_.get(),&QAudioSink::stateChanged,this,[this](QtAudio::State state){
            if(state==QtAudio::StoppedState && sink_->error()!=QtAudio::NoError)
                status("Audio output unavailable. Reconnect it to retry.");
        });
        output_=sink_->start();
        if(!output_) { status("Could not start the audio output."); return; }
        status("Recorded Formula V10 · 7 virtual gears");
    }
    void pump() {
        if(!output_ || !sink_ || sink_->state()==QtAudio::StoppedState) return;
        // Produce small blocks only as the device consumes them. Keep any partial write for next time.
        const auto blockBytes=qsizetype(samples_.size()*channels_*sample_bytes_);
        for(int block=0;block<12 && sink_->bytesFree()>=blockBytes;++block) {
            if(pending_.isEmpty()) {
                synth_->render(samples_); pending_.resize(blockBytes); offset_=0;
                for(std::size_t i=0;i<samples_.size();++i) {
                    for(int ch=0;ch<channels_;++ch) {
                        const float sample=ch<2 ? std::clamp(samples_[i],-1.0f,1.0f) : 0;
                        auto* dest=pending_.data()+(i*channels_+ch)*sample_bytes_;
                        switch(sample_format_) {
                        case QAudioFormat::Float: qToLittleEndian(std::bit_cast<quint32>(sample),dest); break;
                        case QAudioFormat::Int16: qToLittleEndian(qint16(sample*32767),dest); break;
                        case QAudioFormat::Int32: qToLittleEndian(qint32(double(sample)*2147483647),dest); break;
                        case QAudioFormat::UInt8: *dest=char(int(sample*127)+128); break;
                        default: break;
                        }
                    }
                }
            }
            const auto written=output_->write(pending_.constData()+offset_,pending_.size()-offset_);
            if(written<=0) break;
            offset_+=written;
            if(offset_==pending_.size()) pending_.clear(); else break;
        }
        if(++pumps_%125==0) {
            const auto us=sink_->processedUSecs(); const int error=sink_->error(), buffer=int(sink_->bufferSize());
            QMetaObject::invokeMethod(owner_,[owner=owner_,us,error,buffer]{emit owner->deviceProgress(us,error,buffer);},Qt::QueuedConnection);
        }
    }
    RaceAudio* owner_;
    QMediaDevices* devices_{};
    QTimer* timer_{};
    std::unique_ptr<QAudioSink> sink_;
    QIODevice* output_{};
    std::unique_ptr<race_sound::Synth> synth_;
    std::vector<float> samples_;
    QByteArray pending_;
    qsizetype offset_{};
    int channels_{2}, sample_bytes_{2}, pumps_{};
    QAudioFormat::SampleFormat sample_format_{QAudioFormat::Int16};
    race_sound::Frame frame_;
    race_sound::Mix mix_;
};
RaceAudio::RaceAudio(bool hardware,QString settingsPath,QObject* parent):QObject(parent) {
    if(settingsPath.isEmpty()) settingsPath=QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation)+"/sound.ini";
    QDir().mkpath(QFileInfo(settingsPath).absolutePath());
    settings_=std::make_unique<QSettings>(settingsPath,QSettings::IniFormat);
    mix_.enabled=settings_->value("enabled",true).toBool(); mix_.engine=settings_->value("engine",true).toBool();
    mix_.cues=settings_->value("cues",true).toBool();
    bool okay=false; const auto volume=settings_->value("volume",45).toInt(&okay);
    mix_.volume=(okay ? std::clamp(volume,0,100) : 45)/100.0;
    status_=hardware ? "Starting audio..." : "Audio output disabled for verification";
    if(hardware) {
        worker_=new AudioWorker(this); worker_->moveToThread(&thread_);
        connect(&thread_,&QThread::finished,worker_,&QObject::deleteLater);
        thread_.start();
        QMetaObject::invokeMethod(worker_,[this]{worker_->mix(mix_); worker_->start();},Qt::BlockingQueuedConnection);
    }
}
RaceAudio::~RaceAudio() {
    if(worker_) {
        QMetaObject::invokeMethod(worker_,[this]{worker_->stop();},Qt::BlockingQueuedConnection);
        thread_.quit(); thread_.wait();
    }
}
void RaceAudio::commit() {
    settings_->setValue("enabled",mix_.enabled); settings_->setValue("engine",mix_.engine);
    settings_->setValue("cues",mix_.cues); settings_->setValue("volume",volume()); settings_->sync();
    if(settings_->status()!=QSettings::NoError) status_="Sound works, but settings could not be saved.";
    if(worker_) QMetaObject::invokeMethod(worker_,[worker=worker_,value=mix_]{worker->mix(value);},Qt::QueuedConnection);
    emit changed();
}
void RaceAudio::setEnabled(bool value) { if(value!=mix_.enabled) { mix_.enabled=value; commit(); } }
void RaceAudio::setEngine(bool value) { if(value!=mix_.engine) { mix_.engine=value; commit(); } }
void RaceAudio::setCues(bool value) { if(value!=mix_.cues) { mix_.cues=value; commit(); } }
void RaceAudio::setVolume(int value) { if(value>=0 && value<=100 && value!=volume()) { mix_.volume=value/100.0; commit(); } }
void RaceAudio::follow(race_sound::Frame value) {
    eligible_=value.eligible;
    const bool driving=value.eligible && value.running;
    if(driving!=driving_) { driving_=driving; emit changed(); }
    if(worker_) QMetaObject::invokeMethod(worker_,[worker=worker_,value]{worker->follow(value);},Qt::QueuedConnection);
}
void RaceAudio::play(race_sound::Cue value) {
    if(worker_) QMetaObject::invokeMethod(worker_,[worker=worker_,value]{worker->cue(value);},Qt::QueuedConnection);
}
void RaceAudio::feedback() { if(eligible_) play(race_sound::Cue::tap); }
void RaceAudio::previewLap() { if(eligible_) play(race_sound::Cue::lap); }
