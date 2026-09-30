#pragma once
#include "race_sound.hpp"
#include <QObject>
#include <QSettings>
#include <QThread>
#include <memory>

class AudioWorker;
class RaceAudio : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY changed)
    Q_PROPERTY(bool engine READ engine WRITE setEngine NOTIFY changed)
    Q_PROPERTY(bool cues READ cues WRITE setCues NOTIFY changed)
    Q_PROPERTY(int volume READ volume WRITE setVolume NOTIFY changed)
    Q_PROPERTY(QString status READ status NOTIFY changed)
    Q_PROPERTY(bool driving READ driving NOTIFY changed)
public:
    explicit RaceAudio(bool hardware=true, QString settingsPath={}, QObject* parent=nullptr);
    ~RaceAudio() override;
    bool enabled() const { return mix_.enabled; }
    bool engine() const { return mix_.engine; }
    bool cues() const { return mix_.cues; }
    int volume() const { return qRound(mix_.volume*100); }
    QString status() const { return status_; }
    bool driving() const { return driving_; }
    void setEnabled(bool value);
    void setEngine(bool value);
    void setCues(bool value);
    void setVolume(int value);
    void follow(race_sound::Frame value);
    Q_INVOKABLE void feedback();
    Q_INVOKABLE void previewLap();
signals:
    void changed();
    void deviceProgress(qint64 microseconds, int error, int bufferBytes);
private:
    friend class AudioWorker;
    void commit();
    void play(race_sound::Cue value);
    std::unique_ptr<QSettings> settings_;
    QThread thread_;
    AudioWorker* worker_{};
    race_sound::Mix mix_;
    QString status_;
    bool driving_{}, eligible_{};
};
