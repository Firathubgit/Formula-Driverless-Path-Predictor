#pragma once
#include <QObject>
#include <QTimer>
#include <QUrl>
#include <QVariantList>
#include <array>

// Presentation only. No vehicle dynamics, simulation clock or recording state lives here.
// Every directed A -> B edge is exit(A) then enter(B), meeting at the same empty black frame.
class Showroom : public QObject {
    Q_OBJECT
    Q_PROPERTY(QVariantList cars READ cars CONSTANT)
    Q_PROPERTY(QString screen READ screen NOTIFY changed)
    Q_PROPERTY(QString phase READ phase NOTIFY changed)
    Q_PROPERTY(int current READ current NOTIFY changed)
    Q_PROPERTY(int requested READ requested NOTIFY changed)
    Q_PROPERTY(QString currentName READ currentName NOTIFY changed)
    Q_PROPERTY(QUrl poster READ poster NOTIFY changed)
    Q_PROPERTY(QUrl clip READ clip NOTIFY playbackChanged)
    Q_PROPERTY(int serial READ serial NOTIFY playbackChanged)
    Q_PROPERTY(bool looping READ looping NOTIFY playbackChanged)
    Q_PROPERTY(bool confirming READ confirming NOTIFY changed)
    Q_PROPERTY(QString status READ status NOTIFY changed)
public:
    explicit Showroom(const QString& mediaDirectory, QObject* parent = nullptr);
    QVariantList cars() const;
    QString screen() const { return screen_; }
    QString phase() const { return phase_; }
    int current() const { return current_; }
    int requested() const { return requested_; }
    QString currentName() const;
    QUrl poster() const;
    QUrl clip() const { return clip_; }
    int serial() const { return serial_; }
    // Idle completes one orbit before dispatching queued work; its first and last frames are the hero endpoint.
    bool looping() const { return false; }
    bool confirming() const { return confirming_; }
    QString status() const { return status_; }
    Q_INVOKABLE void open();
    Q_INVOKABLE void request(int index);
    Q_INVOKABLE void confirm();
    Q_INVOKABLE void complete(int serial);
    Q_INVOKABLE void failed(int serial, const QString& error);
    Q_INVOKABLE void started(int serial);
    Q_INVOKABLE void bypass();
    Q_INVOKABLE void startDriving();
signals:
    void changed();
    void playbackChanged();
    void playbackStarted(int serial);
    void selected(int appearance);
private:
    struct Car { QString id, name, subtitle; QUrl poster, enter, idle, exit, select, logo; };
    std::array<Car, 4> cars_;
    QString screen_{"driving"}, phase_{"off"}, status_;
    QUrl clip_;
    int current_{}, requested_{}, serial_{};
    bool confirming_{};
    QTimer watchdog_;
    void play(const QString& phase);
    void settle();
    void advance();
};
