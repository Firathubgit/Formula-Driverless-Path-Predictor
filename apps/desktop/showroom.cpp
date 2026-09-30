#include "showroom.hpp"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

Showroom::Showroom(const QString& directory, QObject* parent) : QObject(parent), cars_{{
    {"amr23", "Aston Martin", "AMR23 · 2023", {}, {}, {}, {}, {}},
    {"jesko", "Koenigsegg", "Jesko Attack · 2024", {}, {}, {}, {}, {}},
    {"urus", "Lamborghini", "Urus · 2023", {}, {}, {}, {}, {}},
    {"rb19", "Red Bull Racing", "RB19 · 2023", {}, {}, {}, {}, {}}
}} {
    const QDir root(QFileInfo(directory).absoluteFilePath());
    QFile file(root.filePath("manifest.json"));
    QJsonObject manifest;
    if (file.open(QIODevice::ReadOnly)) manifest = QJsonDocument::fromJson(file.readAll()).object();
    const bool valid = manifest.value("schema_version").toInt(manifest.value("schema").toInt()) == 1;
    if (!valid) status_ = "Showroom media is unavailable. Choose an appearance to continue.";
    const auto entries = manifest.value("cars").toArray();
    for (auto& car : cars_) {
        QJsonObject entry;
        for (const auto& candidate : entries)
            if (candidate.toObject().value("id").toString() == car.id) entry = candidate.toObject();
        const auto source = [&](const QString& role, const QString& extension) {
            const QString relative = entry.value(role).toString(car.id + "/" + role + extension);
            // A local media pack is data, never code or a remote URL. Refuse paths outside that pack.
            const QString path = QDir::cleanPath(root.filePath(relative));
            if (!valid || QDir::isAbsolutePath(relative) || relative.contains(':') ||
                !path.startsWith(root.absolutePath() + '/', Qt::CaseInsensitive) || !QFileInfo(path).isFile()) return QUrl{};
            return QUrl::fromLocalFile(path);
        };
        car.poster = source("poster", ".png");
        car.enter = source("enter", ".mp4");
        car.idle = source("idle", ".mp4");
        car.exit = source("exit", ".mp4");
        car.select = source("select", ".mp4");
        // The pack's logos/{id}.svg or .png, independent of the manifest; none leaves the title text.
        for (const QString extension : {".svg", ".png"})
            if (car.logo.isEmpty() && QFileInfo(root.filePath("logos/" + car.id + extension)).isFile())
                car.logo = QUrl::fromLocalFile(root.filePath("logos/" + car.id + extension));
    }
    watchdog_.setSingleShot(true);
    connect(&watchdog_, &QTimer::timeout, this, [this] { failed(serial_, "Playback timed out"); });
}

QVariantList Showroom::cars() const {
    QVariantList result;
    for (const auto& car : cars_) result.append(QVariantMap{{"id", car.id}, {"name", car.name}, {"subtitle", car.subtitle}, {"logo", car.logo}});
    return result;
}
QString Showroom::currentName() const { return cars_[current_].name + " " + cars_[current_].subtitle.section(" · ", 0, 0); }
QUrl Showroom::poster() const {
    // Enter begins at the empty black hub. Every other clip begins at this car's canonical hero pose.
    return phase_ == "enter" || phase_ == "off" ? QUrl{} : cars_[current_].poster;
}
void Showroom::open() {
    if (screen_ == "showroom") return;
    screen_ = "showroom";
    requested_ = current_;
    confirming_ = false;
    play("enter");
}
void Showroom::request(int index) {
    if (screen_ != "showroom" || index < 0 || index >= static_cast<int>(cars_.size()) || confirming_) return;
    requested_ = index;
    emit changed();
    if (phase_ == "idle" && clip_.isEmpty() && current_ != requested_) play("exit");
}
void Showroom::confirm() {
    if (screen_ != "showroom" || confirming_) return;
    confirming_ = true;
    emit changed();
    if (phase_ == "idle" && clip_.isEmpty()) settle();
}
void Showroom::settle() {
    if (current_ != requested_) play("exit");
    else if (confirming_) play("select");
    else play("idle");
}
void Showroom::play(const QString& phase) {
    watchdog_.stop();
    phase_ = phase;
    const auto& car = cars_[current_];
    clip_ = phase == "enter" ? car.enter : phase == "exit" ? car.exit : phase == "select" ? car.select : car.idle;
    ++serial_;
    emit changed();
    emit playbackChanged();
    const int token = serial_;
    if (clip_.isEmpty()) {
        // A poster is an honest usable fallback, never a claim that missing animation played.
        if (phase != "idle") QTimer::singleShot(0, this, [this, token] { failed(token, "Animation is unavailable"); });
    } else watchdog_.start(15000);
}
void Showroom::started(int token) {
    if (token != serial_ || screen_ != "showroom") return;
    watchdog_.start(30000);
    emit playbackStarted(token);
}
void Showroom::complete(int token) {
    if (token != serial_ || screen_ != "showroom") return;
    watchdog_.stop();
    if (phase_ == "idle") { settle(); return; }
    advance();
}
void Showroom::failed(int token, const QString& error) {
    if (token != serial_ || screen_ != "showroom") return;
    watchdog_.stop();
    status_ = error + (cars_[current_].poster.isEmpty()
        ? ". Choose an appearance to continue without media."
        : ". The still image remains available; selection is ready.");
    if (phase_ == "idle") {
        clip_ = {}; ++serial_; emit changed(); emit playbackChanged();
        if (current_ != requested_ || confirming_) settle();
    }
    else advance();
}
void Showroom::advance() {
    if (phase_ == "enter") settle();
    else if (phase_ == "exit") { current_ = requested_; play("enter"); }
    else if (phase_ == "select") {
        screen_ = "setup";
        phase_ = "off";
        confirming_ = false;
        clip_ = {};
        ++serial_;
        emit selected(current_);
        emit changed();
        emit playbackChanged();
    }
}
void Showroom::bypass() {
    watchdog_.stop();
    screen_ = "driving";
    phase_ = "off";
    confirming_ = false;
    clip_ = {};
    ++serial_;
    emit changed();
    emit playbackChanged();
}
void Showroom::startDriving() { if (screen_ == "setup") { screen_ = "driving"; emit changed(); } }
