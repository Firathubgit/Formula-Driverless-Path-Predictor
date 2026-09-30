#include "showroom.hpp"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>
#include <stdexcept>

namespace {
int checks = 0;
void check(bool value, const char* message) { ++checks; if (!value) throw std::runtime_error(message); }
void write(const QString& path, const QByteArray& data) {
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(data) != data.size()) throw std::runtime_error("fixture could not be written");
}
void drain() { for (int i = 0; i < 12; ++i) QCoreApplication::processEvents(); }
void finish(Showroom& s) { s.complete(s.serial()); }
}
int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    try {
        QTemporaryDir dir;
        check(dir.isValid(), "fixture directory exists");
        QJsonArray cars;
        for (const QString& id : {QString("amr23"), QString("jesko"), QString("urus"), QString("rb19")}) {
            QDir().mkpath(dir.path() + '/' + id);
            QJsonObject car{{"id", id}};
            for (const QString& role : {QString("enter"), QString("idle"), QString("exit"), QString("select"), QString("poster")}) {
                const QString path = id + '/' + role + (role == "poster" ? ".png" : ".mp4");
                write(dir.path() + '/' + path, "decoder-independent state fixture");
                car.insert(role, path);
            }
            cars.append(car);
        }
        write(dir.path() + "/manifest.json", QJsonDocument(QJsonObject{{"schema_version", 1}, {"cars", cars}}).toJson());
        // Each directed pair has exactly one exit and one enter. No selected signal before confirmation.
        for (int a = 0; a < 4; ++a) for (int b = 0; b < 4; ++b) if (a != b) {
            Showroom s(dir.path());
            int selected = -1;
            QObject::connect(&s, &Showroom::selected, [&](int value) { selected = value; });
            s.open(); finish(s);
            if (a) { s.request(a); finish(s); finish(s); finish(s); }
            check(s.phase() == "idle" && s.current() == a, "source reaches its hero endpoint");
            s.request(b);
            check(s.phase() == "idle" && s.requested() == b, "choice waits for the moving idle to reach its canonical endpoint");
            finish(s);
            check(s.phase() == "exit" && s.current() == a && s.clip().path().endsWith("/exit.mp4"), "edge begins with source exit");
            const int exitSerial = s.serial();
            finish(s);
            check(s.phase() == "enter" && s.current() == b && s.poster().isEmpty(), "edge enters destination from empty black hub");
            s.complete(exitSerial); s.failed(exitSerial, "late decoder error");
            check(s.phase() == "enter" && s.status().isEmpty(), "stale exit completion and error cannot skip the destination");
            finish(s);
            check(s.phase() == "idle" && s.current() == b && !s.poster().isEmpty() && selected == -1, "destination waits at hero; preview never commits appearance");
            s.confirm();
            check(s.phase() == "idle", "confirmation also waits for the idle endpoint");
            finish(s);
            check(s.phase() == "select" && selected == -1, "selection animation precedes handoff");
            s.request(a); s.confirm();
            check(s.requested() == b && s.phase() == "select", "confirmed destination is stable through repeated clicks");
            finish(s);
            check(s.screen() == "setup" && selected == b, "only completed selection enters track setup");
            s.startDriving(); check(s.screen() == "driving", "Start leaves setup explicitly");
        }
        {
            Showroom s(dir.path()); s.open(); finish(s);
            s.request(1); s.request(2); s.request(3);
            check(s.current() == 0 && s.requested() == 3, "last click replaces queued destination while source exits");
            finish(s);
            finish(s); check(s.current() == 3, "only last queued car enters");
            s.request(2); s.request(1); s.confirm(); finish(s);
            check(s.phase() == "exit" && s.current() == 3, "new choice during enter waits for endpoint before leaving");
            finish(s); check(s.phase() == "enter" && s.current() == 1, "selection follows newest pre-confirmation request");
            finish(s); check(s.phase() == "select", "queued confirmation waits for target hero");
            finish(s); check(s.screen() == "setup" && s.current() == 1, "queued confirmation finishes with correct car");
        }
        {
            Showroom s(dir.path()); s.open(); finish(s);
            const int idle = s.serial(); s.request(0); s.request(-1); s.request(4);
            check(s.serial() == idle && s.phase() == "idle", "same car and invalid requests do not restart motion");
            s.request(1); finish(s); const int pending = s.serial(); s.bypass();
            s.complete(pending); s.failed(pending, "late callback"); s.confirm(); s.request(2);
            check(s.screen() == "driving" && s.phase() == "off", "replay bypass cancels all queued and late presentation actions");
            s.open(); finish(s); s.failed(s.serial(), "Decoder unavailable");
            check(s.phase() == "idle" && s.clip().isEmpty() && !s.poster().isEmpty(), "decoder failure keeps the usable hero poster");
            s.confirm(); s.failed(s.serial(), "Decoder unavailable");
            check(s.screen() == "setup", "decode failure cannot trap the user at selection");
        }
        {
            Showroom s(dir.path() + "/missing"); s.open(); drain();
            check(s.phase() == "idle" && s.clip().isEmpty() && !s.status().isEmpty(), "missing media uses an honest nonblocking fallback");
            s.request(2); s.confirm(); drain();
            check(s.screen() == "setup" && s.current() == 2, "missing media still honors queued target and confirmation");
        }
        {
            auto car = cars[0].toObject(); car.insert("enter", "../outside.mp4"); cars[0] = car;
            write(dir.path() + "/manifest.json", QJsonDocument(QJsonObject{{"schema_version", 1}, {"cars", cars}}).toJson());
            Showroom s(dir.path()); s.open();
            check(s.clip().isEmpty(), "manifest cannot load media outside its local pack");
        }
        QTextStream(stdout) << "Showroom: " << checks << " checks passed (12 directed pairs, queues, confirmation, stale callbacks, failures and replay bypass)\n";
        return 0;
    } catch (const std::exception& error) { QTextStream(stderr) << "Showroom after " << checks << " checks: " << error.what() << '\n'; return 1; }
}
