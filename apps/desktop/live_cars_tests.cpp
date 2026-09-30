#include "live_cars.hpp"
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTextStream>
#include <functional>
#include <stdexcept>

namespace {
int checks = 0;
void check(bool value, const char* message) { ++checks; if (!value) throw std::runtime_error(message); }
void write(const QString& path, const QByteArray& data) {
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(data) != data.size()) throw std::runtime_error("fixture could not be written");
}
// A built RB19, as build_live_cars.py writes one: its files, and its index entry, which a case may spoil.
QJsonObject rb19(const QString& root) {
    write(root + "/rb19/qt/body/Part.qml", "Node {}");
    write(root + "/rb19/shadow.png", "png");
    QJsonArray wheels;
    const char* const names[] = {"fl", "fr", "rl", "rr"};
    for (int i = 0; i < 4; ++i) {
        const QString component = QString("rb19/qt/wheel_%1/Part.qml").arg(names[i]);
        write(root + '/' + component, "Node {}");
        wheels.append(QJsonObject{{"name", names[i]}, {"component", component}, {"radius", 0.366}, {"width", 0.42},
                                  {"position", QJsonArray{i % 2 ? 0.84 : -0.84, 0.366, i < 2 ? -3.596 : 0.0}}});
    }
    return QJsonObject{{"body", "rb19/qt/body/Part.qml"}, {"wheels", wheels}, {"wheelbase_m", 3.596},
                       {"shadow", QJsonObject{{"file", "rb19/shadow.png"}, {"x", QJsonArray{-1.4, 1.4}}, {"z", QJsonArray{-5.3, 1.0}}}}};
}
void index(const QString& root, const QJsonObject& rb19Entry, int schema = 1) {
    write(root + "/studio.hdr", "hdr");
    write(root + "/index.json", QJsonDocument(QJsonObject{{"schema_version", schema}, {"environment", "studio.hdr"},
                                                          {"cars", QJsonObject{{"rb19", rb19Entry}}}}).toJson());
}
QVariantMap entry(const LiveCars& cars, int appearance) { return cars.cars().at(appearance).toMap(); }
// Builds an RB19 in a live car directory one level inside a temporary one, so a fixture that escapes it stays inside the
// temporary directory; spoils its entry, and returns whether it is still offered.
bool offered(const std::function<void(QJsonObject&, const QString&)>& spoil) {
    QTemporaryDir dir;
    const QString root = dir.path() + "/live-cars";
    QJsonObject car = rb19(root);
    spoil(car, root);
    index(root, car);
    return entry(LiveCars(root), 3).value("available").toBool();
}
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    try {
        {
            // Nothing built: every appearance keeps its procedural car, and the status says how to build them.
            QTemporaryDir dir;
            LiveCars cars(dir.path() + "/not-built");
            check(cars.cars().size() == 4 && cars.available() == 0 && cars.environment().isEmpty(), "no live cars without an index");
            check(cars.status().contains("build_live_cars.py"), "the status names the build script");
        }
        {
            // A built RB19: offered at its appearance, 3, with its parts inside the directory and its wheels in order.
            QTemporaryDir dir;
            index(dir.path(), rb19(dir.path()));
            LiveCars cars(dir.path());
            const auto rb = entry(cars, 3);
            check(cars.available() == 1 && rb.value("available").toBool() && rb.value("id").toString() == "rb19", "the built car is offered at its appearance");
            check(!entry(cars, 0).value("available").toBool() && entry(cars, 0).value("reason").toString() == "not built", "the others keep their procedural cars");
            const QString prefix = QDir(dir.path()).absolutePath() + '/';
            check(rb.value("body").toUrl().isLocalFile() && rb.value("body").toUrl().toLocalFile().startsWith(prefix, Qt::CaseInsensitive), "the body is a local file inside the directory");
            const auto wheels = rb.value("wheels").toList();
            check(wheels.size() == 4, "four wheels");
            const auto fl = wheels[0].toMap(), rr = wheels[3].toMap();
            check(fl.value("name").toString() == "fl" && fl.value("front").toBool() && fl.value("left").toBool() &&
                  !rr.value("front").toBool() && !rr.value("left").toBool(), "the wheels come front left to rear right");
            check(fl.value("position").toList().size() == 3 && fl.value("position").toList()[2].toDouble() == -3.596 &&
                  fl.value("radius").toDouble() == 0.366 && fl.value("width").toDouble() == 0.42, "each wheel keeps where it sits, its radius and its width");
            const auto shadow = rb.value("shadow").toMap();
            check(shadow.value("source").toUrl().isLocalFile() && shadow.value("x").toList()[0].toDouble() == -1.4 &&
                  shadow.value("z").toList()[1].toDouble() == 1.0, "the contact shadow and its extent");
            check(cars.environment().isLocalFile() && cars.environment().toLocalFile().endsWith("studio.hdr"), "the studio lighting is offered");
            check(rb.value("wheelbase").toDouble() == 3.596, "the wheelbase of the model");
        }
        // Anything that is not a whole car inside the directory leaves the appearance to its procedural car.
        check(!offered([](QJsonObject& car, const QString& root) {
            write(root + "/../outside-body.qml", "Node {}");
            car["body"] = "../outside-body.qml";
        }), "a body outside the directory is refused");
        check(!offered([](QJsonObject& car, const QString& root) { car["body"] = QDir(root).absoluteFilePath("rb19/qt/body/Part.qml"); }),
              "an absolute body path is refused");
        check(!offered([](QJsonObject&, const QString& root) { QFile::remove(root + "/rb19/qt/wheel_rl/Part.qml"); }), "a missing wheel is refused");
        check(!offered([](QJsonObject& car, const QString&) {
            auto wheels = car["wheels"].toArray();
            const auto first = wheels[0];
            wheels[0] = wheels[1];
            wheels[1] = first;
            car["wheels"] = wheels;
        }), "wheels out of order are refused");
        check(!offered([](QJsonObject& car, const QString&) {
            auto wheels = car["wheels"].toArray();
            auto wheel = wheels[2].toObject();
            wheel["radius"] = 7.0;
            wheels[2] = wheel;
            car["wheels"] = wheels;
        }), "a radius no wheel has is refused");
        check(!offered([](QJsonObject& car, const QString&) {
            auto wheels = car["wheels"].toArray();
            auto wheel = wheels[1].toObject();
            wheel.remove("width");
            wheels[1] = wheel;
            car["wheels"] = wheels;
        }), "a wheel without a width is refused");
        check(!offered([](QJsonObject& car, const QString&) {
            auto wheels = car["wheels"].toArray();
            wheels.removeLast();
            car["wheels"] = wheels;
        }), "three wheels are refused");
        check(!offered([](QJsonObject& car, const QString&) {
            auto shadow = car["shadow"].toObject();
            shadow["x"] = QJsonArray{1.4, -1.4};
            car["shadow"] = shadow;
        }), "an inverted shadow extent is refused");
        check(!offered([](QJsonObject& car, const QString&) { car["wheelbase_m"] = 0.0; }), "a wheelbase no car has is refused");
        {
            // Another schema is not read at all.
            QTemporaryDir dir;
            index(dir.path(), rb19(dir.path()), 2);
            LiveCars cars(dir.path());
            check(cars.available() == 0 && cars.environment().isEmpty() && cars.status().contains("schema"), "another schema is refused whole");
        }
    } catch (const std::exception& error) {
        QTextStream(stderr) << "live cars: " << error.what() << " after " << checks << " checks\n";
        return 1;
    }
    QTextStream(stdout) << "live cars: " << checks << " checks passed\n";
    return 0;
}
