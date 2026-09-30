#include "live_cars.hpp"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>
#include <cmath>

namespace {
// The showroom's appearance order, and each car's wheels in the simulation's order.
const char* const ids[] = {"amr23", "jesko", "urus", "rb19"};
const char* const corners[] = {"fl", "fr", "rl", "rr"};

// A file named relative to the directory, inside it, that exists; an empty URL otherwise. Local assets are data, never
// code from elsewhere: absolute paths, drives, schemes and escapes are refused.
QUrl inside(const QDir& root, const QJsonValue& value) {
    const QString relative = value.toString();
    if (relative.isEmpty() || QDir::isAbsolutePath(relative) || relative.contains(':')) return {};
    const QString path = QDir::cleanPath(root.filePath(relative));
    if (!path.startsWith(root.absolutePath() + '/', Qt::CaseInsensitive) || !QFileInfo(path).isFile()) return {};
    return QUrl::fromLocalFile(path);
}

// Finite numbers only, count of them exactly as asked.
bool numbers(const QJsonValue& value, int count, QVariantList& out) {
    const QJsonArray array = value.toArray();
    if (array.size() != count) return false;
    out.clear();
    for (const auto& item : array) {
        if (!item.isDouble() || !std::isfinite(item.toDouble())) return false;
        out.append(item.toDouble());
    }
    return true;
}

// One appearance's car, or the reason it has none.
QVariantMap car(const QDir& root, const QString& id, const QJsonObject& entry) {
    const auto refuse = [&](const QString& reason) { return QVariantMap{{"available", false}, {"id", id}, {"reason", reason}}; };
    if (entry.isEmpty()) return refuse("not built");
    const QUrl body = inside(root, entry.value("body"));
    if (body.isEmpty()) return refuse("its body is missing or outside the live car directory");
    const QJsonArray entries = entry.value("wheels").toArray();
    if (entries.size() != 4) return refuse("it does not have four wheels");
    QVariantList wheels;
    for (int i = 0; i < 4; ++i) {
        const QJsonObject wheel = entries.at(i).toObject();
        if (wheel.value("name").toString() != QLatin1String(corners[i])) return refuse("its wheels are not in the order FL, FR, RL, RR");
        const QUrl component = inside(root, wheel.value("component"));
        if (component.isEmpty()) return refuse(QString("its %1 wheel is missing or outside the live car directory").arg(corners[i]));
        QVariantList position;
        if (!numbers(wheel.value("position"), 3, position)) return refuse(QString("its %1 wheel has no position").arg(corners[i]));
        const double radius = wheel.value("radius").toDouble(-1);
        if (!std::isfinite(radius) || radius < 0.1 || radius > 1.0) return refuse(QString("its %1 wheel's radius is not a wheel's").arg(corners[i]));
        const double width = wheel.value("width").toDouble(-1);
        if (!std::isfinite(width) || width < 0.05 || width > 1.0) return refuse(QString("its %1 wheel's width is not a wheel's").arg(corners[i]));
        wheels.append(QVariantMap{{"name", corners[i]}, {"component", component}, {"position", position}, {"radius", radius},
                                  {"width", width}, {"front", i < 2}, {"left", i % 2 == 0}});
    }
    const QJsonObject shadow = entry.value("shadow").toObject();
    const QUrl shadowSource = inside(root, shadow.value("file"));
    QVariantList x, z;
    if (shadowSource.isEmpty() || !numbers(shadow.value("x"), 2, x) || !numbers(shadow.value("z"), 2, z) ||
        x[0].toDouble() >= x[1].toDouble() || z[0].toDouble() >= z[1].toDouble())
        return refuse("its contact shadow is missing or malformed");
    const double wheelbase = entry.value("wheelbase_m").toDouble(-1);
    if (!std::isfinite(wheelbase) || wheelbase < 0.5 || wheelbase > 6.0) return refuse("its wheelbase is not a car's");
    return QVariantMap{{"available", true}, {"id", id}, {"body", body}, {"wheels", wheels},
                       {"shadow", QVariantMap{{"source", shadowSource}, {"x", x}, {"z", z}}}, {"wheelbase", wheelbase}};
}
}

LiveCars::LiveCars(const QString& directory, QObject* parent) : QObject(parent) {
    const QDir root(QFileInfo(directory).absoluteFilePath());
    QFile file(root.filePath("index.json"));
    QJsonObject index;
    if (file.open(QIODevice::ReadOnly)) index = QJsonDocument::fromJson(file.readAll()).object();
    const bool valid = index.value("schema_version").toInt() == 1;
    const QJsonObject entries = valid ? index.value("cars").toObject() : QJsonObject{};
    QStringList missing;
    for (const char* id : ids) {
        const QVariantMap entry = car(root, id, entries.value(id).toObject());
        cars_.append(entry);
        if (!entry.value("available").toBool()) missing.append(QString("%1 (%2)").arg(id, entry.value("reason").toString()));
    }
    if (valid) environment_ = inside(root, index.value("environment"));
    if (!file.exists()) status_ = "No live cars built; run tools/showroom/build_live_cars.py. The procedural cars drive.";
    else if (!valid) status_ = "The live car index is not schema 1; the procedural cars drive.";
    else if (!missing.isEmpty()) status_ = "Procedural car for " + missing.join(", ");
}

int LiveCars::available() const {
    int count = 0;
    for (const auto& entry : cars_) count += entry.toMap().value("available").toBool() ? 1 : 0;
    return count;
}
