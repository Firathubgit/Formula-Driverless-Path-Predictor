#pragma once
#include <QObject>
#include <QString>
#include <QUrl>
#include <QVariantList>

// Presentation only: the live scene's cars, exported from the showroom's prepared cars by
// tools/showroom/build_live_cars.py. For each appearance, in the showroom's order (0 AMR23, 1 Jesko, 2 Urus, 3 RB19), a
// body and four wheels as balsam components, where each wheel sits and turns in the car's frame (origin at the rear
// axle's centre on the ground, right along +X, forward along -Z, metres), and a contact shadow; and the studio lighting.
// Read once from a local directory. A part that is missing, malformed or outside that directory leaves its appearance
// to the procedural silhouette. Nothing here touches the plant: the chosen appearance never selects a physics model.
class LiveCars : public QObject {
    Q_OBJECT
    Q_PROPERTY(QVariantList cars READ cars CONSTANT)
    Q_PROPERTY(QUrl environment READ environment CONSTANT)
    Q_PROPERTY(int available READ available CONSTANT)
    Q_PROPERTY(QString status READ status CONSTANT)
public:
    explicit LiveCars(const QString& directory, QObject* parent = nullptr);
    // One map per appearance: {available, id, body, wheels: [{name, component, position, radius, front, left}],
    // shadow: {source, x, z}, wheelbase}; {available: false, id, reason} where the appearance has no usable car.
    QVariantList cars() const { return cars_; }
    QUrl environment() const { return environment_; }
    int available() const;
    QString status() const { return status_; }
private:
    QVariantList cars_;
    QUrl environment_;
    QString status_;
};
