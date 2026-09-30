#include "bridge.hpp"
#include "showroom.hpp"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QQuickItem>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <QSet>
#include <memory>
#include <functional>
#include <stdexcept>

// Opt-in integration exercise using the actual published Blender pack and actual QML decoder callbacks.
// The sequence covers every directed pair once. No synthetic EndOfMedia is sent here.
void scheduleShowroomVerification(QGuiApplication& app, QQmlApplicationEngine& engine, Bridge& bridge,
                                  Showroom& showroom, const QString& output, QStringList& warnings, bool& finished) {
    struct Context {
        QElapsedTimer elapsed;
        int checks{}, edge{}, selection{}, selectionsDone{};
        bool selecting{}, requested{};
        QSet<int> started;
        QSet<QString> decoded;
        QJsonArray transitions;
    };
    auto context = std::make_shared<Context>();
    context->elapsed.start();
    QDir().mkpath(output);
    auto* window = qobject_cast<QQuickWindow*>(engine.rootObjects().first());
    auto* timer = new QTimer(&app);
    QObject::connect(&showroom, &Showroom::playbackStarted, &app, [context, &showroom](int serial) {
        context->started.insert(serial);
        context->decoded.insert(QString::number(showroom.current()) + ':' + showroom.phase());
    });
    QObject::connect(timer, &QTimer::timeout, &app, [=, &app, &bridge, &showroom, &warnings, &finished] {
        const auto finish = [&](const QString& error) {
            finished = true;
            QJsonArray warningJson; for (const auto& warning : warnings) warningJson.append(warning);
            QJsonArray decodedJson; for (const auto& clip : context->decoded) decodedJson.append(clip);
            QFile evidence(output + "/verification.json");
            if(evidence.open(QIODevice::WriteOnly)) evidence.write(QJsonDocument(QJsonObject{
                {"passed", error.isEmpty()}, {"checks", context->checks}, {"failure", error}, {"qml_warnings", warningJson},
                {"elapsed_s", context->elapsed.elapsed()/1000.0}, {"transitions", context->transitions}, {"decoded_clips", decodedJson},
                {"method", "Local MP4 files through Qt Multimedia; actual QML buttons and EndOfMedia; no synthetic completion"}
            }).toJson());
            QTextStream(error.isEmpty() ? stdout : stderr) << "Showroom playback: " << (error.isEmpty() ? "passed" : error) << '\n';
            timer->stop(); app.exit(error.isEmpty() ? 0 : 8);
        };
        const auto check = [&](bool okay, const char* error) { ++context->checks; if(!okay) throw std::runtime_error(error); };
        const auto click = [&](const QString& name) {
            const std::function<QQuickItem*(QQuickItem*)> search = [&](QQuickItem* item) -> QQuickItem* {
                if(item->objectName() == name) return item;
                for(auto* child : item->childItems()) if(auto* found = search(child)) return found;
                return nullptr;
            };
            auto* control = search(window->contentItem());
            check(control && control->isVisible() && control->isEnabled(), "A showroom button was not usable");
            check(QMetaObject::invokeMethod(control, "clicked"), "A showroom button has no click handler");
        };
        // The next arrow walks AMR23, RB19, Jesko, Urus; rapid presses queue only the last car reached.
        const auto browse = [&](int target) {
            for(int press = 0; press < 3 && showroom.requested() != target; ++press) click("showroomNext");
            check(showroom.requested() == target, "The arrows did not reach the requested car");
        };
        try {
            check(context->elapsed.elapsed() < 300000, "Actual media playback did not complete within 300 seconds");
            check(showroom.status().isEmpty(), "The rendered media pack is incomplete or a clip failed decoding");
            check(!bridge.running() && bridge.simulationTime() == 0, "Showroom playback advanced the simulation plant");
            const QVariantMap car = showroom.cars()[showroom.current()].toMap();
            auto* name = window->findChild<QObject*>("showroomHeroName");
            check(name && name->property("text").toString() == car.value("name").toString(),
                  "The title does not name the car currently on screen");
            if(showroom.screen() == "showroom" && !car.value("logo").toUrl().isEmpty()) {
                // While the showroom is on screen: Image.Ready is 1, so the logo shown is the current car's and it decoded.
                auto* logo = window->findChild<QObject*>("showroomHeroLogo");
                check(logo && logo->property("visible").toBool() && logo->property("source").toUrl() == car.value("logo").toUrl()
                      && logo->property("status").toInt() == 1, "The logo does not show the car currently on screen");
                // Between the arrows, the car the arrows have reached, which may be a car still to come.
                auto* browsed = window->findChild<QObject*>("showroomBrowseLogo");
                check(browsed && browsed->property("source").toUrl() == showroom.cars()[showroom.requested()].toMap().value("logo").toUrl(),
                      "The logo between the arrows is not the car the arrows reached");
            }
            if(showroom.screen() == "setup") {
                auto* curtain = window->findChild<QObject*>("showroomHandoffCurtain");
                if(curtain && curtain->property("opacity").toDouble() > 0) return;
                check(context->selecting && bridge.carAppearance() == context->selection, "Handoff committed the wrong appearance");
                check(window->grabWindow().save(output + "/track-setup-" + QString::number(context->selection) + ".png"), "Track setup capture failed");
                ++context->selectionsDone;
                if(context->selectionsDone == 4) {
                    check(context->transitions.size() == 12, "Not every directed switch played");
                    check(context->decoded.size() == 16, "Not every enter, idle, exit and select clip produced a video frame");
                    check(warnings.isEmpty(), "QML emitted warnings during media playback");
                    finish({}); return;
                }
                ++context->selection;
                context->requested = false;
                click("showroomButton");
                return;
            }
            if(showroom.phase() != "idle" || !context->started.contains(showroom.serial())) return;
            static constexpr int route[]{0, 1, 0, 2, 0, 3, 1, 2, 1, 3, 2, 3, 0};
            if(context->edge < 12) {
                if(context->requested) {
                    if(showroom.current() != route[context->edge+1]) return;
                    context->transitions.append(QJsonObject{{"from", route[context->edge]}, {"to", route[context->edge+1]}});
                    ++context->edge;
                    context->requested = false;
                }
                if(context->edge < 12) {
                    check(showroom.current() == route[context->edge], "A directed transition ended at the wrong car");
                    check(window->grabWindow().save(output + "/car-" + QString::number(showroom.current()) + ".png"), "Car capture failed");
                    browse(route[context->edge+1]);
                    context->requested = true;
                    return;
                }
            }
            if(showroom.current() != context->selection) {
                if(!context->requested) { browse(context->selection); context->requested = true; }
                return;
            }
            context->selecting = true;
            if(!showroom.confirming()) click("showroomContinue");
        } catch(const std::exception& error) { finish(QString::fromUtf8(error.what())); }
    });
    timer->start(60);
}
