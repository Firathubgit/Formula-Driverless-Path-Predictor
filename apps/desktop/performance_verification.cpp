#include "bridge.hpp"
#include "performance_probe.hpp"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QTimer>
#include <QElapsedTimer>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QDir>
#include <QImage>
#include <algorithm>
#include <memory>
#include <vector>

void scheduleKartBenchmark(QGuiApplication& app, QQmlApplicationEngine& engine, Bridge& bridge, const QString& directory) {
    bridge.setGamepadEnabled(false);
    struct Run {
        QElapsedTimer wall, frame;
        std::vector<double> intervals;
        QJsonObject result;
        QJsonArray states;
    };
    auto run=std::make_shared<Run>();
    QTimer::singleShot(2500,&app,[&,run,directory] {
        auto* window=qobject_cast<QQuickWindow*>(engine.rootObjects().first());
        if (!window) { app.exit(2); return; }
        QDir().mkpath(directory);
        run->result["preferred_adapter"]=window->property("preferredGraphicsAdapter").toString();
        window->grabWindow().save(directory+"/scene.png");
        desktop_performance={true};
        // Exactly the same inputs and fixed ticks before/after optimization, including each sampled true state.
        run->wall.start();
        for (int frame=0;frame<240;++frame) {
            bridge.setDriverControls(0.55,0,frame<120 ? 0.16 : -0.12);
            bridge.advanceForCapture(0.015);
            const auto& s=bridge.simulation().state();
            run->states.append(QJsonArray{s.x_m,s.y_m,s.yaw_rad,s.speed_mps,s.steering_rad,s.time_s});
        }
        run->result["fixed_work_wall_ms"]=run->wall.nsecsElapsed()/1e6;
        run->result["advance_ms"]=desktop_performance.advance_ms;
        run->result["publish_ms"]=desktop_performance.publish_ms;
        run->result["geometry_ms"]=desktop_performance.geometry_ms;
        run->result["geometry_rebuilds"]=desktop_performance.geometries;
        run->result["states"]=run->states;
        desktop_performance.enabled=false;
        bridge.reset();
        bridge.setDriverControls(0.55,0,0.16);
        run->wall.restart();
        run->frame.start();
        QObject::connect(window,&QQuickWindow::frameSwapped,&app,[run] {
            run->intervals.push_back(run->frame.nsecsElapsed()/1e6);
            run->frame.restart();
        },Qt::QueuedConnection);
        bridge.toggleRunning();
        QTimer::singleShot(10000,&app,[&,run,directory,window] {
            if (bridge.running()) bridge.toggleRunning();
            const double elapsed=run->wall.nsecsElapsed()/1e9;
            std::sort(run->intervals.begin(),run->intervals.end());
            run->result["frames"]=static_cast<int>(run->intervals.size());
            run->result["wall_seconds"]=elapsed;
            run->result["simulation_seconds"]=bridge.simulationTime();
            run->result["fps"]=run->intervals.size()/elapsed;
            if (!run->intervals.empty()) {
                run->result["frame_p50_ms"]=run->intervals[run->intervals.size()/2];
                run->result["frame_p95_ms"]=run->intervals[run->intervals.size()*95/100];
                run->result["frame_max_ms"]=run->intervals.back();
            }
            run->result["width"]=window->width();
            run->result["height"]=window->height();
            if (auto* view=window->findChild<QObject*>("drivingView")) {
                if (auto* stats=view->property("renderStats").value<QObject*>()) {
                    QJsonObject render;
                    for (const char* key : {"graphicsApiName","frameTime","renderTime","renderPrepareTime","syncTime","lastCompletedGpuTime"})
                        render[key]=QJsonValue::fromVariant(stats->property(key));
                    run->result["render_stats"]=render;
                }
            }
            QFile out(directory+"/benchmark.json");
            if (!out.open(QIODevice::WriteOnly)) { app.exit(2); return; }
            out.write(QJsonDocument(run->result).toJson());
            app.exit(0);
        });
    });
}
