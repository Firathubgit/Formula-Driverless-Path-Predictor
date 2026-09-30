#ifdef FD_HAVE_MPCC
#include "fd/mpcc.hpp"
#endif
#include "bridge.hpp"
#include "race_audio.hpp"
#include "showroom.hpp"
#include "fd/lattice.hpp"
#include "fd/performance_envelope.hpp"
#include "fd/track_conditioning.hpp"
#include "track_geometry.hpp"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickWindow>
#include <QQuickItem>
#include <QImage>
#include <QFile>
#include <QDir>
#include <QTemporaryDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QColor>
#include <QVector3D>
#include <algorithm>
#include <memory>
#include <limits>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <sstream>
#include <numbers>
#include <variant>
#include <stdexcept>

namespace {
// A short recorded run with one grip change and a lane blockage the car steers around,
// produced by the same writer as the headless runner.
QString writeReplayFixture(const QString& output) {
    const QString path = output+"/replay-fixture";
    QDir(path).removeRecursively();
    fd::Simulation simulation;
    simulation.set_obstructions({{62, 68, -4.5, 1.0, "cone-cluster"}});
    fd::RunRequest request{0, 12.0, {{6.0, 0.45}}};
    fd::RecordingWriter writer(std::filesystem::path(path.toStdU16String()), simulation, request, {"ui-verification", "", ""});
    bool applied = false;
    while (simulation.state().time_s+simulation.config().fixed_dt_s <= request.duration_cap_s+1e-9) {
        if (!applied && simulation.state().time_s+1e-10 >= 6.0) { simulation.set_grip(0.45); applied = true; }
        simulation.step();
        writer.record(simulation);
    }
    writer.finish(simulation);
    return path;
}
// A recorded run on cones (decision 0031): the preset laid to 5 m, the car with its instruments, perceiving the cones and
// driving on the path it believes from them, five seconds from the start.
QString writeConesFixture(const QString& output) {
    const QString path = output+"/replay-fixture-cones";
    QDir(path).removeRecursively();
    auto track = fd::make_preset_track();
    track.width_m = 5;
    fd::Simulation simulation(track, fd::Config{});
    simulation.set_sensors(fd::SensorSettings{});
    simulation.set_perception(fd::PerceptionSettings{});
    simulation.set_cone_driving(fd::ConePathSettings{});
    fd::RunRequest request{0, 5.0, {}};
    fd::RecordingWriter writer(std::filesystem::path(path.toStdU16String()), simulation, request, {"ui-verification", "", ""});
    while (simulation.state().time_s+simulation.config().fixed_dt_s <= request.duration_cap_s+1e-9) {
        simulation.step();
        writer.record(simulation);
    }
    writer.finish(simulation);
    return path;
}
// A recorded run on a course with three cones left across the lane 60 m on, which the car runs into (decision 0032).
QString writeKnockedFixture(const QString& output) {
    const QString path = output+"/replay-fixture-knocked";
    QDir(path).removeRecursively();
    fd::Simulation simulation;
    const auto& track = simulation.track();
    auto cones = fd::make_cone_layout(track);
    const auto at = fd::sample(track, 60), ahead = fd::sample(track, 60.05);
    const double tx = ahead.x_m-at.x_m, ty = ahead.y_m-at.y_m, n = std::hypot(tx, ty);
    for (const double offset : {-0.4, 0.0, 0.4}) cones.push_back({{at.x_m-ty/n*offset, at.y_m+tx/n*offset}, fd::ConeColour::orange});
    simulation.set_course(cones, fd::make_gates(track), "cones left in the lane");
    fd::RunRequest request{0, 10.0, {}};
    fd::RecordingWriter writer(std::filesystem::path(path.toStdU16String()), simulation, request, {"ui-verification", "", ""});
    while (simulation.state().time_s+simulation.config().fixed_dt_s <= request.duration_cap_s+1e-9) {
        simulation.step();
        writer.record(simulation);
    }
    writer.finish(simulation);
    return path;
}
// A reference-optimal artefact for the live car on the live corridor, as a solver would write one (decision 0034): the
// centre of gravity on the centreline at every eighth corridor sample at a steady 20 m/s, each tire dissipating 1 kW,
// with two sensitivities. The desktop is given it to load; fd_optimal_lap_tests holds the contract itself.
QString writeOptimalFixture(const QString& output, const Bridge& bridge) {
    const auto& simulation = bridge.simulation();
    const auto problem = fd::make_lap_problem(fd::condition_track(simulation.track()).track,
                                              std::get<fd::FourWheelCar>(simulation.vehicle_model()), simulation.config(), "formula-one-style");
    const QString directory = output+"/optimal-lap";
    QDir(directory).removeRecursively();
    fd::write_lap_problem(std::filesystem::path(directory.toStdU16String()), problem);
    const auto& points = problem.corridor.points;
    const double speed = 20, lap_time = problem.corridor.length_m/speed;
    std::ostringstream csv;
    csv << std::setprecision(17) << "point,s_m,time_s,cog_x_m,cog_y_m,yaw_rad,offset_m,forward_mps,lateral_mps,yaw_rate_radps,steering_rad,throttle";
    for (const char* w : {"fl", "fr", "rl", "rr"})
        csv << ",slip_ratio_" << w << ",slip_angle_" << w << "_rad,force_x_" << w << "_n,force_y_" << w << "_n,load_" << w << "_n,dissipation_" << w << "_w";
    csv << '\n';
    int count = 0;
    for (std::size_t i = 0; i < points.size(); i += 8, ++count) {
        const auto& p = points[i];
        const auto& n = problem.left_normals[i];
        csv << count << ',' << p.s_m << ',' << p.s_m/speed << ',' << p.x_m << ',' << p.y_m << ',' << std::atan2(-n.x, n.y) << ",0," << speed
            << ",0," << speed*p.curvature << ",0.01,0.1";
        for (int w = 0; w < 4; ++w) csv << ",0.01,0.02,100,200,1600,1000";
        csv << '\n';
    }
    std::ostringstream json;
    json << std::setprecision(17) << "{\n  \"schema_version\": 1,\n  \"kind\": \"reference-optimal\",\n"
         << "  \"label\": \"Offline optimum for a steady fixture lap: not a run of this project's plant\",\n"
         << "  \"solver\": {\"tool\": \"fastest-lap\", \"version\": \"0.5\", \"release\": \"fixture\", \"release_sha256\": \""
         << std::string(64, 'a') << "\", \"library_sha256\": \"" << std::string(64, 'b') << "\", \"model\": \"f1-3dof\", "
         << "\"nlp_solver\": \"Ipopt\", \"status\": \"Optimal Solution Found\", \"converged\": true, \"iterations\": 40, "
         << "\"max_iterations\": 3000, \"tolerance\": 1e-08},\n"
         << "  \"mesh_points\": " << count << ",\n  \"lap_time_s\": " << lap_time << ",\n  \"objective\": " << lap_time << ",\n"
         << "  \"speed_cap_mps\": " << problem.config.max_speed_mps << ",\n  \"vehicle_half_width_m\": 0.9,\n"
         << "  \"problem_fingerprint\": \"" << fd::lap_problem_fingerprint(problem) << "\",\n"
         << "  \"mapping\": [\"model: a steady fixture lap\"],\n  \"tire_energy_j\": [" << 1000*lap_time << ", " << 1000*lap_time << ", "
         << 1000*lap_time << ", " << 1000*lap_time << "],\n  \"sensitivities\": [\n"
         << "    {\"parameter\": \"mass_kg\", \"unit\": \"kg\", \"value\": 660, \"seconds_per_unit\": 0.003, \"step\": -10, "
         << "\"method\": \"solver\", \"half_step\": 5, \"cross_check_seconds_per_unit\": null},\n"
         << "    {\"parameter\": \"downforce_area_m2\", \"unit\": \"m^2\", \"value\": 4.5, \"seconds_per_unit\": -0.3, \"step\": 0.1, "
         << "\"method\": \"solver\", \"half_step\": 0.1, \"cross_check_seconds_per_unit\": null}\n  ]\n}\n";
    const auto put = [&](const char* name, const std::string& content) {
        QFile file(directory+"/"+name);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) throw std::runtime_error("optimal lap fixture unwritable");
        file.write(QByteArray::fromStdString(content));
    };
    put("lap.csv", csv.str());
    put("optimal_lap.json", json.str());
    return directory;
}
// An oval 60 m by 30 m on its centreline, 4 m wide, run counterclockwise from its bottom, in PacSim's track format, with
// a timekeeping gate 5 m along: the course the desktop is given to load (decision 0032).
QString writeOvalCourse(const QString& output) {
    const QString path = output+"/fd-oval.yaml";
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) throw std::runtime_error("oval course unwritable");
    QByteArray text = "track:\n  version: 1.0\n  lanesFirstWithLastConnected: true\n  start:\n    position: [0.0, 0.0, 0.0]\n"
                      "    orientation: [0.0, 0.0, 0.0]\n  earthToTrack:\n    position: [0.0, 0.0, 0.0]\n    orientation: [0.0, 0.0, 0.0]\n";
    const auto lane = [&](const char* name, double offset, const char* cls) {
        text += QByteArray("  ")+name+":\n";
        for (int k = 0; k < 48; ++k) {
            const double a = -std::numbers::pi/2+2*std::numbers::pi*k/48;
            const double nx = -std::cos(a)/30, ny = -std::sin(a)/15, n = std::hypot(nx, ny);
            text += "  - position: ["+QByteArray::number(30*std::cos(a)+nx/n*offset, 'g', 17)+", "+
                    QByteArray::number(15*std::sin(a)+15+ny/n*offset, 'g', 17)+", 0.0]\n    class: "+cls+"\n";
        }
    };
    lane("left", 2, "blue");
    lane("right", -2, "yellow");
    text += "  time_keeping:\n  - position: [5.0, -3.0, 0.0]\n    class: timekeeping\n  - position: [5.0, 3.0, 0.0]\n    class: timekeeping\n"
            "  unknown: []\n";
    file.write(text);
    return path;
}
// A recorded dynamic-plant run, entering the first corner.
QString writeDynamicFixture(const QString& output) {
    const QString path = output+"/replay-fixture-dynamic";
    QDir(path).removeRecursively();
    fd::Simulation simulation(fd::make_preset_track(), fd::Config{}, fd::DynamicSingleTrack{});
    fd::RunRequest request{0, 12.0, {}};
    fd::RecordingWriter writer(std::filesystem::path(path.toStdU16String()), simulation, request, {"ui-verification", "", ""});
    while (simulation.state().time_s+simulation.config().fixed_dt_s <= request.duration_cap_s+1e-9) {
        simulation.step();
        writer.record(simulation);
    }
    writer.finish(simulation);
    return path;
}
#ifdef FD_HAVE_MPCC
// A recorded run driven by MPCC, three seconds from the start.
QString writeMpccFixture(const QString& output) {
    const QString path = output+"/replay-fixture-mpcc";
    QDir(path).removeRecursively();
    // The four-wheel car, so the recording carries both MPCC's plans and the wheels whose margins replay measures.
    fd::Simulation simulation(fd::make_preset_track(), fd::Config{}, fd::FourWheelCar{}, fd::SteeringMode::pure_pursuit,
                              fd::SpeedPlanMode::grip_fractions, fd::LocalPlannerMode::lattice, std::make_shared<fd::Mpcc>());
    fd::RunRequest request{0, 3.0, {}};
    fd::RecordingWriter writer(std::filesystem::path(path.toStdU16String()), simulation, request, {"ui-verification", "", ""});
    while (simulation.state().time_s+simulation.config().fixed_dt_s <= request.duration_cap_s+1e-9) {
        simulation.step();
        writer.record(simulation);
    }
    writer.finish(simulation);
    return path;
}
#endif
// A recorded run of the understeering car steered by MAP, entering the first corner.
QString writeMapFixture(const QString& output) {
    const QString path = output+"/replay-fixture-map";
    QDir(path).removeRecursively();
    fd::Simulation simulation(fd::make_preset_track(), fd::Config{}, fd::DynamicSingleTrack::soft_front(),
                              fd::SteeringMode::model_acceleration_pursuit);
    fd::RunRequest request{0, 12.0, {}};
    fd::RecordingWriter writer(std::filesystem::path(path.toStdU16String()), simulation, request, {"ui-verification", "", ""});
    while (simulation.state().time_s+simulation.config().fixed_dt_s <= request.duration_cap_s+1e-9) {
        simulation.step();
        writer.record(simulation);
    }
    writer.finish(simulation);
    return path;
}
// A recorded run of the four-wheel car with all braking on the rear axle, which locks the rear wheels
// braking into the first corner.
QString writeWheelsFixture(const QString& output) {
    const QString path = output+"/replay-fixture-wheels";
    QDir(path).removeRecursively();
    fd::FourWheelCar car;
    car.brake_bias_front = 0;
    fd::Simulation simulation(fd::make_preset_track(), fd::Config{}, car);
    fd::RunRequest request{0, 11.0, {}};
    fd::RecordingWriter writer(std::filesystem::path(path.toStdU16String()), simulation, request, {"ui-verification", "", ""});
    while (simulation.state().time_s+simulation.config().fixed_dt_s <= request.duration_cap_s+1e-9) {
        simulation.step();
        writer.record(simulation);
    }
    writer.finish(simulation);
    return path;
}
// The same run as a schema 1 directory, as that schema was written: no decision, path, profile or command files, no vehicle
// model, steering law, plant, geometric steering, wheel speed or wheel load columns, and metadata declaring schema 1.
QString writeSchema1Fixture(const QString& fixture, const QString& output) {
    const QString path = output+"/replay-fixture-schema1";
    QDir(path).removeRecursively();
    QDir().mkpath(path);
    for (const QString& name : QDir(fixture).entryList(QDir::Files)) {
        if (name == "decisions.csv" || name == "trajectories.csv" || name == "paths.csv" || name == "profiles.csv" ||
            name == "commands.csv" || name == "measurements.csv" || name == "cones.csv" || name == "perception_frames.csv" ||
            name == "detections.csv" || name == "missed.csv" || name == "beliefs.csv" || name == "believed_paths.csv" ||
            name == "timing.csv") continue;
        if (!QFile::copy(fixture+"/"+name, path+"/"+name)) throw std::runtime_error("schema 1 fixture copy failed");
    }
    const auto rewrite = [&](const QString& name, const std::function<QByteArray(const QByteArray&)>& change) {
        QFile file(path+"/"+name);
        if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error("schema 1 fixture file unreadable");
        const QByteArray changed = change(file.readAll());
        file.close();
        if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) throw std::runtime_error("schema 1 fixture file unwritable");
        file.write(changed);
    };
    const auto without_line_containing = [](const QByteArray& text, const QByteArray& key) {
        QList<QByteArray> kept;
        for (const QByteArray& line : text.split('\n')) if (!line.contains(key)) kept.append(line);
        return kept.join('\n');
    };
    rewrite("metadata.json", [&](const QByteArray& text) {
        QByteArray changed = without_line_containing(without_line_containing(text, "\"vehicle_model\""), "\"steering\"");
        changed = without_line_containing(without_line_containing(changed, "\"speed_plan\""), "\"envelope_fraction\"");
        changed = without_line_containing(without_line_containing(changed, "\"local_planner\""), "\"predictive_controller\"");
        // The configuration's last line loses its comma along with the share of the envelope after it.
        const auto last = changed.indexOf("\"lookahead_time_s\": ");
        const auto comma = changed.indexOf(',', last);
        if (last >= 0 && comma >= 0 && comma < changed.indexOf('\n', last)) changed.remove(comma, 1);
        changed = without_line_containing(changed, "\"competition\"");
        changed.replace("\"schema_version\": 18", "\"schema_version\": 1");
        return changed;
    });
    // Schema 1 tracks had no corridor edge columns.
    rewrite("track.csv", [](const QByteArray& text) {
        QList<QByteArray> rows;
        for (QByteArray row : text.split('\n')) {
            if (row.isEmpty()) continue;
            if (row.endsWith('\r')) row.chop(1);
            for (int column = 0; column < 2; ++column) row.truncate(row.lastIndexOf(','));
            rows.append(row);
        }
        return rows.join('\n')+'\n';
    });
    rewrite("summary.json", [&](const QByteArray& text) { return without_line_containing(text, "\"max_rear_sideslip_rad\""); });
    rewrite("telemetry.csv", [](const QByteArray& text) {
        QList<QByteArray> rows;
        for (QByteArray row : text.split('\n')) {
            if (row.isEmpty()) continue;
            if (row.endsWith('\r')) row.chop(1);
            for (int column = 0; column < 13; ++column) row.truncate(row.lastIndexOf(','));
            rows.append(row);
        }
        return rows.join('\n')+'\n';
    });
    return path;
}
double firstCornerSpeed(const Bridge& bridge) {
    const auto& points = bridge.track().points;
    const auto corner = std::find_if(points.begin(), points.end(), [](const fd::PathPoint& p) { return p.curvature > 0.04; });
    if (corner == points.end()) throw std::runtime_error("track has no tight corner");
    return bridge.plan().at(static_cast<std::size_t>(corner-points.begin())).speed_mps;
}
}

// Exercise the real QML signal handlers and bindings, with actual fixed-step motion.
// This is opt-in integration verification, not a mock renderer or alternate simulation.
void scheduleUiVerification(QGuiApplication& app, QQmlApplicationEngine& engine,
                            Bridge& bridge, const QString& output, QStringList& warnings, bool& finished) {
    struct Context {int stage{}, checks{}, plan_changes{}; double paused_time{},paused_turn{},before_kart_length{},before_kart_wheelbase{},start_x{},low_corner{},prediction_end_speed{}; QString fixture; QVariantList paused_prediction;};
    auto context=std::make_shared<Context>();
    auto* timer=new QTimer(&app);
    auto* window=qobject_cast<QQuickWindow*>(engine.rootObjects().first());
    auto* showroom=qobject_cast<Showroom*>(engine.rootContext()->contextProperty("showroom").value<QObject*>());
    auto* liveCars=engine.rootContext()->contextProperty("liveCars").value<QObject*>();
    auto* sound=qobject_cast<RaceAudio*>(engine.rootContext()->contextProperty("raceAudio").value<QObject*>());
    QDir().mkpath(output);
    QObject::connect(&bridge,&Bridge::planChanged,&app,[context]{++context->plan_changes;});
    QObject::connect(timer,&QTimer::timeout,&app,[=,&app,&bridge,&warnings,&finished] {
        QFile stage_file(output+"/stage.txt");
        if(stage_file.open(QIODevice::WriteOnly))stage_file.write(QByteArray::number(context->stage));
        auto check=[&](bool okay,const char* message) {
            ++context->checks;
            if(!okay)throw std::runtime_error(message);
        };
        auto item=[&](const char* name) {
            auto* found=window->findChild<QObject*>(QString::fromLatin1(name));
            // Items a Repeater creates are only reachable through the visual item tree.
            if(!found) {
                const std::function<QObject*(QQuickItem*)> search=[&](QQuickItem* at) -> QObject* {
                    if(at->objectName()==QString::fromLatin1(name)) return at;
                    for(auto* child:at->childItems()) if(auto* hit=search(child)) return hit;
                    return nullptr;
                };
                found=search(window->contentItem());
            }
            if(!found)throw std::runtime_error(std::string("Missing QML object: ")+name);
            return found;
        };
        auto geometry=[&](const char* name) {
            auto* found=window->findChild<TrackGeometry*>(QString::fromLatin1(name));
            if(!found)throw std::runtime_error(std::string("Missing scene geometry: ")+name);
            return found;
        };
        // Whether an item's bottom edge lies above a scene height, for layouts that must fit the compact window.
        auto endsAbove=[&](const char* name,double limit) {
            auto* at=qobject_cast<QQuickItem*>(item(name));
            return at && at->mapToScene(QPointF(0,at->height())).y()<limit;
        };
        // Whether a card readout's value, drawn right-aligned, starts at least 4 px after its label ends.
        auto labelClear=[&](const char* name) {
            auto* at=qobject_cast<QQuickItem*>(item(name));
            if(!at)return false;
            const auto parts=at->childItems();
            if(parts.size()<2)return false;
            const double label_end=parts[0]->x()+parts[0]->implicitWidth();
            const double value_start=parts[1]->x();
            return label_end+4<=value_start;
        };
        // The top of the dock, which every card and panel over the scene must end above.
        auto dockTop=[&] {
            auto* dock=qobject_cast<QQuickItem*>(item("dock"));
            return dock ? dock->mapToScene(QPointF(0,0)).y() : 0.0;
        };
        // Whether an item's top edge lies below a scene height.
        auto startsBelow=[&](const char* name,double limit) {
            auto* at=qobject_cast<QQuickItem*>(item(name));
            return at && at->mapToScene(QPointF(0,0)).y()>limit;
        };
        auto bottomOf=[&](const char* name) {
            auto* at=qobject_cast<QQuickItem*>(item(name));
            return at ? at->mapToScene(QPointF(0,at->height())).y() : 0.0;
        };
        // The panels the dock opens: settings, telemetry and layers. A check of what a panel shows opens it first, as a
        // viewer would, and a capture opens what it is evidence of.
        auto panels=[&](bool settings,bool insights,bool layers) {
            window->setProperty("settingsOpen",settings);
            window->setProperty("insightsOpen",insights);
            window->setProperty("layersOpen",layers);
        };
        // The reason card stays between the speed readouts above it and the dock below it.
        auto whyCardFits=[&] {
            return startsBelow("whyCard",bottomOf("speedCluster")) && endsAbove("whyCard",dockTop());
        };
        auto click=[&](const char* name) {
            check(item(name)->property("enabled").toBool(),"QML control unexpectedly disabled");
            check(QMetaObject::invokeMethod(item(name),"clicked"),"QML click signal unavailable");
        };
        // The scene's car for the plant's appearance (decision 0036): the showroom's exported car, loaded, with each wheel
        // where the export measured it and its contact shadow, wherever LiveCars offers one; the silhouette and its halo
        // otherwise. Its reflection is the same car mirrored in the road's surface.
        auto checkSceneCar=[&] {
            const auto entry=liveCars->property("cars").toList().value(bridge.carAppearance()).toMap();
            const bool built=entry.value("available").toBool();
            auto* car=item("simulatedCar");
            auto* mirrored=item("carReflection");
            check(car->property("live").toBool()==built && mirrored->property("live").toBool()==built,
                  "the scene draws the exported car exactly where one is built, and its reflection the same car");
            check(mirrored->property("scale").value<QVector3D>()==QVector3D(1,-1,1) &&
                  mirrored->property("position").value<QVector3D>()==car->property("position").value<QVector3D>(),
                  "the reflection is the car mirrored in the road's surface, where the car stands");
            if(built) {
                auto* body=item("liveCarBody");
                check(body->property("status").toInt()==1 && body->property("item").value<QObject*>() &&
                      body->property("source").toUrl()==entry.value("body").toUrl(),"the exported body is loaded");
                const auto wheels=entry.value("wheels").toList();
                for(int i=0;i<4;++i) {
                    const QByteArray name="carWheel"+QByteArray::number(i);
                    const auto at=item(name.constData())->property("position").value<QVector3D>();
                    const auto expected=wheels[i].toMap().value("position").toList();
                    check(std::abs(at.x()-expected[0].toDouble())<1e-5 && std::abs(at.y()-expected[1].toDouble())<1e-5 &&
                          std::abs(at.z()-expected[2].toDouble())<1e-5,"each wheel sits where the export measured it");
                }
                check(item("carShadow")->property("visible").toBool() && !item("carHalo")->property("visible").toBool(),
                      "the exported car grounds itself with its contact shadow instead of the silhouette's halo");
            } else {
                check(item("carHalo")->property("visible").toBool() && !item("carShadow")->property("visible").toBool(),
                      "the silhouette keeps its halo");
            }
        };
        auto predictionGeometry=[&] {
            auto* geometry=qobject_cast<TrackGeometry*>(item("predictionGeometry"));
            if (!geometry) throw std::runtime_error("Prediction geometry unavailable");
            return geometry;
        };
        auto checkPrediction=[&] {
            const auto& points=bridge.prediction();
            check(bridge.predictionAvailable() && points.size()>2,"live prediction has future states");
            check(bridge.predictionSeconds()>2.9 && bridge.predictionSeconds()<=3.000001,"prediction remains a finite time horizon");
            check(bridge.predictionDistance()>0 && bridge.predictionDistance()<bridge.trackLength()/3,"prediction covers only the nearby trajectory");
            check(points.front().state.time_s==bridge.simulationTime() && points.front().state.x_m==bridge.x() && points.front().state.y_m==bridge.y(),"visible prediction starts at actual current state");
            check(std::all_of(points.begin()+1,points.end(),[&](const auto& point){return point.state.time_s>bridge.simulationTime();}),"expired prediction samples trimmed");
            const auto bytes=predictionGeometry()->vertexData();
            check(bytes.size()>=6*7*static_cast<qsizetype>(sizeof(float)),"live prediction has ribbon geometry");
            float vertices[14];
            std::memcpy(vertices,bytes.constData(),sizeof(vertices));
            check(std::abs((vertices[0]+vertices[7])/2-bridge.x())<1e-4 && std::abs((vertices[2]+vertices[9])/2+bridge.y())<1e-4,"ribbon begins at actual rear axle instead of track centerline");
            const double acceleration=(points[1].state.speed_mps-points[0].state.speed_mps)/(points[1].state.time_s-points[0].state.time_s);
            const QColor expected(acceleration>fd::plan_color_deadband_mps2?"#89eab5":acceleration < -fd::plan_color_deadband_mps2?"#f47770":"#e6cf78");
            auto linear=[](float v){return v<=0.04045f?v/12.92f:std::pow((v+0.055f)/1.055f,2.4f);};
            check(std::abs(vertices[3]-linear(expected.redF()))<1e-5 && std::abs(vertices[4]-linear(expected.greenF()))<1e-5 && std::abs(vertices[5]-linear(expected.blueF()))<1e-5,"ribbon color follows achieved predicted speed change");
            check(item("plannedTrajectory")->property("visible").toBool(),"prediction mesh visible in live mode");
            check(item("speedPlotLabel")->property("text").toString()=="PREDICTED SPEED","live speed graph labels its prediction");
        };
        auto finish=[&](const QString& failure) {
            finished=true;
            QJsonArray warning_json;for(const auto& w:warnings)warning_json.append(w);
            QFile result(output+"/verification.json");
            if(result.open(QIODevice::WriteOnly)) result.write(QJsonDocument(QJsonObject{
                {"passed",failure.isEmpty()},{"checks",context->checks},{"failure",failure},
                {"qml_warnings",warning_json},{"method","Actual QML controls, bindings and QQuickWindow captures"}
            }).toJson());
            timer->stop();app.exit(failure.isEmpty()?0:3);
        };
        try {
            switch(context->stage++) {
            case 0:
                // A controller in the room must not drive the checks; the controls they need are given here (decision 0037).
                bridge.setGamepadEnabled(false);
                check(showroom && showroom->screen()=="driving", "verification bypasses startup media");
                item("showroomView")->setProperty("showMedia",false);
                click("showroomButton");
                check(showroom->screen()=="showroom" && item("showroomView")->property("visible").toBool(), "Choose car opens the showroom");
                // The arrows browse AMR23, RB19, Jesko, Urus and wrap; from AMR23 back twice is Urus, then Jesko.
                click("showroomNext"); check(showroom->requested()==3, "the next arrow goes from AMR23 to RB19");
                click("showroomNext"); click("showroomNext"); check(showroom->requested()==2, "the next arrow continues to Jesko, then Urus");
                click("showroomNext"); check(showroom->requested()==0, "the next arrow wraps from Urus to AMR23");
                click("showroomPrevious"); check(showroom->requested()==2, "the previous arrow wraps from AMR23 to Urus");
                click("showroomPrevious");
                check(showroom->requested()==1 && bridge.carAppearance()==0, "the arrows queue the last preview without changing the plant appearance");
                click("showroomContinue");
                check(!item("showroomContinue")->property("enabled").toBool(), "selection commit blocks duplicate confirmation");
                for(int transition=0; transition<8 && showroom->screen()=="showroom"; ++transition) showroom->complete(showroom->serial());
                check(showroom->screen()=="setup" && bridge.carAppearance()==1 && !bridge.running() && bridge.simulationTime()==0,
                      "selection completion connects the appearance to parked track setup without advancing physics");
                bridge.setAppearance(0);
                check(!bridge.running() && bridge.simulationTime()==0,"initial lifecycle");
                checkSceneCar();
                check(item("reflectionView")->property("visible").toBool(),"the reflection is drawn beneath the road");
                check(QMetaObject::invokeMethod(item("reflectionLegend"),"toggled") && !window->property("showReflections").toBool() &&
                      !item("reflectionView")->property("visible").toBool(),"the layers switch hides the reflection");
                check(QMetaObject::invokeMethod(item("reflectionLegend"),"toggled") && item("reflectionView")->property("visible").toBool(),
                      "and shows it again");
                check(item("carWheel2")->property("turned").toDouble()==0,"the wheels stand still before the car moves");
                check(bridge.latticeAvailable() && bridge.lattice()->layers.size()==fd::make_lattice(bridge.simulation().track(),bridge.config()).layers.size() &&
                      !bridge.latticeLayersAhead().empty() && !geometry("latticeGeometry")->vertexData().isEmpty() && item("latticeAhead")->property("visible").toBool(),
                      "the offline lattice is laid along the preset and drawn ahead of the car");
                check(item("speedValue")->property("text").toString()=="0","initial speed binding");
                check(!item("playheadPanel")->property("visible").toBool(),"playhead hidden in live mode");
                // The drive alone to begin with: every panel opens from the dock, and the driver chip opens the settings.
                check(!item("settingsDrawer")->property("visible").toBool() && !item("insightsPanel")->property("visible").toBool() &&
                      !item("layersPanel")->property("visible").toBool(),"the drive starts with every panel closed");
                click("settingsButton"); click("insightsButton"); click("layersButton");
                check(item("settingsDrawer")->property("visible").toBool() && item("insightsPanel")->property("visible").toBool() &&
                      item("layersPanel")->property("visible").toBool(),"the dock opens the settings, the telemetry and the layers");
                click("settingsButton"); click("insightsButton"); click("layersButton");
                check(!item("settingsDrawer")->property("visible").toBool() && !item("insightsPanel")->property("visible").toBool() &&
                      !item("layersPanel")->property("visible").toBool(),"and closes them again");
                click("driverChip");
                check(item("settingsDrawer")->property("visible").toBool(),"the driver chip opens the settings");
                click("settingsCloseButton");
                check(!item("settingsDrawer")->property("visible").toBool(),"the settings close from their own header");
                check(item("toastText")->property("text").toString().startsWith("Ready"),"the status line says the car is ready");
                checkPrediction();
                context->start_x=bridge.x();click("runButton");break;
            case 1:case 2:case 3:case 4: break;
            case 5:
                check(bridge.running() && bridge.simulationTime()>0,"Start advances actual simulation");
                check(bridge.x()!=context->start_x && bridge.speedKmh()>0,"plant motion reaches UI adapter");
                check(!item("showroomButton")->property("enabled").toBool(),"moving showroom guard");
                check(!item("openRecordingButton")->property("enabled").toBool(),"moving recording-open guard");
                check(std::abs(item("carWheel2")->property("turned").toDouble())>1 && std::abs(item("carWheel0")->property("turned").toDouble())>1,
                      "the wheels turn as the car rolls");
                click("runButton");context->paused_time=bridge.simulationTime();
                context->paused_turn=item("carWheel2")->property("turned").toDouble();break;
            case 6:
                check(!bridge.running() && bridge.simulationTime()==context->paused_time,"Pause freezes time");
                check(item("carWheel2")->property("turned").toDouble()==context->paused_turn,"the wheels stop with the car");
                check(item("speedValue")->property("text").toString().toInt()==static_cast<int>(std::round(bridge.speedKmh())),"actual speed text");
                check(item("targetSpeedValue")->property("text").toString().toInt()==static_cast<int>(std::round(bridge.targetSpeedKmh())),"target speed text");
                checkPrediction();
                context->prediction_end_speed=bridge.prediction().back().state.speed_mps;
                context->paused_prediction=bridge.predictionPoints();
                item("gripSlider")->setProperty("value",0.45);
                check(QMetaObject::invokeMethod(item("gripSlider"),"moved"),"grip moved handler");
                check(bridge.revision()==1 && bridge.simulation().config().grip_mu==1,"paused change defers active plan");
                check(bridge.grip()==0.45 && bridge.status().contains("queued"),"pending grip disclosed");
                check(bridge.predictionPoints()==context->paused_prediction,"queued grip leaves the active prediction unchanged");
                click("runButton");break;
            case 7:
                click("runButton");
                check(bridge.revision()==2 && bridge.simulation().config().grip_mu==0.45,"grip commit on next tick");
                check(bridge.simulation().events().size()==1,"one recorded change");
                check(bridge.prediction().back().state.speed_mps<context->prediction_end_speed-1,"committed lower grip changes predicted future speed");
                bridge.setAppearance(1);check(bridge.carAppearance()==1,"paused appearance changes the live silhouette");
                click("overviewButton");check(bridge.overview(),"overview signal connected");
                window->resize(1100,700);break;
            case 8:
                // The appearance changed while paused: the scene draws the new car, still where the plant is.
                checkSceneCar();
                check(item("simulatedCar")->property("appearance").toInt()==1,"the scene follows the chosen appearance");
                break;
            case 9: break;
            case 10:
                checkPrediction();
                check(window->grabWindow().save(output+"/compact-overview.png"),"compact capture");
                click("resetButton");
                check(!bridge.running() && bridge.simulationTime()==0 && bridge.speedKmh()==0,"Reset restores ready state");
                check(bridge.revision()==1 && bridge.simulation().events().empty(),"Reset clears run events");
                check(bridge.grip()==0.45,"Reset preserves applied grip");
                click("followViewButton");break;
            case 11: break;
            case 12:
                check(item("speedValue")->property("text").toString()=="0","reset speed binding");
                check(window->grabWindow().save(output+"/compact-ready.png"),"ready capture");
                {
                    // The compact window's HUD: the speed readouts, the reason card, the status line and the dock stand apart,
                    // and every panel, opened, ends above the dock. Rendering lays them out.
                    panels(true,true,true);
                    check(!window->grabWindow().isNull(),"the compact HUD renders with its panels open");
                    check(whyCardFits(),"the reason card sits between the speed readouts and the dock");
                    auto* why=qobject_cast<QQuickItem*>(item("whyCard"));
                    auto* toast=qobject_cast<QQuickItem*>(item("statusToast"));
                    check(why && toast && endsAbove("statusToast",dockTop()) &&
                          why->mapToScene(QPointF(why->width(),0)).x()<=toast->mapToScene(QPointF(0,0)).x(),
                          "the status line stands clear of the reason card, above the dock");
                    check(endsAbove("mapCard",dockTop()) && endsAbove("insightsPanel",dockTop()) && endsAbove("settingsDrawer",dockTop()) &&
                          endsAbove("layersPanel",dockTop()),"the map, the telemetry, the settings and the layers end above the dock");
                    panels(false,false,false);
                }
                // Replay: an invalid directory is refused with the violated rule; a valid one enters replay.
                check(!bridge.loadRecording(output+"/no-such-run") && !bridge.replay(),"invalid recording refused");
                check(bridge.status().contains("Recording contract"),"refusal names the contract");
                context->fixture=writeReplayFixture(output);
                check(bridge.loadRecording(context->fixture),"recording loads");
                check(showroom->screen()=="driving" && !item("showroomButton")->property("enabled").toBool(), "replay bypasses showroom and disables appearance selection");
                // Count revision changes from here, after the load's own plan refresh.
                context->plan_changes=0;
                check(bridge.replay() && !bridge.running() && bridge.simulationTime()==0 && bridge.revision()==1,"replay starts at the first sample");
                check(bridge.recordedEvents()==1 && bridge.appliedEvents()==0,"recorded change counted but not yet applied");
                check(item("playheadPanel")->property("visible").toBool(),"playhead visible in replay");
                check(!item("gripSlider")->property("enabled").toBool(),"grip control disabled in replay");
                check(item("modeLabel")->property("text").toString().contains("REPLAY"),"mode label switches");
                check(item("recordingName")->property("text").toString()=="replay-fixture","recording name shown");
                // Schema 2: the prediction shown is the one recorded with the decision, anchored at the recorded sample.
                check(bridge.recordedDecision() && bridge.recordedDecision()->time_s==0,"the first sample shows the decision recorded at time zero");
                check(bridge.predictionAvailable() && bridge.prediction().front().state.time_s==0 && bridge.prediction().front().state.x_m==bridge.x(),"replay shows the recorded prediction from the recorded state");
                check(item("plannedTrajectory")->property("visible").toBool() && !predictionGeometry()->vertexData().isEmpty(),"replay draws the recorded prediction ribbon");
                check(item("predictionLabel")->property("text").toString()=="RECORDED PREDICTION","replay labels the prediction as recorded");
                check(item("speedPlotLabel")->property("text").toString()=="RECORDED PREDICTION","replay graph labels the recorded prediction");
                check(item("decisionProvenance")->property("text").toString().contains("as the car made it"),"replay states the decision is recorded, not recomputed");
                break;
            case 13:
                bridge.seek(8);
                check(bridge.revision()==2 && bridge.grip()==0.45 && bridge.appliedEvents()==1,"seek forward applies the recorded change");
                check(bridge.predictionAvailable() && std::abs(bridge.prediction().front().state.time_s-8)<1e-9,"the recorded prediction follows the cursor");
                check(bridge.recordedDecision()->time_s<=8 && bridge.recordedDecision()->time_s>8-0.02-1e-9,"the decision in force is the latest made before the sample");
                check(item("gripReadout")->property("text").toString()=="μ 0.45","recorded grip readout");
                check(std::abs(item("playhead")->property("value").toDouble()-8)<1e-6,"playhead binding follows the cursor");
                check(context->plan_changes==1,"plan revision change notified once");
                check(item("speedValue")->property("text").toString().toInt()==static_cast<int>(std::round(bridge.speedKmh())),"recorded speed text");
                context->low_corner=firstCornerSpeed(bridge);
                break;
            case 14:
                bridge.seek(3);
                check(bridge.revision()==1 && bridge.grip()==1 && bridge.appliedEvents()==0,"seek backward restores revision 1 and its configuration");
                check(item("gripReadout")->property("text").toString()=="μ 1.00","initial grip readout restored");
                check(firstCornerSpeed(bridge)>context->low_corner+1,"earlier plan revision restored in the scene data");
                check(context->plan_changes==2,"backward revision change notified");
                check(std::abs(item("playhead")->property("value").toDouble()-3)<1e-6,"playhead follows backward seek");
                check(std::abs(bridge.simulationTime()-3)<1e-9,"displayed time is the recorded sample time");
                check(bridge.status().contains("Replay"),"replay status shown");
                // At 3 s the recorded car was choosing a line around the lane blockage.
                check(bridge.decisionAvailable() && bridge.evaluatedOptions()>1 && bridge.rejectedOptions()>=1,"a recorded decision with alternatives is in force");
                check(bridge.selectedOffset()>1 && !bridge.holdingForBlockage(),"the recorded choice steered left of the blockage");
                check(bridge.localPlanner()==0 && bridge.chosenAction()=="pass left","the recorded lattice planner's action is replayed");
                panels(true,false,false);
                check(!item("plannerRow")->property("visible").toBool() && item("steeringCaption")->property("text").toString().contains("lattice planner"),
                      "replay names the recorded planner instead of offering a choice");
                {
                    // The layers panel explains every colour in the scene, the recorded actions' included, and fits the compact
                    // window above the dock and the scrubber in it. Rendering lays the scene out, so the positions are the drawn ones.
                    panels(false,false,true);
                    check(!window->grabWindow().isNull(),"the replay scene renders");
                    auto* legend=qobject_cast<QQuickItem*>(item("layersPanel"));
                    check(legend && item("sceneLegend") && item("actionLegend")->property("visible").toBool(),"the recorded actions' colours are explained");
                    check(endsAbove("layersPanel",dockTop()) && legend->mapToScene(QPointF(0,0)).y()>=0 &&
                          legend->mapToScene(QPointF(legend->width(),0)).x()<=window->width(),
                          "the layers panel fits the compact window above the dock");
                    panels(false,false,false);
                }
                check(item("decisionPanel")->property("visible").toBool() && item("decisionLabel")->property("text").toString()=="PASS LEFT","recorded decision state is shown");
                check(item("decisionCost")->property("visible").toBool() && item("decisionCost")->property("text").toString().contains("deviation"),
                      "the recorded path's cost breakdown is shown");
                check(!bridge.setLocalPlanner(1) && bridge.status().contains("recorded local planner"),"replay refuses a planner change and says why");
                check(!bridge.actionTimes().isEmpty() && bridge.actionTimes().front().toMap()["seconds"].toDouble()==*bridge.recordedDecision()->options[1].estimated_time_s,
                      "replay labels the recorded pass with its recorded estimated time");
                check(item("decisionReason")->property("text").toString().contains("cone-cluster"),"the recorded reason names the blockage");
                check(item("rejectedAlternatives")->property("visible").toBool() && !geometry("alternativesGeometry")->vertexData().isEmpty(),"recorded rejected alternatives are drawn");
                check(bridge.reason().contains("cone-cluster"),"the recorded blockage explains the recorded speed");
                click("runButton");break;
            case 15:case 16:case 17: break;
            case 18:
                check(bridge.running() && bridge.playheadTime()>3.05 && bridge.playheadTime()<6,"Play advances the playhead in wall time");
                check(bridge.revision()==1,"playback stays on revision 1 before the recorded change");
                check(bridge.simulation().state().time_s==0 && bridge.simulation().diagnostics().revision==1,"playback never advances the live plant");
                click("runButton");
                check(!bridge.running(),"replay pauses");
                check(window->grabWindow().save(output+"/compact-replay.png"),"replay capture");
                item("playhead")->setProperty("value",9.0);
                check(QMetaObject::invokeMethod(item("playhead"),"moved"),"playhead moved handler");
                check(bridge.revision()==2 && std::abs(bridge.playheadTime()-9)<1e-9,"dragging the playhead seeks");
                click("exitReplayButton");
                check(!bridge.replay() && !bridge.running() && bridge.simulationTime()==0 && bridge.revision()==1,"exit replay returns to the untouched live simulation");
                check(bridge.grip()==0.45,"live configuration preserved across replay");
                // A schema 1 run discloses what it lacks instead of inventing a forecast or a choice.
                check(bridge.loadRecording(writeSchema1Fixture(context->fixture,output)),"schema 1 recording loads");
                check(bridge.vehicleModel()==0 && !bridge.tireSlipModeled(),"a schema 1 run replays as the kinematic bicycle it was");
                bridge.seek(3);
                check(!bridge.decisionAvailable() && !bridge.predictionAvailable() && bridge.prediction().empty(),"schema 1 replay exposes no invented prediction or decision");
                check(!item("plannedTrajectory")->property("visible").toBool() && predictionGeometry()->vertexData().isEmpty(),"schema 1 replay has no prediction ribbon");
                check(item("predictionHorizon")->property("text").toString()=="No recorded prediction","schema 1 replay discloses missing recorded predictions");
                check(item("speedPlotLabel")->property("text").toString()=="REFERENCE SPEED","schema 1 replay graph labels the recorded reference");
                check(bridge.obstructionCount()==1 && item("decisionReason")->property("text").toString()=="No recorded decision","schema 1 replay shows the scenario without a decision");
                check(geometry("alternativesGeometry")->vertexData().isEmpty(),"schema 1 replay draws no alternatives");
                click("exitReplayButton");
                check(!bridge.replay() && bridge.simulationTime()==0,"exit from the schema 1 replay");
                check(!item("playheadPanel")->property("visible").toBool(),"playhead hidden after exit");
                check(item("gripSlider")->property("enabled").toBool(),"grip control enabled again");
                checkPrediction();
                break;
            case 19:
                // A deterministic corner approach checks curvature and colors independently
                // of wall-clock frame timing used for the lifecycle checks above.
                bridge.setGrip(1.0);
                bridge.advanceForCapture(8.0);
                checkPrediction();
                {
                    const auto& points=bridge.prediction();
                    const auto& a=points.front().state;
                    const auto& b=points.back().state;
                    const double chord=std::hypot(b.x_m-a.x_m,b.y_m-a.y_m);
                    const bool bends=std::any_of(points.begin(),points.end(),[&](const auto& point){
                        return std::abs((b.x_m-a.x_m)*(point.state.y_m-a.y_m)-(b.y_m-a.y_m)*(point.state.x_m-a.x_m))/std::max(chord,1e-9)>0.25;
                    });
                    check(bends,"local predicted path bends through the upcoming corner");
                    check(std::any_of(points.begin(),points.end(),[](const auto& point){return point.acceleration_mps2 < -fd::plan_color_deadband_mps2;}),"corner prediction includes achieved braking");
                }
                break;
            case 20:
                check(window->grabWindow().save(output+"/compact-prediction-corner.png"),"corner prediction capture");
                // Scenario: a stated blockage on the reference line, and the visible choice.
                check(bridge.obstructionCount()==0,"no blockage is stated to begin with");
                check(!item("decisionPanel")->property("visible").toBool(),"decision panel hidden with a clear corridor");
                panels(true,false,false);
                check(item("plannerRow")->property("visible").toBool() && item("lineRow")->property("visible").toBool(),
                      "the local planner is offered in the settings with the scenario controls, and the line while the corridor is clear");
                check(!item("statedBlockages")->property("visible").toBool(),"no blockage geometry with a clear corridor");
                click("resetButton");
                click("blockLaneButton");
                check(bridge.obstructionCount()==1,"the scenario control states one blockage");
                check(item("plannerRow")->property("visible").toBool() && !item("lineRow")->property("visible").toBool(),
                      "with a blockage stated the local planner stays offered and the line waits for the corridor to clear");
                check(!window->grabWindow().isNull() && endsAbove("plannerRow",dockTop()) && endsAbove("plannerRow",bottomOf("settingsDrawer")),
                      "the planner choice is in view in the settings, above the dock in the compact window");
                check(item("scenarioSummary")->property("text").toString().contains("1 stated"),"the scenario is summarised");
                check(item("statedBlockages")->property("visible").toBool() &&
                      !geometry("blockageGeometry")->vertexData().isEmpty(),"the stated blockage is drawn");
                panels(false,false,false);
                break;
            case 21:
                // Far enough back that the lateral shift fits the grip budget comfortably.
                bridge.advanceForCapture(2.0);
                check(bridge.obstructionCount()==1,"the blockage persists while driving");
                check(!bridge.holdingForBlockage(),"a clear alternative exists, so the car does not brake");
                check(bridge.selectedOffset()>1,"the car chose a line left of the blockage");
                check(bridge.rejectedOptions()>=1,"alternatives were evaluated and rejected");
                check(bridge.evaluatedOptions()>1,"more than one line was considered");
                check(item("decisionPanel")->property("visible").toBool(),"decision panel appears");
                check(bridge.localPlanner()==0 && item("latticePlannerButton")->property("selected").toBool(),"the lattice planner is the default");
                check(item("decisionLabel")->property("text").toString()=="PASS LEFT","the chosen action is shown");
                check(item("decisionCost")->property("visible").toBool() && item("decisionCost")->property("text").toString().contains("turns"),
                      "the chosen path's cost breakdown is shown");
                panels(false,false,true);
                check(item("actionLegend")->property("visible").toBool(),"the action colours are explained");
                panels(false,false,false);
                check(item("toastText")->property("text").toString().startsWith("Passing left"),"the status line says the car is passing left");
                {
                    // The pass is labelled with its estimated time where its predicted line is, framed as the choice.
                    const auto times=bridge.actionTimes();
                    check(times.size()>=1 && times.front().toMap()["chosen"].toBool() && times.front().toMap()["label"].toString()=="Left" &&
                          times.front().toMap()["seconds"].toDouble()>0,"the chosen pass's estimated time is available");
                    check(!window->grabWindow().isNull(),"the scene renders its labels");
                    auto* label=qobject_cast<QQuickItem*>(item("actionTimeLabel0"));
                    check(label && label->isVisible() && label->property("text").toString().startsWith("Left") &&
                          label->property("text").toString().endsWith(" s"),"the pass is labelled with its estimated time in the scene");
                }
                check(item("decisionPanel")->property("visible").toBool() && whyCardFits(),
                      "the passing decision, its count and its cost fit between the speed readouts and the dock in the compact window");
                check(item("decisionReason")->property("text").toString().contains("cone cluster"),
                      "the reason names the blockage");
                check(item("rejectedAlternatives")->property("visible").toBool() &&
                      !geometry("alternativesGeometry")->vertexData().isEmpty(),
                      "the rejected alternatives are drawn beside the chosen line");
                check(item("decisionCount")->property("text").toString().contains("rejected") &&
                      item("decisionCount")->property("text").toString().contains("m across") && !item("decisionOptions")->property("visible").toBool(),
                      "the crossing and the count of rejected actions are shown on the decision's title line");
                break;
            case 22:
                bridge.advanceForCapture(1.6);
                check(bridge.crossTrackError()>0.5,"the car actually moved left of the reference line");
                {
                    // Still labelled where the capture shows it, 15 m along the pass's predicted line.
                    check(!window->grabWindow().isNull(),"the avoiding scene renders");
                    auto* label=qobject_cast<QQuickItem*>(item("actionTimeLabel0"));
                    check(label && label->isVisible(),"the chosen pass's estimated time is labelled in view as the car avoids");
                }
                check(window->grabWindow().save(output+"/compact-avoidance.png"),"avoidance capture");
                click("clearBlockagesButton");
                check(bridge.obstructionCount()==0,"clearing removes the stated blockage");
                check(!item("decisionPanel")->property("visible").toBool(),"decision panel hides again");
                click("blockTrackButton");
                check(bridge.obstructionCount()==1,"a full-width blockage can be stated");
                break;
            case 23:
                bridge.advanceForCapture(2.5);
                check(bridge.holdingForBlockage(),"a full-width blockage leaves no clear line");
                check(item("decisionLabel")->property("text").toString()=="BRAKE" && !item("decisionCost")->property("visible").toBool(),
                      "the brake action is shown, without a path cost");
                check(!window->grabWindow().isNull() && whyCardFits(),"the braking decision fits between the speed readouts and the dock in the compact window");
                check(item("decisionReason")->property("text").toString().contains("stalled car"),
                      "the braking reason names the blockage");
                check(bridge.simulation().decision().choice().speed_limit_mps<bridge.simulation().state().speed_mps ||
                      bridge.simulation().state().speed_mps<0.5,"the car is being held below its blocked-line speed");
                check(window->grabWindow().save(output+"/compact-blocked.png"),"blocked capture");
                // The five-offset planner stays selectable for comparison: a fresh run keeps the scenario, holds for the
                // full-width blockage among its offsets, and shows no lattice cost.
                click("fiveOffsetsPlannerButton");
                check(bridge.localPlanner()==1 && item("fiveOffsetsPlannerButton")->property("selected").toBool() &&
                      bridge.simulationTime()==0 && bridge.obstructionCount()==1,"choosing the five-offset planner starts a fresh run with the same scenario");
                bridge.advanceForCapture(6.0);
                check(bridge.actionTimes().isEmpty(),"the five-offset planner estimates no times");
                check(bridge.holdingForBlockage() && bridge.evaluatedOptions()>4 && item("decisionLabel")->property("text").toString()=="BLOCKED" &&
                      !item("decisionCost")->property("visible").toBool() && !item("actionLegend")->property("visible").toBool(),
                      "the five-offset planner holds among its offsets without a lattice cost");
                check(item("decisionOptions")->property("visible").toBool() && !item("decisionCount")->property("visible").toBool() &&
                      !window->grabWindow().isNull() && whyCardFits(),"the five-offset decision keeps its line count and fits above the dock");
                click("latticePlannerButton");
                check(bridge.localPlanner()==0 && bridge.simulationTime()==0 && bridge.obstructionCount()==1,"the lattice planner is chosen again");
                click("clearBlockagesButton");
                // A narrow blockage entering the first corner leaves both ways past it clear, so both are timed and
                // labelled with their estimated times and the car takes the faster (decision 0023).
                check(bridge.addObstruction({130, 136, -0.5, 1.5, "corner bollard"}),"a narrow blockage can be stated at the first corner");
                // Far enough in that the two paths have separated, so their labels stand apart in the scene.
                bridge.advanceForCapture(6.8);
                {
                    const auto times=bridge.actionTimes();
                    check(times.size()==2,"both ways past a narrow blockage are timed");
                    const auto left=times.front().toMap(), right=times.back().toMap();
                    check(left["label"].toString()=="Left" && right["label"].toString()=="Right" &&
                          left["seconds"].toDouble()>0 && right["seconds"].toDouble()>0,"each is labelled by its side with its own estimated time");
                    check(left["chosen"].toBool()!=right["chosen"].toBool(),"exactly one of them is the choice");
                    const auto& chosen=left["chosen"].toBool() ? left : right, & other=left["chosen"].toBool() ? right : left;
                    check(chosen["seconds"].toDouble()<=other["seconds"].toDouble()+fd::lattice_switch_margin_s,
                          "the choice is the faster of the two, or within the switching margin of it");
                    check(!window->grabWindow().isNull(),"the scene renders both labels");
                    auto* first=qobject_cast<QQuickItem*>(item("actionTimeLabel0"));
                    auto* second=qobject_cast<QQuickItem*>(item("actionTimeLabel1"));
                    check(first && second && first->isVisible() && second->isVisible() &&
                          first->property("text").toString().startsWith("Left") && second->property("text").toString().startsWith("Right") &&
                          first->property("text").toString().endsWith(" s") && second->property("text").toString().endsWith(" s"),
                          "both actions are labelled with their estimated times in the scene");
                    check(first && second &&
                          (std::abs(first->x()-second->x())>(first->width()+second->width())/2 ||
                           first->y()+first->height()<=second->y() || second->y()+second->height()<=first->y()),
                          "neither label covers the other, though the two paths have barely separated");
                    // A reason that carries the margin in seconds takes a second line, so this is the tightest the card gets.
                    check(whyCardFits(),
                          "the decision, its margin in seconds and its cost fit between the speed readouts and the dock in the compact window");
                    check(window->grabWindow().save(output+"/compact-action-times.png"),"action time capture");
                }
                click("clearBlockagesButton");
                check(bridge.obstructionCount()==0,"the narrow blockage is cleared again");
                break;
            case 24:
                // Vehicle model: the kinematic bicycle by default, the dynamic plant on request. From here the telemetry stays
                // open, as a viewer comparing the models would keep it.
                panels(false,true,false);
                check(bridge.vehicleModel()==0 && item("kinematicModelButton")->property("selected").toBool(),"the kinematic bicycle is the default model");
                check(item("insightsEmpty")->property("visible").toBool(),"the telemetry says the kinematic car has no tires to show");
                check(!item("slipCard")->property("visible").toBool(),"no sideslip readout for a model that cannot slide");
                click("dynamicModelButton");
                check(bridge.vehicleModel()==1 && bridge.tireSlipModeled() &&
                      std::holds_alternative<fd::DynamicSingleTrack>(bridge.simulation().vehicle_model()),"the live simulation now uses the dynamic plant");
                check(bridge.simulationTime()==0 && !bridge.running(),"changing the model starts a fresh run");
                check(item("dynamicModelButton")->property("selected").toBool() && item("slipCard")->property("visible").toBool(),
                      "the model choice and the sideslip readout are shown");
                bridge.advanceForCapture(9.5);
                break;
            case 25:
            {
                check(bridge.simulationTime()>9 && bridge.lap()==0,"the dynamic car drove into the first corner");
                check(std::abs(bridge.rearSideslipDegrees())>0.1,"the dynamic car shows sideslip in the corner");
                check(item("sideslipMetric")->property("value").toString().endsWith(QString::fromUtf8("°")),"the sideslip readout is in degrees");
                check(bridge.predictionAvailable(),"the prediction rolls the dynamic plant forward");
                check(window->grabWindow().save(output+"/compact-dynamic.png"),"dynamic model capture");
                click("runButton");
                check(!item("dynamicModelButton")->property("enabled").toBool() && !item("kinematicModelButton")->property("enabled").toBool(),
                      "the model cannot change while running");
                click("runButton");
                const QString dynamic_fixture = writeDynamicFixture(output);
                check(bridge.loadRecording(dynamic_fixture),"a recorded dynamic run loads");
                bridge.seek(10);
                fd::Playback expected(fd::load_recording(std::filesystem::path(dynamic_fixture.toStdU16String())));
                expected.seek_time(10);
                const double recorded = fd::rear_sideslip_rad(expected.sample().plant_state())*180/std::numbers::pi;
                check(bridge.vehicleModel()==1 && std::abs(bridge.rearSideslipDegrees()-recorded)<1e-12,"replay shows the recorded model and recorded sideslip");
                check(!item("kinematicModelButton")->property("enabled").toBool() && item("slipCard")->property("visible").toBool(),
                      "replay locks the model choice and keeps the sideslip readout");
                check(!bridge.setVehicleModel(0) && bridge.status().contains("recorded vehicle model"),"replay refuses a model change and says why");
                click("exitReplayButton");
                click("kinematicModelButton");
                check(bridge.vehicleModel()==0 && !item("slipCard")->property("visible").toBool() &&
                      std::holds_alternative<fd::KinematicBicycle>(bridge.simulation().vehicle_model()),"switching back restores the kinematic bicycle");
                // Steering law: Pure Pursuit by default; MAP only for a model with tires.
                check(bridge.steeringMode()==0 && item("pursuitSteeringButton")->property("selected").toBool() &&
                      !item("steeringCorrectionMetric")->property("visible").toBool(),"Pure Pursuit is the default steering law");
                check(!item("mapSteeringButton")->property("enabled").toBool(),"MAP is not offered for the kinematic bicycle");
                check(!bridge.setSteeringMode(1) && bridge.status().contains("dynamic vehicle model"),"MAP on the kinematic bicycle is refused with the reason");
                click("softFrontModelButton");
                check(bridge.vehicleModel()==2 && item("softFrontModelButton")->property("selected").toBool() && bridge.steeringMode()==0,
                      "the soft-front car is selected, still under Pure Pursuit");
                click("mapSteeringButton");
                check(bridge.steeringMode()==1 && bridge.simulation().steering_table() &&
                      bridge.simulation().steering_table()->generated_for(bridge.simulation().vehicle_model(), bridge.simulation().config()),
                      "MAP runs with a table generated for the live plant");
                check(bridge.simulationTime()==0 && item("steeringCorrectionMetric")->property("visible").toBool() &&
                      item("steeringCaption")->property("text").toString().startsWith("MAP"),"changing the law starts a fresh run and shows MAP");
                bridge.advanceForCapture(9.5);
                break;
            }
            case 26:
            {
                const double live = bridge.steeringCorrectionDegrees();
                const auto& d = bridge.simulation().diagnostics();
                const double limit = bridge.simulation().config().max_steering_rad;
                check(std::abs(live-(std::clamp(d.requested.steering_rad,-limit,limit)-std::clamp(d.geometric_steering_rad,-limit,limit))*180/std::numbers::pi)<1e-12,
                      "the correction readout is the controller's MAP minus geometric steering");
                check(live>0.2,"in the first corner MAP steers the understeering car further than geometry");
                check(item("steeringCorrectionMetric")->property("value").toString().startsWith("+"),"the readout shows the sign of the correction");
                {
                    // Tire slip card and velocity vector for the understeering car in the corner.
                    const double degrees=180/std::numbers::pi;
                    const auto balance=fd::handling_balance(fd::demand(bridge.simulation().vehicle_model(),bridge.simulation().plant_state(),
                                                                      bridge.simulation().config(),d.applied.acceleration_mps2));
                    check(item("slipCard")->property("visible").toBool() && item("velocityVector")->property("visible").toBool(),
                          "a model with tires shows the slip card and the velocity vector");
                    check(std::abs(bridge.frontSlipDegrees()-d.front_slip_angle_rad*degrees)<1e-12 && std::abs(bridge.rearSlipDegrees()-d.rear_slip_angle_rad*degrees)<1e-12 &&
                          std::abs(bridge.frontPeakSlipDegrees()-d.front_peak_slip_angle_rad*degrees)<1e-12,"the card shows the simulation's axle slip angles");
                    check(d.balance==fd::Balance::understeer && balance.balance==d.balance && bridge.handlingBalance()=="UNDERSTEER" &&
                          item("balanceLabel")->property("text").toString()=="UNDERSTEER","the soft-front car is labelled as understeering in the corner");
                    check(std::abs(bridge.understeerDegrees()-d.understeer_angle_rad*degrees)<1e-12 &&
                          item("balanceDefinition")->property("text").toString().contains("Understeer above +0.25"),"the balance is shown with its definition");
                    check(item("frontSlipValue")->property("text").toString().endsWith(QString::number(bridge.frontPeakSlipDegrees(),'f',1)+QString::fromUtf8("°")),
                          "the front bar states its peak");
                    const double arrow=item("velocityArrow")->property("eulerRotation").value<QVector3D>().y();
                    const double heading=item("headingLine")->property("eulerRotation").value<QVector3D>().y();
                    check(std::abs((arrow-heading)-bridge.rearSideslipDegrees())<1e-3 && std::abs(bridge.rearSideslipDegrees())>0.1,
                          "the velocity arrow departs from the heading by the rear sideslip");
                }
                check(window->grabWindow().save(output+"/compact-map.png"),"MAP steering capture");
                click("runButton");
                check(!item("mapSteeringButton")->property("enabled").toBool() && !item("pursuitSteeringButton")->property("enabled").toBool(),
                      "the steering law cannot change while running");
                click("runButton");
                const QString map_fixture = writeMapFixture(output);
                check(bridge.loadRecording(map_fixture),"a recorded MAP run loads");
                bridge.seek(9.5);
                fd::Playback expected(fd::load_recording(std::filesystem::path(map_fixture.toStdU16String())));
                expected.seek_time(9.5);
                const auto& s = expected.sample();
                const double recorded = (std::clamp(s.requested_steering_rad,-limit,limit)-std::clamp(s.geometric_steering_rad,-limit,limit))*180/std::numbers::pi;
                check(bridge.steeringMode()==1 && bridge.vehicleModel()==2 && std::abs(bridge.steeringCorrectionDegrees()-recorded)<1e-12,
                      "replay shows the recorded law, car and correction");
                panels(true,true,false);
                check(!item("steeringRow")->property("visible").toBool() &&
                      item("steeringCaption")->property("text").toString()=="Recorded steering: MAP · grip fraction plan · lattice planner",
                      "replay states the recorded law, plan and planner instead of offering a choice");
                panels(false,true,false);
                check(!bridge.setSteeringMode(0) && bridge.status().contains("recorded steering law"),"replay refuses a steering change and says why");
                {
                    const auto derived=expected.demand();
                    check(std::abs(bridge.frontSlipDegrees()-derived.front_slip_angle_rad*180/std::numbers::pi)<1e-12 &&
                          bridge.handlingBalance()==QString::fromUtf8(fd::balance_name(fd::handling_balance(derived).balance)).toUpper() &&
                          item("slipCard")->property("visible").toBool(),"replay derives the card from the recorded plant state");
                }
                click("exitReplayButton");
                click("kinematicModelButton");
                check(bridge.vehicleModel()==0 && bridge.steeringMode()==0 && !bridge.simulation().steering_table() &&
                      !item("steeringCorrectionMetric")->property("visible").toBool(),"the kinematic bicycle returns to Pure Pursuit");
                check(!item("slipCard")->property("visible").toBool() && !item("velocityVector")->property("visible").toBool() &&
                      bridge.handlingBalance()=="NOT MODELED","the kinematic bicycle shows no tire slip, balance or velocity vector");
                // MPCC (decision 0024) is the third way to drive the chosen action: a fresh run whose chosen action MPCC plans,
                // its plan the prediction drawn, and its status and solve time in the caption.
                check(!item("mpccControlButton")->property("enabled").toBool() && !bridge.setControl(2),
                      "MPCC is not offered for the kinematic bicycle, which has no tires to predict with");
                click("dynamicModelButton");
                check(bridge.mpccAvailable()==item("mpccControlButton")->property("enabled").toBool(),"MPCC is offered exactly when the build has it");
#ifdef FD_HAVE_MPCC
                click("mpccControlButton");
                check(bridge.controllerMode()==1 && bridge.simulationTime()==0 && item("mpccControlButton")->property("selected").toBool() &&
                      !item("pursuitSteeringButton")->property("selected").toBool(),"choosing MPCC starts a fresh run driven by it");
                bridge.advanceForCapture(3.0);
                check(bridge.simulation().controller_outcome().driven_by==fd::ControllerMode::mpcc &&
                      bridge.simulation().decision().trajectory().points.size()==61,"MPCC drives, and its plan is the prediction");
                check(item("steeringCaption")->property("text").toString().startsWith("MPCC drives") &&
                      item("steeringCaption")->property("text").toString().endsWith(" ms"),"the caption gives MPCC's status and solve time");
                check(item("plannedTrajectory")->property("visible").toBool() && !predictionGeometry()->vertexData().isEmpty(),"its plan is drawn as the ribbon");
                check(window->grabWindow().save(output+"/compact-mpcc.png"),"MPCC capture");
                // Into the first corner, where MPCC takes the inside of the corridor rather than the reference.
                bridge.advanceForCapture(6.0);
                check(bridge.simulation().controller_outcome().driven_by==fd::ControllerMode::mpcc && bridge.simulation().diagnostics().plan_valid,
                      "MPCC still drives, with a valid sample, into the first corner");
                check(window->grabWindow().save(output+"/compact-mpcc-corner.png"),"MPCC corner capture");
                click("runButton");
                check(!item("mpccControlButton")->property("enabled").toBool() && !bridge.setControl(0),"what drives cannot change while running");
                click("runButton");
                {
                    const QString mpcc_fixture = writeMpccFixture(output);
                    check(bridge.loadRecording(mpcc_fixture),"a recorded MPCC run loads");
                    bridge.seek(2.0);
                    check(bridge.controllerMode()==1 && item("steeringCaption")->property("text").toString().contains("MPCC drove"),
                          "replay shows the recorded controller and who drove");
                    check(!bridge.setControl(0) && bridge.status().contains("recorded controller"),"replay refuses a change of controller and says why");
                    check(bridge.planTireUseAvailable() && bridge.planTireUse().size()==61 && item("tireMarginPanel")->property("visible").toBool() &&
                          bridge.planTireWorstUse()>0 && bridge.planTireWorstUse()<1.2,
                          "replay drives the recorded plant along the recorded plan's own commands to show each tire's margin");
                    click("exitReplayButton");
                }
                click("kinematicModelButton");
                check(bridge.vehicleModel()==0 && bridge.controllerMode()==0 && item("pursuitSteeringButton")->property("selected").toBool(),
                      "choosing the kinematic bicycle returns to the policy");
                click("dynamicModelButton");
                click("mpccControlButton");
                click("pursuitSteeringButton");
                check(bridge.controllerMode()==0 && bridge.steeringMode()==0 && item("pursuitSteeringButton")->property("selected").toBool(),
                      "Pursuit returns to the policy");
#endif
                click("kinematicModelButton");
                // The four-wheel car: steered by either law, with its wheels shown.
                click("fourWheelModelButton");
                check(bridge.vehicleModel()==3 && std::holds_alternative<fd::FourWheelCar>(bridge.simulation().vehicle_model()) &&
                      bridge.steeringMode()==0,"the four-wheel car is selected under Pure Pursuit");
                check(item("mapSteeringButton")->property("enabled").toBool(),"MAP is offered for the four-wheel car too");
                click("mapSteeringButton");
                check(bridge.steeringMode()==1 && bridge.simulation().steering_table()!=nullptr &&
                      bridge.simulation().steering_table()->generated_for(bridge.simulation().vehicle_model(),bridge.simulation().config()),
                      "and steers it with a table generated for that car");
                click("pursuitSteeringButton");
                check(bridge.steeringMode()==0,"Pure Pursuit returns");
                check(item("wheelGrid")->property("visible").toBool() && item("slipCard")->property("visible").toBool(),"the card shows the four wheels");
                bridge.advanceForCapture(9.5);
                break;
            }
            case 27:
            {
                const auto& plant=bridge.simulation().plant_state();
                const auto live=fd::demand(bridge.simulation().vehicle_model(),plant,bridge.simulation().config(),
                                           bridge.simulation().diagnostics().applied.acceleration_mps2);
                const auto states=bridge.wheelStates();
                const auto ratios=bridge.wheelSlipRatios();
                check(states.size()==4 && ratios.size()==4,"four wheel states and slip ratios are reported");
                for (std::size_t i=0;i<4;++i) {
                    const bool lock=plant.wheel_speeds_radps[i]==0 && plant.pose.speed_mps>0.5;
                    const QString expected=lock ? "LOCK" : live.wheel_combined_slip[i]>1 ? (live.wheel_slip_ratios[i]>0 ? "SPIN" : "SLIDE") : "GRIP";
                    check(states[static_cast<qsizetype>(i)].toString()==expected && ratios[static_cast<qsizetype>(i)].toDouble()==live.wheel_slip_ratios[i],
                          "each wheel's state and slip ratio come from the plant");
                }
                check(item("rearLeftWheelState")->property("text").toString()==states[2].toString(),"the tile shows the wheel's state");
                {
                    // Wheel loads and friction circles from the plant (decision 0014).
                    const auto loads=bridge.wheelLoads();
                    const auto still=bridge.staticWheelLoads();
                    const auto use=bridge.wheelFrictionUse();
                    const auto& car=std::get<fd::FourWheelCar>(bridge.simulation().vehicle_model());
                    const auto expected_still=fd::quasi_static_wheel_loads(car,bridge.simulation().config(),0,0,0);
                    check(loads.size()==4 && still.size()==4 && use.size()==4,"four wheel loads, loads at rest and friction points are reported");
                    double total=0,lightest=1e9,heaviest=0;
                    const QStringList tiles{"frontLeftWheel","frontRightWheel","rearLeftWheel","rearRightWheel"};
                    for (std::size_t i=0;i<4;++i) {
                        const auto q=static_cast<qsizetype>(i);
                        const double load=loads[q].toDouble();
                        const auto pair=use[q].toList();
                        check(load==plant.wheel_loads_n[i] && still[q].toDouble()==expected_still[i],"each wheel's load comes from the plant");
                        check(pair.size()==2 && pair[0].toDouble()==live.wheel_longitudinal_use[i] && pair[1].toDouble()==live.wheel_lateral_use[i] &&
                              std::hypot(pair[0].toDouble(),pair[1].toDouble())<=1+1e-9,"each tire's force lies in its friction circle, from the plant");
                        check(std::abs(item(qPrintable(tiles[q]))->property("load").toDouble()-load)<1e-9 &&
                              item(qPrintable(tiles[q]+"Load"))->property("text").toString()==QString::number(load/1000,'f',2)+" kN","the tile shows the wheel's load");
                        const auto* bar_item=qobject_cast<QQuickItem*>(item(qPrintable(tiles[q]+"LoadBar")));
                        const double bar=bar_item->width();
                        const double track=bar_item->parentItem()->width();
                        check(std::abs(bar-track*std::min(1.0,load/(2*expected_still[i])))<1e-6,"the load bar measures the load against twice the load at rest");
                        total+=load; lightest=std::min(lightest,load); heaviest=std::max(heaviest,load);
                    }
                    check(std::abs(total-car.mass_kg*fd::gravity_mps2)<1e-6*total,"the four loads carry the car's weight");
                    check(heaviest-lightest>200,"in the first corner the wheels visibly carry different loads");
                    // Drag and downforce from the plant state (decision 0015).
                    const auto air=fd::aerodynamic_force(car,plant.pose.speed_mps,plant.lateral_velocity_mps+car.cg_to_rear_m*plant.yaw_rate_radps);
                    const double drag=std::hypot(air.longitudinal_n,air.lateral_n);
                    check(bridge.aerodynamicsModeled() && item("aeroMetric")->property("visible").toBool() &&
                          bridge.dragNewtons()==drag && bridge.downforceNewtons()==air.downforce_n,"the card shows the plant's drag and downforce");
                    check(item("aeroMetric")->property("value").toString()==QString::number(drag/1000,'f',2)+" / "+QString::number(air.downforce_n/1000,'f',2)+" kN" &&
                          drag>0 && air.downforce_n==0,"the readout states the drag in kN, and no downforce for a car without wings");
                }
                check(std::none_of(states.begin(),states.end(),[](const QVariant& s){return s.toString()=="LOCK";}),
                      "the default four-wheel car does not lock a wheel braking into the first corner");
                std::string standard_envelope;
                {
                    // G-G diagram (decision 0017): the envelope derived from this car, its boundary at this speed and the car's point.
                    bridge.deriveEnvelopeNow();
                    const auto& gg_car=std::get<fd::FourWheelCar>(bridge.simulation().vehicle_model());
                    check(bridge.envelopeModeled() && bridge.envelopeReady() && item("ggCard")->property("visible").toBool() &&
                          bridge.envelopeFingerprint().toStdString()==fd::performance_envelope_fingerprint(gg_car,bridge.simulation().config()),
                          "the four-wheel car shows its G-G envelope, derived from this car");
                    standard_envelope=bridge.envelopeFingerprint().toStdString();
                    const auto boundary=bridge.envelopeBoundary();
                    const auto left=bridge.envelope()->side_at(bridge.simulation().state().speed_mps,fd::TurnSide::left);
                    const auto right=bridge.envelope()->side_at(bridge.simulation().state().speed_mps,fd::TurnSide::right);
                    check(boundary.size()==24 && std::abs(boundary[5].toMap()["lateral"].toDouble()-left.levels.back().lateral_mps2)<1e-9 &&
                          std::abs(boundary[0].toMap()["longitudinal"].toDouble()-left.levels.front().forward_mps2)<1e-9 &&
                          std::abs(boundary[6].toMap()["longitudinal"].toDouble()-left.levels.back().braking_mps2)<1e-9 &&
                          std::abs(boundary[17].toMap()["lateral"].toDouble()+right.levels.back().lateral_mps2)<1e-9 &&
                          std::abs(left.levels.back().lateral_mps2-left.lateral_limit_mps2)<0.02*left.lateral_limit_mps2,
                          "the boundary runs through the envelope's levels at the car's speed, out to its lateral limit");
                    check(bridge.ggLateral()==bridge.simulation().diagnostics().lateral_acceleration_mps2 &&
                          std::abs(bridge.ggLateral())<std::max(left.lateral_limit_mps2,right.lateral_limit_mps2),"the car's point is its lateral acceleration, inside the envelope in the corner");
                    check(item("ggStatus")->property("text").toString().contains(bridge.envelopeFingerprint().left(8)) &&
                          item("ggStatus")->property("text").toString().contains("this car") && !bridge.envelopeFailed(),"the diagram names the envelope it draws");
                    check(window->grabWindow().save(output+"/four-wheel-gg.png"),"G-G envelope capture");
                }
                {
                    // Setup controls (decision 0016): the registry's controls, editable only while paused, each change a fresh run.
                    const auto registry=fd::four_wheel_setup_controls();
                    const auto controls=bridge.setupControls();
                    const auto& live_car=std::get<fd::FourWheelCar>(bridge.simulation().vehicle_model());
                    panels(true,true,false);
                    check(controls.size()==static_cast<qsizetype>(registry.size()) && item("setupButton")->property("visible").toBool(),
                          "the four-wheel car offers every setup control");
                    for (qsizetype i=0;i<controls.size();++i) {
                        const auto map=controls[i].toMap();
                        const auto& control=registry[static_cast<std::size_t>(i)];
                        check(map["key"].toString().toStdString()==std::string(control.key) && map["value"].toDouble()==fd::setup_value(live_car,control.key) &&
                              map["minimum"].toDouble()==control.minimum && map["maximum"].toDouble()==control.maximum,"each control shows the live car's value and its offered range");
                    }
                    click("setupButton");
                    check(item("setupPanel")->property("visible").toBool() && item("setupSlider_brake_bias_front")->property("enabled").toBool() &&
                          item("ggCard")->property("visible").toBool(),"the setup panel opens in the settings, editable while paused, beside the G-G diagram");
                    click("runButton");
                    check(!item("setupSlider_brake_bias_front")->property("enabled").toBool() && !bridge.setSetupValue("brake_bias_front",0.2) &&
                          bridge.status().contains("Pause"),"the setup cannot change while running, and says why");
                    click("runButton");
                    const double before_mass=std::get<fd::FourWheelCar>(bridge.simulation().vehicle_model()).mass_kg;
                    check(!bridge.setSetupValue("mass_kg",2000) && bridge.status().contains("offered range") &&
                          std::get<fd::FourWheelCar>(bridge.simulation().vehicle_model()).mass_kg==before_mass,"a value outside the offered range is refused and changes nothing");
                    check(bridge.setSetupValue("brake_bias_front",0.2) && bridge.simulationTime()==0,"a setup change starts a fresh run");
                    bridge.deriveEnvelopeNow();
                    check(bridge.envelopeReady() && bridge.envelopeFingerprint().toStdString()!=standard_envelope,"the new setup has its own envelope");
                    const auto& rearward=std::get<fd::FourWheelCar>(bridge.simulation().vehicle_model());
                    check(rearward.brake_bias_front==0.2 && std::abs(item("setupSlider_brake_bias_front")->property("value").toDouble()-0.2)<1e-12 &&
                          item("setupValue_brake_bias_front")->property("text").toString()=="20 %","the panel shows the new brake bias");
                    check(window->grabWindow().save(output+"/compact-setup.png"),"setup panel capture");
                    click("setupButton");
                    panels(false,true,false);
                    double lock_at=-1;
                    // The planner brakes for the first corner after about 7.5 s; search from there in small steps.
                    bridge.advanceForCapture(7.5);
                    for (int i=0;i<90 && lock_at<0;++i) {
                        bridge.advanceForCapture(0.05);
                        const auto& p=bridge.simulation().plant_state();
                        if (p.pose.speed_mps>1 && (p.wheel_speeds_radps[fd::rear_left]==0 || p.wheel_speeds_radps[fd::rear_right]==0)) lock_at=p.pose.time_s;
                    }
                    check(lock_at>0 && bridge.wheelStates()[2].toString()=="LOCK","with the brake bias moved rearward the rear wheels lock braking into the first corner");
                    check(window->grabWindow().save(output+"/compact-setup-lock.png"),"rearward brake bias lock capture");
                    check(bridge.resetSetup() && std::get<fd::FourWheelCar>(bridge.simulation().vehicle_model()).brake_bias_front==0.5 && bridge.simulationTime()==0,
                          "defaults restore the standard car in a fresh run");
                }
                {
                    // Speed plan (decision 0018): the four-wheel car plans from a share of its envelope as a fresh run.
                    panels(true,true,false);
                    check(item("speedPlanRow")->property("visible").toBool() && bridge.speedPlanMode()==0 && item("envelopePlanButton")->property("enabled").toBool() &&
                          item("fractionPlanButton")->property("selected").toBool() && item("ggCanvas")->property("planShare").toDouble()==0,
                          "the four-wheel car offers the envelope plan while paused");
                    click("runButton");
                    check(!item("envelopePlanButton")->property("enabled").toBool() && !bridge.setSpeedPlanMode(1) && bridge.status().contains("Pause"),
                          "the speed plan cannot change while running, and says why");
                    click("runButton");
                    const auto baseline_plan=bridge.simulation().plan();
                    click("envelopePlanButton");
                    const auto* plan_envelope=bridge.simulation().performance_envelope();
                    check(bridge.speedPlanMode()==1 && plan_envelope && bridge.simulationTime()==0 && item("envelopePlanButton")->property("selected").toBool(),
                          "choosing the envelope plan starts a fresh run planned from the car's envelope");
                    const auto expected_plan=fd::make_speed_plan(bridge.simulation().track(),bridge.simulation().config(),plan_envelope);
                    bool same=expected_plan.size()==bridge.simulation().plan().size();
                    double faster=0;
                    for (std::size_t i=0;same && i<expected_plan.size();++i) {
                        same=bridge.simulation().plan()[i].speed_mps==expected_plan[i].speed_mps;
                        faster=std::max(faster,expected_plan[i].speed_mps-baseline_plan[i].speed_mps);
                    }
                    check(same && faster>0.5,"the plan in force is the envelope plan, faster than the grip fractions");
                    check(bridge.envelopeReady() && bridge.envelopeFingerprint().toStdString()==plan_envelope->fingerprint(),
                          "the G-G diagram shows the envelope the plan was made from");
                    check(item("envelopePlanButton")->property("text").toString()=="Envelope 80%" && item("ggCanvas")->property("planShare").toDouble()==0.8 &&
                          item("ggStatus")->property("text").toString().contains("the plan uses the inner 80%"),
                          "the plan's share of the envelope is named and drawn inside the envelope");
                    const double budget=bridge.config().longitudinal_grip_fraction*bridge.config().grip_mu*fd::gravity_mps2;
                    double hardest=0;
                    QString braking_reason;
                    bridge.advanceForCapture(5.0);
                    for (int i=0;i<80 && hardest>=-budget-0.2;++i) {
                        bridge.advanceForCapture(0.05);
                        hardest=std::min(hardest,bridge.simulation().diagnostics().applied.acceleration_mps2);
                        if (bridge.simulation().diagnostics().limiting_reason=="braking_reachability") braking_reason=bridge.reason();
                    }
                    check(hardest<-budget-0.2,"braking into the first corner goes beyond the grip fraction's bound");
                    check(braking_reason.contains("braking envelope"),"the explanation names the car's braking envelope");
                    check(std::abs(bridge.brakeBound()-bridge.config().envelope_fraction*-plan_envelope->braking_limit(bridge.simulation().state().speed_mps,0))<1e-9,
                          "the braking bound is the share of the car's braking capacity at its speed");
                    panels(false,true,false);
                    check(window->grabWindow().save(output+"/compact-envelope-plan.png"),"envelope plan capture");
                    click("fractionPlanButton");
                    check(bridge.speedPlanMode()==0 && !bridge.simulation().performance_envelope() && bridge.simulationTime()==0,"the grip fractions return as a fresh run");
                }
                {
                    // Racing line (decision 0020): solved once on a worker, offered while paused, driven as a fresh run on the line
                    // with its corridor's edges, and drawn with the centreline; each line's estimated lap under the plan in force.
                    bridge.solveRacingLineNow();
                    const fd::Track preset=bridge.simulation().track();
                    panels(true,true,true);
                    check(!window->grabWindow().isNull() && endsAbove("lineSummary",dockTop()) && endsAbove("lineSummary",bottomOf("settingsDrawer")),
                          "the line choice and its estimates are in view in the settings, above the dock in the compact window");
#ifdef FD_HAVE_RACELINE
                    check(bridge.racingLineAvailable() && bridge.racingLineReady() && bridge.lineMode()==0 && preset.left_edge_m.empty() &&
                          item("lineRow")->property("visible").toBool() && item("centrelineButton")->property("selected").toBool() &&
                          item("racingLineButton")->property("enabled").toBool(),"the solved racing line is offered beside the centreline run");
                    check(item("comparisonLine")->property("visible").toBool() && !geometry("comparisonLineGeometry")->vertexData().isEmpty() &&
                          bridge.comparisonLine().size()>100 && item("comparisonLegend")->property("visible").toBool(),
                          "the racing line is drawn over the centreline run and named in the legend");
                    click("envelopePlanButton");
                    const auto expected_lap=[&](const fd::Track& t) {
                        return fd::estimated_lap_time(t,fd::make_speed_plan(t,bridge.simulation().config(),bridge.simulation().performance_envelope()));
                    };
                    check(bridge.speedPlanMode()==1 && bridge.centrelineLapSeconds()==expected_lap(preset) && bridge.racingLineLapSeconds()>0 &&
                          bridge.racingLineLapSeconds()<bridge.smoothedCentrelineLapSeconds()-1 && bridge.smoothedCentrelineLapSeconds()<bridge.centrelineLapSeconds(),
                          "each line's estimated lap is the envelope plan's, faster on the racing line than on either centreline");
                    check(item("lineSummary")->property("text").toString().contains(QString::number(bridge.racingLineLapSeconds(),'f',2)) &&
                          item("lineSummary")->property("text").toString().contains(QString::number(bridge.centrelineLapSeconds(),'f',2)),
                          "the estimates are shown beside the choice");
                    click("runButton");
                    check(!item("racingLineButton")->property("enabled").toBool() && !bridge.setLineMode(1) && bridge.status().contains("Pause"),
                          "the line cannot change while running, and says why");
                    click("runButton");
                    click("resetButton");
                    check(bridge.placeBlockageAhead(false) && !item("lineRow")->property("visible").toBool() && !bridge.setLineMode(1) &&
                          bridge.status().contains("clear them"),"with a stated blockage the racing line is refused, and the choice is hidden");
                    click("clearBlockagesButton");
                    click("racingLineButton");
                    const fd::Track line=bridge.simulation().track();
                    check(bridge.lineMode()==1 && bridge.simulationTime()==0 && !line.left_edge_m.empty() && item("racingLineButton")->property("selected").toBool() &&
                          bridge.speedPlanMode()==1 && std::holds_alternative<fd::FourWheelCar>(bridge.simulation().vehicle_model()),
                          "choosing the racing line starts a fresh run on it, keeping the car and its plan");
                    check(bridge.comparisonLine().size()==preset.points.size()+1 && bridge.comparisonLine().front().x==preset.points.front().x_m &&
                          item("comparisonLine")->property("visible").toBool(),"the centreline is drawn beside the racing line run");
                    {
                        // The road is drawn between the line's own edges, not half the width each side: at the sample furthest off
                        // centre, the road's two corners lie at that sample's left and right edge.
                        std::size_t widest=0;
                        for (std::size_t i=0;i<line.points.size();++i)
                            if (std::abs(line.left_edge_m[i]-line.right_edge_m[i])>std::abs(line.left_edge_m[widest]-line.right_edge_m[widest])) widest=i;
                        const auto bytes=geometry("roadGeometry")->vertexData();
                        check(bytes.size()==static_cast<qsizetype>(line.points.size()*6*7*sizeof(float)),"one road quad per racing line sample");
                        float corners[14];
                        std::memcpy(corners,bytes.constData()+static_cast<qsizetype>(widest*6*7*sizeof(float)),sizeof(corners));
                        const auto& p=line.points[widest];
                        const double right_side=std::hypot(corners[0]-p.x_m,-corners[2]-p.y_m);
                        const double left_side=std::hypot(corners[7]-p.x_m,-corners[9]-p.y_m);
                        check(std::abs(right_side-line.right_edge_m[widest])<1e-3 && std::abs(left_side-line.left_edge_m[widest])<1e-3 &&
                              std::abs(line.left_edge_m[widest]-line.right_edge_m[widest])>3,"the road is drawn between the racing line's own corridor edges");
                    }
                    bridge.advanceForCapture(8.0);
                    check(bridge.simulationTime()>7.9 && bridge.simulation().diagnostics().within_track && bridge.simulation().diagnostics().plan_valid &&
                          std::abs(bridge.crossTrackError())<0.6,"the car drives the racing line within its corridor");
                    {
                        // The lattice (decision 0021) is laid along the racing line the car drives, and only the edges leaving the
                        // layers within the horizon ahead are drawn, one line per sample step, moving on as the car passes layers.
                        const auto expected=fd::make_lattice(bridge.simulation().track(),bridge.config());
                        check(bridge.lattice() && bridge.lattice()->layers.size()==expected.layers.size() && bridge.lattice()->edges.size()==expected.edges.size() &&
                              bridge.lattice()->layers.front().offsets_m.size()==expected.layers.front().offsets_m.size(),
                              "the lattice is laid along the racing line the car drives");
                        const auto ahead=bridge.latticeLayersAhead();
                        std::size_t steps=0;
                        for(const auto layer:ahead)
                            for(std::size_t n=0;n<expected.layers[layer].offsets_m.size();++n)
                                for(const auto& edge:expected.edges_from(layer,n)) steps+=edge.samples.size()-1;
                        const auto drawn=geometry("latticeGeometry")->vertexData();
                        check(ahead.size()>5 && drawn.size()==static_cast<qsizetype>(steps*2*7*sizeof(float)),"the scene draws the edges leaving the layers ahead");
                        double farthest=0;
                        for(qsizetype v=0;v<drawn.size()/static_cast<qsizetype>(7*sizeof(float));++v) {
                            float xyz[3];
                            std::memcpy(xyz,drawn.constData()+v*static_cast<qsizetype>(7*sizeof(float)),sizeof(xyz));
                            farthest=std::max(farthest,std::hypot(xyz[0]-bridge.x(),-xyz[2]-bridge.y()));
                        }
                        check(farthest<Bridge::lattice_horizon_m+15 && item("latticeLegend")->property("visible").toBool(),
                              "only the lattice near the car is drawn, and the legend names it");
                        bridge.advanceForCapture(1.0);
                        check(bridge.latticeLayersAhead()!=ahead && geometry("latticeGeometry")->vertexData()!=drawn,"the drawn lattice moves on as the car passes layers");
                    }
                    panels(false,false,false);
                    check(window->grabWindow().save(output+"/compact-racing-line.png"),"racing line capture");
                    click("overviewButton");
                    check(window->grabWindow().save(output+"/compact-racing-line-overview.png"),"racing line overview capture");
                    check(!bridge.placeBlockageAhead(false) && bridge.obstructionCount()==0 && bridge.status().contains("choose the centreline"),
                          "a blockage is refused on the racing line, and says why");
                    click("centrelineButton");
                    check(bridge.lineMode()==0 && bridge.simulationTime()==0 && bridge.simulation().track().left_edge_m.empty() &&
                          bridge.simulation().track().points.size()==preset.points.size() && bridge.comparisonLine().size()==line.points.size()+1,
                          "the centreline returns as a fresh run on the preset, with the racing line drawn beside it");
                    check(window->grabWindow().save(output+"/compact-centreline-overview.png"),"centreline with racing line overview capture");
                    click("followViewButton");
                    click("fractionPlanButton");
#else
                    check(!bridge.racingLineAvailable() && !bridge.racingLineReady() && !item("racingLineButton")->property("enabled").toBool() &&
                          !bridge.setLineMode(1) && bridge.status().contains("bootstrap-osqp"),"a build without OSQP offers no racing line, and says why");
#endif
                }
                const QString wheels_fixture=writeWheelsFixture(output);
                panels(true,true,false);
                check(bridge.loadRecording(wheels_fixture),"a recorded four-wheel run loads");
                fd::Playback expected(fd::load_recording(std::filesystem::path(wheels_fixture.toStdU16String())));
                double lock_time=-1;
                for (std::size_t i=0;i<expected.recording().samples.size();++i) {
                    const auto& s=expected.recording().samples[i];
                    if (s.speed_mps>5 && s.wheel_speeds_radps[fd::rear_left]==0 && s.wheel_speeds_radps[fd::front_left]>0) { lock_time=s.time_s; break; }
                }
                check(lock_time>0,"the rear-biased recording locks its rear wheels while the front ones turn");
                bridge.seek(lock_time+0.1);
                check(bridge.wheelsModeled() && bridge.vehicleModel()==3,"replay shows the recorded four-wheel car");
                const auto replay_states=bridge.wheelStates();
                check(replay_states[2].toString()=="LOCK" && replay_states[3].toString()=="LOCK" &&
                      replay_states[0].toString()!="LOCK" && replay_states[1].toString()!="LOCK","replay shows the rear wheels locked and the front ones turning");
                check(item("rearRightWheelState")->property("text").toString()=="LOCK",
                      "the locked wheel's tile says LOCK");
                check(item("carWheel2")->property("locked").toBool() && item("carWheel3")->property("locked").toBool() &&
                      !item("carWheel0")->property("locked").toBool(),"the car draws its locked rear tires red and its front tires normally");
                {
                    expected.seek_time(lock_time+0.1);
                    const auto recorded=expected.sample().wheel_loads_n;
                    const auto loads=bridge.wheelLoads();
                    check(loads.size()==4 && loads[0].toDouble()==recorded[0] && loads[3].toDouble()==recorded[3],"replay shows the recorded wheel loads");
                    check(recorded[fd::front_left]+recorded[fd::front_right]>recorded[fd::rear_left]+recorded[fd::rear_right]+300,
                          "braking on the rear wheels still moves load onto the front axle");
                }
                {
                    // The recorded car's setup, read only.
                    const auto controls=bridge.setupControls();
                    const auto& recorded_car=std::get<fd::FourWheelCar>(expected.recording().metadata.vehicle_model);
                    bool matches=controls.size()==7;
                    for (const auto& entry : controls) {
                        const auto map=entry.toMap();
                        matches=matches && map["value"].toDouble()==fd::setup_value(recorded_car,map["key"].toString().toStdString());
                    }
                    check(matches && item("setupButton")->property("text").toString()=="Recorded car setup","replay shows the recorded car's setup");
                    check(!item("speedPlanRow")->property("visible").toBool() && item("steeringCaption")->property("text").toString().endsWith("grip fraction plan · lattice planner") &&
                          !bridge.setSpeedPlanMode(1),"replay states the recorded speed plan and refuses a change");
                    check(bridge.latticeAvailable() && bridge.lattice()->layers.size()==fd::make_lattice(bridge.track(),bridge.config()).layers.size(),
                          "replay lays the lattice along the recorded track");
                    check(!item("lineRow")->property("visible").toBool() && !item("comparisonLine")->property("visible").toBool() && bridge.comparisonLine().empty() &&
                          !bridge.setLineMode(1) && bridge.status().contains("recorded track"),"replay shows the recorded track alone and refuses a line change");
                    check(!bridge.setSetupValue("mass_kg",900) && bridge.status().contains("recorded setup") && !bridge.resetSetup(),
                          "replay refuses a setup change and says why");
                    bridge.deriveEnvelopeNow();
                    check(bridge.envelopeReady() && bridge.envelopeFingerprint().toStdString()==fd::performance_envelope_fingerprint(recorded_car,bridge.config()) &&
                          item("ggStatus")->property("text").toString().contains("recorded car"),"replay draws the envelope derived from the recorded car");
                }
                panels(false,true,false);
                check(window->grabWindow().save(output+"/compact-wheels.png"),"four-wheel lock capture");
                click("exitReplayButton");
                // Each tire's friction circle along the driving plan's horizon (decision 0027), for the car that has four.
                check(!bridge.planTireUseAvailable() && bridge.planTireUse().isEmpty() && !item("tireMarginPanel")->property("visible").toBool(),
                      "under the policy there is no plan of commands to measure a tire's margin along");
#ifdef FD_HAVE_MPCC
                click("mpccControlButton");
                bridge.advanceForCapture(6.0);
                {
                    const auto rows=bridge.planTireUse();
                    check(bridge.controllerMode()==1 && bridge.planTireUseAvailable() && rows.size()==61,
                          "MPCC's plan reports each tire's use at every one of its points");
                    const double step=bridge.simulation().config().fixed_dt_s;
                    double worst=0,worst_time=0;
                    const double start=rows.front().toList()[0].toDouble();
                    bool shaped=true,real=true;
                    for (qsizetype i=0;i<rows.size();++i) {
                        const auto row=rows[i].toList();
                        shaped=shaped && row.size()==5 && std::abs(row[0].toDouble()-start-0.05*static_cast<double>(i))<1e-9;
                        for (int w=1;w<=4 && row.size()==5;++w) {
                            const double use=row[w].toDouble();
                            real=real && std::isfinite(use) && use>=0;
                            if (use>worst) { worst=use; worst_time=row[0].toDouble()-start; }
                        }
                    }
                    check(shaped,"every row is a time on the plan's own stages and its four wheels");
                    check(real,"every tire's use is a real share of its own circle");
                    check(std::abs(bridge.planTireWorstUse()-worst)<1e-12 && std::abs(bridge.planTireWorstTime()-worst_time)<1e-12,
                          "the readout names the busiest tire along the horizon and when it is busiest");
                    check(worst>0.2 && worst<1.2,"driving the first corner under MPCC works the tires without sliding them");
                    check(item("tireMarginPanel")->property("visible").toBool() &&
                          item("tireMarginWorst")->property("value").toString()==QString::number(std::lround((1-worst)*100))+"% at "+QString::number(worst_time,'f',1)+" s",
                          "the card states the friction left at the busiest point of the plan");
                    check(step>0 && item("tireMarginPlot")->property("visible").toBool(),"the margin along the horizon is drawn");
                    check(window->grabWindow().save(output+"/compact-tire-margin.png"),"tire margin capture");
                    click("measuredStateButton");
                    click("conesPerceivedButton");
                    bridge.advanceForCapture(0.5);
                    check(item("tireMarginPanel")->property("visible").toBool() && item("measuredCard")->property("visible").toBool() &&
                          item("perceptionReadout")->property("visible").toBool() && item("sensorsCard")->property("visible").toBool() &&
                          !window->grabWindow().isNull() && endsAbove("insightsPanel",dockTop()),
                          "beneath MPCC's margin plot, with instruments and cones, the telemetry shows the sensors card and ends above the dock");
                    click("measuredStateButton");
                    click("conesPerceivedButton");
                    check(!bridge.sensorsFitted() && !bridge.perceptionFitted(),"and both come off again");
                }
                click("pursuitSteeringButton");
                check(bridge.controllerMode()==0 && !bridge.planTireUseAvailable() && !item("tireMarginPanel")->property("visible").toBool(),
                      "and the policy's prediction, which states no commands, has no margin to draw");
#endif
                // The car's own instruments (decision 0029): fitted, they report an older, noisier car than the one
                // driving, and what drives the car does not change.
                panels(true,true,false);
                check(!bridge.sensorsFitted() && !bridge.measuredAvailable() && !item("measuredCard")->property("visible").toBool() &&
                      !item("sensorsCard")->property("visible").toBool() && item("measuredStateButton")->property("visible").toBool() &&
                      !item("measuredStateButton")->property("selected").toBool(),
                      "a car without instruments reads its own state exactly, offers them in the settings, and draws no measurement");
                click("measuredStateButton");
                check(bridge.sensorsFitted() && item("measuredStateButton")->property("selected").toBool() &&
                      item("measuredCard")->property("visible").toBool() && !bridge.measuredAvailable(),
                      "fitting instruments shows the card, with nothing delivered yet");
                check(item("measuredPose")->property("value").toString()=="nothing yet",
                      "which the card says rather than showing a pose of zero");
                {
                    const auto before=bridge.simulation().state();
                    bridge.advanceForCapture(4.0);
                    const auto after=bridge.simulation().state();
                    check(after.time_s>before.time_s && after.speed_mps>1,"the car drives on with instruments fitted");
                    check(bridge.measuredAvailable() && bridge.measuredAgeMs()>0 && bridge.measuredAgeMs()<110,
                          "the pose read is late by its dead time and up to a period, and no more");
                    check(bridge.measuredPositionError()>0.05 && bridge.measuredPositionError()<8,
                          "and out by metres, being both late and noisy, without being nonsense");
                    check(std::abs(bridge.measuredSpeedKmh()-after.speed_mps*3.6)>1e-9 &&
                          std::abs(bridge.measuredSpeedErrorKmh())<15,
                          "the speed read differs from the true speed by its own error");
                    check(item("measuredPose")->property("value").toString()==QString::number(std::lround(bridge.measuredAgeMs()))+
                              QString::fromUtf8(" ms old · ")+QString::number(bridge.measuredPositionError(),'f',2)+" m out",
                          "the card states the age and the distance the bridge measures");
                    // Beneath the four-wheel car's tire card, the tallest the telemetry gets, which scrolls rather than run under
                    // the dock.
                    check(!window->grabWindow().isNull() && endsAbove("insightsPanel",dockTop()),
                          "the telemetry, measurements included, ends above the dock in the compact window");
                    {
                        auto* card=qobject_cast<QQuickItem*>(item("sensorsCard"));
                        auto* tires=qobject_cast<QQuickItem*>(item("slipCard"));
                        auto* measured=qobject_cast<QQuickItem*>(item("measuredStateButton"));
                        auto* drawer=qobject_cast<QQuickItem*>(item("settingsDrawer"));
                        check(card && tires && tires->isVisible() &&
                              card->mapToScene(QPointF(0,0)).y()>=tires->mapToScene(QPointF(0,tires->height())).y(),
                              "the sensors card sits beneath the tire card, not over it");
                        check(measured && drawer &&
                              measured->mapToScene(QPointF(measured->width(),0)).x()<=drawer->mapToScene(QPointF(drawer->width(),0)).x()-8,
                              "the instruments choice fits inside the settings");
                    }
                    panels(false,true,false);
                    check(window->grabWindow().save(output+"/compact-measured.png"),"measured state capture");
                }
                // Simulated cone perception (decision 0030): the course's cones and what the newest frames made of them.
                check(!bridge.perceptionFitted() && !item("courseCones")->property("visible").toBool() &&
                      !item("perceptionReadout")->property("visible").toBool(),"without perception no cone is drawn");
                click("conesPerceivedButton");
                check(bridge.perceptionFitted() && item("conesPerceivedButton")->property("selected").toBool() &&
                      item("courseCones")->property("visible").toBool() && bridge.perceivedCones().size()==fd::make_cone_layout(bridge.track()).size(),
                      "perceiving the cones draws the course's own cones");
                check(!geometry("conesGeometry")->vertexData().isEmpty(),"the cones are in the scene");
                bridge.advanceForCapture(1.0);
                {
                    const auto view=bridge.perceptionView();
                    check(bridge.perceptionFrameAvailable() && !view.detections.empty() && bridge.perceptionFrameAgeMs()>=200-1e-6 &&
                          bridge.perceptionFrameAgeMs()<305,"a frame has arrived, a dead time and up to a period old");
                    check(item("simulatedDetections")->property("visible").toBool() && !geometry("detectionsGeometry")->vertexData().isEmpty(),
                          "its detections are drawn where the sensor put them");
                    bool near_cones=true;
                    for(const auto& d:view.detections) {
                        double nearest=1e9;
                        for(const auto& c:bridge.perceivedCones()) nearest=std::min(nearest,std::hypot(d.position.x-c.position.x,d.position.y-c.position.y));
                        near_cones=near_cones && nearest<0.5;
                    }
                    check(near_cones,"placed from the true pose, every detection stands within half a metre of a cone");
                    check(item("perceivedCounts")->property("value").toString()==QString::number(bridge.perceivedDetections())+" / "+
                              QString::number(bridge.perceivedMissed())+" / "+QString::number(bridge.perceivedWrongColour()),
                          "the card counts what the frame saw, missed and mis-coloured");
                    check(labelClear("perceivedCounts") && labelClear("perceptionAge") && labelClear("measuredPose") && labelClear("measuredSpeed"),
                          "each readout's value stands clear of its label");
                    check(item("sensorsCard")->property("visible").toBool() && !window->grabWindow().isNull() && endsAbove("insightsPanel",dockTop()),
                          "with instruments and cones the telemetry still ends above the dock");
                    check(window->grabWindow().save(output+"/compact-perception.png"),"perception capture");
                }
                // Driving on cones (decision 0031): the car drives on the path it believes from those detections, and the
                // view draws that belief beside the true track.
                panels(true,true,false);
                check(item("driveRow")->property("visible").toBool() && item("driveTrackButton")->property("selected").toBool() &&
                      !bridge.drivingOnCones() && !item("believedPath")->property("visible").toBool(),
                      "with perception the car is offered the cones to drive on, and drives on the track");
                click("driveConesButton");
                check(bridge.drivingOnCones() && bridge.simulation().cone_driver() && bridge.simulation().state().time_s==0 &&
                      item("driveConesButton")->property("selected").toBool() && item("believedPath")->property("visible").toBool() &&
                      bridge.beliefSource()=="MEASURED" && !bridge.believedPathAvailable() && bridge.track().width_m==5.0 &&
                      bridge.perceivedCones().size()==fd::make_cone_layout(bridge.track()).size(),
                      "driving on cones starts the run over on the course laid to 5 m, from the measured state, with no path believed yet");
                check(item("believedPathReadout")->property("value").toString()=="no path yet","which the card says");
                bridge.advanceForCapture(4.0);
                {
                    const auto view=bridge.believedView();
                    check(bridge.believedPathAvailable() && view.path.size()>10 && bridge.simulation().state().speed_mps>2,
                          "four seconds on, the car drives on a path it believes");
                    check(!geometry("believedGeometry")->vertexData().isEmpty(),"and that path is drawn");
                    check(bridge.believedPathAheadM()>5 && bridge.believedPathAheadM()<25 && bridge.believedPathAgeMs()>=200-1e-6 &&
                          bridge.believedPathAgeMs()<600,("a path from a frame a dead time or more old, running some metres ahead (got "+
                          std::to_string(bridge.believedPathAheadM())+" m ahead, "+std::to_string(bridge.believedPathAgeMs())+" ms old)").c_str());
                    check(bridge.beliefErrorM()>0 && bridge.beliefErrorM()<1.5,"the car believes itself near, not exactly, where it is");
                    check(labelClear("believedPathReadout") && labelClear("perceivedCounts") && !item("perceptionAge")->property("visible").toBool(),
                          "on cones the belief's readout stands clear of its label, and replaces the frame's age");
                    check(item("modeLabel")->property("text").toString().contains("ON CONES") &&
                          item("steeringCaption")->property("text").toString().contains(QString::fromUtf8("measured state · believed path")),
                          "the header and the steering caption say the car drives on cones from its measured state");
                    check(item("believedPathReadout")->property("label").toString()=="BELIEF" &&
                          item("believedPathReadout")->property("value").toString()==
                              QString::number(bridge.believedPathAheadM(),'f',0)+QString::fromUtf8(" m · ")+
                              QString::number(std::lround(bridge.believedPathAgeMs()))+QString::fromUtf8(" ms · ")+
                              QString::number(bridge.beliefErrorM(),'f',2)+" m off",
                          "the card states what the car believes and how far off it is");
                    panels(false,true,false);
                    check(item("sensorsCaption")->property("visible").toBool() && !window->grabWindow().isNull() && endsAbove("insightsPanel",dockTop()),
                          "driving on cones, the telemetry with the card and its caption still ends above the dock");
                    check(window->grabWindow().save(output+"/compact-cones.png"),"cone driving capture");
                }
                bridge.setPerception(false);
                check(bridge.perceptionFitted() && bridge.status().contains("drives on these cones"),
                      "perception cannot be taken off while the car drives on it");
                click("driveTrackButton");
                check(!bridge.drivingOnCones() && !bridge.simulation().cone_driver() && bridge.simulation().state().time_s==0 &&
                      !item("believedPath")->property("visible").toBool() && bridge.track().width_m==10.0,
                      "back on the track, the run starts over on the preset as it was");
                {
                    const QString cones_fixture=writeConesFixture(output);
                    check(bridge.loadRecording(cones_fixture),"a recorded run on cones loads");
                    bridge.seek(4);
                    const auto view=bridge.believedView();
                    panels(true,true,false);
                    check(bridge.drivingOnCones() && bridge.beliefSource()=="MEASURED" && bridge.believedPathAvailable() &&
                          item("believedPath")->property("visible").toBool() && !geometry("believedGeometry")->vertexData().isEmpty() &&
                          !item("driveRow")->property("visible").toBool(),
                          "replay draws the path the car believed, and offers no choice of what to drive on");
                    panels(false,true,false);
                    fd::Playback recorded_run(fd::load_recording(std::filesystem::path(cones_fixture.toStdU16String())));
                    recorded_run.seek_time(4);
                    const auto& r=recorded_run.recording();
                    const auto d=static_cast<std::size_t>(recorded_run.decision()-r.decisions.data());
                    check(r.beliefs[d].path && view.path.size()==r.believed_paths[*r.beliefs[d].path].path.points.size() &&
                          view.believed.x_m==r.beliefs[d].state.x_m,"the recorded belief of the decision in force");
                    click("exitReplayButton");
                }
                // The judge (decision 0032): cones knocked down or out cost two seconds each, shown by the cursor.
                {
                    check(item("penaltyMetric")->property("text").toString()=="none" && item("lastLapMetric")->property("text").toString()=="-" &&
                          !item("knockedCones")->property("visible").toBool(),"a clean run so far has no penalty and no lap timed");
                    const QString knocked_fixture=writeKnockedFixture(output);
                    check(bridge.loadRecording(knocked_fixture),"a recorded run through cones left in the lane loads");
                    bridge.seek(2);
                    check(bridge.conesHit()==0 && item("penaltyMetric")->property("text").toString()=="none" &&
                          !item("knockedCones")->property("visible").toBool(),"before the cones, nothing is knocked");
                    bridge.seek(10);
                    const auto view=bridge.timingView();
                    check(bridge.conesHit()==3 && bridge.penaltySeconds()==6 && view.hit.size()==3 &&
                          view.hit.front()==bridge.courseCones().size()-3,"the three cones in the lane are down, two seconds each");
                    check(item("penaltyMetric")->property("text").toString()=="+6 s" &&
                          item("penaltyDetail")->property("text").toString()==QString::fromUtf8("3 cones · 0 off") &&
                          item("knockedCones")->property("visible").toBool() && !geometry("knockedGeometry")->vertexData().isEmpty(),
                          "the metrics say so and the knocked cones are drawn");
                    {
                        // Either side of the circuit map, clear of each other and of the map between them.
                        auto* map=qobject_cast<QQuickItem*>(item("circuitOverview"));
                        auto* lap=qobject_cast<QQuickItem*>(item("lastLapReadout"));
                        auto* penalties=qobject_cast<QQuickItem*>(item("penaltyReadout"));
                        check(map && lap && penalties && lap->x()+lap->width()+60<penalties->x() &&
                              penalties->y()>=0 && penalties->y()+penalties->height()<=map->height()+1,
                              "the judge's readouts sit either side of the map, inside it");
                    }
                    check(window->grabWindow().save(output+"/compact-knocked.png"),"knocked cones capture");
                    click("exitReplayButton");
                }
                // A Formula Student layout in PacSim's format (decision 0032): an oval 60 m by 30 m, 4 m wide, with a gate.
                {
                    const QString oval=writeOvalCourse(output);
                    check(bridge.loadCourse(oval),"a PacSim layout loads as the course");
                    check(bridge.courseName()=="fd-oval" && bridge.simulation().cones().size()==96 &&
                          bridge.simulation().course_source()=="PacSim layout fd-oval" &&
                          std::abs(bridge.simulation().config().wheelbase_m-1.53)<1e-12 && !bridge.racingLineReady() && bridge.comparisonLine().empty(),
                          "its cones and gate, a Formula Student car's geometry, and no racing line from the preset");
                    check(item("lineSummary")->property("text").toString().startsWith("Course fd-oval"),"the line caption names the course");
                    check(!bridge.setLineMode(1) && bridge.status().contains("solved for the preset"),"the preset's racing line is refused on it");
                    bridge.advanceForCapture(25.0);
                    check(bridge.simulation().judge().laps()>=1 && bridge.lastLapSeconds()>0 &&
                          item("lastLapMetric")->property("text").toString()==QString::number(bridge.lastLapSeconds(),'f',2)+" s",
                          "a lap of it is timed from its own start line");
                    check(window->grabWindow().save(output+"/compact-course.png"),"course capture");
                    check(bridge.loadPreset() && bridge.courseName().isEmpty() && bridge.simulation().config().wheelbase_m==2.6 &&
                          bridge.track().width_m==10.0 && bridge.simulation().course_source()==fd::Simulation::laid_along_the_track,
                          "back to the preset and the configuration the desktop started with");
                }
                // The theoretical best lap (decision 0034): a steady lap written as a solver would for the Formula One style
                // car on the preset, shown while it is this car's and corridor's, and hidden with the reason once it is not.
                {
                    check(bridge.loadProfile("formula-one-style") && bridge.profileName().startsWith("Formula One style") &&
                          bridge.vehicleModel()==3 && std::abs(bridge.simulation().config().wheelbase_m-3.4)<1e-12,
                          "a physics profile builds the 4 wheels car with its geometry");
                    check(bridge.loadOptimalLap(writeOptimalFixture(output,bridge)) && bridge.optimalLoaded() && bridge.optimalMatches() &&
                          bridge.optimalConverged() &&
                          std::abs(bridge.optimalLapSeconds()-fd::condition_track(bridge.simulation().track()).track.length_m/20)<1e-9,
                          "an optimum solved for this car and corridor loads and is shown as its optimum");
                    check(item("optimalCard")->property("visible").toBool() && item("optimalGhost")->property("visible").toBool() &&
                          item("optimalSourceText")->property("text").toString().startsWith("fastest-lap 0.5") &&
                          item("optimalStatusChip")->property("text").toString()=="SOLVED" && !item("optimalMismatchText")->property("visible").toBool(),
                          "the card and the translucent car show, with the solver and status they are for");
                    const double start_x=bridge.ghostX(), start_y=bridge.ghostY();
                    bridge.advanceForCapture(12.0);
                    check(std::hypot(bridge.ghostX()-start_x,bridge.ghostY()-start_y)>5 && bridge.optimalDeltaAvailable() &&
                          std::isfinite(bridge.optimalDeltaSeconds()) && bridge.optimalFromRest(),
                          "the optimum drives on from the line with the live lap, which is compared with it from rest");
                    const auto sectors=bridge.sectorDeltas();
                    check(sectors.size()==3 && sectors[0].toMap()["timed"].toBool() &&
                          std::abs(sectors[0].toMap()["delta"].toDouble()-(sectors[0].toMap()["live"].toDouble()-sectors[0].toMap()["optimal"].toDouble()))<1e-9,
                          "the first sector is timed against the optimum's");
                    const auto energy=bridge.ghostTireEnergy();
                    check(energy.size()==4 && energy[0].toMap()["used"].toDouble()>0 &&
                          energy[0].toMap()["used"].toDouble()<energy[0].toMap()["total"].toDouble(),
                          "each tire's energy so far on the optimum's lap is part of its whole lap");
                    check(bridge.optimalSensitivities().size()==2 &&
                          item("sensitivity0")->property("visible").toBool(),"the setup sensitivities are listed");
                    {
                        check(!window->grabWindow().isNull(),"the telemetry renders");
                        auto* card=qobject_cast<QQuickItem*>(item("optimalCard"));
                        auto* gg=qobject_cast<QQuickItem*>(item("ggCard"));
                        check(card && gg && card->mapToScene(QPointF(0,0)).y()>=gg->mapToScene(QPointF(0,gg->height())).y() &&
                              endsAbove("insightsPanel",dockTop()),
                              "the card sits beneath the G-G card in the telemetry, which ends above the dock in the compact window");
                    }
                    panels(false,true,true);
                    check(item("ghostLegend")->property("visible").toBool(),"the translucent car is explained");
                    panels(false,true,false);
                    check(window->grabWindow().save(output+"/compact-optimal.png"),"theoretical lap capture");
                    check(bridge.setSetupValue("mass_kg",700) && !bridge.optimalMatches() && !item("optimalGhost")->property("visible").toBool() &&
                          item("optimalMismatchText")->property("visible").toBool() && bridge.optimalMismatch().contains("setup") &&
                          bridge.profileName().isEmpty(),
                          "another setup is another car: the optimum hides and says why");
                    check(bridge.resetSetup() && bridge.optimalMatches() && bridge.profileName().startsWith("Formula One style"),
                          "the profile's own car is its optimum's again");
                }
                click("conesPerceivedButton");
                check(!bridge.perceptionFitted() && !item("courseCones")->property("visible").toBool(),"taking perception off hides the cones");
                // Changing the car keeps the instruments, as it keeps the scenario.
                click("dynamicModelButton");
                check(bridge.sensorsFitted() && !bridge.measuredAvailable(),
                      "a fresh run keeps its instruments and starts measuring again");
                click("measuredStateButton");
                check(!bridge.sensorsFitted() && !bridge.measuredAvailable() && !item("measuredCard")->property("visible").toBool(),
                      "taking the instruments off returns the car to its true state alone");
                click("fourWheelModelButton");
                click("kinematicModelButton");
                check(bridge.vehicleModel()==0 && !item("wheelGrid")->property("visible").toBool() && bridge.wheelStates().isEmpty() &&
                      bridge.wheelLoads().isEmpty() && bridge.staticWheelLoads().isEmpty() && bridge.wheelFrictionUse().isEmpty() &&
                      !bridge.aerodynamicsModeled() && bridge.dragNewtons()==0 && !item("aeroMetric")->property("visible").toBool(),
                      "the kinematic bicycle has no wheel states, loads, friction circles or aerodynamics");
                check(bridge.setupControls().isEmpty() && !item("setupButton")->property("visible").toBool() && !bridge.setSetupValue("mass_kg",900),
                      "the kinematic bicycle offers no setup controls");
                check(!bridge.envelopeModeled() && !bridge.envelopeReady() && bridge.envelopeBoundary().isEmpty() && !item("ggCard")->property("visible").toBool(),
                      "the kinematic bicycle has no derived envelope");
                check(bridge.speedPlanMode()==0 && !item("envelopePlanButton")->property("enabled").toBool() && !bridge.setSpeedPlanMode(1) &&
                      bridge.status().contains("4 wheels"),"the kinematic bicycle has no envelope to plan with, and says why");
                // A person drives next (decision 0037), from the line, the controller's controls given here.
                click("resetButton");
                panels(true,false,false);
                click("humanDriverButton");
                check(bridge.humanDriving() && bridge.simulation().controller_outcome().driven_by==fd::ControllerMode::human,
                      "You takes the car: the decision in force says a person drives it");
                check(item("driverName")->property("text").toString()=="You drive" && !item("planSignRow")->property("visible").toBool() &&
                      !item("whyCard")->property("visible").toBool() && item("driverInputs")->property("visible").toBool() &&
                      item("gamepadNote")->property("visible").toBool(),
                      "the display says who drives and shows the controls in place of the plan's sign and reasons");
                check(item("toastText")->property("text").toString()=="Connect an Xbox controller to drive" && bridge.status()=="Ready / you drive",
                      "without a controller the status line asks for one");
                check(!bridge.setDriverControls(1.5,0,0) && bridge.status().contains("throttle") && bridge.driverThrottle()==0,
                      "controls outside their travel are refused, with the reason, and change nothing");
                check(bridge.setDriverControls(1,0,0) && bridge.driverThrottle()==1 &&
                      item("throttleInputBar")->property("width").toDouble()>60 && item("brakeInputBar")->property("width").toDouble()==0,
                      "the controller's controls reach the car and show floored");
                panels(false,false,false);
                emit bridge.acceleratorAtLine();
                check(bridge.running(),"the accelerator at the line starts the drive");
                break;
            }
            case 28:case 29:case 30:case 31: break;
            case 32:
            {
                const double asked=bridge.simulation().config().grip_mu*fd::gravity_mps2;
                check(bridge.running() && bridge.speedKmh()>5 && std::abs(bridge.simulation().diagnostics().applied.acceleration_mps2-asked)<1e-9,
                      "a floored accelerator drives the car at the grip it asks for");
                check(bridge.simulation().controller_outcome().driven_by==fd::ControllerMode::human && bridge.status()=="Running / you drive",
                      "every decision says the person drove it");
                check(bridge.predictionAvailable() && item("plannedTrajectory")->property("visible").toBool(),
                      "the planner's action stays on the road as a guide, driving nothing");
                check(bridge.setDriverControls(0,1,0.5) && item("brakeInputBar")->property("width").toDouble()>60 &&
                      item("steeringInputDot")->property("x").toDouble()<(84-12)/2.0,"the brake and a left stick show as given");
                context->paused_time=bridge.speedKmh();break;
            }
            case 33:case 34: break;
            case 35:
                check(bridge.speedKmh()<context->paused_time,"the brake slows the car");
                check(window->grabWindow().save(output+"/compact-human-driving.png"),"human driving capture");
                emit bridge.menuPressed();
                check(!bridge.running(),"the Menu button pauses the drive");
                // Gokart mode (decision 0038), offered under You: the Göteborg track and its rental kart.
                {
                    context->before_kart_length=bridge.trackLength();
                    context->before_kart_wheelbase=bridge.wheelbase();
                    panels(true,false,false);
                    auto* kart=item("kartModeSwitch");
                    check(item("kartRow")->property("visible").toBool() && !bridge.kartMode(),"gokart mode is offered to a person who drives");
                    kart->setProperty("checked",true);
                    check(QMetaObject::invokeMethod(kart,"toggled") && bridge.kartMode(),"the switch starts gokart mode");
                    const auto* car=std::get_if<fd::FourWheelCar>(&bridge.simulation().vehicle_model());
                    check(car && std::abs(car->max_drive_speed_mps-60/3.6)<1e-9 && car->mass_kg==261 && std::abs(bridge.wheelbase()-1.045)<1e-9 &&
                          std::abs(bridge.trackLength()-400)<2 && bridge.simulation().cones().empty() && bridge.simulation().human_driving(),
                          "the Göteborg track, 400 m and judged without cones, with the rental kart, and the person still driving");
                    check(item("modeLabel")->property("text").toString().startsWith("GOKART") && item("kartNote")->property("visible").toBool() &&
                          item("driverName")->property("text").toString()=="You drive","the display names the gokart track and who drives");
                    const auto entry=liveCars->property("cars").toList().value(bridge.carAppearance()).toMap();
                    const double model_wheelbase=entry.value("available").toBool() ? entry.value("wheelbase").toDouble() : 2.6;
                    check(std::abs(item("simulatedCar")->property("drawnScale").toDouble()-1.045/model_wheelbase)<1e-6 &&
                          item("carReflection")->property("drawnScale").toDouble()==item("simulatedCar")->property("drawnScale").toDouble(),
                          "the car and its reflection are drawn at the kart's wheelbase");
                    check(!bridge.setVehicleModel(0) && bridge.status().contains("Gokart mode") && !item("kinematicModelButton")->property("enabled").toBool(),
                          "the kart stays the car while the mode is on");
                    check(item("kartCoachingLine")->property("visible").toBool() &&
                          !item("plannedTrajectory")->property("visible").toBool() && !item("latticeAhead")->property("visible").toBool(),
                          "kart defaults to the full-lap coaching line with prediction and lattice hidden");
                    const auto guide=geometry("kartCoachingGeometry")->vertexData();
                    constexpr qsizetype stride=7*sizeof(float);
                    check(guide.size()==1600*6*stride,"the coaching ribbon covers all 1600 poster samples");
                    bool inside=true,on_ground=true,red=false,green=false,blue=false;
                    for(qsizetype k=0;k<guide.size();k+=stride) {
                        float v[7];std::memcpy(v,guide.constData()+k,sizeof(v));
                        const auto at=fd::project(bridge.track(),{v[0],-v[2]});
                        inside=inside && fd::within_corridor(bridge.track(),at,0.15);
                        on_ground=on_ground && std::abs(v[1]-0.04)<1e-6;
                        red=red || (v[3]>v[4] && v[3]>v[5]);
                        green=green || (v[4]>v[3] && v[4]>v[5]);
                        blue=blue || (v[5]>v[3] && v[5]>v[4]);
                    }
                    check(inside && on_ground,"every coaching ribbon vertex is on the ground inside the actual conditioned corridor");
                    check(red && green && blue,"the poster's gas, coast and brake zones all reach the geometry");
                    check(std::memcmp(guide.constData(),guide.constData()+guide.size()-stride,3*sizeof(float))==0 &&
                          std::memcmp(guide.constData()+stride,guide.constData()+guide.size()-2*stride,3*sizeof(float))==0,
                          "the ground line closes at the start with matching edges");
                    auto toggle=[&](const char* name,bool value) {
                        item(name)->setProperty("checked",value);
                        check(QMetaObject::invokeMethod(item(name),"toggled"),"kart guide switch is operable");
                    };
                    const double at_time=bridge.simulationTime(),at_x=bridge.x(),at_y=bridge.y();
                    toggle("kartPredictionSwitch",true);
                    check(item("plannedTrajectory")->property("visible").toBool() && item("kartCoachingLine")->property("visible").toBool(),
                          "both coaching and prediction can be shown");
                    toggle("kartGuideSwitch",false);
                    check(!item("kartCoachingLine")->property("visible").toBool() && item("plannedTrajectory")->property("visible").toBool(),
                          "prediction alone can be shown");
                    toggle("kartPredictionSwitch",false);
                    check(!item("kartCoachingLine")->property("visible").toBool() && !item("plannedTrajectory")->property("visible").toBool(),
                          "both lines can be hidden for unaided driving");
                    toggle("kartGuideSwitch",true);
                    check(bridge.simulationTime()==at_time && bridge.x()==at_x && bridge.y()==at_y &&
                          geometry("kartCoachingGeometry")->vertexData()==guide,
                          "visibility changes leave the plant and the fixed coaching geometry alone");
                    check(window->grabWindow().save(output+"/compact-kart-guidance-settings.png"),"kart guidance settings capture");
                    bridge.setOverview(true);
                    panels(false,false,true);
                    check(window->grabWindow().save(output+"/compact-kart-guidance-overview.png"),"full-lap poster guidance capture");
                    bridge.setOverview(false);
                    panels(false,false,false);
                    check(item("practiceClock")->property("visible").toBool() && bridge.practiceLapSeconds()==0 && bridge.previousPracticeLap()==0,
                          "the kart lap clock waits at zero with no invented previous lap");
                    const auto& start=bridge.simulation().track().points.front();
                    check(std::hypot(bridge.x()-start.x_m,bridge.y()-start.y_m)<1e-9 &&
                          !geometry("kartStartGeometry")->vertexData().isEmpty(),"the kart starts on its marked poster start line");
                    check(bridge.setDriverControls(1,0,0),"the accelerator floored");
                    emit bridge.acceleratorAtLine();
                    check(bridge.running(),"and the kart is away from the line");
                }
                break;
            case 36:case 37:case 38:case 39:case 40: break;
            case 41:
                check(bridge.practiceLapSeconds()>0 && bridge.previousPracticeLap()==0 && bridge.practiceLaps()==0,
                      "gas starts the current lap timer but a partial lap never becomes Previous");
                check(bridge.running() && bridge.speedKmh()>5 && bridge.speedKmh()<=60.0+1e-6,"the kart accelerates, never past its 60 km/h");
                check(!item("velocityVector")->property("visible").toBool(),"the kart's motion arrow starts hidden even at speed");
                check(QMetaObject::invokeMethod(item("motionArrowLegend"),"toggled") && item("velocityVector")->property("visible").toBool(),
                      "the motion arrow has its own switch while driving");
                check(QMetaObject::invokeMethod(item("motionArrowLegend"),"toggled") && !item("velocityVector")->property("visible").toBool(),
                      "the motion arrow can be hidden again without hiding coaching");
                check(QMetaObject::invokeMethod(item("predictionLegend"),"toggled") && item("plannedTrajectory")->property("visible").toBool(),
                      "prediction can be enabled from Layers while driving");
                check(QMetaObject::invokeMethod(item("kartGuideLegend"),"toggled") && !item("kartCoachingLine")->property("visible").toBool() && bridge.running(),
                      "coaching can be hidden from Layers without pausing");
                check(QMetaObject::invokeMethod(item("predictionLegend"),"toggled") && QMetaObject::invokeMethod(item("kartGuideLegend"),"toggled") &&
                      !item("plannedTrajectory")->property("visible").toBool() && item("kartCoachingLine")->property("visible").toBool(),
                      "Layers restores coaching alone while driving");
                check(window->grabWindow().save(output+"/compact-kart.png"),"gokart capture");
                {
                    panels(true,false,false);
                    const double at_time=bridge.simulationTime(),at_x=bridge.x();
                    const int before_revision=bridge.revision();
                    auto* forgiveness=item("forgivenessSwitch");
                    check(item("forgivenessRow")->property("visible").toBool() && !bridge.forgivenessEnabled() && bridge.forgivenessLevel()==100,
                          "forgiveness is offered under You and starts off with 100 ready");
                    forgiveness->setProperty("checked",true);
                    check(QMetaObject::invokeMethod(forgiveness,"toggled") && bridge.forgivenessEnabled(),"the actual switch enables forgiveness");
                    check(bridge.simulationTime()==at_time && bridge.x()==at_x && bridge.revision()==before_revision && !bridge.forgivenessActive(),
                          "changing assistance queues a change without moving the car or changing the current sample");
                    bridge.advanceForCapture(bridge.simulation().config().fixed_dt_s);
                    check(bridge.forgivenessActive() && bridge.revision()==before_revision+1 && bridge.running(),
                          "forgiveness starts at the next fixed tick without pausing the drive");
                    check(item("driverName")->property("text").toString().contains("Assist 100") && !item("slipCard")->property("visible").toBool() &&
                          !window->property("setupEditable").toBool(),"assisted driving is labelled and the ordinary tire limits and setup controls do not claim to apply");
                    check(!bridge.setForgiveness(true,std::numeric_limits<double>::quiet_NaN()) && bridge.forgivenessLevel()==100,
                          "a malformed slider request leaves the last choice intact");
                    auto* slider=item("forgivenessSlider");
                    slider->setProperty("value",50);
                    check(QMetaObject::invokeMethod(slider,"moved") && bridge.forgivenessLevel()==50,"the slider moves down to 50");
                    bridge.advanceForCapture(bridge.simulation().config().fixed_dt_s);
                    check(bridge.simulation().driving_assistance().level==50 && bridge.forgivenessActive(),"50 reaches the plant");
                    slider->setProperty("value",1);
                    check(QMetaObject::invokeMethod(slider,"moved") && bridge.forgivenessLevel()==1,"the slider reaches realistic handling at 1");
                    bridge.advanceForCapture(bridge.simulation().config().fixed_dt_s);
                    check(!bridge.forgivenessActive() && bridge.forgivenessEnabled(),"1 removes the handling override while keeping the slider available");
                    slider->setProperty("value",100);
                    check(QMetaObject::invokeMethod(slider,"moved"),"the slider can return to 100");
                    bridge.advanceForCapture(bridge.simulation().config().fixed_dt_s);
                    check(QMetaObject::invokeMethod(item("forgivenessManualButton"),"clicked") && bridge.forgivenessManual(),
                          "Manual selects independent forgiveness controls");
                    const double tuning_time=bridge.simulationTime(),tuning_x=bridge.x();
                    const auto tune=[&](const char* name,double value) {
                        auto* control=item(name); control->setProperty("value",value);
                        check(QMetaObject::invokeMethod(control,"moved"),"manual slider signal is connected");
                    };
                    tune("manualaccelerationSlider",100); tune("manualbrakingSlider",10); tune("manualsteeringSlider",20);
                    tune("manualgripSlider",100); tune("manualspeedSlider",70);
                    const auto tuning=bridge.forgivenessTuning();
                    check(tuning["acceleration"].toDouble()==100 && tuning["braking"].toDouble()==10 &&
                          tuning["steering"].toDouble()==20 && tuning["grip"].toDouble()==100 && tuning["speed"].toDouble()==70,
                          "high acceleration can coexist with low braking and steering and independently chosen grip and speed");
                    check(bridge.simulationTime()==tuning_time && bridge.x()==tuning_x,"manual tuning never resets or advances the kart");
                    check(!bridge.setForgivenessManual(true,101,10,20,100,70) && bridge.forgivenessTuning()==tuning,
                          "invalid manual edits preserve the queued valid settings");
                    bridge.advanceForCapture(bridge.simulation().config().fixed_dt_s);
                    check(bridge.simulation().driving_assistance().manual && bridge.simulation().driving_assistance().braking==10 &&
                          item("driverName")->property("text").toString().contains("Manual assist"),"manual tuning reaches the plant and is labelled");
                    check(QMetaObject::invokeMethod(item("forgivenessOverallButton"),"clicked") && !bridge.forgivenessManual() &&
                          bridge.forgivenessLevel()==100 && bridge.forgivenessTuning()==tuning,"Overall restores the master value and remembers manual settings");
                    check(QMetaObject::invokeMethod(item("forgivenessManualButton"),"clicked"),"manual tuning can be recalled");
                    bridge.advanceForCapture(bridge.simulation().config().fixed_dt_s);
                    check(bridge.forgivenessPresets().size()==5 && !bridge.forgivenessPresets()[0].toBool(),"five isolated empty local slots");
                    check(QMetaObject::invokeMethod(item("saveForgivenessPreset"),"clicked") && bridge.forgivenessPresets()[0].toBool(),
                          "Save stores the selected preset through the actual button");
                    const double saved_time=bridge.simulationTime(),saved_x=bridge.x();
                    tune("manualaccelerationSlider",34);
                    check(QMetaObject::invokeMethod(item("loadForgivenessPreset"),"clicked") && bridge.forgivenessTuning()==tuning && bridge.forgivenessManual(),
                          "Load restores the complete manual preset");
                    check(bridge.simulationTime()==saved_time && bridge.x()==saved_x,"save and load never reset or move the kart");
                    bridge.advanceForCapture(bridge.simulation().config().fixed_dt_s);
                    check(bridge.simulation().driving_assistance().acceleration==100,"loaded preset reaches the plant at the next tick");
                    check(QMetaObject::invokeMethod(item("forgivenessPresetSlot4"),"clicked") &&
                          QMetaObject::invokeMethod(item("saveForgivenessPreset"),"clicked") && bridge.forgivenessPresets()[4].toBool(),"slot five can be saved");
                    check(QMetaObject::invokeMethod(item("deleteForgivenessPreset"),"clicked") && !bridge.forgivenessPresets()[4].toBool() &&
                          bridge.forgivenessPresets()[0].toBool(),"Delete removes only its selected slot");
                    check(QMetaObject::invokeMethod(item("forgivenessPresetSlot0"),"clicked"),"return to saved slot one");
                    QTemporaryDir restartDir;
                    check(restartDir.isValid(),"isolated restart storage");
                    const auto restartPath=restartDir.filePath("presets.json");
                    {
                        Bridge first(nullptr,restartPath);
                        check(first.setHumanDriving(true) && first.setKartMode(true) && first.setForgiveness(true,72) &&
                              first.setForgivenessManual(true,34,14,40,100,79) && first.saveForgivenessPreset(0),"save manual setup before closing");
                        check(first.setForgivenessManual(false,1,2,3,4,5) && first.loadForgivenessPreset(0),"load saved manual setup before closing");
                    }
                    {
                        Bridge reopened(nullptr,restartPath);
                        check(!reopened.forgivenessActive(),"reopening does not apply kart assistance to ordinary car");
                        check(reopened.setKartMode(true) && reopened.setHumanDriving(true) && reopened.forgivenessEnabled() &&
                              reopened.forgivenessManual() && reopened.forgivenessLevel()==72 &&
                              reopened.forgivenessTuning()["acceleration"].toDouble()==34 &&
                              reopened.forgivenessTuning()["braking"].toDouble()==14 &&
                              reopened.forgivenessTuning()["steering"].toDouble()==40 &&
                              reopened.forgivenessTuning()["grip"].toDouble()==100 &&
                              reopened.forgivenessTuning()["speed"].toDouble()==79,"restart restores all saved manual values on entering kart");
                        check(reopened.simulationTime()==0,"restoration never advances the plant");
                        reopened.advanceForCapture(reopened.simulation().config().fixed_dt_s);
                        check(reopened.simulation().driving_assistance().acceleration==34,"restored setup applies at the next tick");
                    }
                }
                break;
            case 42:case 43: break; // Let the switch animation settle before its evidence capture.
            case 44:
                {
                    check(window->grabWindow().save(output+"/compact-kart-lap-clock.png"),"kart current and previous lap clock capture");
                    check(window->grabWindow().save(output+"/compact-kart-manual-assistance.png"),"manual forgiveness controls capture");
                    check(window->grabWindow().save(output+"/compact-kart-presets.png"),"local presets capture");
                    check(window->grabWindow().save(output+"/compact-kart-forgiveness.png"),"forgiveness settings capture");
                    check(sound && sound->driving() && item("raceSoundPanel")->property("visible").toBool(),"audio follows live human driving");
                    const auto time=bridge.simulationTime(), x=bridge.x(); const int revision=bridge.revision();
                    auto* volume=item("raceVolumeSlider"); volume->setProperty("value",61);
                    check(QMetaObject::invokeMethod(volume,"moved") && sound->volume()==61,"actual audio volume slider");
                    for(const auto* name:{"raceSoundSwitch","raceEngineSwitch","raceCuesSwitch"}) {
                        auto* toggle=item(name); toggle->setProperty("checked",false);
                        check(QMetaObject::invokeMethod(toggle,"toggled"),"actual sound toggle off");
                    }
                    check(!sound->enabled() && !sound->engine() && !sound->cues(),"master and independent audio channels mute");
                    for(const auto* name:{"raceSoundSwitch","raceEngineSwitch","raceCuesSwitch"}) {
                        auto* toggle=item(name); toggle->setProperty("checked",true);
                        check(QMetaObject::invokeMethod(toggle,"toggled"),"actual sound toggle on");
                    }
                    check(QMetaObject::invokeMethod(item("previewLapSound"),"clicked"),"lap chime preview handler");
                    check(bridge.simulationTime()==time && bridge.x()==x && bridge.revision()==revision,"audio controls leave plant and lap clock untouched");
                    sound->setVolume(45);
                    item("settingsFlick")->setProperty("contentY",item("raceSoundPanel")->property("y"));
                }
                break;
            case 45:
                {
                    const auto time=bridge.simulationTime(), x=bridge.x(), y=bridge.y();
                    const int revision=bridge.revision();
                    check(QMetaObject::invokeMethod(item("firstPersonViewButton"),"clicked") && bridge.cameraMode()==2 && !bridge.overview(),
                          "FP button selects first person independently of overview");
                    check(!item("simulatedCar")->property("visible").toBool() && !item("reflectionView")->property("visible").toBool(),
                          "first person hides exterior shell and reflection to keep driver sightline clear");
                    check(item("drivingView")->property("camera").value<QObject*>()==item("firstPersonCamera"),"FP uses the driver eye camera");
                    check(bridge.simulationTime()==time && bridge.x()==x && bridge.y()==y && bridge.revision()==revision,
                          "changing camera does not move or revise the plant");
                    bridge.setCameraMode(9);
                    check(bridge.cameraMode()==2,"invalid camera choice preserves FP");
                    window->setProperty("settingsOpen",false);
                }
                break;
            case 46:
                {
                    check(window->grabWindow().save(output+"/compact-kart-first-person.png"),"first person render capture");
                    check(QMetaObject::invokeMethod(item("overviewButton"),"clicked") && bridge.overview() && bridge.cameraMode()==1,
                          "Overview can be selected from FP");
                    check(QMetaObject::invokeMethod(item("followViewButton"),"clicked") && bridge.cameraMode()==0 &&
                          item("simulatedCar")->property("visible").toBool(),"Follow restores the exterior car");
                    window->setProperty("settingsOpen",true);
                    check(window->grabWindow().save(output+"/compact-kart-sound.png"),"driving sound controls capture");
                    auto* forgiveness=item("forgivenessSwitch");
                    forgiveness->setProperty("checked",false);
                    check(QMetaObject::invokeMethod(forgiveness,"toggled") && !bridge.forgivenessEnabled(),"forgiveness can be switched off while driving");
                    bridge.advanceForCapture(bridge.simulation().config().fixed_dt_s);
                    check(!bridge.forgivenessActive() && bridge.running() && bridge.forgivenessLevel()==100,"off restores normal handling and remembers the slider value");
                    check(bridge.setForgiveness(true,80),"assistance is enabled again before leaving the kart");
                    bridge.advanceForCapture(bridge.simulation().config().fixed_dt_s);
                }
                emit bridge.menuPressed();
                check(!sound->driving(),"pause silences the driving audio layer");
                {
                    const double stopped=bridge.practiceLapSeconds();
                    check(!bridge.running() && stopped>0 && bridge.practiceLapSeconds()==stopped,"pausing holds the practice clock");
                    bridge.reset();
                    const auto& start=bridge.simulation().track().points.front();
                    check(bridge.practiceLapSeconds()==0 && std::hypot(bridge.x()-start.x_m,bridge.y()-start.y_m)<1e-9,
                          "restart returns to the designated start and rearms the gas-started clock");
                }
                check(!bridge.running() && bridge.setKartMode(false) && !bridge.kartMode() &&
                      std::abs(bridge.trackLength()-context->before_kart_length)<1e-9 && bridge.wheelbase()==context->before_kart_wheelbase &&
                      bridge.vehicleModel()==0 && bridge.simulation().human_driving() && item("simulatedCar")->property("drawnScale").toDouble()==1,
                      "leaving gokart mode returns the track and car it came from, at their own size");
                check(!bridge.forgivenessEnabled() && !bridge.forgivenessActive() && !bridge.setForgiveness(true,100),
                      "leaving kart removes the override and refuses it on the ordinary car");
                check(!item("kartCoachingLine")->property("visible").toBool() && geometry("kartCoachingGeometry")->vertexData().isEmpty() &&
                      item("plannedTrajectory")->property("visible").toBool(),
                      "leaving kart removes its guide and restores the ordinary prediction preference");
                check(bridge.setHumanDriving(false) && !bridge.humanDriving() &&
                      bridge.simulation().controller_outcome().driven_by==fd::ControllerMode::policy &&
                      item("planSignRow")->property("visible").toBool() && item("whyCard")->property("visible").toBool() &&
                      !item("driverInputs")->property("visible").toBool(),
                      "handed back, the autonomous driver's decision drives and its display returns");
                check(warnings.isEmpty(),"QML emitted runtime warnings");
                check(!sound->driving() && !item("raceSoundPanel")->property("visible").toBool(),"handing back the car silences and hides human audio");
                finish({});break;
            default:throw std::runtime_error("UI verification state error");
            }
        } catch(const std::exception& e) {finish(QString::fromUtf8(e.what()));}
    });
    timer->start(120);
}
