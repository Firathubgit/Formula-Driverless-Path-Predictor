#include "bridge.hpp"
#include "race_audio.hpp"
#include "live_cars.hpp"
#include "showroom.hpp"
#include "track_geometry.hpp"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickWindow>
#include <QQuickStyle>
#include <QFileInfo>
#include <QDir>
#include <QImage>
#include <QTimer>
#include <QTemporaryDir>
#include <QFile>
#include <QTextStream>
#include <QQmlError>
#include <cmath>
#include <algorithm>

void scheduleUiVerification(QGuiApplication&, QQmlApplicationEngine&, Bridge&, const QString&, QStringList&, bool&);
void scheduleShowroomVerification(QGuiApplication&, QQmlApplicationEngine&, Bridge&, Showroom&, const QString&, QStringList&, bool&);

int main(int argc,char** argv) {
    QGuiApplication app(argc,argv);
    app.setApplicationName("Formula Driverless");
    app.setOrganizationName("Formula Driverless");
    QQuickStyle::setStyle("Basic");
    qmlRegisterType<TrackGeometry>("Formula",1,0,"TrackGeometry");
    const auto args=app.arguments();
    QTemporaryDir testPresets;
    Bridge bridge(nullptr,args.contains("--verify-ui") ? testPresets.filePath("presets.json") : QString{});
    // A Formula Student layout in PacSim's format instead of the preset (decision 0032).
    const int course=args.indexOf("--course");
    if(course>=0) {
        if(course+1>=args.size() || !bridge.loadCourse(args[course+1])) {
            QTextStream(stderr) << "fd_desktop: " << (course+1<args.size()?bridge.status():QString("--course needs a PacSim track file")) << '\n';
            return 5;
        }
    }
    // A documented physics profile for the 4 wheels car, as fd_headless --profile builds it (decision 0033); the car's
    // appearance and the showroom are untouched.
    const int profile=args.indexOf("--profile");
    if(profile>=0) {
        if(profile+1>=args.size() || !bridge.loadProfile(args[profile+1])) {
            QTextStream(stderr) << "fd_desktop: " << (profile+1<args.size()?bridge.status():QString("--profile needs a profile id")) << '\n';
            return 5;
        }
    }
    // The theoretical best lap of a reference-optimal artefact (decision 0034), shown while it is this car's and corridor's.
    const int optimal=args.indexOf("--optimal-lap");
    if(optimal>=0) {
        if(optimal+1>=args.size() || !bridge.loadOptimalLap(args[optimal+1])) {
            QTextStream(stderr) << "fd_desktop: " << (optimal+1<args.size()?bridge.status():QString("--optimal-lap needs an artefact directory")) << '\n';
            return 5;
        }
    }
    const int replay=args.indexOf("--replay");
    if(replay>=0) {
        if(replay+1>=args.size() || !bridge.loadRecording(args[replay+1])) {
            QTextStream(stderr) << "fd_desktop: " << (replay+1<args.size()?bridge.status():QString("--replay needs a run directory")) << '\n';
            return 5;
        }
    }
    // Same scenario syntax as the headless runner: FROM_S:TO_S:FROM_OFFSET:TO_OFFSET[:NAME].
    for(int i=0;i+1<args.size();++i) {
        if(args[i]!="--obstruct")continue;
        const auto parts=args[i+1].split(':');
        bool okay=parts.size()>=4 && parts.size()<=5;
        fd::Obstruction blockage;
        double values[4]{};
        for(int k=0;okay && k<4;++k) values[k]=parts[k].toDouble(&okay);
        if(okay) {
            blockage={values[0],values[1],values[2],values[3],
                      parts.size()==5?parts[4].toStdString():std::string("obstruction")};
            okay=bridge.addObstruction(blockage);
        }
        if(!okay) {
            QTextStream(stderr) << "fd_desktop: --obstruct FROM_S:TO_S:FROM_OFFSET:TO_OFFSET[:NAME] — "
                                << bridge.status() << '\n';
            return 6;
        }
    }
    // Gokart mode (decision 0038): the Gokartcentralen Göteborg track with its rental kart, before any capture time is run.
    if(args.contains("--kart") && !bridge.setKartMode(true)) {
        QTextStream(stderr) << "fd_desktop: --kart: " << bridge.status() << '\n';
        return 5;
    }
    const int at=args.indexOf("--at-time");
    if(at>=0) {
        bool okay=false;
        const double time=at+1<args.size()?args[at+1].toDouble(&okay):-1;
        if(!okay || !std::isfinite(time) || time<0 || time>600)return 4;
        bridge.advanceForCapture(time);
    }
    if(args.contains("--overview")) bridge.setOverview(true);
    if(args.contains("--coupe")) bridge.setAppearance(1);
    // Which car is drawn at the plant's pose, as the showroom would choose it: presentation only.
    const int look=args.indexOf("--appearance");
    if(look>=0) {
        bool okay=false;
        const int value=look+1<args.size()?args[look+1].toInt(&okay):-1;
        if(!okay || value<0 || value>3) {
            QTextStream(stderr) << "--appearance needs 0 (AMR23), 1 (Jesko), 2 (Urus) or 3 (RB19)\n";
            return 4;
        }
        bridge.setAppearance(value);
    }
    const int media = args.indexOf("--showroom-media");
    if(media >= 0 && media + 1 >= args.size()) { QTextStream(stderr) << "--showroom-media needs a media directory\n"; return 4; }
    Showroom showroom(media >= 0 ? args[media + 1] : QStringLiteral(FD_SHOWROOM_MEDIA));
    QObject::connect(&showroom, &Showroom::selected, &bridge, &Bridge::setAppearance);
    const bool silentAudio=args.contains("--verify-ui") || args.contains("--verify-showroom") || args.contains("--capture") || args.contains("--capture-showroom");
    RaceAudio sound(!silentAudio,silentAudio ? testPresets.filePath("sound.ini") : QString{});
    const auto followSound=[&] {
        race_sound::Frame frame;
        frame.eligible=bridge.humanDriving() && !bridge.replay() && showroom.screen()=="driving";
        frame.running=bridge.running(); frame.time=bridge.simulationTime(); frame.speed=bridge.speedKmh()/3.6;
        const auto& simulation=bridge.simulation();
        frame.throttle=simulation.driver_controls().throttle; frame.brake=simulation.driver_controls().brake;
        const auto assistance=simulation.driving_assistance();
        frame.top_speed=fd::assisted_speed_limit_mps(simulation.vehicle_model(),simulation.config(),assistance);
        if(!std::isfinite(frame.top_speed)) {
            frame.top_speed=simulation.config().max_speed_mps;
            if(const auto* car=std::get_if<fd::FourWheelCar>(&simulation.vehicle_model()); car && std::isfinite(car->max_drive_speed_mps))
                frame.top_speed=car->max_drive_speed_mps;
        }
        if(frame.eligible) {
            const auto demand=bridge.tireDemand();
            const double angle=std::max(std::abs(demand.front_slip_angle_rad),std::abs(demand.rear_slip_angle_rad));
            double slip=std::max(0.0,(angle-0.07)/0.20);
            for(double ratio:demand.wheel_slip_ratios) slip=std::max(slip,(std::abs(ratio)-0.12)/0.50);
            frame.slip=std::clamp(slip*(1-fd::driving_assistance_strength(assistance)),0.0,1.0);
        }
        frame.laps=bridge.practiceLaps(); frame.previous_lap=bridge.previousPracticeLap(); frame.best_lap=bridge.bestPracticeLap();
        sound.follow(frame);
    };
    QObject::connect(&bridge,&Bridge::updated,&sound,followSound);
    QObject::connect(&showroom,&Showroom::changed,&sound,followSound);
    QObject::connect(&bridge,&Bridge::presetsChanged,&sound,[&] {
        const auto message=bridge.presetMessage();
        if(message.startsWith("Saved") || message.startsWith("Loaded") || message.startsWith("Deleted")) sound.feedback();
    });
    followSound();
    // The showroom's cars on the track (tools/showroom/build_live_cars.py): presentation only, read once.
    const int liveCarsDirectory = args.indexOf("--live-cars");
    if(liveCarsDirectory >= 0 && liveCarsDirectory + 1 >= args.size()) { QTextStream(stderr) << "--live-cars needs a directory\n"; return 4; }
    LiveCars liveCars(liveCarsDirectory >= 0 ? args[liveCarsDirectory + 1] : QStringLiteral(FD_LIVE_CARS));
    QObject::connect(&bridge, &Bridge::modeChanged, &showroom, [&] { if(bridge.replay()) showroom.bypass(); });
    QQmlApplicationEngine engine;
    QStringList qml_warnings;
    QObject::connect(&engine,&QQmlApplicationEngine::warnings,&app,[&](const QList<QQmlError>& errors) {
        for(const auto& e:errors)qml_warnings.append(e.toString());
    });
    engine.rootContext()->setContextProperty("sim",&bridge);
    engine.rootContext()->setContextProperty("raceAudio",&sound);
    engine.rootContext()->setContextProperty("showroom",&showroom);
    engine.rootContext()->setContextProperty("liveCars",&liveCars);
    engine.load(QUrl("qrc:/qml/Main.qml"));
    if(engine.rootObjects().isEmpty())return 1;
    const bool captureShowroom = args.contains("--capture-showroom");
    const int verifyShowroom = args.indexOf("--verify-showroom");
    if(!bridge.replay() && (captureShowroom || (!args.contains("--verify-ui") && !args.contains("--at-time") && !args.contains("--capture") && !args.contains("--skip-showroom"))))
        showroom.open();
    const int verify=args.indexOf("--verify-ui");
    bool ui_verified=false;
    if(verify>=0 && verify+1<args.size())
        scheduleUiVerification(app,engine,bridge,QFileInfo(args[verify+1]).absoluteFilePath(),qml_warnings,ui_verified);
    if(verifyShowroom>=0 && verifyShowroom+1<args.size())
        scheduleShowroomVerification(app,engine,bridge,showroom,QFileInfo(args[verifyShowroom+1]).absoluteFilePath(),qml_warnings,ui_verified);
    const int capture=captureShowroom ? args.indexOf("--capture-showroom") : args.indexOf("--capture");
    if(capture>=0 && capture+1<args.size()) {
        const QString path=QFileInfo(args[capture+1]).absoluteFilePath();
        QTimer::singleShot(captureShowroom ? 6500 : 1800,&app,[&app,&engine,path] {
            auto* window=qobject_cast<QQuickWindow*>(engine.rootObjects().first());
            QDir().mkpath(QFileInfo(path).absolutePath());
            const bool saved=window && window->grabWindow().save(path);
            app.exit(saved?0:2);
        });
    }
    const int code=app.exec();
    // A verification run that ends without reaching its own verdict, because the window went away or the application
    // was asked to quit, has verified nothing and must not be reported as a pass.
    if(((verify>=0 && verify+1<args.size()) || (verifyShowroom>=0 && verifyShowroom+1<args.size())) && !ui_verified) {
        QTextStream(stderr) << "fd_desktop: --verify-ui ended before its checks finished; no evidence was written\n";
        return 7;
    }
    return code;
}
