#include "bridge.hpp"
#include "fd/track_conditioning.hpp"
#ifdef FD_HAVE_MPCC
#include "fd/mpcc.hpp"
#endif
#ifdef FD_HAVE_RACELINE
#include "fd/raceline.hpp"
#endif
#include <QDir>
#include <QStandardPaths>
#include <QUrl>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace {
QString explain(const std::string& limiting_reason, bool plan_valid, bool envelope_plan) {
    if (!plan_valid) return "Motion exceeds the model envelope. Recovery is not guaranteed.";
    if (envelope_plan) {
        if (limiting_reason == "braking_reachability") return "The car's braking envelope sets this target for the upcoming speed limit.";
        if (limiting_reason == "acceleration_reachability") return "The car's acceleration envelope sets the approach to the next speed limit.";
        if (limiting_reason == "curvature_limit") return "Corner curvature and the car's lateral envelope set this local speed limit.";
    }
    if (limiting_reason == "braking_reachability") return "Braking reachability sets this target for the upcoming curvature limit.";
    if (limiting_reason == "acceleration_reachability") return "Available acceleration sets the approach to the next speed limit.";
    if (limiting_reason == "curvature_limit") return "Corner curvature and supplied grip set this local speed limit.";
    if (limiting_reason == "speed_cap") return "The configured speed cap sets this part of the plan.";
    return QString::fromStdString(limiting_reason);
}
}

Bridge::Bridge(QObject* parent,QString presetsPath) : QObject(parent),
    presets_(presetsPath.isEmpty() ? QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation)+"/forgiveness-presets.json" : presetsPath) {
    preset_message_=presets_.error();
    centreline_ = simulation_.track();
    preset_ = simulation_.track();
    preset_config_ = simulation_.config();
#ifdef FD_HAVE_RACELINE
    // The line depends only on the track, the wheelbase and the steering limit, none of which the desktop changes.
    line_worker_ = std::jthread([this, preset = centreline_, config = simulation_.config()] {
        auto solution = std::make_shared<LineSolution>();
        try {
            const auto smoothed = fd::condition_track(preset);
            const auto line = fd::make_minimum_curvature_line(smoothed, config);
            solution->smoothed_centreline = smoothed.track;
            solution->line = line.track;
            solution->iterations = line.iterations;
            solution->converged = line.converged;
        } catch (const std::exception& e) { solution->failure = e.what(); }
        {
            std::lock_guard lock(line_mutex_);
            line_finished_ = solution;
        }
        QMetaObject::invokeMethod(this, [this] { publishUpdate(); }, Qt::QueuedConnection);
    });
#endif
    ensureLattice();
    refreshPathPoints();
    refreshPrediction();
    elapsed_.start();
    timer_.setTimerType(Qt::PreciseTimer);
    timer_.setInterval(16);
    connect(&timer_, &QTimer::timeout, this, [this] {
        const double wall_seconds = static_cast<double>(elapsed_.nsecsElapsed()) / 1e9;
        elapsed_.restart();
        if (gamepad_enabled_) readGamepad();
        if (!running_) { accumulator_ = 0; return; }
        if (playback_) {
            // Replay consumes wall time as recorded time; the plant is never stepped here.
            const int previous_revision = revision();
            playback_->advance(wall_seconds);
            if (playback_->at_end()) running_ = false;
            publishUpdate(previous_revision != revision());
            return;
        }
        accumulator_ += wall_seconds;
        const auto ticks = std::min(200, static_cast<int>(accumulator_ / simulation_.config().fixed_dt_s));
        // Keep backlog; a slow rendering callback cannot enlarge the plant timestep.
        accumulator_ -= ticks*simulation_.config().fixed_dt_s;
        advanceTicks(ticks);
    });
    timer_.start();
}
void Bridge::advanceTicks(int count) {
    const int previous_revision = revision();
    for (int i=0; i<count; ++i) {
        const auto before = simulation_.state();
        // The decision a step makes is made from the state before that step, which is the state its plan starts in.
        const auto decisions_before = simulation_.decision_count();
        const auto plant_before = simulation_.plant_state();
        simulation_.step();
        if (kart_mode_ && humanDriving() && practice_laps_)
            practice_laps_->observe(before,simulation_.state(),simulation_.driver_controls().throttle);
        if (simulation_.decision_count() != decisions_before) plan_tire_use_state_ = plant_before;
    }
    if (count>0) pending_grip_.reset();
    publishUpdate(previous_revision != revision());
}
void Bridge::readGamepad() {
    const auto before = pad_;
    const auto controls = simulation_.driver_controls();
    pad_ = gamepad_.poll();
    // The Menu button asks to start or pause, on its press.
    const bool pressed = pad_.menu && !menu_held_;
    menu_held_ = pad_.menu;
    if (humanDriving()) {
        // A controller pulled out releases its pedals.
        simulation_.set_driver_controls(pad_.connected ? pad_.controls : fd::DriverControls{});
        // From the line the accelerator asks to start the run.
        if (!running_ && (simulationTime() == 0 || (kart_mode_ && practice_laps_ && !practice_laps_->started())) &&
            pad_.controls.throttle > 0.05) emit acceleratorAtLine();
    }
    if (pressed) emit menuPressed();
    const auto& now = simulation_.driver_controls();
    if (before.connected != pad_.connected || now.throttle != controls.throttle || now.brake != controls.brake ||
        now.steering != controls.steering)
        emit driverChanged();
}
bool Bridge::setHumanDriving(bool on) {
    if (playback_) { error_ = "Replay shows a recorded run; exit replay to drive"; publishUpdate(); return false; }
    if (simulation_.human_driving() == on) return true;
    simulation_.set_human_driving(on);
    if (on && kart_mode_ && presets_.current()) simulation_.set_driving_assistance(*presets_.current());
    if (practice_laps_) practice_laps_->reset();
    // Taking the car, the controller's controls are the person's from the next control decision.
    if (on && pad_.connected) simulation_.set_driver_controls(pad_.controls);
    error_.clear();
    publishUpdate(true, true);
    emit driverChanged();
    return true;
}
bool Bridge::kartRefuses(const char* what) {
    if (!kart_mode_) return false;
    error_ = QString("Gokart mode drives the rental kart as Gokartcentralen runs it; leave gokart mode to change ")+what;
    publishUpdate();
    return true;
}
bool Bridge::setKartMode(bool on) {
    if (kart_mode_ == on) return true;
    if (playback_) { error_ = "Exit replay to drive the gokart track"; publishUpdate(); return false; }
    if (running_) { error_ = "Pause to change the track"; publishUpdate(); return false; }
    try {
        auto grip = pending_grip_.value_or(simulation_.config().grip_mu);
        if (on) {
            // The traced poster, conditioned as any centreline file is, with a light smoothing: the file is already smooth,
            // and heavier smoothing would shorten the published 400 m lap.
            fd::ConditioningOptions options;
            options.smoothing_rms_m = 0.02;
            const auto conditioned = fd::condition_track(fd::load_centreline(std::filesystem::path(QStringLiteral(FD_KART_TRACK).toStdU16String())),
                                                         6.0, "Gokartcentralen Göteborg", options);
            const auto& profile = fd::vehicle_profile("gokartcentralen-rsx2");
            auto config = fd::configured_for(profile, simulation_.config());
            config.grip_mu = grip;
            // Hairpins of 4 to 6 m: Pure Pursuit aims 2 m ahead, not 4, as on the traced Formula Student courses.
            config.lookahead_base_m = 2.0;
            fd::Simulation fresh(conditioned.track, config, profile.car, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                                 simulation_.local_planner_mode());
            fresh.set_course({}, fd::make_gates(conditioned.track), "Gokartcentralen Göteborg: barriers, no cones");
            fresh.set_competition(simulation_.judge().rules());
            if (const auto* sensors = simulation_.sensors()) fresh.set_sensors(sensors->settings());
            if (const auto* perception = simulation_.perception()) fresh.set_perception(perception->settings());
            if (simulation_.human_driving()) fresh.set_human_driving(true);
            // Restore the user's last saved/loaded setup only on the kart, through the normal tick-boundary seam.
            if (fresh.human_driving() && presets_.current()) fresh.set_driving_assistance(*presets_.current());
            before_kart_ = BeforeKart{centreline_, course_name_, simulation_.cones(), simulation_.judge().course().gates,
                                      simulation_.course_source(), simulation_.vehicle_model(), simulation_.steering_mode(),
                                      simulation_.speed_plan_mode(), simulation_.config(), loaded_profile_, preset_config_};
            simulation_ = std::move(fresh);
            centreline_ = conditioned.track;
            course_name_ = "Gokartcentralen Göteborg";
            loaded_profile_ = profile.id;
            line_mode_ = 0;
            kart_mode_ = true;
        } else {
            const auto before = *before_kart_;
            auto config = before.config;
            config.grip_mu = grip;
            fd::Simulation fresh(before.centreline, config, before.model, before.steering, before.speed_plan, simulation_.local_planner_mode());
            if (!before.course_name.isEmpty()) fresh.set_course(before.cones, before.gates, before.course_source);
            fresh.set_competition(simulation_.judge().rules());
            if (const auto* sensors = simulation_.sensors()) fresh.set_sensors(sensors->settings());
            if (const auto* perception = simulation_.perception()) fresh.set_perception(perception->settings());
            if (simulation_.human_driving()) fresh.set_human_driving(true);
            simulation_ = std::move(fresh);
            centreline_ = before.centreline;
            course_name_ = before.course_name;
            loaded_profile_ = before.profile;
            preset_config_ = before.preset_config;
            line_mode_ = 0;
            kart_mode_ = false;
            before_kart_.reset();
        }
    } catch (const std::exception& e) { error_ = QString::fromUtf8(e.what()); publishUpdate(); return false; }
    if (kart_mode_) practice_laps_.emplace(simulation_.track());
    else practice_laps_.reset();
    pending_grip_.reset();
    accumulator_ = 0;
    error_.clear();
    publishUpdate(true, true);
    return true;
}
bool Bridge::setDriverControls(double throttle, double brake, double steering) {
    try { simulation_.set_driver_controls({throttle, brake, steering}); }
    catch (const std::exception& e) { error_ = QString::fromUtf8(e.what()); publishUpdate(); return false; }
    emit driverChanged();
    return true;
}
bool Bridge::setForgiveness(bool enabled, double level) {
    if (!kartMode() || !humanDriving()) {
        error_="Choose You and Gokart to use forgiveness"; publishUpdate(); return false;
    }
    auto assistance=simulation_.requested_driving_assistance();
    assistance.enabled=enabled; assistance.level=level;
    try { simulation_.set_driving_assistance(assistance); }
    catch (const std::exception& e) { error_=QString::fromUtf8(e.what()); publishUpdate(); return false; }
    error_.clear();
    publishUpdate();
    return true;
}
QVariantMap Bridge::forgivenessTuning() const {
    const auto a=simulation_.requested_driving_assistance();
    return {{"acceleration",a.acceleration},{"braking",a.braking},{"steering",a.steering},{"grip",a.grip},{"speed",a.speed}};
}
bool Bridge::setForgivenessManual(bool manual,double acceleration,double braking,double steering,double grip,double speed) {
    if (!kartMode() || !humanDriving()) {
        error_="Choose You and Gokart to tune forgiveness"; publishUpdate(); return false;
    }
    auto a=simulation_.requested_driving_assistance();
    a.manual=manual; a.acceleration=acceleration; a.braking=braking; a.steering=steering; a.grip=grip; a.speed=speed;
    try { simulation_.set_driving_assistance(a); }
    catch (const std::exception& e) { error_=QString::fromUtf8(e.what()); publishUpdate(); return false; }
    error_.clear(); publishUpdate(); return true;
}
QVariantList Bridge::forgivenessPresets() const {
    QVariantList result;
    for(int i=0;i<5;++i) result.append(bool(presets_.slot(i)));
    return result;
}
bool Bridge::saveForgivenessPreset(int slot) {
    if (!kartMode() || !humanDriving()) return false;
    const bool saved=presets_.save(slot,simulation_.requested_driving_assistance());
    preset_message_=saved ? QString("Saved preset %1").arg(slot+1) : presets_.error();
    emit presetsChanged(); return saved;
}
bool Bridge::loadForgivenessPreset(int slot) {
    if (!kartMode() || !humanDriving()) return false;
    try {
        if (!presets_.slot(slot)) { preset_message_="That slot is empty."; emit presetsChanged(); return false; }
        auto settings=*presets_.slot(slot); settings.enabled=true;
        if (!presets_.saveCurrent(settings)) { preset_message_=presets_.error(); emit presetsChanged(); return false; }
        simulation_.set_driving_assistance(settings);
    } catch (const std::exception&) { preset_message_="Could not load that preset."; emit presetsChanged(); return false; }
    preset_message_=QString("Loaded preset %1").arg(slot+1);
    publishUpdate(); emit presetsChanged(); return true;
}
bool Bridge::deleteForgivenessPreset(int slot) {
    if (!kartMode() || !humanDriving()) return false;
    const bool removed=presets_.save(slot,std::nullopt);
    preset_message_=removed ? QString("Deleted preset %1").arg(slot+1) : presets_.error();
    emit presetsChanged(); return removed;
}
void Bridge::publishUpdate(bool plan_changed, bool mode_changed) {
    // QObject property observers can run before ordinary signal slots. Complete all
    // mode-aware caches before notifying any binding or scene geometry observer.
    ensureEnvelope();
    if (takeRacingLine()) mode_changed = plan_changed = true;
    if (ensureLattice()) mode_changed = true;
    // Estimates follow the live plan in force; replay shows the recorded track alone.
    if (plan_changed && !playback_) refreshLineEstimates();
    if (plan_changed) refreshPathPoints();
    refreshPrediction();
    refreshPlanTireUse(plan_changed);
    if (mode_changed) refreshOptimalMatch();
    if (mode_changed) emit modeChanged();
    if (plan_changed) emit planChanged();
    emit updated();
}
void Bridge::advanceForCapture(double seconds) {
    if (playback_) { seek(seconds); return; }
    advanceTicks(static_cast<int>(std::round(seconds/simulation_.config().fixed_dt_s)));
}
double Bridge::targetSpeedKmh() const {
    return (playback_ ? playback_->sample().target_speed_mps : simulation_.diagnostics().target_speed_mps)*3.6;
}
double Bridge::plannedAcceleration() const {
    if (predictionAvailable()) return prediction_.front().acceleration_mps2;
    return playback_ ? plan().at(nearestIndex()).acceleration_mps2 : 0.0;
}
bool Bridge::predictionWithinEnvelope() const {
    if (!predictionAvailable()) return false;
    if (playback_) return recordedDecision() && recordedDecision()->choice().within_envelope;
    return simulation_.trajectory().within_model_envelope;
}
QString Bridge::predictionStatus() const {
    if (playback_ && !recordedDecision()) return "No recorded prediction";
    if (!predictionAvailable()) return "Prediction unavailable";
    if (!predictionWithinEnvelope())
        return QString::fromStdString(playback_ ? recordedDecision()->choice().validity_reason : simulation_.trajectory().validity_reason);
    return "Within model envelope";
}
double Bridge::grip() const {
    if (playback_) return playback_->sample().grip_mu;
    return pending_grip_.value_or(simulation_.config().grip_mu);
}
double Bridge::progress() const {
    const double progressed = playback_ ? playback_->sample().progress_m : simulation_.diagnostics().progress_m;
    return std::fmod(std::max(0.0,progressed), track().length_m) / track().length_m;
}
double Bridge::crossTrackError() const {
    return playback_ ? playback_->sample().cross_track_error_m : simulation_.diagnostics().cross_track_error_m;
}
double Bridge::cornerDistance() const {
    const auto& t=track();
    const double a=t.points.at(limitingIndex()).s_m;
    const double here = playback_ ? playback_->sample().path_s_m : simulation_.diagnostics().path_s_m;
    return std::fmod(a-here+t.length_m,t.length_m);
}
double Bridge::utilization() const {
    return playback_ ? playback_->sample().combined_grip_utilization : simulation_.diagnostics().combined_grip_utilization;
}
int Bridge::revision() const {
    return static_cast<int>(playback_ ? playback_->sample().revision : simulation_.diagnostics().revision);
}
int Bridge::lap() const { return playback_ ? playback_->sample().laps : simulation_.diagnostics().laps; }
QString Bridge::status() const {
    if (!error_.isEmpty()) return error_;
    if (playback_) {
        const auto& s = playback_->sample();
        if (!s.plan_valid || !s.within_track) return "Recorded: "+QString::fromStdString(s.validity_reason);
        if (running_) return "Replaying / recorded state";
        return playback_->at_end() ? "Replay finished / recorded state" : "Replay paused / recorded state";
    }
    if (pending_grip_) return "Grip queued / applies on next simulation tick";
    if (!simulation_.diagnostics().plan_valid || !simulation_.diagnostics().within_track)
        return QString::fromStdString(simulation_.diagnostics().validity_reason);
    if (simulation_.human_driving()) return running_ ? "Running / you drive" : simulationTime() == 0 ? "Ready / you drive" : "Paused / state preserved";
    if (running_) return simulation_.cone_driver() ? (simulation_.sensors() ? "Running / measured state, on cones" : "Running / ideal state, on cones")
                                                   : "Running / ideal state";
    return simulationTime() == 0 ? (simulation_.cone_driver() ? "Ready / on cones" : "Ready / known track") : "Paused / state preserved";
}
QString Bridge::reason() const {
    // A blockage overrides the speed-envelope explanation: it is why the car is doing
    // what it is doing right now, or was doing at this recorded moment.
    if (!blockingIdentifier().isEmpty()) return decisionReason();
    if (playback_) return explain(playback_->sample().limiting_reason, playback_->sample().plan_valid, speedPlanMode() == 1);
    return explain(simulation_.diagnostics().limiting_reason, simulation_.diagnostics().plan_valid, speedPlanMode() == 1);
}
QString Bridge::decisionReason() const {
    if (playback_) return recordedDecision() ? QString::fromStdString(recordedDecision()->reason) : "No recorded decision";
    return QString::fromStdString(simulation_.decision().reason);
}
bool Bridge::holdingForBlockage() const {
    return playback_ ? recordedDecision() && recordedDecision()->holding : simulation_.decision().holding;
}
QString Bridge::blockingIdentifier() const {
    if (playback_) return recordedDecision() ? QString::fromStdString(recordedDecision()->blocking_identifier) : QString();
    return QString::fromStdString(simulation_.decision().blocking_identifier);
}
int Bridge::evaluatedOptions() const {
    if (playback_) return recordedDecision() ? static_cast<int>(recordedDecision()->options.size()) : 0;
    return static_cast<int>(simulation_.decision().options.size());
}
int Bridge::rejectedOptions() const {
    if (playback_) {
        if (!recordedDecision()) return 0;
        const auto& options = recordedDecision()->options;
        return static_cast<int>(std::count_if(options.begin(), options.end(), [](const fd::RecordedOption& o) { return !o.clear; }));
    }
    const auto& options = simulation_.decision().options;
    return static_cast<int>(std::count_if(options.begin(), options.end(),
                                          [](const fd::LocalOption& o) { return !o.clear; }));
}
double Bridge::selectedOffset() const {
    if (playback_) return recordedDecision() ? recordedDecision()->choice().lateral_offset_m : 0.0;
    return simulation_.decision().choice().lateral_offset_m;
}
QString Bridge::chosenAction() const {
    if (!decisionAvailable()) return {};
    return QString::fromUtf8(fd::local_action_name(playback_ ? recordedDecision()->choice().action : simulation_.decision().choice().action));
}
QString Bridge::decisionTitle() const {
    if (!decisionAvailable()) return {};
    const auto action = playback_ ? recordedDecision()->choice().action : simulation_.decision().choice().action;
    const bool path = playback_ ? !recordedDecision()->choice().path.empty() : !simulation_.decision().choice().path.empty();
    if (action == fd::LocalAction::offset) return holdingForBlockage() ? "BLOCKED" : "AVOIDING";
    if (action == fd::LocalAction::straight && path) return "RETURNING";
    return chosenAction().toUpper();
}
QString Bridge::decisionCost() const {
    if (!decisionAvailable()) return {};
    const bool path = playback_ ? !recordedDecision()->choice().path.empty() : !simulation_.decision().choice().path.empty();
    if (!path) return {};
    const auto& cost = playback_ ? recordedDecision()->choice().path_cost : simulation_.decision().choice().path_cost;
    const auto compact = [](double value) {
        if (value >= 1e6) return QString::number(value/1e6, 'f', 2)+"M";
        if (value >= 1e4) return QString::number(value/1e3, 'f', 0)+"k";
        if (value >= 1e3) return QString::number(value/1e3, 'f', 1)+"k";
        return QString::number(value, 'f', 0);
    };
    return "Cost "+compact(cost.total())+"  ·  curvature "+compact(cost.edges.average_curvature)+", range "+
           compact(cost.edges.curvature_range)+", length "+compact(cost.edges.length)+", deviation "+
           compact(cost.edges.reference_deviation)+", turns "+compact(cost.turns)+", goal "+compact(cost.goal);
}
QVariantList Bridge::actionTimes() const {
    QVariantList list;
    if (!decisionAvailable()) return list;
    const auto label = [](fd::LocalAction action, bool path) {
        switch (action) {
        case fd::LocalAction::pass_left: return QStringLiteral("Left");
        case fd::LocalAction::pass_right: return QStringLiteral("Right");
        case fd::LocalAction::straight: return path ? QStringLiteral("Return") : QStringLiteral("Rejoin");
        default: return QString::fromUtf8(fd::local_action_name(action));
        }
    };
    const auto entry = [&](fd::LocalAction action, bool path, double seconds, bool clear, bool chosen, double x, double y) {
        QVariantMap item;
        item["action"] = QString::fromUtf8(fd::local_action_name(action));
        item["label"] = label(action, path);
        item["seconds"] = seconds;
        item["clear"] = clear;
        item["chosen"] = chosen;
        item["x"] = x;
        item["y"] = y;
        list.append(item);
    };
    // Where a predicted line has run label_distance_m, or its end if it is shorter.
    const auto anchor = [](const std::vector<fd::Vec2>& points) {
        double run = 0;
        for (std::size_t k = 1; k < points.size(); ++k) {
            run += std::hypot(points[k].x-points[k-1].x, points[k].y-points[k-1].y);
            if (run >= label_distance_m) return points[k];
        }
        return points.back();
    };
    if (playback_) {
        const auto* decision = recordedDecision();
        for (std::size_t i = 0; i < decision->options.size(); ++i) {
            const auto& option = decision->options[i];
            if (!option.estimated_time_s || option.trajectory.empty()) continue;
            std::vector<fd::Vec2> points;
            for (const auto& p : option.trajectory) points.push_back({p.x_m, p.y_m});
            const auto at = anchor(points);
            entry(option.action, !option.path.empty(), *option.estimated_time_s, option.clear, i == decision->selected, at.x, at.y);
        }
        return list;
    }
    const auto& decision = simulation_.decision();
    for (std::size_t i = 0; i < decision.options.size(); ++i) {
        const auto& option = decision.options[i];
        if (option.speed_profile.empty() || !std::isfinite(option.estimated_time_s) || option.trajectory.points.empty()) continue;
        std::vector<fd::Vec2> points;
        for (const auto& p : option.trajectory.points) points.push_back({p.state.x_m, p.state.y_m});
        const auto at = anchor(points);
        entry(option.action, !option.path.empty(), option.estimated_time_s, option.clear, i == decision.selected, at.x, at.y);
    }
    return list;
}
QColor Bridge::actionColor(fd::LocalAction action, bool clear) {
    QColor color;
    switch (action) {
    case fd::LocalAction::offset: return clear ? QColor("#4c6b5c") : QColor("#6b4a4a");
    case fd::LocalAction::straight: color = QColor("#b8c2c6"); break;
    case fd::LocalAction::pass_left: color = QColor("#9d8cf2"); break;
    case fd::LocalAction::pass_right: color = QColor("#ef8fc3"); break;
    case fd::LocalAction::brake: color = QColor("#f47770"); break;
    }
    return clear ? color : color.darker(220);
}
QString Bridge::actionColorName(const QString& action) const {
    for (const auto candidate : {fd::LocalAction::offset, fd::LocalAction::straight, fd::LocalAction::pass_left,
                                 fd::LocalAction::pass_right, fd::LocalAction::brake})
        if (action == QString::fromUtf8(fd::local_action_name(candidate))) return actionColor(candidate).name();
    return QColor("#87959d").name();
}
std::vector<Bridge::Alternative> Bridge::alternatives() const {
    std::vector<Alternative> lines;
    if (playback_) {
        const auto* decision = recordedDecision();
        for (std::size_t i = 0; decision && i < decision->options.size(); ++i) {
            if (i == decision->selected) continue;
            Alternative line{{}, decision->options[i].clear, decision->options[i].action};
            for (const auto& point : decision->options[i].trajectory) line.points.push_back({point.x_m, point.y_m});
            lines.push_back(std::move(line));
        }
        return lines;
    }
    const auto& decision = simulation_.decision();
    for (std::size_t i = 0; i < decision.options.size(); ++i) {
        if (i == decision.selected) continue;
        Alternative line{{}, decision.options[i].clear, decision.options[i].action};
        for (const auto& point : decision.options[i].trajectory.points) line.points.push_back({point.state.x_m, point.state.y_m});
        lines.push_back(std::move(line));
    }
    return lines;
}
bool Bridge::addObstruction(const fd::Obstruction& obstruction) {
    if (playback_) { error_ = "Replay shows a recorded scenario; exit replay to change the live one"; publishUpdate(); return false; }
    if (line_mode_ == 1) { error_ = "Blockages are stated across the centreline; choose the centreline to place one"; publishUpdate(); return false; }
    auto next = simulation_.obstructions();
    next.push_back(obstruction);
    try { simulation_.set_obstructions(std::move(next)); }
    catch (const std::exception& e) { error_ = QString::fromUtf8(e.what()); publishUpdate(); return false; }
    error_.clear();
    publishUpdate(false, true);
    return true;
}
bool Bridge::placeBlockageAhead(bool fullWidth) {
    if (playback_) { error_ = "Replay shows a recorded scenario; exit replay to change the live one"; publishUpdate(); return false; }
    if (line_mode_ == 1) { error_ = "Blockages are stated across the centreline; choose the centreline to place one"; publishUpdate(); return false; }
    const auto& t = simulation_.track();
    const double here = fd::project(t, {simulation_.state().x_m, simulation_.state().y_m}).s_m;
    // Far enough ahead that the car has room to react at its current speed.
    const double lead = std::max(45.0, simulation_.state().speed_mps*2.5);
    const auto wrap = [&](double s) { return std::fmod(std::fmod(s, t.length_m)+t.length_m, t.length_m); };
    const double half = std::max(0.0, t.width_m/2);
    fd::Obstruction blockage{wrap(here+lead), wrap(here+lead+6),
                             -half, fullWidth ? half : 1.0,
                             fullWidth ? "stalled car" : "cone cluster"};
    return addObstruction(blockage);
}
double Bridge::rearSideslipDegrees() const {
    const double radians = playback_ ? fd::rear_sideslip_rad(playback_->sample().plant_state())
                                     : simulation_.diagnostics().rear_sideslip_rad;
    return radians*180/std::numbers::pi;
}
fd::Demand Bridge::tireDemand() const {
    if (playback_) return playback_->demand();
    return fd::demand(simulation_.vehicle_model(), simulation_.plant_state(), simulation_.config(),
                      simulation_.diagnostics().applied.acceleration_mps2);
}
QString Bridge::wheelState(const fd::Demand& demand, const fd::PlantState& state, std::size_t wheel, double floor) {
    if (state.wheel_loads_n[wheel] == 0) return "LIFT";
    if (state.wheel_speeds_radps[wheel] == 0 && state.pose.speed_mps > floor) return "LOCK";
    if (demand.wheel_combined_slip[wheel] > 1) return demand.wheel_slip_ratios[wheel] > 0 ? "SPIN" : "SLIDE";
    return "GRIP";
}
QVariantList Bridge::wheelSlipRatios() const {
    QVariantList list;
    if (!wheelsModeled()) return list;
    const auto demand = tireDemand();
    for (const double ratio : demand.wheel_slip_ratios) list.append(ratio);
    return list;
}
QVariantList Bridge::wheelStates() const {
    QVariantList list;
    if (forgivenessActive()) {
        for (int i=0;i<4;++i) list.append(QStringLiteral("ASSIST"));
        return list;
    }
    const auto* car = std::get_if<fd::FourWheelCar>(&vehicleModelData());
    if (!car) return list;
    const auto demand = tireDemand();
    const auto plant = playback_ ? playback_->sample().plant_state() : simulation_.plant_state();
    for (std::size_t i = 0; i < 4; ++i) list.append(wheelState(demand, plant, i, car->slip_speed_floor_mps));
    return list;
}
QVariantList Bridge::planTireUse() const {
    QVariantList rows;
    for (const auto& point : plan_tire_use_) {
        QVariantList row{point.time_s};
        for (const double use : point.use) row.append(use);
        rows.append(QVariant(row));
    }
    return rows;
}
double Bridge::planTireWorstUse() const {
    double worst = 0;
    for (const auto& point : plan_tire_use_)
        for (const double use : point.use) worst = std::max(worst, use);
    return worst;
}
double Bridge::planTireWorstTime() const {
    double worst = 0, when = 0;
    for (const auto& point : plan_tire_use_)
        for (const double use : point.use)
            if (use > worst) { worst = use; when = point.time_s-plan_tire_use_.front().time_s; }
    return when;
}
// The plan in force, its commands and the state it was made in, live or recorded; then the plant driven along it.
void Bridge::refreshPlanTireUse(bool plan_changed) {
    const auto* car = std::get_if<fd::FourWheelCar>(&vehicleModelData());
    if (!car) { plan_tire_use_.clear(); return; }
    if (playback_) {
        const auto* decision = recordedDecision();
        if (!decision || decision->choice().commands.empty()) { plan_tire_use_.clear(); return; }
        if (!plan_tire_use_.empty() && plan_tire_use_time_ == decision->time_s) return;
        const auto& recording = playback_->recording();
        const double dt = recording.metadata.initial_config.fixed_dt_s;
        const auto tick = static_cast<std::size_t>(std::llround(decision->time_s/dt));
        if (tick >= recording.samples.size()) { plan_tire_use_.clear(); return; }
        std::vector<double> times;
        for (const auto& point : decision->choice().trajectory) times.push_back(point.time_s);
        const auto control_ticks = static_cast<std::uint64_t>(std::llround(recording.metadata.initial_config.control_dt_s/dt));
        const auto ticks = control_ticks-static_cast<std::uint64_t>(tick)%control_ticks;
        try {
            plan_tire_use_ = fd::plan_tire_use(recording.metadata.vehicle_model, recording.samples[tick].plant_state(),
                                               recording.config_for_revision(recording.samples[std::min(tick+1, recording.samples.size()-1)].revision),
                                               times, decision->choice().commands, ticks);
            plan_tire_use_time_ = decision->time_s;
        } catch (const std::exception&) { plan_tire_use_.clear(); }
        return;
    }
    const auto& trajectory = simulation_.decision().trajectory();
    if (trajectory.commands.size() != trajectory.points.size() || trajectory.points.empty()) { plan_tire_use_.clear(); return; }
    const double made_at = trajectory.points.front().state.time_s;
    // Only when the plan is new, and only a few times a second: this drives the plant three seconds along it.
    const bool stale = plan_tire_use_.empty() || made_at-plan_tire_use_time_ >= 0.2 || plan_changed;
    if (!stale || simulation_.decision_count() == plan_tire_use_decisions_) return;
    if (std::abs(plan_tire_use_state_.pose.time_s-made_at) > 1e-9) return;
    const auto control_ticks = static_cast<std::uint64_t>(std::llround(simulation_.config().control_dt_s/simulation_.config().fixed_dt_s));
    const auto tick = static_cast<std::uint64_t>(std::llround(made_at/simulation_.config().fixed_dt_s));
    try {
        plan_tire_use_ = fd::plan_tire_use(simulation_.vehicle_model(), plan_tire_use_state_, simulation_.config(),
                                           fd::point_times(trajectory), trajectory.commands, control_ticks-tick%control_ticks);
        plan_tire_use_time_ = made_at;
        plan_tire_use_decisions_ = simulation_.decision_count();
    } catch (const std::exception&) { plan_tire_use_.clear(); }
}
QVariantList Bridge::wheelLoads() const {
    QVariantList list;
    if (!wheelsModeled()) return list;
    const auto plant = playback_ ? playback_->sample().plant_state() : simulation_.plant_state();
    for (const double load : plant.wheel_loads_n) list.append(load);
    return list;
}
QVariantList Bridge::staticWheelLoads() const {
    QVariantList list;
    const auto* car = std::get_if<fd::FourWheelCar>(&vehicleModelData());
    if (!car) return list;
    const auto& initial = playback_ ? playback_->recording().metadata.initial_config : simulation_.config();
    for (const double load : fd::quasi_static_wheel_loads(*car, initial, 0, 0, 0)) list.append(load);
    return list;
}
bool Bridge::aerodynamicsModeled() const {
    const auto* car = std::get_if<fd::FourWheelCar>(&vehicleModelData());
    return car && (car->drag_area_m2 > 0 || car->downforce_area_m2 > 0);
}
fd::AerodynamicForce Bridge::aerodynamics() const {
    const auto* car = std::get_if<fd::FourWheelCar>(&vehicleModelData());
    if (!car) return {};
    const auto plant = playback_ ? playback_->sample().plant_state() : simulation_.plant_state();
    // The air meets the centre of gravity, whose lateral velocity adds the yaw rate's share to the rear axle's.
    return fd::aerodynamic_force(*car, plant.pose.speed_mps, plant.lateral_velocity_mps+car->cg_to_rear_m*plant.yaw_rate_radps);
}
QVariantList Bridge::wheelFrictionUse() const {
    QVariantList list;
    if (!wheelsModeled()) return list;
    const auto demand = tireDemand();
    for (std::size_t i = 0; i < 4; ++i) list.append(QVariant(QVariantList{demand.wheel_longitudinal_use[i], demand.wheel_lateral_use[i]}));
    return list;
}
double Bridge::steeringCorrectionDegrees() const {
    const double limit = config().max_steering_rad;
    const double requested = playback_ ? playback_->sample().requested_steering_rad : simulation_.diagnostics().requested.steering_rad;
    const double geometric = playback_ ? playback_->sample().geometric_steering_rad : simulation_.diagnostics().geometric_steering_rad;
    return (std::clamp(requested, -limit, limit)-std::clamp(geometric, -limit, limit))*180/std::numbers::pi;
}
// A fresh live run on the given track, model and steering, keeping the applied grip and the stated scenario.
bool Bridge::rebuild(const fd::Track& track, const fd::VehicleModel& model, fd::SteeringMode steering, fd::SpeedPlanMode speed_plan,
                     std::optional<fd::LocalPlannerMode> planner, std::optional<fd::ControllerMode> controller,
                     std::optional<fd::Config> configuration) {
    auto config = configuration.value_or(simulation_.config());
    if (pending_grip_) config.grip_mu = *pending_grip_;
    const auto obstructions = simulation_.obstructions();
    std::shared_ptr<fd::PredictiveController> driver;
#ifdef FD_HAVE_MPCC
    if (controller.value_or(simulation_.controller_mode()) == fd::ControllerMode::mpcc) driver = std::make_shared<fd::Mpcc>();
#endif
    try {
        fd::Simulation fresh(track, config, model, steering, speed_plan, planner.value_or(simulation_.local_planner_mode()), driver);
        // A loaded course keeps its cones and gates, and every run its rules.
        if (!course_name_.isEmpty()) fresh.set_course(simulation_.cones(), simulation_.judge().course().gates, simulation_.course_source());
        fresh.set_competition(simulation_.judge().rules());
        fresh.set_obstructions(obstructions);
        if (const auto* sensors = simulation_.sensors()) fresh.set_sensors(sensors->settings());
        if (const auto* perception = simulation_.perception()) fresh.set_perception(perception->settings());
        if (const auto* on_cones = simulation_.cone_driver()) fresh.set_cone_driving(on_cones->settings(), on_cones->memory_settings());
        // A person keeps the car through a fresh run, as the other choices keep theirs.
        if (simulation_.human_driving()) {
            fresh.set_human_driving(true);
            fresh.set_driving_assistance(simulation_.requested_driving_assistance());
        }
        simulation_ = std::move(fresh);
        if (practice_laps_) practice_laps_->reset();
    } catch (const std::exception& e) { error_ = QString::fromUtf8(e.what()); publishUpdate(); return false; }
    pending_grip_.reset();
    accumulator_ = 0;
    error_.clear();
    publishUpdate(true, true);
    return true;
}
void Bridge::ensureEnvelope() {
    const auto* car = std::get_if<fd::FourWheelCar>(&vehicleModelData());
    envelope_ready_ = false;
    envelope_failed_now_ = false;
    if (!car) return;
    const fd::Config wanted = config();
    {
        // Take a result the worker finished, if it is for the car shown.
        std::lock_guard lock(envelope_mutex_);
        if (envelope_finished_ && envelope_finished_->generated_for(*car, wanted)) envelope_ = envelope_finished_;
    }
    if (envelope_ && envelope_->generated_for(*car, wanted)) { envelope_ready_ = true; return; }
    // A live run planned from the envelope already derived it; show that one rather than deriving it again.
    if (const auto* planned = playback_ ? nullptr : simulation_.performance_envelope(); planned && planned->generated_for(*car, wanted)) {
        envelope_ = std::make_shared<const fd::PerformanceEnvelope>(*planned);
        envelope_ready_ = true;
        return;
    }
    if (envelope_busy_) return;
    try { envelope_failed_now_ = fd::performance_envelope_fingerprint(*car, wanted) == envelope_failed_; }
    catch (const std::exception&) { envelope_failed_now_ = true; }
    if (envelope_failed_now_) return;
    envelope_busy_ = true;
    if (envelope_worker_.joinable()) envelope_worker_.join();
    envelope_worker_ = std::jthread([this, model = fd::VehicleModel{*car}, wanted] {
        std::shared_ptr<const fd::PerformanceEnvelope> derived;
        std::string failed;
        try { derived = std::make_shared<const fd::PerformanceEnvelope>(model, wanted); }
        catch (const std::exception&) { failed = fd::performance_envelope_fingerprint(model, wanted); }
        {
            std::lock_guard lock(envelope_mutex_);
            envelope_finished_ = derived;
        }
        QMetaObject::invokeMethod(this, [this, failed] {
            envelope_busy_ = false;
            if (!failed.empty()) envelope_failed_ = failed;
            publishUpdate();
        }, Qt::QueuedConnection);
    });
}
void Bridge::deriveEnvelopeNow() {
    if (envelope_worker_.joinable()) envelope_worker_.join();
    envelope_busy_ = false;
    const auto* car = std::get_if<fd::FourWheelCar>(&vehicleModelData());
    if (car) {
        std::lock_guard lock(envelope_mutex_);
        if (envelope_finished_ && envelope_finished_->generated_for(*car, config())) envelope_ = envelope_finished_;
    }
    if (car && !(envelope_ && envelope_->generated_for(*car, config())))
        envelope_ = std::make_shared<const fd::PerformanceEnvelope>(*car, config());
    publishUpdate();
}
QVariantList Bridge::envelopeBoundary() const {
    QVariantList points;
    if (!envelope_ready_) return points;
    const double speed = state().speed_mps;
    const auto left = envelope_->side_at(speed, fd::TurnSide::left);
    const auto right = envelope_->side_at(speed, fd::TurnSide::right);
    const auto point = [&](double lateral, double longitudinal) { points.append(QVariantMap{{"lateral", lateral}, {"longitudinal", longitudinal}}); };
    for (const auto& level : left.levels) point(level.lateral_mps2, level.forward_mps2);
    for (auto level = left.levels.rbegin(); level != left.levels.rend(); ++level) point(level->lateral_mps2, level->braking_mps2);
    for (const auto& level : right.levels) point(-level.lateral_mps2, level.braking_mps2);
    for (auto level = right.levels.rbegin(); level != right.levels.rend(); ++level) point(-level->lateral_mps2, level->forward_mps2);
    return points;
}
double Bridge::ggLateral() const {
    return playback_ ? playback_->sample().lateral_acceleration_mps2 : simulation_.diagnostics().lateral_acceleration_mps2;
}
double Bridge::ggLongitudinal() const {
    // What the car achieved over the last tick, as the simulation and the recording both state it.
    return playback_ ? playback_->sample().longitudinal_acceleration_mps2
                     : simulation_.diagnostics().longitudinal_acceleration_mps2;
}
bool Bridge::setVehicleModel(int model) {
    if (playback_) { error_ = "Replay shows the recorded vehicle model; exit replay to change the live one"; publishUpdate(); return false; }
    if (running_) { error_ = "Pause to change the vehicle model"; publishUpdate(); return false; }
    if (kartRefuses("the car")) return false;
    if (model < 0 || model > 3) return false;
    if (model == vehicleModel()) return true;
    const fd::VehicleModel next = model == 0 ? fd::VehicleModel{fd::KinematicBicycle{}}
                                : model == 1 ? fd::VehicleModel{fd::DynamicSingleTrack{}}
                                : model == 2 ? fd::VehicleModel{fd::DynamicSingleTrack::soft_front()}
                                 : loaded_profile_.empty() ? fd::VehicleModel{fd::FourWheelCar{}}
                                                           : fd::VehicleModel{fd::vehicle_profile(loaded_profile_).car};
    // The kinematic bicycle's steering table would be its geometry, so it is steered by Pure Pursuit.
    // Only the four-wheel car has an envelope to plan with; every other model returns to the grip fractions.
    // MPCC predicts with tires, so the kinematic bicycle returns to the policy.
    return rebuild(simulation_.track(), next, model == 0 ? fd::SteeringMode::pure_pursuit : simulation_.steering_mode(),
                   model == 3 ? simulation_.speed_plan_mode() : fd::SpeedPlanMode::grip_fractions, std::nullopt,
                   model == 0 ? std::optional<fd::ControllerMode>(fd::ControllerMode::policy) : std::nullopt);
}
QVariantList Bridge::setupControls() const {
    QVariantList list;
    const auto* car = std::get_if<fd::FourWheelCar>(&vehicleModelData());
    if (!car) return list;
    const auto text = [](std::string_view view) { return QString::fromUtf8(view.data(), static_cast<qsizetype>(view.size())); };
    for (const auto& control : fd::four_wheel_setup_controls())
        list.append(QVariantMap{{"key", text(control.key)}, {"label", text(control.label)}, {"unit", text(control.unit)},
                                {"scale", control.display_scale}, {"minimum", control.minimum}, {"maximum", control.maximum},
                                {"step", control.step}, {"value", fd::setup_value(*car, control.key)}, {"effect", text(control.effect)}});
    return list;
}
bool Bridge::setSetupValue(const QString& key, double value) {
    if (playback_) { error_ = "Replay shows the recorded setup; exit replay to change the live car"; publishUpdate(); return false; }
    if (running_) { error_ = "Pause to change the car's setup"; publishUpdate(); return false; }
    const auto* car = std::get_if<fd::FourWheelCar>(&simulation_.vehicle_model());
    if (!car) { error_ = "Setup controls apply to the 4 wheels car"; publishUpdate(); return false; }
    const std::string name = key.toStdString();
    fd::FourWheelCar next;
    try { next = fd::with_setup_value(*car, simulation_.config(), name, value); }
    catch (const std::exception& e) { error_ = QString::fromUtf8(e.what()); publishUpdate(); return false; }
    if (fd::setup_value(*car, name) == value) return true;
    return rebuild(simulation_.track(), next, fd::SteeringMode::pure_pursuit, simulation_.speed_plan_mode());
}
bool Bridge::resetSetup() {
    if (playback_) { error_ = "Replay shows the recorded setup; exit replay to change the live car"; publishUpdate(); return false; }
    if (running_) { error_ = "Pause to change the car's setup"; publishUpdate(); return false; }
    if (!std::holds_alternative<fd::FourWheelCar>(simulation_.vehicle_model())) {
        error_ = "Setup controls apply to the 4 wheels car"; publishUpdate(); return false;
    }
    const fd::FourWheelCar defaults = loaded_profile_.empty() ? fd::FourWheelCar{} : fd::vehicle_profile(loaded_profile_).car;
    return rebuild(simulation_.track(), defaults, fd::SteeringMode::pure_pursuit, simulation_.speed_plan_mode());
}
bool Bridge::setSteeringMode(int mode) {
    if (playback_) { error_ = "Replay shows the recorded steering law; exit replay to change the live one"; publishUpdate(); return false; }
    if (running_) { error_ = "Pause to change the steering law"; publishUpdate(); return false; }
    if (kartRefuses("the steering law")) return false;
    if (mode != 0 && mode != 1) return false;
    if (mode == 1 && !tireSlipModeled()) {
        error_ = "MAP steering needs a dynamic vehicle model; the kinematic bicycle's steering table is its geometry";
        publishUpdate();
        return false;
    }
    if (mode == steeringMode()) return true;
    return rebuild(simulation_.track(), simulation_.vehicle_model(), mode == 1 ? fd::SteeringMode::model_acceleration_pursuit : fd::SteeringMode::pure_pursuit,
                   simulation_.speed_plan_mode());
}
bool Bridge::setSpeedPlanMode(int mode) {
    if (playback_) { error_ = "Replay shows the recorded speed plan; exit replay to change the live one"; publishUpdate(); return false; }
    if (running_) { error_ = "Pause to change the speed plan"; publishUpdate(); return false; }
    if (kartRefuses("the speed plan")) return false;
    if (mode != 0 && mode != 1) return false;
    if (mode == 1 && !wheelsModeled()) {
        error_ = "The envelope plan needs the 4 wheels car; no other model has a derived envelope";
        publishUpdate();
        return false;
    }
    if (mode == speedPlanMode()) return true;
    return rebuild(simulation_.track(), simulation_.vehicle_model(), simulation_.steering_mode(),
                   mode == 1 ? fd::SpeedPlanMode::performance_envelope : fd::SpeedPlanMode::grip_fractions);
}
bool Bridge::setLocalPlanner(int planner) {
    if (playback_) { error_ = "Replay shows the recorded local planner; exit replay to change the live one"; publishUpdate(); return false; }
    if (running_) { error_ = "Pause to change the local planner"; publishUpdate(); return false; }
    if (planner != 0 && planner != 1) return false;
    if (planner == localPlanner()) return true;
    return rebuild(simulation_.track(), simulation_.vehicle_model(), simulation_.steering_mode(), simulation_.speed_plan_mode(),
                   planner == 1 ? fd::LocalPlannerMode::five_offsets : fd::LocalPlannerMode::lattice);
}
bool Bridge::sensorsFitted() const {
    return playback_ ? playback_->recording().sensors_recorded() : simulation_.sensors() != nullptr;
}

void Bridge::setSensors(bool fitted) {
    if (playback_ || running_ || sensorsFitted() == fitted) return;
    // The default instruments, after PacSim's own rates and dead times; the headless runner chooses the seed.
    simulation_.set_sensors(fitted ? std::optional<fd::SensorSettings>{fd::SensorSettings{}} : std::nullopt);
    publishUpdate(false, true);
}

bool Bridge::perceptionFitted() const {
    return playback_ ? playback_->recording().perception_recorded() : simulation_.perception() != nullptr;
}

void Bridge::setPerception(bool fitted) {
    if (playback_ || running_ || perceptionFitted() == fitted) return;
    if (!fitted && simulation_.cone_driver()) { error_ = "The car drives on these cones; drive on the track first"; publishUpdate(); return; }
    simulation_.set_perception(fitted ? std::optional<fd::PerceptionSettings>{fd::PerceptionSettings{}} : std::nullopt);
    publishUpdate(true, true);
}

const std::vector<fd::Cone>& Bridge::courseCones() const {
    return playback_ ? playback_->recording().cones : simulation_.cones();
}

Bridge::TimingView Bridge::timingView() const {
    TimingView view;
    if (!playback_) {
        const auto& judge = simulation_.judge();
        view.laps = judge.laps();
        view.required = judge.laps_required();
        view.last_lap_s = judge.lap_times().empty() ? 0 : judge.lap_times().back();
        view.penalty_s = judge.penalty_s();
        view.cones_hit = judge.cones_hit();
        view.off_course = judge.off_course_count();
        view.off_now = judge.off_course();
        view.finished = judge.finished();
        view.stopped = judge.stopped();
        view.dnf = judge.dnf() ? judge.dnf_reason() : std::string();
        for (std::size_t i = 0; i < judge.hit().size(); ++i) if (judge.hit()[i]) view.hit.push_back(i);
        return view;
    }
    const auto& r = playback_->recording();
    if (!r.competition_recorded()) return view;
    const auto& rules = r.metadata.competition->rules;
    view.required = rules.discipline == fd::Discipline::autocross ? 1 : rules.trackdrive_laps;
    const double now = playback_->sample().time_s;
    for (const auto& e : r.timing) {
        if (e.time_s > now+1e-9) break;
        switch (e.kind) {
        case fd::TimingKind::lap: ++view.laps; view.last_lap_s = e.seconds; break;
        case fd::TimingKind::finish: view.finished = true; break;
        case fd::TimingKind::cone_hit: ++view.cones_hit; view.penalty_s += e.seconds; if (e.cone) view.hit.push_back(*e.cone); break;
        case fd::TimingKind::off_course: ++view.off_course; view.penalty_s += e.seconds; view.off_now = true; break;
        case fd::TimingKind::back_on_course: view.off_now = false; break;
        case fd::TimingKind::stopped: view.stopped = true; break;
        case fd::TimingKind::unsafe_stop: view.penalty_s += e.seconds; view.stopped = true; break;
        case fd::TimingKind::dnf: view.dnf = e.reason; break;
        case fd::TimingKind::start: case fd::TimingKind::sector: break;
        }
    }
    return view;
}

QString Bridge::judgeVerdict() const {
    const auto view = timingView();
    if (!view.dnf.empty()) return "DNF: "+QString::fromStdString(view.dnf);
    if (view.off_now) return "OFF COURSE";
    if (view.finished) return "FINISHED";
    return {};
}

// ---------------------------------------------------------------- the theoretical best lap (decision 0034)

bool Bridge::loadProfile(const QString& id) {
    if (playback_) { error_ = "Replay shows the recorded car; exit replay to choose a physics profile"; publishUpdate(); return false; }
    if (running_) { error_ = "Pause to change the car"; publishUpdate(); return false; }
    if (kartRefuses("the car")) return false;
    try {
        const auto& profile = fd::vehicle_profile(id.toStdString());
        const auto config = fd::configured_for(profile, simulation_.config());
        if (!rebuild(simulation_.track(), profile.car, fd::SteeringMode::pure_pursuit, simulation_.speed_plan_mode(), std::nullopt,
                     std::nullopt, config))
            return false;
        loaded_profile_ = profile.id;
        preset_config_ = fd::configured_for(profile, preset_config_);
    } catch (const std::exception& e) { error_ = QString::fromUtf8(e.what()); publishUpdate(); return false; }
    publishUpdate(true, true);
    return true;
}

std::string Bridge::activeProfile() const {
    const auto* car = std::get_if<fd::FourWheelCar>(&vehicleModelData());
    if (!car) return {};
    if (playback_) return playback_->recording().metadata.vehicle_profile;
    if (loaded_profile_.empty()) return {};
    // Still the profile's own car and configuration, or a setup changed since.
    const auto& profile = fd::vehicle_profile(loaded_profile_);
    const bool same = fd::performance_envelope_fingerprint(*car, config()) ==
                      fd::performance_envelope_fingerprint(profile.car, fd::configured_for(profile, config()));
    return same ? loaded_profile_ : std::string();
}

QString Bridge::profileName() const {
    const auto id = activeProfile();
    if (id.empty()) return {};
    try { return QString::fromStdString(fd::vehicle_profile(id).name); }
    catch (const std::exception&) { return QString::fromStdString(id); }
}

bool Bridge::loadOptimalLap(const QString& directory) {
    try { optimal_ = fd::load_optimal_lap(std::filesystem::path(directory.toStdU16String())); }
    catch (const std::exception& e) { error_ = QString::fromUtf8(e.what()); publishUpdate(); return false; }
    error_.clear();
    publishUpdate(false, true);
    return true;
}

QString Bridge::optimalSource() const {
    if (!optimal_) return {};
    std::ostringstream text;
    text << optimal_->tool << ' ' << optimal_->version << " · " << optimal_->model << " · " << optimal_->mesh_points << " pts · tol "
         << std::setprecision(0) << std::scientific << optimal_->tolerance;
    return QString::fromStdString(text.str());
}

void Bridge::refreshOptimalMatch() {
    optimal_matches_ = false;
    optimal_mismatch_.clear();
    optimal_gate_times_.clear();
    if (!optimal_) return;
    const auto* car = std::get_if<fd::FourWheelCar>(&vehicleModelData());
    if (!car) { optimal_mismatch_ = "The optimum is the 4 wheels car's; this run drives another model"; return; }
    try {
        // The corridor as the headless problem poses it: a course as traced, the preset conditioned as a racing line's
        // reference is. The live car's reference line does not change the corridor, so a racing line run keeps it.
        const fd::Track& shown = playback_ ? playback_->recording().track : line_mode_ == 0 ? simulation_.track() : centreline_;
        std::ostringstream key;
        key << std::setprecision(17) << shown.name << '|' << shown.points.size() << '|' << shown.length_m << '|' << shown.width_m << '|'
            << shown.left_edge_m.size() << '|' << shown.points.front().x_m << '|' << shown.points.front().y_m;
        if (key.str() != optimal_corridor_key_) {
            optimal_corridor_ = shown.left_edge_m.empty() ? fd::condition_track(shown).track : shown;
            optimal_corridor_key_ = key.str();
        }
        const auto problem = fd::make_lap_problem(optimal_corridor_, *car, config(), activeProfile());
        if (fd::lap_problem_fingerprint(problem) == optimal_->problem_fingerprint) {
            optimal_matches_ = true;
            const auto& gates = playback_ ? (playback_->recording().competition_recorded() ? playback_->recording().metadata.competition->gates
                                                                                             : std::vector<fd::Gate>{})
                                          : simulation_.judge().course().gates;
            optimal_gate_times_ = optimal_->gate_times(gates);
            return;
        }
        // Which of the two differs: the solved corridor with this car is the solved problem only if the corridor differs.
        auto with_this_car = optimal_->problem;
        with_this_car.car = *car;
        with_this_car.config = config();
        with_this_car.profile = activeProfile();
        const auto& solved = optimal_->problem.profile.empty() ? std::string("the configured car") : optimal_->problem.profile;
        optimal_mismatch_ = fd::lap_problem_fingerprint(with_this_car) == optimal_->problem_fingerprint
            ? QString::fromStdString("Solved on another corridor than this run's " + optimal_corridor_.name)
            : QString::fromStdString("Solved for " + solved + "; this car or its setup differs");
    } catch (const std::exception& e) { optimal_mismatch_ = QString::fromUtf8(e.what()); }
}

std::vector<fd::TimingEvent> Bridge::timingEvents() const {
    if (!playback_) return simulation_.judge().events();
    std::vector<fd::TimingEvent> events;
    const double now = playback_->sample().time_s;
    for (const auto& e : playback_->recording().timing) {
        if (e.time_s > now+1e-9) break;
        events.push_back(e);
    }
    return events;
}

double Bridge::lapStartTime() const {
    double start = -1;
    for (const auto& e : timingEvents())
        if (e.kind == fd::TimingKind::start || e.kind == fd::TimingKind::lap) start = e.time_s;
        else if (e.kind == fd::TimingKind::finish) start = -1;
    return start;
}

fd::OptimalPoint Bridge::ghost() const {
    if (!optimal_ || !optimal_matches_) return {};
    // Timed from its rear axle crossing the start line, as the judge times the live car.
    const double crossing = optimal_gate_times_.empty() || optimal_gate_times_.front() < 0 ? 0.0 : optimal_gate_times_.front();
    const double start = lapStartTime();
    const double since = start < 0 ? 0.0 : std::min(simulationTime()-start, optimal_->lap_time_s);
    return optimal_->at_time(crossing+since);
}

bool Bridge::optimalDeltaAvailable() const { return optimal_matches_ && lapStartTime() >= 0; }

double Bridge::optimalDeltaSeconds() const {
    if (!optimalDeltaAvailable()) return 0;
    const auto s = state();
    const double rear = std::get<fd::FourWheelCar>(vehicleModelData()).cg_to_rear_m;
    const auto at = fd::project(optimal_->problem.corridor, {s.x_m+rear*std::cos(s.yaw_rad), s.y_m+rear*std::sin(s.yaw_rad)});
    const double crossing = optimal_gate_times_.empty() || optimal_gate_times_.front() < 0 ? 0.0 : optimal_gate_times_.front();
    double optimum = optimal_->time_at_station(at.s_m)-crossing;
    if (optimum < 0) optimum += optimal_->lap_time_s;
    return simulationTime()-lapStartTime()-optimum;
}

bool Bridge::optimalFromRest() const {
    for (const auto& e : timingEvents()) if (e.kind == fd::TimingKind::lap) return false;
    return true;
}

QVariantList Bridge::sectorDeltas() const {
    QVariantList list;
    if (!optimal_matches_ || optimal_gate_times_.size() < 2) return list;
    const auto& g = optimal_gate_times_;
    std::vector<double> optimum;
    for (std::size_t k = 0; k+1 < g.size(); ++k) optimum.push_back(g[k+1]-g[k]);
    optimum.push_back(optimal_->lap_time_s+g.front()-g.back());
    // The live sectors of the lap in progress, and of the lap before for those not yet timed.
    std::vector<double> current, previous;
    for (const auto& e : timingEvents()) {
        if (e.kind == fd::TimingKind::start) current.clear();
        else if (e.kind == fd::TimingKind::sector) current.push_back(e.seconds);
        else if (e.kind == fd::TimingKind::lap) {
            double timed = 0;
            for (const double v : current) timed += v;
            current.push_back(e.seconds-timed);
            previous = current;
            current.clear();
        }
    }
    for (std::size_t k = 0; k < optimum.size(); ++k) {
        QVariantMap row{{"name", QString("S%1").arg(k+1)}, {"optimal", optimum[k]}};
        const bool now = k < current.size(), before = !now && k < previous.size();
        row["timed"] = now || before;
        row["previous"] = before;
        if (now || before) {
            const double live = now ? current[k] : previous[k];
            row["live"] = live;
            row["delta"] = live-optimum[k];
        }
        list.append(row);
    }
    return list;
}

QVariantList Bridge::ghostTireEnergy() const {
    QVariantList list;
    if (!optimal_matches_) return list;
    const auto point = ghost();
    static const char* names[4] = {"FL", "FR", "RL", "RR"};
    for (std::size_t w = 0; w < 4; ++w)
        list.append(QVariantMap{{"name", names[w]}, {"used", point.energy_j[w]/1000}, {"total", optimal_->tire_energy_j[w]/1000}});
    return list;
}

QVariantList Bridge::optimalSensitivities() const {
    QVariantList list;
    if (!optimal_) return list;
    for (const auto& s : optimal_->sensitivities) {
        // The step as a person would quote it.
        QString step;
        const QString sign = s.step > 0 ? "+" : "\u2212";
        const double size = std::abs(s.step);
        if (s.parameter == "mass_kg") step = "Mass " + sign + QString::number(size, 'f', 0) + " kg";
        else if (s.parameter == "cg_height_m") step = "CG height " + sign + QString::number(size*100, 'f', 0) + " cm";
        else if (s.parameter == "brake_bias_front") step = "Brake bias " + sign + QString::number(size*100, 'f', 0) + " %";
        else if (s.parameter == "downforce_area_m2") step = "Downforce " + sign + QString::number(size, 'f', 1) + " m\u00b2";
        else if (s.parameter == "max_drive_power_w") step = "Power " + sign + QString::number(size/1000, 'f', 0) + " kW";
        else step = QString::fromStdString(s.parameter) + " " + sign + QString::number(size);
        list.append(QVariantMap{{"step", step}, {"worth", s.seconds_per_unit*s.step}, {"method", QString::fromStdString(s.method)},
                                {"checked", s.cross_check_seconds_per_unit.has_value()}});
    }
    return list;
}

bool Bridge::loadCourse(const QString& path) {
    if (playback_) { error_ = "Exit replay to drive a course"; publishUpdate(); return false; }
    if (kartRefuses("the course")) return false;
    if (running_) { error_ = "Pause to change the course"; publishUpdate(); return false; }
    try {
        const auto layout = fd::load_pacsim_course(std::filesystem::path(path.toStdU16String()));
        const auto course = fd::course_from_layout(layout);
        auto config = simulation_.config();
        if (pending_grip_) config.grip_mu = *pending_grip_;
        config.wheelbase_m = 1.53;
        config.max_steering_rad = 0.6;
        config.lookahead_base_m = 2.5;
        fd::Simulation fresh(course.track, config, fd::KinematicBicycle{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                             simulation_.local_planner_mode());
        fresh.set_course(course.cones, course.gates, "PacSim layout "+layout.name);
        fresh.set_competition(simulation_.judge().rules());
        if (const auto* sensors = simulation_.sensors()) fresh.set_sensors(sensors->settings());
        if (const auto* perception = simulation_.perception()) fresh.set_perception(perception->settings());
        simulation_ = std::move(fresh);
        centreline_ = course.track;
        course_name_ = QString::fromStdString(layout.name);
        line_mode_ = 0;
    } catch (const std::exception& e) { error_ = QString::fromUtf8(e.what()); publishUpdate(); return false; }
    pending_grip_.reset();
    accumulator_ = 0;
    error_.clear();
    publishUpdate(true, true);
    return true;
}

bool Bridge::loadPreset() {
    if (course_name_.isEmpty()) return true;
    if (playback_) { error_ = "Exit replay to drive the preset"; publishUpdate(); return false; }
    if (kartRefuses("the course")) return false;
    if (running_) { error_ = "Pause to change the course"; publishUpdate(); return false; }
    try {
        auto config = preset_config_;
        config.grip_mu = pending_grip_.value_or(simulation_.config().grip_mu);
        fd::Simulation fresh(preset_, config, fd::KinematicBicycle{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                             simulation_.local_planner_mode());
        fresh.set_competition(simulation_.judge().rules());
        if (const auto* sensors = simulation_.sensors()) fresh.set_sensors(sensors->settings());
        if (const auto* perception = simulation_.perception()) fresh.set_perception(perception->settings());
        simulation_ = std::move(fresh);
        centreline_ = preset_;
        course_name_.clear();
        line_mode_ = 0;
    } catch (const std::exception& e) { error_ = QString::fromUtf8(e.what()); publishUpdate(); return false; }
    pending_grip_.reset();
    accumulator_ = 0;
    error_.clear();
    publishUpdate(true, true);
    return true;
}

bool Bridge::drivingOnCones() const {
    return playback_ ? playback_->recording().cone_driving_recorded() : simulation_.cone_driver() != nullptr;
}

void Bridge::setDriveOnCones(bool on) {
    if (playback_ || running_ || drivingOnCones() == on) return;
    if (!on) {
        // Back on the known track: the preset as it was, the run started over.
        simulation_.set_cone_driving(std::nullopt);
        rebuild(centreline_, simulation_.vehicle_model(), simulation_.steering_mode(), simulation_.speed_plan_mode());
        return;
    }
    if (kart_mode_) { error_ = "The gokart track has barriers, not cones: leave gokart mode to drive on cones"; publishUpdate(); return; }
    if (!simulation_.perception()) { error_ = "Give the car cone perception first: it drives on the cones it perceives"; publishUpdate(); return; }
    if (line_mode_ == 1) { error_ = "The cones are laid along the centreline's corridor; drive the centreline to drive on cones"; publishUpdate(); return; }
    if (!simulation_.obstructions().empty()) { error_ = "Driving on cones takes no blockages; clear them first"; publishUpdate(); return; }
    if (simulation_.controller_mode() != fd::ControllerMode::policy) {
        error_ = "Driving on cones is by the policy: MPCC plans along the known track";
        publishUpdate();
        return;
    }
    // FaSTTUBe's planner is tuned for Formula Student courses, 3 to 5 m wide (decision 0031), and finds no start between
    // cones 10 m apart: on cones the preset is laid to 5 m about its centreline.
    auto course = centreline_;
    // A traced layout's corridor is already its cones'; only the preset is laid narrower.
    if (course.left_edge_m.empty()) course.width_m = std::min(course.width_m, cone_course_width_m);
    if (!rebuild(course, simulation_.vehicle_model(), simulation_.steering_mode(), simulation_.speed_plan_mode())) return;
    try { simulation_.set_cone_driving(fd::ConePathSettings{}); }
    catch (const std::exception& e) { error_ = QString::fromUtf8(e.what()); publishUpdate(); return; }
    publishUpdate(true, true);
}

QString Bridge::beliefSource() const {
    if (!drivingOnCones()) return {};
    const auto source = playback_ ? playback_->recording().metadata.cone_driving->source : simulation_.belief_source();
    return source == fd::BeliefSource::measured ? "MEASURED" : "IDEAL";
}

Bridge::BelievedView Bridge::believedView() const {
    BelievedView view;
    const fd::OpenPath* path = nullptr;
    if (!playback_) {
        const auto* driver = simulation_.cone_driver();
        if (!driver) return view;
        view.believed = driver->belief().state;
        view.has_belief = true;
        if (const auto* believed = driver->path()) { path = &believed->path; view.sampled_at_s = believed->pose.time_s; }
    } else {
        const auto& r = playback_->recording();
        const auto* decision = playback_->decision();
        if (!r.cone_driving_recorded() || !decision || r.decisions.empty()) return view;
        const auto d = static_cast<std::size_t>(decision-r.decisions.data());
        if (d >= r.beliefs.size()) return view;
        view.believed = r.beliefs[d].state;
        view.has_belief = true;
        if (const auto index = r.beliefs[d].path) {
            const auto& believed = r.believed_paths[*index];
            path = &believed.path;
            view.sampled_at_s = believed.pose.time_s;
        }
    }
    if (path) {
        view.width_m = path->width_m;
        for (const auto& p : path->points) view.path.push_back({p.x_m, p.y_m});
    }
    return view;
}

double Bridge::believedPathAheadM() const {
    const auto view = believedView();
    if (view.path.size() < 2) return 0;
    // How much of the believed path lies ahead of where the car believed it was.
    fd::OpenPath path;
    path.width_m = std::max(view.width_m, 1e-3);
    double s = 0;
    for (std::size_t k = 0; k < view.path.size(); ++k) {
        if (k) s += std::hypot(view.path[k].x-view.path[k-1].x, view.path[k].y-view.path[k-1].y);
        path.points.push_back({view.path[k].x, view.path[k].y, s, 0});
    }
    const auto at = fd::project_open(path, {view.believed.x_m, view.believed.y_m});
    return std::max(0.0, s-at.s_m);
}

double Bridge::believedPathAgeMs() const {
    const auto view = believedView();
    if (view.sampled_at_s < 0) return 0;
    const double now = playback_ ? playback_->sample().time_s : simulation_.state().time_s;
    return (now-view.sampled_at_s)*1000;
}

double Bridge::beliefErrorM() const {
    const auto view = believedView();
    if (!view.has_belief) return 0;
    if (!playback_) {
        // Live, what the driver would believe now, from what it has read, against where the car is now.
        const auto now = simulation_.cone_driver()->believed_at(simulation_.state().time_s);
        return std::hypot(now.x_m-simulation_.state().x_m, now.y_m-simulation_.state().y_m);
    }
    // Recorded, the belief of the decision in force against the sample of the same time, the one it started from.
    const auto& samples = playback_->recording().samples;
    const auto at = std::lower_bound(samples.begin(), samples.end(), view.believed.time_s-1e-9,
                                     [](const fd::RecordedSample& s, double t) { return s.time_s < t; });
    if (at == samples.end()) return 0;
    return std::hypot(view.believed.x_m-at->x_m, view.believed.y_m-at->y_m);
}

const std::vector<fd::Cone>& Bridge::perceivedCones() const {
    static const std::vector<fd::Cone> none;
    if (playback_) return playback_->recording().cones;
    return simulation_.perception() ? simulation_.perception()->cones() : none;
}

Bridge::PerceptionView Bridge::perceptionView() const {
    PerceptionView view;
    const auto& cones = perceivedCones();
    const auto place = [&](const fd::PerceptionFrame& frame, const fd::State& pose, const std::vector<std::size_t>& source,
                           const std::vector<std::size_t>& missed) {
        const double c = std::cos(pose.yaw_rad), s = std::sin(pose.yaw_rad);
        for (std::size_t k = 0; k < frame.detections.size(); ++k) {
            const auto& d = frame.detections[k];
            DrawnDetection drawn;
            drawn.position = {pose.x_m+c*d.position.x-s*d.position.y, pose.y_m+s*d.position.x+c*d.position.y};
            drawn.colour = d.colour;
            drawn.wrong_colour = d.colour != fd::ObservedColour::unknown && k < source.size() && source[k] < cones.size() &&
                                 d.colour != fd::observed(cones[source[k]].colour);
            drawn.covariance_xx = c*c*d.covariance_xx-2*c*s*d.covariance_xy+s*s*d.covariance_yy;
            drawn.covariance_xy = c*s*(d.covariance_xx-d.covariance_yy)+(c*c-s*s)*d.covariance_xy;
            drawn.covariance_yy = s*s*d.covariance_xx+2*c*s*d.covariance_xy+c*c*d.covariance_yy;
            view.detections.push_back(drawn);
        }
        view.missed.insert(view.missed.end(), missed.begin(), missed.end());
        view.sampled_at_s = std::max(view.sampled_at_s, frame.sampled_at_s);
    };
    if (!playback_) {
        const auto* perception = simulation_.perception();
        if (!perception) return view;
        for (std::size_t i = 0; i < perception->settings().sensors.size(); ++i)
            if (const auto* frame = perception->latest(i)) {
                const auto* truth = perception->latest_truth(i);
                place(*frame, truth->pose, truth->source_cone, truth->missed);
            }
        return view;
    }
    // In replay, each sensor's newest frame delivered by the cursor, from the recorded pose it was sampled at.
    const auto& r = playback_->recording();
    const double now = playback_->sample().time_s;
    std::vector<const fd::RecordedPerceptionFrame*> newest;
    for (const auto& f : r.perception_frames) {
        if (f.time_s > now+1e-9) break;
        if (newest.size() <= f.frame.sensor) newest.resize(f.frame.sensor+1, nullptr);
        newest[f.frame.sensor] = &f;
    }
    for (const auto* f : newest) {
        if (!f) continue;
        const auto at = std::lower_bound(r.samples.begin(), r.samples.end(), f->frame.sampled_at_s-1e-9,
                                         [](const fd::RecordedSample& s, double t) { return s.time_s < t; });
        if (at == r.samples.end()) continue;
        place(f->frame, at->state(), f->source_cone, f->missed);
    }
    return view;
}

int Bridge::perceivedWrongColour() const {
    const auto view = perceptionView();
    return static_cast<int>(std::count_if(view.detections.begin(), view.detections.end(),
                                          [](const DrawnDetection& d) { return d.wrong_colour; }));
}

double Bridge::perceptionFrameAgeMs() const {
    const auto view = perceptionView();
    if (view.sampled_at_s < 0) return 0;
    const double now = playback_ ? playback_->sample().time_s : simulation_.state().time_s;
    return (now-view.sampled_at_s)*1000;
}

const fd::Measurements* Bridge::measured() const {
    if (!playback_) return simulation_.measurements();
    const auto& recorded = playback_->recording().measurements;
    return playback_->index() < recorded.size() ? &recorded[playback_->index()].measured : nullptr;
}

bool Bridge::measuredAvailable() const {
    const auto* m = measured();
    return m && m->pose.measured;
}

double Bridge::measuredAgeMs() const {
    const auto* m = measured();
    if (!m || !m->pose.measured) return 0;
    const double now = playback_ ? playback_->sample().time_s : simulation_.state().time_s;
    return (now-m->pose.sampled_at_s)*1000;
}

double Bridge::measuredPositionError() const {
    const auto* m = measured();
    if (!m || !m->pose.measured) return 0;
    const auto& truth = playback_ ? playback_->sample().state() : simulation_.state();
    return std::hypot(m->x_m-truth.x_m, m->y_m-truth.y_m);
}

double Bridge::measuredSpeedKmh() const {
    const auto* m = measured();
    return m && m->speed.measured ? m->speed_mps*3.6 : 0;
}

double Bridge::measuredSpeedErrorKmh() const {
    const auto* m = measured();
    if (!m || !m->speed.measured) return 0;
    const double truth = playback_ ? playback_->sample().speed_mps : simulation_.state().speed_mps;
    return (m->speed_mps-truth)*3.6;
}

double Bridge::measuredYawRateErrorDegrees() const {
    const auto* m = measured();
    if (!m || !m->imu.measured) return 0;
    const double truth = playback_ ? playback_->sample().yaw_rate_radps : simulation_.plant_state().yaw_rate_radps;
    return (m->yaw_rate_radps-truth)*180/std::numbers::pi;
}

bool Bridge::mpccAvailable() const {
#ifdef FD_HAVE_MPCC
    return true;
#else
    return false;
#endif
}
bool Bridge::setControl(int control) {
    if (playback_) { error_ = "Replay shows the recorded controller; exit replay to change the live one"; publishUpdate(); return false; }
    if (running_) { error_ = "Pause to change what drives the car"; publishUpdate(); return false; }
    if (kartRefuses("what drives the car")) return false;
    if (control < 0 || control > 2) return false;
    if (control == 2 && !mpccAvailable()) {
        error_ = "This build has no MPCC: run scripts/bootstrap-osqp.ps1, then build again";
        publishUpdate();
        return false;
    }
    if (control == 2 && !tireSlipModeled()) {
        error_ = "MPCC predicts with tire forces; the kinematic bicycle has none, so choose a car with tires";
        publishUpdate();
        return false;
    }
    if (control == 1) {
        // MAP's conditions are setSteeringMode's; it leaves the controller alone, so the policy is chosen first.
        if (controllerMode() == 1 && !rebuild(simulation_.track(), simulation_.vehicle_model(), simulation_.steering_mode(),
                                               simulation_.speed_plan_mode(), std::nullopt, fd::ControllerMode::policy)) return false;
        return setSteeringMode(1);
    }
    const auto controller = control == 2 ? fd::ControllerMode::mpcc : fd::ControllerMode::policy;
    if (steeringMode() == 0 && controllerMode() == (control == 2 ? 1 : 0)) return true;
    return rebuild(simulation_.track(), simulation_.vehicle_model(), fd::SteeringMode::pure_pursuit, simulation_.speed_plan_mode(),
                   std::nullopt, controller);
}
QString Bridge::controllerStatus() const {
    if (controllerMode() == 0) return {};
    if (playback_) {
        const auto* decision = recordedDecision();
        if (!decision) return QStringLiteral("MPCC run; no recorded decision here");
        if (decision->driven_by == fd::ControllerMode::mpcc)
            return QStringLiteral("MPCC drove · %1 in %2 steps").arg(QString::fromUtf8(fd::controller_status_name(*decision->controller_status)))
                                                                .arg(decision->controller_iterations);
        return QString::fromStdString(decision->controller_note);
    }
    const auto& outcome = simulation_.controller_outcome();
    if (outcome.driven_by == fd::ControllerMode::mpcc)
        return QStringLiteral("MPCC drives · %1 in %2 steps · %3 ms").arg(QString::fromUtf8(fd::controller_status_name(outcome.status)))
                                                                      .arg(outcome.iterations).arg(outcome.solve_time_s*1000, 0, 'f', 1);
    return QString::fromStdString(outcome.note);
}
bool Bridge::racingLineAvailable() const {
#ifdef FD_HAVE_RACELINE
    return true;
#else
    return false;
#endif
}
bool Bridge::setLineMode(int mode) {
    if (playback_) { error_ = "Replay shows the recorded track; exit replay to choose the live line"; publishUpdate(); return false; }
    if (running_) { error_ = "Pause to change the line"; publishUpdate(); return false; }
    if (mode != 0 && mode != 1) return false;
    if (mode == line_mode_) return true;
    if (mode == 1 && !course_name_.isEmpty()) {
        error_ = "The racing line was solved for the preset; the course "+course_name_+" is driven on its centreline";
        publishUpdate();
        return false;
    }
    if (mode == 1 && !racingLineReady()) {
        error_ = !racingLineAvailable() ? "This build has no racing line: run scripts/bootstrap-osqp.ps1 and build again"
               : line_solution_ ? "The racing line could not be solved: "+QString::fromStdString(line_solution_->failure)
                                : "The racing line is still being solved";
        publishUpdate();
        return false;
    }
    if (mode == 1 && simulation_.cone_driver()) {
        error_ = "The car drives on cones laid along the centreline; drive on the track to drive the racing line";
        publishUpdate();
        return false;
    }
    if (mode == 1 && !simulation_.obstructions().empty()) {
        error_ = "Stated blockages are placed across the centreline; clear them to drive the racing line";
        publishUpdate();
        return false;
    }
    const int previous = line_mode_;
    line_mode_ = mode;
    if (rebuild(mode == 1 ? line_solution_->line : centreline_, simulation_.vehicle_model(), simulation_.steering_mode(), simulation_.speed_plan_mode()))
        return true;
    line_mode_ = previous;
    publishUpdate(false, true);
    return false;
}
bool Bridge::ensureLattice() {
    const auto& t = track();
    std::string key = t.name+"|"+std::to_string(t.length_m)+"|"+std::to_string(t.points.size())+"|"+std::to_string(t.left_edge_m.size());
    if (!t.points.empty()) key += "|"+std::to_string(t.points.front().x_m)+","+std::to_string(t.points.front().y_m);
    if (key == lattice_track_) return false;
    lattice_track_ = key;
    try { lattice_ = fd::make_lattice(t, config()); }
    catch (const std::exception&) { lattice_.reset(); }
    return true;
}
std::vector<std::size_t> Bridge::latticeLayersAhead() const {
    std::vector<std::size_t> ahead;
    if (!lattice_ || lattice_->layers.empty()) return ahead;
    const auto& layers = lattice_->layers;
    const double length = track().length_m;
    const double here = std::fmod(std::fmod(playback_ ? playback_->sample().path_s_m : simulation_.diagnostics().path_s_m, length)+length, length);
    const auto first = std::lower_bound(layers.begin(), layers.end(), here, [](const fd::LatticeLayer& l, double s) { return l.s_m < s; });
    std::size_t index = first == layers.end() ? 0 : static_cast<std::size_t>(first-layers.begin());
    for (std::size_t k = 0; k < layers.size(); ++k, index = (index+1)%layers.size()) {
        if (std::fmod(layers[index].s_m-here+length, length) > lattice_horizon_m) break;
        ahead.push_back(index);
    }
    return ahead;
}
bool Bridge::takeRacingLine() {
    std::lock_guard lock(line_mutex_);
    if (!line_finished_ || line_finished_ == line_solution_) return false;
    line_solution_ = line_finished_;
    return true;
}
void Bridge::solveRacingLineNow() {
    if (line_worker_.joinable()) line_worker_.join();
    publishUpdate();
}
void Bridge::refreshLineEstimates() {
    line_lap_s_ = centreline_lap_s_ = smoothed_lap_s_ = 0;
    comparison_line_.clear();
    if (!racingLineReady()) return;
    const auto& drawn = line_mode_ == 1 ? centreline_ : line_solution_->line;
    for (const auto& point : drawn.points) comparison_line_.push_back({point.x_m, point.y_m});
    if (!comparison_line_.empty()) comparison_line_.push_back(comparison_line_.front());
    // The plan in force: its configuration and, under the envelope plan, the envelope the simulation derived for it.
    const auto lap = [&](const fd::Track& t) {
        return fd::estimated_lap_time(t, fd::make_speed_plan(t, simulation_.config(), simulation_.performance_envelope()));
    };
    try {
        line_lap_s_ = lap(line_solution_->line);
        centreline_lap_s_ = lap(centreline_);
        smoothed_lap_s_ = lap(line_solution_->smoothed_centreline);
    } catch (const std::exception&) { line_lap_s_ = centreline_lap_s_ = smoothed_lap_s_ = 0; }
}
QString Bridge::lineSummary() const {
    if (playback_) return "Recorded track: "+QString::fromStdString(track().name);
    if (!course_name_.isEmpty())
        return QString("Course %1, a PacSim layout: %2 m, %3 cones, driven with a Formula Student car's wheelbase and lock")
            .arg(course_name_).arg(track().length_m, 0, 'f', 0).arg(simulation_.cones().size());
    if (!racingLineAvailable()) return "No racing line in this build: run scripts/bootstrap-osqp.ps1";
    if (!line_solution_) return "Solving the racing line…";
    if (!racingLineReady()) return "No racing line: "+QString::fromStdString(line_solution_->failure);
    if (line_lap_s_ <= 0) return "Racing line solved";
    return QString("Plan lap: racing line %1 s, centreline %2 s (%3 s smoothed)%4")
        .arg(line_lap_s_, 0, 'f', 2).arg(centreline_lap_s_, 0, 'f', 2).arg(smoothed_lap_s_, 0, 'f', 2)
        .arg(line_solution_->converged ? QString() : QString("; line not converged"));
}
QVariantList Bridge::comparisonLinePoints() const {
    QVariantList list;
    for (const auto& point : comparisonLine()) list.append(QVariantMap{{"x", point.x}, {"y", point.y}});
    return list;
}
double Bridge::brakeBound() const {
    if (speedPlanMode() == 1) {
        const auto* plan_envelope = playback_ ? envelope() : simulation_.performance_envelope();
        if (plan_envelope) return config().envelope_fraction*std::max(0.0, -plan_envelope->braking_limit(state().speed_mps, 0));
    }
    return config().grip_mu*fd::gravity_mps2*config().longitudinal_grip_fraction;
}
void Bridge::clearBlockages() {
    if (playback_) return;
    simulation_.set_obstructions({});
    error_.clear();
    publishUpdate(false, true);
}
QVariantList Bridge::eventTimes() const {
    QVariantList list;
    if (playback_) for (const auto& event : playback_->recording().events) list.append(event.time_s);
    return list;
}
void Bridge::refreshPathPoints() {
    QVariantList list;
    const auto& t = track();
    const auto& p = plan();
    list.reserve(qsizetype(t.points.size()));
    for (size_t i=0; i<t.points.size(); ++i) {
        const auto& point=t.points[i]; const auto& v=p[i];
        list.append(QVariantMap{{"x",point.x_m},{"y",point.y_m},{"s",point.s_m},{"speed",v.speed_mps},{"acceleration",v.acceleration_mps2}});
    }
    path_points_=std::move(list);
}
void Bridge::refreshPrediction() {
    prediction_.clear();
    prediction_points_.clear();
    std::vector<fd::TrajectorySample> recorded;
    if (playback_) {
        // Only the rollout recorded with the decision is shown; schema 1 recorded none, and
        // replay never computes a forecast of its own. Recorded points carry no yaw or steering.
        const auto* decision = recordedDecision();
        if (!decision) return;
        for (const auto& point : decision->choice().trajectory)
            recorded.push_back({{point.x_m, point.y_m, 0, point.speed_mps, 0, point.time_s}, point.acceleration_mps2, 0});
    }
    const auto now = state();
    const auto& source = playback_ ? recorded : simulation_.trajectory().points;
    const auto future = std::find_if(source.begin(), source.end(), [&](const auto& p) {
        return p.state.time_s > now.time_s + 1e-9;
    });
    if (future == source.end()) return;
    // The active rollout can be one control interval old. Never draw its expired tail
    // behind the vehicle; use the actual rear-axle state for the first visible point.
    prediction_.reserve(static_cast<std::size_t>(source.end()-future)+1);
    const double remaining_acceleration = (future->state.speed_mps-now.speed_mps)/
                                          (future->state.time_s-now.time_s);
    prediction_.push_back({now, remaining_acceleration, 0.0});
    for (auto it = future; it != source.end(); ++it) {
        auto point = *it;
        const auto& previous = prediction_.back();
        point.distance_m = previous.distance_m + std::hypot(point.state.x_m-previous.state.x_m, point.state.y_m-previous.state.y_m);
        prediction_.push_back(point);
    }
    prediction_points_.reserve(static_cast<qsizetype>(prediction_.size()));
    for (const auto& point : prediction_) {
        prediction_points_.append(QVariantMap{{"x", point.state.x_m}, {"y", point.state.y_m},
            {"time", point.state.time_s-now.time_s}, {"speed", point.state.speed_mps},
            {"acceleration", point.acceleration_mps2}, {"distance", point.distance_m}});
    }
}
void Bridge::toggleRunning() {
    error_.clear();
    // Playing a finished replay starts it again from the first sample.
    if (playback_ && !running_ && playback_->at_end()) seek(0);
    running_=!running_; elapsed_.restart(); accumulator_=0; publishUpdate();
}
void Bridge::reset() {
    running_=false; accumulator_=0; error_.clear();
    if (playback_) { seek(0); return; }
    simulation_.reset();
    if (practice_laps_) practice_laps_->reset();
    pending_grip_.reset(); publishUpdate(true);
}
void Bridge::setGrip(double value) {
    if (playback_) { error_="Replay shows the recorded grip; exit replay to change the live simulation"; publishUpdate(); return; }
    try { simulation_.set_grip(value); pending_grip_=value; error_.clear(); }
    catch(const std::exception& e) { error_=QString::fromUtf8(e.what()); publishUpdate(); return; }
    // In Ready/Paused, the next tick occurs on resume; preserve physical time and pose.
    publishUpdate();
}
void Bridge::setAppearance(int value) {
    if (!running_ && value>=0 && value<=3) { appearance_=value; publishUpdate(); }
}
void Bridge::setOverview(bool value) { setCameraMode(value ? 1 : 0); }
void Bridge::setCameraMode(int value) {
    if (value<0 || value>2) return;
    camera_mode_=value; overview_=value==1; publishUpdate();
}
bool Bridge::loadRecording(const QString& location) {
    QString path = location;
    const QUrl url(location);
    if (url.isLocalFile()) path = url.toLocalFile();
    try {
        playback_.emplace(fd::load_recording(std::filesystem::path(path.toStdU16String())));
    } catch (const std::exception& e) {
        error_ = QString::fromUtf8(e.what());
        publishUpdate();
        return false;
    }
    running_ = false; accumulator_ = 0; pending_grip_.reset(); error_.clear();
    recording_name_ = QDir(path).dirName();
    publishUpdate(true, true);
    return true;
}
void Bridge::exitReplay() {
    if (!playback_) return;
    playback_.reset();
    recording_name_.clear();
    running_ = false; accumulator_ = 0; error_.clear();
    publishUpdate(true, true);
}
void Bridge::seek(double time_s) {
    if (!playback_ || !std::isfinite(time_s)) return;
    const int previous_revision = revision();
    playback_->seek_time(time_s);
    publishUpdate(previous_revision != revision());
}
