#pragma once
#include "assistance_presets.hpp"
#include "fd/lattice.hpp"
#include "fd/optimal_lap.hpp"
#include "fd/performance_envelope.hpp"
#include "fd/recording.hpp"
#include "fd/setup.hpp"
#include "fd/trajectory.hpp"
#include "gamepad.hpp"
#include <QColor>
#include <QObject>
#include <QTimer>
#include <QElapsedTimer>
#include <QVariantList>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <numbers>
#include <optional>

// Presents either the live simulation or a loaded recording to QML. Only the live
// simulation advances the plant; replay moves a cursor over recorded samples and shows
// exactly the plan revision and configuration each sample was produced under.
class Bridge : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool running READ running NOTIFY updated)
    Q_PROPERTY(double speedKmh READ speedKmh NOTIFY updated)
    Q_PROPERTY(double targetSpeedKmh READ targetSpeedKmh NOTIFY updated)
    Q_PROPERTY(double grip READ grip NOTIFY updated)
    Q_PROPERTY(double simulationTime READ simulationTime NOTIFY updated)
    Q_PROPERTY(double x READ x NOTIFY updated)
    Q_PROPERTY(double y READ y NOTIFY updated)
    Q_PROPERTY(double yawDegrees READ yawDegrees NOTIFY updated)
    Q_PROPERTY(double steeringDegrees READ steeringDegrees NOTIFY updated)
    Q_PROPERTY(double progress READ progress NOTIFY updated)
    Q_PROPERTY(double lapTime READ simulationTime NOTIFY updated)
    Q_PROPERTY(double practiceLapSeconds READ practiceLapSeconds NOTIFY updated)
    Q_PROPERTY(double previousPracticeLap READ previousPracticeLap NOTIFY updated)
    Q_PROPERTY(double bestPracticeLap READ bestPracticeLap NOTIFY updated)
    Q_PROPERTY(int practiceLaps READ practiceLaps NOTIFY updated)
    Q_PROPERTY(double crossTrackError READ crossTrackError NOTIFY updated)
    Q_PROPERTY(double brakeBound READ brakeBound NOTIFY updated)
    Q_PROPERTY(double cornerSpeedKmh READ cornerSpeedKmh NOTIFY updated)
    Q_PROPERTY(double cornerDistance READ cornerDistance NOTIFY updated)
    Q_PROPERTY(double plannedAcceleration READ plannedAcceleration NOTIFY updated)
    Q_PROPERTY(double utilization READ utilization NOTIFY updated)
    Q_PROPERTY(double planColorDeadband READ planColorDeadband CONSTANT)
    Q_PROPERTY(double trackLength READ trackLength NOTIFY planChanged)
    Q_PROPERTY(double constraintStation READ constraintStation NOTIFY updated)
    Q_PROPERTY(int revision READ revision NOTIFY updated)
    Q_PROPERTY(int lap READ lap NOTIFY updated)
    Q_PROPERTY(QString status READ status NOTIFY updated)
    Q_PROPERTY(QString reason READ reason NOTIFY updated)
    Q_PROPERTY(QVariantList pathPoints READ pathPoints NOTIFY planChanged)
    Q_PROPERTY(QVariantList predictionPoints READ predictionPoints NOTIFY updated)
    Q_PROPERTY(bool predictionAvailable READ predictionAvailable NOTIFY updated)
    Q_PROPERTY(double predictionSeconds READ predictionSeconds NOTIFY updated)
    Q_PROPERTY(double predictionDistance READ predictionDistance NOTIFY updated)
    Q_PROPERTY(bool predictionWithinEnvelope READ predictionWithinEnvelope NOTIFY updated)
    Q_PROPERTY(QString predictionStatus READ predictionStatus NOTIFY updated)
    Q_PROPERTY(int carAppearance READ carAppearance WRITE setAppearance NOTIFY updated)
    Q_PROPERTY(bool overview READ overview WRITE setOverview NOTIFY updated)
    Q_PROPERTY(int cameraMode READ cameraMode WRITE setCameraMode NOTIFY updated)
    Q_PROPERTY(bool replay READ replay NOTIFY modeChanged)
    Q_PROPERTY(QString recordingName READ recordingName NOTIFY modeChanged)
    Q_PROPERTY(double playheadTime READ playheadTime NOTIFY updated)
    Q_PROPERTY(double recordingDuration READ recordingDuration NOTIFY modeChanged)
    Q_PROPERTY(QVariantList eventTimes READ eventTimes NOTIFY modeChanged)
    Q_PROPERTY(int appliedEvents READ appliedEvents NOTIFY updated)
    Q_PROPERTY(int recordedEvents READ recordedEvents NOTIFY modeChanged)
    Q_PROPERTY(int obstructionCount READ obstructionCount NOTIFY modeChanged)
    Q_PROPERTY(bool decisionAvailable READ decisionAvailable NOTIFY updated)
    Q_PROPERTY(QString decisionReason READ decisionReason NOTIFY updated)
    Q_PROPERTY(bool holdingForBlockage READ holdingForBlockage NOTIFY updated)
    Q_PROPERTY(QString blockingIdentifier READ blockingIdentifier NOTIFY updated)
    Q_PROPERTY(int evaluatedOptions READ evaluatedOptions NOTIFY updated)
    Q_PROPERTY(int rejectedOptions READ rejectedOptions NOTIFY updated)
    Q_PROPERTY(double selectedOffset READ selectedOffset NOTIFY updated)
    Q_PROPERTY(int localPlanner READ localPlanner NOTIFY modeChanged)
    Q_PROPERTY(int controllerMode READ controllerMode NOTIFY modeChanged)
    // The car's own instruments (decision 0029). Nothing they measure drives anything: the panel shows how far what the
    // car could see of itself is from what it is.
    Q_PROPERTY(bool sensorsFitted READ sensorsFitted NOTIFY modeChanged)
    Q_PROPERTY(bool measuredAvailable READ measuredAvailable NOTIFY updated)
    Q_PROPERTY(double measuredAgeMs READ measuredAgeMs NOTIFY updated)
    Q_PROPERTY(double measuredPositionError READ measuredPositionError NOTIFY updated)
    Q_PROPERTY(double measuredSpeedKmh READ measuredSpeedKmh NOTIFY updated)
    Q_PROPERTY(double measuredSpeedErrorKmh READ measuredSpeedErrorKmh NOTIFY updated)
    Q_PROPERTY(double measuredYawRateErrorDegrees READ measuredYawRateErrorDegrees NOTIFY updated)
    // Simulated cone perception (decision 0030): whether the car perceives the course's cones, live or as recorded, and
    // what its newest frame made of them. Nothing drives on it.
    Q_PROPERTY(bool perceptionFitted READ perceptionFitted NOTIFY modeChanged)
    Q_PROPERTY(bool perceptionFrameAvailable READ perceptionFrameAvailable NOTIFY updated)
    Q_PROPERTY(int perceivedDetections READ perceivedDetections NOTIFY updated)
    Q_PROPERTY(int perceivedMissed READ perceivedMissed NOTIFY updated)
    Q_PROPERTY(int perceivedWrongColour READ perceivedWrongColour NOTIFY updated)
    Q_PROPERTY(double perceptionFrameAgeMs READ perceptionFrameAgeMs NOTIFY updated)
    // Driving on cones (decision 0031): whether the car drives on the path it believes from its own detections, live or
    // as recorded, and what it believed: how far its path runs ahead of it, how old the frame it came from is, and how
    // far the car believed itself from where it truly was, which only this evaluation can say.
    Q_PROPERTY(bool drivingOnCones READ drivingOnCones NOTIFY modeChanged)
    Q_PROPERTY(QString beliefSource READ beliefSource NOTIFY modeChanged)
    Q_PROPERTY(bool believedPathAvailable READ believedPathAvailable NOTIFY updated)
    Q_PROPERTY(double believedPathAheadM READ believedPathAheadM NOTIFY updated)
    Q_PROPERTY(double believedPathAgeMs READ believedPathAgeMs NOTIFY updated)
    Q_PROPERTY(double beliefErrorM READ beliefErrorM NOTIFY updated)
    // How the run is judged (decision 0032), live or as recorded: the last lap's time, the penalties and what they were
    // for, and the judge's verdict.
    Q_PROPERTY(double lastLapSeconds READ lastLapSeconds NOTIFY updated)
    Q_PROPERTY(double penaltySeconds READ penaltySeconds NOTIFY updated)
    Q_PROPERTY(int conesHit READ conesHit NOTIFY updated)
    Q_PROPERTY(int offCourseCount READ offCourseCount NOTIFY updated)
    Q_PROPERTY(QString judgeVerdict READ judgeVerdict NOTIFY updated)
    // The theoretical best lap (TrackWayFastPlan Phase 4.3, decision 0034): a reference-optimal artefact loaded with
    // --optimal-lap, replayed as a translucent car from its own mesh by the time since the live car's lap began, and
    // shown only while it is the optimum of the car and corridor on screen; otherwise the card says why not.
    Q_PROPERTY(bool optimalLoaded READ optimalLoaded NOTIFY modeChanged)
    Q_PROPERTY(bool optimalMatches READ optimalMatches NOTIFY modeChanged)
    Q_PROPERTY(bool optimalConverged READ optimalConverged NOTIFY modeChanged)
    Q_PROPERTY(QString optimalStatus READ optimalStatus NOTIFY modeChanged)
    Q_PROPERTY(QString optimalSource READ optimalSource NOTIFY modeChanged)
    Q_PROPERTY(QString optimalMismatch READ optimalMismatch NOTIFY modeChanged)
    Q_PROPERTY(double optimalLapSeconds READ optimalLapSeconds NOTIFY modeChanged)
    Q_PROPERTY(QVariantList optimalSensitivities READ optimalSensitivities NOTIFY modeChanged)
    Q_PROPERTY(double ghostWheelbase READ ghostWheelbase NOTIFY modeChanged)
    Q_PROPERTY(bool ghostVisible READ ghostVisible NOTIFY updated)
    Q_PROPERTY(double ghostX READ ghostX NOTIFY updated)
    Q_PROPERTY(double ghostY READ ghostY NOTIFY updated)
    Q_PROPERTY(double ghostYawDegrees READ ghostYawDegrees NOTIFY updated)
    Q_PROPERTY(bool optimalDeltaAvailable READ optimalDeltaAvailable NOTIFY updated)
    Q_PROPERTY(double optimalDeltaSeconds READ optimalDeltaSeconds NOTIFY updated)
    Q_PROPERTY(bool optimalFromRest READ optimalFromRest NOTIFY updated)
    Q_PROPERTY(QVariantList sectorDeltas READ sectorDeltas NOTIFY updated)
    Q_PROPERTY(QVariantList ghostTireEnergy READ ghostTireEnergy NOTIFY updated)
    Q_PROPERTY(QString profileName READ profileName NOTIFY modeChanged)
    Q_PROPERTY(bool mpccAvailable READ mpccAvailable CONSTANT)
    Q_PROPERTY(QString controllerStatus READ controllerStatus NOTIFY updated)
    Q_PROPERTY(QString chosenAction READ chosenAction NOTIFY updated)
    Q_PROPERTY(QString decisionTitle READ decisionTitle NOTIFY updated)
    Q_PROPERTY(QString decisionCost READ decisionCost NOTIFY updated)
    Q_PROPERTY(QVariantList actionTimes READ actionTimes NOTIFY updated)
    Q_PROPERTY(int vehicleModel READ vehicleModel NOTIFY modeChanged)
    Q_PROPERTY(bool tireSlipModeled READ tireSlipModeled NOTIFY modeChanged)
    Q_PROPERTY(double rearSideslipDegrees READ rearSideslipDegrees NOTIFY updated)
    Q_PROPERTY(int steeringMode READ steeringMode NOTIFY modeChanged)
    Q_PROPERTY(int speedPlanMode READ speedPlanMode NOTIFY modeChanged)
    Q_PROPERTY(double envelopeFraction READ envelopeFraction NOTIFY modeChanged)
    Q_PROPERTY(double frontSlipDegrees READ frontSlipDegrees NOTIFY updated)
    Q_PROPERTY(double rearSlipDegrees READ rearSlipDegrees NOTIFY updated)
    Q_PROPERTY(double frontPeakSlipDegrees READ frontPeakSlipDegrees NOTIFY updated)
    Q_PROPERTY(double rearPeakSlipDegrees READ rearPeakSlipDegrees NOTIFY updated)
    Q_PROPERTY(double understeerDegrees READ understeerDegrees NOTIFY updated)
    Q_PROPERTY(QString handlingBalance READ handlingBalance NOTIFY updated)
    Q_PROPERTY(double balanceDeadbandDegrees READ balanceDeadbandDegrees CONSTANT)
    Q_PROPERTY(double corneringThreshold READ corneringThreshold CONSTANT)
    Q_PROPERTY(bool mapAvailable READ mapAvailable NOTIFY modeChanged)
    Q_PROPERTY(bool wheelsModeled READ wheelsModeled NOTIFY modeChanged)
    Q_PROPERTY(QVariantList wheelSlipRatios READ wheelSlipRatios NOTIFY updated)
    Q_PROPERTY(QVariantList wheelStates READ wheelStates NOTIFY updated)
    // Each tire's use of its own friction circle along the driving plan's horizon (decision 0027): one row per plan
    // point, time first and then the four wheels. Empty unless a predictive controller's plan, with its commands, is
    // driving a car with four wheels.
    Q_PROPERTY(QVariantList planTireUse READ planTireUse NOTIFY updated)
    Q_PROPERTY(bool planTireUseAvailable READ planTireUseAvailable NOTIFY updated)
    Q_PROPERTY(double planTireWorstUse READ planTireWorstUse NOTIFY updated)
    Q_PROPERTY(double planTireWorstTime READ planTireWorstTime NOTIFY updated)
    Q_PROPERTY(QVariantList wheelLoads READ wheelLoads NOTIFY updated)
    Q_PROPERTY(QVariantList staticWheelLoads READ staticWheelLoads NOTIFY modeChanged)
    Q_PROPERTY(QVariantList wheelFrictionUse READ wheelFrictionUse NOTIFY updated)
    Q_PROPERTY(bool aerodynamicsModeled READ aerodynamicsModeled NOTIFY modeChanged)
    Q_PROPERTY(double dragNewtons READ dragNewtons NOTIFY updated)
    Q_PROPERTY(double downforceNewtons READ downforceNewtons NOTIFY updated)
    Q_PROPERTY(QVariantList setupControls READ setupControls NOTIFY modeChanged)
    Q_PROPERTY(bool envelopeModeled READ envelopeModeled NOTIFY modeChanged)
    Q_PROPERTY(bool envelopeReady READ envelopeReady NOTIFY updated)
    Q_PROPERTY(bool envelopeFailed READ envelopeFailed NOTIFY updated)
    Q_PROPERTY(QString envelopeFingerprint READ envelopeFingerprint NOTIFY updated)
    Q_PROPERTY(QVariantList envelopeBoundary READ envelopeBoundary NOTIFY updated)
    Q_PROPERTY(double ggLateral READ ggLateral NOTIFY updated)
    Q_PROPERTY(double ggLongitudinal READ ggLongitudinal NOTIFY updated)
    Q_PROPERTY(double steeringCorrectionDegrees READ steeringCorrectionDegrees NOTIFY updated)
    Q_PROPERTY(int lineMode READ lineMode NOTIFY modeChanged)
    // The Formula Student layout driven instead of the preset (decision 0032), empty for the preset.
    Q_PROPERTY(QString courseName READ courseName NOTIFY modeChanged)
    Q_PROPERTY(bool racingLineAvailable READ racingLineAvailable CONSTANT)
    Q_PROPERTY(bool racingLineReady READ racingLineReady NOTIFY modeChanged)
    Q_PROPERTY(QString lineSummary READ lineSummary NOTIFY updated)
    Q_PROPERTY(QVariantList comparisonLinePoints READ comparisonLinePoints NOTIFY modeChanged)
    Q_PROPERTY(bool latticeAvailable READ latticeAvailable NOTIFY modeChanged)
    // A person driving (decision 0037): whether they drive, whether a controller is connected, and their controls in force.
    Q_PROPERTY(bool humanDriving READ humanDriving NOTIFY modeChanged)
    Q_PROPERTY(bool gamepadConnected READ gamepadConnected NOTIFY driverChanged)
    Q_PROPERTY(double driverThrottle READ driverThrottle NOTIFY driverChanged)
    Q_PROPERTY(double driverBrake READ driverBrake NOTIFY driverChanged)
    Q_PROPERTY(double driverSteering READ driverSteering NOTIFY driverChanged)
    // Gokart mode (decision 0038): the Gokartcentralen Göteborg track with its rental kart, and the car's wheelbase, which
    // the scene draws the kart-sized car at.
    Q_PROPERTY(bool kartMode READ kartMode NOTIFY modeChanged)
    Q_PROPERTY(bool forgivenessEnabled READ forgivenessEnabled NOTIFY updated)
    Q_PROPERTY(double forgivenessLevel READ forgivenessLevel NOTIFY updated)
    Q_PROPERTY(bool forgivenessManual READ forgivenessManual NOTIFY updated)
    Q_PROPERTY(QVariantMap forgivenessTuning READ forgivenessTuning NOTIFY updated)
    Q_PROPERTY(QVariantList forgivenessPresets READ forgivenessPresets NOTIFY presetsChanged)
    Q_PROPERTY(QString presetMessage READ presetMessage NOTIFY presetsChanged)
    Q_PROPERTY(bool forgivenessActive READ forgivenessActive NOTIFY updated)
    Q_PROPERTY(double wheelbase READ wheelbase NOTIFY modeChanged)
public:
    explicit Bridge(QObject* parent = nullptr, QString presetsPath = {});
    bool running() const { return running_; }
    double speedKmh() const { return state().speed_mps*3.6; }
    double targetSpeedKmh() const;
    double grip() const;
    double simulationTime() const { return state().time_s; }
    double x() const { return state().x_m; }
    double y() const { return state().y_m; }
    double yawDegrees() const { return state().yaw_rad*180/std::numbers::pi; }
    double steeringDegrees() const { return state().steering_rad*180/std::numbers::pi; }
    double progress() const;
    double crossTrackError() const;
    // The plan's straight-line braking bound: the longitudinal grip fraction of mu g, or under the envelope plan its share
    // of the car's braking capacity at the present speed (decision 0018).
    double brakeBound() const;
    double cornerSpeedKmh() const { return plan().at(limitingIndex()).speed_mps*3.6; }
    double cornerDistance() const;
    double plannedAcceleration() const;
    double utilization() const;
    double planColorDeadband() const { return fd::plan_color_deadband_mps2; }
    double trackLength() const { return track().length_m; }
    double constraintStation() const { return track().points.at(limitingIndex()).s_m; }
    int revision() const;
    int lap() const;
    double practiceLapSeconds() const { return practice_laps_ ? practice_laps_->current_seconds(simulationTime()) : 0; }
    double previousPracticeLap() const { return practice_laps_ ? practice_laps_->previous_seconds() : 0; }
    double bestPracticeLap() const { return practice_laps_ ? practice_laps_->best_seconds() : 0; }
    int practiceLaps() const { return practice_laps_ ? practice_laps_->completed() : 0; }
    QString status() const;
    QString reason() const;
    QVariantList pathPoints() const { return path_points_; }
    QVariantList predictionPoints() const { return prediction_points_; }
    bool predictionAvailable() const { return prediction_.size() > 1; }
    double predictionSeconds() const { return predictionAvailable() ? prediction_.back().state.time_s-prediction_.front().state.time_s : 0.0; }
    double predictionDistance() const { return predictionAvailable() ? prediction_.back().distance_m : 0.0; }
    bool predictionWithinEnvelope() const;
    QString predictionStatus() const;
    const std::vector<fd::TrajectorySample>& prediction() const { return prediction_; }
    // In replay these come from the decision recorded for the cursor sample. Schema 1 recorded
    // neither the alternatives nor the choice, so replay then reports that there is no recorded
    // decision rather than recomputing one now.
    bool decisionAvailable() const { return !playback_ || recordedDecision(); }
    QString decisionReason() const;
    bool holdingForBlockage() const;
    QString blockingIdentifier() const;
    int evaluatedOptions() const;
    int rejectedOptions() const;
    double selectedOffset() const;
    // Which planner chooses the car's actions, live or as recorded: 0 the lattice planner (decision 0022), 1 the
    // five-offset planner it replaces. Runs recorded before schema 10 were planned by five offsets.
    int localPlanner() const {
        const auto mode = playback_ ? playback_->recording().metadata.local_planner_mode : simulation_.local_planner_mode();
        return mode == fd::LocalPlannerMode::five_offsets ? 1 : 0;
    }
    // Which controller drives the chosen actions, live or as recorded: 0 the policy, 1 MPCC. Runs recorded before schema
    // 12 were driven by the policy.
    int controllerMode() const {
        const auto mode = playback_ ? playback_->recording().metadata.predictive_controller.mode : simulation_.controller_mode();
        return mode == fd::ControllerMode::mpcc ? 1 : 0;
    }
    // Whether this build has MPCC, which needs OSQP's sources like the racing line.
    bool mpccAvailable() const;
    // Whether the car carries instruments, live or as recorded; runs recorded before schema 14 carried none.
    bool sensorsFitted() const;
    // Fits the car with the standard instruments, or takes them off. Live only, and only while paused, like the rest of
    // the header: their cadence and their errors start where they are fitted.
    Q_INVOKABLE void setSensors(bool fitted);
    // Whether the car perceives the course's cones, live or as recorded; runs recorded before schema 15 did not.
    bool perceptionFitted() const;
    // Gives the car simulated cone perception of this track's cones, or takes it off. Live only, while paused.
    Q_INVOKABLE void setPerception(bool fitted);
    // The course's cones, live or as recorded; empty without perception.
    const std::vector<fd::Cone>& perceivedCones() const;
    // What the newest frame of each sensor made of the cones, placed on the map from where the car truly was when the
    // frame was sampled: the evaluation's view, never the car's. A detection says whether its colour is wrong, which
    // an unknown colour is not; its covariance is turned onto the map with it.
    struct DrawnDetection {
        fd::Vec2 position;
        fd::ObservedColour colour{fd::ObservedColour::unknown};
        bool wrong_colour{};
        double covariance_xx{}, covariance_xy{}, covariance_yy{};
    };
    struct PerceptionView {
        std::vector<DrawnDetection> detections;
        std::vector<std::size_t> missed;  // cones in view that the newest frames missed
        double sampled_at_s{-1};           // the newest frame's sample time; negative before the first
    };
    PerceptionView perceptionView() const;
    bool perceptionFrameAvailable() const { return perceptionView().sampled_at_s >= 0; }
    int perceivedDetections() const { return static_cast<int>(perceptionView().detections.size()); }
    int perceivedMissed() const { return static_cast<int>(perceptionView().missed.size()); }
    int perceivedWrongColour() const;
    double perceptionFrameAgeMs() const;
    // Whether the car drives on cones, live or as recorded; runs recorded before schema 16 drove on the known track.
    bool drivingOnCones() const;
    // Drives on the cones the car perceives, or on the known track again; either starts the run over, so the car's map
    // begins where it is placed. Live only, while paused, and only with perception.
    Q_INVOKABLE void setDriveOnCones(bool on);
    // The width the preset is laid to while the car drives on its cones, a Formula Student course's.
    static constexpr double cone_course_width_m = 5.0;
    // "MEASURED" when the driver reads the car's instruments, "IDEAL" when it reads the true state; empty on the track.
    QString beliefSource() const;
    // What the driver believed at the decision in force, live or as recorded: its path on the map, the corridor it
    // believed, and where it believed the car was. Empty before its first path.
    struct BelievedView {
        std::vector<fd::Vec2> path;
        double width_m{};
        double sampled_at_s{-1};  // when the frame it came from was sampled
        fd::State believed;       // the believed state at the decision
        bool has_belief{};
    };
    BelievedView believedView() const;
    bool believedPathAvailable() const { return !believedView().path.empty(); }
    double believedPathAheadM() const;
    double believedPathAgeMs() const;
    double beliefErrorM() const;
    // What the judge has seen by the cursor, live or from the recorded timing events; nothing is judged again in replay.
    struct TimingView {
        int laps{}, required{};
        double last_lap_s{}, penalty_s{};
        int cones_hit{}, off_course{};
        bool off_now{}, finished{}, stopped{};
        std::string dnf;
        std::vector<std::size_t> hit;  // the cones knocked, as indices into perceivedCones()' course
    };
    TimingView timingView() const;
    // The course's cones, live or as recorded, whether or not the car perceives them.
    const std::vector<fd::Cone>& courseCones() const;
    double lastLapSeconds() const { return timingView().last_lap_s; }
    double penaltySeconds() const { return timingView().penalty_s; }
    int conesHit() const { return timingView().cones_hit; }
    int offCourseCount() const { return timingView().off_course; }
    // "DNF: reason", "OFF COURSE", "FINISHED" or empty.
    QString judgeVerdict() const;
    // Loads and validates a reference-optimal artefact (fd::load_optimal_lap); a refused one names why in the status.
    Q_INVOKABLE bool loadOptimalLap(const QString& directory);
    // Builds the live four-wheel car from a documented physics profile (decision 0033) with its wheelbase, lock and speed
    // cap, as fd_headless --profile does; the car's appearance is untouched. Live only, while paused. The car keeps the
    // profile's name only while it is still exactly the profile's car.
    bool loadProfile(const QString& id);
    QString profileName() const;
    bool optimalLoaded() const { return optimal_.has_value(); }
    bool optimalMatches() const { return optimal_matches_; }
    bool optimalConverged() const { return optimal_ && optimal_->converged; }
    QString optimalStatus() const { return optimal_ ? QString::fromStdString(optimal_->status) : QString(); }
    // Which solver, model, mesh and tolerance the optimum is for, as the honesty rules ask it to be stated.
    QString optimalSource() const;
    QString optimalMismatch() const { return optimal_mismatch_; }
    double optimalLapSeconds() const { return optimal_ ? optimal_->lap_time_s : 0.0; }
    // Each setup sensitivity as the card quotes it: the step, what it would be worth, and how it was found.
    QVariantList optimalSensitivities() const;
    double ghostWheelbase() const { return optimal_ ? optimal_->problem.config.wheelbase_m : 0.0; }
    // Where the optimum is at the time since the live car's lap began, held on the line before the start and after its
    // own lap is done. Rear axle, as the car's pose is.
    bool ghostVisible() const { return optimal_matches_; }
    double ghostX() const { return ghost().x_m; }
    double ghostY() const { return ghost().y_m; }
    double ghostYawDegrees() const { return ghost().yaw_rad*180/std::numbers::pi; }
    // The live lap against the optimum: the time since the lap began less the optimum's time at the live car's station,
    // positive when the live car is behind. A first lap starts from rest; the optimum is a flying lap.
    bool optimalDeltaAvailable() const;
    double optimalDeltaSeconds() const;
    bool optimalFromRest() const;
    // Each sector of the lap in progress: the live time where the judge has timed it, the optimum's, and the difference.
    QVariantList sectorDeltas() const;
    // Each tire's energy dissipated since the start line at the optimum's present point, and over its whole lap, in kJ.
    QVariantList ghostTireEnergy() const;
    // What those instruments last delivered against what the car truly is at the same tick, live or as recorded. All
    // zero until a pose has arrived, which measuredAvailable says.
    bool measuredAvailable() const;
    double measuredAgeMs() const;
    double measuredPositionError() const;
    double measuredSpeedKmh() const;
    double measuredSpeedErrorKmh() const;
    double measuredYawRateErrorDegrees() const;
    // Under MPCC, who drives the decision in force and how the solver fared: live with its solve time on this host, in
    // replay as recorded, without one. Empty under the policy.
    QString controllerStatus() const;
    // The chosen option's action, live or recorded: "straight", "pass left", "pass right", "brake", or "offset" under the
    // five-offset planner. Empty without a decision.
    QString chosenAction() const;
    // What the decision panel calls the choice: the lattice planner's action in capitals, RETURNING for a straight
    // action along a lattice path, and the five-offset planner's AVOIDING or BLOCKED. Empty without a decision.
    QString decisionTitle() const;
    // The chosen lattice path's cost and its terms in the planner's order, or empty when the choice follows no path.
    QString decisionCost() const;
    // Every option the decision in force compared by estimated time (decision 0023), live or recorded, in its order: {action,
    // label (Left, Right, Return along the lattice, Rejoin directly), seconds, clear, chosen, x, y}, with x and y where its
    // predicted line has run label_distance_m, within the follow camera's view. Empty without such a decision.
    static constexpr double label_distance_m = 15;
    QVariantList actionTimes() const;
    // Each action's colour, as the alternatives are drawn and the legend shows them; a rejected line is drawn darker.
    static QColor actionColor(fd::LocalAction action, bool clear = true);
    Q_INVOKABLE QString actionColorName(const QString& action) const;
    // Other options the decision in force evaluated, live or recorded, in world coordinates: each predicted line with
    // its action and whether it was clear.
    struct Alternative { std::vector<fd::Vec2> points; bool clear{}; fd::LocalAction action{fd::LocalAction::offset}; };
    std::vector<Alternative> alternatives() const;
    int obstructionCount() const { return static_cast<int>(obstructions().size()); }
    // Live scenario, or the blocked regions the replayed run was recorded against.
    const std::vector<fd::Obstruction>& obstructions() const {
        return playback_ ? playback_->recording().metadata.scenario_obstructions : simulation_.obstructions();
    }
    const fd::RecordedDecision* recordedDecision() const { return playback_ ? playback_->decision() : nullptr; }
    // 0 is the kinematic bicycle, 1 the dynamic single-track model, 2 that car on soft front tires, 3 the
    // four-wheel car: live, or as recorded in replay. A recorded dynamic car with other parameters shows as 1.
    const fd::VehicleModel& vehicleModelData() const {
        return playback_ ? playback_->recording().metadata.vehicle_model : simulation_.vehicle_model();
    }
    int vehicleModel() const {
        if (std::holds_alternative<fd::FourWheelCar>(vehicleModelData())) return 3;
        const auto* car = std::get_if<fd::DynamicSingleTrack>(&vehicleModelData());
        if (!car) return 0;
        const auto soft = fd::DynamicSingleTrack::soft_front();
        return car->front_tire.peak_slip_angle_low_rad == soft.front_tire.peak_slip_angle_low_rad &&
               car->front_tire.peak_slip_angle_high_rad == soft.front_tire.peak_slip_angle_high_rad ? 2 : 1;
    }
    bool tireSlipModeled() const { return vehicleModel() != 0; }
    // 0 is Pure Pursuit, 1 MAP: live, or as recorded in replay.
    int steeringMode() const {
        const auto mode = playback_ ? playback_->recording().metadata.steering_mode : simulation_.steering_mode();
        return mode == fd::SteeringMode::model_acceleration_pursuit ? 1 : 0;
    }
    // What the plan in force was made from, live or as recorded: 0 the grip fractions, 1 the car's performance envelope.
    int speedPlanMode() const {
        const auto mode = playback_ ? playback_->recording().metadata.speed_plan_mode : simulation_.speed_plan_mode();
        return mode == fd::SpeedPlanMode::performance_envelope ? 1 : 0;
    }
    double envelopeFraction() const { return config().envelope_fraction; }
    // MAP's steering minus Pure Pursuit's for the same target, both within the steering limit.
    double steeringCorrectionDegrees() const;
    // What the plant asks of its tires: live from the simulation, in replay derived from the recorded
    // plant state through the vehicle seam. Slip angles are zero for the kinematic bicycle.
    fd::Demand tireDemand() const;
    double frontSlipDegrees() const { return tireDemand().front_slip_angle_rad*180/std::numbers::pi; }
    double rearSlipDegrees() const { return tireDemand().rear_slip_angle_rad*180/std::numbers::pi; }
    double frontPeakSlipDegrees() const { return tireDemand().front_peak_slip_angle_rad*180/std::numbers::pi; }
    double rearPeakSlipDegrees() const { return tireDemand().rear_peak_slip_angle_rad*180/std::numbers::pi; }
    double understeerDegrees() const { return fd::handling_balance(tireDemand()).understeer_angle_rad*180/std::numbers::pi; }
    // The balance label in capitals: UNDERSTEER, OVERSTEER, NEUTRAL, NOT CORNERING or NOT MODELED.
    QString handlingBalance() const { return QString::fromUtf8(fd::balance_name(fd::handling_balance(tireDemand()).balance)).toUpper(); }
    double balanceDeadbandDegrees() const { return fd::balance_deadband_rad*180/std::numbers::pi; }
    double corneringThreshold() const { return fd::cornering_lateral_acceleration_mps2; }
    // MAP needs a steering table, which is generated for the single-track cars only.
    // Every car with tires has a steady-state steering table; the kinematic bicycle's would be its geometry.
    bool mapAvailable() const { return vehicleModel() != 0; }
    bool wheelsModeled() const { return vehicleModel() == 3; }
    QVariantList wheelSlipRatios() const;
    // Each wheel, front left to rear right: LIFT when it carries no load, LOCK when stopped while the car moves,
    // SPIN when driven past its tire's peak, SLIDE when past the peak otherwise, and GRIP. Empty for a car
    // without rotating wheels.
    QVariantList wheelStates() const;
    QVariantList planTireUse() const;
    bool planTireUseAvailable() const { return plan_tire_use_.size() > 1; }
    // The largest use any tire is asked for along the horizon, and how far ahead it falls.
    double planTireWorstUse() const;
    double planTireWorstTime() const;
    // Each wheel's vertical load in newtons, live or as recorded, and its load at rest (decision 0014).
    QVariantList wheelLoads() const;
    QVariantList staticWheelLoads() const;
    // Each tire's force over its peak force at its present load, as [longitudinal, lateral]: a point in its
    // friction circle. Zero at walking pace, where the car moves as the kinematic bicycle.
    QVariantList wheelFrictionUse() const;
    // The four-wheel car's drag and downforce (decision 0015), live or from the recorded plant state; zero for the
    // other models, which have no aerodynamics.
    bool aerodynamicsModeled() const;
    fd::AerodynamicForce aerodynamics() const;
    double dragNewtons() const { const auto f = aerodynamics(); return std::hypot(f.longitudinal_n, f.lateral_n); }
    double downforceNewtons() const { return aerodynamics().downforce_n; }
    double rearSideslipDegrees() const;
    int carAppearance() const { return appearance_; }
    bool overview() const { return overview_; }
    int cameraMode() const { return camera_mode_; }
    bool replay() const { return playback_.has_value(); }
    QString recordingName() const { return recording_name_; }
    double playheadTime() const { return playback_ ? playback_->time_s() : 0.0; }
    double recordingDuration() const { return playback_ ? playback_->duration_s() : 0.0; }
    QVariantList eventTimes() const;
    int appliedEvents() const { return playback_ ? static_cast<int>(playback_->applied_event_count()) : static_cast<int>(simulation_.events().size()); }
    int recordedEvents() const { return playback_ ? static_cast<int>(playback_->recording().events.size()) : 0; }
    // Mode-aware sources for the scene geometry. In replay these come from the recording.
    const fd::Track& track() const { return playback_ ? playback_->recording().track : simulation_.track(); }
    const std::vector<fd::PlanPoint>& plan() const { return playback_ ? playback_->plan().points : simulation_.plan(); }
    fd::State state() const { return playback_ ? playback_->sample().state() : simulation_.state(); }
    const fd::Config& config() const { return playback_ ? playback_->config() : simulation_.config(); }
    const fd::Simulation& simulation() const { return simulation_; }
    Q_INVOKABLE void toggleRunning();
    Q_INVOKABLE void reset();
    Q_INVOKABLE void setGrip(double value);
    Q_INVOKABLE void setAppearance(int value);
    Q_INVOKABLE void setOverview(bool value);
    Q_INVOKABLE void setCameraMode(int value);
    // Accepts a directory path or file:// URL. On failure the status shows the violated rule
    // and the previous mode stays active.
    Q_INVOKABLE bool loadRecording(const QString& location);
    // Drives a Formula Student layout in PacSim's track format instead of the preset (decision 0032): its cones, its traced
    // centreline and corridor, and its gates, with a Formula Student car's wheelbase, lock and aim
    // (configs/formula-student.cfg), which its hairpins need. The kinematic bicycle drives it; instruments and perception
    // stay as they were, and the preset's racing line, solved for the preset, is not offered. Live only, while paused.
    Q_INVOKABLE bool loadCourse(const QString& path);
    QString courseName() const { return course_name_; }
    // Back to the preset and the configuration the desktop started with, keeping the grip, instruments and perception.
    Q_INVOKABLE bool loadPreset();
    Q_INVOKABLE void exitReplay();
    Q_INVOKABLE void seek(double time_s);
    // Scenario controls. A stated blocked region placed ahead of the car, not a detection.
    Q_INVOKABLE bool placeBlockageAhead(bool fullWidth);
    Q_INVOKABLE void clearBlockages();
    // Starts a fresh live run on the chosen model, keeping the applied grip and the stated
    // scenario. Refused while running or in replay, with the reason in the status.
    Q_INVOKABLE bool setVehicleModel(int model);
    // Starts a fresh live run steered by Pure Pursuit (0) or MAP (1). MAP needs a dynamic model;
    // choosing the kinematic bicycle returns to Pure Pursuit. Refused while running or in replay.
    Q_INVOKABLE bool setSteeringMode(int mode);
    // Starts a fresh live run planned from the grip fractions (0) or the four-wheel car's envelope (1), keeping grip,
    // setup and scenario. Refused in replay, while running, and for a model without an envelope, with the reason.
    Q_INVOKABLE bool setSpeedPlanMode(int mode);
    // Starts a fresh live run whose actions the lattice planner (0) or the five-offset planner (1) chooses, keeping grip,
    // car, line and scenario. Refused in replay and while running, with the reason in the status.
    Q_INVOKABLE bool setLocalPlanner(int planner);
    // What drives the chosen action, in one choice (decision 0024): the policy steered by Pure Pursuit (0) or by MAP (1),
    // or MPCC (2), with Pure Pursuit as the policy that predicts the alternatives and drives what MPCC may not. Starts a
    // fresh live run keeping grip, car, plan, line, planner and scenario. Refused in replay, while running, for MAP as
    // setSteeringMode refuses it, and for MPCC in a build without OSQP or for the kinematic bicycle, which has no tires,
    // with the reason in the status. Choosing the kinematic bicycle returns to the policy.
    Q_INVOKABLE bool setControl(int control);
    // The four-wheel car's setup controls (decision 0016), each as {key, label, unit, scale, minimum, maximum, step,
    // value, effect}: the live car's values, or the recorded car's in replay. Empty for the other models.
    QVariantList setupControls() const;
    // Starts a fresh live run with one setup value changed, keeping grip and scenario. Refused in replay, while
    // running, for another model, and outside the control's offered range, with the reason in the status.
    Q_INVOKABLE bool setSetupValue(const QString& key, double value);
    // Starts a fresh live run on the default four-wheel car, under the same refusals.
    Q_INVOKABLE bool resetSetup();
    // The shown four-wheel car's G-G-V performance envelope (decision 0017): derived on a worker thread whenever the
    // car or its configuration changes, from the live car or, in replay, the recorded one. Ready only while it matches.
    bool envelopeModeled() const { return wheelsModeled(); }
    bool envelopeReady() const { return envelope_ready_; }
    // The derivation for the car shown failed; it is not retried until the car or its configuration changes.
    bool envelopeFailed() const { return envelope_failed_now_; }
    QString envelopeFingerprint() const { return envelope_ready_ ? QString::fromStdString(envelope_->fingerprint()) : QString(); }
    const fd::PerformanceEnvelope* envelope() const { return envelope_ready_ ? envelope_.get() : nullptr; }
    // The envelope's boundary at the current speed as {lateral, longitudinal} points around the diagram: out along the
    // left side's forward limits, back along its braking limits, then the right side's, with right turns negative.
    QVariantList envelopeBoundary() const;
    // The car's own point on the diagram: lateral acceleration, left positive, and the longitudinal acceleration it
    // achieved over the last tick, or between the recorded samples in replay.
    double ggLateral() const;
    double ggLongitudinal() const;
    // Waits for any derivation in progress and derives the envelope on this thread if it is still not ready.
    void deriveEnvelopeNow();
    // The line the live car drives (decision 0020): 0 the preset's centreline, 1 the minimum-curvature racing line solved
    // inside its corridor. The racing line is solved once, on a worker thread, from the preset conditioned every metre;
    // a build without OSQP has none. Replay shows the recorded track, with its corridor's edges when it recorded them.
    int lineMode() const { return line_mode_; }
    bool racingLineAvailable() const;
    bool racingLineReady() const { return course_name_.isEmpty() && line_solution_ && line_solution_->failure.empty(); }
    // Starts a fresh live run on the chosen line, keeping grip, car, steering, speed plan and setup. Refused in replay,
    // while running, before the racing line is solved, and with stated blockages, which are placed across the
    // centreline, with the reason in the status.
    Q_INVOKABLE bool setLineMode(int mode);
    // Estimated lap times of the plan in force, made for each line (fd::estimated_lap_time): the racing line, the preset's
    // centreline the car drives otherwise, and the smoothed centreline the racing line was solved against, which
    // separates what smoothing the preset's curvature steps gains from what the line gains. Zero until solved.
    double racingLineLapSeconds() const { return line_lap_s_; }
    double centrelineLapSeconds() const { return centreline_lap_s_; }
    double smoothedCentrelineLapSeconds() const { return smoothed_lap_s_; }
    QString lineSummary() const;
    // The line the live car is not driving, drawn with the one it is: the racing line beside a centreline run, the
    // centreline beside a racing line run. Empty in replay and before the racing line is solved.
    const std::vector<fd::Vec2>& comparisonLine() const { static const std::vector<fd::Vec2> none; return playback_ || !racingLineReady() ? none : comparison_line_; }
    QVariantList comparisonLinePoints() const;
    // Waits for the racing line solve and takes its result.
    void solveRacingLineNow();
    // The offline lattice (decision 0021) laid along the track shown, live or recorded, with the default options and the
    // configuration's steering limit; laid again whenever that track changes. Absent if the track refuses one.
    const fd::Lattice* lattice() const { return lattice_ ? &*lattice_ : nullptr; }
    bool latticeAvailable() const { return lattice_.has_value(); }
    // How far ahead of the car's station the scene draws the lattice: the prediction's reach at the speed cap.
    static constexpr double lattice_horizon_m = 60.0;
    // The layers within that horizon ahead of the car's station, nearest first; empty without a lattice.
    std::vector<std::size_t> latticeLayersAhead() const;
    // Plain-text wheel state from a demand and the plant state, as wheelStates reports it.
    static QString wheelState(const fd::Demand& demand, const fd::PlantState& state, std::size_t wheel, double slip_speed_floor_mps);
    bool addObstruction(const fd::Obstruction& obstruction);
    void advanceForCapture(double seconds);
    // A person drives the live car instead of the autonomous driver, or with false hands it back (decision 0037), at any
    // time: the simulation takes the change at the tick boundary it is made at. The planner's action is still shown, not
    // driven. Refused in replay, which shows a recorded run.
    Q_INVOKABLE bool setHumanDriving(bool on);
    bool humanDriving() const { return !playback_ && simulation_.human_driving(); }
    // The person's controls: the controller gives them every frame while one is connected and a person drives. Refused
    // outside their travel, with the reason in the status.
    Q_INVOKABLE bool setDriverControls(double throttle, double brake, double steering);
    bool gamepadConnected() const { return pad_.connected; }
    double driverThrottle() const { return simulation_.driver_controls().throttle; }
    double driverBrake() const { return simulation_.driver_controls().brake; }
    double driverSteering() const { return simulation_.driver_controls().steering; }
    // Reads the controller every frame unless off; the verification turns it off, so the controls it gives stand.
    void setGamepadEnabled(bool on) { gamepad_enabled_ = on; if (!on) pad_ = {}; }
    // Gokart mode (decision 0038): a fresh run on the Gokartcentralen Göteborg track, traced from its published poster (400 m,
    // 6 m wide, clockwise), with the Sodi RSX2 rental kart's profile, judged on its laps and excursions without cones, as its
    // barriers make it. Whoever drives keeps the car; the car, control, plan and line stay the kart's until the mode is
    // left, which returns the track, car and configuration it came from. Live only, while paused.
    Q_INVOKABLE bool setKartMode(bool on);
    bool kartMode() const { return !playback_ && kart_mode_; }
    Q_INVOKABLE bool setForgiveness(bool enabled, double level);
    Q_INVOKABLE bool setForgivenessManual(bool manual, double acceleration, double braking, double steering, double grip, double speed);
    bool forgivenessManual() const { return simulation_.requested_driving_assistance().manual; }
    QVariantMap forgivenessTuning() const;
    QVariantList forgivenessPresets() const;
    QString presetMessage() const { return preset_message_; }
    Q_INVOKABLE bool saveForgivenessPreset(int slot);
    Q_INVOKABLE bool loadForgivenessPreset(int slot);
    Q_INVOKABLE bool deleteForgivenessPreset(int slot);
    bool forgivenessEnabled() const { return kartMode() && simulation_.requested_driving_assistance().enabled; }
    double forgivenessLevel() const { return simulation_.requested_driving_assistance().level; }
    bool forgivenessActive() const { return kartMode() && simulation_.assistance_active(); }
    double wheelbase() const { return config().wheelbase_m; }
signals:
    void presetsChanged();
    void updated();
    void planChanged();
    void modeChanged();
    void driverChanged();
    // The controller's Menu button was pressed, or its accelerator pressed at the line while a person drives: the screen
    // showing the drive starts or pauses the run, as a game's does, and the car selection does not.
    void menuPressed();
    void acceleratorAtLine();
private:
    void publishUpdate(bool plan_changed = false, bool mode_changed = false);
    // Without a planner, the live run's own.
    // Without a controller, the live run's own.
    bool rebuild(const fd::Track& track, const fd::VehicleModel& model, fd::SteeringMode steering, fd::SpeedPlanMode speed_plan,
                 std::optional<fd::LocalPlannerMode> planner = std::nullopt, std::optional<fd::ControllerMode> controller = std::nullopt,
                 std::optional<fd::Config> configuration = std::nullopt);
    // Whether the loaded optimum is the one of the car and corridor shown, and if not why; on every mode change.
    void refreshOptimalMatch();
    // What the judge has seen by now, live or recorded, and when the lap in progress began (negative before the start).
    std::vector<fd::TimingEvent> timingEvents() const;
    double lapStartTime() const;
    fd::OptimalPoint ghost() const;
    // The physics profile of the car shown, live or recorded, while the car is still exactly that profile's; else empty.
    std::string activeProfile() const;
    bool takeRacingLine();
    void refreshLineEstimates();
    bool ensureLattice();
    void advanceTicks(int count);
    // Reads the controller: its controls to the car while a person drives, and its Menu button to Start and Pause.
    void readGamepad();
    void ensureEnvelope();
    void refreshPathPoints();
    void refreshPrediction();
    // Recomputes the horizon's tire use from the plan in force, at most a few times a second: driving the four-wheel
    // plant along a three-second plan costs milliseconds, and the answer only changes as the plan does.
    void refreshPlanTireUse(bool decision_changed);
    // What the instruments last delivered, live or at the replay cursor; null without instruments.
    const fd::Measurements* measured() const;
    std::size_t nearestIndex() const { return playback_ ? playback_->sample().nearest_index : simulation_.diagnostics().nearest_index; }
    std::size_t limitingIndex() const { return playback_ ? playback_->sample().limiting_index : simulation_.diagnostics().limiting_index; }
    fd::Simulation simulation_;
    AssistancePresets presets_;
    QString preset_message_;
    std::optional<fd::PracticeLaps> practice_laps_;
    std::optional<fd::Playback> playback_;
    QString recording_name_;
    QTimer timer_;
    QElapsedTimer elapsed_;
    double accumulator_{};
    bool running_{false}, overview_{false};
    int camera_mode_{0}; // Follow, overview, first person.
    int appearance_{};
    Gamepad gamepad_;
    PadState pad_;
    bool gamepad_enabled_{true}, menu_held_{};
    // Refuses a change of car or course in gokart mode, saying why; true when refused.
    bool kartRefuses(const char* what);
    bool kart_mode_{};
    // What gokart mode came from, to return to.
    struct BeforeKart {
        fd::Track centreline;
        QString course_name;
        std::vector<fd::Cone> cones;
        std::vector<fd::Gate> gates;
        std::string course_source;
        fd::VehicleModel model;
        fd::SteeringMode steering{};
        fd::SpeedPlanMode speed_plan{};
        fd::Config config;
        std::string profile;
        fd::Config preset_config;
    };
    std::optional<BeforeKart> before_kart_;
    QString error_;
    std::optional<double> pending_grip_;
    QVariantList path_points_;
    QVariantList prediction_points_;
    std::vector<fd::TrajectorySample> prediction_;
    std::vector<fd::PlanTireUse> plan_tire_use_;
    double plan_tire_use_time_{-1};             // the simulation or replay time its plan was made at
    std::uint64_t plan_tire_use_decisions_{};   // the decision count it was computed for
    fd::PlantState plan_tire_use_state_;        // the plant state the live plan was made in
    std::shared_ptr<const fd::PerformanceEnvelope> envelope_;
    bool envelope_ready_{}, envelope_busy_{}, envelope_failed_now_{};
    std::string envelope_failed_;  // the fingerprint of a derivation that failed, not retried
    std::mutex envelope_mutex_;
    std::shared_ptr<const fd::PerformanceEnvelope> envelope_finished_;  // handed over by the worker
    struct LineSolution {
        fd::Track smoothed_centreline, line;
        int iterations{};
        bool converged{};
        std::string failure;  // why the racing line could not be solved, empty when it was
    };
    fd::Track centreline_;  // the preset, or the loaded course, the live car drives on the centreline
    QString course_name_;   // the loaded course; empty for the preset
    fd::Track preset_;      // the preset, and the configuration the desktop started with
    fd::Config preset_config_;
    int line_mode_{};
    std::shared_ptr<const LineSolution> line_solution_;
    std::mutex line_mutex_;
    std::shared_ptr<const LineSolution> line_finished_;  // handed over by the worker
    std::vector<fd::Vec2> comparison_line_;
    double line_lap_s_{}, centreline_lap_s_{}, smoothed_lap_s_{};
    std::optional<fd::Lattice> lattice_;
    std::string lattice_track_;  // what identifies the track the lattice was laid along
    std::optional<fd::OptimalLap> optimal_;
    bool optimal_matches_{};
    QString optimal_mismatch_;
    std::vector<double> optimal_gate_times_;     // when the optimum crosses each of the judge's gates
    std::string optimal_corridor_key_;           // the track the corridor below was conditioned from
    fd::Track optimal_corridor_;
    std::string loaded_profile_;                 // the physics profile the desktop was started with, if any
    // Declared last so they are joined before anything they report to is destroyed.
    std::jthread envelope_worker_;
    std::jthread line_worker_;
};
