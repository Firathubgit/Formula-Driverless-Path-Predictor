pragma ComponentBehavior: Bound
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs
import QtQuick.Shapes
import QtQuick3D
import QtQuick.Effects
import Formula 1.0

ApplicationWindow {
    id: window
    objectName: "driverlessWindow"
    width: 1440
    height: 900
    minimumWidth: 1100
    minimumHeight: 700
    visible: true
    title: "Formula / Driverless"
    color: window.backdrop
    font.family: "Segoe UI"

    // A driver's display: the live scene fills the window and a few quiet readouts float over it; everything else opens
    // from the dock. Colour carries meaning only: green, yellow and red are planned acceleration, holding speed and braking,
    // from the core's deadband; blue marks what is selected. The rest is white type in three strengths on dark glass.
    readonly property color backdrop: "#08090b"
    readonly property color ink: "#f2f4f5"
    readonly property color muted: "#99a1a7"
    readonly property color faint: "#5f676d"
    readonly property color glass: "#e40f1114"
    readonly property color raised: "#ff131518"
    readonly property color hairline: "#1cffffff"
    readonly property color accent: "#4c8dff"
    readonly property color lineColor: "#22272c"
    readonly property color accelerateColor: "#89eab5"
    readonly property color holdColor: "#e6cf78"
    readonly property color brakeColor: "#f47770"
    readonly property bool compact: height < 800
    readonly property real margin: compact ? 18 : 28
    readonly property real sideWidth: compact ? 262 : 304

    // Panels opened from the dock. With all of them closed the window shows the drive alone.
    property bool settingsOpen: false
    property bool insightsOpen: false
    property bool layersOpen: false
    property bool whyExpanded: false
    // Scene layers the viewer may hide from the layers panel. Hiding one hides its drawing, never its data.
    property bool showLattice: true
    property bool showKartLattice: false
    readonly property bool latticeVisible: sim.kartMode ? showKartLattice : showLattice
    property bool showPrediction: true
    property bool showKartPrediction: false
    readonly property bool predictionVisible: sim.kartMode ? showKartPrediction : showPrediction
    function togglePrediction() {
        if (sim.kartMode) showKartPrediction = !showKartPrediction
        else showPrediction = !showPrediction
    }
    property bool showKartGuide: true
    property bool showMotionArrow: true
    property bool showKartMotionArrow: false
    readonly property bool motionArrowVisible: sim.kartMode ? showKartMotionArrow : showMotionArrow
    readonly property color coastColor: "#36adf1"
    property bool showReferenceLine: true
    property bool showAlternatives: true
    property bool showGhost: true
    property bool showReflections: true
    // The four-wheel car's setup panel (decision 0016): open or closed, editable only while paused on a live run.
    property bool setupOpen: false
    readonly property bool setupEditable: sim.wheelsModeled && !sim.running && !sim.replay && !sim.forgivenessEnabled
    property string setupFocus: ""
    function setupText(value, control) {
        var digits = Math.max(0, Math.ceil(-Math.log(control.step * control.scale) / Math.LN10 - 1e-9))
        return (value * control.scale).toFixed(digits) + " " + control.unit
    }
    property var bounds: ({minX: -50, maxX: 50, minY: -50, maxY: 50})
    property var cachedPoints: []
    property var cachedPrediction: []
    property var cachedComparison: []
    readonly property real trackLength: Math.max(1, sim.trackLength)
    property real maximumPlanSpeed: 30
    property real maximumPredictionSpeed: 30
    // Replay of a run without recorded predictions falls back to the recorded reference plan.
    readonly property bool showReference: sim.replay && !sim.predictionAvailable
    readonly property real plotMaximumSpeed: showReference ? maximumPlanSpeed : maximumPredictionSpeed
    readonly property real trackSpan: Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY)
    readonly property int motion: sim.plannedAcceleration < -sim.planColorDeadband ? -1 : sim.plannedAcceleration > sim.planColorDeadband ? 1 : 0
    readonly property string motionLabel: (showReference ? "REFERENCE · " : "") + (motion < 0 ? "BRAKING" : motion > 0 ? "ACCELERATING" : "HOLDING SPEED")
    readonly property color motionColor: trajectoryColor(sim.plannedAcceleration)
    readonly property string vehicleName: sim.kartMode ? "Sodi RSX2 kart" : ["Kinematic", "Dynamic", "Soft front", "4 wheels"][sim.vehicleModel] || "Kinematic"
    readonly property string controllerName: sim.controllerMode === 1 ? "MPCC" : sim.steeringMode === 1 ? "MAP steering" : "Pure Pursuit"
    // The lattice planner's chosen action in its own colour; the five-offset planner's avoiding or blocked.
    readonly property color decisionColor: sim.chosenAction === "offset" || sim.chosenAction === "" ? (sim.holdingForBlockage ? brakeColor : holdColor)
                                                                                                    : sim.actionColorName(sim.chosenAction)
    // A decision worth announcing: the car doing something about a stated blockage, not merely keeping to its line.
    readonly property bool blockageDecision: sim.obstructionCount > 0 && sim.decisionAvailable &&
                                             ["PASS LEFT", "PASS RIGHT", "BRAKE", "BLOCKED", "AVOIDING", "RETURNING"].indexOf(sim.decisionTitle) >= 0
    readonly property bool alerting: (sim.judgeVerdict !== "" && sim.judgeVerdict !== "FINISHED") ||
                                     (!sim.humanDriving && sim.predictionAvailable && !sim.predictionWithinEnvelope)
    readonly property bool ready: !sim.replay && !sim.running && sim.simulationTime === 0
    // One line of what the car is doing and why, as a driver's display says it: the judge's verdict or an invalid plan
    // first, then the decision around a stated blockage, then the planned motion against the limit that sets it. The
    // detail stays in the WHY THIS SPEED card.
    readonly property string toastText: {
        if (sim.judgeVerdict === "FINISHED") return "Finished"
        if (sim.judgeVerdict === "OFF COURSE") return "Off course"
        if (sim.judgeVerdict !== "") return sim.judgeVerdict
        // A person driving (decision 0037) is told what the controller does; the plan on the road is only a guide.
        if (sim.humanDriving) {
            if (!sim.gamepadConnected) return "Connect an Xbox controller to drive"
            if (ready) return "Ready  ·  right trigger to drive"
            return "You drive  ·  RT gas  ·  LT brake  ·  stick steers"
        }
        if (sim.predictionAvailable && !sim.predictionWithinEnvelope) return sim.predictionStatus
        if (blockageDecision) {
            var titles = {"PASS LEFT": "Passing left", "PASS RIGHT": "Passing right", "BRAKE": "Braking for a blockage",
                          "BLOCKED": "Holding for a blockage", "AVOIDING": "Avoiding a blockage", "RETURNING": "Returning to the line"}
            var what = sim.blockingIdentifier.replace(/-/g, " ")
            return titles[sim.decisionTitle] + (what !== "" ? "  ·  " + what : "")
        }
        if (ready) return "Ready  ·  Start or Space to drive"
        // A limit is named only when it is what the motion is heading for: a braking limit below the present speed, an
        // accelerating target above it. Otherwise, as on cones, where the car plans to its believed path's end, the
        // motion is stated alone.
        if (motion < 0) {
            if (sim.cornerSpeedKmh >= sim.speedKmh - 0.5) return "Braking"
            return sim.cornerDistance >= 5 ? "Braking for " + sim.cornerSpeedKmh.toFixed(0) + " km/h in " + sim.cornerDistance.toFixed(0) + " m"
                                           : "Braking to " + sim.cornerSpeedKmh.toFixed(0) + " km/h"
        }
        if (motion > 0) return sim.targetSpeedKmh > sim.speedKmh + 0.5 ? "Accelerating to " + sim.targetSpeedKmh.toFixed(0) + " km/h" : "Accelerating"
        return "Holding " + sim.speedKmh.toFixed(0) + " km/h"
    }
    readonly property color toastColor: alerting ? brakeColor : sim.humanDriving ? (sim.gamepadConnected ? accent : holdColor)
                                        : blockageDecision ? decisionColor : ready ? accent : motionColor

    function trajectoryColor(acceleration) {
        return acceleration < -sim.planColorDeadband ? "#f47770" : acceleration > sim.planColorDeadband ? "#89eab5" : "#e6cf78"
    }
    function timeText(seconds) {
        var s = Math.max(0, seconds)
        return Math.floor(s / 60).toString().padStart(2, "0") + ":" + (s % 60).toFixed(1).padStart(4, "0")
    }
    function lapClock(seconds) {
        var hundredths = Math.floor(Math.max(0, seconds) * 100 + 0.000001)
        return Math.floor(hundredths / 6000).toString().padStart(2, "0") + ":" +
               (Math.floor(hundredths / 100) % 60).toString().padStart(2, "0") + "." +
               (hundredths % 100).toString().padStart(2, "0")
    }
    function forgivenessMode(manual) {
        var tuning = sim.forgivenessTuning
        sim.setForgivenessManual(manual, tuning.acceleration, tuning.braking, tuning.steering, tuning.grip, tuning.speed)
    }
    function tuneForgiveness(key, value) {
        var tuning = sim.forgivenessTuning
        sim.setForgivenessManual(true, key === "acceleration" ? value : tuning.acceleration,
                                key === "braking" ? value : tuning.braking,
                                key === "steering" ? value : tuning.steering,
                                key === "grip" ? value : tuning.grip,
                                key === "speed" ? value : tuning.speed)
    }
    function refreshPlan() {
        // Copy the QObject sequence into ordinary JS values once per plan revision.
        // Reading a reference sequence's indices inside paint can repeatedly invoke
        // its C++ getter and turn a simple plot into quadratic work.
        var source = sim.pathPoints
        var points = []
        for (var index = 0, count = source.length; index < count; ++index) {
            var point = source[index]
            points.push({x: point.x, y: point.y, s: point.s,
                         speed: point.speed, acceleration: point.acceleration})
        }
        cachedPoints = points
        if (points.length < 2) return
        var b = {minX: points[0].x, maxX: points[0].x, minY: points[0].y, maxY: points[0].y}
        var peak = 1
        for (var i = 0; i < points.length; ++i) {
            b.minX = Math.min(b.minX, points[i].x)
            b.maxX = Math.max(b.maxX, points[i].x)
            b.minY = Math.min(b.minY, points[i].y)
            b.maxY = Math.max(b.maxY, points[i].y)
            peak = Math.max(peak, points[i].speed * 3.6)
        }
        bounds = b
        maximumPlanSpeed = Math.ceil(peak / 10) * 10
        circuitMap.requestPaint()
        speedPlot.requestPaint()
    }
    function refreshPrediction() {
        var source = sim.predictionPoints
        var points = []
        var peak = 1
        for (var index = 0, count = source.length; index < count; ++index) {
            var point = source[index]
            points.push({x: point.x, y: point.y, time: point.time, distance: point.distance,
                         speed: point.speed, acceleration: point.acceleration})
            peak = Math.max(peak, point.speed * 3.6)
        }
        cachedPrediction = points
        maximumPredictionSpeed = Math.max(10, Math.ceil(peak / 10) * 10)
        speedPlot.requestPaint()
        tireMarginPlot.requestPaint()
    }
    // The line the car is not driving, for the circuit map (decision 0020).
    function refreshComparison() {
        var source = sim.comparisonLinePoints
        var points = []
        for (var index = 0, count = source.length; index < count; ++index) points.push({x: source[index].x, y: source[index].y})
        cachedComparison = points
        circuitMap.requestPaint()
    }
    Component.onCompleted: { refreshPlan(); refreshPrediction(); refreshComparison() }
    Connections {
        target: sim
        function onPlanChanged() { window.refreshPlan() }
        function onModeChanged() { window.refreshComparison() }
        function onUpdated() { window.refreshPrediction(); circuitMap.requestPaint() }
    }
    Shortcut { sequence: "Space"; enabled: showroom.screen !== "showroom"; onActivated: { showroom.startDriving(); sim.toggleRunning() } }
    Shortcut { sequence: "R"; enabled: showroom.screen !== "showroom"; onActivated: sim.reset() }
    Shortcut { sequence: "V"; enabled: showroom.screen !== "showroom"; onActivated: sim.setCameraMode((sim.cameraMode + 1) % 3) }
    Shortcut { sequence: "Left"; enabled: sim.replay; onActivated: sim.seek(sim.playheadTime - 1) }
    Shortcut { sequence: "Right"; enabled: sim.replay; onActivated: sim.seek(sim.playheadTime + 1) }
    Shortcut { sequence: "B"; enabled: !sim.replay && showroom.screen !== "showroom"; onActivated: sim.placeBlockageAhead(false) }
    Shortcut { sequence: "C"; enabled: !sim.replay && showroom.screen !== "showroom"; onActivated: sim.clearBlockages() }
    Shortcut {
        sequence: "Escape"
        enabled: window.settingsOpen || window.insightsOpen || window.layersOpen
        onActivated: { window.settingsOpen = false; window.insightsOpen = false; window.layersOpen = false }
    }
    FolderDialog {
        id: recordingDialog
        title: "Open a recorded run directory"
        onAccepted: sim.loadRecording(String(selectedFolder))
    }

    component SmallLabel: Text {
        color: window.muted
        font.pixelSize: 10
        font.letterSpacing: 1.8
        font.weight: Font.DemiBold
    }
    component Rule: Rectangle { color: window.lineColor; height: 1 }
    // Line icons on a 24 unit grid, drawn for this app.
    component Icon: Shape {
        id: icon
        property string name
        property color tint: window.ink
        property real size: 20
        property real weight: 1.7
        readonly property bool solid: name === "play" || name === "pause"
        readonly property var paths: ({
            "play": "M8 5.5 L18.5 12 L8 18.5 Z",
            "pause": "M7 5 H10.5 V19 H7 Z M13.5 5 H17 V19 H13.5 Z",
            "reset": "M19 12 A7 7 0 1 1 12 5 H15.5 M13 2.5 L15.5 5 L13 7.5",
            "settings": "M4 7 H7 M11.4 7 H20 M4 17 H12.6 M17 17 H20 M11.4 7 A2.2 2.2 0 1 0 7 7 A2.2 2.2 0 1 0 11.4 7 M17 17 A2.2 2.2 0 1 0 12.6 17 A2.2 2.2 0 1 0 17 17",
            "layers": "M12 3.5 L20.5 8 L12 12.5 L3.5 8 Z M3.5 12 L12 16.5 L20.5 12 M3.5 16 L12 20.5 L20.5 16",
            "telemetry": "M3 12.5 H7 L9.5 6 L14 19 L16.5 12.5 H21",
            "car": "M3.5 16.5 V13 L6.2 8.4 Q6.6 7.8 7.4 7.8 H16.6 Q17.4 7.8 17.8 8.4 L20.5 13 V16.5 Z M3.5 13 H20.5 M7.5 16.5 V19 M16.5 16.5 V19",
            "folder": "M3.5 7 Q3.5 5.5 5 5.5 H9.5 L11.5 7.5 H19 Q20.5 7.5 20.5 9 V17.5 Q20.5 19 19 19 H5 Q3.5 19 3.5 17.5 Z",
            "exit": "M13.5 5 H6.5 Q5 5 5 6.5 V17.5 Q5 19 6.5 19 H13.5 M10.5 12 H20 M16.5 8.5 L20 12 L16.5 15.5",
            "close": "M6.5 6.5 L17.5 17.5 M17.5 6.5 L6.5 17.5",
            "chevronDown": "M6 9.5 L12 15.5 L18 9.5",
            "chevronUp": "M6 14.5 L12 8.5 L18 14.5",
            "chevronRight": "M9.5 6 L15.5 12 L9.5 18"
        })
        width: size
        height: size
        preferredRendererType: Shape.CurveRenderer
        ShapePath {
            strokeColor: icon.solid ? "transparent" : icon.tint
            strokeWidth: icon.weight
            fillColor: icon.solid ? icon.tint : "transparent"
            capStyle: ShapePath.RoundCap
            joinStyle: ShapePath.RoundJoin
            scale: Qt.size(icon.size / 24, icon.size / 24)
            PathSvg { path: icon.paths[icon.name] || "" }
        }
    }
    // A labelled value on one line of a card.
    component CardValue: Item {
        id: cardValue
        property string label
        property string value
        width: parent ? parent.width : 180
        height: 16
        SmallLabel { text: cardValue.label; font.pixelSize: 9; anchors.verticalCenter: parent.verticalCenter }
        Text {
            anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
            text: cardValue.value; color: window.ink; font.pixelSize: 11
        }
    }
    // One axle's slip angle against the angle where its tires peak. The bar spans 125% of the peak;
    // the tick marks the peak, and the bar turns red beyond it, where the tires are sliding.
    component SlipBar: Column {
        id: slipBar
        property string label
        property real slip: 0
        property real peak: 1
        readonly property real share: peak > 0 ? Math.abs(slip) / peak : 0
        spacing: 5
        width: parent ? parent.width : 180
        Item {
            width: parent.width; height: 12
            SmallLabel { text: slipBar.label; font.pixelSize: 9; anchors.verticalCenter: parent.verticalCenter }
            Text {
                objectName: slipBar.objectName + "Value"
                anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                text: Math.abs(slipBar.slip).toFixed(1) + "° of " + slipBar.peak.toFixed(1) + "°"
                color: window.ink; font.pixelSize: 11
            }
        }
        Rectangle {
            width: parent.width; height: 4; radius: 2; color: "#23292e"
            Rectangle {
                width: parent.width * Math.min(1, slipBar.share / 1.25); height: parent.height; radius: 2
                color: slipBar.share > 1 ? window.brakeColor : window.ink
            }
            Rectangle { x: parent.width / 1.25; y: -3; width: 1; height: 10; color: window.muted }
        }
    }
    component Metric: Column {
        id: metric
        property string label
        property string value
        spacing: 3
        SmallLabel { text: metric.label; font.pixelSize: 9; font.letterSpacing: 1.4 }
        Text { text: metric.value; color: window.ink; font.pixelSize: window.compact ? 14 : 16 }
    }
    component ValueRow: RowLayout {
        id: valueRow
        property string label
        property string value
        spacing: 10
        Text { text: valueRow.label; color: window.muted; font.pixelSize: window.compact ? 11 : 12; Layout.fillWidth: true }
        Text { text: valueRow.value; color: window.ink; font.pixelSize: window.compact ? 11 : 12; font.weight: Font.Medium }
    }
    component ActionButton: Button {
        id: control
        property bool selected: false
        property bool primary: false
        // One choice of a segmented control: flat until selected.
        property bool segment: false
        property string glyph: ""
        readonly property color textColor: primary ? "#0b0c0e" : segment ? (selected ? window.ink : window.muted)
                                                   : selected ? "#dbe7ff" : window.ink
        implicitWidth: 112
        implicitHeight: 40
        hoverEnabled: true
        opacity: enabled ? 1 : 0.35
        padding: 8
        contentItem: Item {
            Row {
                anchors.centerIn: parent
                spacing: 7
                Icon {
                    visible: control.glyph !== ""
                    name: control.glyph; size: 15; weight: 1.8; tint: control.textColor
                    anchors.verticalCenter: parent.verticalCenter
                }
                Text {
                    text: control.text
                    color: control.textColor
                    font.pixelSize: window.compact ? 12 : 13
                    font.weight: control.primary || control.selected ? Font.DemiBold : Font.Medium
                    anchors.verticalCenter: parent.verticalCenter
                }
            }
        }
        background: Rectangle {
            radius: control.segment ? 8 : control.primary ? height / 2 : 12
            color: control.primary ? (control.down ? "#cfd4d8" : control.hovered ? "#ffffff" : "#eceff1")
                 : control.segment ? (control.selected ? "#353b42" : control.down ? "#20ffffff" : control.hovered ? "#10ffffff" : "transparent")
                 : control.selected ? "#1e3056" : control.down ? "#262b31" : control.hovered ? "#1d2126" : "#16191d"
            border.color: control.activeFocus ? window.accent : control.segment || control.primary ? "transparent"
                        : control.selected ? "#3a5c9e" : window.hairline
            border.width: 1
        }
    }
    component IconButton: Button {
        id: iconButton
        property string glyph
        property string tip
        property bool active: false
        property real iconSize: window.compact ? 19 : 21
        implicitWidth: window.compact ? 40 : 44
        implicitHeight: implicitWidth
        hoverEnabled: true
        opacity: enabled ? 1 : 0.35
        contentItem: Item {
            Icon { anchors.centerIn: parent; name: iconButton.glyph; size: iconButton.iconSize; tint: iconButton.active ? window.accent : window.ink }
        }
        background: Rectangle {
            radius: Math.min(12, height / 2)
            color: iconButton.active ? "#244c8dff" : iconButton.down ? "#24ffffff" : iconButton.hovered ? "#14ffffff" : "transparent"
        }
        // Its name above it while the pointer rests on it.
        Rectangle {
            visible: iconButton.hovered && iconButton.tip !== ""
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.top
            anchors.bottomMargin: 8
            width: tipText.implicitWidth + 18
            height: tipText.implicitHeight + 10
            radius: 8
            color: window.raised
            border.color: window.hairline
            z: 5
            Text { id: tipText; anchors.centerIn: parent; text: iconButton.tip; color: window.ink; font.pixelSize: 11 }
        }
    }
    // Choices shown side by side in one rounded well; the selected one is raised.
    component Segmented: Rectangle {
        default property alias segments: segmentRow.data
        implicitHeight: window.compact ? 34 : 36
        radius: 10
        color: "#0affffff"
        border.color: window.hairline
        RowLayout { id: segmentRow; anchors.fill: parent; anchors.margins: 3; spacing: 3 }
    }
    // A named setting with its control beside the name.
    component SettingRow: RowLayout {
        id: settingRow
        property string label
        property real labelWidth: window.compact ? 70 : 78
        width: parent ? parent.width : 300
        spacing: 12
        Text { text: settingRow.label; color: window.muted; font.pixelSize: 12; Layout.preferredWidth: settingRow.labelWidth; elide: Text.ElideRight }
    }
    component SectionTitle: Item {
        id: sectionTitle
        property string title
        property string note
        width: parent ? parent.width : 300
        height: 18
        SmallLabel { text: sectionTitle.title; color: window.ink; anchors.verticalCenter: parent.verticalCenter }
        Text {
            anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
            text: sectionTitle.note; color: window.faint; font.pixelSize: 11
        }
    }
    component Card: Rectangle {
        radius: 18
        color: window.glass
        border.color: window.hairline
    }
    component Toggle: Switch {
        id: toggle
        implicitWidth: 36
        implicitHeight: 22
        padding: 0
        indicator: Rectangle {
            implicitWidth: 36; implicitHeight: 20
            y: (toggle.height - height) / 2
            radius: 10
            color: toggle.checked ? window.accent : "#2c3137"
            Rectangle {
                x: toggle.checked ? parent.width - width - 2 : 2; y: 2
                width: 16; height: 16; radius: 8; color: "#ffffff"
                Behavior on x { NumberAnimation { duration: 120 } }
            }
        }
        contentItem: Item {}
    }
    component Swatch: Rectangle { width: 12; height: 4; radius: 2; anchors.verticalCenter: parent ? parent.verticalCenter : undefined }
    // One entry of the layers panel: what a colour in the scene means, and a switch that hides its drawing.
    component LegendRow: RowLayout {
        id: legendRow
        property string label
        property bool toggleable: false
        property bool shown: true
        signal toggled()
        default property alias swatches: swatchRow.data
        width: parent ? parent.width : 220
        spacing: 10
        Row { id: swatchRow; spacing: 4; Layout.preferredWidth: 46; Layout.preferredHeight: 12 }
        Text { text: legendRow.label; color: window.ink; font.pixelSize: 12; Layout.fillWidth: true; elide: Text.ElideRight }
        Toggle { visible: legendRow.toggleable; checked: legendRow.shown; onToggled: legendRow.toggled() }
    }
    // The road and its edges, fading into the backdrop with distance from the car. Overview fades nothing. Opaque unless
    // given a clarity and blending, as the road is, to show the car's reflection beneath it.
    component GroundMaterial: CustomMaterial {
        property vector3d focus: drivingView.ground
        property real fadeNear: 26 * drivingView.reach
        property real fadeFar: 95 * drivingView.reach
        property vector3d horizon: Qt.vector3d(0.031, 0.035, 0.043)
        property real gain: 1
        property real clarity: 0
        shadingMode: CustomMaterial.Unshaded
        cullMode: Material.NoCulling
        vertexShader: "shaders/scene.vert"
        fragmentShader: "shaders/ground.frag"
    }
    // Lines and ribbons over the road, blended, with their own alpha, a strength and the same fade.
    component OverlayMaterial: CustomMaterial {
        property vector3d focus: drivingView.ground
        property real fadeNear: 30 * drivingView.reach
        property real fadeFar: 110 * drivingView.reach
        property real strength: 1
        shadingMode: CustomMaterial.Unshaded
        sourceBlend: CustomMaterial.SrcAlpha
        destinationBlend: CustomMaterial.OneMinusSrcAlpha
        // Coverage accumulates as it does over an opaque backdrop, so what the scene lets through beneath is exact.
        sourceAlphaBlend: CustomMaterial.One
        destinationAlphaBlend: CustomMaterial.OneMinusSrcAlpha
        cullMode: Material.NoCulling
        vertexShader: "shaders/scene.vert"
        fragmentShader: "shaders/overlay.frag"
    }
    // The four-wheel car's wheels, laid out as on the car: state, slip ratio, the tire's force in its friction circle,
    // and vertical load against the load at rest.
    component WheelTile: Rectangle {
        id: wheelTile
        required property int index
        objectName: ["frontLeftWheel", "frontRightWheel", "rearLeftWheel", "rearRightWheel"][index]
        readonly property string wheelState: sim.wheelStates.length > index ? sim.wheelStates[index] : "GRIP"
        readonly property real slipRatio: sim.wheelSlipRatios.length > index ? sim.wheelSlipRatios[index] : 0
        readonly property real load: sim.wheelLoads.length > index ? sim.wheelLoads[index] : 0
        readonly property real staticLoad: sim.staticWheelLoads.length > index ? sim.staticWheelLoads[index] : 1
        readonly property real frictionLongitudinal: sim.wheelFrictionUse.length > index ? sim.wheelFrictionUse[index][0] : 0
        readonly property real frictionLateral: sim.wheelFrictionUse.length > index ? sim.wheelFrictionUse[index][1] : 0
        readonly property bool alarmed: wheelState === "LOCK" || wheelState === "LIFT"
        readonly property color stateColor: alarmed ? window.brakeColor : wheelState === "GRIP" ? window.ink : window.holdColor
        height: 50
        radius: 10
        color: alarmed ? "#3a1a1d" : "#0dffffff"
        border.color: alarmed ? window.brakeColor : wheelState === "GRIP" ? "transparent" : window.holdColor
        Text {
            x: 8; y: 5
            text: ["FL", "FR", "RL", "RR"][wheelTile.index]; color: window.muted; font.pixelSize: 9
        }
        Text {
            objectName: wheelTile.objectName + "State"
            anchors.right: parent.right; anchors.rightMargin: 8; y: 5
            text: wheelTile.wheelState; font.pixelSize: 9; font.weight: Font.DemiBold
            color: wheelTile.stateColor
        }
        // Friction circle seen from above: forward force up, leftward force left, the rim at peak grip.
        Item {
            objectName: wheelTile.objectName + "Friction"
            x: 8; y: 20
            width: 22; height: 22
            Rectangle { anchors.fill: parent; radius: width / 2; color: "transparent"; border.color: "#3b444a" }
            Rectangle { x: parent.width / 2; y: 2; width: 1; height: parent.height - 4; color: "#262d32" }
            Rectangle {
                objectName: wheelTile.objectName + "FrictionDot"
                width: 6; height: 6; radius: 3
                x: (parent.width - width) / 2 - wheelTile.frictionLateral * (parent.width - width) / 2
                y: (parent.height - height) / 2 - wheelTile.frictionLongitudinal * (parent.height - height) / 2
                color: Math.sqrt(wheelTile.frictionLateral * wheelTile.frictionLateral + wheelTile.frictionLongitudinal * wheelTile.frictionLongitudinal) > 0.95
                       ? window.holdColor : window.ink
            }
        }
        Text {
            x: 37; y: 18
            text: (wheelTile.slipRatio >= 0 ? "+" : "") + (wheelTile.slipRatio * 100).toFixed(0) + "% slip"
            color: window.ink; font.pixelSize: 9
        }
        Text {
            objectName: wheelTile.objectName + "Load"
            x: 37; y: 29
            text: (wheelTile.load / 1000).toFixed(2) + " kN"
            color: window.ink; font.pixelSize: 9
        }
        // Load bar: its full width is twice the load at rest, and the tick marks the load at rest.
        Rectangle {
            x: 37; y: 42
            width: parent.width - 45; height: 3
            color: "#23292e"
            Rectangle {
                objectName: wheelTile.objectName + "LoadBar"
                width: parent.width * Math.min(1, Math.max(0, wheelTile.load / (2 * wheelTile.staticLoad)))
                height: parent.height
                color: "#8fb3c4"
            }
            Rectangle { x: parent.width / 2; y: -2; width: 1; height: 7; color: window.muted }
        }
    }

    // The cameras both views look through: behind and above the car, looking along the road, the car low in the frame and
    // the road ahead rising to the horizon, as a driver's display draws it; or the whole circuit from above.
    Node {
        id: cameraRig
        Node {
            position: Qt.vector3d(sim.x, 0, -sim.y)
            eulerRotation.y: sim.yawDegrees - 90
            PerspectiveCamera {
                id: followCamera
                // Closer behind a kart-sized car, which is about a third of a Formula One car's length.
                readonly property real reach: sim.kartMode ? 0.4 : 1
                position: Qt.vector3d(0, 4.9*reach, 9.8*reach)
                eulerRotation.x: -18
                fieldOfView: 50
                clipNear: 0.1
                clipFar: 1500
            }
            PerspectiveCamera {
                id: firstPersonCamera
                objectName: "firstPersonCamera"
                // Driver eye point relative to the true rear axle; local forward is -Z.
                position: Qt.vector3d(0, sim.kartMode ? 0.72 : 1.05, -sim.wheelbase * 0.35)
                eulerRotation.x: -2
                fieldOfView: 75
                clipNear: 0.04
                clipFar: 1500
            }
        }
        PerspectiveCamera {
            id: overviewCamera
            position: Qt.vector3d((window.bounds.minX + window.bounds.maxX) / 2,
                                 window.trackSpan * 1.25 + 15,
                                 -(window.bounds.minY + window.bounds.maxY) / 2 + (window.trackSpan * 1.25 + 15) * 0.57735)
            eulerRotation.x: -60
            fieldOfView: 51
            clipNear: 0.1
            clipFar: 3000
        }
    }

    // The car's reflection in the road, drawn on its own beneath the scene and seen through the road's clarity: the car
    // mirrored in the surface through the same camera, lit by its lights mirrored with it, so what faced the ground faces
    // the dark; fading with depth beneath the surface and softened, as asphalt reflects. Appearance only.
    View3D {
        id: reflectionView
        objectName: "reflectionView"
        visible: drivingView.visible && window.showReflections && sim.cameraMode !== 2
        anchors.fill: parent
        importScene: cameraRig
        camera: drivingView.camera
        environment: SceneEnvironment {
            clearColor: window.backdrop
            backgroundMode: SceneEnvironment.Color
            lightProbe: drivingView.studioLit ? studioLightBelow : null
            probeExposure: 1.0
            // The studio turned over: what shone from above now shines from below.
            probeOrientation: Qt.vector3d(0, 0, 180)
            fog: Fog {
                enabled: true
                color: window.backdrop
                depthEnabled: false
                heightEnabled: true
                leastIntenseY: 0.3
                mostIntenseY: -1.2
                heightCurve: 0.8
                density: 0.9
            }
        }
        Texture { id: studioLightBelow; source: liveCars.environment }
        DirectionalLight { visible: drivingView.studioLit; eulerRotation: Qt.vector3d(50, 35, 0); brightness: 0.7; color: "#fff4e8" }
        DirectionalLight {
            visible: !drivingView.studioLit
            eulerRotation: Qt.vector3d(-55, -35, 0); brightness: 1.4; color: "#dde8ee"; ambientColor: "#5a6a74"
        }
        DirectionalLight { visible: !drivingView.studioLit; eulerRotation: Qt.vector3d(-35, 145, 0); brightness: 0.9; color: "#aebdcb" }
        PointLight {
            visible: !drivingView.studioLit
            position: Qt.vector3d(sim.x + 8, -15, -sim.y + 10)
            color: "#e5eff3"; brightness: 1.2
            constantFade: 1; linearFade: 0; quadraticFade: 0
        }
        Car {
            objectName: "carReflection"
            reflection: true
            scale: Qt.vector3d(1, -1, 1)
            position: sceneCar.position
            eulerRotation.y: sceneCar.eulerRotation.y
            appearance: sceneCar.appearance
            asset: sceneCar.asset
            drawnWheelbase: sceneCar.drawnWheelbase
            steeringDegrees: sceneCar.steeringDegrees
            wheelStates: sceneCar.wheelStates
            speed: sceneCar.speed
            rolling: sceneCar.rolling
            slipRatios: sceneCar.slipRatios
        }
        layer.enabled: true
        layer.effect: MultiEffect { blurEnabled: true; blur: 0.3; blurMax: 24 }
    }

    View3D {
        id: drivingView
        objectName: "drivingView"
        visible: showroom.screen !== "showroom"
        anchors.fill: parent
        importScene: cameraRig
        camera: sim.cameraMode === 2 ? firstPersonCamera : (sim.overview ? overviewCamera : followCamera)
        // The car on the ground, where every fade is measured from; the overview fades nothing.
        readonly property vector3d ground: Qt.vector3d(sim.x, 0, -sim.y)
        readonly property real reach: sim.overview ? 1000 : 1
        // An exported car with the studio it was exported in; the silhouette keeps the scene's own lights.
        readonly property bool studioLit: sceneCar.live && liveCars.environment.toString().length > 0
        environment: SceneEnvironment {
            // Clear, so the reflection beneath shows where the road lets it; the window behind is the same backdrop.
            clearColor: "transparent"
            backgroundMode: SceneEnvironment.Transparent
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
            // The exported cars are lit by the studio they were exported with, as the showroom lit them.
            lightProbe: drivingView.studioLit ? studioLight : null
            probeExposure: 1.9
        }
        Texture { id: studioLight; source: liveCars.environment }
        DirectionalLight { visible: drivingView.studioLit; eulerRotation: Qt.vector3d(-50, 35, 0); brightness: 0.7; color: "#fff4e8" }
        DirectionalLight {
            visible: !drivingView.studioLit
            eulerRotation: Qt.vector3d(55, -35, 0); brightness: 1.4; color: "#dde8ee"; ambientColor: "#5a6a74"
        }
        DirectionalLight { visible: !drivingView.studioLit; eulerRotation: Qt.vector3d(35, 145, 0); brightness: 0.9; color: "#aebdcb" }
        PointLight {
            visible: !drivingView.studioLit
            position: Qt.vector3d(sim.x + 8, 15, -sim.y + 10)
            color: "#e5eff3"
            brightness: 1.2
            constantFade: 1
            linearFade: 0
            quadraticFade: 0
        }

        // Unseen ground beyond the edges, so the reflection under the road shows through the road alone.
        Model {
            objectName: "trackShoulders"
            geometry: TrackGeometry { objectName: "shoulderGeometry"; kind: 11; bridge: sim }
            materials: GroundMaterial {}
        }
        // The road is seen through a little, to the car's reflection beneath it. It is drawn first of everything blended,
        // and into the depth buffer, so nothing beneath its surface is drawn over it afterwards.
        Model {
            objectName: "trackRoad"
            depthBias: 400000
            geometry: TrackGeometry { objectName: "roadGeometry"; kind: 0; bridge: sim }
            materials: GroundMaterial {
                clarity: window.showReflections ? 0.32 : 0
                sourceBlend: CustomMaterial.SrcAlpha
                destinationBlend: CustomMaterial.OneMinusSrcAlpha
                sourceAlphaBlend: CustomMaterial.One
                destinationAlphaBlend: CustomMaterial.OneMinusSrcAlpha
                depthDrawMode: Material.AlwaysDepthDraw
            }
        }
        Model {
            objectName: "trackEdges"
            geometry: TrackGeometry { kind: 1; bridge: sim }
            materials: GroundMaterial {}
        }
        // Transparent layers are drawn farthest first; the depth biases keep them in one order under the car's ribbon.
        Model {
            objectName: "latticeAhead"
            visible: sim.latticeAvailable && window.latticeVisible
            depthBias: 300000
            geometry: TrackGeometry { objectName: "latticeGeometry"; kind: 6; bridge: sim }
            materials: OverlayMaterial { strength: 0.22; fadeNear: 4 * drivingView.reach; fadeFar: 30 * drivingView.reach }
        }
        Model {
            objectName: "comparisonLine"
            visible: !sim.replay && !sim.kartMode && sim.racingLineReady && window.showReferenceLine
            depthBias: 200000
            geometry: TrackGeometry { objectName: "comparisonLineGeometry"; kind: 5; bridge: sim }
            materials: OverlayMaterial { strength: 0.38 }
        }
        Model {
            objectName: "rejectedAlternatives"
            visible: sim.evaluatedOptions > 1 && window.showAlternatives
            depthBias: 100000
            geometry: TrackGeometry { objectName: "alternativesGeometry"; kind: 4; bridge: sim }
            materials: OverlayMaterial { strength: 0.85 }
        }
        Model {
            objectName: "kartCoachingLine"
            visible: sim.kartMode && window.showKartGuide
            depthBias: 50000
            geometry: TrackGeometry { objectName: "kartCoachingGeometry"; kind: 12; bridge: sim }
            materials: OverlayMaterial { fadeNear: 200; fadeFar: 260 }
        }
        Model {
            objectName: "kartStartLine"
            visible: sim.kartMode
            geometry: TrackGeometry { objectName: "kartStartGeometry"; kind: 13; bridge: sim }
            materials: OverlayMaterial { fadeNear: 200; fadeFar: 260 }
        }
        Model {
            objectName: "plannedTrajectory"
            visible: sim.predictionAvailable && window.predictionVisible
            geometry: TrackGeometry { objectName: "predictionGeometry"; kind: 2; bridge: sim }
            materials: OverlayMaterial { fadeNear: 200 * drivingView.reach; fadeFar: 260 * drivingView.reach }
        }
        // The course's cones, ground truth, and what the simulated perception's newest frames made of them
        // (decision 0030), placed from where the car truly was: the evaluation's view, never what the car drives on.
        Model {
            objectName: "courseCones"
            visible: sim.perceptionFitted
            geometry: TrackGeometry { objectName: "conesGeometry"; kind: 7; bridge: sim }
            materials: DefaultMaterial { diffuseColor: "white"; vertexColorsEnabled: true; lighting: DefaultMaterial.NoLighting; cullMode: Material.NoCulling }
        }
        Model {
            objectName: "simulatedDetections"
            visible: sim.perceptionFitted && sim.perceptionFrameAvailable
            geometry: TrackGeometry { objectName: "detectionsGeometry"; kind: 8; bridge: sim }
            materials: DefaultMaterial { diffuseColor: "white"; vertexColorsEnabled: true; lighting: DefaultMaterial.NoLighting; cullMode: Material.NoCulling }
        }
        // Cones the judge saw knocked down or out, whether or not the car perceives the course (decision 0032).
        Model {
            objectName: "knockedCones"
            visible: sim.conesHit > 0
            geometry: TrackGeometry { objectName: "knockedGeometry"; kind: 10; bridge: sim }
            materials: DefaultMaterial { diffuseColor: "white"; vertexColorsEnabled: true; lighting: DefaultMaterial.NoLighting; cullMode: Material.NoCulling }
        }
        // The path the car believes from those detections, where its believed pose placed them (decision 0031).
        Model {
            objectName: "believedPath"
            visible: sim.drivingOnCones
            geometry: TrackGeometry { objectName: "believedGeometry"; kind: 9; bridge: sim }
            materials: DefaultMaterial { diffuseColor: "white"; vertexColorsEnabled: true; lighting: DefaultMaterial.NoLighting; cullMode: Material.NoCulling }
        }
        Model {
            objectName: "statedBlockages"
            visible: sim.obstructionCount > 0
            geometry: TrackGeometry { objectName: "blockageGeometry"; kind: 3; bridge: sim }
            materials: DefaultMaterial { diffuseColor: "white"; vertexColorsEnabled: true; lighting: DefaultMaterial.NoLighting; cullMode: Material.NoCulling }
        }
        // Direction of travel at the rear axle beside the heading, drawn on the road: the angle between them is the sideslip.
        Node {
            objectName: "velocityVector"
            visible: sim.tireSlipModeled && sim.speedKmh > 3.6 && window.motionArrowVisible
            readonly property real length: Math.max(1.0, sim.speedKmh / 3.6 * 0.6)  // 0.6 s of travel
            // Drawn finer beside a kart-sized car (decision 0038).
            readonly property real size: sim.kartMode ? 0.45 : 1
            position: Qt.vector3d(sim.x, 0.09, -sim.y)
            Node {
                objectName: "headingLine"
                eulerRotation.y: sim.yawDegrees - 90
                Model {
                    source: "#Cube"; position: Qt.vector3d(0, 0, -parent.parent.length / 2)
                    scale: Qt.vector3d(0.0005*parent.parent.size, 0.0005*parent.parent.size, parent.parent.length / 100)
                    materials: DefaultMaterial { diffuseColor: "#7d8a92"; lighting: DefaultMaterial.NoLighting }
                }
            }
            Node {
                objectName: "velocityArrow"
                eulerRotation.y: sim.yawDegrees + sim.rearSideslipDegrees - 90
                Model {
                    source: "#Cube"; position: Qt.vector3d(0, 0, -parent.parent.length / 2)
                    scale: Qt.vector3d(0.0012*parent.parent.size, 0.0008*parent.parent.size, parent.parent.length / 100)
                    materials: DefaultMaterial { id: arrowPaint; diffuseColor: "#dfe7ec"; lighting: DefaultMaterial.NoLighting }
                }
                Model {
                    source: "#Cube"; position: Qt.vector3d(0.2*parent.parent.size, 0, -parent.parent.length + 0.35*parent.parent.size); eulerRotation.y: 30
                    scale: Qt.vector3d(0.0012*parent.parent.size, 0.0008*parent.parent.size, 0.008*parent.parent.size); materials: [arrowPaint]
                }
                Model {
                    source: "#Cube"; position: Qt.vector3d(-0.2*parent.parent.size, 0, -parent.parent.length + 0.35*parent.parent.size); eulerRotation.y: -30
                    scale: Qt.vector3d(0.0012*parent.parent.size, 0.0008*parent.parent.size, 0.008*parent.parent.size); materials: [arrowPaint]
                }
            }
        }
        // The showroom's car where it is built (decision 0036), else its silhouette; appearance only, at the plant's pose.
        Car {
            id: sceneCar
            objectName: "simulatedCar"
            visible: sim.cameraMode !== 2
            position: Qt.vector3d(sim.x, 0, -sim.y)
            eulerRotation.y: sim.yawDegrees - 90
            appearance: sim.carAppearance
            asset: liveCars.cars[sim.carAppearance]
            drawnWheelbase: sim.kartMode ? sim.wheelbase : 0
            steeringDegrees: sim.steeringDegrees
            wheelStates: sim.wheelStates
            speed: sim.speedKmh / 3.6
            rolling: sim.running
            slipRatios: sim.wheelSlipRatios
        }
        // The theoretical best lap (decision 0034): a translucent outline of the optimum's car, replayed from its artefact
        // by the time since the live lap began. An offline optimum for its model, never a second plant. Origin at the rear
        // axle and forward along local -Z, as the car's: the wheelbase and 1.4 m of overhang, 1.8 m wide.
        Node {
            id: optimalGhost
            objectName: "optimalGhost"
            visible: sim.ghostVisible && window.showGhost
            position: Qt.vector3d(sim.ghostX, 0, -sim.ghostY)
            eulerRotation.y: sim.ghostYawDegrees - 90
            Model {
                source: "#Cube"
                position: Qt.vector3d(0, 0.35, -sim.ghostWheelbase / 2)
                scale: Qt.vector3d(1.8 / 100, 0.6 / 100, (sim.ghostWheelbase + 1.4) / 100)
                materials: DefaultMaterial { diffuseColor: "#e4ecf0"; opacity: 0.3; lighting: DefaultMaterial.NoLighting }
            }
        }
    }

    // Shade the top and bottom of the scene so the readouts over it stay legible.
    Rectangle {
        anchors.top: parent.top
        width: parent.width
        height: window.compact ? 180 : 230
        gradient: Gradient {
            GradientStop { position: 0; color: "#e008090b" }
            GradientStop { position: 1; color: "#0008090b" }
        }
    }
    Rectangle {
        anchors.bottom: parent.bottom
        width: parent.width
        height: window.compact ? 190 : 250
        gradient: Gradient {
            GradientStop { position: 0; color: "#0008090b" }
            GradientStop { position: 1; color: "#e808090b" }
        }
    }

    // Each action the decision in force compared by time (decision 0023), labelled with its estimated time 15 m along its
    // predicted line, in its action's colour; the chosen one framed. Two paths that have barely separated anchor their
    // labels within a few pixels of each other, so a label that would cover an earlier one sits above it instead.
    Repeater {
        id: actionTimeLabels
        objectName: "actionTimeLabels"
        model: sim.actionTimes
        Rectangle {
            required property var modelData
            required property int index
            objectName: "actionTimeLabel" + index
            // Mapped again whenever the camera itself has moved, not merely the car it follows.
            readonly property vector3d at: {
                drivingView.camera.scenePosition; drivingView.camera.sceneRotation; drivingView.width; drivingView.height
                return drivingView.mapFrom3DScene(Qt.vector3d(modelData.x, 0.3, -modelData.y))
            }
            readonly property string text: modelData.label + "  " + modelData.seconds.toFixed(2) + " s"
            // How many labels this one must clear: those before it whose own place it would otherwise cover.
            readonly property int stackLevel: {
                var level = 0
                for (var j = 0; j < index; ++j) {
                    var other = actionTimeLabels.itemAt(j)
                    if (!other || !other.visible) continue
                    if (Math.abs(other.at.x - at.x) < (width + other.width) / 2 && Math.abs(other.at.y - at.y) < height * 1.5)
                        level = Math.max(level, other.stackLevel + 1)
                }
                return level
            }
            visible: at.z > 0 && at.x > 0 && at.x < drivingView.width && at.y > 0 && at.y < drivingView.height
            x: drivingView.x + at.x - width / 2
            y: drivingView.y + at.y - height - 4 - stackLevel * (height + 3)
            width: labelText.implicitWidth + 18
            height: labelText.implicitHeight + 8
            radius: height / 2
            color: window.raised
            border.width: modelData.chosen ? 1.5 : 1
            border.color: modelData.chosen ? sim.actionColorName(modelData.action) : window.hairline
            opacity: modelData.clear ? 1 : 0.55
            Text {
                id: labelText
                anchors.centerIn: parent
                text: parent.text
                color: sim.actionColorName(parent.modelData.action)
                font.pixelSize: 11
                font.weight: parent.modelData.chosen ? Font.DemiBold : Font.Normal
            }
        }
    }

    // What the translucent car is, said where it is.
    Rectangle {
        id: ghostLabel
        objectName: "ghostLabel"
        readonly property vector3d at: {
            drivingView.camera.scenePosition; drivingView.camera.sceneRotation; drivingView.width; drivingView.height
            return drivingView.mapFrom3DScene(Qt.vector3d(sim.ghostX, 1.2, -sim.ghostY))
        }
        visible: optimalGhost.visible && at.z > 0 && at.x > 0 && at.x < drivingView.width && at.y > 0 && at.y < drivingView.height
        x: drivingView.x + at.x - width / 2
        y: drivingView.y + at.y - height - 6
        width: ghostLabelText.implicitWidth + 18
        height: ghostLabelText.implicitHeight + 8
        radius: height / 2
        color: window.raised
        border.color: window.hairline
        Text {
            id: ghostLabelText
            anchors.centerIn: parent
            text: "Offline optimum"
            color: "#e4ecf0"
            font.pixelSize: 11
        }
    }

    // Speed, the plan's target shown as a road sign shows a limit, and the planned motion: the car's own dial.
    Column {
        id: speedCluster
        objectName: "speedCluster"
        x: window.margin + 4
        y: window.margin
        spacing: 0
        Row {
            spacing: 9
            Rectangle {
                width: 7; height: 7; radius: 3.5
                anchors.verticalCenter: parent.verticalCenter
                color: sim.running ? window.accent : window.faint
            }
            SmallLabel {
                objectName: "modeLabel"
                text: sim.replay ? "REPLAY  ·  RECORDED RUN" : sim.kartMode ? "GOKART  ·  GOKARTCENTRALEN GÖTEBORG"
                      : sim.drivingOnCones ? "SIMULATION  ·  ON CONES" : "SIMULATION  ·  KNOWN TRACK"
                color: window.muted
            }
        }
        Text {
            objectName: "speedValue"
            text: sim.speedKmh.toFixed(0)
            color: window.ink
            font.pixelSize: window.compact ? 86 : 108
            font.weight: Font.Light
            font.letterSpacing: -3
            topPadding: window.compact ? -2 : -4
            bottomPadding: window.compact ? -10 : -14
        }
        Text { text: "km/h"; color: window.muted; font.pixelSize: 13; font.letterSpacing: 1; leftPadding: 5 }
        Item { width: 1; height: window.compact ? 14 : 22 }
        Row {
            objectName: "planSignRow"
            visible: !sim.humanDriving
            spacing: 14
            Rectangle {
                width: window.compact ? 42 : 48
                height: window.compact ? 52 : 60
                radius: 8
                color: "#eef1f3"
                Rectangle { anchors.fill: parent; anchors.margins: 3; radius: 6; color: "transparent"; border.color: "#1a1d20"; border.width: 1.5 }
                Column {
                    anchors.centerIn: parent
                    spacing: -1
                    Text {
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: "PLAN"; color: "#1a1d20"; font.pixelSize: 8; font.weight: Font.DemiBold; font.letterSpacing: 1
                    }
                    Text {
                        objectName: "targetSpeedValue"
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: sim.targetSpeedKmh.toFixed(0)
                        color: "#0b0c0e"; font.pixelSize: window.compact ? 19 : 22; font.weight: Font.DemiBold
                    }
                }
            }
            Row {
                anchors.verticalCenter: parent.verticalCenter
                spacing: 8
                Rectangle { width: 8; height: 8; radius: 4; color: window.motionColor; anchors.verticalCenter: parent.verticalCenter }
                SmallLabel { objectName: "motionLabel"; text: window.motionLabel; color: window.motionColor; font.pixelSize: 11; font.letterSpacing: 1.6 }
            }
        }
    }

    // What drives the car and on which model, as a driver's display names its assistance; it opens the settings.
    Button {
        id: driverChip
        objectName: "driverChip"
        anchors.right: parent.right
        anchors.rightMargin: window.margin - 8
        y: window.margin - 8
        hoverEnabled: true
        padding: 8
        onClicked: window.settingsOpen = !window.settingsOpen
        contentItem: Column {
            spacing: 2
            Text {
                objectName: "driverName"
                anchors.right: parent.right
                text: sim.replay ? "Recorded run" : sim.forgivenessActive ? "You drive · " + (sim.forgivenessManual ? "Manual assist" : "Assist " + sim.forgivenessLevel.toFixed(0))
                      : sim.humanDriving ? "You drive" : window.controllerName
                color: window.accent
                font.pixelSize: window.compact ? 15 : 17
                font.weight: Font.DemiBold
            }
            Row {
                anchors.right: parent.right
                spacing: 4
                Icon { name: "chevronDown"; size: 13; weight: 2; tint: window.muted; anchors.verticalCenter: parent.verticalCenter }
                Text {
                    text: window.vehicleName + (sim.drivingOnCones ? "  ·  on cones" : sim.sensorsFitted ? "  ·  measured" : "")
                    color: window.muted; font.pixelSize: 12
                }
            }
        }
        background: Rectangle { radius: 12; color: driverChip.down ? "#20ffffff" : driverChip.hovered ? "#12ffffff" : "transparent" }
    }

    Column {
        id: practiceClock
        objectName: "practiceClock"
        visible: sim.kartMode && sim.humanDriving
        anchors.right: parent.right; anchors.rightMargin: window.margin
        y: driverChip.y + driverChip.height + 8
        spacing: 3
        Text {
            objectName: "currentPracticeLap"
            anchors.right: parent.right
            text: window.lapClock(sim.practiceLapSeconds)
            color: "white"; font.pixelSize: window.compact ? 26 : 30
            font.weight: Font.Medium; font.family: "Consolas"
        }
        Text {
            objectName: "previousPracticeLap"
            anchors.right: parent.right
            text: "Previous  " + (sim.previousPracticeLap > 0 ? window.lapClock(sim.previousPracticeLap) : "—")
            color: "white"; font.pixelSize: 12; font.family: "Consolas"
        }
    }

    // The circuit and the car on it, with the judge's view either side of the map's foot (decision 0032): the last lap as
    // timed from the start line, and what the penalties cost and were for, with the verdict when there is one.
    Card {
        id: mapCard
        objectName: "mapCard"
        anchors.right: parent.right
        anchors.rightMargin: window.margin
        y: practiceClock.visible ? practiceClock.y + practiceClock.height + 12 : driverChip.y + driverChip.height + 6
        width: window.sideWidth
        height: window.compact ? 148 : 182
        Item {
            x: 16; y: 12
            width: parent.width - 32; height: 14
            SmallLabel { objectName: "referenceMapLabel"; text: "CIRCUIT"; anchors.verticalCenter: parent.verticalCenter }
            Text {
                anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                text: practiceClock.visible ? "Lap " + (sim.practiceLaps + 1) : (sim.progress * 100).toFixed(0) + "%"
                color: window.faint; font.pixelSize: 10
            }
        }
        Canvas {
            id: circuitMap
            objectName: "circuitOverview"
            x: 14; y: 30
            width: parent.width - 28
            height: parent.height - 40
            onPaint: {
                var ctx = getContext("2d")
                ctx.reset()
                var points = window.cachedPoints
                if (!points || points.length < 2) return
                var b = window.bounds
                // The track above the judge's readouts at the foot of the map.
                var area = height - 30
                var factor = Math.min((width - 20) / Math.max(1, b.maxX-b.minX), (area - 8) / Math.max(1, b.maxY-b.minY))
                function px(v) { return width/2 + (v-(b.maxX+b.minX)/2)*factor }
                function py(v) { return area/2 - (v-(b.maxY+b.minY)/2)*factor }
                ctx.lineCap = "round"
                ctx.lineJoin = "round"
                ctx.beginPath()
                ctx.moveTo(px(points[0].x), py(points[0].y))
                for (var i=1; i<points.length; ++i) ctx.lineTo(px(points[i].x), py(points[i].y))
                ctx.closePath()
                ctx.lineWidth=7; ctx.strokeStyle="#20252a"; ctx.stroke()
                var other = window.cachedComparison
                if (other && other.length > 1) {
                    ctx.beginPath()
                    ctx.moveTo(px(other[0].x), py(other[0].y))
                    for (var k=1; k<other.length; ++k) ctx.lineTo(px(other[k].x), py(other[k].y))
                    ctx.lineWidth=1; ctx.strokeStyle=sim.lineMode === 1 ? "#9aa7ad" : "#7fc4e8"; ctx.stroke()
                }
                ctx.lineWidth=2
                for (var j=0; j<points.length; ++j) {
                    var next=points[(j+1)%points.length]
                    ctx.beginPath(); ctx.moveTo(px(points[j].x),py(points[j].y)); ctx.lineTo(px(next.x),py(next.y))
                    ctx.strokeStyle=window.trajectoryColor(points[j].acceleration); ctx.stroke()
                }
                if (sim.ghostVisible) {
                    ctx.beginPath(); ctx.arc(px(sim.ghostX),py(sim.ghostY),4.5,0,Math.PI*2)
                    ctx.lineWidth=1.5; ctx.strokeStyle="#e4ecf0"; ctx.stroke()
                }
                ctx.beginPath(); ctx.arc(px(sim.x),py(sim.y),7,0,Math.PI*2); ctx.fillStyle="rgba(76, 141, 255, 0.3)"; ctx.fill()
                ctx.beginPath(); ctx.arc(px(sim.x),py(sim.y),3.5,0,Math.PI*2); ctx.fillStyle="#ffffff"; ctx.fill()
            }
            Column {
                objectName: "lastLapReadout"
                anchors.left: parent.left; anchors.bottom: parent.bottom
                spacing: 2
                SmallLabel { text: practiceClock.visible ? "SESSION BEST" : "LAST LAP"; font.pixelSize: 8; font.letterSpacing: 1.2 }
                Text { objectName: "lastLapMetric"; text: practiceClock.visible ? (sim.bestPracticeLap > 0 ? window.lapClock(sim.bestPracticeLap) : "—")
                      : sim.lastLapSeconds > 0 ? sim.lastLapSeconds.toFixed(2) + " s" : "-"; color: window.ink; font.pixelSize: 11 }
            }
            Column {
                objectName: "penaltyReadout"
                anchors.right: parent.right; anchors.bottom: parent.bottom
                spacing: 2
                SmallLabel {
                    objectName: "judgeVerdict"; anchors.right: parent.right; font.pixelSize: 8; font.letterSpacing: 1.2
                    text: sim.judgeVerdict !== "" ? sim.judgeVerdict : "PENALTIES"
                    color: sim.judgeVerdict !== "" && sim.judgeVerdict !== "FINISHED" ? window.brakeColor : window.muted
                }
                Text {
                    objectName: "penaltyMetric"; anchors.right: parent.right; color: window.ink; font.pixelSize: 11
                    text: sim.penaltySeconds > 0 ? "+" + sim.penaltySeconds.toFixed(0) + " s" : "none"
                }
                Text {
                    objectName: "penaltyDetail"; anchors.right: parent.right; visible: sim.penaltySeconds > 0
                    text: sim.conesHit + " cones · " + sim.offCourseCount + " off"; color: window.muted; font.pixelSize: 9
                }
            }
        }
    }

    // The car's telemetry, opened from the dock: the cards for what this car models, in a column beneath the map that
    // scrolls when the window is short.
    Item {
        id: insightsPanel
        objectName: "insightsPanel"
        visible: window.insightsOpen
        anchors.right: parent.right
        anchors.rightMargin: window.margin
        y: mapCard.y + mapCard.height + 12
        width: window.sideWidth
        height: dock.y - 12 - y
        Flickable {
            anchors.fill: parent
            clip: true
            contentWidth: width
            contentHeight: insightsColumn.implicitHeight
            boundsBehavior: Flickable.StopAtBounds
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded; width: 6 }
            Column {
                id: insightsColumn
                width: parent.width
                spacing: 12
                Card {
                    objectName: "forgivenessTelemetry"
                    visible: sim.forgivenessActive
                    width: parent.width
                    height: assistanceTelemetryText.implicitHeight + 32
                    Text {
                        id: assistanceTelemetryText
                        x: 16; y: 16; width: parent.width - 32
                        wrapMode: Text.WordWrap; color: window.muted; font.pixelSize: 12
                        text: "Forgiveness overrides normal tire behaviour. Switch it off to compare the kart's tire limits. The prediction still shows the unassisted planner."
                    }
                }
                Card {
                    objectName: "insightsEmpty"
                    visible: !sim.tireSlipModeled && !sim.envelopeModeled && !sim.optimalLoaded && !sim.sensorsFitted && !sim.perceptionFitted
                    width: parent.width
                    height: emptyText.implicitHeight + 32
                    Text {
                        id: emptyText
                        x: 16; y: 16; width: parent.width - 32
                        wrapMode: Text.WordWrap
                        color: window.muted; font.pixelSize: 12; lineHeight: 1.15
                        text: "The kinematic car has no tires to show. Choose Dynamic or 4 wheels, or fit instruments and cones, in Settings."
                    }
                }
                // Tire slip for a model with tires: each axle against its peak, and the balance with its definition.
                Card {
                    id: slipCard
                    objectName: "slipCard"
                    visible: sim.tireSlipModeled && !sim.forgivenessActive
                    width: parent.width
                    height: slipColumn.implicitHeight + 28
                    Column {
                        id: slipColumn
                        x: 16; y: 14
                        width: parent.width - 32
                        spacing: 8
                        Item {
                            width: parent.width; height: 16
                            SmallLabel { text: "TIRES"; color: window.ink; anchors.verticalCenter: parent.verticalCenter }
                            SmallLabel {
                                objectName: "balanceLabel"
                                anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                                text: sim.handlingBalance
                                color: text === "UNDERSTEER" ? window.holdColor : text === "OVERSTEER" ? window.brakeColor : text === "NEUTRAL" ? window.ink : window.muted
                            }
                        }
                        // The rear axle's sideslip, the angle between the velocity arrow and the heading line at the car.
                        CardValue { objectName: "sideslipMetric"; label: "REAR SIDESLIP"; value: sim.rearSideslipDegrees.toFixed(1) + "°" }
                        // Air on the four-wheel car: drag against its motion and downforce onto its wheels.
                        CardValue {
                            objectName: "aeroMetric"; visible: sim.aerodynamicsModeled; label: "DRAG / DOWNFORCE"
                            value: (sim.dragNewtons / 1000).toFixed(2) + " / " + (sim.downforceNewtons / 1000).toFixed(2) + " kN"
                        }
                        // What MAP added to geometric steering at this moment: positive steers further into the turn.
                        CardValue {
                            objectName: "steeringCorrectionMetric"; visible: sim.steeringMode === 1; label: "MAP CORRECTION"
                            value: (sim.steeringCorrectionDegrees >= 0 ? "+" : "") + sim.steeringCorrectionDegrees.toFixed(2) + "°"
                        }
                        SlipBar { objectName: "frontSlip"; label: "FRONT"; slip: sim.frontSlipDegrees; peak: sim.frontPeakSlipDegrees }
                        SlipBar { objectName: "rearSlip"; label: "REAR"; slip: sim.rearSlipDegrees; peak: sim.rearPeakSlipDegrees }
                        // The four wheels around the car seen from above, each where it sits on the car.
                        Item {
                            objectName: "wheelGrid"
                            visible: sim.wheelsModeled
                            width: parent.width
                            height: 2 * 50 + 8
                            readonly property real tileWidth: (width - 52) / 2
                            WheelTile { index: 0; x: 0; y: 0; width: parent.tileWidth }
                            WheelTile { index: 1; x: parent.width - width; y: 0; width: parent.tileWidth }
                            WheelTile { index: 2; x: 0; y: 58; width: parent.tileWidth }
                            WheelTile { index: 3; x: parent.width - width; y: 58; width: parent.tileWidth }
                            Shape {
                                anchors.centerIn: parent
                                width: 28; height: 84
                                preferredRendererType: Shape.CurveRenderer
                                ShapePath {
                                    strokeColor: window.muted; strokeWidth: 1.4; fillColor: "#14ffffff"
                                    joinStyle: ShapePath.RoundJoin
                                    PathSvg { path: "M7 4 Q14 -1 21 4 L24 20 L23 64 Q22 82 14 82 Q6 82 5 64 L4 20 Z" }
                                }
                            }
                        }
                        // Each tire's friction circle along the driving plan's horizon (decision 0027): the four-wheel plant
                        // driven by the plan's own commands, and what each wheel is asked of its own circle at every point of it.
                        Column {
                            objectName: "tireMarginPanel"
                            visible: sim.planTireUseAvailable
                            width: parent.width
                            spacing: 4
                            CardValue {
                                objectName: "tireMarginWorst"
                                label: "FRICTION LEFT AHEAD"
                                value: Math.round((1 - sim.planTireWorstUse) * 100) + "% at " + sim.planTireWorstTime.toFixed(1) + " s"
                            }
                            Canvas {
                                id: tireMarginPlot
                                objectName: "tireMarginPlot"
                                width: parent.width
                                height: window.compact ? 34 : 46
                                onPaint: {
                                    var ctx = getContext("2d")
                                    ctx.reset()
                                    var rows = sim.planTireUse
                                    if (!rows || rows.length < 2) return
                                    var colours = ["#8fb3c4", "#89eab5", "#e6cf78", "#c79bf0"]
                                    var span = Math.max(0.001, rows[rows.length - 1][0] - rows[0][0])
                                    var h = height - 2
                                    // The tires' own limit, and half of it.
                                    ctx.strokeStyle = "#23292e"; ctx.lineWidth = 1
                                    ctx.beginPath(); ctx.moveTo(0, 1 + h * 0.5); ctx.lineTo(width, 1 + h * 0.5); ctx.stroke()
                                    ctx.strokeStyle = "#5a1f22"
                                    ctx.beginPath(); ctx.moveTo(0, 1); ctx.lineTo(width, 1); ctx.stroke()
                                    for (var wheel = 0; wheel < 4; ++wheel) {
                                        ctx.beginPath()
                                        for (var i = 0; i < rows.length; ++i) {
                                            var x = ((rows[i][0] - rows[0][0]) / span) * (width - 2) + 1
                                            var y = 1 + h * (1 - Math.min(1, rows[i][wheel + 1]))
                                            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
                                        }
                                        ctx.strokeStyle = colours[wheel]; ctx.lineWidth = 1.5; ctx.stroke()
                                    }
                                }
                            }
                            RowLayout {
                                width: parent.width
                                spacing: 8
                                Repeater {
                                    model: ["FL", "FR", "RL", "RR"]
                                    Row {
                                        required property int index
                                        required property string modelData
                                        spacing: 3
                                        Rectangle { width: 7; height: 3; y: 5; color: ["#8fb3c4", "#89eab5", "#e6cf78", "#c79bf0"][index] }
                                        Text { text: modelData; color: window.muted; font.pixelSize: 9 }
                                    }
                                }
                                Item { Layout.fillWidth: true }
                                Text { text: "0 - " + sim.predictionSeconds.toFixed(1) + " s"; color: window.muted; font.pixelSize: 9 }
                            }
                        }
                        Text {
                            objectName: "balanceDefinition"
                            width: parent.width
                            wrapMode: Text.WordWrap
                            color: window.muted; font.pixelSize: 10; lineHeight: 1.15
                            text: sim.handlingBalance === "NOT CORNERING"
                                  ? "Balance needs a turn: lateral acceleration above " + sim.corneringThreshold.toFixed(0) + " m/s²."
                                  : "Front minus rear slip toward the turn: " + (sim.understeerDegrees >= 0 ? "+" : "") + sim.understeerDegrees.toFixed(2)
                                    + "°. Understeer above +" + sim.balanceDeadbandDegrees.toFixed(2) + "°, oversteer below −" + sim.balanceDeadbandDegrees.toFixed(2) + "°."
                        }
                    }
                }
                // The four-wheel car's G-G diagram (decision 0017): the accelerations its tires sustain at the present speed,
                // derived from the plant, and the car's own point. Left turns are drawn to the left, acceleration upward.
                Card {
                    id: ggCardItem
                    objectName: "ggCard"
                    visible: sim.envelopeModeled && !sim.forgivenessActive
                    width: parent.width
                    height: ggColumn.implicitHeight + 28
                    Column {
                        id: ggColumn
                        x: 16; y: 14
                        width: parent.width - 32
                        spacing: 8
                        Item {
                            width: parent.width; height: 16
                            SmallLabel { text: "G-G ENVELOPE"; color: window.ink; anchors.verticalCenter: parent.verticalCenter }
                            Text {
                                anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                                text: sim.speedKmh.toFixed(0) + " km/h"; color: window.muted; font.pixelSize: 10
                            }
                        }
                        Canvas {
                            id: ggCanvas
                            objectName: "ggCanvas"
                            width: parent.width
                            height: window.compact ? 118 : 138
                            property var boundary: sim.envelopeBoundary
                            property real lateral: sim.ggLateral
                            property real longitudinal: sim.ggLongitudinal
                            // Under the envelope plan, the share of the envelope the plan uses (decision 0018); zero otherwise.
                            property real planShare: sim.speedPlanMode === 1 ? sim.envelopeFraction : 0
                            onBoundaryChanged: requestPaint()
                            onPlanShareChanged: requestPaint()
                            onLateralChanged: requestPaint()
                            onLongitudinalChanged: requestPaint()
                            onPaint: {
                                var ctx = getContext("2d")
                                ctx.reset()
                                var cx = width / 2, cy = height / 2
                                // Twelve m/s² to the edge; the circle is one g.
                                var scale = (Math.min(width, height) / 2 - 4) / 12
                                ctx.strokeStyle = "#23292e"; ctx.lineWidth = 1
                                ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(width, cy); ctx.moveTo(cx, 0); ctx.lineTo(cx, height); ctx.stroke()
                                ctx.beginPath(); ctx.arc(cx, cy, 9.80665 * scale, 0, 2 * Math.PI); ctx.stroke()
                                var points = boundary
                                if (points.length > 2) {
                                    ctx.beginPath()
                                    for (var i = 0; i < points.length; ++i) {
                                        var px = cx - points[i].lateral * scale, py = cy - points[i].longitudinal * scale
                                        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py)
                                    }
                                    ctx.closePath()
                                    ctx.fillStyle = "rgba(76, 141, 255, 0.14)"; ctx.fill()
                                    ctx.strokeStyle = "#4c8dff"; ctx.lineWidth = 1.5; ctx.stroke()
                                    if (planShare > 0) {
                                        ctx.beginPath()
                                        for (var k = 0; k < points.length; ++k) {
                                            var sx = cx - planShare * points[k].lateral * scale, sy = cy - planShare * points[k].longitudinal * scale
                                            if (k === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy)
                                        }
                                        ctx.closePath()
                                        ctx.strokeStyle = "#e6cf78"; ctx.lineWidth = 1; ctx.stroke()
                                    }
                                }
                                ctx.fillStyle = "#ffffff"
                                ctx.beginPath(); ctx.arc(cx - lateral * scale, cy - longitudinal * scale, 3.5, 0, 2 * Math.PI); ctx.fill()
                            }
                        }
                        Text {
                            objectName: "ggStatus"
                            width: parent.width; wrapMode: Text.WordWrap
                            color: window.muted; font.pixelSize: 10
                            text: sim.envelopeReady ? "Envelope " + sim.envelopeFingerprint.substring(0, 8) + (sim.replay ? ", derived from the recorded car" : ", derived from this car") +
                                                      (sim.speedPlanMode === 1 ? "; the plan uses the inner " + Math.round(sim.envelopeFraction * 100) + "%" : "")
                                : sim.envelopeFailed ? "No envelope: this car cannot hold the lowest speed straight ahead"
                                : "Deriving the envelope from the car…"
                        }
                    }
                }
                // The theoretical best lap (TrackWayFastPlan Phase 4.3, decision 0034): the optimum's lap and the live lap
                // against it, sector by sector, what each tire has dissipated so far on the optimum's lap against its whole
                // lap, and what a setup step would be worth. Always with the solver, mesh, tolerance and status it is for;
                // an unconverged solve says so.
                Card {
                    id: optimalCard
                    objectName: "optimalCard"
                    visible: sim.optimalLoaded
                    width: parent.width
                    height: optimalColumn.implicitHeight + 28
                    Column {
                        id: optimalColumn
                        x: 16; y: 14
                        width: parent.width - 32
                        spacing: 6
                        Item {
                            width: parent.width; height: 14
                            SmallLabel { objectName: "optimalTitle"; text: "THEORETICAL LAP"; color: window.ink; anchors.verticalCenter: parent.verticalCenter }
                            SmallLabel {
                                objectName: "optimalStatusChip"
                                anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                                text: sim.optimalConverged ? "SOLVED" : "NOT CONVERGED"; font.pixelSize: 9; font.letterSpacing: 1
                                color: sim.optimalConverged ? window.accelerateColor : window.brakeColor
                            }
                        }
                        Row {
                            objectName: "optimalLapRow"
                            spacing: 8
                            Text { id: optimalLapText; objectName: "optimalLapMetric"; text: sim.optimalLapSeconds.toFixed(2) + " s"; color: window.ink; font.pixelSize: 18; font.weight: Font.Light }
                            Text {
                                objectName: "optimalDeltaMetric"
                                anchors.baseline: optimalLapText.baseline
                                visible: sim.optimalDeltaAvailable
                                text: "Δ " + (sim.optimalDeltaSeconds >= 0 ? "+" : "") + sim.optimalDeltaSeconds.toFixed(2) + " s" + (sim.optimalFromRest ? " from rest" : "")
                                color: sim.optimalDeltaSeconds > 0 ? window.brakeColor : window.accelerateColor; font.pixelSize: 11
                            }
                        }
                        Text {
                            objectName: "optimalSourceText"
                            width: parent.width; wrapMode: Text.WordWrap
                            text: sim.optimalSource + " · " + sim.optimalStatus; color: window.muted; font.pixelSize: 9
                        }
                        Text {
                            objectName: "optimalMismatchText"
                            visible: !sim.optimalMatches
                            width: parent.width; wrapMode: Text.WordWrap
                            text: sim.optimalMismatch; color: window.holdColor; font.pixelSize: 10
                        }
                        // The live lap in progress against the optimum, sector by sector; a sector not yet timed this lap shows
                        // the lap before, dimmed. The bar grows right in red when slower, left in green when faster, full at 25%.
                        Row {
                            objectName: "sectorDeltaRow"
                            visible: sim.optimalMatches
                            width: parent.width
                            spacing: 6
                            Repeater {
                                model: sim.sectorDeltas
                                Column {
                                    required property var modelData
                                    required property int index
                                    objectName: "sectorDelta" + index
                                    width: (optimalColumn.width - 12) / 3
                                    spacing: 3
                                    opacity: modelData.previous ? 0.55 : 1
                                    Text {
                                        width: parent.width
                                        text: parent.modelData.name + " " + (parent.modelData.timed ? (parent.modelData.delta >= 0 ? "+" : "") + parent.modelData.delta.toFixed(2) : "—")
                                        color: !parent.modelData.timed ? window.muted : parent.modelData.delta > 0 ? window.brakeColor : window.accelerateColor
                                        font.pixelSize: 9
                                    }
                                    Rectangle {
                                        width: parent.width; height: 3; radius: 1.5; color: "#23292e"
                                        Rectangle {
                                            readonly property real share: parent.parent.modelData.timed ? Math.min(1, Math.abs(parent.parent.modelData.delta) / (0.25 * parent.parent.modelData.optimal)) : 0
                                            width: parent.width / 2 * share; height: parent.height; radius: 1.5
                                            x: parent.parent.modelData.timed && parent.parent.modelData.delta > 0 ? parent.width / 2 : parent.width / 2 - width
                                            color: parent.parent.modelData.timed && parent.parent.modelData.delta > 0 ? window.brakeColor : window.accelerateColor
                                        }
                                        Rectangle { x: parent.width / 2; y: -2; width: 1; height: 7; color: window.muted }
                                    }
                                }
                            }
                        }
                        // Each tire's energy dissipated on the optimum's lap so far, against the most any tire dissipates in the
                        // lap; the tick is that tire's whole lap. After fastest-lap's tire energy analysis.
                        Row {
                            objectName: "tireEnergyRow"
                            visible: sim.optimalMatches
                            width: parent.width
                            spacing: 6
                            readonly property real most: {
                                var m = 0
                                for (var i = 0; i < sim.ghostTireEnergy.length; ++i) m = Math.max(m, sim.ghostTireEnergy[i].total)
                                return Math.max(m, 1e-9)
                            }
                            Repeater {
                                model: sim.ghostTireEnergy
                                Column {
                                    required property var modelData
                                    required property int index
                                    objectName: "tireEnergy" + index
                                    width: (optimalColumn.width - 18) / 4
                                    spacing: 2
                                    Rectangle {
                                        width: parent.width; height: 20; radius: 3; color: "#23292e"
                                        Rectangle {
                                            anchors.bottom: parent.bottom
                                            width: parent.width; radius: 3
                                            height: parent.height * Math.min(1, parent.parent.modelData.used / parent.parent.parent.most)
                                            color: window.holdColor
                                        }
                                        Rectangle {
                                            y: parent.height * (1 - Math.min(1, parent.parent.modelData.total / parent.parent.parent.most))
                                            width: parent.width; height: 1; color: window.ink
                                        }
                                    }
                                    Text {
                                        width: parent.width; horizontalAlignment: Text.AlignHCenter
                                        text: parent.modelData.name + " " + parent.modelData.used.toFixed(0)
                                        color: window.muted; font.pixelSize: 8
                                    }
                                }
                            }
                        }
                        Text {
                            visible: sim.optimalMatches
                            text: "Tire energy, kJ, on the optimum's lap"; color: window.muted; font.pixelSize: 9
                        }
                        Rule { width: parent.width }
                        // What a setup step would be worth to the optimum, first order, from the solver's sensitivities.
                        Repeater {
                            model: sim.optimalSensitivities
                            Item {
                                required property var modelData
                                required property int index
                                objectName: "sensitivity" + index
                                width: optimalColumn.width; height: 13
                                Text { text: parent.modelData.step; color: window.muted; font.pixelSize: 10; anchors.verticalCenter: parent.verticalCenter }
                                Text {
                                    anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                                    text: (parent.modelData.worth >= 0 ? "+" : "−") + Math.abs(parent.modelData.worth).toFixed(3) + " s"
                                    color: window.ink; font.pixelSize: 10
                                }
                            }
                        }
                    }
                }
                // What the car's own instruments (decision 0029) and its simulated cone perception (decision 0030) make of
                // the car and the course beside what is true. The pose is late because it is sampled at 20 Hz and arrives a
                // dead time later; cones are missed and mis-coloured more often farther away. Nothing here drives the car
                // unless it is told to drive on the cones (decision 0031), when it drives on the path it believes from them.
                Card {
                    objectName: "sensorsCard"
                    visible: sim.sensorsFitted || sim.perceptionFitted
                    width: parent.width
                    height: sensorsColumn.implicitHeight + 28
                    Column {
                        id: sensorsColumn
                        x: 16; y: 14
                        width: parent.width - 32
                        spacing: 6
                        SmallLabel { text: "SENSORS"; color: window.ink }
                        Column {
                            objectName: "measuredCard"
                            visible: sim.sensorsFitted
                            width: parent.width
                            spacing: 4
                            CardValue {
                                objectName: "measuredPose"
                                label: "POSE"
                                value: sim.measuredAvailable ? Math.round(sim.measuredAgeMs) + " ms old · " +
                                                               sim.measuredPositionError.toFixed(2) + " m out" : "nothing yet"
                            }
                            CardValue {
                                objectName: "measuredSpeed"
                                label: "SPEED · YAW"
                                value: sim.measuredAvailable ? (sim.measuredSpeedErrorKmh >= 0 ? "+" : "") + sim.measuredSpeedErrorKmh.toFixed(1) +
                                                               " km/h · " + (sim.measuredYawRateErrorDegrees >= 0 ? "+" : "") +
                                                               sim.measuredYawRateErrorDegrees.toFixed(2) + "°/s" : "-"
                            }
                        }
                        Column {
                            objectName: "perceptionReadout"
                            visible: sim.perceptionFitted
                            width: parent.width
                            spacing: 4
                            CardValue {
                                objectName: "perceivedCounts"
                                label: "SEEN / MISSED / WRONG"
                                value: sim.perceptionFrameAvailable ? sim.perceivedDetections + " / " + sim.perceivedMissed + " / " +
                                                                      sim.perceivedWrongColour : "nothing yet"
                            }
                            // On cones the believed path's line below states its own frame's age instead.
                            CardValue {
                                objectName: "perceptionAge"
                                visible: !sim.drivingOnCones
                                label: "CONE FRAME AGE"
                                value: sim.perceptionFrameAvailable ? Math.round(sim.perceptionFrameAgeMs) + " ms" : "-"
                            }
                            CardValue {
                                objectName: "believedPathReadout"
                                visible: sim.drivingOnCones
                                label: "BELIEF"
                                value: sim.believedPathAvailable ? sim.believedPathAheadM.toFixed(0) + " m · " +
                                                                   Math.round(sim.believedPathAgeMs) + " ms · " +
                                                                   sim.beliefErrorM.toFixed(2) + " m off"
                                                                 : "no path yet"
                            }
                        }
                        Text {
                            objectName: "sensorsCaption"
                            visible: sim.sensorsFitted || sim.perceptionFitted
                            width: parent.width; wrapMode: Text.WordWrap
                            text: (sim.perceptionFitted ? "Diamonds: simulated detections. Grey rings: missed. Red crosses: wrong colour. " : "") +
                                  (sim.drivingOnCones ? "Violet: the path the car believes and where it believes it is. It drives on these alone."
                                                      : "The car drives on its true state and the known track, not on these.")
                            color: window.muted; font.pixelSize: 10; lineHeight: 1.1
                        }
                    }
                }
            }
        }
    }

    // After choosing a car: what the parked view is for.
    Rectangle {
        objectName: "setupCaption"
        visible: showroom.screen === "setup" && !sim.replay
        anchors.horizontalCenter: parent.horizontalCenter
        y: window.margin
        width: setupCaption.implicitWidth + 36
        height: 36
        radius: 18
        color: window.raised
        border.color: window.hairline
        Text {
            id: setupCaption
            anchors.centerIn: parent
            text: "Track setup  ·  " + showroom.currentName + "  ·  choose your line, then Start"
            color: window.ink; font.pixelSize: 12
        }
    }

    // Why the car is at this speed (the product's question), in a sentence; the decision around a stated blockage when there
    // is one; and, on request, the prediction, the constraint that sets the speed and the speed ahead.
    Card {
        id: whyCard
        objectName: "whyCard"
        visible: !sim.humanDriving
        x: window.margin
        y: dock.y - 12 - height
        width: window.compact ? 336 : 380
        height: whyColumn.implicitHeight + (window.compact ? 26 : 30)
        Column {
            id: whyColumn
            x: window.compact ? 16 : 18
            y: window.compact ? 13 : 15
            width: parent.width - 2 * x
            spacing: window.compact ? 7 : 9
            Item {
                width: parent.width
                height: 24
                SmallLabel { text: "WHY THIS SPEED"; color: window.ink; anchors.verticalCenter: parent.verticalCenter }
                Row {
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 4
                    Text {
                        text: "PLAN " + sim.revision.toString().padStart(3, "0")
                        color: window.faint; font.pixelSize: 10; anchors.verticalCenter: parent.verticalCenter
                    }
                    IconButton {
                        objectName: "whyExpandButton"
                        glyph: window.whyExpanded ? "chevronDown" : "chevronUp"
                        tip: window.whyExpanded ? "Less" : "More"
                        implicitWidth: 26
                        iconSize: 16
                        onClicked: window.whyExpanded = !window.whyExpanded
                    }
                }
            }
            Text {
                objectName: "brakingReason"
                // Around a blockage the decision below gives the same sentence.
                visible: !(sim.obstructionCount > 0 && sim.reason === sim.decisionReason)
                width: parent.width
                text: sim.reason
                color: "#e3e8ea"; font.pixelSize: window.compact ? 13 : 14; wrapMode: Text.WordWrap; lineHeight: 1.12
            }
            // Around stated blockages: the action chosen, where a pass crosses, what was rejected, and at what cost.
            Column {
                objectName: "decisionPanel"
                visible: sim.obstructionCount > 0
                width: parent.width
                spacing: window.compact ? 3 : 5
                Row {
                    spacing: 8
                    Rectangle { width: 7; height: 7; radius: 3.5; anchors.verticalCenter: parent.verticalCenter; color: decisionLabel.color }
                    SmallLabel { id: decisionLabel; objectName: "decisionLabel"; text: sim.decisionTitle; color: window.decisionColor }
                    // Under the lattice planner: where a pass crosses the blockage, and how many actions were rejected.
                    Text {
                        objectName: "decisionCount"
                        visible: sim.decisionAvailable && sim.localPlanner === 0 && sim.evaluatedOptions > 1
                        anchors.verticalCenter: parent.verticalCenter
                        text: (sim.chosenAction === "pass left" || sim.chosenAction === "pass right" ? Math.abs(sim.selectedOffset).toFixed(1) + " m across  ·  " : "") +
                              sim.rejectedOptions + " of " + sim.evaluatedOptions + " rejected"
                        color: window.muted; font.pixelSize: 10
                    }
                }
                Text {
                    objectName: "decisionReason"
                    width: parent.width
                    text: sim.decisionAvailable ? sim.decisionReason : "No recorded decision"
                    color: "#e3e8ea"; font.pixelSize: window.compact ? 13 : 14; wrapMode: Text.WordWrap
                    lineHeight: 1.1
                }
                Text {
                    objectName: "decisionOptions"
                    visible: sim.decisionAvailable && sim.localPlanner === 1 && sim.evaluatedOptions > 1
                    text: sim.rejectedOptions + " of " + sim.evaluatedOptions + " lines rejected  ·  chose " +
                          (Math.abs(sim.selectedOffset) < 0.05 ? "the reference line"
                                                               : sim.selectedOffset.toFixed(1) + " m " + (sim.selectedOffset > 0 ? "left" : "right"))
                    color: window.muted; font.pixelSize: 10; wrapMode: Text.WordWrap; width: parent.width
                }
                // The chosen lattice path's cost, term by term, as the planner weighed it: one line until asked for more.
                Text {
                    objectName: "decisionCost"
                    visible: sim.decisionCost !== ""
                    text: sim.decisionCost
                    maximumLineCount: window.whyExpanded ? 3 : 1
                    elide: Text.ElideRight
                    color: window.muted; font.pixelSize: 10; wrapMode: Text.WordWrap; width: parent.width
                }
                Text {
                    objectName: "decisionProvenance"
                    visible: sim.replay
                    text: sim.decisionAvailable ? "Recorded decision, as the car made it" : "Recorded scenario; the choice itself was not recorded"
                    color: window.muted; font.pixelSize: 10; wrapMode: Text.WordWrap; width: parent.width
                }
            }
            Column {
                objectName: "whyDetails"
                visible: window.whyExpanded
                width: parent.width
                spacing: window.compact ? 6 : 8
                Rule { width: parent.width }
                Row {
                    spacing: 10
                    SmallLabel { objectName: "predictionLabel"; text: !sim.replay ? "LOCAL PREDICTION" : sim.predictionAvailable ? "RECORDED PREDICTION" : "RECORDED STATE"; font.pixelSize: 9; anchors.verticalCenter: parent.verticalCenter }
                    Text {
                        objectName: "predictionHorizon"
                        text: sim.predictionAvailable ? sim.predictionSeconds.toFixed(1) + " s ahead  ·  " + sim.predictionDistance.toFixed(0) + " m" : sim.predictionStatus
                        color: window.ink; font.pixelSize: 12
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
                Text {
                    objectName: "predictionValidity"
                    width: parent.width
                    text: window.showReference ? "Reference speed plan shown on map" : sim.predictionStatus
                    color: window.showReference || sim.predictionWithinEnvelope ? window.muted : window.brakeColor
                    font.pixelSize: 10; wrapMode: Text.WordWrap
                }
                ValueRow { width: parent.width; label: "Constraint speed"; value: sim.cornerSpeedKmh.toFixed(0) + " km/h" }
                ValueRow { width: parent.width; label: "Constraint station"; value: sim.constraintStation.toFixed(0) + " m" }
                ValueRow { width: parent.width; label: "Braking bound"; value: sim.brakeBound.toFixed(2) + " m/s²" }
                ValueRow { width: parent.width; label: window.showReference ? "Reference acceleration" : "Predicted acceleration"; value: sim.plannedAcceleration.toFixed(2) + " m/s²" }
                RowLayout {
                    width: parent.width
                    SmallLabel { objectName: "speedPlotLabel"; text: window.showReference ? "REFERENCE SPEED" : sim.replay ? "RECORDED PREDICTION" : "PREDICTED SPEED"; font.pixelSize: 9; Layout.fillWidth: true }
                    Text { text: window.plotMaximumSpeed.toFixed(0) + " km/h"; color: window.faint; font.pixelSize: 10 }
                }
                Canvas {
                    id: speedPlot
                    objectName: "speedPlanPlot"
                    width: parent.width
                    height: window.compact ? 40 : 56
                    onPaint: {
                        var ctx = getContext("2d")
                        ctx.reset()
                        var reference = window.showReference
                        var points = reference ? window.cachedPoints : window.cachedPrediction
                        if (!points || points.length < 2) return
                        var h=height-8
                        ctx.strokeStyle="#20262b"; ctx.lineWidth=1
                        for(var row=0; row<3; ++row) { ctx.beginPath(); ctx.moveTo(0, 4+h*row/2); ctx.lineTo(width,4+h*row/2); ctx.stroke() }
                        function px(point) { return (reference ? point.s/window.trackLength : point.time/Math.max(0.001, sim.predictionSeconds))*(width-2)+1 }
                        function py(point) { return 4+h*(1-point.speed*3.6/window.plotMaximumSpeed) }
                        ctx.beginPath(); ctx.moveTo(1,height)
                        for(var i=0;i<points.length;++i)ctx.lineTo(px(points[i]),py(points[i]))
                        if (reference) ctx.lineTo(width-1,py(points[0]))
                        ctx.lineTo(width-1,height);ctx.closePath();ctx.fillStyle="rgba(255, 255, 255, 0.05)";ctx.fill()
                        ctx.lineWidth=2
                        for(var j=0;j<points.length-(reference ? 0 : 1);++j) {
                            var next=points[(j+1)%points.length]
                            ctx.beginPath();ctx.moveTo(px(points[j]),py(points[j]));ctx.lineTo(j===points.length-1 ? width-1 : px(next),py(next));
                            ctx.strokeStyle=window.trajectoryColor(points[j].acceleration);ctx.stroke()
                        }
                        var cursor=reference ? Math.max(0,Math.min(1,sim.progress))*(width-2)+1 : 1
                        ctx.strokeStyle="#dce8e2";ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(cursor,0);ctx.lineTo(cursor,height);ctx.stroke()
                    }
                }
            }
        }
    }

    // The controller starts and pauses the drive, as a game's does, and never a run hidden behind the car selection.
    Connections {
        target: sim
        function onMenuPressed() { if (showroom.screen !== "showroom") sim.toggleRunning() }
        function onAcceleratorAtLine() { if (showroom.screen !== "showroom" && !sim.running) sim.toggleRunning() }
    }

    // A person's controls as the car takes them (decision 0037): the brake filling from the centre leftward, the steering
    // as a dot on its travel, the accelerator filling rightward.
    Rectangle {
        id: driverInputs
        objectName: "driverInputs"
        visible: sim.humanDriving
        anchors.horizontalCenter: parent.horizontalCenter
        y: statusToast.y - 10 - height
        width: inputsRow.implicitWidth + 36
        height: window.compact ? 30 : 34
        radius: height / 2
        color: window.raised
        border.color: window.hairline
        Row {
            id: inputsRow
            anchors.centerIn: parent
            spacing: 12
            Text { text: "LT"; color: window.muted; font.pixelSize: 10; font.weight: Font.DemiBold; anchors.verticalCenter: parent.verticalCenter }
            Rectangle {
                width: 64; height: 6; radius: 3; color: "#2a3036"
                anchors.verticalCenter: parent.verticalCenter
                Rectangle {
                    objectName: "brakeInputBar"
                    anchors.right: parent.right
                    width: parent.width * sim.driverBrake; height: parent.height; radius: 3
                    color: window.brakeColor
                }
            }
            Rectangle {
                id: steeringTrack
                width: 84; height: 6; radius: 3; color: "#2a3036"
                anchors.verticalCenter: parent.verticalCenter
                Rectangle { x: parent.width / 2; width: 1; height: parent.height; color: window.faint }
                Rectangle {
                    objectName: "steeringInputDot"
                    width: 12; height: 12; radius: 6
                    anchors.verticalCenter: parent.verticalCenter
                    // Steering is positive to the left, so the dot moves the way the stick does.
                    x: (parent.width - width) / 2 - sim.driverSteering * (parent.width - width) / 2
                    color: window.ink
                }
            }
            Rectangle {
                width: 64; height: 6; radius: 3; color: "#2a3036"
                anchors.verticalCenter: parent.verticalCenter
                Rectangle {
                    objectName: "throttleInputBar"
                    width: parent.width * sim.driverThrottle; height: parent.height; radius: 3
                    color: window.accelerateColor
                }
            }
            Text { text: "RT"; color: window.muted; font.pixelSize: 10; font.weight: Font.DemiBold; anchors.verticalCenter: parent.verticalCenter }
        }
    }

    // What the car is doing, in one line, beneath it.
    Rectangle {
        id: statusToast
        objectName: "statusToast"
        anchors.horizontalCenter: parent.horizontalCenter
        y: dock.y - 14 - height
        width: toastRow.implicitWidth + 40
        height: window.compact ? 38 : 44
        radius: height / 2
        color: window.raised
        border.color: window.hairline
        Row {
            id: toastRow
            anchors.centerIn: parent
            spacing: 10
            Rectangle { width: 8; height: 8; radius: 4; color: window.toastColor; anchors.verticalCenter: parent.verticalCenter }
            Text {
                objectName: "toastText"
                text: window.toastText
                color: window.ink
                font.pixelSize: window.compact ? 13 : 14
                font.weight: Font.Medium
                anchors.verticalCenter: parent.verticalCenter
            }
        }
    }

    // The dock: run, reset and the run's clock at the left; the replay scrubber in the middle when replaying; the camera,
    // the layers, the telemetry, the car, recordings and settings at the right.
    Rectangle {
        id: dock
        objectName: "dock"
        x: window.margin
        width: parent.width - 2 * window.margin
        height: window.compact ? 58 : 66
        y: parent.height - window.margin - height
        radius: 20
        color: window.glass
        border.color: window.hairline
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10
            anchors.rightMargin: 10
            spacing: window.compact ? 6 : 8
            ActionButton {
                objectName: "runButton"
                text: sim.running ? "Pause" : (sim.replay ? "Play" : "Start")
                glyph: sim.running ? "pause" : "play"
                primary: true
                implicitWidth: window.compact ? 104 : 116
                implicitHeight: window.compact ? 40 : 44
                onClicked: { showroom.startDriving(); sim.toggleRunning() }
            }
            IconButton { objectName: "resetButton"; glyph: "reset"; tip: sim.replay ? "Restart" : "Reset"; onClicked: sim.reset() }
            ActionButton {
                objectName: "exitReplayButton"
                visible: sim.replay
                text: "Exit replay"; glyph: "exit"
                implicitWidth: 118; implicitHeight: window.compact ? 36 : 40
                onClicked: sim.exitReplay()
            }
            Rectangle { Layout.preferredWidth: 1; Layout.preferredHeight: 26; color: window.hairline; Layout.leftMargin: 6; Layout.rightMargin: 6 }
            // Live: the run's clock and counters, and what the simulation says of itself.
            Row {
                objectName: "runMetrics"
                visible: !sim.replay
                spacing: window.compact ? 18 : 26
                Metric { label: "RUN TIME"; value: window.timeText(sim.simulationTime) }
                Metric { label: "LAP"; value: sim.lap.toString().padStart(2, "0") }
                Metric { label: "TRACKING"; value: Math.abs(sim.crossTrackError).toFixed(2) + " m" }
            }
            // Long statuses elide rather than run under the controls to the right.
            Text {
                objectName: "simulationStatus"
                visible: !sim.replay
                Layout.fillWidth: true
                Layout.leftMargin: 10
                text: sim.status
                color: window.muted; font.pixelSize: 11
                elide: Text.ElideRight
            }
            // Replay scrubber. Dragging seeks recorded samples; it never steps the plant.
            RowLayout {
                id: playheadPanel
                objectName: "playheadPanel"
                visible: sim.replay
                Layout.fillWidth: true
                spacing: 12
                Column {
                    spacing: 2
                    Layout.maximumWidth: 120
                    SmallLabel { text: "REPLAY"; color: window.accent; font.pixelSize: 9 }
                    Text { objectName: "recordingName"; text: sim.recordingName; color: window.ink; font.pixelSize: 11; elide: Text.ElideRight; width: Math.min(118, implicitWidth) }
                }
                Text { objectName: "playheadTimeText"; text: window.timeText(sim.playheadTime); color: window.ink; font.pixelSize: 13; font.weight: Font.Medium }
                Slider {
                    id: playhead
                    objectName: "playhead"
                    Layout.fillWidth: true
                    Layout.minimumWidth: 100
                    height: 32
                    from: 0
                    to: Math.max(0.001, sim.recordingDuration)
                    stepSize: 0
                    onMoved: sim.seek(value)
                    Binding { target: playhead; property: "value"; value: sim.playheadTime; when: !playhead.pressed }
                    background: Rectangle {
                        x: playhead.leftPadding
                        y: playhead.topPadding + playhead.availableHeight/2 - height/2
                        width: playhead.availableWidth
                        height: 4
                        radius: 2
                        color: "#2a3036"
                        Rectangle { width: playhead.visualPosition * parent.width; height: parent.height; radius: 2; color: window.accent }
                        // Accepted parameter changes at their recorded simulation times.
                        Repeater {
                            model: sim.eventTimes
                            Rectangle {
                                required property var modelData
                                x: modelData / playhead.to * parent.width - 1
                                y: -5
                                width: 2
                                height: 14
                                color: window.holdColor
                            }
                        }
                    }
                    handle: Rectangle {
                        x: playhead.leftPadding + playhead.visualPosition*(playhead.availableWidth-width)
                        y: playhead.topPadding + playhead.availableHeight/2-height/2
                        width: 16; height: 16; radius: 8
                        color: "#ffffff"
                        border.width: playhead.activeFocus ? 2 : 0
                        border.color: window.accent
                    }
                }
                Text { text: window.timeText(sim.recordingDuration); color: window.muted; font.pixelSize: 11 }
                Text {
                    objectName: "replayRevisionText"
                    // Hidden rather than crowding the scrubber when the window is narrow;
                    // the same revision is always shown in the WHY THIS SPEED card.
                    visible: playheadPanel.width > 520
                    text: "PLAN " + sim.revision.toString().padStart(3, "0") + "   ·   " + sim.appliedEvents + "/" + sim.recordedEvents + " CHANGES"
                    color: window.muted
                    font.pixelSize: 9
                    font.letterSpacing: 1
                }
            }
            Segmented {
                Layout.preferredWidth: window.compact ? 206 : 232
                ActionButton {
                    objectName: "followViewButton"; text: "Follow"; segment: true; selected: sim.cameraMode === 0
                    Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                    onClicked: sim.setOverview(false)
                }
                ActionButton {
                    objectName: "overviewButton"; text: "Overview"; segment: true; selected: sim.overview
                    Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                    onClicked: sim.setOverview(true)
                }
                ActionButton {
                    objectName: "firstPersonViewButton"; text: "FP"; segment: true; selected: sim.cameraMode === 2
                    Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 32
                    onClicked: sim.setCameraMode(2)
                }
            }
            Rectangle { Layout.preferredWidth: 1; Layout.preferredHeight: 26; color: window.hairline; Layout.leftMargin: 4; Layout.rightMargin: 4 }
            IconButton {
                objectName: "layersButton"; glyph: "layers"; tip: "Layers"; active: window.layersOpen
                onClicked: window.layersOpen = !window.layersOpen
            }
            IconButton {
                objectName: "insightsButton"; glyph: "telemetry"; tip: "Telemetry"; active: window.insightsOpen
                onClicked: window.insightsOpen = !window.insightsOpen
            }
            IconButton {
                objectName: "showroomButton"; glyph: "car"; tip: "Choose car"
                enabled: !sim.running && !sim.replay
                onClicked: showroom.open()
            }
            IconButton {
                objectName: "openRecordingButton"; glyph: "folder"; tip: "Open recording"
                enabled: !sim.running
                onClicked: recordingDialog.open()
            }
            IconButton {
                objectName: "settingsButton"; glyph: "settings"; tip: "Settings"; active: window.settingsOpen
                onClicked: window.settingsOpen = !window.settingsOpen
            }
        }
    }

    // What the scene's colours mean, with a switch for each layer that can be hidden.
    Rectangle {
        id: layersPanel
        objectName: "layersPanel"
        visible: window.layersOpen
        anchors.right: dock.right
        y: dock.y - 10 - height
        width: window.compact ? 252 : 272
        height: layersColumn.implicitHeight + 30
        radius: 18
        color: window.raised
        border.color: window.hairline
        Column {
            id: layersColumn
            x: 16; y: 15
            width: parent.width - 32
            spacing: 12
            SmallLabel { text: "LAYERS"; color: window.ink }
            Column {
                id: sceneLegend
                objectName: "sceneLegend"
                width: parent.width
                spacing: 10
                LegendRow {
                    objectName: "predictionLegend"
                    label: "Prediction"
                    toggleable: true; shown: window.predictionVisible
                    onToggled: window.togglePrediction()
                }
                LegendRow { label: "Accelerate"; Swatch { color: window.accelerateColor } }
                LegendRow { label: "Hold speed"; Swatch { color: window.holdColor } }
                LegendRow { label: "Brake"; Swatch { color: window.brakeColor } }
                LegendRow {
                    objectName: "motionArrowLegend"
                    visible: sim.tireSlipModeled
                    label: "Motion arrow"
                    toggleable: true; shown: window.motionArrowVisible
                    onToggled: {
                        if (sim.kartMode) window.showKartMotionArrow = !window.showKartMotionArrow
                        else window.showMotionArrow = !window.showMotionArrow
                    }
                    Swatch { color: "#dfe7ec" }
                }
                LegendRow {
                    objectName: "kartGuideLegend"
                    visible: sim.kartMode
                    label: "Poster coaching line"
                    toggleable: true; shown: window.showKartGuide
                    onToggled: window.showKartGuide = !window.showKartGuide
                }
                Text {
                    visible: sim.kartMode
                    width: parent.width; wrapMode: Text.WordWrap
                    text: "Coaching: green = gas, blue = coast, red = brake. Fixed advice from the poster."
                    font.pixelSize: 11; color: window.muted
                }
                // The line drawn beside the car's own once the racing line is solved.
                LegendRow {
                    objectName: "comparisonLegend"
                    visible: !sim.replay && !sim.kartMode && sim.racingLineReady
                    label: sim.lineMode === 1 ? "Centreline" : "Racing line"
                    toggleable: true; shown: window.showReferenceLine
                    onToggled: window.showReferenceLine = !window.showReferenceLine
                    Swatch { color: sim.lineMode === 1 ? "#9aa7ad" : "#7fc4e8" }
                }
                // Each evaluated action's line, drawn beside the chosen one under the lattice planner (decision 0022): going
                // straight on, passing left and passing right.
                LegendRow {
                    objectName: "actionLegend"
                    visible: sim.localPlanner === 0 && sim.evaluatedOptions > 1
                    label: "Straight · left · right"
                    toggleable: true; shown: window.showAlternatives
                    onToggled: window.showAlternatives = !window.showAlternatives
                    Swatch { width: 10; color: sim.actionColorName("straight") }
                    Swatch { width: 10; color: sim.actionColorName("pass left") }
                    Swatch { width: 10; color: sim.actionColorName("pass right") }
                }
                // The offline lattice ahead of the car (decision 0021).
                LegendRow {
                    objectName: "latticeLegend"
                    visible: sim.latticeAvailable
                    label: "Lattice"
                    toggleable: true; shown: window.latticeVisible
                    onToggled: {
                        if (sim.kartMode) window.showKartLattice = !window.showKartLattice
                        else window.showLattice = !window.showLattice
                    }
                    Swatch { height: 1; color: "#6d8795" }
                }
                // The offline optimum's car (decision 0034).
                LegendRow {
                    objectName: "ghostLegend"
                    visible: sim.ghostVisible
                    label: "Offline optimum"
                    toggleable: true; shown: window.showGhost
                    onToggled: window.showGhost = !window.showGhost
                    Swatch { height: 7; color: "#4de4ecf0"; border.color: "#e4ecf0"; border.width: 1 }
                }
                LegendRow {
                    objectName: "blockageLegend"
                    visible: sim.obstructionCount > 0
                    label: "Stated blockage"
                    Swatch { height: 7; color: "#d8574f" }
                }
                // The car mirrored in the road: appearance only.
                LegendRow {
                    objectName: "reflectionLegend"
                    label: "Reflections"
                    toggleable: true; shown: window.showReflections
                    onToggled: window.showReflections = !window.showReflections
                    Swatch {
                        height: 7
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0; color: "#8c98a0" }
                            GradientStop { position: 1; color: "#202329" }
                        }
                    }
                }
            }
        }
    }

    // Settings: every choice of what is simulated and how it is driven, opened from the dock or the driver chip. Most apply
    // as a fresh run and only while paused; each says so where it cannot change.
    Rectangle {
        id: settingsDrawer
        objectName: "settingsDrawer"
        visible: window.settingsOpen
        x: window.margin
        y: window.margin
        width: window.compact ? 344 : 384
        height: dock.y - 12 - y
        radius: 20
        color: window.raised
        border.color: window.hairline
        z: 20
        MouseArea { anchors.fill: parent }
        Item {
            id: settingsHeader
            width: parent.width
            height: 54
            Text { x: 22; anchors.verticalCenter: parent.verticalCenter; text: "Settings"; color: window.ink; font.pixelSize: 17; font.weight: Font.DemiBold }
            IconButton {
                objectName: "settingsCloseButton"
                anchors.right: parent.right; anchors.rightMargin: 10; anchors.verticalCenter: parent.verticalCenter
                glyph: "close"; iconSize: 17; implicitWidth: 34
                onClicked: window.settingsOpen = false
            }
        }
        Flickable {
            id: settingsFlick
            objectName: "settingsFlick"
            y: settingsHeader.height
            width: parent.width
            height: parent.height - y - 10
            clip: true
            contentWidth: width
            contentHeight: settingsColumn.implicitHeight + 16
            boundsBehavior: Flickable.StopAtBounds
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded; width: 6 }
            Column {
                id: settingsColumn
                x: 22
                width: parent.width - 44
                spacing: window.compact ? 12 : 14
                SectionTitle { title: "DRIVER"; note: sim.replay ? "Recorded" : "Change at any time" }
                // Who drives (decision 0037): the autonomous driver, or you with an Xbox controller, on the same plant and
                // judged the same way. Taking the car keeps the plan on the road as a guide; it drives nothing.
                Segmented {
                    width: parent.width
                    ActionButton {
                        objectName: "autonomousDriverButton"; text: "Autonomous"; segment: true; selected: !sim.humanDriving
                        enabled: !sim.replay; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                        onClicked: sim.setHumanDriving(false)
                    }
                    ActionButton {
                        objectName: "humanDriverButton"; text: "You"; segment: true; selected: sim.humanDriving
                        enabled: !sim.replay; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                        onClicked: sim.setHumanDriving(true)
                    }
                }
                Text {
                    objectName: "gamepadNote"
                    visible: sim.humanDriving
                    width: parent.width
                    wrapMode: Text.WordWrap
                    lineHeight: 1.1
                    color: sim.gamepadConnected ? window.muted : window.holdColor
                    font.pixelSize: 11
                    text: (sim.gamepadConnected
                           ? "Xbox controller connected. Right trigger accelerates, left trigger brakes, the left stick steers, and Menu starts and pauses." +
                             (sim.forgivenessEnabled ? " Forgiveness adjusts handling and speed." : " Nothing limits the speed but the car.")
                           : "No controller found. Connect an Xbox controller; it is found within two seconds.") +
                          (sim.tireSlipModeled ? "" : " The kinematic car cannot slide; choose 4 wheels for tires that can.")
                }
                // Gokart mode (decision 0038): the Gokartcentralen Göteborg track and its rental kart, offered to a person who
                // drives. The car keeps its looks at the kart's size; its physics are the kart's until the mode is left.
                SettingRow {
                    objectName: "kartRow"
                    visible: !sim.replay && (sim.humanDriving || sim.kartMode)
                    label: "Gokart"
                    Text {
                        text: sim.kartMode ? "Gokartcentralen Göteborg" : "Off"
                        color: window.ink; font.pixelSize: 12; elide: Text.ElideRight; Layout.fillWidth: true
                    }
                    Toggle {
                        objectName: "kartModeSwitch"
                        checked: sim.kartMode
                        enabled: !sim.running && !sim.replay
                        onToggled: if (!sim.setKartMode(checked)) checked = sim.kartMode
                    }
                }
                Text {
                    objectName: "kartNote"
                    visible: sim.kartMode && !(sim.forgivenessEnabled && sim.forgivenessManual)
                    width: parent.width
                    wrapMode: Text.WordWrap
                    lineHeight: 1.1
                    color: window.muted
                    font.pixelSize: 11
                    text: "400 m, 6 m wide, clockwise, traced from Gokartcentralen's own track map. A Sodi RSX2 rental kart: " +
                          "261 kg with its driver, about 14 hp, about 60 km/h. Your car is drawn at the kart's size."
                }
                SettingRow {
                    objectName: "forgivenessRow"
                    visible: sim.kartMode && sim.humanDriving
                    label: "Forgiveness"
                    labelWidth: 150
                    Item { Layout.fillWidth: true }
                    Toggle {
                        objectName: "forgivenessSwitch"
                        checked: sim.forgivenessEnabled
                        onToggled: if (!sim.setForgiveness(checked, sim.forgivenessLevel)) checked = sim.forgivenessEnabled
                    }
                }
                Column {
                    id: presetsPanel
                    objectName: "forgivenessPresetsPanel"
                    visible: sim.kartMode && sim.humanDriving
                    property int selectedSlot: 0
                    width: parent.width; spacing: 6
                    Text { text: "LOCAL PRESETS · " + (presetsPanel.selectedSlot + 1) + (sim.forgivenessPresets[presetsPanel.selectedSlot] ? " saved" : " empty")
                           color: window.muted; font.pixelSize: 11 }
                    RowLayout {
                        width: parent.width; spacing: 6
                        Repeater {
                            model: 5
                            ActionButton {
                                required property int index
                                objectName: "forgivenessPresetSlot" + index
                                text: (index+1).toString() + (sim.forgivenessPresets[index] ? " •" : "")
                                selected: presetsPanel.selectedSlot === index
                                Layout.fillWidth: true; implicitWidth: 30
                                onClicked: presetsPanel.selectedSlot = index
                            }
                        }
                    }
                    RowLayout {
                        width: parent.width; spacing: 6
                        ActionButton {
                            objectName: "saveForgivenessPreset"; Layout.fillWidth: true; implicitWidth: 40
                            text: sim.forgivenessPresets[presetsPanel.selectedSlot] ? "Replace" : "Save"
                            onClicked: sim.saveForgivenessPreset(presetsPanel.selectedSlot)
                        }
                        ActionButton {
                            objectName: "loadForgivenessPreset"; text: "Load"; Layout.fillWidth: true; implicitWidth: 40
                            enabled: sim.forgivenessPresets[presetsPanel.selectedSlot]
                            onClicked: sim.loadForgivenessPreset(presetsPanel.selectedSlot)
                        }
                        ActionButton {
                            objectName: "deleteForgivenessPreset"; text: "Delete"; Layout.fillWidth: true; implicitWidth: 40
                            enabled: sim.forgivenessPresets[presetsPanel.selectedSlot]
                            onClicked: sim.deleteForgivenessPreset(presetsPanel.selectedSlot)
                        }
                    }
                    Text {
                        width: parent.width; wrapMode: Text.WordWrap; color: window.muted; font.pixelSize: 10
                        text: sim.presetMessage || "Save or Load remembers this setup for your next Gokart session."
                    }
                }
                Column {
                    objectName: "forgivenessPanel"
                    visible: sim.kartMode && sim.humanDriving && sim.forgivenessEnabled
                    width: parent.width; spacing: 8
                    Segmented {
                        width: parent.width
                        ActionButton {
                            objectName: "forgivenessOverallButton"; text: "Overall"; segment: true; selected: !sim.forgivenessManual
                            Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: window.forgivenessMode(false)
                        }
                        ActionButton {
                            objectName: "forgivenessManualButton"; text: "Manual"; segment: true; selected: sim.forgivenessManual
                            Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: window.forgivenessMode(true)
                        }
                    }
                    RowLayout {
                        visible: !sim.forgivenessManual
                        width: parent.width
                        Text { text: "Realistic  1"; color: window.muted; font.pixelSize: 11; Layout.fillWidth: true }
                        Text { objectName: "forgivenessValue"; text: sim.forgivenessLevel.toFixed(0); color: window.ink; font.pixelSize: 18; font.weight: Font.DemiBold }
                        Text { text: "100  Arcade"; color: window.muted; font.pixelSize: 11; Layout.fillWidth: true; horizontalAlignment: Text.AlignRight }
                    }
                    Slider {
                        id: forgivenessSlider
                        visible: !sim.forgivenessManual
                        objectName: "forgivenessSlider"
                        width: parent.width; height: 26
                        from: 1; to: 100; stepSize: 1; snapMode: Slider.SnapAlways
                        value: sim.forgivenessLevel
                        onMoved: sim.setForgiveness(true, value)
                        background: Rectangle {
                            x: forgivenessSlider.leftPadding
                            y: forgivenessSlider.topPadding + forgivenessSlider.availableHeight/2 - height/2
                            width: forgivenessSlider.availableWidth; height: 4; radius: 2; color: "#2a3036"
                            Rectangle { width: forgivenessSlider.visualPosition * parent.width; height: parent.height; radius: 2; color: window.accent }
                        }
                        handle: Rectangle {
                            x: forgivenessSlider.leftPadding + forgivenessSlider.visualPosition*(forgivenessSlider.availableWidth-width)
                            y: forgivenessSlider.topPadding + forgivenessSlider.availableHeight/2-height/2
                            width: 16; height: 16; radius: 8; color: window.ink
                        }
                    }
                    Column {
                        objectName: "manualForgivenessControls"
                        visible: sim.forgivenessManual
                        width: parent.width; spacing: 6
                        Repeater {
                            model: [{key: "acceleration", label: "Acceleration"}, {key: "braking", label: "Braking"},
                                    {key: "steering", label: "Joystick steering"}, {key: "grip", label: "Grip assistance"},
                                    {key: "speed", label: "Top speed"}]
                            delegate: Column {
                                id: manualRow
                                required property var modelData
                                width: parent.width; spacing: 2
                                RowLayout {
                                    width: parent.width
                                    Text { text: manualRow.modelData.label; color: window.ink; font.pixelSize: 12; Layout.fillWidth: true }
                                    Text {
                                        text: sim.forgivenessTuning[manualRow.modelData.key].toFixed(0) +
                                              (manualRow.modelData.key === "speed" ? "  ·  " + (10 + 50*(sim.forgivenessTuning.speed-1)/99).toFixed(0) + " km/h" : "")
                                        color: window.ink; font.pixelSize: 12
                                    }
                                }
                                Slider {
                                    id: manualSlider
                                    objectName: "manual" + manualRow.modelData.key + "Slider"
                                    width: parent.width; height: 24
                                    from: 1; to: 100; stepSize: 1; snapMode: Slider.SnapAlways
                                    value: sim.forgivenessTuning[manualRow.modelData.key]
                                    onMoved: window.tuneForgiveness(manualRow.modelData.key, value)
                                    background: Rectangle {
                                        x: manualSlider.leftPadding; y: manualSlider.topPadding + manualSlider.availableHeight/2 - 2
                                        width: manualSlider.availableWidth; height: 4; radius: 2; color: "#2a3036"
                                        Rectangle { width: manualSlider.visualPosition * parent.width; height: 4; radius: 2; color: window.accent }
                                    }
                                    handle: Rectangle {
                                        x: manualSlider.leftPadding + manualSlider.visualPosition*(manualSlider.availableWidth-width)
                                        y: manualSlider.topPadding + manualSlider.availableHeight/2-height/2
                                        width: 16; height: 16; radius: 8; color: window.ink
                                    }
                                }
                            }
                        }
                        Text {
                            width: parent.width; wrapMode: Text.WordWrap; color: window.muted; font.pixelSize: 11
                            text: "1 = less, 100 = more. Steering changes stick sensitivity and response. Grip 100 prevents sliding; 1 keeps normal tire grip. Each control is independent. Switch Forgiveness off for the original kart."
                        }
                    }
                    Text {
                        visible: !sim.forgivenessManual
                        width: parent.width; wrapMode: Text.WordWrap; color: window.muted; font.pixelSize: 11
                        text: "100: planted arcade handling, strong brakes, 35 km/h. 50: still very easy. 1: normal kart physics. Start high and work down. This overrides normal handling; switch it off for the original kart and setup controls."
                    }
                }
                SettingRow {
                    visible: sim.kartMode
                    label: "Coaching line"
                    labelWidth: 150
                    Item { Layout.fillWidth: true }
                    Toggle {
                        objectName: "kartGuideSwitch"
                        checked: window.showKartGuide
                        onToggled: window.showKartGuide = checked
                    }
                }
                SettingRow {
                    visible: sim.kartMode
                    label: "Prediction line"
                    labelWidth: 150
                    Item { Layout.fillWidth: true }
                    Toggle {
                        objectName: "kartPredictionSwitch"
                        checked: window.showKartPrediction
                        onToggled: window.showKartPrediction = checked
                    }
                }
                SettingRow {
                    visible: sim.kartMode
                    label: "Motion arrow"
                    labelWidth: 150
                    Item { Layout.fillWidth: true }
                    Toggle {
                        objectName: "kartMotionArrowSwitch"
                        checked: window.showKartMotionArrow
                        onToggled: window.showKartMotionArrow = checked
                    }
                }
                Row {
                    visible: sim.kartMode
                    spacing: 7
                    Swatch { color: window.accelerateColor; anchors.verticalCenter: parent.verticalCenter }
                    Text { text: "Gas"; color: window.muted; font.pixelSize: 11 }
                    Swatch { color: window.coastColor; anchors.verticalCenter: parent.verticalCenter }
                    Text { text: "Coast"; color: window.muted; font.pixelSize: 11 }
                    Swatch { color: window.brakeColor; anchors.verticalCenter: parent.verticalCenter }
                    Text { text: "Brake"; color: window.muted; font.pixelSize: 11 }
                }
                Text {
                    objectName: "kartGuideNote"
                    visible: sim.kartMode
                    width: parent.width; wrapMode: Text.WordWrap
                    text: "Full-lap coaching from the poster, painted on the road. Fixed advice; it does not adjust to your speed. Switch either line on or off while driving."
                    color: window.muted; font.pixelSize: 11
                }
                Rule { width: parent.width }
                Column {
                    objectName: "raceSoundPanel"
                    visible: sim.humanDriving && !sim.replay
                    width: parent.width; spacing: 10
                    SectionTitle { title: "SOUND"; note: "Saved on this computer" }
                    SettingRow {
                        label: "Driving sounds"; labelWidth: 150
                        Item { Layout.fillWidth: true }
                        Toggle { objectName: "raceSoundSwitch"; checked: raceAudio.enabled; onToggled: raceAudio.enabled = checked }
                    }
                    RowLayout {
                        width: parent.width
                        Text { text: "Volume"; color: window.ink; font.pixelSize: 12; Layout.fillWidth: true }
                        Text { text: raceAudio.volume + "%"; color: window.ink; font.pixelSize: 12 }
                    }
                    Slider {
                        id: raceVolume; objectName: "raceVolumeSlider"
                        width: parent.width; height: 26; enabled: raceAudio.enabled
                        from: 0; to: 100; stepSize: 1; value: raceAudio.volume
                        onMoved: raceAudio.volume = Math.round(value)
                        background: Rectangle {
                            x: raceVolume.leftPadding; y: raceVolume.topPadding + raceVolume.availableHeight/2 - height/2
                            width: raceVolume.availableWidth; height: 4; radius: 2; color: "#2a3036"
                            Rectangle { width: raceVolume.visualPosition * parent.width; height: parent.height; radius: 2; color: window.accent }
                        }
                        handle: Rectangle {
                            x: raceVolume.leftPadding + raceVolume.visualPosition*(raceVolume.availableWidth-width)
                            y: raceVolume.topPadding + raceVolume.availableHeight/2-height/2
                            width: 16; height: 16; radius: 8; color: window.ink
                        }
                    }
                    SettingRow {
                        label: "Engine & tires"; labelWidth: 150; enabled: raceAudio.enabled
                        Item { Layout.fillWidth: true }
                        Toggle { objectName: "raceEngineSwitch"; checked: raceAudio.engine; onToggled: raceAudio.engine = checked }
                    }
                    SettingRow {
                        label: "Lap & menu sounds"; labelWidth: 150; enabled: raceAudio.enabled
                        Item { Layout.fillWidth: true }
                        Toggle { objectName: "raceCuesSwitch"; checked: raceAudio.cues; onToggled: raceAudio.cues = checked }
                    }
                    ActionButton {
                        objectName: "previewLapSound"; width: parent.width; text: "Preview lap chime"
                        enabled: raceAudio.enabled && raceAudio.cues && raceAudio.volume > 0
                        onClicked: raceAudio.previewLap()
                    }
                    Text {
                        width: parent.width; wrapMode: Text.WordWrap; color: window.muted; font.pixelSize: 11
                        text: raceAudio.status + ". V10 engine and tire samples from Speed Dreams. Lift off for a softer exhaust note; braking brings downshift blips. Gears affect sound only. A pling marks a full lap; a rising chime marks a new session best."
                    }
                    Rule { width: parent.width }
                }
                SectionTitle {
                    title: "VEHICLE MODEL"
                    note: sim.replay ? "Recorded" : sim.running ? "Pause to change" : "Each change starts a fresh run"
                }
                // The plant is part of what is being simulated (decision 0005): the kinematic bicycle, the dynamic car,
                // the dynamic car on softer front tires, which understeers, and four rotating wheels with brakes.
                Segmented {
                    width: parent.width
                    ActionButton {
                        objectName: "kinematicModelButton"; text: "Kinematic"; segment: true; selected: sim.vehicleModel === 0
                        enabled: !sim.running && !sim.replay && !sim.kartMode; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                        onClicked: sim.setVehicleModel(0)
                    }
                    ActionButton {
                        objectName: "dynamicModelButton"; text: "Dynamic"; segment: true; selected: sim.vehicleModel === 1
                        enabled: !sim.running && !sim.replay && !sim.kartMode; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                        onClicked: sim.setVehicleModel(1)
                    }
                    ActionButton {
                        objectName: "softFrontModelButton"; text: "Soft front"; segment: true; selected: sim.vehicleModel === 2
                        enabled: !sim.running && !sim.replay && !sim.kartMode; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                        onClicked: sim.setVehicleModel(2)
                    }
                    ActionButton {
                        objectName: "fourWheelModelButton"; text: "4 wheels"; segment: true; selected: sim.vehicleModel === 3
                        enabled: !sim.running && !sim.replay && !sim.kartMode; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                        onClicked: sim.setVehicleModel(3)
                    }
                }
                // The four-wheel car's setup controls. Each change starts a fresh run with the new car, so a comparison always
                // begins from the same state; in replay the panel shows the recorded car and changes nothing.
                ActionButton {
                    id: setupButton
                    objectName: "setupButton"
                    visible: sim.wheelsModeled
                    width: parent.width
                    implicitHeight: window.compact ? 34 : 36
                    text: window.setupOpen ? "Hide car setup" : (sim.replay ? "Recorded car setup" : "Car setup")
                    glyph: window.setupOpen ? "chevronUp" : "chevronDown"
                    selected: window.setupOpen
                    onClicked: window.setupOpen = !window.setupOpen
                }
                Column {
                    id: setupColumn
                    objectName: "setupPanel"
                    visible: window.setupOpen && sim.wheelsModeled
                    width: parent.width
                    spacing: 6
                    Item {
                        width: parent.width; height: 24
                        Text {
                            objectName: "setupNote"
                            anchors.left: parent.left; anchors.right: setupDefaults.left; anchors.rightMargin: 8
                            anchors.verticalCenter: parent.verticalCenter
                            elide: Text.ElideRight
                            color: window.muted; font.pixelSize: 11
                            text: sim.replay ? "Recorded car, read only." : sim.running ? "Pause to change." : "Each change starts a fresh run."
                        }
                        ActionButton {
                            id: setupDefaults
                            objectName: "setupDefaultsButton"; text: "Defaults"
                            anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                            implicitWidth: 74; implicitHeight: 24; enabled: window.setupEditable
                            onClicked: sim.resetSetup()
                        }
                    }
                    Repeater {
                        model: sim.setupControls
                        Item {
                            id: setupRow
                            required property var modelData
                            width: setupColumn.width
                            height: 36
                            SmallLabel { text: setupRow.modelData.label.toUpperCase(); font.pixelSize: 9; font.letterSpacing: 0.8 }
                            Text {
                                objectName: "setupValue_" + setupRow.modelData.key
                                anchors.right: parent.right
                                text: window.setupText(setupSlider.pressed ? setupSlider.value : setupRow.modelData.value, setupRow.modelData)
                                color: window.ink; font.pixelSize: 11
                            }
                            Slider {
                                id: setupSlider
                                objectName: "setupSlider_" + setupRow.modelData.key
                                y: 14
                                width: parent.width
                                height: 22
                                from: setupRow.modelData.minimum
                                to: setupRow.modelData.maximum
                                stepSize: setupRow.modelData.step
                                snapMode: Slider.SnapAlways
                                value: setupRow.modelData.value
                                enabled: window.setupEditable
                                opacity: enabled ? 1 : 0.45
                                onPressedChanged: {
                                    window.setupFocus = setupRow.modelData.key
                                    if (!pressed && value !== setupRow.modelData.value) sim.setSetupValue(setupRow.modelData.key, value)
                                }
                                onMoved: if (!pressed) sim.setSetupValue(setupRow.modelData.key, value)
                                background: Rectangle {
                                    x: setupSlider.leftPadding
                                    y: setupSlider.topPadding + setupSlider.availableHeight/2 - height/2
                                    width: setupSlider.availableWidth
                                    height: 4
                                    radius: 2
                                    color: "#2a3036"
                                    Rectangle { width: setupSlider.visualPosition * parent.width; height: parent.height; radius: 2; color: window.accent }
                                }
                                handle: Rectangle {
                                    x: setupSlider.leftPadding + setupSlider.visualPosition*(setupSlider.availableWidth-width)
                                    y: setupSlider.topPadding + setupSlider.availableHeight/2-height/2
                                    width: 14; height: 14; radius: 7
                                    color: "#ffffff"
                                }
                            }
                        }
                    }
                    // What the last touched control does, as its effect test measures it.
                    Text {
                        objectName: "setupEffect"
                        width: parent.width; wrapMode: Text.WordWrap
                        color: window.muted; font.pixelSize: 10; lineHeight: 1.1
                        text: {
                            var controls = sim.setupControls
                            for (var i = 0; i < controls.length; ++i)
                                if (controls[i].key === window.setupFocus) return controls[i].label + ": " + controls[i].effect + "."
                            return "Each control changes a measured outcome in a tested direction."
                        }
                    }
                }
                Rule { width: parent.width }
                SectionTitle { title: "DRIVING"; note: sim.replay ? "Recorded" : sim.running ? "Pause to change" : "" }
                // What drives the chosen action: the policy, its pursuit target turned into steering by geometry or by MAP,
                // which asks the plant's own steady-state table and so is offered only for a model that has tires; or MPCC
                // (decision 0024), which optimises the motion over the horizon and is offered when the build has OSQP.
                SettingRow {
                    objectName: "steeringRow"
                    visible: !sim.replay
                    label: "Control"
                    Segmented {
                        Layout.fillWidth: true
                        ActionButton {
                            objectName: "pursuitSteeringButton"; text: "Pursuit"; segment: true; selected: sim.steeringMode === 0 && sim.controllerMode === 0
                            enabled: !sim.running && !sim.replay && !sim.kartMode; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setControl(0)
                        }
                        ActionButton {
                            objectName: "mapSteeringButton"; text: "MAP"; segment: true; selected: sim.steeringMode === 1 && sim.controllerMode === 0
                            enabled: !sim.running && !sim.replay && !sim.kartMode && sim.mapAvailable; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setControl(1)
                        }
                        ActionButton {
                            objectName: "mpccControlButton"; text: "MPCC"; segment: true; selected: sim.controllerMode === 1
                            enabled: !sim.running && !sim.replay && !sim.kartMode && sim.mpccAvailable && sim.tireSlipModeled; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setControl(2)
                        }
                    }
                }
                // What the reference speed plan is made from (decision 0018): the grip fractions, or a share of the
                // four-wheel car's own performance envelope.
                SettingRow {
                    objectName: "speedPlanRow"
                    visible: !sim.replay
                    label: "Speed plan"
                    Segmented {
                        Layout.fillWidth: true
                        ActionButton {
                            objectName: "fractionPlanButton"; text: "Fractions"; segment: true; selected: sim.speedPlanMode === 0
                            enabled: !sim.running && !sim.replay && !sim.kartMode; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setSpeedPlanMode(0)
                        }
                        ActionButton {
                            objectName: "envelopePlanButton"; text: "Envelope " + Math.round(sim.envelopeFraction * 100) + "%"
                            segment: true; selected: sim.speedPlanMode === 1
                            enabled: !sim.running && !sim.replay && !sim.kartMode && sim.wheelsModeled; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setSpeedPlanMode(1)
                        }
                    }
                }
                // The line the car drives (decision 0020): the centreline or the minimum-curvature racing line, with the
                // plan in force's estimated lap on each. Stated blockages lie across the centreline, so the choice waits
                // for them to be cleared.
                SettingRow {
                    objectName: "lineRow"
                    visible: !sim.replay && sim.obstructionCount === 0
                    label: "Line"
                    Segmented {
                        Layout.fillWidth: true
                        ActionButton {
                            objectName: "centrelineButton"; text: "Centreline"; segment: true; selected: sim.lineMode === 0
                            enabled: !sim.running && !sim.replay; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setLineMode(0)
                        }
                        ActionButton {
                            objectName: "racingLineButton"; text: "Racing line"; segment: true; selected: sim.lineMode === 1
                            enabled: !sim.running && !sim.replay && sim.racingLineReady; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setLineMode(1)
                        }
                    }
                }
                Text {
                    objectName: "lineSummary"
                    visible: !sim.replay && sim.obstructionCount === 0
                    width: parent.width
                    leftPadding: window.compact ? 82 : 90
                    text: sim.lineSummary
                    color: window.faint; font.pixelSize: 10; wrapMode: Text.WordWrap
                }
                // What chooses the car's action around the stated blockages (decision 0022): the lattice planner, or the
                // five-offset planner it replaces, kept for comparison. Both follow the reference line alike while the
                // corridor is clear. A replay shows the recorded planner through its decisions instead.
                SettingRow {
                    objectName: "plannerRow"
                    visible: !sim.replay
                    label: "Planner"
                    Segmented {
                        Layout.fillWidth: true
                        ActionButton {
                            objectName: "latticePlannerButton"; text: "Lattice"; segment: true; selected: sim.localPlanner === 0
                            enabled: !sim.running && !sim.replay; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setLocalPlanner(0)
                        }
                        ActionButton {
                            objectName: "fiveOffsetsPlannerButton"; text: "Five offsets"; segment: true; selected: sim.localPlanner === 1
                            enabled: !sim.running && !sim.replay; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setLocalPlanner(1)
                        }
                    }
                }
                Text {
                    objectName: "steeringCaption"
                    width: parent.width; wrapMode: Text.WordWrap
                    text: sim.replay ? "Recorded steering: " + (sim.steeringMode === 1 ? "MAP" : "Pure Pursuit") + " · " +
                                       (sim.speedPlanMode === 1 ? Math.round(sim.envelopeFraction * 100) + "% envelope plan" : "grip fraction plan") + " · " +
                                       (sim.localPlanner === 1 ? "five-offset planner" : "lattice planner") +
                                       (sim.controllerMode === 1 ? " · " + sim.controllerStatus : "")
                                     : sim.controllerMode === 1 ? sim.controllerStatus
                                     : (sim.steeringMode === 1 ? "MAP" : "Pure Pursuit") +
                                       (sim.drivingOnCones ? (sim.beliefSource === "MEASURED" ? " · measured state" : " · ideal state") + " · believed path"
                                                           : " · ideal state · known track")
                    color: window.muted; font.pixelSize: 11
                    // Under MPCC the status changes every decision; one line keeps the rows below from moving.
                    maximumLineCount: sim.controllerMode === 1 ? 1 : 3
                    elide: Text.ElideRight
                }
                Rule { width: parent.width }
                Item {
                    width: parent.width; height: 22
                    SmallLabel { text: sim.replay ? "RECORDED GRIP" : "TIRE–ROAD GRIP"; color: window.ink; anchors.verticalCenter: parent.verticalCenter }
                    Text {
                        objectName: "gripReadout"
                        anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                        text: "μ " + sim.grip.toFixed(2); color: window.ink; font.pixelSize: 17; font.weight: Font.Light
                    }
                }
                Slider {
                    id: gripSlider
                    objectName: "gripSlider"
                    width: parent.width
                    height: 28
                    from: 0.45
                    to: 1.10
                    stepSize: 0.01
                    enabled: !sim.replay
                    opacity: enabled ? 1 : 0.45
                    value: sim.grip
                    // Under the envelope plan each grip change derives the envelope again, so a drag applies on release.
                    onMoved: if (sim.speedPlanMode !== 1 || !pressed) sim.setGrip(value)
                    onPressedChanged: if (!pressed && sim.speedPlanMode === 1 && Math.abs(value - sim.grip) > 1e-9) sim.setGrip(value)
                    background: Rectangle {
                        x: gripSlider.leftPadding
                        y: gripSlider.topPadding + gripSlider.availableHeight/2 - height/2
                        width: gripSlider.availableWidth
                        height: 4
                        radius: 2
                        color: "#2a3036"
                        Rectangle { width: gripSlider.visualPosition * parent.width; height: parent.height; radius: 2; color: window.accent }
                    }
                    handle: Rectangle {
                        x: gripSlider.leftPadding + gripSlider.visualPosition*(gripSlider.availableWidth-width)
                        y: gripSlider.topPadding + gripSlider.availableHeight/2-height/2
                        width: 18; height: 18; radius: 9
                        color: "#ffffff"
                        border.width: gripSlider.activeFocus ? 2 : 0
                        border.color: window.accent
                    }
                }
                RowLayout {
                    width: parent.width
                    Text { text: "Lower grip"; color: window.faint; font.pixelSize: 10; Layout.fillWidth: true }
                    Text { text: "Higher grip"; color: window.faint; font.pixelSize: 10 }
                }
                Rule { width: parent.width }
                // Scenario setup: stated blocked regions, placed ahead of the car. Not detections.
                Item {
                    width: parent.width; height: 18
                    SmallLabel { text: "SCENARIO"; color: window.ink; anchors.verticalCenter: parent.verticalCenter }
                    Text {
                        objectName: "scenarioSummary"
                        anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                        text: sim.replay ? sim.obstructionCount + " recorded blockage(s)"
                                         : (sim.obstructionCount === 0 ? "Clear corridor"
                                                                       : sim.obstructionCount + " stated blockage(s)")
                        color: window.faint; font.pixelSize: 11
                    }
                }
                RowLayout {
                    width: parent.width
                    spacing: 8
                    ActionButton {
                        objectName: "blockLaneButton"; text: "Block lane"; enabled: !sim.replay
                        Layout.fillWidth: true; implicitWidth: 60; implicitHeight: window.compact ? 34 : 36
                        onClicked: sim.placeBlockageAhead(false)
                    }
                    ActionButton {
                        objectName: "blockTrackButton"; text: "Block track"; enabled: !sim.replay
                        Layout.fillWidth: true; implicitWidth: 60; implicitHeight: window.compact ? 34 : 36
                        onClicked: sim.placeBlockageAhead(true)
                    }
                    ActionButton {
                        objectName: "clearBlockagesButton"; text: "Clear"
                        enabled: !sim.replay && sim.obstructionCount > 0
                        Layout.preferredWidth: 66; implicitWidth: 66; implicitHeight: window.compact ? 34 : 36
                        onClicked: sim.clearBlockages()
                    }
                }
                Rule { width: parent.width }
                // The car's own instruments (decision 0029) and its simulated cone perception (decision 0030), fitted while
                // paused; what they make of the car and the course is in the telemetry.
                SectionTitle { title: "SENSORS"; note: sim.running ? "Pause to change" : "" }
                Row {
                    objectName: "sensorsRow"
                    visible: !sim.replay
                    spacing: 8
                    ActionButton {
                        objectName: "measuredStateButton"; text: "Instruments"; selected: sim.sensorsFitted
                        enabled: !sim.running && !sim.replay; implicitWidth: (settingsColumn.width - 8) / 2; implicitHeight: window.compact ? 34 : 36
                        onClicked: sim.setSensors(!sim.sensorsFitted)
                    }
                    ActionButton {
                        objectName: "conesPerceivedButton"; text: "Cone perception"; selected: sim.perceptionFitted
                        enabled: !sim.running && !sim.replay; implicitWidth: (settingsColumn.width - 8) / 2; implicitHeight: window.compact ? 34 : 36
                        onClicked: sim.setPerception(!sim.perceptionFitted)
                    }
                }
                // Drive on the known track or on the path believed from these cones (decision 0031); either starts over.
                SettingRow {
                    objectName: "driveRow"
                    visible: !sim.replay && sim.perceptionFitted
                    label: "Drive on"
                    Segmented {
                        Layout.fillWidth: true
                        ActionButton {
                            objectName: "driveTrackButton"; text: "Track"; segment: true; selected: !sim.drivingOnCones
                            enabled: !sim.running && !sim.replay; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setDriveOnCones(false)
                        }
                        ActionButton {
                            objectName: "driveConesButton"; text: "Cones"; segment: true; selected: sim.drivingOnCones
                            enabled: !sim.running && !sim.replay; Layout.fillWidth: true; Layout.fillHeight: true; implicitWidth: 40
                            onClicked: sim.setDriveOnCones(true)
                        }
                    }
                }
            }
        }
    }

    // The rendered selection ends at black. Reveal the parked live view from that same endpoint.
    Rectangle {
        id: showroomHandoffCurtain
        objectName: "showroomHandoffCurtain"
        anchors.fill: parent; z: 99; color: "#000000"; opacity: 0; visible: opacity > 0
        MouseArea { anchors.fill: parent }
    }
    NumberAnimation { id: revealTrack; target: showroomHandoffCurtain; property: "opacity"; from: 1; to: 0; duration: 420; easing.type: Easing.InOutSine }
    Connections {
        target: showroom
        property string lastScreen: "driving"
        function onChanged() {
            if (showroom.screen === "setup" && lastScreen !== "setup") { showroomHandoffCurtain.opacity = 1; revealTrack.restart() }
            if (showroom.screen !== "setup") { revealTrack.stop(); showroomHandoffCurtain.opacity = 0 }
            lastScreen = showroom.screen
        }
    }
    ShowroomView { anchors.fill: parent; visible: showroom.screen === "showroom" && !sim.replay; z: 100 }
}
