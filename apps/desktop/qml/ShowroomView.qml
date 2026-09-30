pragma ComponentBehavior: Bound
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Shapes
import QtQuick.Effects

Rectangle {
    id: root
    objectName: "showroomView"
    color: "#000000"
    property bool showMedia: true
    readonly property color ink: "#edf0f0"
    readonly property color muted: "#92989c"
    // The studio occupies a clean 16:9 plate, preserving the animation's endpoint framing at any window size.
    MouseArea { anchors.fill: parent }
    Image { anchors.fill: parent; source: showroom.poster; fillMode: Image.PreserveAspectFit; cache: true; asynchronous: false }
    Loader { id: playback; anchors.fill: parent }
    function loadClip() {
        if (root.visible && root.showMedia && playback.item && playback.item.token === showroom.serial) return
        playback.source = ""
        if (root.visible && root.showMedia && String(showroom.clip).length > 0)
            playback.setSource("ShowroomPlayer.qml", {token: showroom.serial, clipSource: showroom.clip, loopClip: showroom.looping})
    }
    Connections { target: showroom; function onPlaybackChanged() { root.loadClip() } }
    onVisibleChanged: loadClip()
    onShowMediaChanged: loadClip()
    Component.onCompleted: loadClip()

    Rectangle {
        anchors.top: parent.top; width: parent.width; height: 205
        gradient: Gradient { GradientStop { position: 0; color: "#e8000000" } GradientStop { position: 1; color: "#00000000" } }
    }
    // The brand's logo names the car; its text is only the fallback when the pack has no logo.
    // Every logo is centred in one box sized to the Red Bull mark, so wide and tall marks share an axis.
    Image {
        objectName: "showroomHeroLogo"
        anchors { left: parent.left; top: parent.top; leftMargin: 46; topMargin: 90 }
        width: root.width < 1250 ? 190 : 236; height: root.width < 1250 ? 88 : 110
        source: showroom.cars[showroom.current].logo
        sourceSize.height: 2 * height
        fillMode: Image.PreserveAspectFit; horizontalAlignment: Image.AlignHCenter; verticalAlignment: Image.AlignVCenter
        smooth: true; mipmap: true
        visible: String(source).length > 0
    }
    Text {
        objectName: "showroomHeroName"
        anchors { left: parent.left; top: parent.top; leftMargin: 46; topMargin: 112 }
        visible: String(showroom.cars[showroom.current].logo).length === 0
        text: showroom.cars[showroom.current].name; color: root.ink; font.pixelSize: root.width < 1250 ? 34 : 43; font.weight: Font.Light
    }
    Rectangle {
        anchors.bottom: parent.bottom; width: parent.width; height: 210
        gradient: Gradient { GradientStop { position: 0; color: "#00000000" } GradientStop { position: 0.4; color: "#d9000000" } GradientStop { position: 1; color: "#ff000000" } }
    }
    ColumnLayout {
        anchors { left: parent.left; right: parent.right; bottom: parent.bottom; margins: 46 }
        spacing: 19
        Item {
            Layout.fillWidth: true
            id: browseBar
            Layout.preferredHeight: 66
            // Browsing order: AMR23, RB19, Jesko, Urus (appearance indices 0, 3, 1, 2). Each arrow requests the
            // neighbour of the requested car; rapid presses keep only the latest destination.
            readonly property var order: [0, 3, 1, 2]
            function step(direction) {
                const position = order.indexOf(showroom.requested)
                showroom.request(order[(position + direction + order.length) % order.length])
            }
            Row {
                id: browser
                anchors.centerIn: parent
                spacing: 110 + 2 * 36
                Repeater {
                    model: [{name: "showroomPrevious", glyph: "‹", direction: -1}, {name: "showroomNext", glyph: "›", direction: 1}]
                    delegate: Button {
                        id: arrow
                        required property var modelData
                        required property int index
                        objectName: modelData.name
                        width: 58; height: 58
                        enabled: !showroom.confirming
                        hoverEnabled: true
                        onClicked: browseBar.step(arrow.modelData.direction)
                        background: Item {}
                        contentItem: Text {
                            text: arrow.modelData.glyph; color: root.ink; font.pixelSize: 72; font.weight: Font.Light; opacity: arrow.down ? 0.55 : arrow.hovered ? 1 : 0.85
                            horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                            bottomPadding: 14
                        }
                    }
                }
            }
            // The requested car's logo sits between the arrows, centred on the bottom of the frame.
            Image {
                objectName: "showroomBrowseLogo"
                anchors.centerIn: parent
                width: 110; height: 40
                source: showroom.cars[showroom.requested].logo
                sourceSize.height: 2 * height
                fillMode: Image.PreserveAspectFit; horizontalAlignment: Image.AlignHCenter; verticalAlignment: Image.AlignVCenter
                smooth: true; mipmap: true
            }
            Button {
                id: continueButton
                objectName: "showroomContinue"
                text: showroom.confirming ? "Preparing track…" : "Select car    →"
                anchors { right: parent.right; verticalCenter: parent.verticalCenter }
                width: 210; height: 62
                enabled: !showroom.confirming
                hoverEnabled: true
                onClicked: showroom.confirm()
                // Transparent button: at rest only a faint top edge and its chamfer. On hover a short line in the car's
                // colour runs one lap of the chamfered outline and settles on the top edge.
                readonly property real chamfer: 16
                readonly property real stroke: 3
                readonly property real accent: width - chamfer + chamfer * Math.SQRT2
                readonly property real perimeter: 2 * (width - chamfer) + 2 * (height - chamfer) + 2 * chamfer * Math.SQRT2
                readonly property var colours: ({amr23: ["#0b3d2e", "#1f9e62"], rb19: ["#d8141f", "#101f5c"], jesko: ["#f5c400", "#b87400"], urus: ["#e2b64a", "#8a6414"]})
                readonly property var carColours: colours[showroom.cars[showroom.requested].id] || ["#f5c400", "#b87400"]
                readonly property bool pointed: pointer.hovered && enabled
                property real lap: 0
                HoverHandler { id: pointer; cursorShape: Qt.PointingHandCursor }
                NumberAnimation { id: lapRun; target: continueButton; property: "lap"; from: 0; to: continueButton.perimeter; duration: 650; easing.type: Easing.InOutCubic }
                onPointedChanged: if (pointed) lapRun.restart()
                // One line only: at rest it is the faint top edge and chamfer; on hover it takes the car's gradient, runs one
                // lap of the chamfered outline at the same length, and settles back on the top edge. The line is a mask over
                // a horizontal gradient, whose colours blend when the car or the hover state changes.
                background: Item {
                    Shape {
                        id: lineMask
                        anchors.fill: parent
                        visible: false
                        layer.enabled: true
                        ShapePath {
                            strokeColor: "white"; strokeWidth: continueButton.stroke; fillColor: "transparent"
                            capStyle: ShapePath.FlatCap; joinStyle: ShapePath.MiterJoin
                            strokeStyle: ShapePath.DashLine
                            dashPattern: [continueButton.accent / continueButton.stroke,
                                          (continueButton.perimeter - continueButton.accent) / continueButton.stroke]
                            dashOffset: -continueButton.lap / continueButton.stroke
                            startX: 0; startY: 0
                            PathLine { x: continueButton.width - continueButton.chamfer; y: 0 }
                            PathLine { x: continueButton.width; y: continueButton.chamfer }
                            PathLine { x: continueButton.width; y: continueButton.height }
                            PathLine { x: continueButton.chamfer; y: continueButton.height }
                            PathLine { x: 0; y: continueButton.height - continueButton.chamfer }
                            PathLine { x: 0; y: 0 }
                        }
                    }
                    Rectangle {
                        id: lineFill
                        anchors.fill: parent
                        visible: false
                        layer.enabled: true
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop {
                                position: 0; color: continueButton.pointed ? continueButton.carColours[0] : "#59ffffff"
                                Behavior on color { ColorAnimation { duration: 260 } }
                            }
                            GradientStop {
                                position: 1; color: continueButton.pointed ? continueButton.carColours[1] : "#59ffffff"
                                Behavior on color { ColorAnimation { duration: 260 } }
                            }
                        }
                    }
                    MultiEffect { anchors.fill: parent; source: lineFill; maskEnabled: true; maskSource: lineMask }
                }
                contentItem: Text {
                    text: continueButton.text; color: "#f2f4f4"; font.pixelSize: 15
                    horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                }
            }
        }
        // Only a media failure is worth words here.
        Text {
            objectName: "showroomMediaStatus"
            visible: showroom.status.length > 0
            text: showroom.status
            color: root.muted; font.pixelSize: 10; Layout.fillWidth: true; elide: Text.ElideRight
        }
    }
}
