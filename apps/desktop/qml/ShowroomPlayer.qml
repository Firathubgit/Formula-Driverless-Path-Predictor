pragma ComponentBehavior: Bound
import QtQuick
import QtMultimedia

// A fresh player for every clip. The immutable token makes late decoder callbacks harmless.
Item {
    id: root
    required property int token
    required property url clipSource
    required property bool loopClip
    property bool firstFrame: false
    MediaPlayer {
        id: player
        source: root.clipSource
        videoOutput: output
        loops: root.loopClip ? MediaPlayer.Infinite : 1
        onMediaStatusChanged: {
            if (mediaStatus === MediaPlayer.EndOfMedia) showroom.complete(root.token)
            else if (mediaStatus === MediaPlayer.InvalidMedia) showroom.failed(root.token, "This animation could not be decoded")
        }
        onErrorOccurred: function(error, errorString) { showroom.failed(root.token, "This animation could not be played") }
    }
    VideoOutput { id: output; anchors.fill: parent; fillMode: VideoOutput.PreserveAspectFit; visible: root.firstFrame }
    Connections {
        target: output.videoSink
        function onVideoFrameChanged(frame) {
            // A fresh sink may announce an empty reset before decoding. Keep the canonical poster until it has pixels.
            if (!root.firstFrame && output.videoSink.videoSize.width > 0 && output.videoSink.videoSize.height > 0) {
                root.firstFrame = true
                showroom.started(root.token)
            }
        }
    }
    Component.onCompleted: player.play()
    Component.onDestruction: player.stop()
}
