pragma ComponentBehavior: Bound
import QtQuick
import QtQuick.Shapes
import QtQuick3D

// The live scene's car: the showroom's own car for the chosen appearance, exported with its textures and turning wheels
// (tools/showroom/build_live_cars.py, read by LiveCars), or a procedural silhouette where none is built. Either way the
// appearance has no effect on the plant and the car stands where the plant is: origin at the rear axle, forward along
// local -Z, local +X the car's right. The exported car keeps its own dimensions, so its front axle sits at its own
// wheelbase ahead of the rear, not the plant's.
Node {
    id: car
    property int appearance: 0
    property real steeringDegrees: 0
    // The procedural silhouette's wheelbase.
    property real wheelbase: 2.6
    // Front left, front right, rear left, rear right, as the simulation reports them; a locked wheel shows red.
    property var wheelStates: []
    // The exported car for this appearance, LiveCars.cars[appearance]; one that is not available leaves the silhouette.
    property var asset: null
    readonly property bool live: !!asset && asset.available === true
    // A mirrored copy under the translucent road is the car's reflection: it draws no shadow or halo, and its parts
    // carry no names; those are the car's own.
    property bool reflection: false
    // The wheels turn with the car's speed in m/s while the plant moves on, each by its own reported slip ratio.
    property real speed: 0
    property bool rolling: false
    property var slipRatios: []
    // Drawn at another wheelbase, as gokart mode draws a kart-sized car (decision 0038): the whole car scaled uniformly
    // about its rear axle, so its front wheels sit on the plant's front axle. Zero draws it at its own size.
    property real drawnWheelbase: 0
    readonly property real modelWheelbase: live ? asset.wheelbase : wheelbase
    readonly property real drawnScale: drawnWheelbase > 0 ? drawnWheelbase/modelWheelbase : 1

    // Each appearance's livery, lighter than the road so the car reads against it: Aston Martin green, silver, graphite
    // and Red Bull navy, glossed with a clear coat. Livery accents are not plan colours; those live on the path alone.
    PrincipledMaterial {
        id: paint
        baseColor: ["#17664f", "#c3c9ce", "#4b5359", "#1f2f66"][Math.max(0, Math.min(3, car.appearance))]
        metalness: 0.35; roughness: 0.3; clearcoatAmount: 0.7; clearcoatRoughnessAmount: 0.12
    }
    PrincipledMaterial { id: carbon; baseColor: "#1b2025"; metalness: 0.2; roughness: 0.55 }
    PrincipledMaterial { id: glass; baseColor: "#101820"; metalness: 0.6; roughness: 0.12 }
    PrincipledMaterial { id: accent; baseColor: car.appearance === 3 ? "#d8342c" : "#c6d63a"; roughness: 0.4 }
    PrincipledMaterial { id: rubber; baseColor: "#11171c"; roughness: 0.94 }
    DefaultMaterial { id: lockedRubber; diffuseColor: "#f0484f"; lighting: DefaultMaterial.NoLighting }
    PrincipledMaterial { id: alloy; baseColor: "#77848d"; metalness: 0.8; roughness: 0.25 }
    DefaultMaterial { id: lamp; diffuseColor: "#dcf3f5"; lighting: DefaultMaterial.NoLighting }
    DefaultMaterial { id: rearLamp; diffuseColor: "#ea5b60"; lighting: DefaultMaterial.NoLighting }
    PrincipledMaterial {
        id: lockGlow
        baseColor: "#f0484f"; lighting: PrincipledMaterial.NoLighting
        alphaMode: PrincipledMaterial.Blend; opacity: 0.5; cullMode: Material.NoCulling
    }

    Node {
        scale: Qt.vector3d(car.drawnScale, car.drawnScale, car.drawnScale)
        // The exported body: every panel, decal and lamp of the showroom's car, its wheels apart.
        Loader3D {
            objectName: car.reflection ? "" : "liveCarBody"
            active: car.live
            source: car.live ? car.asset.body : ""
        }

        // The exported car's contact shadow, measured from its own lowest geometry, darkening the road beneath it.
        Model {
            id: contactShadow
            objectName: car.reflection ? "" : "carShadow"
            readonly property var extent: car.live ? car.asset.shadow : null
            visible: !!extent && !car.reflection
            source: "#Rectangle"
            position: extent ? Qt.vector3d((extent.x[0] + extent.x[1]) / 2, 0.016, (extent.z[0] + extent.z[1]) / 2) : Qt.vector3d(0, 0, 0)
            eulerRotation.x: -90
            scale: extent ? Qt.vector3d((extent.x[1] - extent.x[0]) / 100, (extent.z[1] - extent.z[0]) / 100, 1) : Qt.vector3d(0.01, 0.01, 1)
            depthBias: 350000
            materials: PrincipledMaterial {
                lighting: PrincipledMaterial.NoLighting
                alphaMode: PrincipledMaterial.Blend
                cullMode: Material.NoCulling
                baseColor: "black"
                baseColorMap: Texture { source: contactShadow.extent ? contactShadow.extent.source : "" }
            }
        }

        // A soft pool of light on the road beneath the silhouette, as a driver's display grounds its own car.
        Model {
            objectName: car.reflection ? "" : "carHalo"
            visible: !car.live && !car.reflection
            source: "#Rectangle"
            position: Qt.vector3d(0, 0.02, -car.wheelbase / 2)
            eulerRotation.x: -90
            scale: Qt.vector3d(0.042, 0.078, 1)
            depthBias: 250000
            materials: PrincipledMaterial {
                lighting: PrincipledMaterial.NoLighting
                alphaMode: PrincipledMaterial.Blend
                cullMode: Material.NoCulling
                baseColorMap: Texture {
                    sourceItem: Shape {
                        width: 128; height: 128
                        ShapePath {
                            strokeColor: "transparent"
                            fillGradient: RadialGradient {
                                centerX: 64; centerY: 64; centerRadius: 64; focalX: 64; focalY: 64
                                GradientStop { position: 0; color: "#50dfe8ff" }
                                GradientStop { position: 0.55; color: "#1cdfe8ff" }
                                GradientStop { position: 1; color: "#00dfe8ff" }
                            }
                            PathLine { x: 128; y: 0 }
                            PathLine { x: 128; y: 128 }
                            PathLine { x: 0; y: 128 }
                            PathLine { x: 0; y: 0 }
                        }
                    }
                }
            }
        }

    }

    component Part: Model {
        source: "#Cube"
        property vector3d dimensions: Qt.vector3d(1, 1, 1)
        scale: Qt.vector3d(dimensions.x / 100, dimensions.y / 100, dimensions.z / 100)
        materials: [paint]
    }

    // One wheel of either car. The exported car's wheel sits where the export measured it; the silhouette's at its own
    // track and wheelbase. Front wheels steer about their centre by the plant's steering angle; every wheel turns about
    // its axle by the distance it rolls.
    component Wheel: Node {
        id: wheelNode
        property int wheel: 0
        readonly property bool front: wheel < 2
        readonly property var exported: car.live ? car.asset.wheels[wheel] : null
        readonly property real radius: exported ? exported.radius : 0.35
        readonly property real width: exported ? exported.width : 0.32
        readonly property bool locked: car.wheelStates.length > wheel && car.wheelStates[wheel] === "LOCK"
        // Degrees turned about the axle so far, rolling forward.
        property real turned: 0
        objectName: car.reflection ? "" : "carWheel" + wheel
        position: exported ? Qt.vector3d(exported.position[0], exported.position[1], exported.position[2])
                           : Qt.vector3d(wheel % 2 ? 0.84 : -0.84, 0.35, front ? -car.wheelbase : 0)
        eulerRotation.y: front ? car.steeringDegrees : 0
        Node {
            // Rolling toward -Z turns a wheel about -X.
            eulerRotation.x: -wheelNode.turned
            Loader3D {
                active: car.live
                source: wheelNode.exported ? wheelNode.exported.component : ""
            }
            Node {
                visible: !car.live
                Model {
                    objectName: car.reflection ? "" : "tire" + wheelNode.wheel
                    source: "#Cylinder"
                    eulerRotation.z: 90
                    scale: Qt.vector3d(0.007, 0.0032, 0.007)
                    materials: wheelNode.locked ? [lockedRubber] : [rubber]
                }
                Model {
                    source: "#Cylinder"
                    eulerRotation.z: 90
                    scale: Qt.vector3d(0.0042, 0.00325, 0.0042)
                    materials: [alloy]
                }
                Model {
                    source: "#Cylinder"
                    eulerRotation.z: 90
                    scale: Qt.vector3d(0.0017, 0.0033, 0.0017)
                    materials: [carbon]
                }
            }
        }
        // A locked wheel of the exported car glows red about its tire, as the silhouette's tire turns red.
        Model {
            visible: car.live && wheelNode.locked
            source: "#Cylinder"
            eulerRotation.z: 90
            scale: Qt.vector3d(wheelNode.radius * 1.08 / 50, wheelNode.width * 1.1 / 100, wheelNode.radius * 1.08 / 50)
            materials: [lockGlow]
        }
        FrameAnimation {
            running: car.rolling
            onTriggered: {
                var slip = car.slipRatios.length > wheelNode.wheel ? car.slipRatios[wheelNode.wheel] : 0
                var step = car.speed * (1 + slip) * frameTime / (wheelNode.radius * car.drawnScale)
                // At most 40 degrees a frame, so fast spokes read as turning forward rather than strobing backward.
                step = Math.max(-0.7, Math.min(0.7, step))
                wheelNode.turned = (wheelNode.turned + step * 180 / Math.PI) % 360
            }
        }
    }

    Node {
        scale: Qt.vector3d(car.drawnScale, car.drawnScale, car.drawnScale)
        Wheel { wheel: 0 }
        Wheel { wheel: 1 }
        Wheel { wheel: 2 }
        Wheel { wheel: 3 }

        Node {
            visible: !car.live && (car.appearance === 0 || car.appearance === 3)
            Part { position: Qt.vector3d(0, 0.26, -1.14); dimensions: Qt.vector3d(1.36, 0.12, 2.8); materials: [carbon] }
            Part { position: Qt.vector3d(0, 0.52, -0.94); dimensions: Qt.vector3d(0.72, 0.4, 2.15) }
            Part { position: Qt.vector3d(-0.47, 0.44, -0.63); dimensions: Qt.vector3d(0.27, 0.3, 1.45) }
            Part { position: Qt.vector3d(0.47, 0.44, -0.63); dimensions: Qt.vector3d(0.27, 0.3, 1.45) }
            Part { position: Qt.vector3d(0, 0.41, -2.19); dimensions: Qt.vector3d(0.36, 0.22, 1.19) }
            Part { position: Qt.vector3d(0, 0.28, -3.06); dimensions: Qt.vector3d(1.76, 0.09, 0.34); materials: [carbon] }
            Part { position: Qt.vector3d(0, 0.32, -3.16); dimensions: Qt.vector3d(1.55, 0.035, 0.09); materials: [accent] }
            Part { position: Qt.vector3d(-0.84, 0.34, -3.05); dimensions: Qt.vector3d(0.04, 0.22, 0.4) }
            Part { position: Qt.vector3d(0.84, 0.34, -3.05); dimensions: Qt.vector3d(0.04, 0.22, 0.4) }
            Part { position: Qt.vector3d(0, 0.86, 0.63); dimensions: Qt.vector3d(1.62, 0.13, 0.42); materials: [carbon] }
            Part { position: Qt.vector3d(0, 0.94, 0.74); dimensions: Qt.vector3d(1.52, 0.05, 0.06); materials: [accent] }
            Part { position: Qt.vector3d(-0.78, 0.84, 0.64); dimensions: Qt.vector3d(0.05, 0.38, 0.5) }
            Part { position: Qt.vector3d(0.78, 0.84, 0.64); dimensions: Qt.vector3d(0.05, 0.38, 0.5) }
            Part { position: Qt.vector3d(-0.31, 0.57, 0.46); dimensions: Qt.vector3d(0.05, 0.49, 0.1); materials: [carbon] }
            Part { position: Qt.vector3d(0.31, 0.57, 0.46); dimensions: Qt.vector3d(0.05, 0.49, 0.1); materials: [carbon] }
            Model { source: "#Sphere"; position: Qt.vector3d(0, 0.74, -0.87); scale: Qt.vector3d(0.0055, 0.0026, 0.008); materials: [glass] }
            Part { position: Qt.vector3d(0, 0.91, -0.49); dimensions: Qt.vector3d(0.39, 0.13, 0.15); materials: [carbon] }
            Part { position: Qt.vector3d(-0.24, 0.8, -0.92); dimensions: Qt.vector3d(0.04, 0.07, 0.83); materials: [carbon] }
            Part { position: Qt.vector3d(0.24, 0.8, -0.92); dimensions: Qt.vector3d(0.04, 0.07, 0.83); materials: [carbon] }
            Part { position: Qt.vector3d(0, 0.535, -2.06); dimensions: Qt.vector3d(0.09, 0.01, 1.05); materials: [accent] }
            Part { position: Qt.vector3d(0, 0.56, 0.21); dimensions: Qt.vector3d(0.13, 0.09, 0.03); materials: [rearLamp] }
            Part { position: Qt.vector3d(0, 0.32, 0); dimensions: Qt.vector3d(1.75, 0.05, 0.08); materials: [carbon] }
            Part { position: Qt.vector3d(0, 0.32, -car.wheelbase); dimensions: Qt.vector3d(1.75, 0.05, 0.08); materials: [carbon] }
        }

        Node {
            visible: !car.live && (car.appearance === 1 || car.appearance === 2)
            scale.y: car.appearance === 2 ? 1.4 : 1
            Part { position: Qt.vector3d(0, 0.36, -1.17); dimensions: Qt.vector3d(1.73, 0.2, 4.03); materials: [carbon] }
            Part { position: Qt.vector3d(0, 0.64, -1.17); dimensions: Qt.vector3d(1.68, 0.43, 3.98) }
            Part { position: Qt.vector3d(0, 0.91, -1.1); dimensions: Qt.vector3d(1.42, 0.34, 1.84); materials: [glass] }
            Part { position: Qt.vector3d(0, 1.13, -0.98); dimensions: Qt.vector3d(1.25, 0.12, 1.09) }
            Part { position: Qt.vector3d(0, 0.91, -2.04); dimensions: Qt.vector3d(1.4, 0.035, 0.6); eulerRotation.x: 32; materials: [glass] }
            Part { position: Qt.vector3d(0, 0.93, 0.01); dimensions: Qt.vector3d(1.38, 0.035, 0.6); eulerRotation.x: -28; materials: [glass] }
            Part { position: Qt.vector3d(-0.63, 0.76, -3.17); dimensions: Qt.vector3d(0.39, 0.075, 0.025); materials: [lamp] }
            Part { position: Qt.vector3d(0.63, 0.76, -3.17); dimensions: Qt.vector3d(0.39, 0.075, 0.025); materials: [lamp] }
            Part { position: Qt.vector3d(0, 0.68, 0.835); dimensions: Qt.vector3d(1.38, 0.055, 0.025); materials: [rearLamp] }
            Part { position: Qt.vector3d(0, 0.94, 0.63); dimensions: Qt.vector3d(1.53, 0.07, 0.19); materials: [carbon] }
            Part { position: Qt.vector3d(-0.91, 0.9, -1.65); dimensions: Qt.vector3d(0.17, 0.11, 0.2) }
            Part { position: Qt.vector3d(0.91, 0.9, -1.65); dimensions: Qt.vector3d(0.17, 0.11, 0.2) }
        }
    }
}
