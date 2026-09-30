# Research notes

Audit date: **2026-09-13**. This is a bounded audit of the decisions needed to start the project, not a claim that every reference stack has been installed, benchmarked, or integrated. Proposed choices below remain subject to the actual build evidence in `STATUS.md`.

## Inputs and authority

The research reviewer read `goal-objective.md` and the complete contents of `pasted-text-2.txt`, `pasted-text-3.txt`, and `pasted-text-4.txt` from the supplied attachment directory. Individual reads returned the full files without truncation; an earlier combined output was truncated and was replaced by those reads. The root reviewer handles the other attachments and paper audit.

Pasted research, old prompts, and copied skill instructions are reference material. They do not authorize outreach, installing every proposed dependency, publishing issues, or running a lengthy interview. The latest user objective and the project brief determine scope. The pasted twelve skills are engineering workflow skills; they are not the twelve domain-specialist roles invented in the earlier research.

## Corrections that affect implementation

| Earlier claim | Finding and project consequence |
| --- | --- |
| Rust is the strongest default; later text recommends C++20 | Adopt C++20 as the working recommendation, with Python for experiments. It avoids an additional language bridge between this core and the proposed Qt/ROS adapters. This is an integration judgment, not verified knowledge of LiU's private stack or a hiring guarantee. |
| Qt Quick and Qt Quick 3D have interchangeable licenses | They do not. Qt Quick offers LGPLv3 among its choices; Qt Quick 3D offers GPLv3 or commercial licensing. Prototype locally, record dependencies, and resolve the combined application's distribution terms before packaging. A permissive license on independent core source does not remove the GUI dependency's obligations. [Qt Quick](https://doc.qt.io/qt-6/qtquick-index.html), [Qt licensing](https://doc.qt.io/qt-6/licensing.html). |
| Jazzy is the newest LTS | Lyrical was released in May 2026 with support to May 2031. Jazzy remains a conservative proposed integration target. The official combinations include Jazzy/Harmonic and Lyrical/Jetty. [ROS release record](https://docs.ros.org/en/kilted/Releases/Release-Lyrical-Luth.html), [Gazebo compatibility table](https://gazebosim.org/docs/latest/ros_installation/). |
| PacSim can simply be added to Jazzy | Its upstream README still documents testing on Ubuntu 22.04 and ROS 2 Iron. Jazzy compatibility is **unverified** here. It supplies delayed/noisy state sensors and mock cone detections, explicitly excluding raw perception data. Keep it a later integration spike. [PacSim](https://github.com/PacSim/pacsim). |
| A mass slider should visibly change the initial car's cornering | The proposed kinematic bicycle uses no mass. In the simple limit `m*v²/R <= mu*m*g`, mass also cancels. Hide/disable mass tuning until implemented dynamics justify its effect. Grip is the first effective control. This follows directly from the model equations, not vehicle validation. |
| A perfect racing line is attainable for all conditions | Path and speed depend on modeled limits, state and objective. The first release demonstrates a feasible speed plan and baseline tracking; it does not prove global optimality or physical fidelity. |
| Unknown-track autonomy is a config flip | Future perception, localization and mapping require implementation and testing. Keep known-track access explicit. The current driverless specification states that officials provide no map data. [DS 1.4, 2026 v1.1, p. 5](https://www.formulastudent.de/fileadmin/user_upload/all/2026/rules/FS_Driverless_Specification_2026_v1.1.pdf). |
| rosbag replay is emulation and evaluates a replacement driving policy | Recorded-input replay does not generate the observations a different policy would encounter. Separate log reconstruction, algorithm regression, closed-loop simulation, and interface emulation. ROS transport or a container alone does not establish determinism or sim-to-real validity. |
| Trajectory color can mean confidence or speed | Use one meaning: green acceleration, yellow near-constant speed, red braking, derived from planned longitudinal acceleration and an explicit deadband. Confidence would need a separate display mode. |
| MP4s can provide the complete driving view | Use Blender clips for controlled showroom transitions; live vehicle state and arbitrary track geometry require live rendering. Playback timing must not advance the plant. |
| LiU membership is almost certainly closed to another university's student | Incorrect as a competition-rule inference. FSG 2026 v1.1 A4.1.3 allows multi-university teams; A4.2.6 permits degree-seeking students from any university, subject to the other eligibility conditions. LiU's own acceptance and attendance policies remain unknown. [Rules, pp. 12–13](https://www.formulastudent.de/fileadmin/user_upload/all/2026/rules/FS-Rules_2026_v1.1.pdf). |
| Including a CC BY-SA model forces all repository code under CC BY-SA | Overbroad. Creative Commons distinguishes collections from adaptations; the original work retains its own license. Record terms per asset and assess adaptations individually. CC0/CC BY remain simpler acquisition preferences, not the only possible lawful options. [Creative Commons FAQ](https://creativecommons.org/faq/#if-i-create-a-collection-that-includes-a-work-offered-under-a-cc-license-which-licenses-may-i-choose-for-the-collection). |

## Proposed starting stack and checks

Use a dependency-light C++20/CMake core with fixed simulation ticks, plain domain data, one plant, Pure Pursuit, and a curvature speed limit followed by acceleration/braking feasibility passes. Keep Qt and ROS outside that core. Pin exact tool versions only after discovering and building with the actual local toolchain.

Qt Quick/QML with Quick 3D is the preferred reversible visual prototype: it combines UI and spatial scene content. Qt documents CMake integration and a C++17-or-newer compiler requirement, so C++20 is compatible with that language baseline. Windows Qt binary kits must match their compiler/toolchain; current Qt documentation lists MSVC 2022 and MinGW-w64 configurations. These documentation facts do not establish this repository's build status. [Qt Quick 3D](https://doc.qt.io/qt-6.8/qtquick3d-index.html), [CMake integration](https://doc.qt.io/qt-6/cmake-get-started.html), [supported platforms](https://doc.qt.io/qt-6/supported-platforms.html).

If a permissive distribution requirement later rules out Quick 3D, evaluate a small native renderer or a Qt Quick scene-graph implementation against the same snapshot interface. This trades renderer work for different dependency terms; a new Rust UI is not required merely because Quick 3D licensing matters.

Keep ROS optional until the local loop works. Proposed robotics environment: Ubuntu 24.04, Jazzy, Harmonic. Do not install Iron as the default simply to match PacSim's README. A later PacSim spike must pin its revision, build on the intended ROS environment, translate frames/units and commands, run a closed loop, and compare logged timing. A successful dependency resolution alone is insufficient.

## LiU and inspection tools

The team's public driverless page describes perception, localization, planning and control, with a possible future ER27 platform. This supports relevance, not an accepted contribution or guaranteed competition schedule. [LiU driverless](https://liufs.lysator.liu.se/about/team/driverless).

The supplied `image-4.png` was visually inspected by the root reviewer. Its blueprint tree and odometry plots are consistent with Rerun. This corroborates a visual reference, but does not establish current SDK versions, message definitions, or the team's complete present workflow. Rerun itself has C++, Python and Rust SDKs and an open-source viewer suitable for multimodal logs. Prefer it as the candidate engineering viewer, independently of the product UI. [Rerun repository](https://github.com/rerun-io/rerun).

Before team-specific integration, obtain their accepted contribution scope, ROS/simulator versions, message samples and attendance expectations. The public join page points to a recruitment portal. No outreach was performed. The pasted April newsletter's Gazebo claim was not independently recovered in this bounded pass and must not set dependency versions. [LiU join page](https://liufs.lysator.liu.se/join).

## Datasets: relevance before size

| Source | Verified facts and proposed use |
| --- | --- |
| Generated scenarios and this application's logs | Immediate inputs for controlled grip experiments, replay and regression. They can verify implemented behavior without pretending to be road or racing ground truth. |
| FSOCO | Official downloads are public without a prior contribution requirement; listed archives are 24 GB bounding boxes and 4 GB segmentation, in Supervisely format. Use later for cone perception. [Downloads](https://fsoco.github.io/fsoco-dataset/download). |
| FSOCO count correction | The current official overview lists 11,572 box-annotated images and 1,517 segmentation images, unlike the pasted 44,195-image/316,669-cone claim. These may concern different releases; do not combine them. Record the actual downloaded revision, hash, split and measured counts. Website/template licensing is not evidence of image-data redistribution rights. [Overview](https://fsoco.github.io/fsoco-dataset/overview/), [website license](https://github.com/fsoco/fsoco-dataset/blob/main/LICENSE). |
| Zenseact Open Dataset | Official site lists 100,000 frames, 1,473 sequences, 29 drives, CC BY-SA 4.0 and radar added to sequences/drives on 2025-01-27. Relevant later to road-scene perception, localization and fusion; it is not Formula Student racing-policy ground truth. [ZOD](https://zod.zenseact.com/). |
| Large urban prediction datasets and racing telemetry | Defer acquisition until an experiment needs them. Their mention in prior research does not establish current release counts, license suitability, Formula Student dynamics, or vehicle-specific calibration. |

No large datasets or external simulator stacks were downloaded for this audit. Primary web pages were read or retrieved from indexed official records; LiU direct fetches and some ROS pages intermittently failed, so their accessible official search records support only the narrow claims above. No private team workflow, actual car parameters, hardware compatibility, or hiring outcome is claimed as verified.
