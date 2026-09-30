# 0001: C++20 core, native Qt prototype, ROS later

Date: 2026-09-13. Status: accepted for internal development; distribution route proposed.

The product needs a controlled vehicle whose behavior and speed plan are visible in a native
3D interface. The machine has MSVC and Blender; it initially lacked Qt SDK, ROS and WSL.
The latest supplied brief favors C++ and Qt. Rust would add a language bridge without an
active benefit. A browser dashboard would change the intended product.

Use C++20/CMake with no external core dependencies, a small local kinematic simulator, and
Qt Quick/QML + Quick 3D for live presentation. A project-local Qt 6.8.3 MSVC 2022 kit is used;
this is an explicit tested version target, not a claim that it is the newest Qt release.
The headless executable can build with no Qt. Windows x64 is the initial reference host.
ROS 2/Jazzy on Ubuntu 24.04 remains the proposed later integration environment, subject to
actual team and simulator compatibility.

Trade-off: the native renderer fits the composition and integration goals but adds SDK setup
and distribution obligations. Qt Quick 3D is GPLv3/commercial; project-owned code and assets
need their own license decision before public packaging. Proposed public route is a compatible
open-source application, with notices/source obligations assessed for the actual dependencies.
No commercial license was purchased and no permissive license is silently asserted for the
combined application. Local reversible development can continue under the supplied brief.

If a permissive combined product becomes a requirement, evaluate replacing the renderer
behind the existing snapshot adapter. Folder separation alone does not settle linked licensing.

Primary references: [Qt Quick 3D](https://doc.qt.io/qt-6.8/qtquick3d-index.html),
[Qt licensing](https://doc.qt.io/qt-6/licensing.html),
[ROS/Gazebo compatibility](https://gazebosim.org/docs/latest/ros_installation/).
