find_package(Qt6 6.8 REQUIRED COMPONENTS Multimedia)
qt_add_library(fd_race_audio STATIC race_sound.cpp race_sound.hpp race_audio.cpp race_audio.hpp
  sample_loop.cpp sample_loop.hpp race_samples.cpp race_samples.hpp)
target_link_libraries(fd_race_audio PUBLIC Qt6::Core Qt6::Multimedia)
# Pinned media is local, ignored by Git and explicitly bootstrapped; configure never downloads.
set(FD_AUDIO_DIR "${CMAKE_SOURCE_DIR}/artifacts/audio")
foreach(pair "f1v10.wav|4ec848701dd8123a450bee27da7715efef708751820e67faaee9fd65cd639e97"
             "skid_tyres.wav|da5c521bd98da2c71d189db15e4facb26530163bab92861bd1dbf86177f3d0db")
  string(REPLACE "|" ";" parts "${pair}")
  list(GET parts 0 name)
  list(GET parts 1 expected)
  if(NOT EXISTS "${FD_AUDIO_DIR}/${name}")
    message(FATAL_ERROR "Recorded engine audio missing. Run scripts/bootstrap-audio.ps1 before the desktop build.")
  endif()
  file(SHA256 "${FD_AUDIO_DIR}/${name}" actual)
  if(NOT actual STREQUAL expected)
    message(FATAL_ERROR "${name} checksum mismatch. Run scripts/bootstrap-audio.ps1 to restore the pinned recording.")
  endif()
endforeach()
qt_add_resources(fd_race_audio "race_recordings" PREFIX "/audio" BASE "${FD_AUDIO_DIR}"
  FILES "${FD_AUDIO_DIR}/f1v10.wav" "${FD_AUDIO_DIR}/skid_tyres.wav")
qt_add_resources(fd_race_audio "race_audio_licenses" PREFIX "/audio/licenses" BASE "${CMAKE_SOURCE_DIR}/assets/audio-licenses"
  FILES "${CMAKE_SOURCE_DIR}/assets/audio-licenses/sources.json"
        "${CMAKE_SOURCE_DIR}/assets/audio-licenses/Free-Art-License-1.3.txt"
        "${CMAKE_SOURCE_DIR}/assets/audio-licenses/ExhaustNote-MIT.txt")
qt_add_executable(fd_desktop WIN32 main.cpp bridge.cpp bridge.hpp assistance_presets.cpp assistance_presets.hpp track_geometry.cpp track_geometry.hpp ui_verification.cpp showroom.cpp showroom.hpp showroom_verification.cpp live_cars.cpp live_cars.hpp gamepad.cpp gamepad.hpp)
target_link_libraries(fd_desktop PRIVATE fd_recording fd_race_audio Qt6::Core Qt6::Gui Qt6::Qml Qt6::Quick Qt6::QuickControls2 Qt6::Quick3D Qt6::Multimedia)
# A person drives with an Xbox controller through XInput, part of Windows (decision 0037).
if(WIN32)
  target_link_libraries(fd_desktop PRIVATE xinput)
endif()
target_compile_definitions(fd_desktop PRIVATE FD_SHOWROOM_MEDIA="${CMAKE_SOURCE_DIR}/artifacts/showroom/media"
  FD_LIVE_CARS="${CMAKE_SOURCE_DIR}/artifacts/live-cars"
  FD_KART_TRACK="${CMAKE_SOURCE_DIR}/tracks/gokartcentralen-goteborg.csv")
qt_add_resources(fd_desktop "qml" PREFIX "/" FILES qml/Main.qml qml/Car.qml qml/ShowroomView.qml qml/ShowroomPlayer.qml
  qml/shaders/scene.vert qml/shaders/ground.frag qml/shaders/overlay.frag)
qt_add_resources(fd_desktop "kart_guide" PREFIX "/kart" BASE "${CMAKE_SOURCE_DIR}/tracks"
  FILES "${CMAKE_SOURCE_DIR}/tracks/gokartcentralen-goteborg-guide.csv")
# The racing line needs fd_raceline, which is built only when OSQP's sources are present (decision 0020).
if(FD_HAVE_RACELINE)
  target_link_libraries(fd_desktop PRIVATE fd_raceline)
  target_compile_definitions(fd_desktop PRIVATE FD_HAVE_RACELINE)
endif()
# So does MPCC (decision 0024).
if(FD_HAVE_MPCC)
  target_link_libraries(fd_desktop PRIVATE fd_mpcc)
  target_compile_definitions(fd_desktop PRIVATE FD_HAVE_MPCC)
endif()
if(MSVC)
  target_compile_options(fd_desktop PRIVATE /W4 /utf-8)
endif()
if(BUILD_TESTING)
  qt_add_executable(fd_race_audio_tests race_audio_tests.cpp)
  target_link_libraries(fd_race_audio_tests PRIVATE fd_race_audio)
  add_test(NAME fd_race_audio COMMAND fd_race_audio_tests)
  set_tests_properties(fd_race_audio PROPERTIES TIMEOUT 30
    ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>")
  qt_add_executable(fd_assistance_presets_tests assistance_presets_tests.cpp assistance_presets.cpp)
  target_link_libraries(fd_assistance_presets_tests PRIVATE fd_core Qt6::Core)
  add_test(NAME fd_assistance_presets COMMAND fd_assistance_presets_tests)
  set_tests_properties(fd_assistance_presets PROPERTIES TIMEOUT 20
    ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>")
  qt_add_executable(fd_showroom_tests showroom_tests.cpp showroom.cpp showroom.hpp)
  target_link_libraries(fd_showroom_tests PRIVATE Qt6::Core)
  add_test(NAME fd_showroom_state COMMAND fd_showroom_tests)
  set_tests_properties(fd_showroom_state PROPERTIES TIMEOUT 20
    ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>")
  add_executable(fd_gamepad_tests gamepad_tests.cpp gamepad.cpp gamepad.hpp)
  target_link_libraries(fd_gamepad_tests PRIVATE fd_core)
  if(WIN32)
    target_link_libraries(fd_gamepad_tests PRIVATE xinput)
  endif()
  add_test(NAME fd_gamepad COMMAND fd_gamepad_tests)
  set_tests_properties(fd_gamepad PROPERTIES TIMEOUT 20)
  qt_add_executable(fd_live_cars_tests live_cars_tests.cpp live_cars.cpp live_cars.hpp)
  target_link_libraries(fd_live_cars_tests PRIVATE Qt6::Core)
  add_test(NAME fd_live_cars COMMAND fd_live_cars_tests)
  set_tests_properties(fd_live_cars PROPERTIES TIMEOUT 20
    ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>")
  get_filename_component(FD_QT_PREFIX "${Qt6_DIR}/../../.." ABSOLUTE)
  add_test(NAME fd_desktop_controls COMMAND fd_desktop --verify-ui "${CMAKE_BINARY_DIR}/ui-evidence")
  # 30 s until the setup controls added a rear-lock search on a fresh four-wheel run (decision 0016); 60 s until the racing
  # line checks added its solve and 8 s of the four-wheel car on the line, which took 52 s under a parallel ctest (decision 0020).
  # Shared host rendering can slow the UI integration suite; no simulation/control timing changes are made. 180 s until the
  # theoretical best lap's stage (decision 0034): 615 checks took 162 s alone and ran past 180 s beside offline solves.
  set_tests_properties(fd_desktop_controls PROPERTIES TIMEOUT 300
    ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>"
    ENVIRONMENT "QT_PLUGIN_PATH=${FD_QT_PREFIX}/plugins;QT_QPA_PLATFORM_PLUGIN_PATH=${FD_QT_PREFIX}/plugins/platforms;QML_IMPORT_PATH=${FD_QT_PREFIX}/qml")
endif()
