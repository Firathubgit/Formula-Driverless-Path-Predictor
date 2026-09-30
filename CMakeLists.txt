cmake_minimum_required(VERSION 3.24)
project(FormulaDriverless VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(FD_BUILD_DESKTOP "Build the native Qt Quick 3D application" OFF)
include(CTest)
add_subdirectory(core)
add_subdirectory(adapters/simulation)
add_subdirectory(adapters/recording)
# The instruments a car reads itself with: rates, dead-time queues and seeded Gaussian error (decision 0029).
add_subdirectory(adapters/sensors)
# fd_raceline solves the minimum-curvature racing line with OSQP (decision 0020). scripts/bootstrap-osqp.ps1 places the
# pinned, checksum-verified OSQP and qdldl sources in .tools/osqp; without them fd_raceline, its tests and the headless
# --line racing option are left out and everything else builds as before.
set(FD_OSQP_SOURCE_DIR "${CMAKE_SOURCE_DIR}/.tools/osqp/v1.0.0" CACHE PATH "OSQP v1.0.0 source")
set(FD_QDLDL_SOURCE_DIR "${CMAKE_SOURCE_DIR}/.tools/osqp/qdldl-v0.1.8" CACHE PATH "qdldl v0.1.8 source used by OSQP")
if(EXISTS "${FD_OSQP_SOURCE_DIR}/CMakeLists.txt" AND EXISTS "${FD_QDLDL_SOURCE_DIR}/CMakeLists.txt")
  enable_language(C)
  # OSQP's own CMake would clone qdldl at configure time; point it at the verified local copy instead.
  set(FETCHCONTENT_SOURCE_DIR_QDLDL "${FD_QDLDL_SOURCE_DIR}" CACHE PATH "qdldl source used by OSQP" FORCE)
  foreach(fd_osqp_off OSQP_BUILD_SHARED_LIB OSQP_BUILD_DEMO_EXE OSQP_BUILD_UNITTESTS OSQP_ENABLE_PRINTING
          OSQP_ENABLE_PROFILING OSQP_ENABLE_INTERRUPT)
    set(${fd_osqp_off} OFF CACHE BOOL "Set by FormulaDriverless for fd_raceline" FORCE)
  endforeach()
  set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "Set by FormulaDriverless for fd_raceline" FORCE)
  add_subdirectory("${FD_OSQP_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/third_party/osqp" EXCLUDE_FROM_ALL)
  add_subdirectory(raceline)
  set(FD_HAVE_RACELINE ON)
  # Model predictive contouring control solves its quadratic programmes with the same OSQP (decision 0024).
  add_subdirectory(mpcc)
  set(FD_HAVE_MPCC ON)
else()
  set(FD_HAVE_RACELINE OFF)
  set(FD_HAVE_MPCC OFF)
  message(STATUS "OSQP sources not found in .tools/osqp: fd_raceline and fd_mpcc are skipped; run scripts/bootstrap-osqp.ps1 to build them")
endif()
add_subdirectory(apps/headless)
# Record the algorithm and recording source identity even before the first Git commit.
file(GLOB_RECURSE FD_RECORDED_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/core/*.cpp" "${CMAKE_SOURCE_DIR}/core/*.hpp"
  "${CMAKE_SOURCE_DIR}/adapters/simulation/*.cpp" "${CMAKE_SOURCE_DIR}/adapters/simulation/*.hpp"
  "${CMAKE_SOURCE_DIR}/adapters/recording/*.cpp" "${CMAKE_SOURCE_DIR}/adapters/recording/*.hpp"
  "${CMAKE_SOURCE_DIR}/adapters/sensors/*.cpp" "${CMAKE_SOURCE_DIR}/adapters/sensors/*.hpp"
  "${CMAKE_SOURCE_DIR}/raceline/*.cpp" "${CMAKE_SOURCE_DIR}/raceline/*.hpp"
  "${CMAKE_SOURCE_DIR}/mpcc/*.cpp" "${CMAKE_SOURCE_DIR}/mpcc/*.hpp"
  "${CMAKE_SOURCE_DIR}/apps/headless/*.cpp")
list(SORT FD_RECORDED_SOURCES)
set(FD_SOURCE_DIGESTS "")
foreach(FD_SOURCE IN LISTS FD_RECORDED_SOURCES)
  file(SHA256 "${FD_SOURCE}" FD_DIGEST)
  file(RELATIVE_PATH FD_RELATIVE_SOURCE "${CMAKE_SOURCE_DIR}" "${FD_SOURCE}")
  string(APPEND FD_SOURCE_DIGESTS "${FD_RELATIVE_SOURCE}:${FD_DIGEST}\n")
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${FD_RECORDED_SOURCES})
string(SHA256 FD_SOURCE_FINGERPRINT "${FD_SOURCE_DIGESTS}")
target_compile_definitions(fd_headless PRIVATE
  FD_SOURCE_FINGERPRINT="${FD_SOURCE_FINGERPRINT}"
  FD_COMPILER_ID="${CMAKE_CXX_COMPILER_ID}"
  FD_COMPILER_VERSION="${CMAKE_CXX_COMPILER_VERSION}")
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
if(FD_BUILD_DESKTOP)
  find_package(Qt6 6.8 REQUIRED COMPONENTS Core Gui Qml Quick QuickControls2 Quick3D)
  qt_standard_project_setup(REQUIRES 6.8)
  add_subdirectory(apps/desktop)
endif()
