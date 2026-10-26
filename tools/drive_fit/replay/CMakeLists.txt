# Host build of the drive-profile replay against the espp cross-platform library,
# so it runs the same TrajectoryPlanner, task, and timer code the firmware uses.
# check_replay.py configures and builds this; see ../README.md.
cmake_minimum_required(VERSION 3.21)
project(drive_profile_replay LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CPM_DOWNLOAD_VERSION 0.43.2)
set(CPM_DOWNLOAD_LOCATION "${CMAKE_BINARY_DIR}/cmake/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
if(NOT EXISTS "${CPM_DOWNLOAD_LOCATION}")
  file(DOWNLOAD
    "https://github.com/cpm-cmake/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake"
    "${CPM_DOWNLOAD_LOCATION}"
    EXPECTED_HASH SHA256=49a3bef91ceb65bb66d57255e12d1ffd22abc2f6408fa9fe4534c544a2f232aa)
endif()
include("${CPM_DOWNLOAD_LOCATION}")

# Keep in step with espp/trajectory_planner in the firmware's dependencies.lock.
set(ESPP_VERSION "v1.3.10" CACHE STRING "espp git tag to replay against")
CPMAddPackage(
  NAME espp
  GITHUB_REPOSITORY esp-cpp/espp
  GIT_TAG ${ESPP_VERSION}
  SOURCE_SUBDIR lib)

set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
add_executable(replay_drive_profile replay_drive_profile.cpp)
target_include_directories(replay_drive_profile PRIVATE
  "${REPO_ROOT}/components/DriveController/include"
  "${REPO_ROOT}/components/MIBConfig/include")
target_link_libraries(replay_drive_profile PRIVATE espp::espp)
