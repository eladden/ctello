cmake_minimum_required(VERSION 3.10)

project(CTello)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -march=native -Wno-deprecated-declarations -Wno-unused-result  -Wno-sign-compare")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g3 -O")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3")

find_package(spdlog REQUIRED)
find_package(OpenCV 4.0)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

set(ORB_SLAM2_DIR /media/eladden/DATA/orbslamhome/ORB_SLAM2)
set(ORB_SLAM2_INCLUDE_DIR /media/eladden/DATA/orbslamhome/ORB_SLAM2/include)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
message("Pangolin dir: " ${Pangolin_DIR})
message("Pangolin include dirs: " ${Pangolin_INCLUDE_DIRS})

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${ORB_SLAM2_DIR}
        ${ORB_SLAM2_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(ORB_SLAM2 SHARED
        ${PROJECT_SOURCE_DIR}/src/ctello_ORB_support.cpp
        ${ORB_SLAM2_DIR}/src/System.cc
        ${ORB_SLAM2_DIR}/src/Tracking.cc
        ${ORB_SLAM2_DIR}/src/LocalMapping.cc
        ${ORB_SLAM2_DIR}/src/LoopClosing.cc
        ${ORB_SLAM2_DIR}/src/ORBextractor.cc
        ${ORB_SLAM2_DIR}/src/ORBmatcher.cc
        ${ORB_SLAM2_DIR}/src/FrameDrawer.cc
        ${ORB_SLAM2_DIR}/src/Converter.cc
        ${ORB_SLAM2_DIR}/src/MapPoint.cc
        ${ORB_SLAM2_DIR}/src/KeyFrame.cc
        ${ORB_SLAM2_DIR}/src/Map.cc
        ${ORB_SLAM2_DIR}/src/MapDrawer.cc
        ${ORB_SLAM2_DIR}/src/Optimizer.cc
        ${ORB_SLAM2_DIR}/src/PnPsolver.cc
        ${ORB_SLAM2_DIR}/src/Frame.cc
        ${ORB_SLAM2_DIR}/src/KeyFrameDatabase.cc
        ${ORB_SLAM2_DIR}/src/Sim3Solver.cc
        ${ORB_SLAM2_DIR}/src/Initializer.cc
        ${ORB_SLAM2_DIR}/src/Viewer.cc
        )

target_link_libraries(ORB_SLAM2
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${ORB_SLAM2_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${ORB_SLAM2_DIR}/Thirdparty/g2o/lib/libg2o.so
        )

# CTello Shared Library =======================================================

add_library(ctello SHARED src/ctello.cpp)

target_include_directories(ctello PRIVATE include)

target_link_libraries(ctello PRIVATE spdlog::spdlog)

install(TARGETS ctello DESTINATION lib)
install(FILES include/ctello.h DESTINATION include)

# CTello Command ==============================================================

add_executable(ctello-command src/ctello_command.cpp)

target_include_directories(ctello-command PRIVATE include)

target_link_libraries(ctello-command ctello ORB_SLAM2)

install(TARGETS ctello-command DESTINATION bin)

# CTello State ================================================================

add_executable(ctello-state src/ctello_state.cpp)

target_include_directories(ctello-state PRIVATE include)

target_link_libraries(ctello-state ctello)

install(TARGETS ctello-state DESTINATION bin)

# CTello Stream ===============================================================

add_executable(ctello-stream src/ctello_stream.cpp)

target_include_directories(ctello-stream PRIVATE include)

target_link_libraries(ctello-stream ctello)
target_link_libraries(ctello-stream ${OpenCV_LIBS})

install(TARGETS ctello-stream DESTINATION bin)


# CTello Examples =============================================================

## Scan Room --------------------------------------------------------------------
add_executable(scan_room examples/scan_room.cpp)

target_include_directories(scan_room PRIVATE include)

target_link_libraries(scan_room ctello ${Pangolin_LIBRARIES})
target_link_libraries(scan_room ${OpenCV_LIBS} ORB_SLAM2)

## Flip -----------------------------------------------------------------------
add_executable(flip examples/flip.cpp)

target_include_directories(flip PRIVATE include)

target_link_libraries(flip ctello)
target_link_libraries(flip ${OpenCV_LIBS})

## Follow ---------------------------------------------------------------------
add_executable(follow examples/follow.cpp)

target_include_directories(follow PRIVATE include)

target_link_libraries(follow ctello)
target_link_libraries(follow ${OpenCV_LIBS})
