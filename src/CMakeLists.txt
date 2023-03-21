cmake_minimum_required (VERSION 3.16.2)

project (
    LCC_MAP_TEST
    VERSION 1.0
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

find_package(matioCpp REQUIRED)

if(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS "-march=native -O3 -pthread -Wall -DQT_NO_VERSION_TAGGING")
    find_package(PythonLibs REQUIRED)

    find_package(Qt5 COMPONENTS Widgets REQUIRED)
    find_package(Qt5 COMPONENTS Core REQUIRED)
    find_package(Qt5 COMPONENTS Gui REQUIRED)
    find_package(Eigen3)

    if(Eigen3_FOUND)
        set(CUSTOM_EIGEN ${EIGEN3_INCLUDE_DIR})
    else()
        find_package(Eigen)
        set(CUSTOM_EIGEN ${EIGEN_INCLUDE_DIR})
    endif()

    include_directories(.)
    include_directories(
            ${CUSTOM_EIGEN}
    )

    include_directories(${PYTHON_INCLUDE_DIRS})
    include_directories("localization/frenet_optimal_trajectory_planner")
    include_directories("localization/frenet_optimal_trajectory_planner/src")
    include_directories("localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory")

    add_executable(main
            main.cpp
            localization/frenet_optimal_trajectory_planner/src/utils.h
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/QuarticPolynomial.cpp
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/QuarticPolynomial.h
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/QuinticPolynomial.cpp
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/QuinticPolynomial.h
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/CubicSpline1D.cpp
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/CubicSpline1D.h
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/CubicSpline2D.cpp
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/CubicSpline2D.h
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.cpp
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.h
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.cpp
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/fot_wrapper.cpp
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/Obstacle.cpp
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/Obstacle.h
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/Car.cpp
            localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/Car.h)

            target_link_libraries(main PRIVATE matioCpp::matioCpp Eigen3::Eigen Qt5::Gui Qt5::Widgets)
endif(UNIX AND NOT APPLE)

# add_executable(mapInit routePlanner.cpp)
# target_link_libraries(mapInit PRIVATE matioCpp::matioCpp)
# if(UNIX AND NOT APPLE)
#     target_link_libraries(mapInit PRIVATE ${PYTHON_LIBRARIES})
# endif(UNIX AND NOT APPLE)