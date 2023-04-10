cmake_minimum_required (VERSION 3.16.2)

project (
    cadd-e
    VERSION 1.0
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
set(CMAKE_PREFIX_PATH "/opt/ros/foxy; ${CMAKE_MODULE_PATH}; ${CMAKE_MODULE_PATH}/cadd_e_interface/install; ${CMAKE_MODULE_PATH}/mpc/install")

find_package(matioCpp REQUIRED)

if(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS "-march=native -O3 -pthread -Wall -DQT_NO_VERSION_TAGGING")
    find_package(PythonLibs REQUIRED)

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(cadd_e_interface REQUIRED)
    find_package(interfaces REQUIRED)  # MPC interfaces.

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

    include_directories(${ament_cmake_INCLUDE_DIRS})
    include_directories(${cadd_e_interface_INCLUDE_DIRS})
    include_directories(${interfaces_INCLUDE_DIRS})

    include_directories(${PYTHON_INCLUDE_DIRS})
    include_directories("localization/frenet_optimal_trajectory_planner")
    include_directories("localization/frenet_optimal_trajectory_planner/src")
    include_directories("localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory")

    add_executable(main
        src/main.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/utils.h
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/QuarticPolynomial.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/QuarticPolynomial.h
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/QuinticPolynomial.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/QuinticPolynomial.h
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/CubicSpline1D.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/CubicSpline1D.h
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/CubicSpline2D.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/CubicSpline2D.h
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.h
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/fot_wrapper.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/Obstacle.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/Obstacle.h
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/Car.cpp
        src/localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/Car.h)

    # ament_target_dependencies(cadde rclcpp)
    # install(TARGETS cadde DESTINATION lib/${PROJECT_NAME})

    target_link_libraries(main PRIVATE ${cadd_e_interface_LIBRARIES} ${interfaces_LIBRARIES} ${ament_cmake_LIBRARIES} ${rclcpp_LIBRARIES} matioCpp::matioCpp Eigen3::Eigen Qt5::Gui Qt5::Widgets)

    ament_package()
endif(UNIX AND NOT APPLE)