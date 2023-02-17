cmake_minimum_required (VERSION 3.22.2)

project (
    LCC_MAP_TEST
    VERSION 1.0
    LANGUAGES CXX)

find_package(matioCpp REQUIRED)

#add_executable(readMLTest readMapMatTest.cpp)
add_executable(mapInit routePlanner.cpp)

#target_link_libraries(readMLTest PRIVATE matioCpp::matioCpp)
target_link_libraries(mapInit PRIVATE matioCpp::matioCpp)