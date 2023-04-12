# CMakeList.txt : CMake project for PathPlannerComp2_0, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("PathPlannerComp2_0")

# Add source to this project's executable.
add_executable (PathPlannerComp2_0 
"PathPlannerComp2_0.cpp" 
"PathPlannerComp2_0.h"
"Source Files/RenderClass.cpp"
"Source Files/VAO.cpp"
"Source Files/VBO.cpp"
"Source Files/EBO.cpp"
"Source Files/shaderClass.cpp"
"Source Files/Map2D.cpp"
"Source Files/elliptical_approx.cpp"
"Source Files/Algorithms/a_star.cpp"
"Source Files/Algorithms/genetic.cpp"
"Source Files/Algorithms/voronoi.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET PathPlannerComp2_0 PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
# this is heuristically generated, and may not be correct
find_package(cpr CONFIG REQUIRED)
target_link_libraries(PathPlannerComp2_0 PRIVATE cpr::cpr)

    # this is heuristically generated, and may not be correct
    find_package(glad CONFIG REQUIRED)
    target_link_libraries(PathPlannerComp2_0 PRIVATE glad::glad)

    # this is heuristically generated, and may not be correct
    find_package(glfw3 CONFIG REQUIRED)
    target_link_libraries(PathPlannerComp2_0 PRIVATE glfw)

    find_package(dlib CONFIG REQUIRED)
    target_link_libraries(PathPlannerComp2_0 PRIVATE dlib::dlib)

    find_package(Eigen3 CONFIG REQUIRED)
    target_link_libraries(PathPlannerComp2_0 PRIVATE Eigen3::Eigen)

set(OpenCV_DIR "${VCPKG_INSTALLED_DIR}/x64-windows/share/opencv4")
    find_package(OpenCV REQUIRED)

    find_package(Stb REQUIRED)
    target_include_directories(PathPlannerComp2_0 PRIVATE ${Stb_INCLUDE_DIR})