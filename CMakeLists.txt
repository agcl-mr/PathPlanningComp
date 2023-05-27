# CMakeList.txt : CMake project for PathPlanning, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("PathPlanning")

# Add source to this project's executable.
add_executable (PathPlanning 
"PathPlanning.cpp"
"PathPlanning.h"
"Source Files/RenderClass.cpp"
"Source Files/mcc.cpp"
"Source Files/vao.cpp"
"Source Files/vbo.cpp"
"Source Files/ebo.cpp"
"Source Files/Map2D.cpp"
"Source Files/ShaderClass.cpp"
"Source Files/pre_built_algo.cpp"
"Source Files/elliptical_approx.cpp"
"Source Files/Algorithms/a_star.cpp"
"Source Files/Algorithms/voronoi.cpp"
"Source Files/Algorithms/genetic.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET PathPlanning PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.

    # this is heuristically generated, and may not be correct
    find_package(glad CONFIG REQUIRED)
    target_link_libraries(PathPlanning PRIVATE glad::glad)

    # this is heuristically generated, and may not be correct
    find_package(glfw3 CONFIG REQUIRED)
    target_link_libraries(PathPlanning PRIVATE glfw)

    # this is heuristically generated, and may not be correct
    find_package(ompl CONFIG REQUIRED)
    target_link_libraries(PathPlanning PRIVATE ompl)

    # this is heuristically generated, and may not be correct
    find_package(dlib CONFIG REQUIRED)
    target_link_libraries(PathPlanning PRIVATE dlib::dlib)

    # this is heuristically generated, and may not be correct
    find_package(Eigen3 CONFIG REQUIRED)
    target_link_libraries(PathPlanning PRIVATE Eigen3::Eigen)

    find_package(Stb REQUIRED)
    target_include_directories(PathPlanning PRIVATE ${Stb_INCLUDE_DIR})