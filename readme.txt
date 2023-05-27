** INTRO **
Path planning is one of the most crucial technologies that make a robotic agent take decisions autonomously. It is a way of finding the best suitable paths for a robotic agent. The simplest problem involves finding the path for a point object in the obstacle course. The more involved ones deal with finding the orientations and actions that the agent should perform to reach its goal pose.

This project work involves designing a new data structure to represent nodes and obstacles, and building graphs to handle obstacles in the cluttered environment more efficiently. The aim is to exploit the concepts of Computational Geometry to handle the geometry of obstacles more effectively, and at the same time use elliptical approximations to address the problem of higher Complexity of Geometry based path planning algorithms.

** ENVIRONMENT DETAILS **
 - Visual Studio 2022 Project
 - CMake Package Manager -- binds and builds project files
 - vcpkg Dependency Manager -- installs required packages
 - C++ compiler

** INSTALLATION **
* To add any new library to package:
add the library name in vcpkg_rf.txt file and run this code in command line:
in command prompt, navigate to project folder and run following command(vcpkg_rf.txt should be in the same directory):
 vcpkg "@.\vcpkg_rf.txt"
// https://www.youtube.com/watch?v=FeBzSYiWkEU
--> It will prompt to add some lines of code to CMakeLists.txt; at the end of installation
--> Copy those lines; repllace "main" with your Project name
--> Update the CMake TOOLCHAIN_PATH in CMakePrests.json according to your install location:
this as : 
            "CMAKE_TOOLCHAIN_FILE": "C:/Users/shrey/Documents/vcpkg-master/vcpkg-master/scripts/buildsystems/vcpkg.cmake"


