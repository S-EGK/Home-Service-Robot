# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/segk/home_service_robot/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/segk/home_service_robot/build

# Include any dependencies generated for this target.
include add_markers/CMakeFiles/add_markers_comb.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include add_markers/CMakeFiles/add_markers_comb.dir/compiler_depend.make

# Include the progress variables for this target.
include add_markers/CMakeFiles/add_markers_comb.dir/progress.make

# Include the compile flags for this target's objects.
include add_markers/CMakeFiles/add_markers_comb.dir/flags.make

add_markers/CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.o: add_markers/CMakeFiles/add_markers_comb.dir/flags.make
add_markers/CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.o: /home/segk/home_service_robot/src/add_markers/src/add_markers_comb.cpp
add_markers/CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.o: add_markers/CMakeFiles/add_markers_comb.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/segk/home_service_robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object add_markers/CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.o"
	cd /home/segk/home_service_robot/build/add_markers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT add_markers/CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.o -MF CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.o.d -o CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.o -c /home/segk/home_service_robot/src/add_markers/src/add_markers_comb.cpp

add_markers/CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.i"
	cd /home/segk/home_service_robot/build/add_markers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/segk/home_service_robot/src/add_markers/src/add_markers_comb.cpp > CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.i

add_markers/CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.s"
	cd /home/segk/home_service_robot/build/add_markers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/segk/home_service_robot/src/add_markers/src/add_markers_comb.cpp -o CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.s

# Object files for target add_markers_comb
add_markers_comb_OBJECTS = \
"CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.o"

# External object files for target add_markers_comb
add_markers_comb_EXTERNAL_OBJECTS =

/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: add_markers/CMakeFiles/add_markers_comb.dir/src/add_markers_comb.cpp.o
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: add_markers/CMakeFiles/add_markers_comb.dir/build.make
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /opt/ros/kinetic/lib/libroscpp.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /opt/ros/kinetic/lib/librosconsole.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /opt/ros/kinetic/lib/librostime.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /opt/ros/kinetic/lib/libcpp_common.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb: add_markers/CMakeFiles/add_markers_comb.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/segk/home_service_robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb"
	cd /home/segk/home_service_robot/build/add_markers && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/add_markers_comb.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
add_markers/CMakeFiles/add_markers_comb.dir/build: /home/segk/home_service_robot/devel/lib/add_markers/add_markers_comb
.PHONY : add_markers/CMakeFiles/add_markers_comb.dir/build

add_markers/CMakeFiles/add_markers_comb.dir/clean:
	cd /home/segk/home_service_robot/build/add_markers && $(CMAKE_COMMAND) -P CMakeFiles/add_markers_comb.dir/cmake_clean.cmake
.PHONY : add_markers/CMakeFiles/add_markers_comb.dir/clean

add_markers/CMakeFiles/add_markers_comb.dir/depend:
	cd /home/segk/home_service_robot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/segk/home_service_robot/src /home/segk/home_service_robot/src/add_markers /home/segk/home_service_robot/build /home/segk/home_service_robot/build/add_markers /home/segk/home_service_robot/build/add_markers/CMakeFiles/add_markers_comb.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : add_markers/CMakeFiles/add_markers_comb.dir/depend

