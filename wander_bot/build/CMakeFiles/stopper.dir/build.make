# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/marat/catkin_ws/src/wander_bot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marat/catkin_ws/src/wander_bot/build

# Include any dependencies generated for this target.
include CMakeFiles/stopper.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stopper.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stopper.dir/flags.make

CMakeFiles/stopper.dir/src/Stopper.cpp.o: CMakeFiles/stopper.dir/flags.make
CMakeFiles/stopper.dir/src/Stopper.cpp.o: ../src/Stopper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/marat/catkin_ws/src/wander_bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stopper.dir/src/Stopper.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stopper.dir/src/Stopper.cpp.o -c /home/marat/catkin_ws/src/wander_bot/src/Stopper.cpp

CMakeFiles/stopper.dir/src/Stopper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stopper.dir/src/Stopper.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/marat/catkin_ws/src/wander_bot/src/Stopper.cpp > CMakeFiles/stopper.dir/src/Stopper.cpp.i

CMakeFiles/stopper.dir/src/Stopper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stopper.dir/src/Stopper.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/marat/catkin_ws/src/wander_bot/src/Stopper.cpp -o CMakeFiles/stopper.dir/src/Stopper.cpp.s

CMakeFiles/stopper.dir/src/run_stopper.cpp.o: CMakeFiles/stopper.dir/flags.make
CMakeFiles/stopper.dir/src/run_stopper.cpp.o: ../src/run_stopper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/marat/catkin_ws/src/wander_bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/stopper.dir/src/run_stopper.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stopper.dir/src/run_stopper.cpp.o -c /home/marat/catkin_ws/src/wander_bot/src/run_stopper.cpp

CMakeFiles/stopper.dir/src/run_stopper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stopper.dir/src/run_stopper.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/marat/catkin_ws/src/wander_bot/src/run_stopper.cpp > CMakeFiles/stopper.dir/src/run_stopper.cpp.i

CMakeFiles/stopper.dir/src/run_stopper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stopper.dir/src/run_stopper.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/marat/catkin_ws/src/wander_bot/src/run_stopper.cpp -o CMakeFiles/stopper.dir/src/run_stopper.cpp.s

# Object files for target stopper
stopper_OBJECTS = \
"CMakeFiles/stopper.dir/src/Stopper.cpp.o" \
"CMakeFiles/stopper.dir/src/run_stopper.cpp.o"

# External object files for target stopper
stopper_EXTERNAL_OBJECTS =

devel/lib/wander_bot/stopper: CMakeFiles/stopper.dir/src/Stopper.cpp.o
devel/lib/wander_bot/stopper: CMakeFiles/stopper.dir/src/run_stopper.cpp.o
devel/lib/wander_bot/stopper: CMakeFiles/stopper.dir/build.make
devel/lib/wander_bot/stopper: /opt/ros/noetic/lib/libroscpp.so
devel/lib/wander_bot/stopper: /usr/lib/aarch64-linux-gnu/libpthread.so
devel/lib/wander_bot/stopper: /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
devel/lib/wander_bot/stopper: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
devel/lib/wander_bot/stopper: /opt/ros/noetic/lib/librosconsole.so
devel/lib/wander_bot/stopper: /opt/ros/noetic/lib/librosconsole_log4cxx.so
devel/lib/wander_bot/stopper: /opt/ros/noetic/lib/librosconsole_backend_interface.so
devel/lib/wander_bot/stopper: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
devel/lib/wander_bot/stopper: /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
devel/lib/wander_bot/stopper: /opt/ros/noetic/lib/libxmlrpcpp.so
devel/lib/wander_bot/stopper: /opt/ros/noetic/lib/libroscpp_serialization.so
devel/lib/wander_bot/stopper: /opt/ros/noetic/lib/librostime.so
devel/lib/wander_bot/stopper: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
devel/lib/wander_bot/stopper: /opt/ros/noetic/lib/libcpp_common.so
devel/lib/wander_bot/stopper: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
devel/lib/wander_bot/stopper: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
devel/lib/wander_bot/stopper: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/wander_bot/stopper: CMakeFiles/stopper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/marat/catkin_ws/src/wander_bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable devel/lib/wander_bot/stopper"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stopper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stopper.dir/build: devel/lib/wander_bot/stopper

.PHONY : CMakeFiles/stopper.dir/build

CMakeFiles/stopper.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stopper.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stopper.dir/clean

CMakeFiles/stopper.dir/depend:
	cd /home/marat/catkin_ws/src/wander_bot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marat/catkin_ws/src/wander_bot /home/marat/catkin_ws/src/wander_bot /home/marat/catkin_ws/src/wander_bot/build /home/marat/catkin_ws/src/wander_bot/build /home/marat/catkin_ws/src/wander_bot/build/CMakeFiles/stopper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stopper.dir/depend

