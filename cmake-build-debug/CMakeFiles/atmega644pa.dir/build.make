# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/ra/Documents/coding/smart-trains/atmega644pa

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/ra/Documents/coding/smart-trains/atmega644pa/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/atmega644pa.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/atmega644pa.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/atmega644pa.dir/flags.make

CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o: CMakeFiles/atmega644pa.dir/flags.make
CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o: ../ASF/common/boards/user_board/init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ra/Documents/coding/smart-trains/atmega644pa/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o   -c /Users/ra/Documents/coding/smart-trains/atmega644pa/ASF/common/boards/user_board/init.c

CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.i"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/ra/Documents/coding/smart-trains/atmega644pa/ASF/common/boards/user_board/init.c > CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.i

CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.s"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/ra/Documents/coding/smart-trains/atmega644pa/ASF/common/boards/user_board/init.c -o CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.s

CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o.requires:

.PHONY : CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o.requires

CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o.provides: CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o.requires
	$(MAKE) -f CMakeFiles/atmega644pa.dir/build.make CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o.provides.build
.PHONY : CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o.provides

CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o.provides.build: CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o


CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o: CMakeFiles/atmega644pa.dir/flags.make
CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o: ../ASF/common/services/clock/mega/sysclk.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ra/Documents/coding/smart-trains/atmega644pa/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o   -c /Users/ra/Documents/coding/smart-trains/atmega644pa/ASF/common/services/clock/mega/sysclk.c

CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.i"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/ra/Documents/coding/smart-trains/atmega644pa/ASF/common/services/clock/mega/sysclk.c > CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.i

CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.s"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/ra/Documents/coding/smart-trains/atmega644pa/ASF/common/services/clock/mega/sysclk.c -o CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.s

CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o.requires:

.PHONY : CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o.requires

CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o.provides: CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o.requires
	$(MAKE) -f CMakeFiles/atmega644pa.dir/build.make CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o.provides.build
.PHONY : CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o.provides

CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o.provides.build: CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o


CMakeFiles/atmega644pa.dir/main.c.o: CMakeFiles/atmega644pa.dir/flags.make
CMakeFiles/atmega644pa.dir/main.c.o: ../main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ra/Documents/coding/smart-trains/atmega644pa/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/atmega644pa.dir/main.c.o"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/atmega644pa.dir/main.c.o   -c /Users/ra/Documents/coding/smart-trains/atmega644pa/main.c

CMakeFiles/atmega644pa.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/atmega644pa.dir/main.c.i"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/ra/Documents/coding/smart-trains/atmega644pa/main.c > CMakeFiles/atmega644pa.dir/main.c.i

CMakeFiles/atmega644pa.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/atmega644pa.dir/main.c.s"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/ra/Documents/coding/smart-trains/atmega644pa/main.c -o CMakeFiles/atmega644pa.dir/main.c.s

CMakeFiles/atmega644pa.dir/main.c.o.requires:

.PHONY : CMakeFiles/atmega644pa.dir/main.c.o.requires

CMakeFiles/atmega644pa.dir/main.c.o.provides: CMakeFiles/atmega644pa.dir/main.c.o.requires
	$(MAKE) -f CMakeFiles/atmega644pa.dir/build.make CMakeFiles/atmega644pa.dir/main.c.o.provides.build
.PHONY : CMakeFiles/atmega644pa.dir/main.c.o.provides

CMakeFiles/atmega644pa.dir/main.c.o.provides.build: CMakeFiles/atmega644pa.dir/main.c.o


# Object files for target atmega644pa
atmega644pa_OBJECTS = \
"CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o" \
"CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o" \
"CMakeFiles/atmega644pa.dir/main.c.o"

# External object files for target atmega644pa
atmega644pa_EXTERNAL_OBJECTS =

atmega644pa: CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o
atmega644pa: CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o
atmega644pa: CMakeFiles/atmega644pa.dir/main.c.o
atmega644pa: CMakeFiles/atmega644pa.dir/build.make
atmega644pa: CMakeFiles/atmega644pa.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/ra/Documents/coding/smart-trains/atmega644pa/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking C executable atmega644pa"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/atmega644pa.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/atmega644pa.dir/build: atmega644pa

.PHONY : CMakeFiles/atmega644pa.dir/build

CMakeFiles/atmega644pa.dir/requires: CMakeFiles/atmega644pa.dir/ASF/common/boards/user_board/init.c.o.requires
CMakeFiles/atmega644pa.dir/requires: CMakeFiles/atmega644pa.dir/ASF/common/services/clock/mega/sysclk.c.o.requires
CMakeFiles/atmega644pa.dir/requires: CMakeFiles/atmega644pa.dir/main.c.o.requires

.PHONY : CMakeFiles/atmega644pa.dir/requires

CMakeFiles/atmega644pa.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/atmega644pa.dir/cmake_clean.cmake
.PHONY : CMakeFiles/atmega644pa.dir/clean

CMakeFiles/atmega644pa.dir/depend:
	cd /Users/ra/Documents/coding/smart-trains/atmega644pa/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/ra/Documents/coding/smart-trains/atmega644pa /Users/ra/Documents/coding/smart-trains/atmega644pa /Users/ra/Documents/coding/smart-trains/atmega644pa/cmake-build-debug /Users/ra/Documents/coding/smart-trains/atmega644pa/cmake-build-debug /Users/ra/Documents/coding/smart-trains/atmega644pa/cmake-build-debug/CMakeFiles/atmega644pa.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/atmega644pa.dir/depend
