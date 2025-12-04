###########################################################################
# CMakeLists.txt generated for component longitudinal_mpc
# Product type: executable
###########################################################################
cmake_minimum_required(VERSION 3.12)
project(longitudinal_mpc)

# Propagate the CMAKE_EXPORT_COMPILE_COMMANDS variable from the 
# environment if it is defined as an environment variable, but not as a 
# CMake variable. This is to work around a bug in CMake 3.19 when the 
# "NMake Makefiles" generator is selected.
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()


# Set directory properties.
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT longitudinal_mpc)

###########################################################################
## Path variables
###########################################################################
# Derive an absolute path to the code generation anchor folder.
get_filename_component(START_DIR .. ABSOLUTE)

# Special directories defined by using CACHE variables can be overridden 
# by setting the variable from the command line, e.g.,
# 
# 	cmake . -DMATLAB_ROOT=/path/to/another/matlab/root
set(MATLAB_ROOT C:/Program\ Files/MATLAB/R2024a CACHE PATH "")

# Additional variables that are defined conditionally.
if("${CMAKE_CURRENT_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    set(BINARY_START_DIR "${START_DIR}")
else()
    set(BINARY_START_DIR "${CMAKE_BINARY_DIR}")
endif()

###########################################################################
## System Libraries
###########################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES} ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

###########################################################################
## Target definition and commands
###########################################################################

# Definition of target "longitudinal_mpc".
add_executable(longitudinal_mpc  ${START_DIR}/longitudinal_mpc_grt_rtw/longitudinal_mpc.cpp
    ${START_DIR}/longitudinal_mpc_grt_rtw/rtGetInf.cpp
    ${START_DIR}/longitudinal_mpc_grt_rtw/rtGetNaN.cpp
    ${START_DIR}/longitudinal_mpc_grt_rtw/rt_nonfinite.cpp
    ${MATLAB_ROOT}/rtw/c/src/common/rt_cppclass_main.cpp)

# Set properties for target "longitudinal_mpc".
set_target_properties(longitudinal_mpc PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON
    RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>"
    LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>"
    ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>")

# Specify language features required for target "longitudinal_mpc".
target_compile_features(longitudinal_mpc PUBLIC cxx_std_11)

# Specify compiler preprocessor definitions for target "longitudinal_mpc".
target_compile_definitions(longitudinal_mpc PRIVATE -DMODEL=longitudinal_mpc
    -DNUMST=1
    -DNCSTATES=0
    -DHAVESTDIO
    -DRT
    -DUSE_RTMODEL
    -DCLASSIC_INTERFACE=0
    -DALLOCATIONFCN=0
    -DTID01EQ=0
    -DMAT_FILE=0
    -DONESTEPFCN=1
    -DTERMFCN=1
    -DMULTI_INSTANCE_CODE=1
    -DINTEGER_CODE=0
    -DMT=0)

# Specify include directories for target "longitudinal_mpc".
target_include_directories(longitudinal_mpc PRIVATE ${START_DIR}
    ${START_DIR}/longitudinal_mpc_grt_rtw
    ${MATLAB_ROOT}/extern/include
    ${MATLAB_ROOT}/simulink/include
    ${MATLAB_ROOT}/rtw/c/src
    ${MATLAB_ROOT}/rtw/c/src/ext_mode/common)

# Specify library link dependencies for target "longitudinal_mpc". CMake 
# generator expressions are used to create a CMakeLists.txt file that 
# supports multiple platforms with differently named system library 
# dependencies.
target_link_libraries(longitudinal_mpc PRIVATE $<$<BOOL:${FOUND_LIBM}>:m>
    $<$<BOOL:${WIN32}>:kernel32>
    $<$<BOOL:${WIN32}>:ws2_32>
    $<$<BOOL:${WIN32}>:mswsock>
    $<$<BOOL:${WIN32}>:advapi32>)

# Extract DWARF debug symbols into a separate file when building 
# executable programs for Apple platforms.
if(APPLE AND (CMAKE_C_COMPILER_ID STREQUAL "AppleClang"))
    string(APPEND _dsymutilcmd "$<IF:$<CONFIG:DEBUG,RELWITHDEBINFO>,"
                "xcrun;dsymutil;$<TARGET_FILE:longitudinal_mpc>;--flat,"
                "true"
        >)
    add_custom_command(TARGET longitudinal_mpc
            POST_BUILD
            COMMAND "${_dsymutilcmd}"
            COMMAND_EXPAND_LISTS
            BYPRODUCTS longitudinal_mpc.dwarf)
endif()


###########################################################################
## Build success message
###########################################################################
add_custom_command(TARGET longitudinal_mpc POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created executable: $<TARGET_FILE:longitudinal_mpc>")

###########################################################################
## Call toolchain hook function if defined
###########################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(longitudinal_mpc)
endif()
