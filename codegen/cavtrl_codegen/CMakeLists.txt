## Helper functions
macro(PRINT_ALL_VARIABLES)
  message(STATUS "print_all_variables------------------------------------------{")
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
  message(STATUS "print_all_variables------------------------------------------}")
endmacro()

MACRO(GET_HEADER_DIRECTORIES return_list starting_dir) # https://stackoverflow.com/questions/17653738/recursive-cmake-search-for-header-and-source-files
  FILE(GLOB_RECURSE new_list ${starting_dir}/*.h)
  SET(dir_list "")
  FOREACH(file_path ${new_list})
    GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
    SET(dir_list ${dir_list} ${dir_path})
  ENDFOREACH()
  LIST(REMOVE_DUPLICATES dir_list)
  SET(${return_list} ${dir_list})
ENDMACRO()

##############
## Settings ##
##############

cmake_minimum_required(VERSION 3.5)
project(cav_ctrl VERSION 1.0 DESCRIPTION "Example C API bindings for Matlab Coder model")

####################
## Build 1 Option ##
####################

## Handle the autogen codes here

# Set the name of the top-level directory that should contain subdirectories like 
#  src
#  include
#  CAV_mdl <- path to this one
#  |> ...

# Special directories defined by using CACHE variables can be overridden 
# by setting the variable from the command line, e.g.,
# 
# 	cmake . -DAUTOGEN_DIR=/path/to/another

# Set autogen sources and headers
set(AUTOGEN_DIR . CACHE PATH "")

GET_HEADER_DIRECTORIES(autogen_header_dirs ${AUTOGEN_DIR}) # Recursive search for the subdirectories that have header files starting from <arg2>

file(GLOB_RECURSE autogen_sources 
  ${AUTOGEN_DIR}/CAV_mdl/*.cpp 
  ${AUTOGEN_DIR}/CAV_mdl/*.c) # Don't want to find any source files in the Matlab R202x folders

# Status update
list(LENGTH autogen_header_dirs autogen_header_dirs_length)
message(STATUS "[INFO] Found n=${header_dir_list_length} header directories.")

###############
## Debugging ##
# PRINT_ALL_VARIABLES()
###############

##########
## Make ##
##########

# Create target - add sources
add_library(cav_so SHARED
    ${autogen_sources}
)

# Properties
set_target_properties(cav_so PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  PUBLIC_HEADER ${AUTOGEN_DIR}/CAV_mdl/CAV_ctrl_mdl_wTraJ_241219.h
)

# Include
target_include_directories(cav_so PRIVATE include)
target_include_directories(cav_so PRIVATE ${autogen_header_dirs})

# target_link_libraries(cav_so PUBLIC 
#     m
# )

####################
## Compiler Flags ##
####################

# Compiler options
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  if(CMAKE_BUILD_TYPE MATCHES Release OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    target_compile_options(cav_so PRIVATE
      "-O2"
    )
  endif()

  target_compile_options(cav_so PRIVATE
    -Wall -fPIC -fvisibility=hidden
  )

elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  if(CMAKE_BUILD_TYPE MATCHES Release OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    target_compile_options(cav_so PRIVATE
      "/Ox"
    )
  endif()

  target_compile_options(cav_so PRIVATE
    "/W4"
  )

elseif(CMAKE_CXX_COMPILER_ID MATCHES "CLANG")
  if(CMAKE_BUILD_TYPE MATCHES Release OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    target_compile_options(cav_so PRIVATE
      "-O2"
    )
  endif()

  target_compile_options(cav_so PRIVATE
    "-Wall"
  )

endif()

######################
## Compiler Defines ##
######################

# Specify compiler preprocessor definitions for target 
# from the MATLAB-provided CMakeLists.txt.
target_compile_definitions(cav_so PRIVATE
    -DMODEL=CAV_mdl
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
    -DMT=0
    -DBUILDING_CAV_CTRL_MDL_WTRAJ_241219 # Important
)

## Function Exports on Windows builds
if(MSVC)
    target_link_options(cav_so PRIVATE 
        /DEF:${CMAKE_SOURCE_DIR}/CAV_mdl/CAV_ctrl_mdl_wTraJ_241219.def
    )
elseif(MINGW)
    target_sources(cav_so PRIVATE
        ${CMAKE_SOURCE_DIR}/CAV_mdl/CAV_ctrl_mdl_wTraJ_241219.def
    )
endif()

#############
## Install ##
#############

install(
    TARGETS cav_so
    DESTINATION ${CMAKE_SOURCE_DIR}/lib/
)

#############
## Testing ##
#############

#############
## Example ##
#############

### https://stackoverflow.com/questions/17511496/how-to-create-a-shared-library-with-cmake
# cmake_minimum_required(VERSION 3.9)
# project(mylib VERSION 1.0.1 DESCRIPTION "mylib description")
# include(GNUInstallDirs)
# add_library(mylib SHARED src/mylib.c)
# set_target_properties(mylib PROPERTIES
#     VERSION ${PROJECT_VERSION}
#     SOVERSION ${PROJECT_VERSION_MAJOR}
#     PUBLIC_HEADER api/mylib.h)
# configure_file(mylib.pc.in mylib.pc @ONLY)
# target_include_directories(mylib PRIVATE .)
# install(TARGETS mylib
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# install(FILES ${CMAKE_BINARY_DIR}/mylib.pc
#     DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)