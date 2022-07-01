# Import the gtwrap package.
find_package(gtwrap)

set(WRAP_PYTHON_VERSION
    "Default"
    CACHE STRING "The Python version to use for wrapping")

gtwrap_get_python_version(${WRAP_PYTHON_VERSION})

#############################################################
##### Necessary variables to set for Matlab wrapper to work
# Set up cache options
option(WRAP_MEX_BUILD_STATIC_MODULE
       "Build MATLAB wrapper statically (increases build time)" OFF)
set(WRAP_BUILD_MEX_BINARY_FLAGS
    ""
    CACHE STRING "Extra flags for running Matlab MEX compilation")
set(WRAP_TOOLBOX_INSTALL_PATH
    "${CMAKE_INSTALL_PREFIX}/gpmp2_toolbox"
    CACHE
      PATH
      "Matlab toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/gpmp2_toolbox"
)

option(
  WRAP_BUILD_TYPE_POSTFIXES
  "Enable/Disable appending the build type to the name of compiled libraries"
  ON)
#############################################################

# Copy matlab.h to the correct folder.
configure_file(${PROJECT_SOURCE_DIR}/matlab/matlab.h
  ${PROJECT_BINARY_DIR}/wrap/matlab.h COPYONLY)

# Include the Matlab related code.
include(MatlabWrap)

# Ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

# Install library
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# ##############################################################################
# Build MATLAB wrapper
# (CMake tracks the dependecy to link with GTSAM through our project's static library)
set(interface_files "${PROJECT_SOURCE_DIR}/gpmp2.i")
matlab_wrap("${interface_files}" "${PROJECT_NAME}" "" "" "" "")

# We print out our configuration for an easy visual check
message("========== Configuration Options ==========")
message(STATUS "Project:                  ${PROJECT_NAME}")
message(STATUS "MATLAB Root:              ${MATLAB_ROOT}")
message(STATUS "Mex Compiler:             ${Matlab_MEX_COMPILER}")
message("===========================================")
