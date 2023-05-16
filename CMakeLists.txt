cmake_minimum_required(VERSION 3.0)
enable_testing()
project(gpmp2
        LANGUAGES CXX C
        VERSION 1.0.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

add_compile_options(-faligned-new)

# Enforce c++17 standards
add_compile_options(-std=c++17) # CMake 3.1 and earlier
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#TODO(Varun) Verify C++17 is enabled on Windows as well
# Look at GtsamBuildTypes L153

# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

# version indicator
set(GPMP2_VERSION_MAJOR 1)
set(GPMP2_VERSION_MINOR 0)
set(GPMP2_VERSION_PATCH 0)
set(gpmp2_VERSION_STRING "${GPMP2_VERSION_MAJOR}.${GPMP2_VERSION_MINOR}.${GPMP2_VERSION_PATCH}")
set(GPMP2_VERSION_STRING gpmp2_VERSION_STRING)

# options: whether turn on Matlab/Python toolbox
option(GPMP2_BUILD_STATIC_LIBRARY "whether build static library" OFF)
option(GPMP2_BUILD_MATLAB_TOOLBOX "whether build matlab toolbox, need shared lib" OFF)
option(GPMP2_BUILD_PYTHON_TOOLBOX "whether build python toolbox, need shared lib" OFF)

# Enable or disable serialization with GPMP2_ENABLE_BOOST_SERIALIZATION
option(GPMP2_ENABLE_BOOST_SERIALIZATION "Enable Boost serialization" ON)
if(GPMP2_ENABLE_BOOST_SERIALIZATION)
  add_compile_definitions(GTSAM_ENABLE_BOOST_SERIALIZATION)
  add_compile_definitions(GPMP2_ENABLE_BOOST_SERIALIZATION)
endif()

if(GPMP2_BUILD_STATIC_LIBRARY AND GPMP2_BUILD_MATLAB_TOOLBOX)
  message(FATAL_ERROR "Matlab toolbox needs static lib to be built")
endif()


# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
# find_package(Boost 1.65 COMPONENTS filesystem system thread serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIR})


# Generate and install config and dllexport files
configure_file("gpmp2/config.h.in" "gpmp2/config.h")
list(APPEND gpmp2_srcs "${PROJECT_BINARY_DIR}/gpmp2/config.h")
include_directories(BEFORE ${PROJECT_BINARY_DIR}) # So we can include generated config header files
install(FILES "${PROJECT_BINARY_DIR}/gpmp2/config.h" DESTINATION include/gpmp2)


# include current source folder, at the very beginning
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})

# Process source subdirs
add_subdirectory(gpmp2)

# Wrapping to MATLAB
if(GPMP2_BUILD_MATLAB_TOOLBOX)
  # Build Matlab package
  add_subdirectory(matlab)
endif()


# Wrapping to Python
if(GPMP2_BUILD_PYTHON_TOOLBOX)
  add_subdirectory(python)
endif()

# Install config and export files
GtsamMakeConfigFile(gpmp2)
export(TARGETS ${GPMP2_EXPORTED_TARGETS} FILE gpmp2-exports.cmake)
