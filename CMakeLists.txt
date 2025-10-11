cmake_minimum_required(VERSION 3.12)
enable_testing()
project(gpmp2
        LANGUAGES CXX C
        VERSION 1.0.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install" CACHE PATH "Local install prefix" FORCE)

add_compile_options(-faligned-new)

# Enforce c++17 standards
add_compile_options(-std=c++17) # CMake 3.1 and earlier
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#TODO(Varun) Verify C++17 is enabled on Windows as well
# Look at GtsamBuildTypes L153

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
  message(FATAL_ERROR "Matlab toolbox needs shared lib to be built")
endif()

# === Local GTSAM bootstrap ===
set(GTSAM_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/gtsam")
set(GTSAM_DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/gtsam")
set(GTSAM_BUILD_DIR "${GTSAM_DEPS_DIR}/build")
set(GTSAM_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/gtsam")

# Ensure submodule is initialized
if(NOT EXISTS "${GTSAM_SUBMODULE_DIR}/CMakeLists.txt")
  message(STATUS "Initializing GTSAM submodule...")
  execute_process(
    COMMAND git submodule update --init --recursive extern/gtsam
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE GTSAM_SUBMODULE_RESULT
  )
  if(NOT GTSAM_SUBMODULE_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to initialize GTSAM submodule.")
    return()
  endif()
endif()

# Build and install GTSAM locally if not already built
if(NOT EXISTS "${GTSAM_INSTALL_DIR}/lib/cmake/GTSAM/GTSAMConfig.cmake")
  message(STATUS "Building and installing GTSAM locally...")

  file(MAKE_DIRECTORY ${GTSAM_BUILD_DIR})

  execute_process(
    COMMAND cmake
      -DCMAKE_INSTALL_PREFIX=${GTSAM_INSTALL_DIR}
      -DGTSAM_BUILD_DOCS=OFF
      -DGTSAM_BUILD_DOC_HTML=OFF
      -DGTSAM_BUILD_DOC_LATEX=OFF
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
      -DGTSAM_BUILD_TESTS=OFF
      ${GTSAM_SUBMODULE_DIR}
    WORKING_DIRECTORY ${GTSAM_BUILD_DIR}
    RESULT_VARIABLE GTSAM_CMAKE_RESULT
  )
  if(NOT GTSAM_CMAKE_RESULT EQUAL 0)
    message(FATAL_ERROR "CMake configuration for GTSAM failed.")
    return()
  endif()

  include(ProcessorCount)
  ProcessorCount(NPROC)
  execute_process(
    COMMAND make -j${NPROC} install
    WORKING_DIRECTORY ${GTSAM_BUILD_DIR}
    RESULT_VARIABLE GTSAM_BUILD_RESULT
  )
  if(NOT GTSAM_BUILD_RESULT EQUAL 0)
    message(FATAL_ERROR "GTSAM build or install failed.")
  endif()
endif()

# Point GPMP2 to local GTSAM install
set(GTSAM_DIR "${GTSAM_INSTALL_DIR}/lib/cmake/GTSAM" CACHE PATH "Local GTSAM" FORCE)
find_package(GTSAM REQUIRED NO_DEFAULT_PATH)

message(STATUS "Using local GTSAM from: ${GTSAM_DIR}")
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

# Include GTSAM CMake tools (installed or source fallback)
set(GTSAMCMakeTools_INSTALLED_DIR "${GTSAM_INSTALL_DIR}/lib/cmake/GTSAMCMakeTools")
set(GTSAMCMakeTools_SOURCE_DIR "${GTSAM_SUBMODULE_DIR}/cmake")

if(EXISTS "${GTSAMCMakeTools_INSTALLED_DIR}/GtsamTesting.cmake")
  list(APPEND CMAKE_MODULE_PATH "${GTSAMCMakeTools_INSTALLED_DIR}")
elseif(EXISTS "${GTSAMCMakeTools_SOURCE_DIR}/GtsamTesting.cmake")
  list(APPEND CMAKE_MODULE_PATH "${GTSAMCMakeTools_SOURCE_DIR}")
else()
  message(FATAL_ERROR "Could not locate GTSAMCMakeTools (neither installed nor source).")
endif()

include(GtsamBuildTypes)
include(GtsamTesting)
include(GtsamPrinting)

# for unit tests and scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
# find_package(Boost 1.65 COMPONENTS filesystem system thread serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIR})


# Generate and install config and dllexport files
configure_file("gpmp2/config.h.in" "gpmp2/config.h")
list(APPEND gpmp2_srcs "${PROJECT_BINARY_DIR}/gpmp2/config.h")
include_directories(BEFORE ${PROJECT_BINARY_DIR}) # So we can include generated config header files
install(FILES "${PROJECT_BINARY_DIR}/gpmp2/config.h" DESTINATION ${CMAKE_INSTALL_PREFIX}/gpmp2)

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
# GtsamMakeConfigFile(gpmp2)
# export(TARGETS ${GPMP2_EXPORTED_TARGETS} FILE gpmp2-exports.cmake)
