find_package(gtwrap)

### Get the default python version if not specified
set(WRAP_PYTHON_VERSION
  "Default"
  CACHE STRING "The Python version to use for wrapping")
gtwrap_get_python_version(${WRAP_PYTHON_VERSION})
message(STATUS "Wrapper Python Version: ${WRAP_PYTHON_VERSION}")

set(PYTHON_REQUIREMENTS_PATH "requirements.txt")
set(PYTHON_TESTS "${PROJECT_NAME}.tests")

### Set the path to the Python directory of GPMP2 and the Python binary directory.
set(GPMP2_PYTHON_PATH ${PROJECT_SOURCE_DIR}/python/)
file(TO_NATIVE_PATH "${PROJECT_BINARY_DIR}/python" GPMP2_PYTHON_BINARY_DIR)

### Configure templates and generate necessary files in build.
configure_file(${GPMP2_PYTHON_PATH}/templates/setup.py.in
  ${GPMP2_PYTHON_BINARY_DIR}/setup.py)
configure_file(${GPMP2_PYTHON_PATH}/requirements.txt
  ${GPMP2_PYTHON_BINARY_DIR}/requirements.txt COPYONLY)
configure_file(${GPMP2_PYTHON_PATH}/templates/${PROJECT_NAME}.tpl
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl)

### Copy the gpmp2 python directory
file(COPY ${GPMP2_PYTHON_PATH}/${PROJECT_NAME}
  DESTINATION ${GPMP2_PYTHON_BINARY_DIR})

### Set the name(s) of the interface file used for wrapping
set(interface_headers
  ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}.i
  ${ADDITIONAL_INTERFACE_FILES}
)

pybind_wrap(
  ${PROJECT_NAME}_py # target
  "${interface_headers}" # interface header file
  "${PROJECT_NAME}.cpp" # the generated cpp
  "${PROJECT_NAME}" # module_name
  "${PROJECT_NAME}" # top namespace in the cpp file
  "${ignore}" # ignore classes
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl
  ${PROJECT_NAME} # libraries to link against
  "${PROJECT_NAME}" # dependencies to build first
  ON # use boost serialization
)

set_target_properties(
  ${PROJECT_NAME}_py
  PROPERTIES OUTPUT_NAME "${PROJECT_NAME}"
  LIBRARY_OUTPUT_DIRECTORY "${GPMP2_PYTHON_BINARY_DIR}/${PROJECT_NAME}"
  DEBUG_POSTFIX "" # Otherwise you will have a wrong name
  RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
)

### Custom make command to install python library `make python-install`
add_custom_target(
  python-install
  COMMAND ${PYTHON_EXECUTABLE} -m pip install .
  DEPENDS ${PROJECT_NAME}_py
  WORKING_DIRECTORY ${GPMP2_PYTHON_BINARY_DIR})

if(UNIX)
  set(GPMP2_PATH_SEP ":")
else()
  set(GPMP2_PATH_SEP ";")
endif()

# Unit tests
set(python_unit_test_suites)

macro(PYTHON_UNIT_TEST_SUITE suiteName directory)
  add_custom_target(
    python-test.${suiteName}
    COMMAND
    ${CMAKE_COMMAND} -E env # add package to python path so no need to install
    "PYTHONPATH=${GPMP2_PYTHON_BINARY_DIR}${GPMP2_PATH_SEP}$ENV{PYTHONPATH}"
    ${PYTHON_EXECUTABLE} -m unittest discover -v -s .
    DEPENDS ${PROJECT_NAME}_py
    WORKING_DIRECTORY ${directory})
  list(APPEND python_unit_test_suites python-test.${suiteName})
endmacro()

PYTHON_UNIT_TEST_SUITE(base ${PROJECT_SOURCE_DIR}/python/tests)

# Custom make command to run all Python tests
add_custom_target(
  python-test
  DEPENDS ${PROJECT_NAME}_py ${python_unit_test_suites})
