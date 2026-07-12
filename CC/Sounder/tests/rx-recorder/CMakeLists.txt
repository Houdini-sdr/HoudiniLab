cmake_minimum_required(VERSION 3.10)
project(rx-recorder-tests CXX)

# Radio-free tests for the rx-recorder pipeline: build + run without
# SoapySDR or hardware (only HDF5 + pthread). Mirrors the standalone
# style of ../comms-func.
#   cmake ./ && make && ./test_grid && ./test_capture_paths \
#     && ./test_rx_pipeline && ./test_h5_error

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra -O2")
add_definitions(-DMLPD_LOG_LEVEL=1)

set(SOUNDER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)

find_package(HDF5 1.10 REQUIRED COMPONENTS CXX)

include_directories(
    ${SOUNDER_DIR}
    ${SOUNDER_DIR}/third_party
    ${SOUNDER_DIR}/third_party/nlohmann/single_include
    ${HDF5_INCLUDE_DIRS})

set(RX_RECORDER_TEST_SOURCES
    ${SOUNDER_DIR}/rx_recorder_config.cc
    ${SOUNDER_DIR}/rx_recorder_worker.cc
    ${SOUNDER_DIR}/recorder_thread.cc
    ${SOUNDER_DIR}/hdf5_lib.cc
    ${SOUNDER_DIR}/utils.cc)

# Pure header tests: no HDF5, no pipeline sources.
add_executable(test_grid test_grid.cc)
target_link_libraries(test_grid -lpthread)

add_executable(test_capture_paths test_capture_paths.cc)
target_link_libraries(test_capture_paths -lpthread)

add_executable(test_rx_pipeline test_rx_pipeline.cc ${RX_RECORDER_TEST_SOURCES})
target_link_libraries(test_rx_pipeline ${HDF5_LIBRARIES} -lpthread)

add_executable(test_h5_error test_h5_error.cc ${RX_RECORDER_TEST_SOURCES})
target_link_libraries(test_h5_error ${HDF5_LIBRARIES} -lpthread)

enable_testing()
add_test(NAME grid COMMAND test_grid)
add_test(NAME capture_paths COMMAND test_capture_paths)
add_test(NAME pipeline COMMAND test_rx_pipeline)
add_test(NAME h5_error COMMAND test_h5_error)
