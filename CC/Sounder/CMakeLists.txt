cmake_minimum_required(VERSION 3.10)
#Allow project version
cmake_policy(SET CMP0048 NEW)
project(Sounder VERSION 1.2.0)

configure_file(${CMAKE_SOURCE_DIR}/include/version_config.h.in ${CMAKE_SOURCE_DIR}/include/version_config.h)

# Off by default: on, every build directory wrote its libraries into the
# source tree's build/, so a second build directory (a Debug or sanitizer
# build) overwrote the first one's archive and the first then linked it.
# The usual build in build/ is the same either way.
option(FORCE_BUILD_PATH "Hardcode the build directory path to be 'build/'" OFF)
if(FORCE_BUILD_PATH)
  message(STATUS "Setting the build directory to build folder")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
  #set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
endif()

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}'.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Verbose Makefile")
  set( CMAKE_VERBOSE_MAKEFILE on )
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb3 -fstack-protector-all -fsanitize=undefined -fsanitize=address -fsanitize=null")
  #-fsanitize=thread- fstack-protector-all -fsanitize=address -fstack-check
  #-fsanitize=address -fsanitize=pointer-compare
  #-fsanitize=address -fsanitize=pointer-subtract
  #-fsanitize=leak -fsanitize=undefined -fsanitize=null
endif()

# New versions of GCC enable position-independent executables by default. We
# don't want to compile FlexRAN libraries with -fPIC because it reduces
# performance by around 40%.
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie -pthread")

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  message(STATUS "Using GNU compiler, compiler ID ${CMAKE_C_COMPILER_ID}")
  #For Ubuntu 1804 need to keep the c11 std for thread check
  set(CMAKE_C_FLAGS "-std=c11 -Wall")
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    set(ARCH_FLAGS "-mcpu=native")   # ARM (e.g. DGX Spark / Grace): NEON/SVE
  else()
    set(ARCH_FLAGS "-march=native")
  endif()
  # -Wshadow is in neither -Wall nor -Wextra, so it has to be asked for. It is
  # on here to keep a shadowed declaration from silently hiding an outer one:
  # the tree was clean of all 7 when it was turned on, and third_party/ is a
  # SYSTEM include below, so it contributes no noise.
  # -Wno-psabi: GCC's x86 note that passing a struct with a std::complex<float>
  # member (the detector's Detection) "changed ABI in GCC 4.4" matters only when
  # linking against objects built with a GCC older than 4.4; it printed once per
  # call site on every x86 build and buried real warnings. The ARM build never
  # printed it.
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra -Wshadow -Wno-psabi ${ARCH_FLAGS}")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
else()
  message(FATAL_ERROR "Unsupported version of compiler")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiling with CXX Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "Debug Flags: ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "Release Flags: ${CMAKE_CXX_FLAGS_RELEASE}")

# Console logging level
if(LOG_LEVEL STREQUAL "none")
  message(STATUS "Logging level = none.")
  add_definitions(-DMLPD_LOG_LEVEL=0)
elseif(LOG_LEVEL STREQUAL "error")
  message(STATUS "Logging level = error.")
  add_definitions(-DMLPD_LOG_LEVEL=1)
elseif(LOG_LEVEL STREQUAL "warn")
  message(STATUS "Logging level = warn.")
  add_definitions(-DMLPD_LOG_LEVEL=2)
elseif(LOG_LEVEL STREQUAL "info")
  message(STATUS "Logging level = info.")
  add_definitions(-DMLPD_LOG_LEVEL=3)
elseif(LOG_LEVEL STREQUAL "frame")
  message(STATUS "Logging level = frame. Warning: Performance will be low.")
  add_definitions(-DMLPD_LOG_LEVEL=4)
elseif(LOG_LEVEL STREQUAL "subframe")
  message(STATUS "Logging level = subframe. Warning: Performance will be low.")
  add_definitions(-DMLPD_LOG_LEVEL=5)
elseif(LOG_LEVEL STREQUAL "trace")
  message(STATUS "Logging level = trace. Warning: Performance will be low.")
  add_definitions(-DMLPD_LOG_LEVEL=6)
else()
  message(STATUS "No logging level specified. Using warning level.")
  add_definitions(-DMLPD_LOG_LEVEL=3)
endif()

#Gflags
find_package(gflags REQUIRED)
message(VERBOSE "  gflags: Includes ${GFLAGS_INCLUDE_DIR} Libraries: ${GFLAGS_LIBRARIES}")
include_directories(${GFLAGS_INCLUDE_DIR})

set(THREADS_PREFER_PTHREAD_FLAG TRUE)
#find_package(Threads REQUIRED)
message(STATUS "Using Pthread Library: ${CMAKE_THREAD_LIBS_INIT}: ${CMAKE_USE_PTHREADS_INIT}")

set(RADIO_TYPE SOAPY_IRIS CACHE STRING "RADIO_TYPE defaulting to 'SOAPY_IRIS', valid types are SOAPY_IRIS / SOAPY_UHD / PURE_UHD")
message(STATUS "RADIO_TYPE: ${RADIO_TYPE}")
if(RADIO_TYPE STREQUAL PURE_UHD)
  message(STATUS "Enabled PURE_UHD radio")
  add_definitions(-DUSE_UHD)
  # Adding dependency of UHD and boost external libraries
  find_package(UHD "3.15.0" REQUIRED)
  include_directories(${UHD_INCLUDE_DIRS})
  message(STATUS "Using UHD Version: ${UHD_VERSION}")

  ########################################################################
  # PURE UHD dependencies
  ########################################################################
  # Adding dependency of Boost to souder
  set(BOOST_REQUIRED_COMPONENTS
      program_options
      system
      thread)

  if(MSVC)
      set(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
      if(BOOST_ALL_DYN_LINK)
          add_definitions(-DBOOST_ALL_DYN_LINK) #setup boost auto-linking in msvc
      else(BOOST_ALL_DYN_LINK)
          set(BOOST_REQUIRED_COMPONENTS) #empty components list for static link
      endif(BOOST_ALL_DYN_LINK)
  endif(MSVC)
  find_package(Boost "1.65" REQUIRED ${BOOST_REQUIRED_COMPONENTS})
  message(STATUS "Using Boost Version: ${Boost_VERSION}")

  set(PURE_UHD_SOURCES 
      BaseRadioSetUHD.cc
      ClientRadioSetUHD.cc
      RadioUHD.cc)
elseif(RADIO_TYPE STREQUAL SOAPY_UHD)
  message(STATUS "Enabled SOAPY UHD radio")
  add_definitions(-DUSE_SOAPYUHD)
else()
  message(STATUS "Enabled SOAPY Iris radio")
endif()

########################################################################
# SoapySDR dependency
########################################################################
find_package(SoapySDR 0.7 CONFIG)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "SoapySDR development files not found")
    return()
endif ()

find_package(HDF5 1.10 REQUIRED COMPONENTS CXX)
if (NOT HDF5_FOUND)
    message(FATAL_ERROR "HDF5 development files not found")
    return()
endif()
message(VERBOSE "  HDF5 Includes: ${HDF5_INCLUDE_DIRS} Libraries: ${HDF5_LIBRARIES}")


set(directory "logs")
file(MAKE_DIRECTORY ${directory})

add_definitions(-DTHREADED_INIT)

message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")
message(STATUS "HDF5_INCLUDE_DIRS: ${HDF5_INCLUDE_DIRS}")
message(STATUS "HDF5_LIBRARIES: ${HDF5_LIBRARIES}")
include_directories(${SoapySDR_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS})
# third_party/ is vendored and not ours to patch, so include it as SYSTEM: the
# compiler then suppresses warnings raised inside those headers (GCC 13 flags
# -Wpessimizing-move in concurrentqueue.h) while still finding them. Keep our
# own headers OUT of this call so their warnings stay visible.
include_directories(SYSTEM third_party/ third_party/nlohmann/single_include)

set(SOUNDER_SOURCES ${PURE_UHD_SOURCES}
    ClientRadioSet.cc
    config.cc
    data_generator.cc
    Radio.cc RadioSoapy.cc RadioHoudini.cc BeaconFramer.cc HoudiniFramer.cc IrisFramer.cc RadioSetFactory.cc
    receiver.cc
    scheduler.cc
    recorder_worker.cc
    hdf5_lib.cc
    hdf5_reader.cc
    recorder_thread.cc
    BaseRadioSet.cc
    BaseRadioSet-calibrate-digital.cc
    BaseRadioSet-calibrate-analog.cc
    signalHandler.cpp)

# Optional GPU beacon correlator (DGX Spark / GB10). find_beacon_cuda.cu runs
# the detection pipeline on the GPU over unified memory; the portable CPU
# find_beacon_avx stays the default and the numeric reference.
option(HOUDINI_USE_CUDA "Build the GPU beacon correlator (DGX Spark)" OFF)
if (HOUDINI_USE_CUDA)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  if (NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES native)
  endif()
  add_definitions(-DUSE_CUDA)
  # The kernel is a backend of houdini_sync's Detector (sync/detector.cc), so
  # it is compiled into the library, not into the sounder's sources: every
  # consumer of the library then links.
  set(HOUDINI_CUDA_SOURCES find_beacon_cuda.cu)
  message(STATUS "GPU beacon correlator: ENABLED (CUDA arch ${CMAKE_CUDA_ARCHITECTURES})")
endif()

add_executable(sounder
    main.cc
    ${SOUNDER_SOURCES})

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64|arm") 
    set(MUFFT_LIBRARIES
        ${CMAKE_SOURCE_DIR}/mufft/libmuFFT.a)
else()
    set(MUFFT_LIBRARIES
        ${CMAKE_SOURCE_DIR}/mufft/libmuFFT.a
        ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-sse.a
        ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-sse3.a
        ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-avx.a)
endif()

target_link_libraries(sounder houdini_sync -lpthread ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${MUFFT_LIBRARIES})

add_library(sounder_module MODULE
    ${SOUNDER_SOURCES})

target_link_libraries(sounder_module houdini_sync -lpthread ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    -Wl,--whole-archive
    ${MUFFT_LIBRARIES}
    -Wl,--no-whole-archive
    ${HDF5_LIBRARIES}
    ${SoapySDR_LIBRARIES})

enable_testing()
# ---- houdini_sync: the UE synchronisation library ---------------------------
# One classed home for the beacon shapes, the detector, the confirm, the carrier
# estimator and their configuration (docs/SYNC_LIBRARY_ARCHITECTURE.md). Static,
# portable (x86 and aarch64), no dependency beyond what the sounder links.
# comms-lib and utils are compiled HERE and nowhere else: the sounder, the
# module, the two HIL tools, the geometry test, the dumper and the beacon TX
# tool all take them from this library, so there is one find_beacon_avx in
# the tree.
add_library(houdini_sync STATIC
    sync/sync_config.cc
    sync/beacon_shape.cc
    sync/detector.cc
    comms-lib.cc comms-lib-avx.cc comms-lib-portable.cc utils.cc
    ${HOUDINI_CUDA_SOURCES})
# Consumers see include/ only: a library consumer must not be able to reach
# the sounder's sources through the library's include path.
target_include_directories(houdini_sync PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Linked into sounder_module (a MODULE) as well as the executables.
set_target_properties(houdini_sync PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(houdini_sync PUBLIC ${MUFFT_LIBRARIES})

# The radio-free tests and the bench tools. ON by default so the gate keeps
# building them; a packager can turn them off.
option(SOUNDER_BUILD_TESTS "Build the radio-free tests and bench tools (ctest)" ON)
if (SOUNDER_BUILD_TESTS)
add_executable(sync_config_test tests/comms-func/sync_config_test.cc)
target_link_libraries(sync_config_test houdini_sync -lpthread)
add_test(NAME sync_config COMMAND sync_config_test ${CMAKE_CURRENT_SOURCE_DIR}/CSI_DEMO_WALKTHROUGH.md)

add_executable(sync_config_schema tests/comms-func/sync_config_schema.cc)
target_link_libraries(sync_config_schema houdini_sync -lpthread)

add_executable(golden_window_test tests/comms-func/golden_window_test.cc)
target_link_libraries(golden_window_test houdini_sync -lpthread)
add_test(NAME golden_window COMMAND golden_window_test
         ${CMAKE_CURRENT_SOURCE_DIR}/tests/comms-func/fixtures/golden)

# The UE beacon-sync geometry across the whole rate ladder. Links NOTHING
# (sync_geometry.h is header-only and dependency-free) so it builds and runs
# anywhere, which is the point: the defects it covers were only visible by
# running a UE against silicon at one rate.
add_executable(sync_geometry_test tests/comms-func/sync_geometry_test.cc)
target_include_directories(sync_geometry_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME sync_geometry COMMAND sync_geometry_test)

# The resync state machine, every transition without a radio.
add_executable(resync_policy_test tests/comms-func/resync_policy_test.cc)
target_include_directories(resync_policy_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME resync_policy COMMAND resync_policy_test)

# Radio-free A/B: AP-41's kalman against the shipped alpha-beta over the SAME
# synthetic detections. The rig can only run one at a time; this pairs them.
add_executable(grid_tracker_test tests/comms-func/grid_tracker_test.cc)
target_include_directories(grid_tracker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME grid_tracker COMMAND grid_tracker_test)

# AP-79: the x2 TX interpolator and the sub-6 RX channel filter, measured on
# the implementation, with the mutation matrix. Header-only; no radio.
add_executable(band_filters_test tests/comms-func/band_filters_test.cc)
target_include_directories(band_filters_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME band_filters COMMAND band_filters_test)

# AP-79: the per-channel RF-DC plan (zone, calibration mode, RX filter)
# derived from the NCO, checked against the HS-202 plan's table.
add_executable(rf_plan_test tests/comms-func/rf_plan_test.cc)
target_include_directories(rf_plan_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME rf_plan COMMAND rf_plan_test)

# AP-79: the mode-V bring-up order and values against a recording fake device.
add_executable(mode_v_bringup_test tests/comms-func/mode_v_bringup_test.cc)
target_include_directories(mode_v_bringup_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mode_v_bringup_test ${SoapySDR_LIBRARIES})
add_test(NAME mode_v_bringup COMMAND mode_v_bringup_test)

# AP-79: the TX burst interpolation and RX lane filtering at the radio boundary.
add_executable(tx_rx_boundary_test tests/comms-func/tx_rx_boundary_test.cc)
target_include_directories(tx_rx_boundary_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME tx_rx_boundary COMMAND tx_rx_boundary_test)
# The same test under AddressSanitizer whatever the build type: its cache
# over-read case keeps the output right, so only the sanitizer can fail it.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "-fsanitize=address,undefined")
set(CMAKE_REQUIRED_LIBRARIES "-fsanitize=address,undefined")  # the link, on CMake 3.10
check_cxx_source_compiles("int main() { return 0; }" SOUNDER_HAVE_ASAN)
unset(CMAKE_REQUIRED_FLAGS)
unset(CMAKE_REQUIRED_LIBRARIES)
if (SOUNDER_HAVE_ASAN)
  add_executable(tx_rx_boundary_asan_test tests/comms-func/tx_rx_boundary_test.cc)
  target_include_directories(tx_rx_boundary_asan_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_options(tx_rx_boundary_asan_test PRIVATE
      -O1 -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
  target_link_libraries(tx_rx_boundary_asan_test PRIVATE -fsanitize=address,undefined)
  add_test(NAME tx_rx_boundary_asan COMMAND tx_rx_boundary_asan_test)
else()
  message(WARNING "no AddressSanitizer: tx_rx_boundary_asan is not built, so the TX interpolator's cache over-read case is uncovered")
endif()

# AP-79: the band-limited beacon: its band, its fit, and the real detector
# through the channel-filtered lane.
add_executable(beacon_bandlimit_test tests/comms-func/beacon_bandlimit_test.cc)
target_link_libraries(beacon_bandlimit_test houdini_sync -lpthread ${MUFFT_LIBRARIES})
add_test(NAME beacon_bandlimit COMMAND beacon_bandlimit_test)

# AP-79: CommsLib::FFT / IFFT on an input shorter than the transform (the
# inherited code overran the heap there).
add_executable(commslib_fft_test tests/comms-func/commslib_fft_test.cc)
target_link_libraries(commslib_fft_test houdini_sync -lpthread ${MUFFT_LIBRARIES})
add_test(NAME commslib_fft COMMAND commslib_fft_test)

# AP-79: the CSI view's per-symbol FFT against the DFT-matrix definition.
add_executable(dc_fft_test tests/comms-func/dc_fft_test.cc)
target_include_directories(dc_fft_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/mufft)
target_link_libraries(dc_fft_test ${MUFFT_LIBRARIES})
add_test(NAME dc_fft COMMAND dc_fft_test)
add_executable(cir_test tests/comms-func/cir_test.cc)
target_include_directories(cir_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/mufft)
target_link_libraries(cir_test ${MUFFT_LIBRARIES})
add_test(NAME cir COMMAND cir_test)
# The Spectrum tab's Welch average of the pilot slot (SPC1): dBFS known answers.
add_executable(spectrum_test tests/comms-func/spectrum_test.cc)
target_include_directories(spectrum_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/mufft)
target_link_libraries(spectrum_test ${MUFFT_LIBRARIES})
add_test(NAME spectrum COMMAND spectrum_test)
add_test(NAME csi_gui_metrics COMMAND python3 test_metrics.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/csi_gui)
add_test(NAME csi_gui_control COMMAND python3 test_control.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/csi_gui)
add_test(NAME csi_gui_check_setup COMMAND python3 test_check_setup.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/csi_gui)
add_test(NAME csi_gui_record_replay COMMAND python3 test_record_replay.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/csi_gui)
add_test(NAME rung_report COMMAND python3 rung_report_test.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests/demo-verify)
add_test(NAME freeze_watch COMMAND python3 freeze_watch_test.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests/demo-verify)
add_test(NAME pacer_core_check COMMAND python3 pacer_core_check_test.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests/demo-verify)
add_test(NAME config_keys COMMAND python3 tests/config_keys_test.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME mer_sampler COMMAND python3 mer_sampler_test.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests/demo-verify)
add_test(NAME run_summary COMMAND python3 run_summary_test.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests/demo-verify)
add_test(NAME sounder_log COMMAND python3 sounder_log_test.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests/demo-verify)
add_test(NAME syn1_wire COMMAND python3 syn1_wire_test.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests/demo-verify)
# fstage_report needs numpy (the rig host's venv has it); registered only where it imports.
execute_process(COMMAND python3 -c "import numpy" RESULT_VARIABLE HAVE_NUMPY_RC OUTPUT_QUIET ERROR_QUIET)
if(HAVE_NUMPY_RC EQUAL 0)
  add_test(NAME fstage_report COMMAND python3 fstage_report_test.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests/demo-verify)
else()
  message(STATUS "python3 has no numpy: the fstage_report test is not registered")
endif()

# AP-79: the link-health monitor port, pinned to the software lane's T0 cases
# and to real captures from a streaming mode-V node.
add_executable(link_health_test tests/comms-func/link_health_test.cc)
target_include_directories(link_health_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME link_health COMMAND link_health_test
         ${CMAKE_CURRENT_SOURCE_DIR}/tests/comms-func/fixtures/link_health/mode_v_streaming_21.txt
         ${CMAKE_CURRENT_SOURCE_DIR}/tests/comms-func/fixtures/link_health/demo_stack_fi1.txt)

# AP-79: the replay strobe offs/len in the driver's units.
add_executable(replay_strobe_test tests/comms-func/replay_strobe_test.cc)
target_include_directories(replay_strobe_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME replay_strobe COMMAND replay_strobe_test)
add_executable(pilot_ladder_test tests/comms-func/pilot_ladder_test.cc)
target_include_directories(pilot_ladder_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME pilot_ladder COMMAND pilot_ladder_test)
add_executable(slot_align_test tests/comms-func/slot_align_test.cc)
target_include_directories(slot_align_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME slot_align COMMAND slot_align_test)
add_executable(grid_check_test tests/comms-func/grid_check_test.cc)
target_include_directories(grid_check_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME grid_check COMMAND grid_check_test)
add_executable(pre_cfo_test tests/comms-func/pre_cfo_test.cc)
target_include_directories(pre_cfo_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME pre_cfo COMMAND pre_cfo_test)
add_executable(bs_slots_test tests/comms-func/bs_slots_test.cc)
target_include_directories(bs_slots_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME bs_slots COMMAND bs_slots_test)
add_executable(node_version_test tests/comms-func/node_version_test.cc)
target_include_directories(node_version_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME node_version COMMAND node_version_test)
add_executable(logger_macro_test tests/comms-func/logger_macro_test.cc)
target_include_directories(logger_macro_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME logger_macro COMMAND logger_macro_test)
add_executable(rx_packet_test tests/comms-func/rx_packet_test.cc)
target_include_directories(rx_packet_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME rx_packet COMMAND rx_packet_test)
add_executable(resync_window_test tests/comms-func/resync_window_test.cc)
target_include_directories(resync_window_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME resync_window COMMAND resync_window_test)
add_executable(stream_args_test tests/comms-func/stream_args_test.cc)
target_include_directories(stream_args_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME stream_args COMMAND stream_args_test)
add_executable(core_map_test tests/comms-func/core_map_test.cc)
target_include_directories(core_map_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME core_map COMMAND core_map_test)
add_executable(pilot_slope_fit_test tests/comms-func/pilot_slope_fit_test.cc)
target_include_directories(pilot_slope_fit_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME pilot_slope_fit COMMAND pilot_slope_fit_test)
add_executable(clock_steer_test tests/comms-func/clock_steer_test.cc)
target_include_directories(clock_steer_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME clock_steer COMMAND clock_steer_test)
add_executable(cns_sample_test tests/comms-func/cns_sample_test.cc)
target_include_directories(cns_sample_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME cns_sample COMMAND cns_sample_test)
add_executable(stream_result_test tests/comms-func/stream_result_test.cc)
target_include_directories(stream_result_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(stream_result_test -lpthread)
add_test(NAME stream_result COMMAND stream_result_test)

# AP-79: nr_pss_bl's acquisition thresholds derived on a simulated link.
add_executable(detection_calibration_test tests/comms-func/detection_calibration_test.cc)
target_link_libraries(detection_calibration_test houdini_sync -lpthread ${MUFFT_LIBRARIES})
add_test(NAME detection_calibration COMMAND detection_calibration_test)

# AP-79: the run configs through the real Config (the pilot slot is one slot,
# fft_size symbols, inside the channel). Builds the sounder's own sources.
add_executable(config_pilot_test tests/comms-func/config_pilot_test.cc ${SOUNDER_SOURCES})
target_include_directories(config_pilot_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(config_pilot_test houdini_sync -lpthread ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES} ${HDF5_LIBRARIES} ${MUFFT_LIBRARIES})
add_test(NAME config_pilot COMMAND config_pilot_test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# AP-85: per-band channel widths through the real Config: the backward
# compatibility pin (the configs without per-band widths, byte for byte) and the wide band.
add_executable(per_band_test tests/comms-func/per_band_test.cc ${SOUNDER_SOURCES})
target_include_directories(per_band_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(per_band_test houdini_sync -lpthread ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES} ${HDF5_LIBRARIES} ${MUFFT_LIBRARIES})
add_test(NAME per_band COMMAND per_band_test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# RadioHoudini's device and stream arguments for the demo config, from the
# static builders (no device). Builds the sounder's own sources.
add_executable(radio_houdini_args_test tests/comms-func/radio_houdini_args_test.cc ${SOUNDER_SOURCES})
target_include_directories(radio_houdini_args_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(radio_houdini_args_test houdini_sync -lpthread ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES} ${HDF5_LIBRARIES} ${MUFFT_LIBRARIES})
add_test(NAME radio_houdini_args COMMAND radio_houdini_args_test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# How the receiver's threads end a run on an ended Houdini stream, through fake
# radio sets: this test's own factory replaces RadioSetFactory.cc.
set(RECEIVER_FAULT_SOURCES ${SOUNDER_SOURCES})
list(FILTER RECEIVER_FAULT_SOURCES EXCLUDE REGEX "RadioSetFactory\\.cc$")
add_executable(receiver_fault_test tests/comms-func/receiver_fault_test.cc ${RECEIVER_FAULT_SOURCES})
target_include_directories(receiver_fault_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(receiver_fault_test houdini_sync -lpthread ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES} ${HDF5_LIBRARIES} ${MUFFT_LIBRARIES})
add_test(NAME receiver_fault COMMAND receiver_fault_test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
# A radio whose channel setup throws on a set's init thread fails the set
# instead of aborting: this test's fake radio replaces Radio.cc.
set(RADIO_SET_SETUP_SOURCES ${SOUNDER_SOURCES})
list(FILTER RADIO_SET_SETUP_SOURCES EXCLUDE REGEX "^Radio\\.cc$")
add_executable(radio_set_setup_test tests/comms-func/radio_set_setup_test.cc ${RADIO_SET_SETUP_SOURCES})
target_include_directories(radio_set_setup_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(radio_set_setup_test houdini_sync -lpthread ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES} ${HDF5_LIBRARIES} ${MUFFT_LIBRARIES})
add_test(NAME radio_set_setup COMMAND radio_set_setup_test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
# The TDD framer's arm retry reloads the beacon as a fresh load, through a fake
# radio and device.
add_executable(framer_rearm_test tests/comms-func/framer_rearm_test.cc ${SOUNDER_SOURCES})
target_include_directories(framer_rearm_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(framer_rearm_test houdini_sync -lpthread ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES} ${HDF5_LIBRARIES} ${MUFFT_LIBRARIES})
add_test(NAME framer_rearm COMMAND framer_rearm_test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
# AP-85: the host cost of the UE's TX interpolation per lane path. Standalone,
# not a test: run it on the rig host (./build/tx_interp_bench [rounds]).
add_executable(tx_interp_bench tests/comms-func/tx_interp_bench.cc)
target_include_directories(tx_interp_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# The index-convention check AP-34(a) did not have. Links the real CommsLib so
# it exercises the REAL detector; a replica would agree with itself.
add_executable(beacon_geometry_test
    tests/comms-func/beacon_geometry_test.cc)
target_link_libraries(beacon_geometry_test houdini_sync -lpthread ${MUFFT_LIBRARIES})
add_test(NAME beacon_geometry COMMAND beacon_geometry_test)

# Bench helper: each candidate beacon's TX core and correlator replica, written
# from the one definition in include/sync/beacon_shapes.h for the probes to read.
add_executable(beacon_shape_dump
    tests/comms-func/beacon_shape_dump.cc)
target_link_libraries(beacon_shape_dump houdini_sync -lpthread ${MUFFT_LIBRARIES})

# HIL helper: transmit the real Gold beacon from a Houdini board so a Houdini UE
# running the unmodified receiver.cc client-sync path can acquire it.
add_executable(beacon_tx_gold
    tests/comms-func/beacon_tx_gold.cc)
target_link_libraries(beacon_tx_gold houdini_sync -lpthread
    ${SoapySDR_LIBRARIES}
    ${MUFFT_LIBRARIES})

# HIL harness: drive both ends with the real BaseRadioSet (BS beacon) +
# ClientRadioSet (UE sync) -- proves the BaseRadioSet Houdini backend TXes the
# beacon the UE acquires, without the un-runnable BS receive loop.
add_executable(houdini_bs_ue_sync
    tests/comms-func/houdini_bs_ue_sync.cc
    ${SOUNDER_SOURCES})
target_link_libraries(houdini_bs_ue_sync houdini_sync -lpthread ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${MUFFT_LIBRARIES})

# HIL closed-loop: BS beacon -> UE sync -> UE pilot -> BS receive, all through
# the real BaseRadioSet + ClientRadioSet over the bidirectional Houdini link.
add_executable(houdini_loopback
    tests/comms-func/houdini_loopback.cc
    ${SOUNDER_SOURCES})
target_link_libraries(houdini_loopback houdini_sync -lpthread ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${MUFFT_LIBRARIES})
endif()  # SOUNDER_BUILD_TESTS
