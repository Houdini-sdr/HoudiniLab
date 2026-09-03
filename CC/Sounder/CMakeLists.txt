cmake_minimum_required(VERSION 3.10)
#Allow project version
cmake_policy(SET CMP0048 NEW)
project(Sounder VERSION 1.2.0)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

configure_file(${CMAKE_SOURCE_DIR}/include/version_config.h.in ${CMAKE_SOURCE_DIR}/include/version_config.h)

option(FORCE_BUILD_PATH "Hardcode the build directory path to be 'build/'" ON)
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
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra -Wshadow ${ARCH_FLAGS}")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
else()
  message(FATAL_ERROR "Unsupported version of compiler")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
    Radio.cc
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
  list(APPEND SOUNDER_SOURCES find_beacon_cuda.cu)
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

target_link_libraries(sounder houdini_sync -lpthread --enable-threadsafe ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${MUFFT_LIBRARIES})

add_library(sounder_module MODULE
    ${SOUNDER_SOURCES})

target_link_libraries(sounder_module houdini_sync -lpthread --enable-threadsafe ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    -Wl,--whole-archive
    ${MUFFT_LIBRARIES}
    -Wl,--no-whole-archive
    ${HDF5_LIBRARIES}
    ${SoapySDR_LIBRARIES})

# Radio-free unit test: the UE beacon-sync geometry across the whole rate
# ladder. Links NOTHING (sync_geometry.h is header-only and dependency-free) so
# it builds and runs anywhere, which is the point -- the two defects it covers
# were only visible by running a UE against silicon at one rate.
enable_testing()
# ---- houdini_sync: the UE synchronisation library ---------------------------
# One classed home for the beacon shapes, the detector, the confirm, the carrier
# estimator and their configuration (docs/SYNC_LIBRARY_ARCHITECTURE.md). Static,
# portable (x86 and aarch64), no dependency beyond what the sounder links.
# comms-lib and utils are compiled HERE and nowhere else: the sounder, the
# module and the sync tools all take them from this library, so there is one
# find_beacon_avx in the tree (review round 2, item 6).
add_library(houdini_sync STATIC
    sync/sync_config.cc
    sync/detector.cc
    comms-lib.cc comms-lib-avx.cc comms-lib-portable.cc utils.cc)
target_include_directories(houdini_sync PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Linked into sounder_module (a MODULE) as well as the executables.
set_target_properties(houdini_sync PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(houdini_sync PUBLIC ${MUFFT_DEFINITIONS})
target_link_libraries(houdini_sync PUBLIC ${MUFFT_LIBRARIES})

add_executable(sync_config_test tests/comms-func/sync_config_test.cc)
target_link_libraries(sync_config_test houdini_sync -lpthread)
add_test(NAME sync_config COMMAND sync_config_test)

add_executable(sync_config_schema tests/comms-func/sync_config_schema.cc)
target_link_libraries(sync_config_schema houdini_sync -lpthread)

add_executable(golden_window_test tests/comms-func/golden_window_test.cc)
target_link_libraries(golden_window_test houdini_sync -lpthread)
add_test(NAME golden_window COMMAND golden_window_test
         ${CMAKE_CURRENT_SOURCE_DIR}/tests/comms-func/fixtures/golden)

add_executable(sync_geometry_test tests/comms-func/sync_geometry_test.cc)
target_include_directories(sync_geometry_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME sync_geometry COMMAND sync_geometry_test)

# Radio-free A/B: AP-41's kalman against the shipped alpha-beta over the SAME
# synthetic detections. The rig can only run one at a time; this pairs them.
add_executable(grid_tracker_test tests/comms-func/grid_tracker_test.cc)
target_include_directories(grid_tracker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME grid_tracker COMMAND grid_tracker_test)

# The index-convention check AP-34(a) did not have. Links the real CommsLib so
# it exercises the REAL detector; a replica would agree with itself.
add_executable(beacon_geometry_test
    tests/comms-func/beacon_geometry_test.cc
    comms-lib.cc comms-lib-avx.cc comms-lib-portable.cc utils.cc)
target_include_directories(beacon_geometry_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(beacon_geometry_test -lpthread ${MUFFT_LIBRARIES})
add_test(NAME beacon_geometry COMMAND beacon_geometry_test)

# HIL helper: transmit the real Gold beacon from a Houdini board so a Houdini UE
# running the unmodified receiver.cc client-sync path can acquire it.
add_executable(beacon_shape_dump
    tests/comms-func/beacon_shape_dump.cc
    comms-lib.cc comms-lib-portable.cc utils.cc)
target_include_directories(beacon_shape_dump PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(beacon_shape_dump -lpthread ${MUFFT_LIBRARIES})

add_executable(beacon_tx_gold
    tests/comms-func/beacon_tx_gold.cc
    comms-lib.cc
    comms-lib-avx.cc
    comms-lib-portable.cc
    utils.cc)
target_include_directories(beacon_tx_gold PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(beacon_tx_gold -lpthread
    ${SoapySDR_LIBRARIES}
    ${MUFFT_LIBRARIES})

# HIL harness: drive both ends with the real BaseRadioSet (BS beacon) +
# ClientRadioSet (UE sync) -- proves the BaseRadioSet Houdini backend TXes the
# beacon the UE acquires, without the un-runnable BS receive loop.
add_executable(houdini_bs_ue_sync
    tests/comms-func/houdini_bs_ue_sync.cc
    ${SOUNDER_SOURCES})
target_include_directories(houdini_bs_ue_sync PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(houdini_bs_ue_sync houdini_sync -lpthread --enable-threadsafe ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${MUFFT_LIBRARIES})

# HIL closed-loop: BS beacon -> UE sync -> UE pilot -> BS receive, all through
# the real BaseRadioSet + ClientRadioSet over the bidirectional Houdini link.
add_executable(houdini_loopback
    tests/comms-func/houdini_loopback.cc
    ${SOUNDER_SOURCES})
target_include_directories(houdini_loopback PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(houdini_loopback houdini_sync -lpthread --enable-threadsafe ${GFLAGS_LIBRARIES} ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${MUFFT_LIBRARIES})
