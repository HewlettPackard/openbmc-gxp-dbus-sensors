cmake_minimum_required (VERSION 2.8.10 FATAL_ERROR)
project (gxp-dbus-sensors)


set (BUILD_SHARED_LIBRARIES OFF)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -lstdc++fs \
    -Werror \
    -Wall \
    -Wextra \
    -Wnon-virtual-dtor \
    -Wold-style-cast \
    -Wcast-align \
    -Wunused \
    -Woverloaded-virtual \
    -Wpedantic \
    -Wmisleading-indentation \
    -Wduplicated-cond \
    -Wduplicated-branches \
    -Wlogical-op \
    -Wnull-dereference \
    -Wuseless-cast \
    -Wdouble-promotion \
    -Wformat=2 \
    -Wno-sign-compare \
    -Wno-reorder \
"
)

add_definitions (-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions (-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions (-DBOOST_ALL_NO_LIB)
add_definitions (-DBOOST_NO_RTTI)
add_definitions (-DBOOST_NO_TYPEID)
add_definitions (-DBOOST_ASIO_DISABLE_THREADS)

link_directories (${EXTERNAL_INSTALL_LOCATION}/lib)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)
set (SERVICE_FILE_SRC_DIR ${PROJECT_SOURCE_DIR}/service_files)
set (SERVICE_FILE_INSTALL_DIR /lib/systemd/system/)
set (SENSOR_LINK_LIBS -lsystemd stdc++fs sdbusplus)

set (GXPTEMPSENSOR_SRC_FILES src/GxpTempSensor.cpp src/Utils.cpp src/Thresholds.cpp)
add_executable (gxptempsensor ${GXPTEMPSENSOR_SRC_FILES})
add_dependencies (gxptempsensor sdbusplus-project)
target_link_libraries (gxptempsensor ${SENSOR_LINK_LIBS})
install (TARGETS gxptempsensor RUNTIME DESTINATION bin)

install (FILES ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.GxpTempSensor.service DESTINATION ${SERVICE_FILE_INSTALL_DIR})

set (GXPFANSENSOR_SRC_FILES src/GxpFanMain.cpp src/GxpPwmSensor.cpp src/GxpTachSensor.cpp src/Utils.cpp src/Thresholds.cpp)
add_executable (gxpfansensor ${GXPFANSENSOR_SRC_FILES})
add_dependencies (gxpfansensor sdbusplus-project)
target_link_libraries (gxpfansensor ${SENSOR_LINK_LIBS})
target_link_libraries (gxpfansensor gpiodcxx)
install (TARGETS gxpfansensor RUNTIME DESTINATION bin)
install (FILES ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.GxpFanSensor.service DESTINATION ${SERVICE_FILE_INSTALL_DIR})
