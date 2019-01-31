cmake_minimum_required (VERSION 3.5 FATAL_ERROR)
project (ipmbbridged CXX)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-rtti")

# check for necessary headers / download if missing
enable_language (C)
include (CheckIncludeFile)
check_include_file ("linux/i2c-dev-user.h" header_present)
if (NOT header_present)
    message (
        "Could not find linux/i2c-dev-user.h: Attempting to download locally "
        "for building from https://raw.githubusercontent.com/openbmc/linux/"
        "dev-4.13/include/uapi/linux/i2c-dev.h"
    )

    string (CONCAT url "https://raw.githubusercontent.com/openbmc/linux/"
                   "dev-4.13/include/uapi/linux/i2c-dev.h")

    file (DOWNLOAD ${url} "linux/i2c-dev-user.h" SHOW_PROGRESS STATUS status)

    list (GET status 0 status_code)

    if (NOT status_code EQUAL 0)
        message (FATAL_ERROR "i2c-dev-user.h missing")
    endif ()
endif ()

include_directories (${CMAKE_CURRENT_SOURCE_DIR})
find_package (Boost REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})
add_definitions (-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions (-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions (-DBOOST_ALL_NO_LIB)
add_definitions (-DBOOST_NO_RTTI)
add_definitions (-DBOOST_NO_TYPEID)
add_definitions (-DBOOST_ASIO_DISABLE_THREADS)

set (SRC_FILES ipmbbridged.cpp ipmbutils.cpp)

# import libsystemd
find_package (PkgConfig REQUIRED)
pkg_check_modules (SYSTEMD libsystemd REQUIRED)

# import sdbusplus
find_package (PkgConfig REQUIRED)
pkg_check_modules (SDBUSPLUSPLUS sdbusplus REQUIRED)

# import phosphor-logging
find_package (PkgConfig REQUIRED)
pkg_check_modules (LOGGING phosphor-logging REQUIRED)

add_executable (${PROJECT_NAME} ${SRC_FILES})
target_link_libraries (${PROJECT_NAME} systemd)
target_link_libraries (${PROJECT_NAME} boost_coroutine)
target_link_libraries (${PROJECT_NAME} sdbusplus -lstdc++fs)
target_link_libraries (${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} phosphor_logging)
target_link_libraries(${PROJECT_NAME} i2c)

link_directories (${EXTERNAL_INSTALL_LOCATION}/lib)

set (SERVICE_FILES ${PROJECT_SOURCE_DIR}/ipmb.service)

install (TARGETS ${PROJECT_NAME} DESTINATION sbin)
install (FILES ${SERVICE_FILES} DESTINATION /lib/systemd/system/)
