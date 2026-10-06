# Locate the Raspberry Pi Pico SDK.
# Uses PICO_SDK_PATH (env or cache); set PICO_SDK_FETCH_FROM_GIT=1 to download it.

if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif ()
if (DEFINED ENV{PICO_SDK_FETCH_FROM_GIT} AND (NOT PICO_SDK_FETCH_FROM_GIT))
    set(PICO_SDK_FETCH_FROM_GIT $ENV{PICO_SDK_FETCH_FROM_GIT})
endif ()

set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Path to the Raspberry Pi Pico SDK")
set(PICO_SDK_FETCH_FROM_GIT "${PICO_SDK_FETCH_FROM_GIT}" CACHE BOOL "Fetch the SDK from git if not found")

if (NOT PICO_SDK_PATH)
    if (PICO_SDK_FETCH_FROM_GIT)
        include(FetchContent)
        FetchContent_Declare(pico_sdk
            GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk
            GIT_TAG 2.1.1
            GIT_SUBMODULES_RECURSE FALSE)
        FetchContent_GetProperties(pico_sdk)
        if (NOT pico_sdk_POPULATED)
            message("Downloading Raspberry Pi Pico SDK")
            FetchContent_Populate(pico_sdk)
        endif ()
        set(PICO_SDK_PATH ${pico_sdk_SOURCE_DIR})
    else ()
        message(FATAL_ERROR "Set PICO_SDK_PATH, or PICO_SDK_FETCH_FROM_GIT=1 to download the SDK")
    endif ()
endif ()

get_filename_component(PICO_SDK_PATH "${PICO_SDK_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${PICO_SDK_PATH}/pico_sdk_init.cmake)
    message(FATAL_ERROR "Directory '${PICO_SDK_PATH}' is not a Pico SDK")
endif ()

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
