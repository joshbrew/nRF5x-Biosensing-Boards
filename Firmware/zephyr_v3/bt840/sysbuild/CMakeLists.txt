# Standard Zephyr sysbuild entry point, with local cache and board validation.
include(${CMAKE_CURRENT_LIST_DIR}/../board.cmake)
set(USER_CACHE_DIR "${CMAKE_BINARY_DIR}/.cache" CACHE PATH "Local compiler cache")
find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sysbuild LANGUAGES)
