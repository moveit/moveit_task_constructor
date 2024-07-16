cmake_minimum_required(VERSION 3.11) # for FetchContent
project(scope_guard)

# handle inclusions and dependencies
include(CheckCXXSymbolExists)
include(CheckCXXCompilerFlag)
include(GNUInstallDirs)


add_library(scope_guard INTERFACE)
add_library(scope_guard::scope_guard ALIAS scope_guard) # for consumption via add_subdirectory()

target_include_directories(scope_guard
  INTERFACE
    include # assuming we move the header file to an `include` directory
)
