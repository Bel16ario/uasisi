# Find Boost (Math is header-only)
find_package(Boost QUIET)
if(NOT Boost_FOUND)
    message(STATUS "  ├─ randomOpt: SKIPPED (Boost not found)")
    return()
endif()
message(STATUS "  ├─ randomOpt: ENABLED ✓")

# Register sources
set(MODULE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/randomOpt.cpp)
list(APPEND UASISI_MODULE_SOURCES ${MODULE_SOURCES})
set(UASISI_MODULE_SOURCES ${UASISI_MODULE_SOURCES} PARENT_SCOPE)

# Register include directories (Boost.Math is header-only, no libraries needed)
list(APPEND UASISI_MODULE_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
set(UASISI_MODULE_INCLUDE_DIRS ${UASISI_MODULE_INCLUDE_DIRS} PARENT_SCOPE)
