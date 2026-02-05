# modules/monitors/simpleLogger/CMakeLists.txt

# Find HDF5
find_package(HDF5 QUIET COMPONENTS CXX)

if(NOT HDF5_FOUND)
    message(STATUS "  ├─ simpleLogger: SKIPPED (HDF5 not found)")
    return()
endif()

message(STATUS "  ├─ simpleLogger: ENABLED ✓")

# Register sources
set(MODULE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/simpleLogger.cpp)
list(APPEND UASISI_MODULE_SOURCES ${MODULE_SOURCES})
set(UASISI_MODULE_SOURCES ${UASISI_MODULE_SOURCES} PARENT_SCOPE)

# Register HDF5 libraries
list(APPEND UASISI_MODULE_LIBRARIES ${HDF5_LIBRARIES})
set(UASISI_MODULE_LIBRARIES ${UASISI_MODULE_LIBRARIES} PARENT_SCOPE)

# Handle HighFive
if(NOT TARGET HighFive)
    add_subdirectory(
        ${PROJECT_SOURCE_DIR}/third_party/HighFive
        ${CMAKE_BINARY_DIR}/third_party/HighFive
    )
endif()

list(APPEND UASISI_MODULE_LIBRARIES HighFive)
set(UASISI_MODULE_LIBRARIES ${UASISI_MODULE_LIBRARIES} PARENT_SCOPE)

# Register include directories (NOT apply them)
list(APPEND UASISI_MODULE_INCLUDE_DIRS ${HDF5_INCLUDE_DIRS})
list(APPEND UASISI_MODULE_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/third_party/HighFive/include)
set(UASISI_MODULE_INCLUDE_DIRS ${UASISI_MODULE_INCLUDE_DIRS} PARENT_SCOPE)
