find_package(Python 3.8 COMPONENTS Interpreter Development QUIET)
find_package(pybind11 CONFIG QUIET)

if(NOT Python_FOUND)
    message(STATUS "  ├─ pythonWrapper: SKIPPED (Python not found)")
    return()
endif()

if(NOT pybind11_FOUND)
    message(STATUS "  ├─ pythonWrapper: SKIPPED (pybind11 not found)")
    message(STATUS "  │  └─ Install with: pip install pybind11")
    return()
endif()

message(STATUS "  ├─ pythonWrapper: ENABLED ✓")
message(STATUS "  │  ├─ Python: ${Python_VERSION}")
message(STATUS "  │  └─ pybind11: ${pybind11_VERSION}")

set(MODULE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/pythonWrapper.cpp)
list(APPEND UASISI_MODULE_SOURCES ${MODULE_SOURCES})
set(UASISI_MODULE_SOURCES ${UASISI_MODULE_SOURCES} PARENT_SCOPE)

list(APPEND UASISI_MODULE_LIBRARIES pybind11::embed)
set(UASISI_MODULE_LIBRARIES ${UASISI_MODULE_LIBRARIES} PARENT_SCOPE)

add_compile_definitions(UASISI_HAS_PYTHON_MODULE)
