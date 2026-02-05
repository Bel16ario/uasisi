# modules/pythonWrapper/CMakeLists.txt

message(STATUS "  ├─ Checking pythonWrapper dependencies...")

find_package(Python 3.8 COMPONENTS Interpreter Development QUIET)

if(NOT Python_FOUND)
    message(STATUS "  ├─ pythonWrapper: SKIPPED (Python not found)")
    message(STATUS "  │  └─ Activate venv: source pytorch-env/bin/activate")
    return()
endif()

message(STATUS "  │  ├─ Python: ${Python_VERSION}")
message(STATUS "  │  │  └─ Executable: ${Python_EXECUTABLE}")

find_package(pybind11 CONFIG QUIET)

if(NOT pybind11_FOUND)
    message(STATUS "  ├─ pythonWrapper: SKIPPED (pybind11 not found)")
    message(STATUS "  │  └─ Install: pip install pybind11")
    return()
endif()

message(STATUS "  │  ├─ pybind11: ${pybind11_VERSION}")

# Check for PyTorch (optional)
execute_process(
    COMMAND ${Python_EXECUTABLE} -c "import torch; print(torch.__version__)"
    OUTPUT_VARIABLE PYTORCH_VERSION
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(PYTORCH_VERSION)
    message(STATUS "  │  ├─ PyTorch: ${PYTORCH_VERSION}")
    # PyTorch flag - needs to be set at parent scope
    set(UASISI_HAS_PYTORCH TRUE PARENT_SCOPE)
else()
    message(STATUS "  │  ├─ PyTorch: not found (optional)")
    set(UASISI_HAS_PYTORCH FALSE PARENT_SCOPE)
endif()

message(STATUS "  ├─ pythonWrapper: ENABLED ✓")

# Register module sources
set(MODULE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/pythonWrapper.cpp)
list(APPEND UASISI_MODULE_SOURCES ${MODULE_SOURCES})
set(UASISI_MODULE_SOURCES ${UASISI_MODULE_SOURCES} PARENT_SCOPE)

# Add pybind11::embed for embedding Python interpreter
list(APPEND UASISI_MODULE_LIBRARIES pybind11::embed)
set(UASISI_MODULE_LIBRARIES ${UASISI_MODULE_LIBRARIES} PARENT_SCOPE)

# CRITICAL: Set compile definition at PARENT scope so it affects uasisi_modules target
set(UASISI_PYTHON_ENABLED TRUE PARENT_SCOPE)
