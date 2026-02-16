cmake_minimum_required(VERSION 3.15)
project(uasisi VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wno-attributes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(UASISI_PYTHON_ENABLED FALSE)
set(UASISI_HAS_PYTORCH FALSE)

if(DEFINED ENV{VIRTUAL_ENV})
    message(STATUS "Detected Python venv: $ENV{VIRTUAL_ENV}")
    set(Python_ROOT_DIR "$ENV{VIRTUAL_ENV}")
    set(Python_FIND_VIRTUALENV FIRST)
    set(Python_FIND_STRATEGY LOCATION)
endif()
# ============================================================================
# GENERIC OPTIONS (No module-specific knowledge)
# ============================================================================

option(UASISI_AUTO_DISCOVER_MODULES "Automatically discover all modules" ON)

# Generic skip list - user can skip any module by name
set(UASISI_SKIP_MODULES "" CACHE STRING "Semicolon-separated list of modules to skip")

# ============================================================================
# FIND CORE DEPENDENCIES (Always required by everyone)
# ============================================================================

find_package(GSL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/modules
    ${HDF5_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/third_party/HighFive/include
)

# ============================================================================
# CORE LIBRARY (Orchestrator, types, config)
# ============================================================================

add_library(uasisi_core
    src/core/orchestrator.cpp
    src/io/config.cpp
)

target_link_libraries(uasisi_core
    PUBLIC
    Eigen3::Eigen
    GSL::gsl
    GSL::gslcblas
)

# ============================================================================
# MODULE TRACKING (Modules will populate these)
# ============================================================================

# Initialize empty lists that modules will append to
set(UASISI_MODULE_SOURCES "" CACHE INTERNAL "List of module source files")
set(UASISI_MODULE_LIBRARIES "" CACHE INTERNAL "List of module library dependencies")
set(UASISI_MODULE_INCLUDE_DIRS "" CACHE INTERNAL "List of module include directories")

# ============================================================================
# GENERIC MODULE DISCOVERY
# ============================================================================

# Function to check if module should be skipped
function(should_skip_module MODULE_NAME RESULT_VAR)
    if("${MODULE_NAME}" IN_LIST UASISI_SKIP_MODULES)
        set(${RESULT_VAR} TRUE PARENT_SCOPE)
    else()
        set(${RESULT_VAR} FALSE PARENT_SCOPE)
    endif()
endfunction()

# Function to register a module (completely generic)
function(register_module MODULE_NAME MODULE_PATH)
    # Check if module is in skip list
    should_skip_module(${MODULE_NAME} SKIP_MODULE)
    if(SKIP_MODULE)
        message(STATUS "Skipping module: ${MODULE_NAME} (in skip list)")
        return()
    endif()
    
    # Check if module has its own CMakeLists.txt
    if(EXISTS ${MODULE_PATH}/CMakeLists.txt)
        message(STATUS "Loading module: ${MODULE_NAME}")
        add_subdirectory(${MODULE_PATH} ${CMAKE_BINARY_DIR}/modules/${MODULE_NAME})
        
        # Propagate Python flags from subdirectory to parent scope
        if(UASISI_PYTHON_ENABLED)
            set(UASISI_PYTHON_ENABLED TRUE PARENT_SCOPE)
        endif()
        if(UASISI_HAS_PYTORCH)
            set(UASISI_HAS_PYTORCH TRUE PARENT_SCOPE)
        endif()
        
        # Propagate module tracking variables
        set(UASISI_MODULE_SOURCES ${UASISI_MODULE_SOURCES} PARENT_SCOPE)
        set(UASISI_MODULE_LIBRARIES ${UASISI_MODULE_LIBRARIES} PARENT_SCOPE)
        set(UASISI_MODULE_INCLUDE_DIRS ${UASISI_MODULE_INCLUDE_DIRS} PARENT_SCOPE)
    else()
        # No CMakeLists.txt - use default behavior (just compile .cpp files)
        file(GLOB_RECURSE MODULE_SRC "${MODULE_PATH}/*.cpp")
        if(MODULE_SRC)
            list(APPEND UASISI_MODULE_SOURCES ${MODULE_SRC})
            set(UASISI_MODULE_SOURCES ${UASISI_MODULE_SOURCES} PARENT_SCOPE)
            message(STATUS "Module ${MODULE_NAME}: auto-compiled (no CMakeLists.txt)")
        endif()
    endif()
endfunction()

# ============================================================================
# AUTO-DISCOVER ALL MODULES (Completely generic - no module names hardcoded)
# ============================================================================

if(UASISI_AUTO_DISCOVER_MODULES)
    message(STATUS "")
    message(STATUS "=== Auto-discovering modules ===")
    
    # List of module categories to scan
    set(MODULE_CATEGORIES physics control actuators sensors optimizer monitors modules)
    
    foreach(CATEGORY ${MODULE_CATEGORIES})
        set(CATEGORY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules/${CATEGORY})
        
        if(EXISTS ${CATEGORY_PATH})
            file(GLOB MODULE_DIRS ${CATEGORY_PATH}/*)
            
            foreach(MODULE_DIR ${MODULE_DIRS})
                if(IS_DIRECTORY ${MODULE_DIR})
                    get_filename_component(MODULE_NAME ${MODULE_DIR} NAME)
                    register_module(${MODULE_NAME} ${MODULE_DIR})
                endif()
            endforeach()
        endif()
    endforeach()
    
    message(STATUS "=== Module discovery complete ===")
    message(STATUS "")
endif()

if(UASISI_PYTHON_ENABLED)
    find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
    find_package(pybind11 CONFIG REQUIRED)
    message(STATUS "Python dependencies loaded in root scope for module linking")
endif()

# ============================================================================
# MODULES LIBRARY (Built from whatever modules registered)
# ============================================================================

# Only create library if we have sources
if(UASISI_MODULE_SOURCES)

    set(ALL_MODULE_SOURCES ${UASISI_MODULE_SOURCES})
    if(UASISI_PYTHON_ENABLED)
        list(APPEND ALL_MODULE_SOURCES ${PROJECT_SOURCE_DIR}/src/utils/pybindhelper.cpp)
    endif()

    add_library(uasisi_modules OBJECT ${ALL_MODULE_SOURCES})
    
    target_link_libraries(uasisi_modules
        PUBLIC
        uasisi_core
        GSL::gsl
        GSL::gslcblas
        Eigen3::Eigen
        ${UASISI_MODULE_LIBRARIES}  # Whatever modules added
    )
    
    target_include_directories(uasisi_modules
        PUBLIC
        ${PROJECT_SOURCE_DIR}/modules
        ${UASISI_MODULE_INCLUDE_DIRS}
    )
    if(UASISI_PYTHON_ENABLED)
        target_compile_definitions(uasisi_modules PUBLIC UASISI_HAS_PYTHON_MODULE)
        message(STATUS "Python module enabled - adding UASISI_HAS_PYTHON_MODULE definition")
        
        if(UASISI_HAS_PYTORCH)
            target_compile_definitions(uasisi_modules PUBLIC UASISI_HAS_PYTORCH)
            message(STATUS "PyTorch enabled - adding UASISI_HAS_PYTORCH definition")
        endif()
    endif()
else()
    message(WARNING "No module sources found!")
endif()

# ============================================================================
# EXECUTABLES
# ============================================================================
message(STATUS "DEBUG: UASISI_PYTHON_ENABLED = ${UASISI_PYTHON_ENABLED}")
message(STATUS "DEBUG: UASISI_HAS_PYTORCH = ${UASISI_HAS_PYTORCH}")
message(STATUS "DEBUG: TARGET uasisi_modules exists = $<TARGET_EXISTS:uasisi_modules>")
add_executable(reinforcementLearning
    examples/reinforcementLearning/reinforcementLearning.cpp
)

if(TARGET uasisi_modules)
    target_link_libraries(reinforcementLearning
        uasisi_core
        uasisi_modules
    )
else()
    target_link_libraries(reinforcementLearning
        uasisi_core
    )
endif()

add_executable(simpleRun
    examples/simpleRun/simpleRun.cpp
)

if(TARGET uasisi_modules)
    target_link_libraries(simpleRun
        uasisi_core
        uasisi_modules
    )
else()
    target_link_libraries(simpleRun
        uasisi_core
    )
endif()

# ============================================================================
# INSTALLATION
# ============================================================================

install(TARGETS uasisi_core
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

if(TARGET uasisi_modules)
    install(TARGETS uasisi_modules
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
endif()

install(DIRECTORY include/uasisi
    DESTINATION include
)

# ============================================================================
# SUMMARY
# ============================================================================

message(STATUS "")
message(STATUS "=================================")
message(STATUS "UASISI Configuration Summary")
message(STATUS "=================================")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Auto-discover: ${UASISI_AUTO_DISCOVER_MODULES}")
if(UASISI_SKIP_MODULES)
    message(STATUS "Skipped modules: ${UASISI_SKIP_MODULES}")
endif()
message(STATUS "=================================")
message(STATUS "")
