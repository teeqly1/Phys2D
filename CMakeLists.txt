cmake_minimum_required(VERSION 3.16)
project(phys2d LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ---------------------------------------------------------------- опции
option(PHYS2D_USE_TBB      "Использовать Intel TBB для широкой фазы" OFF)
option(PHYS2D_SIMD_SSE     "Включить SSE2-ветки векторной математики" OFF)
option(PHYS2D_SIMD_AVX     "Включить AVX-ветки векторной математики" OFF)
option(PHYS2D_FMA          "Включить FMA (-mfma)" OFF)
option(PHYS2D_NATIVE       "-march=native" ON)
option(PHYS2D_BIG_BINARY   "Встраивать таблицы данных (бинарник >= 30 МБ)" ON)
option(PHYS2D_DEBUG_ASSERTS "Ассерты и трассировка ошибок" OFF)
set(PHYS2D_TABLE_MB 34 CACHE STRING "Размер встроенных таблиц в МБ")

# ---------------------------------------------------------------- флаги
add_library(phys2d_flags INTERFACE)
target_compile_options(phys2d_flags INTERFACE
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-funroll-loops>
    -fno-math-errno
    -Wall -Wextra -Wno-unused-parameter)
if(PHYS2D_NATIVE)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(-march=native PHYS2D_HAS_NATIVE)
    if(PHYS2D_HAS_NATIVE)
        target_compile_options(phys2d_flags INTERFACE -march=native)
    endif()
endif()
if(PHYS2D_SIMD_AVX)
    target_compile_definitions(phys2d_flags INTERFACE PHYS2D_SIMD_AVX=1)
    target_compile_options(phys2d_flags INTERFACE -mavx)
elseif(PHYS2D_SIMD_SSE)
    target_compile_definitions(phys2d_flags INTERFACE PHYS2D_SIMD_SSE=1)
    target_compile_options(phys2d_flags INTERFACE -msse2)
endif()
if(PHYS2D_FMA)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(-mfma PHYS2D_HAS_FMA)
    if(PHYS2D_HAS_FMA)
        target_compile_definitions(phys2d_flags INTERFACE PHYS2D_FMA=1)
        target_compile_options(phys2d_flags INTERFACE -mfma)
    endif()
endif()
if(PHYS2D_DEBUG_ASSERTS)
    target_compile_definitions(phys2d_flags INTERFACE PHYS2D_DEBUG_ASSERTS=1)
endif()

# ---------------------------------------------------------------- встроенные таблицы
set(PHYS2D_EXTRA_SOURCES "")
if(PHYS2D_BIG_BINARY)
    set(PHYS2D_BLOB "${CMAKE_BINARY_DIR}/phys2d_tables.bin")
    add_custom_command(
        OUTPUT  ${PHYS2D_BLOB}
        COMMAND ${CMAKE_COMMAND}
                -DOUT_FILE=${PHYS2D_BLOB}
                -DSIZE_MB=${PHYS2D_TABLE_MB}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateBlob.cmake
        COMMENT "Генерация встроенных таблиц данных (${PHYS2D_TABLE_MB} МБ)"
        VERBATIM)
    add_custom_target(phys2d_blob DEPENDS ${PHYS2D_BLOB})
    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/embedded_tables.S
        PROPERTIES OBJECT_DEPENDS ${PHYS2D_BLOB}
                   COMPILE_OPTIONS "-I${CMAKE_BINARY_DIR}")
    list(APPEND PHYS2D_EXTRA_SOURCES src/embedded_tables.S)
endif()

# ---------------------------------------------------------------- библиотека
set(PHYS2D_SOURCES
    src/Shape.cpp
    src/Body.cpp
    src/Collision.cpp
    src/Constraints.cpp
    src/BroadPhase.cpp
    src/Profiler.cpp
    src/DebugDraw.cpp
    src/Serialization.cpp
    src/World.cpp
    src/c_api.cpp
    src/template_instantiations.cpp
    # ---- v2 (Extras)
    src/Extras_core.cpp
    src/Extras_joints.cpp
    src/Extras_sim.cpp
    src/Extras_geometry.cpp
    src/Extras_worldio.cpp
    src/Extras_runtime.cpp
    src/Blood.cpp
    ${PHYS2D_EXTRA_SOURCES})

add_library(phys2d STATIC ${PHYS2D_SOURCES})
target_include_directories(phys2d PUBLIC include)
target_link_libraries(phys2d PUBLIC phys2d_flags)
find_package(Threads REQUIRED)
target_link_libraries(phys2d PUBLIC Threads::Threads)
if(PHYS2D_BIG_BINARY)
    add_dependencies(phys2d phys2d_blob)
endif()

add_library(phys2d_shared SHARED ${PHYS2D_SOURCES})
target_include_directories(phys2d_shared PUBLIC include)
target_link_libraries(phys2d_shared PUBLIC phys2d_flags Threads::Threads)
set_target_properties(phys2d_shared PROPERTIES OUTPUT_NAME phys2d)
if(PHYS2D_BIG_BINARY)
    add_dependencies(phys2d_shared phys2d_blob)
endif()

if(PHYS2D_USE_TBB)
    find_package(TBB REQUIRED)
    target_compile_definitions(phys2d        PUBLIC PHYS2D_USE_TBB=1)
    target_compile_definitions(phys2d_shared PUBLIC PHYS2D_USE_TBB=1)
    target_link_libraries(phys2d        PUBLIC TBB::tbb)
    target_link_libraries(phys2d_shared PUBLIC TBB::tbb)
endif()

# ---------------------------------------------------------------- тестовая сцена
add_executable(phys2d_test_scene examples/test_scene.cpp)
target_link_libraries(phys2d_test_scene PRIVATE phys2d)

add_executable(phys2d_c_example examples/c_example.c)
target_link_libraries(phys2d_c_example PRIVATE phys2d)

# ---------------------------------------------------------------- сцена в реальном времени
add_executable(phys2d_realtime examples/realtime_scene.cpp)
target_link_libraries(phys2d_realtime PRIVATE phys2d)
if(WIN32)
    target_link_libraries(phys2d_realtime PRIVATE gdi32 user32)
endif()

# ---------------------------------------------------------------- демо v2 + бенчмарки
add_executable(phys2d_demo_v2 examples/demo_v2.cpp)
target_link_libraries(phys2d_demo_v2 PRIVATE phys2d)

# ---------------------------------------------------------------- GORE LAB (sandbox game)
add_executable(phys2d_sandbox examples/sandbox_game.cpp)
target_link_libraries(phys2d_sandbox PRIVATE phys2d)
if(WIN32)
    target_link_libraries(phys2d_sandbox PRIVATE gdi32 user32)
endif()

# ---------------------------------------------------------------- GORE LAB (super-optimized renderer)
add_executable(phys2d_sandbox_opt examples/sandbox_opt.cpp)
target_link_libraries(phys2d_sandbox_opt PRIVATE phys2d)
if(WIN32)
    target_link_libraries(phys2d_sandbox_opt PRIVATE gdi32 user32)
endif()

install(TARGETS phys2d phys2d_shared ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/phys2d DESTINATION include)

# ---------------------------------------------------------------- GoreLab Remake
add_executable(phys2d_gorelab examples/gorelab_remake.cpp)
target_link_libraries(phys2d_gorelab PRIVATE phys2d phys2d_flags)
if(WIN32)
    target_link_libraries(phys2d_gorelab PRIVATE gdi32 user32)
    set_target_properties(phys2d_gorelab PROPERTIES WIN32_EXECUTABLE TRUE)
endif()
