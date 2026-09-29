cmake_minimum_required(VERSION 3.15)
project(GolfCartControl)

set(CMAKE_CXX_STANDARD 17)

# =============================================================================
#  CODEGEN — same path as original project
# =============================================================================
set(CODEGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/ekf_fusion_codegen)

file(GLOB CODEGEN_SOURCES
    "${CODEGEN_DIR}/*.c"
)

# =============================================================================
#  ISOLATED VERSION — single unified executable
# =============================================================================
add_executable(GolfCartControl
    main.cpp
    ui.cpp
    localization.cpp
    control.cpp
    planning.cpp
    communication.cpp
    steer_test.cpp
    loc_test.cpp
    ${CODEGEN_SOURCES}
)

target_include_directories(GolfCartControl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/raylib-5.0_win64_mingw-w64/include
    ${CODEGEN_DIR}
)

target_link_directories(GolfCartControl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/raylib-5.0_win64_mingw-w64/lib
)

target_link_libraries(GolfCartControl PRIVATE
    raylib
    ws2_32
    winmm
    gdi32
    iphlpapi
    shell32
)

set_source_files_properties(${CODEGEN_SOURCES} PROPERTIES
    LANGUAGE C
    COMPILE_FLAGS "-std=c99"
)

target_compile_options(GolfCartControl PRIVATE
    # -O2: compiler optimization for the whole app (was previously unoptimized).
    # Standard, IEEE-safe (no -ffast-math), no -DNDEBUG so asserts stay live.
    # Control loop is rate-limited (sleep_until), so its 50 Hz timing is unchanged.
    -O2
    $<$<COMPILE_LANGUAGE:C>:
        -Wno-unused-variable
        -Wno-unused-but-set-variable
    >
    # OpenMP — parallelizes the particle-filter scoring loop across cores
    # (localization.cpp). Only the #pragma-annotated loop is affected; all other
    # code is unchanged.
    $<$<COMPILE_LANGUAGE:CXX>:-fopenmp>
)
target_link_options(GolfCartControl PRIVATE -fopenmp -mwindows)
# Auto-copy exe to runtime_data after every build
add_custom_command(TARGET GolfCartControl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:GolfCartControl>
        "${CMAKE_SOURCE_DIR}/runtime_data/GolfCartControl.exe"
    COMMENT "Copying exe to runtime_data..."
)
# Bundle the MinGW runtime DLLs (libstdc++, libgcc_s_seh, libwinpthread, libgomp) next
# to the exe. Windows searches the application's own directory BEFORE PATH, so this
# guarantees the exe always loads the DLL versions it was actually built/linked
# against — regardless of what other MinGW toolchains (e.g. Git for Windows' bundled
# mingw64) might be earlier on the launching shell's PATH. Without this, a PATH that
# resolves to a different, ABI-incompatible libstdc++-6.dll causes an immediate
# segfault inside the C++ runtime (observed inside load_ply() at startup).
get_filename_component(CXX_COMPILER_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)
foreach(RUNTIME_DLL libgcc_s_seh-1.dll libgomp-1.dll libstdc++-6.dll libwinpthread-1.dll)
    if(EXISTS "${CXX_COMPILER_DIR}/${RUNTIME_DLL}")
        add_custom_command(TARGET GolfCartControl POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CXX_COMPILER_DIR}/${RUNTIME_DLL}"
                "${CMAKE_SOURCE_DIR}/runtime_data/${RUNTIME_DLL}"
            COMMENT "Bundling ${RUNTIME_DLL} into runtime_data..."
        )
    endif()
endforeach()
add_custom_command(TARGET GolfCartControl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/runtime_data/campus_bg.png"
        "${CMAKE_SOURCE_DIR}/runtime_data/campus_bg.png"
    COMMENT "campus_bg.png already in runtime_data..."
)