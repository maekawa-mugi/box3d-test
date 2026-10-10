# Experimental PS2 Emotion Engine toolchain (ps2dev).
# Configure with: cmake -S . -B build/ps2-ee \
#   -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/ps2-ee.cmake -DBOX3D_PS2=ON
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR mips64r5900el)

# The modern ps2dev toolchain installs EE GCC as follows.
# A PATH installation is equally valid.
if(DEFINED ENV{PS2DEV})
    list(PREPEND CMAKE_PROGRAM_PATH "$ENV{PS2DEV}/ee/bin")
endif()
find_program(PS2_EE_GCC NAMES mips64r5900el-ps2-elf-gcc REQUIRED)
set(CMAKE_C_COMPILER "${PS2_EE_GCC}")
# The Box3D EE baseline uses C only; no C++ compiler is required here.

# Do not link a test executable during configure: ps2sdk link flags depend
# on the application and are unnecessary for a static-library build.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
