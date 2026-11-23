set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(CMAKE_C_COMPILER NAMES arm-none-eabi-gcc REQUIRED)
set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
get_filename_component(arm_gcc_dir "${CMAKE_C_COMPILER}" DIRECTORY)
find_program(CMAKE_OBJCOPY NAMES arm-none-eabi-objcopy HINTS "${arm_gcc_dir}" REQUIRED)
