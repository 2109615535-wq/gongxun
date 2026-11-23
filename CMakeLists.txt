cmake_minimum_required(VERSION 3.22)
project(gongxun1 LANGUAGES C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(firmware_dir "${CMAKE_CURRENT_SOURCE_DIR}/project")
set(core_dir "${firmware_dir}/Core")
set(drivers_dir "${firmware_dir}/Drivers")
set(hal_dir "${drivers_dir}/STM32F4xx_HAL_Driver")
set(cmsis_dir "${drivers_dir}/CMSIS")
set(device_dir "${cmsis_dir}/Device/ST/STM32F4xx")
set(linker_script "${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32F407xx_FLASH.ld")

# Keep the firmware sources in sync with project/MDK-ARM/project.uvprojx.
add_executable(gongxun1
    "${core_dir}/Src/OLED.c"
    "${core_dir}/Src/OLED_Data.c"
    "${core_dir}/Src/decode_move.c"
    "${core_dir}/Src/encoder.c"
    "${core_dir}/Src/faraway.c"
    "${core_dir}/Src/field_map.c"
    "${core_dir}/Src/hdz601.c"
    "${core_dir}/Src/hui_ku.c"
    "${core_dir}/Src/imu.c"
    "${core_dir}/Src/key.c"
    "${core_dir}/Src/main.c"
    "${core_dir}/Src/motor.c"
    "${core_dir}/Src/pid.c"
    "${core_dir}/Src/scheduler.c"
    "${core_dir}/Src/servo.c"
    "${core_dir}/Src/stm32f4xx_hal_msp.c"
    "${core_dir}/Src/stm32f4xx_it.c"
    "${core_dir}/Src/system_stm32f4xx.c"
    "${core_dir}/Src/vision.c"
    "${core_dir}/Src/vision_simple.c"
    "${hal_dir}/Src/stm32f4xx_hal.c"
    "${hal_dir}/Src/stm32f4xx_hal_cortex.c"
    "${hal_dir}/Src/stm32f4xx_hal_dma.c"
    "${hal_dir}/Src/stm32f4xx_hal_dma_ex.c"
    "${hal_dir}/Src/stm32f4xx_hal_exti.c"
    "${hal_dir}/Src/stm32f4xx_hal_flash.c"
    "${hal_dir}/Src/stm32f4xx_hal_flash_ex.c"
    "${hal_dir}/Src/stm32f4xx_hal_flash_ramfunc.c"
    "${hal_dir}/Src/stm32f4xx_hal_gpio.c"
    "${hal_dir}/Src/stm32f4xx_hal_pwr.c"
    "${hal_dir}/Src/stm32f4xx_hal_pwr_ex.c"
    "${hal_dir}/Src/stm32f4xx_hal_rcc.c"
    "${hal_dir}/Src/stm32f4xx_hal_rcc_ex.c"
    "${hal_dir}/Src/stm32f4xx_hal_spi.c"
    "${hal_dir}/Src/stm32f4xx_hal_tim.c"
    "${hal_dir}/Src/stm32f4xx_hal_tim_ex.c"
    "${hal_dir}/Src/stm32f4xx_hal_uart.c"
    "${device_dir}/Source/Templates/gcc/startup_stm32f407xx.s"
    cmake/syscalls.c
    cmake/sysmem.c
)

set_target_properties(gongxun1 PROPERTIES
    SUFFIX ".elf"
    LINK_DEPENDS "${linker_script}"
    ADDITIONAL_CLEAN_FILES "gongxun1.map"
)
target_compile_features(gongxun1 PRIVATE c_std_11)
target_compile_definitions(gongxun1 PRIVATE USE_HAL_DRIVER STM32F407xx)
target_include_directories(gongxun1 PRIVATE
    "${core_dir}/Inc"
    "${hal_dir}/Inc"
    "${hal_dir}/Inc/Legacy"
    "${device_dir}/Include"
    "${cmsis_dir}/Include"
)

set(cpu_flags -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_options(gongxun1 PRIVATE
    ${cpu_flags}
    $<$<COMPILE_LANGUAGE:C>:-ffunction-sections>
    $<$<COMPILE_LANGUAGE:C>:-fdata-sections>
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Release>:-Os>
)
target_link_options(gongxun1 PRIVATE
    ${cpu_flags}
    --specs=nano.specs
    # OLED output and UART telemetry use floating-point printf formats.
    -u _printf_float
    "-T${linker_script}"
    LINKER:--gc-sections
    LINKER:-Map,gongxun1.map
    LINKER:--print-memory-usage
)
target_link_libraries(gongxun1 PRIVATE m)

add_custom_command(TARGET gongxun1 POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:gongxun1>" gongxun1.hex
    BYPRODUCTS gongxun1.hex
    VERBATIM
)
