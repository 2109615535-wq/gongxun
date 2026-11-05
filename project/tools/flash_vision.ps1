$ErrorActionPreference = 'Stop'
$projectDirectory = Split-Path -Parent $PSScriptRoot
$firmware = Join-Path $projectDirectory 'build\project.hex'
$openocd = Get-ChildItem -Path 'C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*mcu.externaltools.openocd*\tools\bin\openocd.exe' |
    Sort-Object FullName | Select-Object -Last 1
$scripts = Get-Item -Path 'C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*mcu.debug.openocd*\resources\openocd\st_scripts' |
    Sort-Object FullName | Select-Object -Last 1

if (-not (Test-Path -LiteralPath $firmware)) {
    throw 'Build the firmware with python project/tools/build_vision.py first.'
}
if (-not $openocd -or -not $scripts) {
    throw 'STM32CubeIDE OpenOCD and its target scripts were not found.'
}

Push-Location (Split-Path -Parent $firmware)
try {
    & $openocd.FullName -s $scripts.FullName -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg `
        -c 'adapter speed 1000' -c 'program project.hex verify reset exit'
    if ($LASTEXITCODE -ne 0) {
        throw 'CMSIS-DAP programming failed. Check the data USB cable, SWD wiring and target power.'
    }
} finally {
    Pop-Location
}
