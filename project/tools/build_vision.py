"""Build the Keil project's C sources with an installed STM32 ARM GCC toolchain."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[1]


def find_toolchain(directory):
    executable = "arm-none-eabi-gcc.exe" if os.name == "nt" else "arm-none-eabi-gcc"
    if directory:
        candidate = Path(directory) / executable
    elif shutil.which(executable):
        candidate = Path(shutil.which(executable))
    else:
        candidates = sorted(Path("C:/ST").glob(
            "STM32CubeIDE_*/STM32CubeIDE/plugins/*gnu-tools*/tools/bin/arm-none-eabi-gcc.exe"))
        if not candidates:
            raise SystemExit("Set ARM_GCC_BIN to your ARM GCC bin directory, or use --gcc-bin.")
        candidate = candidates[-1]
    if not candidate.is_file():
        raise SystemExit(f"Compiler not found: {candidate}")
    return candidate.parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc-bin", default=os.environ.get("ARM_GCC_BIN"))
    args = parser.parse_args()
    toolchain = find_toolchain(args.gcc_bin)
    suffix = ".exe" if os.name == "nt" else ""
    gcc = toolchain / ("arm-none-eabi-gcc" + suffix)
    output = PROJECT / "build"
    output.mkdir(exist_ok=True)
    manifest = PROJECT / "MDK-ARM/project.uvprojx"
    target = ET.parse(manifest).find(".//Target")
    controls = target.find(".//Cads/VariousControls")
    includes = [(manifest.parent / path).resolve() for path in controls.findtext("IncludePath").split(";")]
    defines = controls.findtext("Define").split(",")
    sources = [(manifest.parent / file.findtext("FilePath")).resolve()
               for file in target.findall("./Groups/Group/Files/File")
               if file.findtext("FileType") == "1"]
    startup = PROJECT / "Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s"
    sources.append(startup)
    common = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard",
              "-Og", "-g3", "-ffunction-sections", "-fdata-sections", "-fstack-usage"]
    compile_options = ["-std=gnu99", "-Wall", "-Wextra"]
    compile_options += ["-D" + define for define in defines]
    compile_options += ["-I" + str(path) for path in includes]
    objects = []
    for source in sources:
        destination = output / (source.stem + ".o")
        if destination in objects:
            raise SystemExit(f"Duplicate source name in project: {source.name}")
        print("Compiling", source.name, flush=True)
        command = [str(gcc), *common]
        if source.suffix.lower() == ".c":
            command += compile_options
        subprocess.run([*command, "-c", str(source), "-o", str(destination)], cwd=output, check=True)
        objects.append(destination)
    firmware = output / "project.elf"
    command = [str(gcc), *common, *map(str, objects), "-T", str(PROJECT / "tools/stm32f407vetx.ld"),
               "--specs=nano.specs", "--specs=nosys.specs", "-Wl,--gc-sections", "-Wl,--print-memory-usage",
               "-Wl,-Map=" + str(output / "project.map"), "-Wl,-u,_printf_float",
               "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group", "-o", str(firmware)]
    subprocess.run(command, cwd=output, check=True)
    subprocess.run([str(toolchain / ("arm-none-eabi-objcopy" + suffix)), "-O", "ihex",
                    str(firmware), str(output / "project.hex")], check=True)
    subprocess.run([str(toolchain / ("arm-none-eabi-size" + suffix)), str(firmware)], check=True)
    print("Firmware:", output / "project.hex")


if __name__ == "__main__":
    main()
