from os import makedirs
from pathlib import PureWindowsPath
from shutil import rmtree
from subprocess import check_output
import platform

Import("env")

bc = env.BoardConfig()

if "m3" in bc.get("build.cpu"):
    core = 3
    ivt = 0
else:
    core = 0
    ivt = 0x150

rom_size = f"{bc.get('upload.maximum_size') // 1024}K"
ram_size = f"{bc.get('upload.maximum_ram_size') // 1024}K"

boot_size = env.GetProjectOption("board_bootloader_size", "")

env.Append(CCFLAGS=["-I", f"lib/freertos/portable/GCC/ARM_CM{core}",
                    f"-DFLASH_SIZE={bc.get('upload.maximum_size')}",
                    f"-DRAM_SIZE={bc.get('upload.maximum_ram_size')}",
                    f"-DRAM_{ram_size}",
                    f"-DIVT_SIZE={ivt}"])
if boot_size:
    env.Append(CCFLAGS=[f"-DFLASH_BOOTLDR_SIZE={boot_size}"])

env.Append(LINKFLAGS=[f"-Wl,--defsym=__rom_size__={rom_size}",
                      f"-Wl,--defsym=__ram_size__={ram_size}"])

if "bootloader" in env["PIOENV"]:
    env.Append(LINKFLAGS=["-Wl,--defsym=__boot_size__=0",
                          f"-Wl,--defsym=__ivt_size__=0"])
else:
    env.Append(LINKFLAGS=[f"-Wl,--defsym=__boot_size__={boot_size}",
                          f"-Wl,--defsym=__ivt_size__={ivt}"])

# mark these libs as system to ignore GCC warnings
env.Append(CCFLAGS=["-isystem", "lib/uip/uip",
                    "-isystem", "lib/matiec/lib/C"])

env.Append(CFLAGS=["-Wimplicit-function-declaration",
                   "-Wmissing-prototypes",
                   "-Wstrict-prototypes"])

def skip_from_build(node):
    n = node.get_path()
    if platform.system() == "Windows":
        n = PureWindowsPath(n).as_posix()

    # filter UIP src and matiec dir
    skip = ["uip/apps", "uip/unix", "uip/doc", "lib/matiec"]
    if any(s in n for s in skip):
        return None

    # filter freertos src
    keep = [f"GCC/ARM_CM{core}/", "MemMang/heap_4"]
    if "freertos/portable" in n:
        if not any(s in n for s in keep):
            return None

    # otherwise allow all
    return node

def fix_pous_c(node):
    # don't compile POUS.c
    if "POUS" not in node.name:
        return node

# Register callbacks
env.AddBuildMiddleware(skip_from_build, "*")
env.AddBuildMiddleware(fix_pous_c)

print("Compiling plc_prog.st ...")

gdir = f"{env['PROJECT_SRC_DIR']}/generated"
rmtree(gdir, ignore_errors=True)
makedirs(gdir, exist_ok=True)

path = env.PioPlatform().get_package_dir("tool-matiec")
cmd = "iec2c.exe" if platform.system() == "Windows" else "iec2c"

check_output(f"{path}/bin/{cmd} -I lib/matiec/lib -T src/generated plc_prog.st", shell=True)
