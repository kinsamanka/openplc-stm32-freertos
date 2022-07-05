from shutil import rmtree
from subprocess import check_output
import platform

Import("env")

# mark these libs as system to ignore GCC warnings
env.Append(CCFLAGS=["-isystem", "lib/uip/uip",
                    "-isystem", "lib/matiec/lib/C"])

env.Append(CFLAGS=["-Wimplicit-function-declaration",
                   "-Wmissing-prototypes",
                   "-Wstrict-prototypes"])

def skip_from_build(node):
    n = node.get_path()

    # filter UIP src and matiec dir
    skip = ["uip/apps", "uip/unix", "uip/doc", "lib/matiec"]
    if any(s in n for s in skip):
        return None

    # filter freertos src
    keep = ["GCC/ARM_CM3/", "MemMang/heap_4"]
    if "freertos/portable" in n:
        if not any(s in n for s in keep):
            return None

    # otherwise allow all
    return node

# Register callback
env.AddBuildMiddleware(skip_from_build, "*")

print("Compiling plc_prog.st ...")

rmtree("src/generated/*", ignore_errors=True)

path = env.PioPlatform().get_package_dir("tool-matiec")
cmd = "iec2c.exe" if platform.system() == "Windows" else "iec2c"

check_output(f"{path}/bin/{cmd} -I lib/matiec/lib -T src/generated plc_prog.st", shell=True)
