import struct

Import("env")

boot_size = env.GetProjectOption("board_bootloader_size", "")
if boot_size:
    boot_size = int(boot_size, 0)
else:
    boot_size = int("0x1000", 0)

if "bootloader" not in env["PIOENV"]:
    start = 0x08000000 + boot_size
else:
    start = 0x08000000

if (env['UPLOAD_PROTOCOL'] == 'serial') & ("bootloader" not in env["PIOENV"]):
    l = env["UPLOADERFLAGS"]
    # update bootloader start
    l = list(map(lambda x: x.replace('0x08000000', hex(start)), l))
    # fix platformio bug
    l = list(map(lambda x: x.replace('-g', '-S'), l))
    l.insert(0, '-g')
    l.insert(1, '0')

    env["UPLOADERFLAGS"] = l

def before_upload(source, target, env):
    if (env['UPLOAD_PROTOCOL'] == 'serial') & ("bootloader" not in env["PIOENV"]):
        # send magic string
        from serial import Serial

        serial = Serial(env['UPLOAD_PORT'], env['UPLOAD_SPEED'], timeout = 1, parity='E')
        serial.write(b'\xff\xffBOOTLOADER\xff\xff')
        serial.close()

def calculate_checksum(source, target, env):
    """
    source: https://github.com/davidgfnet/stm32-dfu-bootloader/checksum.py
    """

    if env["PIOENV"] == "bootloader":
        return

    fname = target[0].get_abspath()
    fwbin = open(fname, "rb").read()

    # Ensure the firmware is word size aligned
    print("Firmware size", len(fwbin))
    while len(fwbin) % 4 != 0:
        fwbin += b"\x00"
    print("Firmware size after padding", len(fwbin))

    fwlen = len(fwbin)

    print("Firmware size for checksum purposes", fwlen)

    # Patch 0x1C with zero, 0x20 with the FW size too
    sizestr = struct.pack("<I", fwlen // 4)
    fwbin = fwbin[:0x1C] + b"\x00\x00\x00\x00" + sizestr + fwbin[0x24:]

    # Calculate the checksum, whole file with padding
    xorv = 0
    for i in range(0, fwlen, 4):
        xorv ^= struct.unpack("<I", fwbin[i:i+4])[0]

    # Pack everything
    xorv = struct.pack("<I", xorv)
    fwbin = fwbin[:0x1C] + xorv + fwbin[0x20:]

    # Overwrite firmware file
    open(fname, "wb").write(fwbin)

env.AddPreAction("upload", before_upload)

env.AddPostAction("$BUILD_DIR/firmware.bin", calculate_checksum)
