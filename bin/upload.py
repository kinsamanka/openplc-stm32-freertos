Import("env")

def before_upload(source, target, env):
    if env['UPLOAD_PROTOCOL'] == 'serial':
        # send magic string
        from serial import Serial

        serial = Serial(env['UPLOAD_PORT'], env['UPLOAD_SPEED'], timeout = 1, parity='E')
        serial.write(b'\xff\xffBOOTLOADER\xff\xff')
        serial.close()

env.AddPreAction("upload", before_upload)

