import serial
import time
import crcmod

crc16_func = crcmod.predefined.mkCrcFun('modbus')

def crc16(data):
    return crc16_func(data)

def build_cmd(addr, reg, count):
    cmd = bytes([addr, 0x03,
                 (reg >> 8) & 0xFF, reg & 0xFF,
                 (count >> 8) & 0xFF, count & 0xFF])
    crc = crc16(cmd)
    return cmd + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def parse_int16(high, low):
    val = (high << 8) | low
    if val > 32767:
        val -= 65536
    return val

def send_read(ser, addr, reg, count):
    cmd = build_cmd(addr, reg, count)
    expected_len = 3 + count * 2 + 2

    for attempt in range(3):
        ser.reset_input_buffer()
        ser.write(cmd)
        ser.flush()

        timeout = time.time() + 0.1
        while ser.in_waiting < expected_len:
            if time.time() > timeout:
                break
            time.sleep(0.0001)

        if ser.in_waiting >= expected_len:
            response = ser.read(expected_len)
            if len(response) != expected_len:
                continue
            crc_recv = (response[-1] << 8) | response[-2]
            crc_calc = crc16(response[:-2])
            if crc_recv != crc_calc:
                print(f'CRC mismatch on attempt {attempt+1}')
                continue
            return response[3:-2]

        print(f'Timeout on attempt {attempt+1}')
        time.sleep(0.001)

    return None

ser = serial.Serial('/dev/ttySC1', 115200, timeout=1.0)

print("Testing WT901C with retry mechanism...")
success = 0
fail = 0

for i in range(1000):
    data = send_read(ser, 0x50, 0x34, 3)

    if data is not None:
        ax = parse_int16(data[0], data[1]) / 32768.0 * 16.0 * 9.8
        ay = parse_int16(data[2], data[3]) / 32768.0 * 16.0 * 9.8
        az = parse_int16(data[4], data[5]) / 32768.0 * 16.0 * 9.8
        print(f"[{i}] AX:{ax:.3f} AY:{ay:.3f} AZ:{az:.3f} m/s²")
        success += 1
    else:
        print(f"[{i}] FAIL after 3 attempts")
        fail += 1

    time.sleep(0.001)

ser.close()
print(f"\nSuccess: {success}/1000  Fail: {fail}/1000")