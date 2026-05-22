import serial
import time

# Register map (RH56F1 Humanoid Five-Finger Dexterous Hand – User Manual Section 2.4)
regdict = {
    'ID': 1000,
    'baudrate': 1001,
    'mode': 1100,
    'clearErr': 1003,
    'forceClb': 1007,
    'angleSet': 1040,
    'forceSet': 1046,
    'speedSet': 1052,
    'angleAct': 1064,
    'forceAct': 1070,
    'errCode': 1082,
    'statusCode': 1088,
    'temp': 1094,
    'ip': 1700,
    'actionSeq': 2160,
    'actionRun': 2162
}

def openSerial(port, baudrate):
    ser = serial.Serial(port, baudrate, timeout=0.05)
    return ser

def writeRegister(ser, device_id, addr, num_bytes, values):
    frame = [0xEB, 0x90, device_id, num_bytes + 3, 0x12, addr & 0xFF, (addr >> 8) & 0xFF]
    frame.extend(values)
    checksum = sum(frame[2:]) & 0xFF
    frame.append(checksum)
    ser.reset_input_buffer()
    ser.write(bytes(frame))
    time.sleep(0.01)

def readRegister(ser, device_id, addr, num_bytes, mute=True):
    expected_len = 8 + num_bytes
    frame = [0xEB, 0x90, device_id, 0x04, 0x11, addr & 0xFF, (addr >> 8) & 0xFF, num_bytes]
    checksum = sum(frame[2:]) & 0xFF
    frame.append(checksum)
    ser.reset_input_buffer()
    ser.write(bytes(frame))
    time.sleep(0.01)
    recv = ser.read(expected_len + 10)
    header_pos = recv.rfind(b'\x90\xEB')
    recv = recv[header_pos:]
    values = list(recv[7:7 + num_bytes])
    if not mute:
        print("Read values:", values)
    return values

def write6(ser, device_id, key, values):
    if key not in ['angleSet', 'forceSet', 'speedSet', 'mode']:
        print("Invalid key")
        return
    val_reg = []
    for v in values:
        val_reg.append(v & 0xFF)
        val_reg.append((v >> 8) & 0xFF)
    writeRegister(ser, device_id, regdict[key], 12, val_reg)

def read6(ser, device_id, key):
    if key not in regdict:
        print("Invalid key")
        return None
    raw = readRegister(ser, device_id, regdict[key], 12)
    if not raw or len(raw) < 12:
        print("No or incomplete data received")
        return None
    results = []
    for i in range(6):
        value = raw[2 * i] | (raw[2 * i + 1] << 8)
        if value > 32767:
            value -= 65536
        results.append(value)
    return results

def readTouchData(ser):
    """
    Read touch sensor data from all fingers and palm.
    Returns (finger_results, palm_results) or (None, None) on failure.
    finger_results: dict keyed by finger name, each value is a list of sensor readings.
    palm_results: dict keyed by 'palm_data_1' .. 'palm_data_9'.
    """
    cmd = bytes([0xEB, 0x90, 0x01, 0x04, 0x11, 0xB8, 0x0B, 0x44, 0x1D])
    ser.write(cmd)
    time.sleep(0.025)
    recv = ser.read_all()

    max_attempts = 5
    for attempt in range(max_attempts):
        try:
            start_idx = recv.index(0xB8)
            if recv[start_idx + 1] == 0x0B:
                data_start = start_idx + 2
                break
            else:
                print(f"Locate attempt {attempt + 1}: Expected B8 0B not found")
        except ValueError:
            print(f"Locate attempt {attempt + 1}: Touch data address B8 0B not found")
        time.sleep(0.01)
    else:
        return None, None

    fingers = ['little', 'ring', 'middle', 'index', 'thumb']
    finger_results = {}

    for i, finger in enumerate(fingers):
        base_idx = data_start + i * 10
        bytes_data = recv[base_idx:base_idx + 10]
        data_bytes = [
            (bytes_data[j] | (bytes_data[j + 1] << 8)) for j in range(0, 6, 2)
        ]
        combined_value = (bytes_data[6] | (bytes_data[7] << 8) | (bytes_data[8] << 16))
        data_bytes.append(combined_value)
        finger_results[finger] = data_bytes

    palm_results = {}
    palm_start_idx = data_start + len(fingers) * 10

    if palm_start_idx + 17 < len(recv):
        palm_data = [recv[palm_start_idx + j] for j in range(18)]
        for j in range(9):
            value = palm_data[j * 2] | (palm_data[j * 2 + 1] << 8)
            palm_results[f'palm_data_{j + 1}'] = value
    else:
        print("Palm data out of bounds, unable to read")

    ser.read_all()
    return finger_results, palm_results


if __name__ == '__main__':
    ser = openSerial('/dev/ttyUSB0', 115200)
    write6(ser, 1, 'speedSet', [100, 100, 100, 100, 100, 100])
    time.sleep(2)
    write6(ser, 1, 'forceSet', [500, 500, 500, 500, 500, 500])
    time.sleep(1)
    write6(ser, 1, 'angleSet', [0, 0, 0, 0, 400, -1])
    time.sleep(3)
    read6(ser, 1, 'angleAct')
    time.sleep(1)
    write6(ser, 1, 'angleSet', [1000, 1000, 1000, 1000, 400, -1])
    time.sleep(3)
    read6(ser, 1, 'angleAct')
    time.sleep(1)
    read6(ser, 1, 'errCode')
