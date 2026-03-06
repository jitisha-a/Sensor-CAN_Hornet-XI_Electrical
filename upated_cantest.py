import socket
import struct

CAN_IFACE = "can0"

#can ids
CAN_ID_ROLL     = 0x300
CAN_ID_PITCH    = 0x301
CAN_ID_YAW      = 0x302
CAN_ID_PRESSURE = 0x303
CAN_ID_DEPTH    = 0x304
CAN_ID_LIN_ACC  = 0x305 # ADDED NEW
CAN_ID_ANG_VEL  = 0x306 # ADDED NEW
CAN_ID_QUATERNION = 0x307 # ADDED NEW

#our scaling factorsss to go back to decimal values from integers
SCALE_RPY = 10000.0  #for 4dp
SCALE_PD  = 100.0    #for 2dp

#helper functions to decode little-endian integers from bytes (reconstruction)

def u16_le(b0, b1): #b0 is LSB and b1 is MSB
    return b0 | (b1 << 8)

def i16_le(b0, b1):
    v = u16_le(b0, b1)
    # convert unsigned 16 to signed 16
    if v >= 0x8000:
        v -= 0x10000
    return v

def i32_le(b0, b1, b2, b3):
    v = (b0 | (b1 << 8) | (b2 << 16) | (b3 << 24))
    # convert unsigned 32 to signed 32
    if v >= 0x80000000:
        v -= 0x100000000
    return v

def main():
    # 1) open raw CAN socket (SocketCAN)
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

    # 2) bind to interface
    s.bind((CAN_IFACE,))

    print(f"Listening on {CAN_IFACE} for IDs 0x300, 0x301, 0x302, 0x303, 0x304, 0x305")

    # dictionary keeps latest values so we can print a combined line of all 5 values
    latest = {
        "roll": None,
        "pitch": None,
        "yaw": None,
        "pressure": None,
        "depth": None,
        "ctr_roll": None,
        "ctr_pitch": None,
        "ctr_yaw": None,
        "ctr_pressure": None,
        "ctr_depth": None,
        "ax": None,
        "ay": None,
        "az": None,
        "gx": None,
        "gy": None,
        "gz": None,
        "q1": None,
        "q2": None,
        "q3": None,
        "q0": None,
    }

    while True: #infinite loop that reads and processed CAN frames forever
        # 3) read one frame (16 bytes as linux gives CAN frames in a 16 byte structure)
        frame = s.recv(16)

        # 4) unpacking the frameeee: can_id (u32), dlc (u8), padding, data (8 bytes)
        can_id, dlc, data = struct.unpack("=IB3x8s", frame)

        # 5) mask out flags - basically cleans the ID as linux may store flags in the upper bits
        can_id = can_id & 0x1FFFFFFF

        # 6) keep only the actual bytes that were sent (dlc tells you how many)
        data = data[:dlc]
        if dlc != 8:
            continue

        b = list(data) #convert bytes to list of integers for easier processing

        #decoding blocks of if-elif to handle each CAN ID separately

        if can_id == CAN_ID_ROLL:
            raw = i32_le(b[0], b[1], b[2], b[3]) #first 4 bytes are the roll value
            ctr = u16_le(b[4], b[5]) #next 2 bytes are the counter
            latest["roll"] = raw / SCALE_RPY #scale back to decimal
            latest["ctr_roll"] = ctr #store counter

        elif can_id == CAN_ID_PITCH:
            raw = i32_le(b[0], b[1], b[2], b[3]) #first 4 bytes are the pitch value
            ctr = u16_le(b[4], b[5]) #next 2 bytes are the counter
            latest["pitch"] = raw / SCALE_RPY #scale back to decimal
            latest["ctr_pitch"] = ctr #store counter

        elif can_id == CAN_ID_YAW:
            raw = i32_le(b[0], b[1], b[2], b[3]) #first 4 bytes are the yaw value
            ctr = u16_le(b[4], b[5]) #next 2 bytes are the counter
            latest["yaw"] = raw / SCALE_RPY #scale back to decimal
            latest["ctr_yaw"] = ctr #store counter

        elif can_id == CAN_ID_PRESSURE:
            raw = u16_le(b[0], b[1]) #first 2 bytes are the pressure value
            ctr = u16_le(b[2], b[3]) #next 2 bytes are the counter
            latest["pressure"] = raw / SCALE_PD #scale back to decimal
            latest["ctr_pressure"] = ctr #store counter

        elif can_id == CAN_ID_DEPTH:
            raw = i16_le(b[0], b[1]) #first 2 bytes are the depth value
            ctr = u16_le(b[2], b[3]) #next 2 bytes are the counter
            latest["depth"] = raw / SCALE_PD #scale back to decimal
            latest["ctr_depth"] = ctr #store counter
        
        elif can_id == CAN_ID_LIN_ACC:
            ax_raw = i16_le(b[0], b[1]) #first 2 bytes are ax
            ay_raw = i16_le(b[2], b[3]) #next 2 bytes are ay
            az_raw = i16_le(b[4], b[5]) #next 2 bytes are az
            latest["ax"] = ax_raw / SCALE_PD #scale back to decimal
            latest["ay"] = ay_raw / SCALE_PD #scale back to decimal
            latest["az"] = az_raw / SCALE_PD #scale back to decimal

        elif can_id == CAN_ID_ANG_VEL:
            gx_raw = i16_le(b[0], b[1]) #first 2 bytes are gx
            gy_raw = i16_le(b[2], b[3]) #next 2 bytes are gy
            gz_raw = i16_le(b[4], b[5]) #next 2 bytes are gz
            latest["gx"] = gx_raw / SCALE_PD #scale back to decimal
            latest["gy"] = gy_raw / SCALE_PD #scale back to decimal
            latest["gz"] = gz_raw / SCALE_PD #scale back to decimal
        
        elif can_id == CAN_ID_QUATERNION:
            q1_raw = i16_le(b[0], b[1]) #first 2 bytes are q1
            q2_raw = i16_le(b[2], b[3]) #next 2 bytes are q2
            q3_raw = i16_le(b[4], b[5]) #next 2 bytes are q3
            q0_raw = i16_le(b[6], b[7]) #next 2 bytes are q0
            latest["q1"] = q1_raw / SCALE_PD #scale back to decimal
            latest["q2"] = q2_raw / SCALE_PD #scale back to decimal
            latest["q3"] = q3_raw / SCALE_PD #scale back to decimal
            latest["q0"] = q0_raw / SCALE_PD #scale back to decimal

        else:
            continue

        # print the latest values of all parameters
        print("Roll: ", latest['roll'], "Count: ", latest['ctr_roll'])
        print("Pitch: ", latest['pitch'], "Count: ", latest['ctr_pitch'])
        print("Yaw: ", latest['yaw'], "Count: ",  latest['ctr_yaw'])
        print("Pressure: ", latest['pressure'], "Count: ", latest['ctr_pressure'])
        print("Depth: ", latest['depth'], "Count: ", latest['ctr_depth'])
        print("Linear Acceleration -")
        print("ax2: ", latest['ax'])
        print("ay2: ", latest['ay'])
        print("az2: ", latest['az'])
        print("Angular Velocity -")
        print("gx2: ", latest['gx'])
        print("gy2: ", latest['gy'])
        print("gz2: ", latest['gz'])
        print("Quaternion -")
        print("q1: ", latest['q1'])
        print("q2: ", latest['q2'])
        print("q3: ", latest['q3'])
        print("q0: ", latest['q0'])


if __name__ == "__main__":
    main()

                              

                                           
