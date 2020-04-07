import serial
import time
import numpy as np
import copy

ser1 = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.2, xonxoff=False, rtscts=False,
                    write_timeout=0.5, dsrdtr=False, inter_byte_timeout=None, exclusive=None)
ser2 = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.2, xonxoff=False, rtscts=False,
                    write_timeout=0.5, dsrdtr=False, inter_byte_timeout=None, exclusive=None)
ser3 = serial.Serial('/dev/ttyUSB2', baudrate=115200, timeout=0.2, xonxoff=False, rtscts=False,
                    write_timeout=0.5, dsrdtr=False, inter_byte_timeout=None, exclusive=None)

POT_Condition_suj1_ecm = [1, 1, 1, 1, 1, 1,
                          0, 1, 1, 1, 1, 1]
POT_Condition_suj2 = [1, 1, 1, 1, 1, 1,
                      1, 1, 1, 1, 1, 1]
# calibration coefficients
suj1_calibration_positive = [1, 0.8333, 1, 1, 1, 1]
suj1_calibration_negative = [1, 0.7143, 1, 1, 1, 1]
suj2_calibration_positive = [1, 0.8333, 0.91, 1, 1, 0.8889]
suj2_calibration_negative = [1, 0.7143, 0.91, 1, 1, 0.8889]
ecm_calibration = [1, 0.7692, 1, 1]

full_range = int('FFFFFF', 16)
pi = np.pi
joint_offset_suj1 = [-0.115, -pi, -pi, -pi, -pi, -pi]
joint_offset_suj2 = [-0.109, -pi, -pi, -pi, -pi, -pi]
joint_offset_ecm = [-0.141, -pi, -pi, -pi, -pi, -pi]


def get_suj_joint_reading(serial_port):
    readings = []
    POT_sum = 0
    valid_reading = True
    # voltages = np.zeros((12, 1)).tolist()
    voltages = [0 for i in range(12)]

    serial_port.write(b"AT+ADDRESS\r\n")
    serial_port_address = serial_port.read_until().decode('utf-8')
    address_num = int(serial_port_address[0], 16)
    isGetAddressScuess = serial_port.read_until()
    if isGetAddressScuess == b"OK\r\n":
        print('Port Address Obtained Successfully')
    else:
        serial_port.reset_input_buffer()
        print('Failed to Get Port Address')

    if address_num == 3:
        arm = 'SUJ1'
    elif address_num == 1:
        arm = 'SUJ2'
    else:
        arm = "ECM"

    serial_port.write(b"AT+READALL\r\n")
    for i in range(13):
        readings.append(serial_port.read_until())
        if i == 12:
            if readings[i][-4:] == b"OK\r\n":
                print('Reading Success.')
            else:
                serial_port.reset_input_buffer()
                print("Reading Error.")
    # print(readings)
    for reading_ in readings[:-1]:
        reading_ = reading_.decode('utf-8')
        POT = int(reading_[-3], 16)
        POT_sum += POT
        voltage = float(int(reading_[0:6], 16)) / float(full_range) * 2.5
        voltages[POT] = voltage
    # It is noted that the readings can contain duplicate data from
    # the ADC due to the hardware reason. And the duplicate readings at the first and last position in the 12 readings.
    # ***** Check methead 1 *****
    # if readings[0].decode('utf-8')[-3] == readings[11].decode('utf-8').[-3] :
    #   valid_reading = False
    # ***** Check method 2 *****
    # 66 = sum(0 to 11)
    if POT_sum != 66:
        valid_reading = False
    print(arm + ' reading:')
    print(readings)
    return voltages, valid_reading, arm


def get_suj_joint_pos(voltages, suj_type):
    if suj_type == 'SUJ1':
        joint_pos = copy.deepcopy(joint_offset_suj1)
        POT_Condition = POT_Condition_suj1_ecm
    elif suj_type == 'SUJ2':
        joint_pos = copy.deepcopy(joint_offset_suj2)
        POT_Condition = POT_Condition_suj2
    else:
        joint_pos = copy.deepcopy(joint_offset_ecm)
        POT_Condition = POT_Condition_suj1_ecm

    for joint_ in range(6):
        if joint_ == 0:
            if suj_type == 'SUJ1':
                joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_]
                                    + voltages[joint_+6] * POT_Condition[joint_+6]) * 0.460422
            elif suj_type == 'SUJ2':
                joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_]
                                    + voltages[joint_+6] * POT_Condition[joint_+6]) / 2 * 0.462567

            else:
                joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_]
                                    + voltages[joint_+6] * POT_Condition[joint_+6]) / 2 * 0.440917
        else:
            joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_] + voltages[joint_+6] * POT_Condition[joint_+6])\
                                / 2 / 2.5 * 2 * pi
    joint_pos[3] *= -1
    if suj_type == 'ECM':
        joint_pos = joint_pos[0:4]
    return joint_pos


def release_brakes(ser):
    ser.write(b"AT+FREEALL\r\n")
    respond = ser.read_until()
    if respond == b"OK\r\n":
        print("All brakes are released successfully.")
    else:
        ser.reset_input_buffer()
        print("All Brakes Release Failed.")


def release_brakes_single(joint_num, ser):
    if joint_num not in [1, 2, 3, 4, 5, 6]:
        print("Error: joint index out of range [1, 2, 3, 4, 5, 6].")
        return

    joint_index = str(joint_num).encode()
    ser.write(b"AT+FREE=" + joint_index + b"\r\n")
    respond = ser.read_until()

    if respond == b"OK\r\n":
        print("Succeed to release joint brake: %d" % joint_num)
    else:
        ser.reset_input_buffer()
        print("Fail to release joint brake: %d" % joint_num)


def lock_brakes(ser):
    ser.write(b"AT+LOCKALL\r\n")
    respond = ser.read_until()
    if respond == b"OK\r\n":
        print("All brakes are locked successfully.")
    else:
        ser.reset_input_buffer()
        print("All Brakes Lock Failed.")


def get_data():
    suj1_joint = [0 for i in range(6)]
    suj2_joint = [0 for i in range(6)]
    ecm_joint = [0 for i in range(4)]
    allIsValid = False
    while not(allIsValid):
        [reading1, isValid1, arm1] = get_suj_joint_reading(ser1)
        [reading2, isValid2, arm2] = get_suj_joint_reading(ser2)
        [reading3, isValid3, arm3] = get_suj_joint_reading(ser3)
        allIsValid = (isValid1 and isValid2 and isValid3)

    armSerialport = {}
    if arm1 == 'SUJ1':
        suj1_joint = get_suj_joint_pos(reading1, 'SUJ1')
        armSerialport['SUJ1'] = ser1
    elif arm1 == 'SUJ2':
        suj2_joint = get_suj_joint_pos(reading1, 'SUJ2')
        armSerialport['SUJ2'] = ser1
    else:
        ecm_joint = get_suj_joint_pos(reading1, 'ECM')
        armSerialport['ECM'] = ser1

    if arm2 == 'SUJ1':
        suj1_joint = get_suj_joint_pos(reading2, 'SUJ1')
        armSerialport['SUJ1'] = ser2
    elif arm2 == 'SUJ2':
        suj2_joint = get_suj_joint_pos(reading2, 'SUJ2')
        armSerialport['SUJ2'] = ser2
    else:
        ecm_joint = get_suj_joint_pos(reading2, 'ECM')
        armSerialport['ECM'] = ser2

    if arm3 == 'SUJ1':
        suj1_joint = get_suj_joint_pos(reading3, 'SUJ1')
        armSerialport['SUJ1'] = ser3
    elif arm3 == 'SUJ2':
        suj2_joint = get_suj_joint_pos(reading3, 'SUJ2')
        armSerialport['SUJ2'] = ser3
    else:
        ecm_joint = get_suj_joint_pos(reading3, 'ECM')
        armSerialport['ECM'] = ser3

    # get results calibrated
    if suj1_joint[1] > 0:
        suj1_joint = [a*b for a,b in zip(suj1_joint, suj1_calibration_positive)]
    else:
        suj1_joint = [a*b for a, b in zip(suj1_joint, suj1_calibration_negative)]

    if suj2_joint > 0:
        suj2_joint = [a*b for a,b in zip(suj2_joint, suj2_calibration_positive)]
    else:
        suj2_joint = [a*b for a,b in zip(suj2_joint, suj2_calibration_negative)]
    
    ecm_joint = [a*b for a,b in zip(ecm_joint, ecm_calibration)]

    suj1_joint_deg = [180 / pi * i for i in suj1_joint]
    suj2_joint_deg = [180 / pi * i for i in suj2_joint]
    ecm_joint_deg = [180 / pi * i for i in ecm_joint]
    suj1_joint_deg[0] = suj1_joint[0]
    suj2_joint_deg[0] = suj2_joint[0]
    ecm_joint_deg[0] = ecm_joint[0]

    # ser1.close()
    # ser2.close()
    # ser3.close()
    # return [suj1_joint, suj2_joint, ecm_joint], armSerialport
    return [suj1_joint_deg, suj2_joint_deg, ecm_joint_deg], armSerialport


if __name__ == '__main__':
    print('Initial Reading:')
    [data, armSerialportDic] = get_data()
    print('\n\nSUJ1 --> SUJ2 --> ECM')
    for data_ in data:
        print(data_)

    while True:
        print('\n--->Choose Mode:')
        action_mode = input('1: Release Single Joint \n2: Release All Joints \
                            \n3: Read Joint position \n4: Lock All Joints \n5: Exit\n---> \n')
        if action_mode == 1:
            print('Use Input List Form: [ArmIndex, JointIndex] \nSUJ1: 1 \nSUJ2: 2 \nECM: 3')
            print('SUJ1 & SUJ2 Joint Index: 1-6 \nECM Joint Index: 1-4')
            user_input = input('Example Input: [1, 1] \n')
            arm = user_input[0]
            joint = user_input[1]
            if arm == 1:
                release_brakes_single(joint, armSerialportDic['SUJ1'])
            elif arm == 2:
                release_brakes_single(joint, armSerialportDic['SUJ2'])
            elif arm == 3:
                release_brakes_single(joint, armSerialportDic['ECM'])
            else:
                print('Error: Invalid ArmIndex')

        elif action_mode == 2:
            release_brakes(ser1)
            release_brakes(ser2)
            release_brakes(ser3)

        elif action_mode == 3:
            [data, armSerialportDic] = get_data()
            print('\n\nSUJ1 --> SUJ2 --> ECM')
            for data_ in data:
                print(data_)

        elif action_mode == 4:
            lock_brakes(ser1)
            lock_brakes(ser2)
            lock_brakes(ser3)

        elif action_mode == 5:
            break
        else:
            print('Error: Invalid Mode')
            break

    print('Auto Lock All Joints ......')
    lock_brakes(ser1)
    lock_brakes(ser2)
    lock_brakes(ser3)
    ser1.close()
    ser2.close()
    ser3.close()
    print('Lock Successfully')

