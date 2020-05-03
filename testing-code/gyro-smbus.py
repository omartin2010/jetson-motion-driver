import smbus
bus = smbus.SMBus(17)

Register = bus.read_i2c_block_data(0x42, 0, 23)
acc_x = Register[0] * 1.0
acc_y = Register[1] * 1.0
acc_z = Register[2] * 1.0
acc_tilt = Register[3]
