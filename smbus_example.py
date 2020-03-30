from smbus import SMBus

with SMBus(1) as bus:
    block = bus.read_i2c_block_data(45, 0, 16)
    print(block)
