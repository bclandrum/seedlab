from smbus2 import SMBus
from time import sleep

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)

# Do in a loop
while(True):
    # Get user input for offset
    offset = int(input("Enter an offset (7 to quit): "))
    
    # Provide an exit key
    if(offset == 7):
        break
        
    # Get user input for string
    string = input("Enter string of 32 characters or less:")
    
    command = [ord(character) for character in string]
    
    i2c.write_i2c_block_data(ARD_ADDR, offset, command)
