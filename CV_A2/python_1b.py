# raspberry pi code for 1b.
import time
import board
import busio
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
# LCD Display Setup (via github)
lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows, address=0x20)
lcd.backlight = True

# Arduiono I2C Setup
ARDUINO_ADDR = 8
bus = SMBus(1)
# Read in user provided integer value
def get_valid_int():
    while True:
        try:
            value = int(input("Enter an integer value between 0 and 100: "))
            if 0<=value<=100:
                return value
            print("Value is not between 0 and 100. Please try again.")
        except ValueError:
            print("Value is not a valid integer. Please try again.")
# __main__
def main():
    while True:
        value=get_valid_int()
        bus.write_byte(ARDUINO_ADDR,value) # starts receiving on Arduino
        time.sleep(0.1)

        response=bus.read_byte(ARDUINO_ADDR) # starts reading on Arduino
        print(f"Arduino returned: {response}, (expected {value+100})")
        lcd.clear()
        lcd.message = f"You entered:\n {response}"
        time.sleep(0.5)
        lcd.backlight = False
if __name__ == "__main__":
    main()
