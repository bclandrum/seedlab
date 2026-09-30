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
# Set goal location to send to Arduino
def goal_loc(cx,cy,frame_width=640,frame_height=480):
    x_axis= frame_height//2 # horizontal threshold (divides vertical in half across the middle)
    y_axis= frame_width//2 # vertical threshold (divides horizontal in half across the middle)

    if cx >= y_axis and cy <= x_axis: #north east quadrant
        left,right = 0,0
        lcd.Text = f"Goal Position:\n{left},{right}"
    elif cx <= y_axis and cy <= x_axis: #north west quadrant
        left,right = 0,1
        lcd.Text = f"Goal Position:\n{left},{right}"
    elif cx <= y_axis and cy >= x_axis: #south west quadrant
        left,right = 1,1
        lcd.Text = f"Goal Position:\n{left},{right}"
    elif cx >= y_axis and cy >= x_axis: #south east quadrant
        left,right,1,0
        lcd.Text = f"Goal Position:\n{left},{right}"

    value = (left << 1)|(right) # goal location formatting
    return left,right,value

# __main__
def main():
    cx, cy = 100,100 #REPLACE WITH ACTUAL COORDINATES FOR ARUCO MARKER
    while True:
        left, right, value = goal_loc(cx, cy)
        bus.write_byte(ARDUINO_ADDR, value) # starts receiving on Arduino
        time.sleep(0.1)
        response = bus.read_byte(ARDUINO_ADDR) # starts reading on Arduino
        print(f"Coordinates Received: {response}")
        lcd.clear()
        time.sleep(1)       
if __name__ == "__main__":
    main()
