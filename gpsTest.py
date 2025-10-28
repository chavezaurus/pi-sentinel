import serial

# Set the serial port and baud rate according to your GPS module
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 38400

# Open the serial port
ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=2)

print("Reading GPS data. Press Ctrl+C to stop.")
try:
    while True:
        line = ser.readline().decode('utf-8', errors='replace')
        print(line.strip())
except KeyboardInterrupt:
    print("Stopped reading GPS data.")
finally:
    ser.close()
