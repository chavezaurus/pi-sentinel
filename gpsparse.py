import serial
import sys
from datetime import datetime
from threading import Thread
import RPi.GPIO as GPIO
import socket
import os

# port = "/dev/ttyACM0"
port = "/dev/ttyS0"
now = datetime.utcnow()
delta = 0.0
lat = 0.0
lon = 0.0
current = False

def decode(coord,direction):
    #Converts DDDMM.MMMMM > degrees
    s = coord.split('.')
    degrees = int(s[0][:-2])
    minutes = float(s[0][-2:]+'.'+s[1])

    magnitude = degrees + minutes/60.0
    if direction in ["W","S"]:
        magnitude = -magnitude

    return magnitude

def parseGPS(data):
    # print( "raw:", data ) #prints raw data
    global delta, current, now, lat, lon
    if data[0:6] != '$GNRMC':
        return

    # print(data)
    sdata = data.split(",")
    if sdata[2] == 'V':
        print( "no satellite data available", file=sys.stderr, flush=True )
        return
    # print( "---Parsing GPRMC---", )
    hour = int(sdata[1][0:2])
    minute = int(sdata[1][2:4])
    second = int(sdata[1][4:6])
    hundredths = int(sdata[1][7:9])
    
    year = int(sdata[9][4:6]) + 2000
    month = int(sdata[9][2:4])
    day = int(sdata[9][0:2])

    measured = datetime(year,month,day,hour,minute,second,hundredths*10000)
    difference = now - measured
    delta = 0.98*delta + 0.02*difference.total_seconds()
    
    time = sdata[1][0:2] + ":" + sdata[1][2:4] + ":" + sdata[1][4:6]
    lat = decode(sdata[3],sdata[4]) #latitude
    lon = decode(sdata[5],sdata[6]) #longitute
    speed = sdata[7]       #Speed in knots
    trCourse = sdata[8]    #True course
    date = sdata[9][0:2] + "/" + sdata[9][2:4] + "/" + sdata[9][4:6]#date

    current = False
 
def my_callback(which):
    global now
    now = datetime.utcnow()
    current = true
    # print(now)

def readThread():
    global current
    try:
        ser = serial.Serial(port, baudrate = 9600, timeout = 0.5)
    except:
        print("GPS Port not available", file=sys.stderr)
        return

    print( "Receiving GPS data", file=sys.stderr )
    while True:
        data = ser.readline()
        if current != False:
            try:
                parseGPS(data.decode("utf-8"))
            except:
                current = False


server_address = '/tmp/sentinel_time.sock'

# Make sure the socket does not already exist
try:
    os.unlink(server_address)
except OSError:
    if os.path.exists(server_address):
        exit()

GPIO.setmode(GPIO.BCM)
GPIO.setup(4,GPIO.IN)
GPIO.add_event_detect( 4,GPIO.RISING,callback=my_callback,bouncetime=100)

gps = Thread(target = readThread)
gps.daemon = True
gps.start()

# Create a UDS socket
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

# Bind the socket to the port
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

while True:
    # Wait for a connection
    connection, client_address = sock.accept()
    try:
        data = "%s %7.3f %7.4f %7.4f" %  (now, delta, lat,lon)
        connection.sendall(data.encode())
            
    finally:
        # Clean up the connection
        connection.close()

