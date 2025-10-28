import RPi.GPIO as GPIO

def my_callback(channel):
    print(f"Edge detected on channel {channel}")

GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(13, GPIO.RISING, callback=my_callback, bouncetime=100)

try:
    while True:
        pass
except KeyboardInterrupt:
    GPIO.cleanup()
