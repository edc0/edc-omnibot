import RPi.GPIO as GPIO            # import RPi.GPIO module
from time import sleep             # lets us have a delay

GPIO.setmode(GPIO.BCM)             # choose BCM or BOARD
GPIO.setup(24, GPIO.OUT)           # set GPIO24 as an output
GPIO.setup(23, GPIO.OUT)           # set GPIO23 as an output

a = GPIO.PWM(24,100)               # pwm com 100 Hz no pino 24
#b = GPIO.PWM(23,100)

a.start(1)
#b.start(0)

try:
    while True:

        for dc in range (0,101,5):
            a.ChangeDutyCycle(dc)
            sleep(50)
        for dc in range (100,-1,5):
            a.ChangeDutyCycle(dc)
            sleep(50)
        
except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt
    a.stop()
    GPIO.cleanup()                 # resets all GPIO ports used by this program
