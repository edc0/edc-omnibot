import RPi.GPIO as GPIO            # import RPi.GPIO module
from time import sleep             # lets us have a delay

GPIO.setmode(GPIO.BCM)             # choose BCM or BOARD
GPIO.setup(24, GPIO.OUT)           # set GPIO24 as an output
GPIO.setup(23, GPIO.OUT)           # set GPIO23 as an output

a = GPIO.PWM(24,400)               # pwm com 100 Hz no pino 24
b = GPIO.PWM(23,400)

a.start(1)

try:
    while True:
        a.start(1)
        for dc in range (0,100,5):
            a.ChangeDutyCycle(dc)
            sleep(0.5)
        sleep(5)
        for dc in range (100,0,-5):
            a.ChangeDutyCycle(dc)
            sleep(0.5)
        a.stop()
        sleep(1)
        b.start(1)
        for dc in range (0,100,5):
            b.ChangeDutyCycle(dc)
            sleep(0.1)
        sleep(5)
        for dc in range (100,0,-5):
            b.ChangeDutyCycle(dc)
            sleep(0.1)
        b.stop()
        sleep(1)

except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt
    a.stop()
    GPIO.cleanup()                 # resets all GPIO ports used by this program
