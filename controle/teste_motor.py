import RPi.GPIO as GPIO            # import RPi.GPIO module
from time import sleep             # lets us have a delay

GPIO.setmode(GPIO.BCM)             # choose BCM or BOARD
GPIO.setup(24, GPIO.OUT)           # set GPIO24 as an output
GPIO.setup(23, GPIO.OUT)           # set GPIO23 as an output

Ma = GPIO.PWM(24,400)               # pwm com 100 Hz no pino 24
Mb = GPIO.PWM(23,400)

Ma.start(1)

try:
    while True:
        Ma.start(1)
        for dc in range (0,100,1):
            Ma.ChangeDutyCycle(dc)
            sleep(0.1)
        sleep(5)
        for dc in range (100,0,-1):
            Ma.ChangeDutyCycle(dc)
            sleep(0.1)
        Ma.stop()
        sleep(1)
        Mb.start(1)
        for dc in range (0,100,1):
            Mb.ChangeDutyCycle(dc)
            sleep(0.1)
        sleep(5)
        for dc in range (100,0,-1):
            Mb.ChangeDutyCycle(dc)
            sleep(0.1)
        Mb.stop()
        sleep(1)

except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt
    Ma.stop()
    Mb.stop()
    GPIO.cleanup()                 # resets all GPIO ports used by this program
