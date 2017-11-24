import RPi.GPIO as GPIO            # import RPi.GPIO module
from time import sleep             # lets us have a delay

GPIO.setmode(GPIO.BCM)             # choose BCM or BOARD
GPIO.setup(2,  GPIO.OUT)           # set GPIO24 as an output
GPIO.setup(3,  GPIO.OUT)           # set GPIO23 as an output
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)

M1a = GPIO.PWM(2, 400000)               # pwm com 100 Hz no pino 24
M1b = GPIO.PWM(3, 400000)

M2a = GPIO.PWM(17, 400)
M2b = GPIO.PWM(27, 400)

M1a.start(1)
M2a.start(1)

try:
    while True:
        M1a.start(1)
        M2a.start(1)
        for dc in range (0,100,1):
            M1a.ChangeDutyCycle(dc)
            M2a.ChangeDutyCycle(dc)
            sleep(0.1)
        sleep(5)
        for dc in range (100,0,-1):
            M1a.ChangeDutyCycle(dc)
            M2a.ChangeDutyCycle(dc)
            sleep(0.1)
        M1a.stop()
        M2a.stop()
        sleep(1)
        M1b.start(1)
        M2b.start(1)
        for dc in range (0,100,1):
            M1b.ChangeDutyCycle(dc)
            M2b.ChangeDutyCycle(dc)
            sleep(0.1)
        sleep(5)
        for dc in range (100,0,-1):
            M1b.ChangeDutyCycle(dc)
            M2b.ChangeDutyCycle(dc)
            sleep(0.1)
        M1b.stop()
        M2b.stop()
        sleep(1)

except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt
    M1a.stop()
    M2a.stop()
    M1b.stop()
    M2b.stop()
    GPIO.cleanup()                 # resets all GPIO ports used by this program
