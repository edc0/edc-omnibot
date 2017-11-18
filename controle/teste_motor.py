import RPi.GPIO as GPIO            # import RPi.GPIO module
from time import sleep             # lets us have a delay
from RPIO import PWM               # PWM em qualquer pino, bitches!

GPIO.setmode(GPIO.BCM)             # choose BCM or BOARD
GPIO.setup(24, GPIO.OUT)           # set GPIO24 as an output
GPIO.setup(23, GPIO.OUT)           # set GPIO23 as an output

PWM.setup(1) # incrementos de 1us
PWM.init_channel(1,3000) # criando canal 1, 3ms


'''
try:
    while True:
        GPIO.output(24, 1)         # set GPIO24 to 1/GPIO.HIGH/True
        GPIO.output(23, 0)
        sleep(1)                 # wait half a second
        GPIO.output(24, 0)         # set GPIO24 to 0/GPIO.LOW/False
        GPIO.output(23, 0)
        sleep(1)                 # wait half a second
        GPIO.output(24, 0)
        GPIO.output(23, 1)
        sleep(1)
        GPIO.output(24, 0)
        GPIO.output(23, 0)
        sleep(1)
'''

try:
    while True:

        PWM.add_channel_pulse(1, 23, 0, 300)
        PWM.add_channel_pulse(1, 24, 0, 0)
        sleep(1000)

        PWM.add_channel_pulse(1, 23, 0, 1250)
        PWM.add_channel_pulse(1, 24, 0, 0)
        sleep(1000)

        PWM.add_channel_pulse(1, 23, 0, 3000)
        PWM.add_channel_pulse(1, 24, 0, 0)
        sleep(1000)

        PWM.add_channel_pulse(1, 23, 0, 300)
        PWM.add_channel_pulse(1, 24, 0, 0)
        sleep(1000)

        PWM.add_channel_pulse(1, 23, 0, 0)
        PWM.add_channel_pulse(1, 24, 0, 0)
        sleep(1000)

        PWM.add_channel_pulse(1, 23, 0, 0)
        PWM.add_channel_pulse(1, 24, 0, 250)
        sleep(1000)

        PWM.add_channel_pulse(1, 23, 0, 0)
        PWM.add_channel_pulse(1, 24, 0, 1000)
        sleep(1000)

        PWM.add_channel_pulse(1, 23, 0, 0)
        PWM.add_channel_pulse(1, 24, 0, 450)
        sleep(1000)

        PWM.add_channel_pulse(1, 23, 0, 0)
        PWM.add_channel_pulse(1, 24, 0, 0)
        sleep(1000)
        
except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt
    GPIO.cleanup()                 # resets all GPIO ports used by this program
