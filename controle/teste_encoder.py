
import RPi.GPIO as GPIO
import threading
from time import sleep

						# GPIO Ports
Enc_A = 4  				# Encoder input A: input GPIO 4
Enc_B = 14  			        # Encoder input B: input GPIO 14
Mot_A = 23
Mot_B = 24

Rotary_counter = 0  			# Start counting from 0
Current_A = 1					# Assume that rotary switch is not
Current_B = 1					# moving while we init software

LockRotary = threading.Lock()		# create lock for rotary switch


# initialize interrupt handlers
def init():
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM) # Use BCM mode

    GPIO.setup(Enc_A, GPIO.IN)              # setup callback thread for the A and B encoder
    GPIO.setup(Enc_B, GPIO.IN)              # use interrupts for all inputs
    GPIO.setup(Mot_A, GPIO.OUT)
    GPIO.setup(Mot_B, GPIO.OUT)

    GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=rotary_interrupt) 				# NO bouncetime
    GPIO.add_event_detect(Enc_B, GPIO.RISING, callback=rotary_interrupt) 				# NO bouncetime

    #Ma = GPIO.PWM(Mot_A,400)               # pwm com 400 Hz no pino 24
    #Mb = GPIO.PWM(Mot_B,400)

    return



# Rotarty encoder interrupt:
# this one is called for both inputs from rotary switch (A and B)
def rotary_interrupt(A_or_B):
	global Rotary_counter, Current_A, Current_B, LockRotary
													# read both of the switches
	Switch_A = GPIO.input(Enc_A)
	Switch_B = GPIO.input(Enc_B)
													# now check if state of A or B has changed
													# if not that means that bouncing caused it
	if Current_A == Switch_A and Current_B == Switch_B:		# Same interrupt as before (Bouncing)?
		return										# ignore interrupt!

	Current_A = Switch_A								# remember new state
	Current_B = Switch_B								# for next bouncing check


	if (Switch_A and Switch_B):						# Both one active? Yes -> end of sequence
		LockRotary.acquire()						# get lock
		if A_or_B == Enc_B:							# Turning direction depends on
			Rotary_counter += 1						# which input gave last interrupt
		else:										# so depending on direction either
			Rotary_counter -= 1						# increase or decrease counter
		LockRotary.release()						# and release lock
	return											# THAT'S IT

# Main loop. Demonstrate reading, direction and speed of turning left/rignt
def main():
	global Rotary_counter, LockRotary


	Volume = 0									# Current Volume
	NewCounter = 0								# for faster reading with locks
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.OUT)
        GPIO.setup(24, GPIO.OUT)
        Ma = GPIO.PWM(23, 400)
        Mb = GPIO.PWM(24, 400)


	init()										# Init interrupts, GPIO, ...

	while True :
            Mb.start(100)
            Ma.stop()
            sleep(0.1)

            					# because of threading make sure no thread
            					# changes value until we get them
            					# and reset them

            LockRotary.acquire()					# get lock for rotary switch
            NewCounter = Rotary_counter			# get counter value
            Rotary_counter = 0						# RESET IT TO 0
            LockRotary.release()					# and release lock

            if (NewCounter !=0):					# Counter has CHANGED
            	Volume = Volume + NewCounter*abs(NewCounter)	# Decrease or increase volume
            	if Volume < 0:						# limit volume to 0...100
            		Volume = 0
            	if Volume > 100:					# limit volume to 0...100
            		Volume = 100
            	print NewCounter, Volume			# some test print



# start main demo function
main()
