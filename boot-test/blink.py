import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
from time import sleep # Import the sleep function from the time module
LED_1v8 = 6
LED_1 = 17
LED_2 = 27
LED_3 = 18
LED_4 = 12
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BCM) # Use GPIO pin numbering
GPIO.setup(LED_1v8, GPIO.OUT, initial=GPIO.LOW) # Set pin to be an output pin and set initial value to low (off)
GPIO.setup(LED_1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LED_2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LED_3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LED_4, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(LED_1v8, GPIO.HIGH) # Turn on
while True: # Run forever
 GPIO.output(LED_1, GPIO.HIGH) # Turn on
 sleep(1/8) # Sleep for 0.25 second
 GPIO.output(LED_2, GPIO.HIGH)
 sleep(1/8)
 GPIO.output(LED_3, GPIO.HIGH)
 sleep(1/8)
 GPIO.output(LED_4, GPIO.HIGH)
 sleep(1/8)
 GPIO.output(LED_1, GPIO.LOW)
 sleep(1/8)
 GPIO.output(LED_2, GPIO.LOW)
 sleep(1/8)
 GPIO.output(LED_3, GPIO.LOW)
 sleep(1/8)
 GPIO.output(LED_4, GPIO.LOW)
 sleep(1/8)