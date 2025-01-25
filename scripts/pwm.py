import RPi.GPIO as GPIO

GPIO.setup(0, GPIO.out)
pwm_pin = GPIO.PWM(0, 5000)
pwm_pin.start(25)