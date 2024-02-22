import RPi.GPIO as GPIO
import time

# Assign GPIO pin numbers to variables
# left color sensor
l_s2 = 16
l_s3 = 18
l_sig = 22 # labeled "out" on your board
# right color sensor
r_s2 = 15
r_s3 = 11
r_sig = 13 # labeled "out" on your board
# motor 
ena = 3
in1 = 29
in2 = 31
in3 = 33
in4 = 35
enb = 5

cycles = 10


# Setup GPIO and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
# right color sensor 
GPIO.setup(r_s2, GPIO.OUT)
GPIO.setup(r_s3, GPIO.OUT)
GPIO.setup(r_sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# left color sensor
GPIO.setup(l_s2, GPIO.OUT)
GPIO.setup(l_s3, GPIO.OUT)
GPIO.setup(l_sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# motor
GPIO.setup(ena, GPIO.OUT) 
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT)

# control speed of motors
p1 = GPIO.PWM(ena, 50)
p2 = GPIO.PWM(enb, 50)
p1.ChangeDutyCycle(100)
p2.ChangeDutyCycle(100)
p1.start(0)
p2.start(0)

def RunMotor():
    p1.ChangeDutyCycle(18)
    p2.ChangeDutyCycle(18)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def TurnMotorRight(duty_cycle):
    p1.ChangeDutyCycle(duty_cycle)
    p2.ChangeDutyCycle(0)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def TurnMotorLeft(duty_cycle):
    p1.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(duty_cycle)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def StopMotor():
    p1.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(0)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def SensorRight():
    colors = [0, 0, 0]
    print("Right Sensor:", end = " ")
    # Detect red values
    GPIO.output(r_s2, GPIO.LOW)
    GPIO.output(r_s3, GPIO.LOW)

    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(r_sig, GPIO.FALLING)
    duration = time.time() - start_time
    red = cycles / duration
    colors[0] = red
    print("red value - ", red, end = " ")

    # Detect green values
    GPIO.output(r_s2, GPIO.HIGH)
    GPIO.output(r_s3, GPIO.HIGH)

    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(r_sig, GPIO.FALLING)
    duration = time.time() - start_time
    green = cycles / duration
    colors[1] = green
    print("green value - ", green, end = " ")

    # Detect blue values
    GPIO.output(r_s2, GPIO.LOW)
    GPIO.output(r_s3, GPIO.HIGH)

    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(r_sig, GPIO.FALLING)
    duration = time.time() - start_time
    blue = cycles / duration
    colors[2] = blue
    print("blue value - ", blue)

    return(colors)

def SensorLeft():
    colors = [0, 0, 0]
    #print("Left Sensor: ", end = " ")
    # Detect red values
    GPIO.output(l_s2, GPIO.LOW)
    GPIO.output(l_s3, GPIO.LOW)

    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(l_sig, GPIO.FALLING)
    duration = time.time() - start_time
    red = cycles / duration
    colors[0] = red
    # print("red value - ", red, end = " ")

    # Detect green values
    GPIO.output(l_s2, GPIO.HIGH)
    GPIO.output(l_s3, GPIO.HIGH)

    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(l_sig, GPIO.FALLING)
    duration = time.time() - start_time
    green = cycles / duration
    colors[1] = green
    # print("green value - ", green, end = " ")


    # Detect blue values
    GPIO.output(l_s2, GPIO.LOW)
    GPIO.output(l_s3, GPIO.HIGH)

    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(l_sig, GPIO.FALLING)
    duration = time.time() - start_time
    blue = cycles / duration
    colors[2] = blue
    # print("blue value - ", blue)

    return(colors)

def get_pwm_from_color_diff(prev_color, new_color):
    diff = min(max(prev_color - new_color, 0), 5000)
    return 0.00375 * diff + 15

# blue:   33000, index 0
# red:    41000, index 2
# green:  41000, index 2
# purple: 37000, index 1

try:
    i = 0 
    while True:
        color_choice = 'green'
        index = 0 if color_choice == 'blue' else 1 if color_choice == 'purple' else 2
        ideal_value = 33000 if color_choice == 'blue' else 37000 if color_choice == 'purple' else 41000
        SensorRight()
        SensorLeft()
        i += 1
        if i > 0: 
            RunMotor()
            right_sensor = SensorRight()
            left_sensor = SensorLeft()
            new_right = right_sensor[index]
            new_left = left_sensor[index]
            print("Right, " + str(new_right))
            print("Left, " + str(new_left))

            if (ideal_value - new_left) > 1000:
                StopMotor()

                while ideal_value - new_left > 1000:
                    TurnMotorLeft(get_pwm_from_color_diff(ideal_value, new_left))
                    right_sensor = SensorRight()
                    left_sensor = SensorLeft()
                    new_right = right_sensor[index]
                    new_left = left_sensor[index]

            elif (ideal_value - new_right) > 1000: 
                StopMotor()

                while ideal_value - new_right > 1000:
                    TurnMotorRight(get_pwm_from_color_diff(ideal_value, new_right))
                    right_sensor = SensorRight()
                    left_sensor = SensorLeft()
                    new_right = right_sensor[index]
                    new_left = left_sensor[index]
            else:
                print("Straight")

except KeyboardInterrupt:
    GPIO.cleanup()