# Import necessary modules
import RPi.GPIO as GPIO
import time

HIGH = True
LOW = False

GPIO.setmode(GPIO.BCM)

IR_PIN = 4

# Set up GPIO pins

# pull_up_down=GPIO.PUD_DOWN or pull_up_down=GPIO.PUD_UP
GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def setup():
    # setup somthing (do 1 time)
    pass


def loop():
    # continuously do something

    try:
        while True:
            # Read button state
            if not GPIO.input(IR_PIN):
                print("okay, go")
                time.sleep(0.1)
            else:
                pass

            # wait 10 ms to give CPU chance to do other things
            time.sleep(0.001)

    except KeyboardInterrupt:  # if Ctrl C is pressed...
        print("Program stopped and furnace shut off.")  # print a clean exit message
    GPIO.cleanup()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
