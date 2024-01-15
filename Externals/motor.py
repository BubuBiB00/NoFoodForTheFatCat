from gpiozero import Servo
from gpiozero import MotionSensor
from time import sleep
from gpiozero.pins.mock import MockFactory

servo = Servo(17)
servo.pin_factory = MockFactory()

servo.min()

motion = MotionSensor(27)


motion.wait_for_active()
sleep(1)

servo.max()
sleep(1)
servo.min()
sleep(1)

servo.detach()