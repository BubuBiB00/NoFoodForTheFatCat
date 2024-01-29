import torch
import os
import cv2
from time import sleep
from gpiozero import MotionSensor, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import sys

class AutomaticFeeder:
    def __init__(self, camera, delay, num_images):
        pin_factory = PiGPIOFactory()
        # Pin setup
        self.servo = Servo(27,pin_factory=pin_factory)
        self.motion_sensor = MotionSensor(17)
        
        self.camera = camera 
        self.detections = []
        self.image_path = 'test_images/'
        self.images = []
        self.delay = delay
        self.first_delay = delay * 1.5
        self.num_images = num_images
        self.model = 0

        # Den Servo kalibrieren
        self.servo.min()
        self.servo.mid()
        self.servo.max()
        self.servo.detach()

    def procedure_controller(self):
        while True:
            print('Detecting motion...\n')
            self.detect_motion()

            print('Taking pictures...\n')
            self.take_pictures()

            print('Scanning images...\n')
            self.image_recognition()

            print('Looking for dogs...')
            if self.detect_dogs():
                print('Found dog!\n')
                print('Opening lid...\n')
                self.open_lid()
            else:
                print('No doggo detected :(\n')
                sleep(2)

    def detect_motion(self): 
        self.motion_sensor.wait_for_active()
        sleep(1)

    def take_pictures(self):
        used_cam = cv2.VideoCapture(self.camera)

        if not os.path.exists(self.image_path):
            os.makedirs(self.image_path)

        if not used_cam.isOpened():
            print("Error: Could not open camera.")
            return

        try:
            sleep(self.first_delay)

            for i in range(self.num_images):
                sleep(self.delay)
                ret, frame = used_cam.read()

                filename = os.path.join(self.image_path, f'image_{i+1}.jpg')
                self.images.append(filename)
                cv2.imwrite(filename, frame)

        except Exception as e:
            print(e)

        finally:
            # Release the camera and destroy OpenCV windows regardless of exceptions
            used_cam.release()
            cv2.destroyAllWindows()

    def image_recognition(self):
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.model.classes = [15,16]

        for image in self.images:   
            results = self.model(image)

            prediction = results.pred[0]
            class_names = results.names
            
            for det in prediction:
                label_index = int(det[-1])
                class_name = class_names[label_index]
                self.detections.append(class_name)

    def detect_dogs(self):
        print('Detections...')
        print(self.detections)

        if 'dog' in self.detections:
            return True
        else:
            return False

    def open_lid(self):
        self.servo.value = 0.2  # Set to mid position
        sleep(0.5)
        self.servo.value = 0.0  # Set to min position
        sleep(1)
        self.servo.value = 0.5  # Set back to mid position
        sleep(0.5)
        self.servo.value = 1.0  # Set to max position
        sleep(1)
        self.servo.detach()


args = sys.argv

feeder = AutomaticFeeder(camera = args[1], delay = 0.5, num_images = 5)
try:
    feeder.procedure_controller()
except Exception as e:
    print(e)
