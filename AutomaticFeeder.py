import torch
import os
import cv2
from time import sleep
from gpiozero import MotionSensor
from gpiozero import Servo
from gpiozero.pins.mock import MockFactory

class AutomaticFeeder:
    def __init__(self, camera, delay, num_images) -> None:
        self.detections = []
        self.image_path = 'test_images/'
        self.images = []
        self.device = cv2.VideoCapture(camera)
        self.delay = delay
        self.first_delay = delay * 1.5
        self.num_images = num_images
        self.model = 0
    	
        myCorrection=0.45
        maxPW=(2.0+myCorrection)/1000
        minPW=(1.0-myCorrection)/1000

        self.servo = Servo(27)
        print("Calibrating the servo...")
        self.servo.min()
        sleep(1)
        self.servo.mid()
        sleep(1)
        self.servo.max()
        sleep(1)
        print("Calibration complete.")
        self.servo.detach()

        self.motion_sensor = MotionSensor(17)

    def motion_detection(self):
        motion = 0
        while motion == 0:
            print('Detecting Movement')
            self.motion_sensor.wait_for_active()
            motion = 1
            sleep(1)

        print('taking_pictures')
        self.take_pictures()

    def take_pictures(self):
        if not os.path.exists(self.image_path):
            os.makedirs(self.image_path)

        if not self.device.isOpened():
            print("Error: Could not open camera.")
            return

        try:
            sleep(self.first_delay)

            for i in range(self.num_images):
                sleep(self.delay)
                ret, frame = self.device.read()

                filename = os.path.join(self.image_path, f'image_{i+1}.jpg')
                self.images.append(filename)
                cv2.imwrite(filename, frame)

        except Exception as e:
            print(e)

        finally:
            # Release the camera and destroy OpenCV windows regardless of exceptions
            self.device.release()
            cv2.destroyAllWindows()

        print('Detecting')
        self.detect()


    def old_take_pictures(self):
        if not os.path.exists(self.image_path):
            os.makedirs(self.image_path)

        if not self.device.isOpened():
            print("Error: Could not open camera.")
            return
        
        sleep(self.first_delay)

        for i in range(self.num_images):
            sleep(self.delay)
            try:
                ret, frame = self.device.read()

            except Exception as e:
                print(e)

            try:
                filename = os.path.join(self.image_path, f'image_{i+1}.jpg')
                self.images.append(filename)
                cv2.imwrite(filename, frame)
                
            except Exception as e:
                print(e)

        self.device.release()
        cv2.destroyAllWindows()
        
        print('Detecting')
        self.detect()

    def detect(self):
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        #self.model.classes = [15,16]

        for image in self.images:   
            results = self.model(image)
            #results.show()

            prediction = results.pred[0]
            class_names = results.names
            
            for det in prediction:
                label_index = int(det[-1])
                class_name = class_names[label_index]
                self.detections.append(class_name)

        #if ('dog' in self.detections):
        print('Opening Lid!')
        self.open_lid()
        #else:
        #    pass

    def open_lid(self):
        self.servo.value = 0.5  # Set to mid position
        sleep(0.5)
        self.servo.value = 0.0  # Set to min position
        sleep(1)
        self.servo.value = 0.5  # Set back to mid position
        sleep(0.5)
        self.servo.value = 1.0  # Set to max position
        sleep(1)
        self.servo.detach()

feeder = AutomaticFeeder(camera = 0, delay = 0.5, num_images = 5)

print('Starting')
feeder.motion_detection()

