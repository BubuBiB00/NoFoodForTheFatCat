import torch
import os
import cv2
from time import sleep
from gpiozero import MotionSensor
from gpiozero import Servo

class AutomaticFeeder:
    def __init__(self, camera, delay, num_images):
        # Pin setup
        self.servo = Servo(27)
        self.motion_sensor = MotionSensor(17)
        
        self.camera = camera 
        self.detections = []
        self.image_path = 'test_images/'
        self.images = []
        self.delay = delay
        self.first_delay = delay * 1.5
        self.num_images = num_images
        self.model = 0
        self.motion_detected = True

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
                print('Opening lid...')
            else:
                print('No doggo detected :(')

    def detect_motion(self): 
        while self.motion_detected:
            self.motion_sensor.wait_for_active()
            sleep(1)
            self.motion_detected = False

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

    def image_recognition(self):
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

    def detect_dogs(self):
        if 'dog' in self.detections:
            return True
        else:
            return False

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
try:
    feeder.procedure_controller()
except Exception as e:
    print(e)
