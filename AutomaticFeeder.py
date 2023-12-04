import torch
import os
import time
import cv2
from gpiozero import MotionSensor
from gpiozero import Servo

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
        self.pir = MotionSensor(18)
        self.motor = Servo(17)

    def motion_detection(self):
        self.pir.when_activated = self.take_pictures()

    def take_pictures(self):
        if not os.path.exists(self.image_path):
            os.makedirs(self.image_path)

        if not self.device.isOpened():
            print("Error: Could not open camera.")
            return
        
        time.sleep(self.first_delay)

        for i in range(self.num_images):
            time.sleep(self.delay)
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

    def get_detections(self):
        print(self.detections)
        return self.detections

    def open_lid(self):
        self.motor


feeder = AutomaticFeeder(camera = 0, delay = 0.5, num_images = 5)


feeder.motion_detection()

print('taking_pictures')
feeder.take_pictures()

print('detecting')
feeder.detect()

print('getting detections')
feeder.get_detections()
