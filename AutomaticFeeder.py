import torch
import os
import time
import cv2

class AutomaticFeeder:
    def __init__(self, camera, delay, num_images) -> None:
        self.detections = []
        self.image_path = 'C:/Users/Florentin/Documents/Schule/Spl/FinalDogDetection/test_images/'
        self.device = cv2.VideoCapture(camera)
        self.delay = delay
        self.num_images = num_images
        self.model = 0

    def take_pictures(self):
        if not os.path.exists(self.image_path):
            os.makedirs(self.image_path)

        if not self.device.isOpened():
            print("Error: Could not open camera.")
            return

        for i in range(self.num_images):
            time.sleep(self.delay)
            try:
                ret, picture = self.device.read()

            except Exception as e:
                print(e)

            try:
                filename = os.path.join(self.image_path, f'image_{i+1}.jpg')
                print(filename)
                cv2.imwrite(filename, picture)
                
            except Exception as e:
                print(e)

            finally:
                self.device.release()
                cv2.destroyAllWindows()

    def detect(self):
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        detections = self.model(self.image_path)
        detections.show()


    def clean_image_folder():
        pass

    def get_detections(self):
        return self.detections


feeder = AutomaticFeeder(camera = 1, delay = 0.5, num_images = 5)
feeder.take_pictures()
feeder.detect()

def detect_animal(self, image_path):
        results = model(image_path)
        results.show()
        prediction = results.pred[0]
        class_names = results.names

        classes = []
        for det in prediction:
            label_index = int(det[-1])
            class_name = class_names[label_index]
            classes.append(class_name)

        return classes

    
def classify_animal(self, classes):
        if 'dog' in classes:
            return 'Hund'

        sorted_images = os.listdir(self.output_folder)
        for image in sorted_images:

            image_path = os.path.join(self.output_folder,image)
            classes = self.detect_animal(image_path)
            animal_type = self.classify_animal(classes)
            print(animal_type)

