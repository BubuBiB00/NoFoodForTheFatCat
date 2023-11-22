import torch
import os, fnmatch

model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

class AutomaticFeeder:

    def __init__(self) -> None:
        pass
    
    def take_pictures(self):
        pass

    def detect_animal(image_path):
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

        #classes = []
        #for detection in prediction:
        #    label = detection[-1]
        #    classes.append(label)
        #print(f'Klassen: {classes}')
        #return classes

    def classify_animal(classes):
        if 'dog' in classes:
            return 'Hund'
        #elif 'cat' in classes:
        #    return 'Katze'
        #else:
        #    return 'Nicht sicher'

    pictures_path = "C:/Users/Florentin/Documents/Schule/Spl/FinalDogDetection/test_images/"
    sorted_images = fnmatch.filter(os.listdir(pictures_path), 'pic*.jpg')
    for image in sorted_images:

        image_path = os.path.join(pictures_path,image)
        classes = detect_animal(image_path)
        animal_type = classify_animal(classes)
        print(animal_type)

