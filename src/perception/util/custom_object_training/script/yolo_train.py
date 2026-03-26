import os
from ultralytics import YOLO

current_dir = os.path.dirname(os.path.abspath(__file__))

data_path = os.path.join(current_dir, 'data.yaml')
model = YOLO(os.path.join(current_dir, 'yolo26n.pt'))


epochs = 200
batch = 16
imgsz = 640
if __name__ == '__main__':
    results = model.train(data=data_path,
                      epochs=epochs, 
                      batch=batch, 
                      imgsz=imgsz, 
                      name='redball',
                      device='cuda')
