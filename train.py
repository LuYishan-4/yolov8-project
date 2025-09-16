from ultralytics import YOLO
import torch

def main():
 model = YOLO('yolov8n.pt')
 print(torch.cuda.is_available())
 print(torch.version.cuda)
 model.train(
data="D:/xvideo.v1i.yolov8/data.yaml",
 epochs=50,
batch =32,
 imgsz=640,
 workers=4
     )
if __name__ == '__main__':
    import multiprocessing
    multiprocessing.freeze_support()
    main()