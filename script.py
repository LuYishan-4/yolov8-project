import cv2
from ultralytics import YOLO
import supervision as sv
import torch

import time
import sumbus as bus


address = 0x60


left = 0
right = 0
think = False
think2 = False
x11 = x22 = y11 = y22 = 0
center_x = center_y = z = 0
confidencee = 0.0
def read_motor_speeds():
    try:
        data = bus.read_i2c_block_data(address, 0x00, 2)
        left = data[0]
        right = data[1]
        return left, right
    except Exception as ee:
        print("error:", ee)
        return None, None




def cccc(paper, plasticbottle, th, th2):
    if paper > plasticbottle:
        print('paper')
        th = True
        th2 = False
    elif paper < plasticbottle:
        print('bottle')
        th = False
        th2 = True
    else:
        print('not')
        th = False
        th2 = False
    return 0, 0, time.time(), th, th2

def send_data_i2c(x1, x2, y1, y2, cx, cy, z, think, think2):
    try:
        data = [
            int(x1) >> 8, int(x1) & 0xFF,
            int(x2) >> 8, int(x2) & 0xFF,
            int(y1) >> 8, int(y1) & 0xFF,
            int(y2) >> 8, int(y2) & 0xFF,
            int(cx) >> 8, int(cx) & 0xFF,
            int(cy) >> 8, int(cy) & 0xFF,
            int(z) >> 8, int(z) & 0xFF,
            int(think), int(think2)
        ]
        bus.write_i2c_block_data(address, 0x00, data)
    except Exception as e:
        print("I2C error:", e)


paper = 0
plasticbottle = 0
print(torch.cuda.is_available())
print(torch.version.cuda)
model = YOLO("best.pt")

frame_count = 0
skip_rate = 2
last_results = None

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE, -5)

draw = sv.BoxAnnotator(thickness=2)
t0 = time.time()
while True:
    start_time = time.time() - t0
    ret, frame = cap.read()
    if not ret:
        print("no")
        break

    if frame_count % skip_rate == 0:
        last_results = model.predict(source=frame, stream=False)[0]

    results = last_results
    frame_count += 1

    detections = sv.Detections.from_ultralytics(results)
    detections = detections[detections.confidence > 0.7]

    confidencee = 0.0
    x11 = x22 = y11 = y22 = center_x = center_y = z = 0

    for box, class_id, confidence in zip(detections.xyxy, detections.class_id, detections.confidence):
        label = model.model.names[class_id]
        if label not in ["bottles", "crumpledPaper"]:
            continue
        x1, y1, x2, y2 = box
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        z = int(y2 - y1)
        x11 = int(x1)
        x22 = int(x2)
        y11 = int(y1)
        y22 = int(y2)
        confidencee = float(confidence)


        if label == "crumpledPaper":
         paper += 1
        if label == 'bottles':
         plasticbottle += 1

    print(f"paper{think}")
    print(f"bottle{think2}")
    print(f" Confidence: {confidencee:.2%}, Position: X={center_x}, Y={center_y}, Z={z}")
    send_data_i2c(x11,x22,y11,y22,center_x,center_y,z,think,think2)
    left_speed, right_speed = read_motor_speeds()
    if left_speed is not None:
        print(f"Left: {left_speed}, Right: {right_speed}")
    frame = draw.annotate(scene=frame, detections=detections)
    cv2.imshow("ii", frame)
    if start_time >= 5:
        paper, plasticbottle, t0 ,think,think2= cccc(paper, plasticbottle,think,think2)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()