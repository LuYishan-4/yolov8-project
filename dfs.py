import numpy as np
import time
import cv2
from ultralytics import YOLO
import supervision as sv
import torch
from p import PIDController
import smbus

address = 0x60

bus = smbus.SMBus(1)

think=False
think2=False

def cccc(paper, plasticbottle,th,th2):
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
def send_data_i2c(lp,rp, think, think2):
    try:
        data = [
            int(rp) >> 8, int(y1) & 0xFF,
            int(lp) >> 8, int(y2) & 0xFF,
            int(think), int(think2)
        ]
        bus.write_i2c_block_data(address, 0x00, data)
    except Exception as e:
        print("I2C error:", e)
rp = 0
lp = 0
mx = 320
my = 180
uy = 179
uy2=20

paper = 0
plasticbottle = 0
print(torch.cuda.is_available())
print(torch.version.cuda)
model = YOLO("best.pt")
model.to("cuda")
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
    detections = detections[detections.confidence > 0.5]

    for box, class_id, confidence in zip(detections.xyxy, detections.class_id, detections.confidence):
        label = model.model.names[class_id]
        x1, y1, x2, y2 = box
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        z = int(y2 - y1)

        pid_x = mx - center_x
        pid_y = y2 - uy

        max_speed = 255
        pid = PIDController(Kp=0.4, Ki=0, Kd=0.0, setpoint=pid_x)
        output1 = pid.compute(2, 1)
        #print(output1)

        pid = PIDController(Kp=0.7, Ki=0, Kd=0.0, setpoint=pid_y)
        output2 = pid.compute(2, 1)
        #print(output2)

        right = output1 + output2
        left = output1 - output2

        right = max(-max_speed, min(max_speed, right))
        left = max(-max_speed, min(max_speed, left))
        rp = abs(right)
        lp = abs(left)
        fps_start = cv2.getTickCount()

        print(f"Right Motor Speed: {rp}")
        print(f"Left Motor Speed: {lp}")
        if label == "crumpledPaper":
            paper+=1
        if label == 'bottles':
            plasticbottle+=1


    print(f"paper{think}")
    print(f"bottle{think2}")
    send_data_i2c(rp, lp, think, think2)
    frame = draw.annotate(scene=frame, detections=detections)
    cv2.imshow("ii", frame)
    if start_time >= 5:
        paper, plasticbottle, t0 ,think,think2= cccc(paper, plasticbottle,think,think2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()