from ultralytics import YOLO

model = YOLO("best.pt")
# model.train(imgsz=640, epochs=150)
model.predict(source=0, show=True, conf=0.40)
model = YOLO("gamepiece2026.pt")
model.predict(source="http://10.91.63.2:1181/?action=stream", show=True, conf=0.20)

# from networktables import NetworkTables
# import time

# NetworkTables.initialize(server="10.91.63.2")
# time.sleep(1)

# for name in ["limelight-back", "limelight", "limelight_front", "limelight-front"]:
#     t = NetworkTables.getTable(name)
#     try:
#         print(name, "keys:", t.getKeys())
#     except Exception as e:
#         print(name, "erro:", e)