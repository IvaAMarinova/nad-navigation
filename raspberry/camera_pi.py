import cv2
import time
import subprocess
import numpy as np
import onnxruntime as ort
import threading
import socket
import os

# --- CONFIG ---
MODEL_PATH = "best_final.onnx"
UDP_IP = "127.0.0.1"  
UDP_PORT = 9001
WIDTH, HEIGHT = 320, 320
CONF_THRESHOLD = 0.45

class BalloonDetector:
    def __init__(self, model_path):
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        opts.intra_op_num_threads = 4

        self.session = ort.InferenceSession(model_path, opts, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        self.frame = None
        self.running = True

        # Споделени данни между нишките
        self.lock = threading.Lock()
        self.latest_data = {
            "detected": False,
            "conf": 0.0,
            "cx": 0.0,
            "cy": 0.0,
            "size": 0.0000
        }

    def preprocess(self, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = img.transpose(2, 0, 1)
        img = np.ascontiguousarray(img, dtype=np.float32) / 255.0
        return np.expand_dims(img, axis=0)

    def run_inference(self):
        while self.running:
            if self.frame is not None:
                blob = self.preprocess(self.frame)
                outputs = self.session.run([self.output_name], {self.input_name: blob})
                output = outputs[0][0] # Shape (6, 2100)

                # Търсим балона (Row 5 - Confidence)
                conf_row = output[5, :]
                best_idx = np.argmax(conf_row)
                max_conf = conf_row[best_idx]

                with self.lock:
                    if max_conf > CONF_THRESHOLD:
                        # Извличаме YOLO координатите (Row 0,1,2,3)
                        x_px = output[0, best_idx]
                        y_px = output[1, best_idx]
                        w_px = output[2, best_idx]
                        h_px = output[3, best_idx]

                        # Нормализация за drone-nav [-1, 1]
                        cx = (x_px - WIDTH/2) / (WIDTH/2)
                        cy = (y_px - HEIGHT/2) / (HEIGHT/2)
                        size = (w_px * h_px) / (WIDTH * HEIGHT)

                        self.latest_data = {
                            "detected": True,
                            "conf": float(max_conf),
                            "cx": float(cx),
                            "cy": float(cy),
                            "size": float(size)
                        }
                    else:
                        self.latest_data = {
                            "detected": False, "conf": 0.0, "cx": 0.0, "cy": 0.0, "size": 0.0000
                        }
            else:
                time.sleep(0.01)

def main():
    # Настройка на UDP сокета
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest_addr = (UDP_IP, UDP_PORT)

    detector = BalloonDetector(MODEL_PATH)
    threading.Thread(target=detector.run_inference, daemon=True).start()

    # Камера пайп
    cmd = [
        "rpicam-vid", "-t", "0", "--inline", "--width", str(WIDTH), "--height", str(HEIGHT),
        "--framerate", "30", "--codec", "yuv420", "--nopreview", "-o", "-"
    ]
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**5)
    frame_size = int(WIDTH * HEIGHT * 1.5)

    start_time = time.time()
    print(f"🚀 SENDING UDP TO {UDP_IP}:{UDP_PORT}...")

    try:
        while True:
            raw_data = process.stdout.read(frame_size)
            if not raw_data: break

            yuv = np.frombuffer(raw_data, dtype=np.uint8).reshape((int(HEIGHT * 1.5), WIDTH))
            detector.frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

            # 1. Изчисляваме времето t
            t_now = time.time() - start_time

            # 2. Взимаме последната детекция
            with detector.lock:
                d = detector.latest_data

            # 3. Стриктно форматиране на CSV реда
            # t,Detected,conf,cx,cy,size
            csv_line = f"{t_now:.3f},{str(d['detected']).capitalize()},{d['conf']:.3f},{d['cx']:.3f},{d['cy']:.3f},{d['size']:.4f}"

            # 4. Изпращане по UDP
            sock.sendto(csv_line.encode(), dest_addr)

            # Дебъг принт (можеш да го спреш за скорост)
            print(csv_line)

    except KeyboardInterrupt:
        detector.running = False
        process.terminate()
        sock.close()

if __name__ == "__main__":
    main()
