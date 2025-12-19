#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
from datetime import datetime

import RPi.GPIO as GPIO
import cv2

# ============================================================
# GPIO 腳位設定（BCM編號，可自行修改）
# ============================================================
TRIG1 = 23
ECHO1 = 24

TRIG2 = 27
ECHO2 = 22

LED_R = 17
LED_Y = 18
LED_G = 16

BUZZER = 12  # 有源蜂鳴器：HIGH=響；無源需PWM版本

# ============================================================
# 參數設定
# ============================================================
THRESH_NEAR_CM = 10.0       # 任一距離 < 10cm → 紅燈 + 拍照（只拍一次直到離開）
THRESH_DIFF_CM = 3.0        # |d1-d2| > 3cm → 黃燈（蜂鳴器另外有條件）
MEASURE_INTERVAL = 0.15
SAVE_DIR = "/home/user/Desktop/picture"

ECHO_TIMEOUT_SEC = 0.03

# USB Webcam（by-id 固定指定）
CAM_PATH = "/dev/v4l/by-id/usb-046d_0825_179C67D0-video-index0"
CAM_WIDTH = 1280
CAM_HEIGHT = 720
CAM_WARMUP_FRAMES = 10

# ============================================================
# 工具函式
# ============================================================
def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def set_led(r=False, y=False, g=False):
    GPIO.output(LED_R, GPIO.HIGH if r else GPIO.LOW)
    GPIO.output(LED_Y, GPIO.HIGH if y else GPIO.LOW)
    GPIO.output(LED_G, GPIO.HIGH if g else GPIO.LOW)

def buzzer_on():
    GPIO.output(BUZZER, GPIO.HIGH)

def buzzer_off():
    GPIO.output(BUZZER, GPIO.LOW)

def measure_distance_cm(trig_pin: int, echo_pin: int) -> float:
    GPIO.output(trig_pin, GPIO.LOW)
    time.sleep(0.0002)

    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    start_wait = time.time()
    while GPIO.input(echo_pin) == 0:
        if time.time() - start_wait > ECHO_TIMEOUT_SEC:
            return float("inf")
    t_start = time.time()

    while GPIO.input(echo_pin) == 1:
        if time.time() - t_start > ECHO_TIMEOUT_SEC:
            return float("inf")
    t_end = time.time()

    duration = t_end - t_start
    return (duration * 34300.0) / 2.0

def take_photo_usb(cap: cv2.VideoCapture, folder: str) -> str:
    for _ in range(3):
        cap.read()

    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("USB Webcam 擷取失敗：cap.read() 回傳失敗")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"capture_{ts}.jpg"
    path = os.path.join(folder, filename)

    ok = cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    if not ok:
        raise RuntimeError("照片存檔失敗：cv2.imwrite() 失敗")

    return path

# ============================================================
# 主程式
# ============================================================
def main():
    ensure_dir(SAVE_DIR)

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(TRIG1, GPIO.OUT)
    GPIO.setup(ECHO1, GPIO.IN)
    GPIO.setup(TRIG2, GPIO.OUT)
    GPIO.setup(ECHO2, GPIO.IN)

    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_Y, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)

    GPIO.setup(BUZZER, GPIO.OUT)

    set_led(r=False, y=False, g=True)
    buzzer_off()

    cap = cv2.VideoCapture(CAM_PATH, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(
            f"找不到 USB Webcam（CAM_PATH={CAM_PATH}）。\n"
            f"請確認該路徑存在：ls -l {CAM_PATH}\n"
            f"或改用另一個 index（例如 video-index1）。"
        )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

    for _ in range(CAM_WARMUP_FRAMES):
        cap.read()
        time.sleep(0.02)

    near_latched = False

    print("System started (USB Webcam by-id). Press Ctrl+C to stop.")
    try:
        while True:
            d1 = measure_distance_cm(TRIG1, ECHO1)
            d2 = measure_distance_cm(TRIG2, ECHO2)

            # 任一 <10cm
            near_trigger = (d1 < THRESH_NEAR_CM) or (d2 < THRESH_NEAR_CM)

            # diff：若量測失敗就當很大（可視為異常）
            if d1 == float("inf") or d2 == float("inf"):
                diff = float("inf")
            else:
                diff = abs(d1 - d2)

            # ✅ 蜂鳴器新條件：必須同時 (near_trigger) AND (diff > 3cm)
            buzzer_trigger = near_trigger and (diff > THRESH_DIFF_CM)

            # 先統一控制蜂鳴器（避免各分支漏掉）
            if buzzer_trigger:
                buzzer_on()
            else:
                buzzer_off()

            # ====================================================
            # LED/拍照邏輯維持不變
            # ====================================================
            if near_trigger:
                # 任一 <10cm → 紅燈 + 拍照（只拍一次直到離開）
                set_led(r=True, y=False, g=False)

                if not near_latched:
                    try:
                        path = take_photo_usb(cap, SAVE_DIR)
                        near_latched = True
                        print(f"[NEAR] d1={d1:.2f}cm d2={d2:.2f}cm diff={diff} -> Photo saved: {path} | buzzer={buzzer_trigger}")
                    except Exception as e:
                        print(f"[NEAR] d1={d1:.2f}cm d2={d2:.2f}cm diff={diff} -> Photo ERROR: {e} | buzzer={buzzer_trigger}")
                else:
                    print(f"[NEAR] d1={d1:.2f}cm d2={d2:.2f}cm diff={diff} -> already captured | buzzer={buzzer_trigger}")

            else:
                # 離開 <10cm → 解除拍照鎖
                near_latched = False

                # 這段維持：差>3 黃燈；否則綠燈
                if diff > THRESH_DIFF_CM:
                    set_led(r=False, y=True, g=False)
                    if diff == float("inf"):
                        print(f"[DIFF] d1={d1} d2={d2} -> sensor timeout -> Yellow | buzzer={buzzer_trigger}")
                    else:
                        print(f"[DIFF] d1={d1:.2f}cm d2={d2:.2f}cm |diff|={diff:.2f} > {THRESH_DIFF_CM} -> Yellow | buzzer={buzzer_trigger}")
                else:
                    set_led(r=False, y=False, g=True)
                    print(f"[OK]   d1={d1:.2f}cm d2={d2:.2f}cm |diff|={diff:.2f} <= {THRESH_DIFF_CM} -> Green | buzzer={buzzer_trigger}")

            time.sleep(MEASURE_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        try:
            cap.release()
        except Exception:
            pass
        set_led(False, False, False)
        buzzer_off()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
