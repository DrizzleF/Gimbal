import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart

uart = YbUart(baudrate=115200)
pto = YbProtocol()

# 显示参数
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

# 颜色阈值 0=残次品 1=合格品
COLOR_THRESHOLDS = [
    (48, 62, 50, 114, 35, 58),
    (65, 30, -33, 51, -80, -17)
]
DRAW_COLORS = [(128,0,127),(0,255,0)]
COLOR_LABELS = ['残次品','合格品']

# ===================== 稳定计数核心 =====================
good_count = 0
bad_count = 0
# 计数锁：一个物体只计1次
good_locked = False
bad_locked = False
# 计数框（中心点进就计数）
ROI_X1, ROI_Y1 = 200, 100
ROI_X2, ROI_Y2 = 300, 250
# ======================================================

def init_sensor():
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor

def init_display():
    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()

def in_roi(x, y):
    return ROI_X1 < x < ROI_X2 and ROI_Y1 < y < ROI_Y2

def process_blobs(img, threshold_idx):
    global good_count, bad_count, good_locked, bad_locked

    blobs = img.find_blobs([COLOR_THRESHOLDS[threshold_idx]], area_threshold=300, merge=True)

    # ============== 关键：没识别到物体 = 解锁锁（夹走后生效） ==============
    if not blobs:
        if threshold_idx == 1: good_locked = False
        if threshold_idx == 0: bad_locked = False
        return

    # 只取第一个最大的物体（防止多物体干扰）
    blob = blobs[0]
    x = blob[5]
    y = blob[6]

    # 绘制识别框
    img.draw_rectangle(blob[0:4], thickness=4, color=DRAW_COLORS[threshold_idx])
    img.draw_cross(x, y, thickness=2)
    img.draw_string_advanced(blob[0], blob[1]-35, 30, COLOR_LABELS[threshold_idx], color=DRAW_COLORS[threshold_idx])

    # ===================== 计数逻辑=====================
    if in_roi(x, y):
        # 合格品
        if threshold_idx == 1 and not good_locked:
            good_count += 1
            good_locked = True
            #uart.send(f"#X{x}Y{y}Z")
            #uart.send(f"#G{good_count}B{bad_count}Z")
            print(f"✅ 合格品+1：{good_count}")

        # 残次品
        if threshold_idx == 0 and not bad_locked:
            bad_count += 1
            bad_locked = True
            uart.send(f"#X{x}Y{y}Z")
            uart.send(f"#G{good_count}B{bad_count}Z")
            print(f"❌ 残次品+1：{bad_count}")
            print(f"#X{x}Y{y}Z")
            print(f"#G{good_count}B{bad_count}Z")
def main():
    try:
        sensor = init_sensor()
        init_display()
        sensor.run()
        clock = time.clock()

        while True:
            clock.tick()
            img = sensor.snapshot()
            # 画计数框
            img.draw_rectangle([ROI_X1,ROI_Y1,ROI_X2-ROI_X1,ROI_Y2-ROI_Y1], color=(255,255,0), thickness=2)
            # 显示计数
            img.draw_string_advanced(10,10,30,f"合格:{good_count}",(0,255,0))
            img.draw_string_advanced(10,40,30,f"残次:{bad_count}",(255,0,0))

            process_blobs(img, 0) # 残次品
            process_blobs(img, 1) # 合格品
            Display.show_image(img)

    except KeyboardInterrupt:
        print("停止")
    finally:
        if 'sensor' in locals(): sensor.stop()
        Display.deinit()
        MediaManager.deinit()

if __name__ == "__main__":
    main()
