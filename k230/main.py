import time
from media.sensor import *
from media.display import *
from media.media import *
from ybUtils.YbUart import YbUart

# ==================== 配置 ====================
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
CENTER_X = DISPLAY_WIDTH // 2
CENTER_Y = DISPLAY_HEIGHT // 2

# 橙色阈值 (Lab)
ORANGE_THRESHOLD = (40, 85, 20, 60, 30, 80)

# 发送间隔 (ms)
SEND_INTERVAL_MS = 25

# ==================== 状态 ====================
last_send = 0
uart = None

def init_system():
    global uart
    uart = YbUart(baudrate=115200)
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor

def init_display():
    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()

def main():
    global last_send, uart

    try:
        sensor = init_system()
        init_display()
        sensor.run()
        clock = time.clock()

        while True:
            clock.tick()
            img = sensor.snapshot()

            # 十字准心
            img.draw_cross(CENTER_X, CENTER_Y, size=15, thickness=2, color=(0, 255, 0))
            img.draw_circle(CENTER_X, CENTER_Y, 20, color=(0, 255, 0), thickness=1)

            # 目标检测
            blobs = img.find_blobs([ORANGE_THRESHOLD], area_threshold=200, merge=True)

            if blobs:
                blob = max(blobs, key=lambda b: b[4])
                bx, by = blob[5], blob[6]

                # 绘制识别框
                img.draw_rectangle(blob[0:4], thickness=3, color=(255, 165, 0))
                img.draw_cross(bx, by, size=10, thickness=2, color=(255, 165, 0))
                img.draw_line(CENTER_X, CENTER_Y, bx, by, color=(200, 200, 0), thickness=1)

                state_text = "TRACK"
                state_color = (0, 255, 0)
            else:
                bx, by = 0, 0
                state_text = "LOST"
                state_color = (255, 0, 0)

            # 定时发送坐标
            now = time.ticks_ms()
            if now - last_send > SEND_INTERVAL_MS:
                if blobs:
                    cmd = "X%dY%d\n" % (bx, by)
                    uart.send(cmd)
                last_send = now

            # OSD信息
            img.draw_string_advanced(5, 5, 24, state_text, color=state_color)
            img.draw_string_advanced(5, 30, 18, "X:%d Y:%d" % (bx, by), color=(255, 255, 255))
            img.draw_string_advanced(5, 50, 18, "FPS:%.1f" % clock.fps(), color=(255, 255, 255))

            Display.show_image(img)

    except KeyboardInterrupt:
        print("stop")
    finally:
        if 'sensor' in locals():
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()

if __name__ == "__main__":
    main()
