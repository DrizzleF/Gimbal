import time, os, sys, gc, math
from media.sensor import *
from media.display import *
from media.media import *
import cv_lite
from ybUtils.YbUart import YbUart

# ==================== 配置 ====================
IMAGE_WIDTH  = 640
IMAGE_HEIGHT = 480
CENTER_X = IMAGE_WIDTH // 2
CENTER_Y = IMAGE_HEIGHT // 2

# 矩形检测参数
canny_thresh1       = 50
canny_thresh2       = 150
approx_epsilon      = 0.04
area_min_ratio      = 0.001
max_angle_cos       = 0.3
gaussian_blur_size  = 5

AREA_THRESHOLD      = 2000   # 最小面积阈值（国一方案参考值）
MIN_EDGE_LEN        = 30     # 每条边最小长度(px)
EDGE_DIFF_THRESH    = 120    # 对边长度差阈值(px)
ANGLE_TOLERANCE     = 30     # 平行/垂直判定容差(度)

def edge_len(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def edge_angle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

def angle_diff_deg(a1, a2):
    d = abs(a1 - a2) % math.pi
    if d > math.pi / 2:
        d = math.pi - d
    return math.degrees(d)

def validate_rect(r):
    """验证矩形是否合格：对边平行、邻边垂直、边长合格"""
    x0, y0, x1, y1, x2, y2, x3, y3 = r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11]
    # 四条边长度
    l01 = edge_len(x0, y0, x1, y1)
    l12 = edge_len(x1, y1, x2, y2)
    l23 = edge_len(x2, y2, x3, y3)
    l30 = edge_len(x3, y3, x0, y0)
    # 每条边 >= 30px
    if l01 < MIN_EDGE_LEN or l12 < MIN_EDGE_LEN or l23 < MIN_EDGE_LEN or l30 < MIN_EDGE_LEN:
        return False
    # 对边长度差 < 120px
    if abs(l01 - l23) > EDGE_DIFF_THRESH:
        return False
    if abs(l12 - l30) > EDGE_DIFF_THRESH:
        return False
    # 对边平行、邻边垂直
    a01 = edge_angle(x0, y0, x1, y1)
    a12 = edge_angle(x1, y1, x2, y2)
    a23 = edge_angle(x2, y2, x3, y3)
    a30 = edge_angle(x3, y3, x0, y0)
    # 对边平行: a01 vs a23, a12 vs a30
    if angle_diff_deg(a01, a23) > ANGLE_TOLERANCE:
        return False
    if angle_diff_deg(a12, a30) > ANGLE_TOLERANCE:
        return False
    # 邻边垂直: a01 vs a12
    perp_err = abs(angle_diff_deg(a01, a12) - 90)
    if perp_err > ANGLE_TOLERANCE:
        return False
    return True

# ==================== 主程序 ====================
sensor = None

try:
    uart = YbUart(baudrate=115200)

    # ---- 初始化 ----
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)

    Display.init(Display.ST7701, width=IMAGE_WIDTH, height=IMAGE_HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()

    clock = time.clock()
    image_shape = [IMAGE_HEIGHT, IMAGE_WIDTH]

    while True:
        clock.tick()
        os.exitpoint()
        img = sensor.snapshot()
        img_np = img.to_numpy_ref()

        # ---- 灰度矩形检测 ----
        rects = cv_lite.grayscale_find_rectangles_with_corners(
            image_shape, img_np,
            canny_thresh1, canny_thresh2,
            approx_epsilon, area_min_ratio,
            max_angle_cos, gaussian_blur_size
        )

        target_x = CENTER_X
        target_y = CENTER_Y
        found = False
        area = 0

        if rects:
            # 取面积最大的合格矩形
            valid_rects = [r for r in rects if validate_rect(r)]
            if valid_rects:
                r = max(valid_rects, key=lambda r: r[2] * r[3])
                area = r[2] * r[3]

            if valid_rects and area >= AREA_THRESHOLD:
                # 对角线交点取靶心
                target_x = (r[4] + r[8]) // 2
                target_y = (r[5] + r[9]) // 2
                found = True

                # 画矩形框和角点
                img.draw_rectangle(r[0], r[1], r[2], r[3],
                                   color=(255, 255, 255), thickness=2)
                img.draw_cross(r[4],  r[5],  color=(255, 255, 255), size=5, thickness=2)
                img.draw_cross(r[6],  r[7],  color=(255, 255, 255), size=5, thickness=2)
                img.draw_cross(r[8],  r[9],  color=(255, 255, 255), size=5, thickness=2)
                img.draw_cross(r[10], r[11], color=(255, 255, 255), size=5, thickness=2)

        # ---- 发送像素误差 + 有效标志 ----
        if found:
            err_x = target_x - CENTER_X
            err_y = target_y - CENTER_Y
            uart.write("X%dY%dZ1\n" % (err_x, err_y))
            print("X%dY%dZ1\n" % (err_x, err_y))
        else:
            uart.write("X0Y0Z0\n")

        # ---- 画准星和目标 ----
        img.draw_cross(target_x, target_y, color=(0, 0, 0), size=10, thickness=2)
        img.draw_circle(CENTER_X, CENTER_Y, 4, color=(0, 0, 0), thickness=2, fill=False)

        # ---- OSD ----
        img.draw_string_advanced(0, 0, 30, "FPS:%.1f" % clock.fps(), color=(0, 0, 0))
        if found:
            img.draw_string_advanced(0, 30, 30, "area:%d" % area, color=(0, 0, 0))
            img.draw_string_advanced(0, 60, 24, "TRACK", color=(0, 0, 0))
        else:
            img.draw_string_advanced(0, 60, 24, "LOST", color=(255, 0, 0))

        Display.show_image(img)
        gc.collect()

except KeyboardInterrupt:
    print("user stop")
except BaseException as e:
    print(f"Exception: {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
