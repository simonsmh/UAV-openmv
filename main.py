# main.py
import math
import struct
import sys
import time

import gif
import image
import mjpeg
import network
import pyb
import sensor
import usocket
import utime
from machine import Pin, I2C
from vl53l1x import VL53L1X
import micropython


"""
单板初始化
"""

micropython.alloc_emergency_exception_buf(100)
clock = time.clock()


"""
USART 通信
"""

# USART 字符列表
R = ord("R")
L = ord("L")
S = ord("S")
B = ord("B")
G = ord("G")
C = ord("C")
F = ord("F")
U = ord("U")
D = ord("D")
E = ord("E")
H = ord("H")
I = ord("I")

# USART 初始化
uart = pyb.UART(1, 115200, timeout_char=500)

# USART 串口协议
def send_direction_packet(direct, velcity):
    s = 0xAA + 0x8C + direct + (int(velcity / 256)) + (int(velcity % 256))
    s = int(s % 256)
    temp_flow = struct.pack("<BBBBBhB", 0x00, 0xAA, 0x89, 3, direct, int(velcity), s)
    uart.write(temp_flow)


def send_yaw_packet(direct, velcity):
    s = 0xAA + 0x9B + direct + (int(velcity / 256)) + (int(velcity % 256))
    s = int(s % 256)
    temp_flow = struct.pack("<BBBBBhB", 0x00, 0xAA, 0x98, 3, direct, int(velcity), s)
    uart.write(temp_flow)


"""
WiFi模块初始化
"""

SSID = "OPENMV_AP"  # Network SSID
KEY = "1234567890"  # Network key (must be 10 chars)
HOST = ""  # Use first available interface
PORT = 8080  # Arbitrary non-privileged port

# wlan = network.WINC(mode=network.WINC.MODE_AP)
# wlan.start_ap(SSID, key=KEY, security=wlan.OPEN, channel=2)


"""
视频录制
"""
record_pin = Pin("P6", Pin.IN, Pin.PULL_UP)


def saveVideo(save_start=1, frames=100000, format="mjpeg"):
    global save_flag
    if save_flag:
        global FRAMES
        if format == "mjpeg":
            global m
        else:
            global g
        if save_start == 0:
            pyb.LED(3).on()
            if format == "mjpeg":
                m = mjpeg.Mjpeg("test_%s.mjpeg" % pyb.rng())
            else:
                g = gif.Gif("test_%s.gif" % pyb.rng())
            FRAMES = frames
        elif save_start == 1:
            global img
            # 记录帧
            if record_pin.value() and FRAMES > 0:
                if format == "mjpeg":
                    m.add_frame(img, quality=30)
                else:
                    g.add_frame(img, delay=6)  # 10FPS
                FRAMES -= 1
            else:
                saveVideo(2)
        else:
            pyb.LED(3).off()
            if format == "mjpeg":
                m.close(clock.fps())
            else:
                g.close()
            save_flag = 0


"""
PWM蜂鸣器控制
"""
ch = pyb.Timer(4, freq=3500).channel(1, pyb.Timer.PWM, pin=Pin("P7"))
ch.pulse_width_percent(0)


"""
超声波中断控制
"""
wave_echo_pin = Pin("P8", Pin.IN, Pin.PULL_DOWN)
wave_trig_pin = Pin("P9", Pin.OUT_PP, Pin.PULL_DOWN)

wave_distance = 0
tim_counter = 0
flag_wave = 0


def wave_distance_process():
    global flag_wave
    if flag_wave == 0:
        wave_trig_pin.value(0)
        utime.sleep_us(2)
        wave_trig_pin.value(1)
        utime.sleep_us(15)
        wave_trig_pin.value(0)
    elif flag_wave == 2:
        global tim_counter
        global wave_distance
        wave_distance = tim_counter * 2 * 0.017
        flag_wave = 0


def wave_handler(line):
    global flag_wave
    global tim_counter
    if wave_echo_pin.value():
        tim.init(prescaler=480, period=65535)
        flag_wave = 1
    else:
        tim.deinit()
        tim_counter = tim.counter()
        tim.counter(0)
        flag_wave = 2


tim = pyb.Timer(2, prescaler=480, period=65535)  # 相当于freq=0.5M
# wave_echo_pin.irq(handler=wave_handler, trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING)
extint = pyb.ExtInt(
    wave_echo_pin, pyb.ExtInt.IRQ_RISING_FALLING, pyb.Pin.PULL_DOWN, wave_handler
)

"""
I2C激光，P4 SCL, P5 SDA, 3.3V
"""
i2c = I2C(2)
distance = VL53L1X(i2c)


"""
姿态识别解算控制
"""


def judge_stop():
    pyb.LED(1).on()
    send_direction_packet(S, 0)


def judge_end():
    global judge_flag
    judge_flag = 0
    send_direction_packet(E, 0)
    pyb.LED(2).on()
    saveVideo(2)
    # uart.write("\r\n###SUCCESS###")


def judge_direction_blob(blob, speed=10):
    global img
    pyb.LED(1).off()
    x_cal = blob.cx() - WINDOW_CENTER_X
    y_cal = blob.cy() - WINDOW_CENTER_Y
    x_speed = min(speed, x_cal)
    y_speed = min(speed, y_cal)
    if judge_flag == 1:
        if abs(x_cal) > 10 or abs(y_cal) > 10:
            if x_cal > 0:
                img.draw_string(2, 64, "R", color=128, scale=2, mono_space=False)
                send_direction_packet(R, x_speed)
            else:
                img.draw_string(2, 64, "L", color=128, scale=2, mono_space=False)
                send_direction_packet(L, x_speed)
            if y_cal > 0:
                img.draw_string(18, 64, "B", color=128, scale=2, mono_space=False)
                send_direction_packet(B, y_speed)
            else:
                img.draw_string(18, 64, "G", color=128, scale=2, mono_space=False)
                send_direction_packet(G, y_speed)
        else:
            img.draw_string(2, 64, "E", color=128, scale=2, mono_space=False)
            judge_end()


def judge_direction_line(line, speed=6000):
    global img
    pyb.LED(1).off()
    x_cal = (line.x1() + line.x2()) / 2 - WINDOW_CENTER_X
    speed = min(abs(x_cal) * 60, speed)
    if judge_flag == 1:
        # send_direction_packet(G, 5)
        if abs(x_cal) > 10:
            if x_cal > 0:
                img.draw_string(
                    2,
                    56 * SCALE,
                    "F: %s" % speed,
                    color=128,
                    scale=SCALE,
                    mono_space=False,
                )
                send_yaw_packet(F, speed)
            else:
                img.draw_string(
                    2,
                    56 * SCALE,
                    "C %s" % speed,
                    color=128,
                    scale=SCALE,
                    mono_space=False,
                )
                send_yaw_packet(C, speed)
        else:
            send_yaw_packet(C, 0)


def DistanceToCenter(x, y):
    x_cal = WINDOW_CENTER_X - x
    y_cal = WINDOW_CENTER_Y - y
    return math.sqrt(math.pow(x_cal, 2) + math.pow(y_cal, 2))


def CompareBlob(blob_1, blob_2):
    # 大小 (最小)
    # tmp = DistanceToCenter(blob_1.cx(),blob_1.cy()) - DistanceToCenter(blob_2.cx(),blob_2.cy())
    # 面积 (最大)
    # tmp = blob_2.pixels() - blob_1.pixels()
    # 圆度 (最大)
    tmp = blob_2.roundness() - blob_1.roundness()
    if tmp > 0:
        return blob_2
    else:
        return blob_1


def LineCenter(line):
    x_cal = (line.x1() + line.x2()) / 2
    y_cal = (line.y1() + line.y2()) / 2
    return (x_cal, y_cal)


def LineAngle(line_1, line_2):
    return abs(line_1.theta() - line_2.theta())


def LineVertical(lines):
    r = []
    if len(lines) >= 2:
        for i in range(len(lines)):
            for j in range(len(lines) - 1):
                if LineAngle(lines[i], lines[j]) >= 80:
                    r.append((lines[i], lines[j]))
    return r


def LineCrossCenter(line_1, line_2):
    a1 = line_1.y2() - line_1.y1()
    b1 = line_1.x1() - line_1.x2()
    c1 = line_1.x2() * line_1.y1() - line_1.x1() * line_1.y2()
    a2 = line_2.y2() - line_2.y1()
    b2 = line_2.x1() - line_2.x2()
    c2 = line_2.x2() * line_2.y1() - line_2.x1() * line_2.y2()
    m = a1 * b2 - a2 * b1
    if m != 0:
        cross_x = int((b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1))
        cross_y = int((c1 * a2 - c2 * a1) / (a1 * b2 - a2 * b1))
        return (cross_x, cross_y)
    else:
        return (-1, -1)


def LineDistanceToCenter(line):
    (x, y) = LineCenter(line)
    return DistanceToCenter(x, y)


def DistanceToLine(line):
    a = line.y2() - line.y1()
    b = line.x1() - line.x2()
    c = line.x2() * line.y1() - line.x1() * line.y2()
    (cx, cy) = LineCenter(line)
    dis = abs(a * WINDOW_CENTER_X + b * WINDOW_CENTER_Y + c) / math.sqrt(
        math.pow(a, 2) + math.pow(b, 2)
    )
    return dis


def VerticleAngle(line):
    return 90 - abs(line.theta() - 90)


def HorizonAngle(line):
    return abs(line.theta() - 90)


def CompareLine(line_1, line_2):
    # 大小 (最小)
    # tmp = LineDistanceToCenter(line_1) - LineDistanceToCenter(line_2)
    # tmp = DistanceToLine(line_1) - DistanceToLine(line_2)
    # 竖直 (最小)
    tmp = VerticleAngle(line_1) - VerticleAngle(line_2)
    # 水平 (最小)
    # tmp = HorizonAngle(line_1) - HorizonAngle(line_2)
    if tmp > 0:
        return line_2
    else:
        return line_1


def CompareCross(cross_x_1, cross_y_1, cross_x_2, cross_y_2):
    tmp = DistanceToCenter(cross_x_1, cross_y_1) - DistanceToCenter(
        cross_x_2, cross_y_2
    )
    if tmp > 0:
        return (cross_x_2, cross_y_2)
    else:
        return (cross_x_1, cross_y_1)


"""
感光度校准
"""


def ISO_Tune(amount):
    gain = sensor.get_gain_db()
    # 感光度自动调节 范围11-16左右
    if amount == 0 and gain > 11:
        sensor.set_auto_gain(False, gain_db=sensor.get_gain_db() * 0.9)  # 画面明亮时调低gain
    elif amount >= 3 and gain < 16:
        sensor.set_auto_gain(
            False, gain_db=sensor.get_gain_db() * 1.3
        )  # 画面过暗时调高gain,稳定性差


def Res_init():
    global WINDOW_CENTER_X
    global WINDOW_CENTER_Y
    global RES
    global SCALE
    if RES == 1:
        # VGA
        WINDOW_CENTER_X = 320
        WINDOW_CENTER_Y = 240
        sensor.reset()
        sensor.set_pixformat(sensor.GRAYSCALE)
        # VGA640*480
        sensor.set_framesize(sensor.VGA)
        sensor.set_windowing((WINDOW_CENTER_X * 2, WINDOW_CENTER_Y * 2))
        sensor.skip_frames(time=1000)
        # sensor.set_auto_gain(False,gain_db=20) # 画面明亮时调低gain
        # sensor.set_auto_whitebal(True,(-5.5, -6.5, -3)) # must be turned off for color tracking
        # sensor.set_auto_exposure(True, exposure_us = 20000)
        RES = 0
        SCALE = 4
    if RES == 2:
        # find_qrcodes
        WINDOW_CENTER_X = 240
        WINDOW_CENTER_Y = 240
        sensor.reset()
        sensor.set_pixformat(sensor.GRAYSCALE)
        # VGA640*480
        sensor.set_framesize(sensor.VGA)
        sensor.set_windowing((WINDOW_CENTER_X * 2, WINDOW_CENTER_Y * 2))
        sensor.skip_frames(time=1000)
        # sensor.set_auto_gain(False,gain_db=20) # 画面明亮时调低gain
        # sensor.set_auto_whitebal(True,(-5.5, -6.5, -3)) # must be turned off for color tracking
        # sensor.set_auto_exposure(True, exposure_us = 20000)
        RES = 0
        SCALE = 4
    elif RES == 3:
        WINDOW_CENTER_X = 160
        WINDOW_CENTER_Y = 120
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        # QVGA320*240
        sensor.set_framesize(sensor.QVGA)
        sensor.set_windowing((WINDOW_CENTER_X * 2, WINDOW_CENTER_Y * 2))
        sensor.skip_frames(time=1000)
        # sensor.set_auto_gain(False,gain_db=20) # 画面明亮时调低gain
        sensor.set_auto_whitebal(False)  # must be turned off for color tracking
        # sensor.set_auto_exposure(True, exposure_us = 20000)
        RES = 0
        SCALE = 2
    elif RES == 4:
        WINDOW_CENTER_X = 320
        WINDOW_CENTER_Y = 240
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        # VGA640*480 分割400x300进Buffer 镜头矫正1.5
        sensor.set_framesize(sensor.VGA)
        sensor.set_windowing((WINDOW_CENTER_X * 2, WINDOW_CENTER_Y * 2))
        sensor.skip_frames(time=1000)
        # sensor.set_auto_gain(False,gain_db=20) # 画面明亮时调低gain
        sensor.set_auto_whitebal(False)  # must be turned off for color tracking
        # sensor.set_auto_exposure(True, exposure_us = 20000)
        RES = 0
        SCALE = 2


"""
主循环
"""

THRESHOLD = [(255, 255), (0, 9)]
temp_threshold = 0
NUM = 0
tof_distance = 0
state_flag = 0
judge_flag = 1
save_flag = 0
DEBUG = 1

while True:
    ##等待接受任务信号
    if DEBUG:
        if state_flag == 0:
            state_flag = 2
            RES = 3
            Res_init()
            saveVideo(0, frames=5000)

    else:
        while state_flag == 0:
            pyb.LED(1).on()
            if uart.readchar() == H:
                state_flag = 2
                RES = 3
                Res_init()
                saveVideo(0, frames=5000)
                pyb.LED(1).off()
            else:
                state_flag = 0

    Res_init()
    clock.tick()

    if state_flag == -1:
        img = sensor.snapshot()
        roi = (0, 0, WINDOW_CENTER_X * 2, WINDOW_CENTER_Y * 2)
    if state_flag == 1:

        img = sensor.snapshot()
        # temp_threshold = img.get_histogram().get_threshold().value()  # Otsu

        ###############
        # 寻直线，寻交叉点
        roi = (0, 0, WINDOW_CENTER_X * 2, WINDOW_CENTER_Y * 2)
        lines_pre = img.find_lines(
            x_stride=10,
            y_stride=1,
            roi=roi,
            threshold=5000,
            theta_margin=40,
            rho_margin=30,
        )

        ###########
        # 寻找最近的竖直线
        if len(lines_pre):
            near_line = lines_pre[0]
            for line in lines_pre:
                near_line = CompareLine(near_line, line)

            img.draw_line(near_line.line(), color=128, thickness=5)
            judge_direction_line(near_line)
            ############
            # 寻找最近的交叉点
            lines = LineVertical(lines_pre)
            if len(lines):
                crosses = []
                for (line_1, line_2) in lines:
                    img.draw_line(line_1.line())
                    img.draw_line(line_2.line())
                    crosses.append(LineCrossCenter(line_1, line_2))
                    for (cross_x, cross_y) in crosses:
                        img.draw_circle(cross_x, cross_y, 4)
                if len(crosses):
                    (near_cross_x, near_cross_y) = crosses[0]
                    for (cross_x, cross_y) in crosses:
                        (near_cross_x, near_cross_y) = CompareCross(
                            near_cross_x, near_cross_y, cross_x, cross_y
                        )

                img.draw_circle(near_cross_x, near_cross_y, 6)
        else:
            judge_stop()

    elif state_flag == 2:

        img = sensor.snapshot()
        ###############
        # 寻找黄色物块
        roi = (
            round(WINDOW_CENTER_X / 2),
            round(WINDOW_CENTER_Y / 2),
            WINDOW_CENTER_X,
            WINDOW_CENTER_Y,
        )

        blobs = img.find_blobs(
            [(30, 100, -15, 127, 20, 57)], roi=roi, area_threshold=600, merge=True
        )
        if len(blobs):
            ch.pulse_width_percent(50)
            for blob in blobs:
                img.draw_rectangle(blob.rect())
            ##############
            # 进入下一个模式
            state_flag = 3
            RES = 2
        #else:
            #ch.pulse_width_percent(0)

    elif state_flag == 3:

        img = sensor.snapshot()
        #temp_threshold = img.get_histogram().get_threshold().value()
        #img = img.binary([(0, min(125, temp_threshold))])
        ###############
        # 寻找条形码并拍照
        roi = (
            0,
            round(3 * WINDOW_CENTER_Y / 4),
            WINDOW_CENTER_X * 2,
            round(WINDOW_CENTER_Y / 2),
        )

        if NUM < 3:
            codes = img.find_barcodes(roi=roi)
            if len(codes):
                ch.pulse_width_percent(50)
                for code in codes:
                    img.draw_rectangle(code.rect())
                    print(code)

                sensor.snapshot().save("BAR_%s.jpg"%NUM,quallity=100)
                NUM += 1
            #else:
                #ch.pulse_width_percent(0)

        else:
            ch = pyb.Timer(4, freq=1000).channel(1, pyb.Timer.PWM, pin=Pin("P7"))
            ch.pulse_width_percent(0)
            state_flag = 4
            NUM = 0

    elif state_flag == 4:

        img = sensor.snapshot().laplacian(2, sharpen=True)  #
        temp_threshold = img.get_histogram().get_threshold().value()
        img = img  # .binary([(0, max(125, temp_threshold))]).invert().open(1)
        ###############
        # 寻找QR码并拍照
        roi = (0, 0, WINDOW_CENTER_X * 2, WINDOW_CENTER_Y * 2)

        if NUM < 3:
            codes = img.find_qrcodes(roi=roi)
            if len(codes):
                ch.pulse_width_percent(50)
                for code in codes:
                    img.draw_rectangle(code.rect())
                    print(code)

                sensor.snapshot().save("QR_%s.jpg"%NUM,quallity=100)
                NUM += 1

        else:
            ch.pulse_width_percent(0)
            state_flag = -1
            NUM = 0

    ################
    # 图像信息打印
    img.draw_rectangle(roi)
    img.draw_cross(WINDOW_CENTER_X, WINDOW_CENTER_Y)
    img.draw_string(
        2,
        0 * SCALE,
        "GAIN: %s" % sensor.get_gain_db(),
        color=128,
        scale=SCALE,
        mono_space=False,
    )
    img.draw_string(
        2, 8 * SCALE, "FPS: %s" % clock.fps(), color=128, scale=SCALE, mono_space=False
    )
    img.draw_string(
        2,
        16 * SCALE,
        "STATE_FLAG: %s" % state_flag,
        color=128,
        scale=SCALE,
        mono_space=False,
    )
    img.draw_string(
        2,
        24 * SCALE,
        "THRESHOLD: %s" % temp_threshold,
        color=128,
        scale=SCALE,
        mono_space=False,
    )
    img.draw_string(
        2,
        32 * SCALE,
        "WAVE_DISTANCE: %s" % wave_distance,
        color=128,
        scale=SCALE,
        mono_space=False,
    )
    img.draw_string(
        2,
        40 * SCALE,
        "TOF_DISTANCE: %s" % tof_distance,
        color=128,
        scale=SCALE,
        mono_space=False,
    )

    saveVideo()
    wave_distance_process()
    tof_distance = distance.read() / 10
