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
from machine import Pin

"""
单板初始化
"""


clock = time.clock()


"""
USART 通信
"""

# USART 字符列表
R = ord('R')
L = ord('L')
S = ord('S')
B = ord('B')
G = ord('G')
C = ord('C')
F = ord('F')
U = ord('U')
D = ord('D')
E = ord('E')
H = ord('H')
I = ord('I')

# USART 初始化
uart = pyb.UART(3, 115200, timeout_char=1000)

# USART 串口协议
def send_direction_packet(direct,velcity):
    s = 0xAA+0x8C+direct+(int(velcity/256))+(int(velcity%256))
    s = int(s % 256)
    temp_flow = struct.pack("<BBBBBhB",
                   0x00,
                   0xAA,
                   0x89,
                   3,
                   direct,
                   int(velcity),
                   s)
    uart.write(temp_flow)

def send_yaw_packet(direct,velcity):
    s = 0xAA+0x9B+direct+(int(velcity/256))+(int(velcity%256))
    s = int(s % 256)
    temp_flow = struct.pack("<BBBBhB",
                   0xAA,
                   0x98,
                   3,
                   direct,
                   int(velcity),
                   s)
    uart.write(temp_flow)


"""
WiFi模块初始化
"""

SSID ='OPENMV_AP'    # Network SSID
KEY  ='1234567890'    # Network key (must be 10 chars)
HOST = ''           # Use first available interface
PORT = 8080         # Arbitrary non-privileged port

#wlan = network.WINC(mode=network.WINC.MODE_AP)
#wlan.start_ap(SSID, key=KEY, security=wlan.OPEN, channel=2)


"""
视频录制
"""

def saveVideo(save_start=1,frames=100000,format='mjpeg'):
    global save_flag
    if save_flag:
        global FRAMES
        if format == 'mjpeg':
            global m
        else:
            global g
        if save_start == 0:
            pyb.LED(3).on()
            if format == 'mjpeg':
                m = mjpeg.Mjpeg("test_%s.mjpeg"%pyb.rng())
            else:
                g = gif.Gif("test_%s.gif"%pyb.rng())
            FRAMES = frames
            save_start = 1
        elif save_start == 1:
            global img
            #记录帧
            if FRAMES > 0:
                if format == 'mjpeg':
                    m.add_frame(img,quality=30)
                else:
                    g.add_frame(img,delay=6) #10FPS
                FRAMES -= 1
            else:
                save_start = 2
        else:
            pyb.LED(3).off()
            if format == 'mjpeg':
                m.close(clock.fps())
            else:
                g.close()
            save_flag = 0
            save_start = 0

"""
Pin中断控制
"""
record_pin = Pin('P7', Pin.IN, Pin.PULL_UP)

def record_pin_handler(record_pin):
    saveVideo(2)
    record_pin.irq(trigger=0)

record_pin.irq(handler=record_pin_handler, trigger=Pin.IRQ_RISING)


"""
超声波中断控制
"""
wave_echo_pin = Pin('P8', Pin.IN, Pin.PULL_NONE)
wave_trig_pin = Pin('P9', Pin.OUT_PP, Pin.PULL_DOWN)

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
        wave_distance = tim_counter*0.017
        flag_wave = 0

def wave_handler(line):
    global flag_wave
    global tim_counter
    if wave_echo_pin.value():
        tim.init(prescaler=240, period=65535)
        flag_wave = 1
    else:
        tim.deinit()
        tim_counter = tim.counter()
        tim.counter(0)
        flag_wave = 2

tim =pyb.Timer(2, prescaler=240, period=65535)  #相当于freq=1M
extint = pyb.ExtInt(wave_echo_pin, pyb.ExtInt.IRQ_RISING_FALLING, pyb.Pin.PULL_DOWN, wave_handler)


"""
姿态识别解算控制
"""

def judge_stop():
    pyb.LED(1).on()
    send_direction_packet(S,0)

def judge_end():
    global judge_flag
    judge_flag = 0
    send_direction_packet(E,0)
    pyb.LED(2).on()
    saveVideo(2)
    #uart.write("\r\n###SUCCESS###")

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
                img.draw_string(2, 64, "R",color=128,scale=2,mono_space=False)
                send_direction_packet(R,x_speed)
            else:
                img.draw_string(2, 64, "L",color=128,scale=2,mono_space=False)
                send_direction_packet(L,x_speed)
            if y_cal > 0:
                img.draw_string(18, 64, "B",color=128,scale=2,mono_space=False)
                send_direction_packet(B,y_speed)
            else:
                img.draw_string(18, 64, "G",color=128,scale=2,mono_space=False)
                send_direction_packet(G,y_speed)
        else:
            img.draw_string(2, 64, "E",color=128,scale=2,mono_space=False)
            judge_end()

def judge_direction_line(line, speed=5):
    pyb.LED(1).off()
    x_cal=(line.x1()+line.x2())/2-WINDOW_CENTER_X
    theta=line.theta()-90
    t_speed=(90-abs(theta))*80
    if judge_flag == 1:
        #if temp_threshold > 140:
            #judge_end()
        #else:
        send_direction_packet(G,speed)
        if t_speed > 4:
            if theta < 0:
                img.draw_string(18, 64, "F",color=128,scale=2,mono_space=False)
                send_direction_packet(F,t_speed)
            else:
                img.draw_string(18, 64, "C",color=128,scale=2,mono_space=False)
                send_direction_packet(C,t_speed)
        else:
            send_direction_packet(F,0)
        if abs(x_cal) > 10:
            if x_cal > 0:
                img.draw_string(2, 64, "R",color=128,scale=2,mono_space=False)
                send_direction_packet(R,speed)
            else:
                img.draw_string(2, 64, "L",color=128,scale=2,mono_space=False)
                send_direction_packet(L,speed)

def DistanceToCenter(x,y):
    x_cal=WINDOW_CENTER_X-x
    y_cal=WINDOW_CENTER_Y-y
    return math.sqrt( math.pow(x_cal,2) + math.pow(y_cal,2) )

def CompareBlob(blob_1,blob_2):
    #大小 (最小)
    #tmp = DistanceToCenter(blob_1.cx(),blob_1.cy()) - DistanceToCenter(blob_2.cx(),blob_2.cy())
    #面积 (最大)
    #tmp = blob_2.pixels() - blob_1.pixels()
    #圆度 (最大)
    tmp = blob_2.roundness() - blob_1.roundness()
    if tmp > 0:
        return blob_2
    else:
        return blob_1

def LineCenter(line):
    x_cal=(line.x1()+line.x2())/2
    y_cal=(line.y1()+line.y2())/2
    return (x_cal,y_cal)

def LineAngle(line_1,line_2):
    return abs(line_1.theta() - line_2.theta())

def LineVertical(lines):
    r=[]
    if len(lines) >= 2:
        for i in range(len(lines)):
            for j in range(len(lines)-1):
                if LineAngle(lines[i],lines[j]) >= 80:
                    r.append((lines[i],lines[j]))
    return r

def LineCrossCenter(line_1,line_2):
    a1 = line_1.y2() - line_1.y1()
    b1 = line_1.x1() - line_1.x2()
    c1 = line_1.x2() * line_1.y1() - line_1.x1() * line_1.y2()
    a2 = line_2.y2() - line_2.y1()
    b2 = line_2.x1() - line_2.x2()
    c2 = line_2.x2() * line_2.y1() - line_2.x1() * line_2.y2()
    m = a1 * b2 - a2 * b1
    if m != 0:
        cross_x = int((b1 * c2 - b2 * c1)/(a1 * b2 - a2 * b1))
        cross_y = int((c1 * a2 - c2 * a1)/(a1 * b2 - a2 * b1))
        return (cross_x, cross_y)
    else:
        return (-1,-1)

def LineDistanceToCenter(line):
    (x,y)=LineCenter(line)
    return DistanceToCenter(x,y)

def DistanceToLine(line):
    a = line.y2() - line.y1()
    b = line.x1() - line.x2()
    c = line.x2() * line.y1() - line.x1() * line.y2()
    (cx,cy) = LineCenter(line)
    dis = abs(a * WINDOW_CENTER_X + b * WINDOW_CENTER_Y + c) / math.sqrt( math.pow(a,2) + math.pow(b,2) );
    return dis

def VerticleAngle(line):
    return 90-abs(line.theta()-90)

def HorizonAngle(line):
    return abs(line.theta()-90)

def CompareLine(line_1,line_2):
    #大小 (最小)
    #tmp = LineDistanceToCenter(line_1) - LineDistanceToCenter(line_2)
    #tmp = DistanceToLine(line_1) - DistanceToLine(line_2)
    #竖直 (最小)
    tmp = VerticleAngle(line_1) - VerticleAngle(line_2)
    #水平 (最小)
    #tmp = HorizonAngle(line_1) - HorizonAngle(line_2)
    if tmp > 0:
        return line_2
    else:
        return line_1

"""
感光度校准
"""

def ISO_Tune(amount):
    gain = sensor.get_gain_db()
    #感光度自动调节 范围11-16左右
    if amount == 0 and gain > 11:
        sensor.set_auto_gain(False,gain_db=sensor.get_gain_db()*0.9) # 画面明亮时调低gain
    elif amount >= 3 and gain < 16:
        sensor.set_auto_gain(False,gain_db=sensor.get_gain_db()*1.3) # 画面过暗时调高gain,稳定性差



def Res_init():
    global WINDOW_CENTER_X
    global WINDOW_CENTER_Y
    global RES
    if RES == 1:
        # lenscorr
        WINDOW_CENTER_X=264
        WINDOW_CENTER_Y=240
        sensor.reset()
        sensor.set_pixformat(sensor.GRAYSCALE)
        # VGA640*480 分割400x300进Buffer 镜头矫正1.5
        sensor.set_framesize(sensor.VGA)
        sensor.set_windowing((WINDOW_CENTER_X*2,WINDOW_CENTER_Y*2))
        sensor.skip_frames(time = 2000)
        #sensor.set_auto_gain(False,gain_db=20) # 画面明亮时调低gain
        #sensor.set_auto_whitebal(True,(-5.5, -6.5, -3)) # must be turned off for color tracking
        #sensor.set_auto_exposure(True, exposure_us = 20000)
        RES = 0
    if RES == 2:
        # find_qrcodes
        WINDOW_CENTER_X=240
        WINDOW_CENTER_Y=240
        sensor.reset()
        sensor.set_pixformat(sensor.GRAYSCALE)
        # VGA640*480 分割400x300进Buffer 镜头矫正1.5
        sensor.set_framesize(sensor.VGA)
        sensor.set_windowing((WINDOW_CENTER_X*2,WINDOW_CENTER_Y*2))
        sensor.skip_frames(time = 2000)
        #sensor.set_auto_gain(False,gain_db=20) # 画面明亮时调低gain
        #sensor.set_auto_whitebal(True,(-5.5, -6.5, -3)) # must be turned off for color tracking
        #sensor.set_auto_exposure(True, exposure_us = 20000)
        RES = 0
    elif RES == 3:
        WINDOW_CENTER_X=160
        WINDOW_CENTER_Y=120
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        # VGA640*480 分割400x300进Buffer 镜头矫正1.5
        sensor.set_framesize(sensor.QVGA)
        sensor.set_windowing((WINDOW_CENTER_X*2,WINDOW_CENTER_Y*2))
        sensor.skip_frames(time = 2000)
        #sensor.set_auto_gain(False,gain_db=20) # 画面明亮时调低gain
        sensor.set_auto_whitebal(False) # must be turned off for color tracking
        #sensor.set_auto_exposure(True, exposure_us = 20000)
        RES = 0


"""
主循环
"""

THRESHOLD = [(255,255),(0, 9)]
state_flag = 0
save_flag = 0
judge_flag = 1
NUM = 0
DEBUG = 1

while(True):
    ##等待接受任务信号
    if DEBUG:
        if state_flag == 0:
             state_flag = 3
             saveVideo(0,frames=5000)
             RES=3
    else:
        while (state_flag == 0):
            pyb.LED(1).on()
            if uart.readchar() == H:
                state_flag = 3
                saveVideo(0,frames=5000)
                pyb.LED(1).off()
            else :
                state_flag = 0

    Res_init()
    #roi = (0, 0, WINDOW_CENTER_X * 2, WINDOW_CENTER_Y * 2)
    #img = img#.binary([(0,min(125,temp_threshold))])
    clock.tick()

    if state_flag == -1:
        pyb.delay(1000)
        send_direction_packet(G,10)
        pyb.delay(1500)
        send_direction_packet(S,0)
        pyb.delay(200)
        send_direction_packet(L,10)
        pyb.delay(1500)
        send_direction_packet(S,0)
        pyb.delay(200)
        send_direction_packet(B,10)
        pyb.delay(1500)
        send_direction_packet(S,0)
        pyb.delay(200)
        send_direction_packet(R,10)
        pyb.delay(1500)
        send_direction_packet(S,0)
        pyb.delay(200)
        judge_end()
        pyb.delay(100000)

    #elif state_flag == 1:

        #if temp_threshold > 140:
            #judge_stop()
        #else:
            #########
            ##中心区域的圆形色块
            #roi = (0,0,WINDOW_CENTER_X*2,WINDOW_CENTER_Y*2)
            ## roi = (round(WINDOW_CENTER_X/2),round(WINDOW_CENTER_Y/2),WINDOW_CENTER_X,WINDOW_CENTER_Y)
            #blobs_pre = img.erode(7).find_blobs([THRESHOLD[0]], roi=roi, pixels_threshold=100, merge=False) #.gaussian(1, threshold=True, offset=20, invert=True)
            #blobs = []
            #if len(blobs_pre):
                ##预检过滤非圆物块
                #for blob in blobs_pre:
                    #if blob.roundness():
                        #blobs.append(blob)
            #if len(blobs):
                ##物块巡航
                #near_blob = blobs[0]
                #for blob in blobs:
                    #near_blob = CompareBlob(near_blob, blob)
                    #img.draw_rectangle(blob.rect())
                    #img.draw_cross(blob.cx(), blob.cy())
                    #print(blob.roundness())
                #judge_direction_blob(near_blob)
                #img.draw_line(WINDOW_CENTER_X, WINDOW_CENTER_Y, near_blob.cx(), near_blob.cy())
            #else:
                #judge_stop()
            ##ISO_Tune(len(blobs_pre))

    elif state_flag == 2:

        img = sensor.snapshot().lens_corr(1.8)
        ########
        #巡线/寻直角
        roi = (0,0,WINDOW_CENTER_X*2,WINDOW_CENTER_Y*2)
        lines_pre = img.find_lines(x_stride=2, y_stride=2, roi=roi, threshold=1000, theta_margin=40, rho_margin=30)

        if len(lines_pre):
            near_line = lines_pre[0]
            for line in lines_pre:
                near_line = CompareLine(near_line, line)
                img.draw_line(line.line())
            judge_direction_line(near_line)


            img.draw_line(near_line.line(),color=128,thickness=4)
        else:
            judge_stop()

        lines=LineVertical(lines_pre)
        crosses=[]
        if len(lines):
            for (line_1,line_2) in lines:
                img.draw_line(line_1.line())
                img.draw_line(line_2.line())
                crosses.append(LineCrossCenter(line_1,line_2))
                for (cross_x,cross_y) in crosses:
                    img.draw_circle(cross_x,cross_y,4)
                    print(cross_x,cross_y)

        ISO_Tune(len(lines_pre))

    elif state_flag == 3:

        img = sensor.snapshot().lens_corr(1.8)
        ###############
        #寻找黄色物块
        roi = (round(WINDOW_CENTER_X/2),round(WINDOW_CENTER_Y/2),WINDOW_CENTER_X,WINDOW_CENTER_Y)
        #temp_threshold = img.get_histogram().get_threshold().value() #Otsu

        blobs = img.find_blobs([(30, 100, -15, 127, 20, 57)], roi=roi, area_threshold=600, merge=True)
        if len(blobs):
            for blob in blobs:
                img.draw_rectangle(blob.rect())
            ##############
            #进入下一个模式
            state_flag = 4
            RES=2

    elif state_flag == 4:

        img = sensor.snapshot().lens_corr(1.55)
        ###############
        #寻找条形码并拍照
        roi = (0,round(3*WINDOW_CENTER_Y/4),WINDOW_CENTER_X*2,round(WINDOW_CENTER_Y/2))

        if NUM < 3:
            codes = img.find_barcodes(roi=roi)
            if len(codes):
                for code in codes:
                    img.draw_rectangle(code.rect())
                    print(code)

                sensor.snapshot().lens_corr(1.55).save("BAR_%s.jpg"%NUM)
                NUM+=1

        else:
            state_flag = 6
            NUM=0

        ISO_Tune(len(codes))
    elif state_flag == 5:

        img = sensor.snapshot().lens_corr(1.55)
        ###############
        #寻找QR码并拍照
        roi = (0,0,WINDOW_CENTER_X*2,WINDOW_CENTER_Y*2)

        if NUM < 3:
            codes = img.find_qrcodes(roi=roi)
            if len(codes):
                for code in codes:
                    img.draw_rectangle(code.rect())
                    print(code)

                sensor.snapshot().lens_corr(1.55).save("QR_%s.jpg"%NUM)
                NUM+=1

        else:
            state_flag = 6
            NUM=0

        ISO_Tune(len(codes))


    img.draw_rectangle(roi)
    img.draw_cross(WINDOW_CENTER_X, WINDOW_CENTER_Y)
    img.draw_string(2, 0, "GAIN: %s"%sensor.get_gain_db(),color=128,scale=2,mono_space=False)
    img.draw_string(2, 16, "FPS: %s"%clock.fps(),color=128,scale=2,mono_space=False)
    img.draw_string(2, 32, "STATE_FLAG: %s"%state_flag,color=128,scale=2,mono_space=False)
    #img.draw_string(2, 48, "THRESHOLD: %s"%temp_threshold,color=128,scale=2,mono_space=False)
    img.draw_string(2, 80, "DISTANCE: %s"%wave_distance,color=128,scale=2,mono_space=False)

    saveVideo()
    wave_distance_process()
    print()
