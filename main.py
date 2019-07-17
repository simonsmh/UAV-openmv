# main.py
import pyb,sensor, image, time, math, struct, network, usocket, sys, mjpeg, gif
from pyb import UART, LED

"""
单板初始化
"""

WINDOW_CENTER_X = 240
WINDOW_CENTER_Y = 240

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
# VGA640*480 分割400x300进Buffer 镜头矫正1.5
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((WINDOW_CENTER_X*2,WINDOW_CENTER_Y*2))
sensor.set_auto_gain(False,gain_db=15) # 画面明亮时调低gain
#sensor.set_auto_whitebal(True,(-5.5, -6.5, -3)) # must be turned off for color tracking
sensor.set_auto_exposure(True, exposure_us = 20000)
sensor.skip_frames(time = 2000)
#
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

# USART 初始化
uart = UART(3, 500000, timeout_char=1000)

# USART 串口协议
def send_direction_packet(direct,velcity):
    s = 0xAA+0x8C+direct+(int(velcity/256))+(int(velcity%256))
    s = int(s % 256)
    temp_flow = struct.pack("<BBBBhB",
                   0xAA,
                   0x89,
                   03,
                   direct,
                   velcity,
                   s)
    uart.write(temp_flow)

def send_yaw_packet(direct,velcity):
    s = 0xAA+0x9B+direct+(int(velcity/256))+(int(velcity%256))
    s = int(s % 256)
    temp_flow = struct.pack("<BBBBhB",
                   0xAA,
                   0x98,
                   03,
                   direct,
                   velcity,
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
            LED(3).on()
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
            LED(3).off()
            if format == 'mjpeg':
                m.close(clock.fps())
            else:
                g.close()
            save_flag = 0
            save_start = 0

"""
姿态识别解算控制
"""

def judge_stop():
    send_direction_packet(S,0)

def judge_end():
    global judge_flag
    judge_flag = 0
    send_direction_packet(E,0)
    LED(2).on()
    saveVideo(2)
    #uart.write("\r\n###SUCCESS###")

def judge_direction(blob,speed=5):
    x_cal=WINDOW_CENTER_X-blob.cx()
    y_cal=WINDOW_CENTER_Y-blob.cy()
    if judge_flag == 1:
        if abs(x_cal) > 5 or abs(y_cal) > 5:
            if x_cal < 0:
                send_direction_packet(R,speed)
            else:
                send_direction_packet(L,speed)
            if y_cal < 0:
                send_direction_packet(B,speed)
            else:
                send_direction_packet(G,speed)
        else:
            judge_end()

def distanceToCenter(blob):
    x_cal=WINDOW_CENTER_X-blob.cx()
    y_cal=WINDOW_CENTER_Y-blob.cy()
    return math.sqrt( math.pow(x_cal,2) + math.pow(y_cal,2) )

def compareBlob(blob_1,blob_2):
    #大小 (最小)
    #tmp = distanceToCenter(blob_1) - distanceToCenter(blob_2)
    #面积 (最大)
    #tmp = blob_2.pixels() - blob_1.pixels()
    #圆度 (最大)
    tmp = blob_2.roundness() - blob_1.roundness()
    if tmp > 0:
        return blob_2;
    else:
        return blob_1;


"""
感光度校准
"""

#ISO=False
#def ISO_Tune():
    #global ISO


"""
主循环
"""

THRESHOLD = [(255,255),(0, 40)]
state_flag = 0
save_flag = 0
judge_flag = 1

while(True):
    ##等待接受任务信号
    #while (state_flag == 0):
        #if uart.readchar() == H:
            #state_flag = 1
            #saveVideo(0,frames=5000)
        #else :
            #state_flag = 0
    #调试
    if (state_flag == 0):
        state_flag = 1
        saveVideo(0,frames=5000)

    clock.tick()
    img = sensor.snapshot().lens_corr(1.4)#.binary([THRESHOLD[1]])
    blobs_pre = img.find_blobs([THRESHOLD[1]], pixels_threshold=300, margin=5, merge=True)
    blobs = []
    if len(blobs_pre):
        #预检过滤非圆物块
        for blob in blobs_pre:
            if blob.roundness() > 0.75:
                blobs.append(blob)
    if len(blobs):
        #物块巡航
        near_blob = blobs[0]
        for blob in blobs:
            near_blob = compareBlob(near_blob, blob)
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            print(blob.roundness())
        judge_direction(near_blob)
        img.draw_line(WINDOW_CENTER_X, WINDOW_CENTER_Y, near_blob.cx(), near_blob.cy())
    else:
        judge_stop()

    #感光度自动调节 范围11-16左右
    if (len(blobs_pre) == 0 and sensor.get_gain_db() > 11):
        sensor.set_auto_gain(False,gain_db=sensor.get_gain_db()*0.9) # 画面明亮时调低gain
    if (len(blobs_pre) >= 3 and sensor.get_gain_db() < 16):
        sensor.set_auto_gain(False,gain_db=sensor.get_gain_db()*1.3) # 画面过暗时调高gain,稳定性差

    img.draw_cross(WINDOW_CENTER_X, WINDOW_CENTER_Y)
    img.draw_string(0, 0, "GAIN: %s"%sensor.get_gain_db(),scale=2,mono_space=False)
    img.draw_string(0, 16, "FPS: %s"%clock.fps(),scale=2,mono_space=False)

    saveVideo()
