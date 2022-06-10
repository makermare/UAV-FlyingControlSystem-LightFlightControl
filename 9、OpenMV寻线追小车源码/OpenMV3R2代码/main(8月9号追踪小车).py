# Position_Control And Tracking Line - By: Linlin's Space - 周三 7月 5 2017

import sensor, image, time, math,pyb
from pyb import Pin, Timer,UART

uart = pyb.UART(3,115200,timeout_char=1000)#串口初始化

# Tracks a black line. Use [(128, 255)] for a tracking a white line.
GRAYSCALE_THRESHOLD = [(0, 150)]

ROIS = [
        (0,0,64,64,0)
       ]
#----------------------------------------寻找直线变量----------------------------------------#
#---------------------------------------摄像头初始化-----------------------------------------#
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.B64X64)                  # 颜色追踪:160*120  光流检测:40*30
sensor.skip_frames(20)
sensor.set_auto_gain(False)                         # 颜色追踪关闭自动增益
sensor.set_auto_whitebal(False)                     # 颜色追踪关闭白平衡
Sum_x=0;
Sum_y=0;
flag=0;
i=0;  #记录第几行数据
j=0;  #记录直线数量
led=pyb.LED(3)#必要的时候进行红外补光

#检测圆形中心点的坐标
center_x=0;
center_y=0;
center_update=1;#中心圆位置更新的标志
center_x_old=0;
center_y_old=0;
center_pos_old=0;

center_x_down=0;
center_y_down=0;

center_x_up=0;
center_y_up=0;

center_x_mid=0;
center_y_mid=0;

center_y_left=0;
center_x_left=0;

center_y_right=0;
center_x_right=0;

center_y_rect=0;
center_x_rect=0;

center_y_rect_old=0;
center_x_rect_old=0;

center_flag1=0;#上下
center_flag2=0;#左右
center_flag3=0;#通过roll来调整黑线的位置  通过yaw来调整机头方向  矩形1和2=0;
center_flag4=0;
center_flag5=0;
center_flag6=0;
turn_flag=0;#转弯的标志
last_x=0;
last_y=0;
center_pos=0;
yaw_angle=0;
out_str1='';
clock = time.clock()

led=pyb.LED(2)#必要的时候进行红外补光
#定义一个定时发送数据的函数
def tick(timer):#we will receive the timer object when being called
        global flag
        flag=1
tim = Timer(4,freq=20)            # create a timer object using timer 4 - trigger at 1Hz
tim.callback(tick)                # set the callback to our tick function
#--------------------------------------while循环开始-----------------------------------------#

while(True):
    if(flag==1):
        led.on();
        img=sensor.snapshot()
        img_old=img.copy()
        #--------------------------------------光流定点-----------------------------------------#
        img.lens_corr(1.5)#for 2.8mm lens...摄像头畸变纠正
        #--------------------------------------检测直线交点的位置---------------------------------------#
        #img.binary(GRAYSCALE_THRESHOLD,invert=1);
        #--------------------------------------寻找圆心的位置--------------------------------------#
        #检测圆形位置
        for r in ROIS:
            blobs=img_old.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True,pixels_area=10) # r[0:4] is roi tuple.
            if blobs:#如果找到了颜色块
                largest_blob=max(blobs, key=lambda b: b.pixels())
                if(largest_blob[4]>=10 and largest_blob[4]<=1500):#像素点个数
                    if(largest_blob[3]>=3 and largest_blob[3]<=60):
                        if(largest_blob[2]>=3 and largest_blob[2]<=60):
                            center_y_rect=largest_blob.cy();
                            center_x_rect=largest_blob.cx();
                            img.draw_rectangle(largest_blob.rect())
        img.draw_cross(center_x_rect,center_y_rect,2)
        #50ms发送一次数据到飞控
        if(yaw_angle<0):
            out_str1='-'
            out_str1+= '%.2d'% int(-yaw_angle)    #寻找黑线中心位置计算出偏转角度
        else:
            out_str1='+'
            out_str1+= '%.2d'% int(yaw_angle)     #寻找黑线中心位置计算出偏转角度
        if(Sum_x<0):
            out_str1+='-'
            out_str1+='%.2d'%  int(-Sum_x);       #光流数据
        else:
            out_str1+='+'
            out_str1+='%.2d'%  int(Sum_x)         #寻找黑线中心位置计算出偏转角度
        if(Sum_y<0):
            out_str1+='-'
            out_str1+= '%.2d'% int(-Sum_y);       #光流数据
        else:
            out_str1+='+'
            out_str1+= '%.2d'% int(Sum_y);        #光流数据

        out_str1+='%.2d'%      int(center_pos);
        out_str1+='%.2d'%      int(center_x_rect); #圆心的位置
        out_str1+='%.2d'%      int(center_y_rect);
        out_str1+='%.2d'%      int(turn_flag);     #直角标志位
        out_str1+='%.2d'%      int(last_x);        #直角交点位置
        out_str1+='%.2d'%      int(last_y);
        uart.write('s'+out_str1+'#')
        #像素位移之和清零
        turn_flag=0;
        Sum_x=0
        Sum_y=0
        out_str1=''#清除之前的数据
        flag=0;
        led.off();
        #-----------------------------------串口打印数据-----------------------------------------#



