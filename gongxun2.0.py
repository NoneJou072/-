import sensor, image, time, pyb, ujson
from pyb import UART

green_threshold   = (10, 78, -53, -7, 14, 91)#绿色阈值；颜色代号red=2,green=1,blue=4
red_threshold   = (20, 100, 34, 127, -4, 127)#红色阈值
#blue_threshold   = (74, 21, -128, 127, -128, -24)#蓝色阈值
blue_threshold   = (100, 8, -128, 127, -128, -29)

getrx = 0#串口接收
collast = 0#上一颜色
coli = 0
renum = 0
ptrposition = 0
ii = 0
iii = 0
col = [0,0,0]
FH = bytearray([0xb3,0xb3])#帧头
cirset = 0
n = 0#计次
circopy = 0 #计数

sensor.reset()# 初始化摄像头
sensor.set_pixformat(sensor.RGB565)# 格式为 RGB565.
sensor.set_framesize(sensor.CIF) # 使用 QQVGA 速度快一些
#sensor.set_windowing((240, 240)) # look at center 240x240 pixels of the VGA resolution.
sensor.skip_frames(time = 2000) # 跳过2000s，使新设置生效,并自动调节白平衡
sensor.set_auto_gain(False) # 关闭自动自动增益。默认开启的，在颜色识别中，一定要关闭白平衡。
sensor.set_auto_whitebal(False)
#关闭白平衡。白平衡是默认开启的，在颜色识别中，一定要关闭白平衡。
clock = time.clock() # 追踪帧率
led = pyb.LED(1) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.
uart = UART(3, 115200, timeout_char = 1000)
uart.init(115200, bits=8, parity=None, stop=1)  #8位数据位，无校验位，1位停止位、
led.on()
#二维码加密，减少在通讯过程中的错误
def Rec_NUM1(lista):#上层二维码
    if (lista[0]=='1' and lista[1]=='2' and lista[2]=='3'):
        return 1
    elif (lista[0]=='1' and lista[1]=='3' and lista[2]=='2'):
        return 2
    elif (lista[0]=='2' and lista[1]=='1' and lista[2]=='3'):
        return 3
    elif (lista[0]=='2' and lista[1]=='3' and lista[2]=='1'):
        return 4
    elif (lista[0]=='3' and lista[1]=='1' and lista[2]=='2'):
        return 5
    elif (lista[0]=='3' and lista[1]=='2' and lista[2]=='1'):
        return 6

def Rec_NUM2(lista):#下层二维码
    if (lista[4]=='1' and lista[5]=='2' and lista[6]=='3'):
        return 1
    elif (lista[4]=='1' and lista[5]=='3' and lista[6]=='2'):
        return 2
    elif (lista[4]=='2' and lista[5]=='1' and lista[6]=='3'):
        return 3
    elif (lista[4]=='2' and lista[5]=='3' and lista[6]=='1'):
        return 4
    elif (lista[4]=='3' and lista[5]=='1' and lista[6]=='2'):
        return 5
    elif (lista[4]=='3' and lista[5]=='2' and lista[6]=='1'):
        return 6

def cirSensor():#色环识别初始化
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.HQVGA)
    sensor.skip_frames(time = 5000)
    sensor.set_auto_gain(False) # must be turned off for color tracking
    sensor.set_auto_whitebal(False) # must be turned off for color tracking
    #sensor.set_auto_exposure(False, 7000)

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    if uart.any():#如果接收到数据，小灯闪烁并变蓝
        led.off()
        getrx = uart.readline()#接收到的数据保存到getrx
        time.sleep(150)     #延时150ms
        led = pyb.LED(2)
        led.on()
    img = sensor.snapshot()# 从感光芯片获得一张图像
    #img.lens_corr(strength = 1.8, zoom = 1.0) # 镜头畸变矫正
    blobs = img.find_blobs([green_threshold,red_threshold,blue_threshold],roi = (50, 10, 300, 300),x_stride=25,y_stride=50,area_threshold=8000)
    if getrx==b'2':#如果接收到0x32，执行二维码扫描
        if ii==0:
            led.off()#小灯闪烁并变绿一次
            time.sleep(150)
            led = pyb.LED(1)
            led.on()
            ii=1
        for code in img.find_qrcodes():
            uart.write(FH)#串口发送帧头
            output_str="%s" % code.payload() #方式1
            #renum = int(Rec_NUM1(output_str)*10 + Rec_NUM2(output_str))
            codex=int(Rec_NUM1(output_str))
            codey=int(Rec_NUM2(output_str))
            data=bytearray([codex,codey])#数据转码并发送至串口
            uart.write(data)
            uart.write(data)
            uart.write(data)
            #uart.write(ujson.dumps(renum))
            print(FH,data)
            led.off()#成功扫描后，小灯闪烁并变绿
            time.sleep(100)
            led = pyb.LED(1)
            led.on()
    if getrx==b'3':#如果接收0x33且视野里有目标颜色，执行颜色识别，并将识别到的颜色代号以串口发出
        if coli!=-1:
            if blobs:
                led.off()#小灯闪烁并变蓝
                time.sleep(100)     #延时100ms
                led = pyb.LED(2)
                led.on()
            for b in blobs:
                # Draw a rect around the blob.
                img.draw_rectangle(b[0:4]) # rect#用矩形标记出目标颜色区域
                img.draw_cross(b[5], b[6]) # cx, cy#在目标颜色区域的中心画十字形标记

                #print(b[0:4])
                #uart.write(ujson.dumps(b[8]))
                if b[8]==1:#将颜色序号转为方便接收的数字
                    colnum=2#red
                elif b[8]==2:
                    colnum=1#green
                elif b[8]==4:
                    colnum=3#blue
                #判断是否和识别到的上一颜色相同
                if coli==0 and collast==0:#如果是第一次识别到，保存颜色
                    collast = colnum
                    col[coli]=colnum
                    coli+=1
                elif coli==1 and collast != colnum: #如果颜色发生变化，保存颜色
                    collast = colnum
                    col[coli]=colnum
                    coli+=1
                elif coli==2 and collast != colnum: #如果颜色发生变化，保存颜色
                    collast = colnum
                    col[coli]=colnum
                    coli=-1
        else:#识别颜色顺序完毕，向串口发送
            #coli=-2#是否只发送一次
            #getrx=-1是否只识别一次
            colbyte=bytearray([col[0],col[1],col[2],col[0],col[1],col[2]])
            uart.write(FH)
            uart.write(colbyte)
            print(col)
            #print(col)
    if getrx==b'4':#如果接收到0x34,执行色环扫描
        led = pyb.LED(1)
        led.on()
        if cirset==0:#色环识别初始化
            cirset = 1
            cirSensor()
            img = sensor.snapshot()# 重新从感光芯片获得一张图像
        #L亮度；a的正数代表红色，负端代表绿色；b的正数代表黄色，负端代表兰色。
        #area为识别到的三个区域，用来判定三种颜色
        area1 = (40, 80, 30, 30)
        area2 = (120, 80, 30, 30)
        area3 = (200, 80, 30, 30)
        #像素颜色统计
        statistics1 = img.get_statistics(roi = area1)
        statistics2 = img.get_statistics(roi = area2)
        statistics3 = img.get_statistics(roi = area3)
        #计算颜色,放大lab差值
        #l,a,b 的最大值与最大灰度值的比例，放大十倍后与最小灰度的差值
        labmax1 = ((statistics1.l_max() + statistics1.a_max() + statistics1.b_max()) / statistics1.max())*10 - statistics1.min()
        labmax2 = ((statistics2.l_max() + statistics2.a_max() + statistics2.b_max()) / statistics2.max())*10 - statistics2.min()
        labmax3 = ((statistics3.l_max() + statistics3.a_max() + statistics3.b_max()) / statistics3.max())*10 - statistics3.min()
        #l,a,b加权平均和,用来区分红环
        l_p = -1#比例系数
        a_p = 3
        b_p = 2
        ud = 50#补偿值
        statisticMeam1 = statistics1.l_min()*l_p + statistics1.a_mean()*a_p + statistics1.b_min()*b_p + ud
        statisticMeam2 = statistics2.l_min()*l_p + statistics2.a_mean()*a_p + statistics2.b_min()*b_p + ud
        statisticMeam3 = statistics3.l_min()*l_p + statistics3.a_mean()*a_p + statistics3.b_min()*b_p + ud
        #放大差值，用来区分绿环
        statisticTol1 = int(labmax1 - statisticMeam1)
        statisticTol2 = int(labmax2 - statisticMeam2)
        statisticTol3 = int(labmax3 - statisticMeam3)
        #print('l={0},a={1},b={2}'.format(labmax1,labmax2,labmax3))#
        #print('left={0},mid={1},right={2}'.format(statisticMeam1,statisticMeam2,statisticMeam3))#
        #print('l={0},m={1},r={2}'.format(statisticTol1,statisticTol2,statisticTol3))#
        statisticMeamMin = max(statisticMeam1, statisticMeam2, statisticMeam3)
        statisticTolMin = max(statisticTol1, statisticTol2, statisticTol3)
        cirlist = [-1, -1, -1]#储存各位置颜色
        #绿1  红2  蓝3
        if statisticMeamMin == statisticMeam1:
            cirlist[0] = 2
        elif statisticMeamMin == statisticMeam2:
            cirlist[1] = 2
        elif statisticMeamMin == statisticMeam3:
            cirlist[2] = 2
        if statisticTolMin == statisticTol1:
            cirlist[0] = 3
        elif statisticTolMin == statisticTol2:
            cirlist[1] = 3
        elif statisticTolMin == statisticTol3:
            cirlist[2] = 3
        if cirlist[0]<=0:
            cirlist[0] = 1
        elif cirlist[1]<=0:
            cirlist[1] = 1
        elif cirlist[2]<=0:
            cirlist[2] = 1
        #在统计区域画圈,便于观测
        img.draw_circle(40, 80, 30, color = (255, 0, 0))
        img.draw_circle(120, 80, 30, color = (255, 0, 0))
        img.draw_circle(200, 80, 30, color = (255, 0, 0))
        if cirlist[0]>0 and cirlist[1]>0 and cirlist[2]>0 and cirlist[0]!=cirlist[1]!=cirlist[2] and n>=10:
            circolbyte=bytearray([cirlist[0],cirlist[1],cirlist[2],cirlist[0],cirlist[1],cirlist[2]])
            uart.write(FH)
            uart.write(circolbyte)
            print(cirlist)
        pyb.delay(200)
        n += 1
        if (circopy != cirlist[0]) or n>=11:
            n = 0
        circopy = cirlist[0]
        #print(n,circopy,cirlist[0])
