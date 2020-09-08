import serial
import struct
#import numpy 
#import string
#import binascii
from time import sleep

global hand_id
hand_id = 1

#把数据分成高字节和低字节
def data2bytes(data):
    rdata = [0xff]*2
    if data == -1:
        rdata[0] = 0xff
        rdata[1] = 0xff
    else:
        rdata[0] = data&0xff
        rdata[1] = (data>>8)&(0xff)
    return rdata

#把十六进制或十进制的数转成bytes
def num2str(num):
    str = hex(num)
    str = str[2:4]
    if(len(str) == 1):
        str = '0'+ str
    str = bytes.fromhex(str)     
    #print(str)
    return str

#求校验和
def checknum(data,leng):
    result = 0
    for i in range(2,leng):
        result += data[i]
    result = result&0xff
    #print(result)
    return result

#设置驱动器位置
def setpos(pos1,pos2,pos3,pos4,pos5,pos6):
    global hand_id
    if pos1 <-1 or pos1 >2000:
        print('数据超出正确范围：-1-2000')
        return
    if pos2 <-1 or pos2 >2000:
        print('数据超出正确范围：-1-2000')
        return
    if pos3 <-1 or pos3 >2000:
        print('数据超出正确范围：-1-2000')
        return
    if pos4 <-1 or pos4 >2000:
        print('数据超出正确范围：-1-2000')
        return
    if pos5 <-1 or pos5 >2000:
        print('数据超出正确范围：-1-2000')
        return
    if pos6 <-1 or pos6 >2000:
        print('数据超出正确范围：-1-2000')
        return
    
    datanum = 0x0F
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #写操作
    b[4] = 0x12
    
    #地址
    b[5] = 0xC2
    b[6] = 0x05
     
    #数据
    b[7] = data2bytes(pos1)[0]
    b[8] = data2bytes(pos1)[1]
    
    b[9] = data2bytes(pos2)[0]
    b[10] = data2bytes(pos2)[1]
    
    b[11] = data2bytes(pos3)[0]
    b[12] = data2bytes(pos3)[1]
    
    b[13] = data2bytes(pos4)[0]
    b[14] = data2bytes(pos4)[1]
    
    b[15] = data2bytes(pos5)[0]
    b[16] = data2bytes(pos5)[1]
    
    b[17] = data2bytes(pos6)[0]
    b[18] = data2bytes(pos6)[1]
    
    #校验和
    b[19] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    #print('发送的数据：',putdata)
    
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
        
    getdata= ser.read(9)
    #print('返回的数据：',getdata)
    print('返回的数据：')
    for i in range(1,10):
        print(hex(getdata[i-1]))

#设置角度
def setangle(angle1,angle2,angle3,angle4,angle5,angle6):
    global hand_id
    if angle1 <-1 or angle1 >1000:
        print('数据超出正确范围：-1-1000')
        return
    if angle2 <-1 or angle2 >1000:
        print('数据超出正确范围：-1-1000')
        return
    if angle3 <-1 or angle3 >1000:
        print('数据超出正确范围：-1-1000')
        return
    if angle4 <-1 or angle4 >1000:
        print('数据超出正确范围：-1-1000')
        return
    if angle5 <-1 or angle5 >1000:
        print('数据超出正确范围：-1-1000')
        return
    if angle6 <-1 or angle6 >1000:
        print('数据超出正确范围：-1-1000')
        return
    
    datanum = 0x0F
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #写操作
    b[4] = 0x12
    
    #地址
    b[5] = 0xCE
    b[6] = 0x05
     
    #数据
    b[7] = data2bytes(angle1)[0]
    b[8] = data2bytes(angle1)[1]
    
    b[9] = data2bytes(angle2)[0]
    b[10] = data2bytes(angle2)[1]
    
    b[11] = data2bytes(angle3)[0]
    b[12] = data2bytes(angle3)[1]
    
    b[13] = data2bytes(angle4)[0]
    b[14] = data2bytes(angle4)[1]
    
    b[15] = data2bytes(angle5)[0]
    b[16] = data2bytes(angle5)[1]
    
    b[17] = data2bytes(angle6)[0]
    b[18] = data2bytes(angle6)[1]
    
    #校验和
    b[19] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(9)
    print('返回的数据：')
    for i in range(1,10):
        print(hex(getdata[i-1]))


#设置力控阈值
def setpower(power1,power2,power3,power4,power5,power6):
    global hand_id
    if power1 <0 or power1 >1000:
        print('数据超出正确范围：0-1000')
        return
    if power2 <0 or power2 >1000:
        print('数据超出正确范围：0-1000')
        return
    if power3 <0 or power3 >1000:
        print('数据超出正确范围：0-1000')
        return
    if power4 <0 or power4 >1000:
        print('数据超出正确范围：0-1000')
        return
    if power5 <0 or power5 >1000:
        print('数据超出正确范围：0-1000')
        return
    if power6 <0 or power6 >1000:
        print('数据超出正确范围：0-1000')
        return
    
    datanum = 0x0F
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #写操作
    b[4] = 0x12
    
    #地址
    b[5] = 0xDA
    b[6] = 0x05
     
    #数据
    b[7] = data2bytes(power1)[0]
    b[8] = data2bytes(power1)[1]
    
    b[9] = data2bytes(power2)[0]
    b[10] = data2bytes(power2)[1]
    
    b[11] = data2bytes(power3)[0]
    b[12] = data2bytes(power3)[1]
    
    b[13] = data2bytes(power4)[0]
    b[14] = data2bytes(power4)[1]
    
    b[15] = data2bytes(power5)[0]
    b[16] = data2bytes(power5)[1]
    
    b[17] = data2bytes(power6)[0]
    b[18] = data2bytes(power6)[1]
    
    #校验和
    b[19] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    
    getdata= ser.read(9)
    print('返回的数据：')
    for i in range(1,10):
        print(hex(getdata[i-1]))


#设置速度
def setspeed(speed1,speed2,speed3,speed4,speed5,speed6):
    global hand_id
    if speed1 <0 or speed1 >1000:
        print('数据超出正确范围：0-1000')
        return
    if speed2 <0 or speed2 >1000:
        print('数据超出正确范围：0-1000')
        return
    if speed3 <0 or speed3 >1000:
        print('数据超出正确范围：0-1000')
        return
    if speed4 <0 or speed4 >1000:
        print('数据超出正确范围：0-1000')
        return
    if speed5 <0 or speed5 >1000:
        print('数据超出正确范围：0-1000')
        return
    if speed6 <0 or speed6 >1000:
        print('数据超出正确范围：0-1000')
        return
    
    datanum = 0x0F
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #写操作
    b[4] = 0x12
    
    #地址
    b[5] = 0xF2
    b[6] = 0x05
     
    #数据
    b[7] = data2bytes(speed1)[0]
    b[8] = data2bytes(speed1)[1]
    
    b[9] = data2bytes(speed2)[0]
    b[10] = data2bytes(speed2)[1]
    
    b[11] = data2bytes(speed3)[0]
    b[12] = data2bytes(speed3)[1]
    
    b[13] = data2bytes(speed4)[0]
    b[14] = data2bytes(speed4)[1]
    
    b[15] = data2bytes(speed5)[0]
    b[16] = data2bytes(speed5)[1]
    
    b[17] = data2bytes(speed6)[0]
    b[18] = data2bytes(speed6)[1]
    
    #校验和
    b[19] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
        
    getdata= ser.read(9)
    print('返回的数据：')
    for i in range(1,10):
        print(hex(getdata[i-1]))

#读取驱动器实际的位置值
def get_setpos():
    global hand_id
    
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #读操作
    b[4] = 0x11
    
    #地址
    b[5] = 0xC2
    b[6] = 0x05
     
    #读取寄存器的长度
    b[7] = 0x0C
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    #print('发送的数据：',putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
        
    getdata= ser.read(20)
    print('返回的数据：')
    for i in range(1,21):
        print(hex(getdata[i-1]))
    
    setpos = [0]*6
    for i in range(1,7):
        if getdata[i*2+5]== 0xff and getdata[i*2+6]== 0xff:
            setpos[i-1] = -1
        else:
            setpos[i-1] = getdata[i*2+5] + (getdata[i*2+6]<<8)
    return setpos

#读取设置角度
def get_setangle():
    global hand_id
    
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #读操作
    b[4] = 0x11
    
    #地址
    b[5] = 0xCE
    b[6] = 0x05
     
    #读取寄存器的长度
    b[7] = 0x0C
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(20)
    print('返回的数据：')
    for i in range(1,21):
        print(hex(getdata[i-1]))
    
    
    setangle = [0]*6
    for i in range(1,7):
        if getdata[i*2+5]== 0xff and getdata[i*2+6]== 0xff:
            setangle[i-1] = -1
        else:
            setangle[i-1] = getdata[i*2+5] + (getdata[i*2+6]<<8)
    return setangle
  

#读取驱动器设置的力控阈值
def get_setpower():
    global hand_id
    
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #读操作
    b[4] = 0x11
    
    #地址
    b[5] = 0xDA
    b[6] = 0x05
     
    #读取寄存器的长度
    b[7] = 0x0C
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(20)
    print('返回的数据：')
    for i in range(1,21):
        print(hex(getdata[i-1]))
    
    setpower = [0]*6
    for i in range(1,7):
        if getdata[i*2+5]== 0xff and getdata[i*2+6]== 0xff:
            setpower[i-1] = -1
        else:
            setpower[i-1] = getdata[i*2+5] + (getdata[i*2+6]<<8)
    return setpower

#读取驱动器实际的位置值
def get_actpos():
    global hand_id
    
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #读操作
    b[4] = 0x11
    
    #地址
    b[5] = 0xFE
    b[6] = 0x05
     
    #读取寄存器的长度
    b[7] = 0x0C
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(20)
    print('返回的数据：')
    for i in range(1,21):
        print(hex(getdata[i-1]))
    
    actpos = [0]*6
    for i in range(1,7):
        if getdata[i*2+5]== 0xff and getdata[i*2+6]== 0xff:
            actpos[i-1] = -1
        else:
            actpos[i-1] = getdata[i*2+5] + (getdata[i*2+6]<<8)
    return actpos

 #读取实际的角度值
def get_actangle():
    global hand_id
    
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #读操作
    b[4] = 0x11
    
    #地址
    b[5] = 0x0A
    b[6] = 0x06
     
    #读取寄存器的长度
    b[7] = 0x0C
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(20)
    print('返回的数据：')
    for i in range(1,21):
        print(hex(getdata[i-1]))
    
    actangle = [0]*6
    for i in range(1,7):
        if getdata[i*2+5]== 0xff and getdata[i*2+6]== 0xff:
            actangle[i-1] = -1
        else:
            actangle[i-1] = getdata[i*2+5] + (getdata[i*2+6]<<8)
    return actangle

 #读取实际的受力
def get_actforce():
    global hand_id
    
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #读操作
    b[4] = 0x11
    
    #地址
    b[5] = 0x2E
    b[6] = 0x06
     
    #读取寄存器的长度
    b[7] = 0x0C
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(20)
    print('返回的数据：')
    for i in range(1,21):
        print(hex(getdata[i-1]))
    
    actforce = [0]*6
    for i in range(1,7):
        if getdata[i*2+5]== 0xff and getdata[i*2+6]== 0xff:
            actforce[i-1] = -1
        else:
            actforce[i-1] = getdata[i*2+5] + (getdata[i*2+6]<<8)
    return actforce

#读取电流
def get_current():
    global hand_id
    
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #读操作
    b[4] = 0x11
    
    #地址
    b[5] = 0x3A
    b[6] = 0x06
     
    #读取寄存器的长度
    b[7] = 0x0C
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(20)
    print('返回的数据：')
    for i in range(1,21):
        print(hex(getdata[i-1]))
    
    current = [0]*6
    for i in range(1,7):
        if getdata[i*2+5]== 0xff and getdata[i*2+6]== 0xff:
            current[i-1] = -1
        else:
            current[i-1] = getdata[i*2+5] + (getdata[i*2+6]<<8)
    return current

#读取故障信息
def get_error():
    global hand_id
    
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #读操作
    b[4] = 0x11
    
    #地址
    b[5] = 0x46
    b[6] = 0x06
     
    #读取寄存器的长度
    b[7] = 0x06
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(14)
    print('返回的数据：')
    for i in range(1,15):
        print(hex(getdata[i-1]))
    
    error = [0]*6
    for i in range(1,7):
        error[i-1] = getdata[i+6]
    return error

 #读取状态信息
def get_status():
    global hand_id
    
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #读操作
    b[4] = 0x11
    
    #地址
    b[5] = 0x4C
    b[6] = 0x06
     
    #读取寄存器的长度
    b[7] = 0x06
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
        ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(14)
    print('返回的数据：')
    for i in range(1,15):
        print(hex(getdata[i-1]))
    
    status = [0]*6
    for i in range(1,7):
        status[i-1] = getdata[i+6]
    return status
    

#读取温度信息
def get_temp():
    global hand_id
    
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #读操作
    b[4] = 0x11
    
    #地址
    b[5] = 0x52
    b[6] = 0x06
     
    #读取寄存器的长度
    b[7] = 0x06
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(14)
    print('返回的数据：')
    for i in range(1,15):
        print(hex(getdata[i-1]))
    
    temp = [0]*6
    for i in range(1,7):
        temp[i-1] = getdata[i+6]
    return temp


#清除错误
def set_clear_error():
    global hand_id
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #写操作
    b[4] = 0x12
    
    #地址
    b[5] = 0xEC
    b[6] = 0x03
     
    #数据
    b[7] = 0x01
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(9)
    print('返回的数据：')
    for i in range(1,10):
        print(hex(getdata[i-1]))


#保存参数到FLASH
def set_save_flash():
    global hand_id
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #写操作
    b[4] = 0x12
    
    #地址
    b[5] = 0xED
    b[6] = 0x03
     
    #数据
    b[7] = 0x01
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(18)
    print('返回的数据：')
    for i in range(1,19):
        print(hex(getdata[i-1]))

#力传感器校准
def gesture_force_clb():
    global hand_id
    datanum = 0x04
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #写操作
    b[4] = 0x12
    
    #地址
    b[5] = 0xF1
    b[6] = 0x03
     
    #数据
    b[7] = 0x01
    
    #校验和
    b[8] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
    
    getdata= ser.read(18)
    print('返回的数据：')
    for i in range(1,19):
        print(hex(getdata[i-1]))
#设置上电速度
def setdefaultspeed(speed1,speed2,speed3,speed4,speed5,speed6):
    global hand_id
    if speed1 <0 or speed1 >1000:
        print('数据超出正确范围：0-1000')
        return
    if speed2 <0 or speed2 >1000:
        return
    if speed3 <0 or speed3 >1000:
        return
    if speed4 <0 or speed4 >1000:
        return
    if speed5 <0 or speed5 >1000:
        return
    if speed6 <0 or speed6 >1000:
        return
    
    datanum = 0x0F
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #写操作
    b[4] = 0x12
    
    #地址
    b[5] = 0x08
    b[6] = 0x04
     
    #数据
    b[7] = data2bytes(speed1)[0]
    b[8] = data2bytes(speed1)[1]
    
    b[9] = data2bytes(speed2)[0]
    b[10] = data2bytes(speed2)[1]
    
    b[11] = data2bytes(speed3)[0]
    b[12] = data2bytes(speed3)[1]
    
    b[13] = data2bytes(speed4)[0]
    b[14] = data2bytes(speed4)[1]
    
    b[15] = data2bytes(speed5)[0]
    b[16] = data2bytes(speed5)[1]
    
    b[17] = data2bytes(speed6)[0]
    b[18] = data2bytes(speed6)[1]
    
    #校验和
    b[19] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
        
    getdata= ser.read(9)
    print('返回的数据：')
    for i in range(1,10):
        print(hex(getdata[i-1]))
    
#设置上电力控阈值
def setdefaultpower(power1,power2,power3,power4,power5,power6):
    global hand_id
    if power1 <0 or power1 >1000:
        print('数据超出正确范围：0-1000')
        return
    if power2 <0 or power2 >1000:
        return
    if power3 <0 or power3 >1000:
        return
    if power4 <0 or power4 >1000:
        return
    if power5 <0 or power5 >1000:
        return
    if power6 <0 or power6 >1000:
        return
    
    datanum = 0x0F
    b = [0]*(datanum + 5)
    #包头
    b[0] = 0xEB
    b[1] = 0x90

    #hand_id号
    b[2] = hand_id

    #数据个数
    b[3] = datanum
    
    #写操作
    b[4] = 0x12
    
    #地址
    b[5] = 0x14
    b[6] = 0x04
     
    #数据
    b[7] = data2bytes(power1)[0]
    b[8] = data2bytes(power1)[1]
    
    b[9] = data2bytes(power2)[0]
    b[10] = data2bytes(power2)[1]
    
    b[11] = data2bytes(power3)[0]
    b[12] = data2bytes(power3)[1]
    
    b[13] = data2bytes(power4)[0]
    b[14] = data2bytes(power4)[1]
    
    b[15] = data2bytes(power5)[0]
    b[16] = data2bytes(power5)[1]
    
    b[17] = data2bytes(power6)[0]
    b[18] = data2bytes(power6)[1]
    
    #校验和
    b[19] = checknum(b,datanum+4)
    
    #向串口发送数据
    putdata = b''
    
    for i in range(1,datanum+6):
        putdata = putdata + num2str(b[i-1])
    ser.write(putdata)
    print('发送的数据：')
    for i in range(1,datanum+6):
        print(hex(putdata[i-1]))
        
    getdata= ser.read(9)
    print('返回的数据：')
    for i in range(1,10):
        print(hex(getdata[i-1]))

#串口设置
ser=serial.Serial('/dev/ttyUSB0',115200)
ser.isOpen()

setpos(0,0,0,0,0,0)

sleep(1)

setspeed(1000,1000,1000,1000,1000,1000)

sleep(1)

setpos(1000,1000,1000,1000,1000,1000)





