1、开发环境：树莓派4b + Thonny Python IDE 

2、控制接口：
     (1)setpos(pos1,pos2,pos3,pos4,pos5,pos6) 
      设置六个驱动器位置------参数pos范围-1-2000 

      (2)setangle(angle1,angle2,angle3,angle4,angle5,angle6) 
      设置灵巧手角度------参数angle范围-1-1000

      (3)setpower(power1,power2,power3,power4,power5,power6) 
      设置力控阈值------参数power范围0-1000

      (4)setspeed(speed1,speed2,speed3,speed4,speed5,speed6) 
      设置速度------参数speed范围0-1000

      (5)get_actpos()
      读取驱动器实际的位置值

      (6)get_actangle()
      读取实际的角度值

      (7)get_actforce()
      读取实际的受力

      (8)get_setpos()
      读取驱动器设置的位置值

      (9)get_setangle()
      读取设置的角度值

      (10)get_setpower()
      读取设置的力控阈值
      
      (11)get_error()
      读取故障信息

      (12)get_status()
      读取状态信息

      (13)get_temp()
      读取温度信息

      (14)get_current()
      读取电流

      (15)set_clear_error()
      清除错误
     
      (16)setdefaultspeed(speed1,speed2,speed3,speed4,speed5,speed6)
      设置上电速度------参数speed范围0-1000

      (17)setdefaultpower(power1,power2,power3,power4,power5,power6)
      设置上电力控阈值------参数power范围0-1000

      (18)set_save_flash()
      保存参数到FLASH

      (19)set_force_clb()
      校准力传感器


3、灵巧手使用
    1）将灵巧手与电脑连接，查看连接端口名称，
    2）用Python IDE打开inspire_hand.py 文件，把ttyUSB0改成你的端口号
       （端口名称应该会显示ttyUSB0或者是ttyUSB1、ttyUSB2等），
    3）在Python IDE的shell中运行上面的控制接口即可。




     

