import spidev
import time
import pigpio

class AMT22Encoder:
    """
    AMT22 绝对值编码器类 (14位分辨率)
    """
    def __init__(self, pi_instance, cs_pin, bus=1, device=0, speed_hz=500000):
        self.pi = pi_instance
        self.cs_pin = cs_pin
        
        # 初始化 SPI
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.no_cs = True           # 必须为 True，因为我们要手动控制 CS 引脚
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0b00
        self.spi.bits_per_word = 8
        
        # 初始化 CS 引脚
        self.pi.set_mode(self.cs_pin, pigpio.OUTPUT)
        self.pi.write(self.cs_pin, 1)   # 初始状态设为高电平
        
        time.sleep(0.1) # 等待初始化稳定

    def _cs_low(self):
        self.pi.write(self.cs_pin, 0)
        time.sleep(0.00001) # 10us 建立时间

    def _cs_high(self):
        time.sleep(0.00001) # 10us 保持时间
        self.pi.write(self.cs_pin, 1)

    def read_data(self):
        """
        读取编码器原始数据并计算角度
        返回: (raw, pos, angle)
        """
        try:
            self._cs_low()
            # 发送两个 NOP 指令 (0x00) 来换取 16 位数据
            rx = self.spi.xfer2([0x00, 0x00])
            self._cs_high()

            raw = (rx[0] << 8) | rx[1]
            
            # AMT22 的高2位通常是校验位，低14位是位置
            pos = raw & 0x3FFF  
            angle = pos * 360.0 / 16384.0
            
            return raw, pos, angle
        except Exception as e:
            print(f"Encoder Read Error: {e}")
            return None, None, None

    def close(self):
        """释放资源"""
        self.spi.close()
