import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 500000
spi.mode = 0

try:
    while True:
        # 发送读取命令 0x41
        resp = spi.xfer2([0x41, 0x00])
        # 打印原始字节数据
        print(f"Received: {[hex(x) for x in resp]}")
        time.sleep(0.5)
except Exception as e:
    print(f"Error: {e}")
finally:
    spi.close()

