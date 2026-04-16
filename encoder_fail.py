import spidev
import time

# 初始化 SPI1
spi = spidev.SpiDev()
try:
    # 打开总线 1, 设备 0 (对应 /dev/spidev1.0)
    spi.open(1, 0)
    
    # 极低速测试，防止线缆干扰 (50kHz)
    spi.max_speed_hz = 50000
    # AMT22 标准模式
    spi.mode = 0
    
    print("开始读取 SPI1 数据 (CS已物理接地)...")
    print("按下 Ctrl+C 停止")
    
    while True:
        # 发送 0x41 (AMT22 读取位置指令) 和一个空字节
        # 使用 xfer2 保持片选（虽然我们已经物理接地了，但这是标准做法）
        resp = spi.xfer2([0x41, 0x00])
        
        # 格式化输出原始十六进制
        hex_resp = [hex(b) for b in resp]
        
        # 解析 14 位位置数据 (根据 AMT22 手册)
        # 高位字节在前，取低 6 位；低位字节在后。
        pos = ((resp[0] & 0x3F) << 8) | resp[1]
        
        print(f"Raw: {hex_resp} | Position: {pos}")
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n测试停止")
except Exception as e:
    print(f"\n发生错误: {e}")
finally:
    spi.close()
