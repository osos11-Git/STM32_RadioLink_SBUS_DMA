# STM32_RadioLink_SBUS_DMA
 STM32_RadioLink_SBUS_DMA

STM32F401CCU6 Blackpill

STM32 F4 FW 1.26.2

STM32CubeIDE v1.16.0

HAL

UART2 with DMA .

This code communicates with SBUS receiver. Only decoder. 
```

The SBUS protocol uses an inverted serial logic with a baud rate of 100000, 8 data bits, even parity, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:

   * Byte[0]: SBUS header, 0x0F
   * Byte[1 -22]: 16 channels, 11 bits each
   * Byte[23]
      * Bit 7: channel 17 
      * Bit 6: channel 18 
      * Bit 5: SBUS_SIGNAL_LOST (0x01) 
      * Bit 4: SBUS_SIGNAL_FAILSAFE (0x03) 
   * Byte[24]: SBUS footer (0x00)
 ```  
   
Channels data is transferred to the `CH` variable.

Tested with R7FG v1.2 and RC6GS v1. 
Tested with R7FG v1.4 and RC6GS v3. 
 
R7FG receiver will output a range of 170 - 1876 (neutral 1023) with channels set to a range of -100% to +100%. Using extended limits of -120% to +120% outputs a range of 0 to 2047, which is the maximum range acheivable with 11 bits of data.  

# Breadboard Circuit:

![alt text](https://github.com/osos11-Git/STM32_RadioLink_SBUS_DMA/blob/main/Pics/breadboard.png?raw=true)

# CubeMX configs:

![alt text](https://github.com/osos11-Git/STM32_RadioLink_SBUS_DMA/blob/main/Pics/pins.JPG?raw=true)
![alt text](https://github.com/osos11-Git/STM32_RadioLink_SBUS_DMA/blob/main/Pics/uart.JPG?raw=true)
![alt text](https://github.com/osos11-Git/STM32_RadioLink_SBUS_DMA/blob/main/Pics/uart_dma.JPG?raw=true)

