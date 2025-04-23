moteus-n1 is intended as a higher power/voltage, more flexible, yet
smaller in footprint version of the moteus brushless controller.
<!-- moteus-n1 是一种更高功率/电压、更灵活且占用空间更小的无刷控制器版本。 -->

# Features #
<!-- 特性 -->

 * 51V max voltage (vs 44V for moteus)
   <!-- 最大电压51V（相比moteus的44V） -->
 * More flexible aux1/aux2 connectors
   <!-- 更灵活的aux1/aux2连接器 -->
   * both JST GH (aux1 is 8 pin, aux2 is 7 pin)
     <!-- 都是JST GH连接器（aux1是8针，aux2是7针） -->
   * 200mA of 5V between both connectors
     <!-- 两个连接器之间提供200mA的5V电流 -->
   * 200mA of 3V between both connectors
     <!-- 两个连接器之间提供200mA的3V电流 -->
   * aux1 now has two pins that can be used even if onboard encoder is used
     <!-- 即使使用板载编码器，aux1现在也有两个可用针脚 -->
   * SPI on both connectors
     <!-- 两个连接器均支持SPI -->
   * UART on both connectors
     <!-- 两个连接器均支持UART -->
   * ADC on both connectors
     <!-- 两个连接器均支持ADC -->
   * Sine/cosine on both connectors
     <!-- 两个连接器均支持正弦/余弦信号 -->
   * Hardware quadrature (higher count rates)
     <!-- 硬件正交编码器（更高的计数率） -->
 * RS422 transceiver and connector (consumes the two spare pins on AUX1)
   <!-- RS422收发器和连接器（占用AUX1上的两个备用针脚） -->
 * No connectors required on bottom of board for smaller footprint
   <!-- 板底无需连接器以减少占用空间 -->
 * Optional solder pads for power in for higher current capability
   <!-- 可选的电源输入焊盘以支持更高电流能力 -->
 * Variants
   <!-- 变体 -->
   * optional power/CAN connectors on bottom for daisy chaining
     <!-- 板底可选电源/CAN连接器以支持菊花链连接 -->
 * CAN transceiver with 58V bus fault voltage
   <!-- 带有58V总线故障电压的CAN收发器 -->
 * 46x46mm
   <!-- 尺寸为46x46mm -->

# Pin Allocation #
<!-- 引脚分配 -->

 * PA0  - MOTOR 1       (fast ADC1/2) <!-- 电机1（快速ADC1/2） -->
 * PA1  - MOTOR 2       (fast ADC1/2) <!-- 电机2（快速ADC1/2） -->
 * PA2  - MOTOR 3       (fast ADC1)   <!-- 电机3（快速ADC1） -->
 * PA3  - CURRENT       (fast ADC1)   <!-- 电流（快速ADC1） -->
 * PA4  - DEBUG_DAC                    <!-- 调试DAC -->
 * PA5  - AUX1_A                       <!-- AUX1_A -->
 * PA6  - CURRENT       (fast ADC2)   <!-- 电流（快速ADC2） -->
 * PA7  - AUX1_C                       <!-- AUX1_C -->
 * PA8  - TSENSE2       (fast ADC5)   <!-- 温度传感器2（快速ADC5） -->
 * PA9  - VSENSE        (fast ADC5)   <!-- 电压传感器（快速ADC5） -->
 * PA10 - AUX2_A                       <!-- AUX2_A -->
 * PA11 - AUX2_B                       <!-- AUX2_B -->
 * PA12 - AUX2_I2C_PULLUP              <!-- AUX2 I2C上拉 -->
 * PA13 - SWDIO                        <!-- SWDIO -->
 * PA14 - SWCLK                        <!-- SWCLK -->
 * PA15 - AUX1_D                       <!-- AUX1_D -->
 * PB0  - DRV8353 CS                   <!-- DRV8353片选 -->
 * PB1  - CURRENT        (fast ADC3)  <!-- 电流（快速ADC3） -->
 * PB2  - AS5047P CS     (ADC2)       <!-- AS5047P片选（ADC2） -->
 * PB3  - AUX1_E                       <!-- AUX1_E -->
 * PB4  - AUX1_B                       <!-- AUX1_B -->
 * PB5  - FDCAN2_RX                    <!-- FDCAN2接收 -->
 * PB6  - FDCAN2_TX                    <!-- FDCAN2发送 -->
 * PB7  - AUX2_C                       <!-- AUX2_C -->
 * PB8  - AUX1_I2C_PULLUP              <!-- AUX1 I2C上拉 -->
 * PB9  - AUX1_E                       <!-- AUX1_E -->
 * PB10 - RS422_RE                     <!-- RS422接收使能 -->
 * PB11 - RS422_DE      (ADC1/2)      <!-- RS422发送使能（ADC1/2） -->
 * PB12 - TSENSE1      (fast ADC4)    <!-- 温度传感器1（快速ADC4） -->
 * PB13 - MOTOR_FAULT  (fast ADC3)    <!-- 电机故障（快速ADC3） -->
 * PB14 - AUX1_A       (fast ADC1/4)  <!-- AUX1_A（快速ADC1/4） -->
 * PB15 - LED1         (fast ADC4)    <!-- LED1（快速ADC4） -->
 * PC4  - AUX2_B       (fast ADC2)    <!-- AUX2_B（快速ADC2） -->
 * PC6  - LED2                        <!-- LED2 -->
 * PC10 - DRV8323 SCK                  <!-- DRV8323时钟 -->
 * PC11 - DRV8323 MISO                 <!-- DRV8323主输入从输出 -->
 * PC13 - DRV8323 MOSI                 <!-- DRV8323主输出从输入 -->
 * PC14 - MOTOR_ENABLE                 <!-- 电机使能 -->
 * PC15 - MOTOR_HIZ                    <!-- 电机高阻态 -->
 * PF0 - AUX2_A                        <!-- AUX2_A -->
 * PF1 - AUX2_D                        <!-- AUX2_D -->

# Pins Required #
<!-- 所需引脚 -->

     * LEDs - 2
       <!-- LED - 2 -->
     * as5047 CS - 1
       <!-- as5047片选 - 1 -->
     * drv8353 CS - 1
       <!-- drv8353片选 - 1 -->
     * 2x temp sense - 2
       <!-- 2个温度传感器 - 2 -->
     * debug uart - 2
       <!-- 调试UART - 2 -->
     * swd - 2
       <!-- SWD - 2 -->
     * DAC debug?
       <!-- DAC调试？ -->
     * voltage sense - 1
       <!-- 电压传感器 - 1 -->
     * aux ADC - 2
       <!-- 辅助ADC - 2 -->
     * primary aux - 1
       <!-- 主辅助 - 1 -->
     * CAN - 2
       <!-- CAN - 2 -->
     * current sense - 3
       <!-- 电流传感器 - 3 -->
       * needs to be "fast" ADC channels
         <!-- 需要是“快速”ADC通道 -->
     * aux fixed conn - 2
       <!-- 辅助固定连接 - 2 -->
     * primary fixed conn - 4 (SPI shared with as5047)
       <!-- 主固定连接 - 4（与as5047共享SPI） -->
     * drv8353 SPI - 3
       <!-- drv8353 SPI - 3 -->
     * gate PWM - 3
       <!-- 门极PWM - 3 -->

# Connectors #
<!-- 连接器 -->

## Aux 1 ##
<!-- 辅助1 -->

 * notional pins (8  pins)
   <!-- 标称针脚（8针） -->
  * 5V
    <!-- 5V -->
  * 3.3v
    <!-- 3.3V -->
  * A: SPI1_SCK  / ADC1 / TIM2_CH1                             - PA5 / PB14
    <!-- A: SPI1_SCK / ADC1 / TIM2_CH1 -->
  * B: SPI1_MISO /      / TIM3_CH1 /          / USART2_RX / 5V - PB4
    <!-- B: SPI1_MISO / / TIM3_CH1 / / USART2_RX / 5V -->
  * C: SPI1_MOSI / ADC2 / TIM3_CH2                             - PA7
    <!-- C: SPI1_MOSI / ADC2 / TIM3_CH2 -->
  * D:           /      / TIM2_CH1 / I2C1_SCL / USART2_RX / 5V - PA15
    <!-- D: / / TIM2_CH1 / I2C1_SCL / USART2_RX / 5V -->
  * E: SPI1_SCK  /      / TIM2_CH2 / I2C1_SDA / USART2_TX / 5V - PB3 / PB9
    <!-- E: SPI1_SCK / / TIM2_CH2 / I2C1_SDA / USART2_TX / 5V -->
  * gnd
    <!-- 地 -->

## Aux 2 ##
<!-- 辅助2 -->

 * 3V and 5V power
   <!-- 3V和5V电源 -->
 * I2C
   <!-- I2C -->
 * quadrature
   <!-- 正交 -->
 * index
   <!-- 索引 -->
 * uart
   <!-- UART -->
 * SPI
   <!-- SPI -->
 * PA14/PA15 have I2C1 and USART2 and TIM8_CH1/2
   <!-- PA14/PA15具有I2C1和USART2以及TIM8_CH1/2 -->
 * PB8/PB9 have I2C1 and USART3 and TIM4_CH3/4
   <!-- PB8/PB9具有I2C1和USART3以及TIM4_CH3/4 -->
 * TIM1/2 options: PA0/1/2/3/5/6/7/12/15
                   PB2/3/4/5/6/7/8/9/14/15
   <!-- TIM1/2选项：PA0/1/2/3/5/6/7/12/15 PB2/3/4/5/6/7/8/9/14/15 -->
 * notional pins (7 pins)
   <!-- 标称针脚（7针） -->
   * 5V
     <!-- 5V -->
   * 3V
     <!-- 3V -->
   * A: SPI2 SCK  /           /          / ADC2 /           - PF1
     <!-- A: SPI2 SCK / / / ADC2 / -->
   * B: SPI2 MISO / USART1_RX / I2C2_SDA / ADC1             - PA10 / PF0
     <!-- B: SPI2 MISO / USART1_RX / I2C2_SDA / ADC1 -->
   * C: SPI2 MOSI / USART1_TX / I2C2_SCL / ADC2 / TIM4_CH1  - PA11 / PC4
     <!-- C: SPI2 MOSI / USART1_TX / I2C2_SCL / ADC2 / TIM4_CH1 -->
   * D:             USART1_RX /                 / TIM4_CH2  - PB7
     <!-- D: USART1_RX / / TIM4_CH2 -->
   * gnd
     <!-- 地 -->
