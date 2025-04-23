// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// moteus.cc - moteus伺服驱动器固件的主要实现文件
// 文件功能：
// 1. 系统初始化和配置
// 2. 外设驱动（UART、CAN、ADC等）管理
// 3. 主循环实现，包括通信、控制和故障检测

#include <inttypes.h>

#include <functional>

#include "mbed.h"

#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"
#include "mjlib/multiplex/micro_server.h"
#include "mjlib/multiplex/micro_stream_datagram.h"

#include "fw/board_debug.h"
#include "fw/clock_manager.h"
#include "fw/firmware_info.h"
#include "fw/git_info.h"
#include "fw/millisecond_timer.h"
#include "fw/moteus_controller.h"
#include "fw/moteus_hw.h"
#include "fw/system_info.h"
#include "fw/uuid.h"

// 针对STM32G4目标平台引入相关头文件
#if defined(TARGET_STM32G4)
#include "fw/fdcan.h"               // CAN总线驱动
#include "fw/fdcan_micro_server.h"  // CAN通信服务器
#include "fw/stm32g4_async_uart.h"  // 异步UART通信
#include "fw/stm32g4_flash.h"       // Flash存储操作
#else
#error "Unknown target"
#endif

// 定义固件版本号
extern "C" {
  uint32_t kMoteusFirmwareVersion = MOTEUS_FIRMWARE_VERSION;
}

// 硬件寄存器指针
auto* const MyDWT = DWT;    // DWT是ARM内核中的一个调试单元，用于数据观察和跟踪。
auto* const MyFLASH = FLASH;  // Flash控制器

// 命名空间别名定义，命名空间（namespace）是 C++ 中的一种机制，用于组织代码并避免命名冲突。它可以将一组相关的类、函数、变量等放在一个逻辑分组中，从而使代码更易于维护和理解。
using namespace moteus;//引入namespace，使得在当前文件中可以直接使用该命名空间中的标识符，而无需每次都加上 moteus:: 前缀。
namespace micro = mjlib::micro;        //在mjlib下面设置一个叫做micro的子空间，用于微控制器基础功能
namespace multiplex = mjlib::multiplex;  // 同上，用于多路复用通信

#if defined(TARGET_STM32G4)
using HardwareUart = Stm32G4AsyncUart;
using Stm32Flash = Stm32G4Flash;
#else
#error "Unknown target"
#endif

extern "C" {
// 系统时钟初始化函数
void SetupClock() {
#if defined(TARGET_STM32G4)
  {
    // 使能系统配置和电源控制时钟
    __HAL_RCC_SYSCFG_CLK_ENABLE(); // 使能系统配置控制器时钟
    __HAL_RCC_PWR_CLK_ENABLE();// 使能电源控制模块时钟

    // 配置系统时钟
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    // 设置时钟类型和频率
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;    // 使用PLL作为系统时钟源
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // AHB时钟 170 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   // APB1时钟 85 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;   // APB2时钟 85 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
      mbed_die();
    }

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
    // 选择需要配置时钟的外设
    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_FDCAN |
        RCC_PERIPHCLK_USART2 |
        RCC_PERIPHCLK_USART3 |
        RCC_PERIPHCLK_ADC12 |
        RCC_PERIPHCLK_ADC345 |
        RCC_PERIPHCLK_I2C1
        ;
    // 配置各外设时钟源
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1; // CAN通信
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;// 串口2
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;// 串口3
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;// ADC1/2
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;// ADC3/4/5
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;// I2C1
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      mbed_die();
    }

    __HAL_RCC_TIM2_CLK_ENABLE();// 启用定时器2时钟
    __HAL_RCC_TIM3_CLK_ENABLE();// 启用定时器3时钟
    __HAL_RCC_TIM4_CLK_ENABLE();// 启用定时器4时钟
  }
#endif
}
}

#if defined(TARGET_STM32G4)
extern "C" {
extern char _sccmram;
extern char _siccmram;
extern char _eccmram;
extern char _sccmram;
}
#endif

namespace moteus {
volatile uint8_t g_measured_hw_family;
volatile uint8_t g_measured_hw_rev;
volatile uint8_t g_measured_hw_pins;
MoteusHwPins g_hw_pins;
}

namespace {
struct CanConfig {
  uint32_t prefix = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(prefix));
  }

  bool operator==(const CanConfig& rhs) const {
    return prefix == rhs.prefix;
  }
};
}

// 主函数
int main(void) {
#if defined(TARGET_STM32G4)
  // 将CCM RAM数据从初始位置复制到运行时位置
  std::memcpy(&_sccmram, &_siccmram, &_eccmram - &_sccmram);
#endif

  // 使能Flash预取、指令缓存和数据缓存
  FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

#if 0
  // We map SRAM1 onto address 0 just to make finding accesses to
  // address 0 easier.
  SYSCFG->MEMRMP = (SYSCFG->MEMRMP & ~SYSCFG_MEMRMP_MEM_MODE_Msk) | 0x03;  // SRAM1
#endif

  // 初始化系统时钟
  SetupClock();

  // 创建毫秒定时器
  // 注：之前使用Ticker以1ms间隔入队事件，但会导致电流采样中断抖动
  // 现在改为轮询方式检查毫秒翻转
  MillisecondTimer timer;

  // 检测并获取硬件信息
  const auto family_and_version = DetectMoteusFamily(&timer);
  g_measured_hw_family = family_and_version.family;     // 硬件系列
  g_measured_hw_rev = family_and_version.hw_version;    // 硬件版本
  g_measured_hw_pins = family_and_version.hw_pins;      // 硬件引脚配置

  // 根据硬件信息查找对应的引脚定义
  g_hw_pins = FindHardwarePins(family_and_version);

  // We do our compatibility check *after* calling FindHardwarePins so
  // the the family specific pins are known.  That way mbed_die
  // doesn't choke being unable to find the power LED for instance.

  if (family_and_version.hw_version < 0) {
    // This firmware is not compatible with this board.
    mbed_die();
  }

  // We require cycle counting be enabled for some things.
  {
    ITM->LAR = 0xC5ACCE55;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  // Turn on our power light.
  // 初始化电源LED指示灯
  DigitalOut power_led(g_hw_pins.power_led, 0);

  // 创建内存池（20KB）用于动态内存分配
  micro::SizedPool<20000> pool;

  // 如果有UART引脚定义，则初始化RS485通信
  std::optional<HardwareUart> rs485;
  if (g_hw_pins.uart_tx != NC) {
    rs485.emplace(&pool, &timer, []() {
      HardwareUart::Options options;
      options.tx = g_hw_pins.uart_tx;        // 发送引脚
      options.rx = g_hw_pins.uart_rx;        // 接收引脚
      options.dir = g_hw_pins.uart_dir;      // 方向控制引脚
      options.baud_rate = 3000000;           // 波特率3Mbps
      return options;
                                 }());
  }

  FDCan fdcan([]() {
      FDCan::Options options;

      options.td = g_hw_pins.can_td;
      options.rd = g_hw_pins.can_rd;

      options.slow_bitrate = 1000000;
      options.fast_bitrate = 5000000;

      options.fdcan_frame = true;
      options.bitrate_switch = true;
      options.automatic_retransmission = true;

      // Family 0 uses a TCAN334GDCNT, which has a very low loop
      // delay.  Other families use chips with a longer loop delay.
      options.delay_compensation = g_measured_hw_family != 0;

      options.tdc_offset = 13;  // 13 / 85MHz ~= 152ns
      options.tdc_filter = 2; // 2 / 85MHz ~= 23ns

      return options;
    }());
  FDCanMicroServer fdcan_micro_server(&fdcan);
  multiplex::MicroServer multiplex_protocol(
      &pool, &fdcan_micro_server,
      []() {
        multiplex::MicroServer::Options options;
        options.max_tunnel_streams = 3;
        return options;
      }());

  micro::AsyncStream* serial = multiplex_protocol.MakeTunnel(1);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(serial);
  micro::CommandManager command_manager(&pool, serial, &write_stream);
  char micro_output_buffer[2048] = {};
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream, micro_output_buffer);
  Stm32Flash flash_interface;
  micro::PersistentConfig persistent_config(
      pool, command_manager, flash_interface, micro_output_buffer);

  SystemInfo system_info(pool, telemetry_manager);
  FirmwareInfo firmware_info(pool, telemetry_manager,
                             kMoteusFirmwareVersion, MOTEUS_MODEL_NUMBER);
  Uuid uuid(persistent_config);
  ClockManager clock(&timer, persistent_config, command_manager);

  MoteusController moteus_controller(
      &pool, &persistent_config,
      &command_manager,
      &telemetry_manager,
      &multiplex_protocol,
      &clock,
      &system_info,
      &timer,
      &firmware_info,
      &uuid);

  BoardDebug board_debug(
      &pool, &command_manager, &telemetry_manager, &multiplex_protocol,
      moteus_controller.bldc_servo());

  persistent_config.Register("id", multiplex_protocol.config(), [](){});

  GitInfo git_info;
  telemetry_manager.Register("git", &git_info);

  CanConfig can_config, old_can_config;

  persistent_config.Register(
      "can", &can_config,
      [&can_config, &fdcan, &fdcan_micro_server, &old_can_config]() {
        // We only update our config if it has actually changed.
        // Re-initializing the CAN-FD controller can cause packets to
        // be lost, so don't do it unless actually necessary.
        if (can_config == old_can_config) {
          return;
        }
        old_can_config = can_config;

        FDCan::Filter filters[1] = {};
        filters[0].id1 = can_config.prefix << 16;
        filters[0].id2 = 0x1fff0000u;
        filters[0].mode = FDCan::FilterMode::kMask;
        filters[0].action = FDCan::FilterAction::kAccept;
        filters[0].type = FDCan::FilterType::kExtended;
        FDCan::FilterConfig filter_config;
        filter_config.begin = std::begin(filters);
        filter_config.end = std::end(filters);
        filter_config.global_std_action = FDCan::FilterAction::kAccept;
        filter_config.global_ext_action = FDCan::FilterAction::kReject;
        fdcan.ConfigureFilters(filter_config);

        fdcan_micro_server.SetPrefix(can_config.prefix);
      });

  persistent_config.Load();

  moteus_controller.Start();
  command_manager.AsyncStart();
  multiplex_protocol.Start(moteus_controller.multiplex_server());

  auto old_time = timer.read_us();

  for (;;) {
    if (rs485) {
      rs485->Poll();
    }
#if defined(TARGET_STM32G4)
    fdcan_micro_server.Poll();
#endif
    moteus_controller.Poll();
    multiplex_protocol.Poll();

    const auto new_time = timer.read_us();

    // 计算两次轮询之间的时间间隔
    const auto delta_us = MillisecondTimer::subtract_us(new_time, old_time);
    
    // 如果使能了定时故障检测且间隔超过4ms，则触发故障
    if (moteus_controller.bldc_servo()->config().timing_fault &&
        delta_us >= 4000) {
      // 由于错过了多个轮询周期，触发定时违规故障
      moteus_controller.bldc_servo()->Fault(moteus::errc::kTimingViolation);
    }

    // 每1ms执行一次的任务
    if (delta_us >= 1000) {
      telemetry_manager.PollMillisecond();      // 遥测管理器轮询
      system_info.PollMillisecond();            // 系统信息轮询
      moteus_controller.PollMillisecond();      // 电机控制器轮询
      board_debug.PollMillisecond();            // 板级调试轮询
      system_info.SetCanResetCount(fdcan_micro_server.can_reset_count()); // 更新CAN复位计数

      // 更新时间戳
      old_time += 1000;
    }

    // 空闲计数器递增
    SystemInfo::idle_count++;
  }

  return 0;
}

extern "C" {
  void abort() {
    mbed_die();
  }
}
