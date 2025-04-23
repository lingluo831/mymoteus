# Getting started with the moteus controller #
<!-- 开始使用 moteus 控制器 -->

If you are starting from a developer kit, then skip down to the section [Software](#software).
<!-- 如果您从开发套件开始，请跳到[软件](#software)部分。 -->

# Hardware #
<!-- 硬件 -->

## Mechanical Mounting ##
<!-- 机械安装 -->

If using the default onboard encoder, the following steps must be taken:
<!-- 如果使用默认的板载编码器，必须执行以下步骤： -->

1. A diametrically magnetized sense magnet should be attached to the rotor.  The fastening must be rigid, allowing no slip.  Adhesive is generally required, cyanoacrylate or epoxy can be used.
   <!-- 应将径向磁化的感应磁铁安装到转子上。固定必须牢固，不能滑动。通常需要使用粘合剂，可以使用氰基丙烯酸酯或环氧树脂。 -->
2. The moteus controller must be mounted so that the encoder is (a) centered laterally on the sense magnet and (b) has an air-gap of approximately 2mm.  The moteus-r4 encoder is *not* centered under the bolt pattern, check the 2D CAD for details.  This mount must be rigid and allow no slip.
   <!-- 必须安装 moteus 控制器，使编码器 (a) 在横向上居中于感应磁铁，并且 (b) 与磁铁之间的气隙约为 2mm。moteus-r4 编码器*并未*在螺栓图案下居中，请查看2D CAD以获取详细信息。此安装必须牢固且不能滑动。 -->

## Electrical ##
<!-- 电气连接 -->

Phase wires for the motor should be soldered to the pads labeled A, B, and C.  The order does not matter.  An inline connector, like an MR30, MR60, or bullet connectors can be used if repeated disconnection is desired.
<!-- 电机的相线应焊接到标有 A、B 和 C 的焊盘上。顺序无关紧要。如果需要频繁断开连接，可以使用内联连接器，例如 MR30、MR60 或子弹连接器。 -->

*IMPORTANT NOTE ON ELECTRICAL DAMAGE*
<!-- *关于电气损坏的重要说明* -->

moteus uses moderately large currents and moderately high voltages. It is important to take many factors in consideration when deploying it to avoid electrical damage to the controller.  Before soldering cables, or attaching moteus to a new motor, be sure to read and understand each of the following sections in the reference manual:
<!-- moteus 使用中等大的电流和中等高的电压。在部署时，重要的是要考虑许多因素以避免对控制器造成电气损坏。在焊接电缆或将 moteus 连接到新电机之前，请务必阅读并理解参考手册中的以下各部分： -->

 * [Phase Wire Soldering](reference.md#phase-wire-soldering)
   <!-- [相线焊接](reference.md#phase-wire-soldering) -->
 * [Cable Construction](reference.md#power-cable-construction)
   <!-- [电缆结构](reference.md#power-cable-construction) -->
 * [Power Connectorization](reference.md#power-connectorization)
   <!-- [电源连接器化](reference.md#power-connectorization) -->
 * [Regenerative Braking Safety](reference.md#regenerative-braking-safety)
   <!-- [再生制动安全](reference.md#regenerative-braking-safety) -->

# Initial Parameters #
<!-- 初始参数 -->

There are a few parameters you will likely want to configure early on in your setup.
<!-- 在设置初期，您可能需要配置一些参数。 -->

* `servopos.position_min` and `servopos.position_max` these define the bounds of motion which the controller will allow when in position control mode.  Attempting to start beyond this region will fault, and if outside the region in operation, no torque will be applied to go further outside.
  <!-- `servopos.position_min` 和 `servopos.position_max` 定义了位置控制模式下控制器允许的运动范围。尝试从该范围之外启动将导致故障，如果在运行中超出该范围，将不会施加扭矩以进一步超出。 -->
* `servo.max_current_A` the maximum phase current to apply to the motor.  This can be used to limit the maximum torque that the system is capable of regardless of any command sent.
  <!-- `servo.max_current_A` 是施加到电机的最大相电流。无论发送什么命令，这都可以用来限制系统能够提供的最大扭矩。 -->
* `servo.max_velocity` limits the maximum speed the motor is permitted to achieve before no torque is produced
  <!-- `servo.max_velocity` 限制了电机在不产生扭矩之前允许达到的最大速度。 -->
* `servo.default_velocity_limit` / `servo.default_accel_limit` controls how fast the motor can accelerate and spin in order to reach position and velocity targets.  Bare boards ship with these unset, while development kits ship with human-eye pleasing values.
  <!-- `servo.default_velocity_limit` / `servo.default_accel_limit` 控制电机加速和旋转的速度，以达到位置和速度目标。裸板出厂时未设置这些值，而开发套件出厂时设置为适合人眼的值。 -->
* `servo.pid_position` the PID parameters for the position control loop, a [possible tuning procedure](reference.md#pid-tuning) can be found in the reference manual.
  <!-- `servo.pid_position` 是位置控制环的 PID 参数，可以在参考手册中找到[可能的调节过程](reference.md#pid-tuning)。 -->
* `motor_position.rotor_to_output_ratio` any gearbox scaling, a reducing gearbox should be configured with a number smaller than one, e.g. 0.25 for a 4x reduction gearbox.  This affects reported position, speed, and torques.
  <!-- `motor_position.rotor_to_output_ratio` 是任何齿轮箱的缩放比例，减速齿轮箱应配置为小于1的数值，例如4倍减速齿轮箱的值为0.25。这会影响报告的位置、速度和扭矩。 -->
* `id.id` the CAN-FD id used by this device
  <!-- `id.id` 是此设备使用的 CAN-FD ID。 -->

A larger set of parameters is documented in the reference manual.
<!-- 更多参数集记录在参考手册中。 -->

# Calibration #
<!-- 校准 -->

If you started from a bare moteus board, you will need to calibrate it for the attached motor before any control modes are possible.
<!-- 如果您从裸 moteus 板开始，则需要为所连接的电机校准它，然后才能使用任何控制模式。 -->

```
python3 -m moteus.moteus_tool --target 1 --calibrate
使用这个安装
需要python3.10，nypum2.0以下
```

WARNING: Any attached motor must be able to spin freely.  It will be spun in both directions and at high speed.
<!-- 警告：任何连接的电机必须能够自由旋转。它将以高速向两个方向旋转。 -->

# Software #
<!-- 软件 -->

Binary versions of `tview` and `moteus_tool` can be installed for most platforms (desktop Linux, Windows, Mac) via python using the `moteus_gui` package.
<!-- `tview` 和 `moteus_tool` 的二进制版本可以通过 python 使用 `moteus_gui` 包安装在大多数平台（桌面 Linux、Windows、Mac）上。 -->

```
pip3 install moteus_gui
```

## Raspberry Pi ##
<!-- 树莓派 -->

If you have a Raspberry Pi, see the [instructions here](raspberry_pi.md).
<!-- 如果您有树莓派，请参阅[此处的说明](raspberry_pi.md)。 -->

## Linux ##
<!-- Linux -->

If using the fdcanusb, you will need to have udev rules set up in order for regular users to access the device.  Follow the instructions at: https://github.com/mjbots/fdcanusb/blob/master/70-fdcanusb.rules
<!-- 如果使用 fdcanusb，您需要设置 udev 规则，以便普通用户可以访问该设备。请按照以下说明操作：https://github.com/mjbots/fdcanusb/blob/master/70-fdcanusb.rules -->

## Windows ##
<!-- Windows -->

On some modern Windows installations, you need to invoke python as `python`, not `python3`.
<!-- 在某些现代 Windows 安装中，您需要将 python 调用为 `python`，而不是 `python3`。 -->

# Running tview #
<!-- 运行 tview -->

tview lets you configure and inspect the state of the controller.  It can be run like:
<!-- tview 允许您配置和检查控制器的状态。可以像这样运行： -->

```
*python -m moteus_gui.tview --devices=1*  启动命令
```

(Your pip may have installed a `tview` script into your path which you could also use).
<!-- （您的 pip 可能已将 `tview` 脚本安装到您的路径中，您也可以使用它）。 -->

By default, it attempts to detect an attached fdcanusb to communicate with the target.
<!-- 默认情况下，它会尝试检测连接的 fdcanusb 以与目标通信。 -->

tview has three panes, left, right, and bottom.  Further, the left pane has two tabs.
<!-- tview 有三个窗格，左、右和底部。此外，左窗格有两个选项卡。 -->

Left pane, right tab (the default) shows a hierarchical tree of all telemetry items.
<!-- 左窗格，右选项卡（默认）显示所有遥测项目的分层树。 -->

Left pane, left tab shows a hierarchical tree of all configurable parameters.  Parameter values can be updated by double clicking on their value and entering a new one.
<!-- 左窗格，左选项卡显示所有可配置参数的分层树。可以通过双击其值并输入新值来更新参数值。 -->

The right pane shows real time plots of telemetry items.  It can be populated with plots by right clicking on telemetry items in the telemetry tab.
<!-- 右窗格显示遥测项目的实时图表。可以通过右键单击遥测选项卡中的遥测项目来填充图表。 -->

The bottom pane has a command line console which shows the commands sent internally by tview and their responses, and provides an interactive console to interact with the device using the diagnostic protocol.
<!-- 底部窗格有一个命令行控制台，显示 tview 内部发送的命令及其响应，并提供一个交互式控制台，用于使用诊断协议与设备交互。 -->

# How position mode works #
<!-- 位置模式如何工作 -->

The primary control mode for the moteus controller is an integrated position/velocity controller.  The semantics of the control command are somewhat unorthodox, so as to be easily amenable to idempotent commands sent from a higher level controller.  Each command looks like:
<!-- moteus 控制器的主要控制模式是集成位置/速度控制器。控制命令的语义有些非正统，因此可以轻松适应从更高级别控制器发送的幂等命令。每个命令如下所示： -->

 * Position: The desired position in revolutions
   <!-- 位置：以转数表示的目标位置 -->
 * Velocity: The rate at which the desired position changes in revolutions / s
   <!-- 速度：目标位置变化的速率，以转数/秒为单位 -->
 * Maximum torque: Never use more than this amount of torque when controlling
   <!-- 最大扭矩：控制时绝不使用超过此量的扭矩 -->
 * Feedforward torque: Give this much extra torque beyond what the normal control loop says
   <!-- 前馈扭矩：在正常控制环所述的基础上提供额外的扭矩 -->
 * Stop position: If non-special, never move the desired position away from this target.
   <!-- 停止位置：如果不是特殊值，则永远不要将目标位置移离此目标。 -->
 * kp scale: Scale the proportional constant by this factor
   <!-- kp 缩放：按此因子缩放比例常数 -->
 * kd scale: Scale the derivative constant by this factor
   <!-- kd 缩放：按此因子缩放微分常数 -->
 * Velocity limit override: If non-special, override the configured velocity limit, which constrains how quickly the target position is reached.
   <!-- 速度限制覆盖：如果不是特殊值，则覆盖配置的速度限制，该限制约束目标位置的到达速度。 -->
 * Acceleration limit override: If non-special, override the configured acceleration limit, which constrains how quickly the target position is reached.
   <!-- 加速度限制覆盖：如果不是特殊值，则覆盖配置的加速度限制，该限制约束目标位置的到达速度。 -->

Additionally, the position may be set as a "special value" (NaN for floating point and the debug interface, maximal negative for integer encodings).  In that case, the position selected is "wherever you are right now".
<!-- 此外，位置可以设置为“特殊值”（浮点数和调试接口为 NaN，整数编码为最大负值）。在这种情况下，选择的位置是“您现在所在的位置”。 -->

A pure velocity mode can be obtained by setting the kp scale to 0 (or permanently so by configuring the kp constant to 0).  In this case, using the `servo.max_position_slip` configurable parameter may be valuable as per the [reference manual](reference.md#velocity-control).
<!-- 通过将 kp 缩放设置为 0（或通过将 kp 常数配置为 0 来永久设置），可以获得纯速度模式。在这种情况下，根据[参考手册](reference.md#velocity-control)，使用 `servo.max_position_slip` 可配置参数可能很有价值。 -->

# Learning more #
<!-- 了解更多 -->

The complete reference documentation can be found at:
[Reference](reference.md)
<!-- 完整的参考文档可以在以下位置找到：
[参考](reference.md) -->
