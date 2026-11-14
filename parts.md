```flow
上位机 | 调试软件 | user
总线主站 | PLC 或运动控制器 | user
电机 | 真实电机 | spark | 外部
模型、契约与工具 | 在电脑上设计模型、定义契约、生成代码，不在驱动器里运行
上位机 <-> 硬件与运行平台: 调试报文
总线主站 <-> 硬件与运行平台: 总线报文
硬件与运行平台 <-> 对外通信: 收发报文
对外通信 <-> 伺服控制核心: 参数与目标
伺服控制核心 <-> 硬件与运行平台: 采样与电压
硬件与运行平台 <-> 电机: PWM 与反馈
```

```flow 上位机调一个控制参数
上位机 | 调试软件 | user
上位机 -> Y01 驱动器固件: 串口报文
Y01 驱动器固件 -> 上位机调试通道: 收到字节
上位机调试通道 <-> MAVLink 协议库: 解包
上位机调试通道 -> 电机控制环: 新参数
电机控制环 <-> 控制算法: 下周期生效
```

```flow 总线主站让电机走到目标位置
总线主站 | PLC 或运动控制器 | user
总线主站 -> Y01 驱动器固件: CAN 报文
Y01 驱动器固件 -> CANopen 协议栈: 收到报文
CANopen 协议栈 -> CiA402 驱动规范: 目标位置
CiA402 驱动规范 -> 运动模式与应用: 切换模式
运动模式与应用 -> 电机控制环: 目标位置
电机控制环 <-> 控制算法: 算电压
电机控制环 -> 硬件公共接口: 三相电压
硬件公共接口 -> Y01 驱动器固件: 输出 PWM
```

# 伺服控制核心

驱动器里真正让电机按指令转起来的那部分：选运动模式、跑控制环、管参数。
图标：spark

```flow
对外通信 -> 运动模式与应用: 模式与目标
对外通信 <-> 参数数据库: 参数读写
对外通信 <-> 电机控制环: 控制环变量
对外通信 <-> 软件示波器: 配置与波形
运动模式与应用 <-> 参数数据库: 参数
运动模式与应用 <-> 电机控制环: 调度与目标
运动模式与应用 -> 硬件与运行平台: 抱闸与存储
参数数据库 <-> 电机控制环: 同步变量
电机控制环 -> 软件示波器: 每周期采样
电机控制环 <-> 控制算法: 算一步
电机控制环 <-> 硬件与运行平台: 采样与电压
通用工具库 | 数学与单位换算，供各部分取用
```

## 运动模式与应用

决定电机此刻做什么：位置/速度/力矩各种运动模式、电机参数辨识、抱闸、状态检查和参数保存。
图标：play

- `app/`

```flow
对外通信 -> 应用调度: 切换模式 `set_app`
电机控制环 -> 应用调度: 每周期 `AppRun`
电机控制环 -> 状态检查: 每周期 `AppStatusCheck`
应用调度 | 管各模式的启动、运行、停止
运动模式 | 位置、速度、力矩、MIT、回零
辨识模式 | 电阻电感、极对数、电角度等
状态检查 | 过流、过压、过温等故障
保存与抱闸 | 参数写 Flash，控制抱闸
应用调度 -> 运动模式: 启停运行
应用调度 -> 辨识模式: 启停运行
运动模式 -> 电机控制环: 目标与模式
辨识模式 -> 电机控制环: 辨识指令
状态检查 -> 参数数据库: 错误与告警 `app_get_check_error_val`
参数数据库 <-> 保存与抱闸: 参数
保存与抱闸 -> 硬件与运行平台: 存储与抱闸
```

## 电机控制环

驱动器的心跳：每个电流环周期读电流、算电压、输出 PWM；每个位置环周期读编码器、跑应用、算位置速度。
图标：spark

- `motor_control/`

```flow
硬件与运行平台 <-> 电流环: 电流与电压 `CurrentLoopCtrl`
硬件与运行平台 -> 位置速度环: 编码器 `PosSpeedLoopCtrl`
硬件与运行平台 -> 1ms 任务: 定时触发 `UnrealTimeBase1ms`
电流环 | 约 20kHz，读电流算电压
位置速度环 | 读位置，跑应用，算电流目标
1ms 任务 | 电压温度、抱闸、存储
轴数据 | 一根轴的全部控制变量
对外通信 <-> 轴数据: 控制环变量
轴数据 <-> 位置速度环: 配置与状态
位置速度环 <-> 运动模式与应用: 调度与目标 `AppRun`
1ms 任务 -> 运动模式与应用: 慢速任务 `AppStoreUpdata1ms`
位置速度环 <-> 控制算法: 电流目标
电流环 <-> 控制算法: 算电压
位置速度环 -> 软件示波器: 采样 `ScopeIsrHandle`
位置速度环 -> 参数数据库: 同步变量 `AppParamSyncUser`
```

## 控制算法

由 Simulink 模型自动生成的 C 代码：电流环、位置速度环、观测器、滤波器、辨识算法和虚拟电机。
图标：flask
规矩：这里的代码由 Simulink 模型生成，改算法应改模型再重新生成（simulink_project/utilities/code_gen_to_project.m）

- `simulink_code_gen/`

```flow
电机控制环 -> 位置反馈: 编码器计数
电机控制环 -> 轨迹规划: 目标位置
位置反馈 | 编码器换算与测速观测
轨迹规划 | 位置轨迹与输入整形
位置速度控制 | 位置速度调节与陷波
电流控制 | 电流环矢量控制
辨识算法 | 电参数与机械参数辨识
虚拟电机 | 仿真时代替真实电机
位置反馈 -> 位置速度控制: 实际位置
轨迹规划 -> 位置速度控制: 规划位置
位置速度控制 -> 电流控制: 电流目标
电流控制 -> 电机控制环: 三相电压
辨识算法 -> 电机控制环: 辨识结果
电机控制环 <-> 虚拟电机: 电压与电流
```

## 参数数据库

驱动器的全部参数和运行变量都放在这里，通信和应用都从这里读写。
图标：database
敏感：电机参数和保护阈值都在这里，写错会让电机失控或保护失效

- `database/`

```flow
初始化 | 上电载入默认值 `AppParamInit`
参数表 | 全部参数与运行变量
范围限制 | 修改后检查上下限
同步 | 拉取控制环实时变量
初始化 -> 参数表: 默认值
对外通信 <-> 参数表: 参数读写
运动模式与应用 <-> 参数表: 参数
参数表 -> 范围限制: 新值 `app_param_restrictions_update`
范围限制 -> 参数表: 限幅后的值
电机控制环 -> 同步: 每周期 `AppParamSyncUser`
同步 -> 参数表: 实时变量
```

## 通用工具库

数学函数、环形缓冲区、单位与坐标换算，各部分都会用到。
图标：box

- `common/math_lib/`
- `common/ring_buff/`
- `common/unit_conversion/`

## 软件示波器

在驱动器内存里按控制周期采波形，再交给上位机显示。
图标：search

- `common/software_scope/`

```flow
对外通信 -> 示波器配置: 通道与触发 `ControlWordWriteCallback`
电机控制环 -> 采样与触发: 每周期 `ScopeIsrHandle`
示波器配置 | 选通道、采样率、触发条件
采样与触发 | 按周期取样，等触发
波形缓冲 | 环形缓冲里的样本
示波器配置 -> 采样与触发: 配置
采样与触发 -> 波形缓冲: 样本
波形缓冲 -> 对外通信: 波形数据 `ScopeChannelDataRead`
```

# 对外通信

驱动器和外面说话的通道：调试用的上位机通道，以及工业现场用的 CANopen 和 EtherCAT。
图标：plug

```flow
硬件与运行平台 <-> 上位机调试通道: 调试报文
硬件与运行平台 <-> CANopen 协议栈: CAN 报文
硬件与运行平台 <-> EtherCAT 协议栈: EtherCAT 帧
上位机调试通道 <-> MAVLink 协议库: 解包打包
CANopen 协议栈 -> CiA402 驱动规范: 对象读写
EtherCAT 协议栈 -> CiA402 驱动规范: 对象读写
上位机调试通道 <-> 伺服控制核心: 参数与变量
CiA402 驱动规范 <-> 伺服控制核心: 目标与状态
```

## 上位机调试通道

接收上位机发来的 MAVLink 调试指令，读写控制参数、配置示波器、升级固件。
图标：plug

- `communication/callback/mavlink/`

```flow
硬件与运行平台 <-> 解包分发: 调试报文 `MavlinkRecvCallback`
解包分发 | 按消息类型分到各处
示波器接口 | 示波器配置与取数
解包分发 <-> MAVLink 协议库: 解包打包
解包分发 <-> 参数数据库: 参数读写
解包分发 <-> 电机控制环: 控制环变量
解包分发 -> 示波器接口: 示波器指令
示波器接口 <-> 软件示波器: 配置与波形
```

## MAVLink 协议库

MAVLink 消息的打包解包代码，以及由参数表生成的伺服专用消息定义。
图标：mail
外来：MAVLink 官方 C 头文件库；伺服专用消息定义看文件名是由 tools/database_generate 生成

- `communication/protocol/MAVLink/`
- `communication/protocol/README.txt`

## CiA402 驱动规范

工业伺服通用的 CiA402 规范：驱动状态机、状态字、错误码，以及对象字典到参数数据库的读写桥。
图标：grid

- `communication/protocol/CiA402/`
- `communication/callback/cia402_od/`
- `communication/callback/canopen/`
- `communication/callback/ethercat/`

```flow
CANopen 协议栈 -> 对象字典回调: 对象读写
EtherCAT 协议栈 -> 对象字典回调: 对象读写
对象字典回调 | 每个标准对象的读写函数
驱动状态机 | 上电、使能、快停、故障
状态字与错误码 | 把状态和故障编成标准码
对象字典回调 <-> 参数数据库: 参数值
对象字典回调 -> 运动模式与应用: 切换模式
电机控制环 -> 驱动状态机: 每周期 `PDS_FSA_Run`
驱动状态机 -> 参数数据库: 使能失能 `set_app_Controlword`
驱动状态机 -> 状态字与错误码: 当前状态
状态字与错误码 -> 对象字典回调: 状态字
```

## CANopen 协议栈

CAN 总线上的 CANopen 协议：报文收发、PDO/SDO、网络管理，以及本机的对象字典。
图标：plug
外来：CanFestival 开源协议栈；SlaveOD 对象字典由 tools/canopen_codegen 生成
规矩：SlaveOD.c/.h/.eds 由生成器管理，不手改，改对象要扩展生成器后重新生成（service/factory/README.md，2026-11）

- `communication/protocol/CANopen/`

## EtherCAT 协议栈

基于 LAN9252 芯片的 EtherCAT 从站协议栈，以及生成它的 SSC 工具包。
图标：plug
外来：Beckhoff SSC 工具生成的从站协议栈和 Microchip LAN9252 驱动示例

- `communication/protocol/EtherCAT_LAN925x/`
- `tools/SSC Generate LAN9252_EtherCAT Protocol Stack/`

# 硬件与运行平台

让同一套控制核心跑起来的地方：真实的 Y01 驱动器板子，或者电脑上的虚拟驱动器。
图标：home

```flow
上位机 | 调试软件 | user
总线主站 | PLC 或运动控制器 | user
电机 | 真实电机 | spark | 外部
上位机 <-> Y01 驱动器固件: 串口报文
上位机 <-> 电脑仿真驱动器: ZeroMQ 报文
总线主站 <-> Y01 驱动器固件: 总线报文
Y01 驱动器固件 <-> 对外通信: 收发报文
电脑仿真驱动器 <-> 对外通信: 收发报文
Y01 驱动器固件 -> 伺服控制核心: 定时中断
电脑仿真驱动器 <-> 伺服控制核心: 定时与仿真
伺服控制核心 <-> 硬件公共接口: 采样与电压
硬件公共接口 <-> Y01 驱动器固件: 板上实现
硬件公共接口 <-> 电脑仿真驱动器: 虚拟实现
Y01 驱动器固件 <-> Flash 存储库: 参数存取
Y01 驱动器固件 <-> 电机: PWM 与反馈
```

## Y01 驱动器固件

STM32H743 驱动器板子的工程：芯片启动、中断定时、编码器/CAN/Flash 驱动、引导程序和固件打包。
图标：home

- `embedded_project/`

```flow
上位机 | 调试软件 | user
总线主站 | PLC 或运动控制器 | user
电机 | 真实电机 | spark | 外部
固件打包 | 把引导程序和主程序合成烧录文件，供烧录使用
引导程序 -> 主程序入口: 跳转启动
主程序入口 -> 板级驱动: 初始化 `BspInit`
板级驱动 <-> STM32 官方驱动: 外设操作
上位机 <-> 板级驱动: 串口报文
总线主站 <-> 板级驱动: 总线报文
板级驱动 <-> 对外通信: 收发报文
板级驱动 -> 伺服控制核心: 定时中断 `CurrentLoopCtrl`
硬件公共接口 <-> 板级驱动: 板上实现
板级驱动 <-> Flash 存储库: 参数存取
板级驱动 <-> 电机: PWM 与反馈
```

### 板级驱动

把芯片外设包装成控制核心能用的接口：电流采样、PWM、编码器、CAN、Flash、定时中断、固件升级。
图标：plug

- `embedded_project/Y01/Board/`

### 主程序入口

芯片上电后的初始化和 Keil 工程配置，由 STM32CubeMX 生成框架。
图标：play

- `embedded_project/Y01/Core/`
- `embedded_project/Y01/MDK-ARM/`
- `embedded_project/Y01/*.ioc`
- `embedded_project/keilkill.bat`

### 引导程序

上电先运行的小程序：检查并搬运新固件，再跳转到主程序。
图标：lock
敏感：引导程序出错会让板子无法启动或无法升级

- `embedded_project/Y01/BootLoader/`

### 固件打包

把引导程序和主程序合成一个烧录文件并计算校验。
图标：box

- `embedded_project/Y01/FwProcess/`

### STM32 官方驱动

ST 官方的 HAL 库和 CMSIS。
图标：box
外来：ST 官方 STM32H7 HAL 与 ARM CMSIS

- `embedded_project/Y01/Drivers/`
- `embedded_project/Y01/BootLoader/Drivers/`

## 硬件公共接口

控制核心和任何硬件之间的约定：一组 bsp_ 函数接口、各型号电机和驱动器的参数表、上电自检和电流校准。
图标：card

- `board_public/`

```flow
伺服控制核心 <-> bsp 接口: 采样与电压 `bsp_set_phase_voltage`
bsp 接口 | 控制核心调用的硬件函数约定
电机参数表 | 各型号电机与驱动器参数
上电自检 | 母线电压检查与电流校准
电机参数表 -> 伺服控制核心: 默认参数
Y01 驱动器固件 -> 上电自检: 每次中断 `HardwareSelfTestRun`
上电自检 -> 伺服控制核心: 校准状态
bsp 接口 <-> Y01 驱动器固件: 板上实现
bsp 接口 <-> 电脑仿真驱动器: 虚拟实现
```

## Flash 存储库

把参数以键值形式存进 Flash 的开源库。
图标：database
外来：FlashDB 与 FAL 开源库（armink）

- `board_public/flash_db/`

## 电脑仿真驱动器

在电脑上运行的虚拟驱动器：同一套控制代码配上虚拟电机，通过 ZeroMQ 和上位机通信。
图标：cloud

- `SIL_project/`
- `CMakeLists.txt`
- `CMakePresets.json`

```flow
启动入口 | 初始化各模块
ZeroMQ 收发 | 和上位机交换字节
高精度定时器 | 每 1ms 触发一次
虚拟硬件 | 用虚拟电机代替真实硬件
启动入口 -> ZeroMQ 收发: 建立连接 `ZmqHandlerInit`
启动入口 -> 高精度定时器: 启动
上位机 <-> ZeroMQ 收发: ZeroMQ 报文
ZeroMQ 收发 <-> 对外通信: 收发报文 `MavlinkRecvCallback`
高精度定时器 -> 伺服控制核心: 定时回调 `CurrentLoopCtrl`
硬件公共接口 <-> 虚拟硬件: 虚拟实现
虚拟硬件 <-> 伺服控制核心: 虚拟电机
```

# 模型、契约与工具

不跑在驱动器里，但决定驱动器代码长什么样：Simulink 模型、产品契约、工厂服务定义和各种生成器。
图标：flask

```flow
Simulink 模型 | 设计和生成控制算法
产品契约 | 对外参数和错误码的权威定义
工厂服务 | 研发与产线服务通道的定义
代码生成工具 | 生成参数库、消息和对象字典
项目说明与配置 | 说明、许可证、编辑器设置
```

## Simulink 模型

控制算法的源头：在 Simulink 里设计、仿真、测试，再一键生成 C 代码。
图标：flask

- `simulink_project/`

```flow
工程资源 | MATLAB 工程元数据，供 MATLAB 打开工程
生成的代码 | 拷进驱动器工程的 C 代码
模型库与数据字典 -> 控制器模型: 模块与参数
控制器模型 -> 仿真与测试: 被测模型
被控对象模型 -> 仿真与测试: 虚拟电机
控制器模型 -> 生成与调参脚本: 模型
生成与调参脚本 -> 生成的代码: 生成拷贝
```

### 控制器模型

电流环、位置速度环、轨迹规划、滤波器、辨识等算法模型。
图标：spark

- `simulink_project/controller/`

### 被控对象模型

虚拟电机、编码器、霍尔、电源等被控对象模型。
图标：home

- `simulink_project/plant/`

### 仿真与测试

闭环仿真模型和针对单个模块的测试框架。
图标：flask

- `simulink_project/simulation/`
- `simulink_project/test_harness/`

### 模型库与数据字典

公共模块库、模型模板、数据字典和整体架构模型。
图标：grid

- `simulink_project/lib/`
- `simulink_project/data/`
- `simulink_project/architecture/`
- `simulink_project/module_code_gen/`

### 生成与调参脚本

MATLAB 脚本：生成代码并拷进工程，滤波器和 PID 计算，性能评估。
图标：note

- `simulink_project/utilities/`

### 工程资源

MATLAB 工程的元数据文件。
图标：box

- `simulink_project/resources/`
- `simulink_project/*.prj`

## 产品契约

对外承诺的参数、信号、错误码的唯一权威定义，由它生成主机端表格和 C 头文件，并在合并前检查固件没有偏离。
图标：card
规矩：消费方只能锁定 ServoHub 的某个提交并使用生成物，不得另存一份可编辑的契约（contract/README.md，2026-11）
规矩：改契约字段的顺序：先改 product_contract.json，再校验、重新生成、改绑定、最后更新消费方锁定（contract/README.md，2026-11）

- `contract/`
- `.github/`

```flow
维护者 | 改契约的人 | user
固件源码 | 参数头文件与对象字典 EDS
契约定义 | 参数、信号、错误码的 JSON
校验与生成 | 校验、生成、查漂移
生成物 | 主机表格、C 头文件、EDS 片段
合并检查 | 每次提交自动运行
主机软件 | servo_host、ROS 2 等 | user | 外部
维护者 -> 契约定义: 改字段
契约定义 -> 校验与生成: 定义
固件源码 -> 校验与生成: 字段与对象
校验与生成 -> 生成物: 重新生成
合并检查 -> 校验与生成: 触发 `contract_tool.py`
生成物 -> 主机软件: 锁定使用
```

## 工厂服务

研发和产线用的服务通道：定义工厂指令，并在电流环里录制有界波形供分析。
图标：lock
规矩：工厂服务编号不是产品契约编号，不得作为普通产品语义暴露（service/factory/README.md，2026-11）
规矩：未知的 schema 或协议版本一律拒绝（service/factory/README.md，2026-11）

- `service/`
- `tools/factory_service_codegen/`
- `cmake/FactoryServiceCodegen.cmake`

```flow
工厂主机 | 研发与产线软件，传输尚未接通 | user | 外部
服务定义 | 指令、信号、诊断的 JSON
生成器 | 生成编号头文件与 CANopen 绑定
服务接口 | 版本协商、诊断、录制控制
波形录制器 | 有界录制，电流环挂钩尚未接
服务定义 -> 生成器: 定义
生成器 -> 服务接口: 编号头文件
工厂主机 <-> 服务接口: 配置与读取
服务接口 <-> 波形录制器: 触发与样本 `FactoryServiceTraceConfigure`
```

## 代码生成工具

从参数表和 EDS 生成参数数据库、MAVLink 消息和 CANopen 对象字典的工具。
图标：spark

- `tools/canopen_codegen/`
- `tools/database_generate/`
- `cmake/CanopenCodegen.cmake`

```flow
参数表 | 伺服参数与 MAVLink 指令表格
EDS 文件 | 对象字典定义与 CiA402 配置
数据库生成器 | 现成的 Windows 程序
CANopen 生成器 | 包装 objdictgen 的脚本
生成代码 | 参数库、消息、对象字典
参数表 -> 数据库生成器: 表格
EDS 文件 -> CANopen 生成器: 对象定义
数据库生成器 -> 生成代码: 参数与消息
CANopen 生成器 -> 生成代码: 对象字典
```

## 项目说明与配置

仓库说明、许可证和编辑器设置。
图标：note

- `README.md`
- `README.en.md`
- `LICENSE`
- `.gitignore`
- `.gitattributes`
- `.vscode/`
- `docs/`
