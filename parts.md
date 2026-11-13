```flow 一条运动指令怎么变成电机转动
上位机 | 电脑或主控制器 | user | 外部
对外通信
运动模式
控制核心
控制算法
硬件接口
Y01 驱动板
电机 | 真的转起来 | spark | 外部
上位机 -> 对外通信: 指令
对外通信 -> 运动模式: 选模式
运动模式 -> 控制核心: 目标值
控制核心 -> 控制算法: 每拍计算
控制核心 -> 硬件接口: 三相电压
硬件接口 -> Y01 驱动板: 落到芯片
Y01 驱动板 -> 电机: 通电
```

```flow 没有硬件时在电脑上跑
调试软件 | 电脑上的上位机 | user | 外部
虚拟驱动器
对外通信
控制核心
控制算法
波形结果 | 回到调试软件 | layout
调试软件 -> 虚拟驱动器: 本机网络
虚拟驱动器 -> 对外通信: 收到消息
对外通信 -> 控制核心: 参数指令
控制核心 -> 控制算法: 含虚拟电机
控制核心 -> 波形结果: 状态数据
```

```flow 从模型到固件
工程师 | 在 Simulink 里画控制器 | user
模型工程
控制算法
控制核心
Y01 驱动板
烧进驱动器 | 一份固件 | box
工程师 -> 模型工程: 改模型
模型工程 -> 控制算法: 生成 C 代码
控制算法 -> 控制核心: 被调用
控制核心 -> Y01 驱动板: 一起编译
Y01 驱动板 -> 烧进驱动器: 固件
```

# 驱动器里的程序

## 运动模式
驱动器能做的每一种动作各是一个"模式"：走到某位置、保持某速度、出某力矩、回零等，一次只运行一个。
图标：play
- `app/`

```flow
选模式指令 | 来自对外通信 | mail
调度器 | 同类模式一次只跑一个
准备 | 初始化并等待启动
运行 | 硬件就绪才每拍执行
收尾 | 成功或出错都停下
控制核心
选模式指令 -> 调度器: 换模式
调度器 -> 准备: 先停旧的
准备 -> 运行: 启动成功
运行 -> 控制核心: 目标值
运行 -> 收尾: 完成或出错
```

## 电机辨识
自动测出一台陌生电机的特性：转向、极对数、电角度零点、电气和机械参数、摩擦与力矩波动。
图标：search
- `app/Identify/`

## 安全监测
不停检查电压、电流、温度、编码器等是否正常，管抱闸，判断现在允不允许运动。
图标：lock
敏感：这里决定什么时候报错停机、什么时候松开抱闸，改错了电机可能在不该动的时候动。
- `app/app_status_check.*`
- `app/app_brake_control.*`
- `app/app_motion_precondition.*`

## 参数保存
把参数、故障记录和摩擦补偿表存进闪存，断电不丢。
图标：database
敏感：闪存擦写次数有限；程序里有"短时间写太多就上锁、重启才解锁"的保护。
- `app/app_store_param.*`
- `board_public/flash_db/`

```flow
保存指令 | 写入或擦除 | mail
保存任务 | 每毫秒看一次
写入保护 | 写得太勤就上锁
FlashDB 库
板上闪存 | 断电不丢 | database
保存指令 -> 保存任务: 请求
保存任务 -> 写入保护: 先检查
写入保护 -> FlashDB 库: 允许才写
FlashDB 库 -> 板上闪存: 落盘
```

### FlashDB 库
别人写的小型闪存数据库，负责把一条条数据可靠地写进闪存。
图标：box
外来：开源库 FlashDB 及其 FAL 闪存抽象层（作者 armink），整份拷进仓库。
- `board_public/flash_db/`

## 控制核心
固定节拍反复执行的主循环：读测量值，按当前模式算出该给电机多大电压，再交给硬件输出。
图标：spark
敏感：这是直接决定电机电压的地方，节拍和顺序都不能乱。
- `motor_control/`

```flow
测量值 | 电流、位置、母线电压 | plug
位置速度环 | 较慢的一拍
运动模式
模式状态机 | 按模式算目标电流
电流环 | 最快的一拍，默认每秒两万次
控制算法
三相电压 | 交给硬件输出 | spark
测量值 -> 位置速度环: 位置
位置速度环 -> 运动模式: 先跑模式
位置速度环 -> 模式状态机: 目标
模式状态机 -> 电流环: 目标电流
电流环 -> 控制算法: 调用
电流环 -> 三相电压: 电压
```

## 控制算法
真正的数学部分：电流环、位置速度环、轨迹规划、陷波滤波、输入整形、各种辨识，还有一个虚拟电机。全部由 Simulink 模型自动生成，不手改。
图标：grid
- `simulink_code_gen/`

## 参数总表
驱动器所有参数和状态量的集中存放处，通信、模式、算法都从这里读写。
图标：database
- `database/`

## 软件示波器
在驱动器内部录下任意变量的波形，再传给电脑看。
图标：layout
- `common/software_scope/`

## 通用工具
到处都用得上的小零件：数学函数、环形缓冲区、单位与坐标换算。
图标：box
- `common/`

## 对外通信
驱动器和外界说话的全部方式：调试用的 MAVLink，工业现场用的 CANopen 和 EtherCAT，以及它们共用的 CiA402 驱动器规范。
图标：plug
- `communication/`

```flow
上位机 | 电脑或主控制器 | user | 外部
MAVLink 通道
CANopen 协议栈
EtherCAT 从站
CiA402 规范
参数总表
上位机 -> MAVLink 通道: 串口或 CAN
上位机 -> CANopen 协议栈: CAN 总线
上位机 -> EtherCAT 从站: 网线
CANopen 协议栈 -> CiA402 规范: 对象读写
EtherCAT 从站 -> CiA402 规范: 对象读写
MAVLink 通道 -> 参数总表: 读写参数
CiA402 规范 -> 参数总表: 读写参数
```

### MAVLink 通道
调试和调参用的消息通道，上位机软件靠它看波形、改参数、传文件。消息定义的头文件是生成的。
图标：mail
- `communication/callback/mavlink/`
- `communication/protocol/MAVLink/`

### CANopen 协议栈
CAN 总线上的标准协议实现，以及描述"驱动器有哪些可读写项"的对象字典。
图标：plug
外来：开源协议栈 CanFestival；其中 SlaveOD 对象字典是按本项目的 EDS 文件生成的。
- `communication/protocol/CANopen/`
- `communication/callback/canopen/`

### EtherCAT 从站
通过网线接入 EtherCAT 主站的从站协议栈（配 LAN925x 芯片）。
图标：cloud
外来：由 Beckhoff 的 SSC 工具生成的 EtherCAT 从站协议栈；回调文件是本项目写的。
- `communication/protocol/EtherCAT_LAN925x/`
- `communication/callback/ethercat/`

### CiA402 规范
工业伺服驱动器的通用"行为规范"：上电、使能、运行、故障这些状态怎么切换，控制字和状态字怎么读写。CANopen 和 EtherCAT 都走这里。
图标：book
- `communication/protocol/CiA402/`
- `communication/callback/cia402_od/`

## 出厂诊断
给工厂和研发用的服务：在芯片内存里高速录下电流环的波形，录完再整批取走。默认不编译进固件。
图标：flask
- `service/`
- `tools/factory_service_codegen/`
- `cmake/FactoryServiceCodegen.cmake`

```flow
电流环采样 | 接入点已定，尚未接进正式固件 | spark
录波器 | 存在芯片内存，不占堆
触发 | 手动或按信号沿
冻结 | 录满就停
整批读取 | 录完才传，不实时发 | mail
电流环采样 -> 录波器: 每拍一组
触发 -> 录波器: 开始
录波器 -> 冻结: 录满
冻结 -> 整批读取: 取走
```

# 硬件与运行环境

## 硬件接口
程序和电路板之间的一份约定：读电流、读编码器、输出电压等都通过它，换板子时上面的程序不用改。还包含上电自检和各型号电机的参数表。
图标：plug
- `board_public/`

```flow
上电 | 驱动器通电 | spark
母线电压检查 | 稳定半秒才算正常
编码器检查 | 两个编码器都要就绪
电流零点校准 | 第一次使能时采一千多次取平均
可以运行 | 硬件就绪 | play
上电 -> 母线电压检查: 先看电压
母线电压检查 -> 编码器检查: 正常
编码器检查 -> 电流零点校准: 就绪
电流零点校准 -> 可以运行: 校准成功
```

## Y01 驱动板
把程序装到真实电路板（STM32H743 芯片）上的工程：芯片外设配置、板级驱动、引导程序、固件打包。
图标：home
- `embedded_project/`

```flow
上电 | 板子通电 | spark
引导程序
启动与外设
板级驱动
控制核心
芯片官方库
上电 -> 引导程序: 先运行
引导程序 -> 启动与外设: 跳到固件
启动与外设 -> 板级驱动: 初始化
板级驱动 -> 控制核心: 中断里调用
板级驱动 -> 芯片官方库: 操作芯片
```

### 板级驱动
本项目自己写的驱动：CAN、编码器、闪存、固件升级，以及在定时中断里调用控制循环的代码。
图标：plug
- `embedded_project/Y01/Board/`

### 启动与外设
芯片厂工具生成的启动代码：配时钟、定时器、模数转换、中断入口。
图标：play
- `embedded_project/Y01/Core/`
- `embedded_project/Y01/*.ioc`

### 引导程序
上电后最先运行的小程序：校验固件，需要时从外部闪存升级或恢复，然后跳进正式固件。
图标：lock
敏感：升级和恢复固件的逻辑在这里，出错可能让驱动器启动不了。
- `embedded_project/Y01/BootLoader/`

### 芯片官方库
芯片厂商提供的底层库。
图标：box
外来：意法半导体的 STM32H7 HAL 驱动库和 ARM 的 CMSIS，正式固件和引导程序各带一份。
- `embedded_project/Y01/Drivers/`
- `embedded_project/Y01/BootLoader/Drivers/`

### 工程与打包
Keil 工程文件，以及把引导程序和固件合成一个文件、计算校验值的脚本。
图标：box
- `embedded_project/Y01/MDK-ARM/`
- `embedded_project/Y01/FwProcess/`
- `embedded_project/keilkill.bat`

## 虚拟驱动器
不接任何硬件，在 Windows 电脑上把整套程序跑起来：用假的硬件接口加虚拟电机，通过本机网络和调试软件通信。
图标：flask
- `SIL_project/`
- `CMakeLists.txt`
- `CMakePresets.json`
- `.vscode/`

```flow
调试软件 | 电脑上的上位机 | user | 外部
ZeroMQ 库
对外通信
控制核心
虚拟硬件 | 假的电流、编码器读数 | plug
虚拟电机 | 算法里的电机模型 | spark
调试软件 -> ZeroMQ 库: 本机 5555 端口
ZeroMQ 库 -> 对外通信: MAVLink 消息
对外通信 -> 控制核心: 参数指令
控制核心 <-> 虚拟硬件: 读写
虚拟硬件 <-> 虚拟电机: 电压与反馈
```

### ZeroMQ 库
别人写的消息通信库，这里用来在本机和调试软件之间传数据。
图标：box
外来：开源库 ZeroMQ 的 Windows 预编译版本和头文件；收发封装是本项目写的。
- `SIL_project/zmq/`

# 设计与生成

## 模型工程
控制算法的源头：在 MATLAB Simulink 里画的控制器模型、电机模型、仿真和测试，生成代码的脚本也在这里。
图标：layout
- `simulink_project/`

```flow
脚本与模块库
控制器模型
被控对象
仿真与测试
控制算法
脚本与模块库 -> 控制器模型: 提供模块
控制器模型 <-> 被控对象: 闭环
控制器模型 -> 仿真与测试: 验证
脚本与模块库 -> 控制算法: 生成 C 代码
```

### 控制器模型
每个控制功能各一个模型：电流环、位置速度环、轨迹规划、滤波、辨识等。
图标：grid
- `simulink_project/controller/`
- `simulink_project/architecture/`
- `simulink_project/module_code_gen/`

### 被控对象
电机、编码器、电源等的仿真模型，用来代替真实硬件。
图标：spark
- `simulink_project/plant/`

### 仿真与测试
各个环路的仿真模型和测试用例。
图标：flask
- `simulink_project/simulation/`
- `simulink_project/test_harness/`

### 脚本与模块库
MATLAB 脚本（生成代码、整定参数、算滤波器）和自定义模块库。
图标：note
- `simulink_project/utilities/`
- `simulink_project/lib/`

### 工程资源
Simulink 工程自己的配置文件和数据字典，基本是软件自动维护的。
图标：box
- `simulink_project/resources/`
- `simulink_project/data/`
- `simulink_project/simulink_project.prj`

## 产品契约
一份"唯一真相"文件，规定参数、信号、故障码的编号和含义；工具据此生成给各方用的文件，并自动检查固件有没有和它对不上。
图标：note
规矩：改契约覆盖的字段，先改 product_contract.json，再校验、重新生成，不许另存第二份可编辑的副本（contract/README.md）
- `contract/`
- `.github/`

```flow
契约文件 | 一份 JSON | note
校验工具 | 四个 Python 脚本
生成物 | 头文件、表格、清单 | box
参数总表
对外通信
自动检查 | 每次提交都跑 | lock
契约文件 -> 校验工具: 读入
校验工具 -> 生成物: 生成
校验工具 -> 参数总表: 核对字段
校验工具 -> 对外通信: 核对地址
自动检查 -> 校验工具: 触发
```

## 代码生成工具
把表格和配置变成代码的辅助工具，主要用于生成通信用的对象字典。
图标：box
- `tools/`
- `cmake/`

### 对象字典生成
从 EDS 文件和 CiA402 配置生成 CANopen 对象字典的代码和文档。
图标：note
- `tools/canopen_codegen/`
- `cmake/CanopenCodegen.cmake`

### 数据库生成
参数总表和 MAVLink 消息的源表格，以及生成它们的程序（只有可执行文件，没有源码）。
图标：database
- `tools/database_generate/`

### EtherCAT 资料
生成 EtherCAT 从站协议栈用的厂商工具包、移植驱动和教程。
图标：book
外来：Beckhoff SSC 工具、Microchip LAN9252 SDK 及移植驱动。
- `tools/SSC Generate LAN9252_EtherCAT Protocol Stack/`

## 项目说明
仓库首页的说明和许可证。说明文件目前还是代码托管平台的默认模板，没有写项目内容。
图标：book
- `README.md`
- `README.en.md`
- `LICENSE`
- `.gitignore`
- `.gitattributes`
