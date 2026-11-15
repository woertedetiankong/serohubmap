说明：这是一台伺服驱动器的软件：它接收上位机或总线主站的指令，每秒几万次地读电流和编码器、算出该加多大的电压，让电机按要求转到位置、速度或力矩。同一套控制核心既能烧进 Y01 驱动器板子，也能在电脑上配一台虚拟电机跑，所以很多调试可以先不碰真电机。控制算法不是手写的，而是从 Simulink 模型生成；参数库、通信消息和对象字典也大多由生成器产出，改这些要改源头再重新生成。对外承诺的参数和错误码由产品契约统一定义，每次合并都会自动检查固件有没有偏离它。

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

```calls 上位机调一个控制参数
HOST_UART_IRQ_TASK | 串口收完一段报文、线路空闲时进这个中断，把收到的字节整段交出去
  MavlinkRecvCallback | 拼出消息，按消息类型把新参数写进轴数据，再回显给上位机
    进 rx_data: 串口收到的字节
    出 axis->pos_speed_ctl_config: 位置速度环的新增益
    mavlink_parse_char | 逐字节拼出一条完整消息
      出 r_message: 拼好的消息
    mavlink_msg_posspeedctlconfig_decode | 把“位置速度环配置”消息拆成各个字段
      出 posspeedctlconfig: 上位机发来的增益和开关
    MotorCtlParamSetUpdata | 按电机参数把周期、限速、编码器分辨率等关联配置重算
      notch_filter_config_set | 按周期重算陷波滤波器系数
    CallSendCallback | 把回显的配置发回上位机，通过回调
PosSpeedCtlLoopTask | 下一个位置环周期由定时中断触发，控制环带着新参数跑
  ModeRun | 按当前运动模式选一步来跑
    PositionStep | 位置模式下规划轨迹、算电流目标
      pos_speed_ctl_pip | 用新增益算出电流目标
        进 rtu_config: 含新增益的位置速度环配置
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

```calls 总线主站让电机走到目标位置
canDispatch | CANopen 协议栈收到驱动器板子转来的一帧，按类型交给 SDO、PDO 处理
  进 m: 一帧 CAN 报文
OD_0x607A_Target_position_Write_Callback | 协议栈写目标位置对象时调用它，经对象字典的函数指针 `OD_Write_Callback` 转来
  进 value: 主站写来的目标位置
  set_Target_position | 记下目标位置，并转存进参数数据库
    进 val: 目标位置
PosSpeedLoopCtrl | 每个位置环周期：读编码器，跑应用，算电流目标
  AppRun | 应用调度，运行当前模式
    PpModeRun | 轮廓位置模式：读出目标位置，换算成电机端的规划目标，通过函数指针 `run`
      PosTrajectoryPlanningHandle | 调规划器算出这一周期的位置指令
        出 axis->input_shaping_input.pos_cmd_p: 交给控制环的位置指令
  PosSpeedCtlLoopTask | 控制层状态机在使能时按位置模式做位置速度调节，得出电流目标
CurrentLoopCtrl | 每个电流环中断：读电流，算电压，输出
  CurrentCtlLoopTask | 把轴数据交给电流环算法
    current_ctl_loop_task | 矢量控制，算出三相目标电压
  embedded_project/Y01/Board/bsp_api.c:bsp_set_phase_voltage | 板子上把三相电压换成 PWM 占空比输出
```

# 伺服控制核心

驱动器里真正让电机按指令转起来的那部分：选运动模式、跑控制环、管参数。
图标：spark
说明：这里的代码不关心自己跑在板子上还是电脑上，只通过硬件公共接口取采样、送电压。两个中断撑起它的节奏：电流环约每秒两万次算一次电压，位置环约每秒一万次，在里面先查故障、跑当前的运动模式，再算出电流目标。外面的指令不直接改控制变量，而是先写进参数数据库，由运动模式在下一个周期读走，所以参数数据库是各部分会合的地方。

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
说明：每种模式都登记在一张表里，各有初始化、启动、运行、停止四步；切换时同一时间只留一个运动模式在跑，旧的会先被停掉。模式本身不算电流电压：它从参数数据库读目标位置、轮廓速度这些量，换算到电机端，再交给规划器，写进电机控制环的轴数据。辨识模式也是这样，真正的辨识计算在控制环里跑，这里只管启停和看它做完没有。故障检查一发现问题就把控制字改成失能；存参数、抱闸、指示灯这些慢事每 1ms 处理一次。

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

```calls
set_app | 切换到某个模式：先停掉正在跑的同类模式，再初始化新模式
  进 name: 要切到的模式
  AppStop | 停掉原来那个模式
AppRun | 每个位置环周期，按各模式所处的状态依次调用启动、运行、停止
  进 hw_ready_state: 硬件是否就绪，没就绪的模式退回等待
  PpModeRun | 轮廓位置模式：读目标位置和速度参数，换算成电机端的规划目标，通过函数指针 `run`
    出 kPpMode.traj: 交给规划器的目标位置、速度和加减速
    PosTrajectoryPlanningHandle | 调规划器算出这一周期的位置指令
      出 axis->input_shaping_input.pos_cmd_p: 交给控制环的位置指令
  IdElecModeRun | 电参数辨识：计算在控制环里跑，这里看它做完就失能电机，通过函数指针 `run`
    进 axis->elec_id_sin_output.state_now: 辨识进行到哪一步
AppStatusScanFast | 每个位置环周期跑快速故障检查，有故障就把控制字改成失能
  出 kAppCheck.error: 各项故障的标志位
AppStatusScanSlow | 每 1ms 跑过温、过载等慢速检查，有新故障就排进错误记录
  set_app_Storage_cmd | 提出“把错误记录写进 Flash”的请求
AppStoreUpdata1ms | 每 1ms 看有没有存取请求，有就读写 Flash 或恢复出厂
  进 store_cmd: 读参数、写参数、恢复出厂或写错误记录
AppBrakeControlRun | 每 1ms 按使能状态和转速决定抱闸还是松闸
```

## 电机控制环

驱动器的心跳：每个电流环周期读电流、算电压、输出 PWM；每个位置环周期读编码器、跑应用、算位置速度。
图标：spark
说明：一根轴的全部控制变量放在一个叫 `kAxis` 的结构里，上电时用所选电机型号的默认参数填好，控制环、应用和通信都对着它读写。电流环中断只做三件事：读电流、调生成的电流环算法、把电压交给硬件。位置环中断里先依次跑故障检查、CiA402 状态机和应用调度，再由控制层状态机决定失能、使能还是进故障；电流校准没完成或硬件没就绪时，即使收到使能也不会打开 PWM。这里的代码在中断里运行，加进任何耗时的事都会挤占控制周期。

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

```calls
MotorCtrlInit | 上电时把电机、编码器、保护和各环的默认参数填进轴数据
  出 kAxis.pmsm_config: 电机参数
CurrentLoopCtrl | 电流环中断：读三相电流，算电压，交给硬件输出
  进 kAxis.current_ctl_input.iabc_now_A: 实际三相电流
  出 kAxis.current_ctl_output.uabc_tar_comp_V: 补偿后的三相目标电压
  CurrentCtlLoopTask | 把轴数据交给生成的电流环算法
    current_ctl_loop_task | 控制算法里的电流环矢量控制
PosSpeedLoopCtrl | 位置环中断：读编码器，跑故障检查和应用，算电流目标，再同步变量、采波形
  出 kAxis.motor_ctl_sm_input.hw_ready_state: PWM 已开且电流校准完成
  AppRun | 应用调度，让当前模式写下这一周期的目标
  PosSpeedCtlLoopTask | 位置反馈、测速、保护检查，再由控制层状态机决定失能、使能或故障，使能时按模式运行
    出 axis->motor_ctl_sm_output.state: 失能、使能或故障
    motor_pos_sensor | 把编码器计数换成累计位置和电角度
  AppParamSyncUser | 把实时量同步到参数数据库
  ScopeIsrHandle | 让软件示波器采一个点
UnrealTimeBase1ms | 1ms 任务：滤波母线电压和温度，跑慢速检查、抱闸、指示灯和参数存储
  AppStoreUpdata1ms | 处理存取 Flash 的请求
```

## 控制算法

由 Simulink 模型自动生成的 C 代码：电流环、位置速度环、观测器、滤波器、辨识算法和虚拟电机。
图标：flask
规矩：这里的代码由 Simulink 模型生成，改算法应改模型再重新生成（simulink_project/utilities/code_gen_to_project.m）
说明：每个模块都是一个独立的函数，统一接收“输入、配置、输出、内部状态”四个结构，自己不存全局变量，所以电机控制环可以按模式挑着调用。位置环周期里依次把编码器换成位置和电角度、估出转速、检查保护，再做位置速度调节并滤掉共振；电流环周期里做矢量控制，输出三相电压。虚拟电机只在电脑仿真或不接电机调试时用。这些文件是生成出来的，直接手改会在下次生成时被覆盖。

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

```calls 一个控制周期
PosSpeedCtlLoopTask | 电机控制环每个位置环周期把轴数据交进来
  motor_pos_sensor | 把编码器计数换成累计位置和电角度
    进 rtu_input: 编码器当前计数
    出 rty_output: 累计位置和电角度
  speed_obs_pll | 用锁相环从位置估出转速
    出 rty_output: 估出的转速
  protect_check | 检查过流、母线电压、温度、跟随误差和超速
    出 rty_error: 故障码，0 表示正常
PositionStep | 位置模式下由控制层状态机调用
  input_shaping_ei | 对位置指令做输入整形，抑制末端振动
  pos_speed_ctl_pip | 位置速度调节，算出 q 轴电流目标
    出 rty_output: 电流目标与跟随误差
  CommandCurrentFilter | 把电流目标滤波后交给电流环
    notch_filter | 陷波滤掉电流目标里的共振频率
current_ctl_loop_task | 电流环中断里：坐标变换、电流调节、死区补偿
  进 rtu_input: 实际电流、电角度和电流目标
  出 rty_output: 三相目标电压
SimPlantStep | 仿真时每个电流环周期调用一次
  sim_plant | 虚拟电机：由电压算出电流、转速和编码器计数
```

## 参数数据库

驱动器的全部参数和运行变量都放在这里，通信和应用都从这里读写。
图标：database
敏感：电机参数和保护阈值都在这里，写错会让电机失控或保护失效
说明：所有参数按类别放在一组结构里，每个参数都有一对读写函数，通信、应用和示波器都只通过这些函数读写。写函数会把关：超出范围或电机使能时不许改的值会被拒绝，改了电流或速度上限还会把相关目标一起收紧；切换运行模式也从这里走，它通过事先登记的回调让应用调度换上对应模式。读实时量时，读函数当场从电机控制环取值并换成对外单位，位置环每个周期都调一遍。文件里只有标为用户区的地方是手写的，其余看起来由生成器产出。

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

```calls
AppParamInit | 上电或恢复出厂时，把控制字、状态、编码器、限位等参数填成默认值
  出 kAppBaseConfig: 基础配置，如 CAN 地址和波特率
  set_app_Max_profile_velocity | 写最大轮廓速度，电机使能时不许改
    进 val: 新的最大轮廓速度
    app_param_restrictions_update | 按峰值电流和最高转速，把相关上限和目标收紧
      出 axis->pos_speed_ctl_input.iq_max_A: 交给控制环的电流上限
  app_param_update | 把默认值逐个再写一遍，让有联动的参数各自生效
    set_app_Target_position | 写目标位置，超出软件限位就拒绝
      进 val: 目标位置
      出 返回: 写成功还是超出范围
    set_app_Modes_of_operation | 写运行模式，改控制层模式，再通过回调让应用调度切换
      出 axis->motor_ctl_sm_config.mode: 控制层用的模式
AppParamSyncUser | 每个位置环周期把控制环的实时量换算进参数表
  get_app_Position_actual_value_inc | 从编码器累计位置算出负载端位置
    出 kAppMotionInfo.Position_actual_value_inc: 负载端当前位置
  get_app_Velocity_actual_value | 把观测到的转速换成负载端每分钟转数
```

## 通用工具库

数学函数、环形缓冲区、单位与坐标换算，各部分都会用到。
图标：box

- `common/math_lib/`
- `common/ring_buff/`
- `common/unit_conversion/`

说明：数学部分是一个头文件，按芯片挑选正弦、余弦、开方的实现（ARM 的 DSP 库、另一家芯片的库，或标准库），再加上常用常数和绝对值、数组长度这类小宏；旁边还有一个第三方的快速整数除法库。环形缓冲区目前主要给软件示波器存波形。单位与坐标换算负责在用户单位和内部编码器脉冲之间来回换算，并处理运动方向的正负；参数数据库在写入极性、减速比时会调用它。

## 软件示波器

在驱动器内存里按控制周期采波形，再交给上位机显示。
图标：search

- `common/software_scope/`

说明：上位机先选好要看哪几个参数、采样间隔、点数和触发条件，示波器为每个通道在一块固定大小的内存里分出一段环形缓冲。之后每个位置环周期按间隔读一次这些参数的当前值：触发前循环覆盖，触发后再采够设定的点数就停，所以能看到触发之前的一段。采完的波形由上位机分帧取走。缓冲区总大小是固定的，通道开得越多，每个通道能采的点就越少。

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

```calls
ControlWordWriteCallback | 上位机发来示波器控制字，按它配置、开始采样、停止或重置
  进 control_word: 配置、等触发、立即采、停止或重置
  scope_config | 检查采样点数和间隔，再配置通道
    scope_channel_config | 给每个打开的通道找到它在参数数据库里的变量，并分好环形缓冲
      get_database_param_info | 按参数编号查出变量的地址、类型和字节数
ScopeIsrHandle | 每个位置环周期由电机控制环调用：按间隔采一个点，等触发，判断采满没有
  出 kScopeObject.status_word: 采完后变成采样成功
  scope_data_sample_run | 每个打开的通道读一次变量，放进环形缓冲
  scope_trigger_catch | 看触发通道有没有越过阈值，触发后从触发点前后截取数据
  sample_complete_judge | 采够点数就标记采完
ScopeChannelDataRead | 上位机来取波形时，按帧从环形缓冲里读出一段
  进 channel_num: 第几个通道
  进 subindex: 第几帧
```

# 对外通信

驱动器和外面说话的通道：调试用的上位机通道，以及工业现场用的 CANopen 和 EtherCAT。
图标：plug
说明：有两条互不相干的路：上位机走 MAVLink 调试报文，能读写几乎所有参数、看波形；总线主站走 CANopen 或 EtherCAT，按工业标准 CiA402 只碰规定好的对象。两条路最后都落到参数数据库，所以驱动器用一个“控制权”开关决定此刻听谁的，另一边的写入会被拒绝或忽略。三个协议栈本身都是外面拿来或生成器生成的代码，项目自己写的是两边的回调和 CiA402 状态机。

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
说明：每次收到的一段字节，先一个个拼成一条完整、校验通过的消息；连续拼不出来就把通道清空重来，免得一段坏数据把它卡住。读请求把轴数据或参数数据库里的当前值打包发回；写消息把字段写进去，再回一份当前值，所以上位机总能看到实际写进去的结果。大多数写入只在“通信控制权”交给上位机时才生效，否则被悄悄忽略；只有增益、前馈这类调试量随时可写。写完后控制环会按电机参数把相关配置一起重算，所以有些值写进去后会被换算覆盖。

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

```calls
MavlinkRecvCallback | 板子收到一段字节后调到这里，拼出消息并按类型分发，最后回一条应答
  进 rx_data: 这次收到的字节
  进 axis: 要读写的那根轴的控制变量
  出 send_msg: 准备回给上位机的应答
  mavlink_parse_char | 喂进一个字节，拼满一条且校验通过就交回
    出 r_message: 拼好的一条消息
  mavlink_scope_control_callback | 示波器的开始、停止等指令转交给软件示波器
    ControlWordWriteCallback | 软件示波器按指令配置、复位或开始采样
  set_app_Controlword | 写类消息按字段写进参数数据库，比如控制字；上位机握有控制权时才写
  MotorCtlParamSetUpdata | 写完后让控制环按电机参数把关联配置一起重算
  CallSendCallback | 把打包好的应答交给板子登记的发送函数发回上位机，通过回调
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
说明：总线主站不直接碰驱动器内部，只按标准编号读写对象，比如控制字、运动模式、目标位置；这里把每个对象翻成参数数据库里对应的参数，CANopen 和 EtherCAT 走的是同一套读写函数。主站写控制字时，状态机立刻按标准走一步（准备好、使能运行、快停、故障复位），真正让电机使能或失能的，是状态机写回参数数据库的控制指令。每个位置环周期它还会跑一次：把应用层查出的故障按优先级翻成标准错误码，有故障就转进故障状态，再按当前运动模式更新状态字给主站看。上位机握有控制权时，主站的写入一律被拒绝，两边不能同时指挥电机。

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

```calls 主站写一个对象
SlaveOD_Write_Callback | CANopen 协议栈收到主站写对象时调到这里，通过回调
  进 index: 对象编号，例如 0x6040 控制字、0x6060 运动模式
  进 data: 主站写来的值
  master_cia402_write_callback | 上位机握有控制权就拒绝，否则按对象编号分给对应的写函数
    出 返回: 中止码，告诉主站写成功还是被拒
    OD_0x6040_Controlword_Write_Callback | 写控制字
      set_Controlword | 记下控制字，并立刻让状态机按它跳一步
        出 kPDS_FSA.Controlword: 主站最新的控制字
    OD_0x6060_Modes_of_operation_Write_Callback | 写运动模式
      set_Modes_of_operation | 把标准模式号换成驱动器自己的模式，写进参数数据库
        进 val: 标准模式号（PP、PV、CSP 等）
```

```calls 每个位置环周期
Cia402ErrorCodeScan | 把应用层查出的故障位按优先级翻成一个标准错误码
  进 check_error_val: 应用层报来的故障位
PDS_FSA_Run | 有错误就转进故障状态，有急停请求就先减速停下
  出 CurrentState: 状态机当前所在的状态
  Transition_13_Event | 错误码不为零就判定出了故障
  QuickStopRun | 让应用层急停，速度降到零才算停完
  PDS_FSA_set_statusword | 把当前状态编进状态字的状态位
UpdateStatusword | 按当前运动模式补上目标到达、跟随误差等状态位
  进 kCheckStatusVal: 应用层报来的运动状态
  出 statusword: 给主站看的状态字
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
说明：控制核心只认硬件公共接口里约定的那组函数，板子和电脑各自实现一份：板子上是真的 ADC、PWM 和编码器，电脑上是一台虚拟电机。板子靠芯片中断定时触发控制环，电脑靠 Windows 定时器模拟同样的节奏。能在电脑上验证的（调参、看波形、改算法）先在电脑上做；CAN、EtherCAT、参数存储、引导和升级只有板子上有。

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
说明：上电后先跑引导程序，确认主程序完好才跳过去；主程序配好时钟和外设，启动采样、PWM、编码器和通信，之后主循环什么也不做，所有工作都在中断里进行：电流采样完成触发电流环，位置环定时器触发位置速度环，串口或 CAN 收到数据就交给通信部分。控制算法本身不在这里，这里只负责让芯片按时把数据送进去、把电压送出来。同一套代码靠编译开关分成只用 CAN、带 EtherCAT、正式发布等几个版本，只有正式版启用固件升级；要换电机型号，就在板子的配置里换选另一张电机参数表。

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

```calls 上电到控制环
embedded_project/Y01/Core/Src/main.c:main | 芯片上电后的主程序：配好时钟和外设，登记回调，初始化软件和硬件，然后把一切交给中断
  HalInit | 依次初始化参数、控制环、各应用、参数存储、示波器、抱闸
  BspInit | 启动硬件：等母线电压稳定、校准电流采样、开 1ms 定时器、电流采样中断、PWM、CAN 和调试串口
    出 mavlink_rx_buff: 调试串口开始往这里收字节
CURRENT_LOOP_IRQ_TASK | 每次三相电流采样完成触发一次，约 20kHz，通过中断
  进 hadc: 触发中断的那路 ADC
  出 kBspData.pl_start_state: 第一次跑完电流环后置位，同时启动位置环定时器
  CurrentLoopCtrl | 自检和电流校准都通过后，跑一次电流环
POSITION_LOOP_TIM_IRQ_TASK | 位置环定时器到点触发，约 10kHz，通过中断
  HardwareSelfTestRun | 自检没完成时调用，等编码器就绪
  EncoderDataRead | 读电机端和负载端编码器
  PosSpeedLoopCtrl | 自检完成后跑位置速度环和应用
HOST_UART_IRQ_TASK | 调试串口一帧收完、线路空闲时触发，通过中断
  进 mavlink_rx_buff: 这一帧收到的字节
  MavlinkRecvCallback | 把这些字节交给上位机调试通道解包
```

### 板级驱动

把芯片外设包装成控制核心能用的接口：电流采样、PWM、编码器、CAN、Flash、定时中断、固件升级。
图标：plug
说明：控制核心要的 `bsp_` 函数在这里有板上的实现，比如把三相目标电压按母线电压换算成 PWM 占空比，把 ADC 读数减去校准零点换成电流。各个中断的处理也在这里，按“先自检、再读编码器、再调控制环”的顺序办事，所以改这里的先后或频率，会直接改变控制环的节奏。固件升级也由它完成：新固件先下载进外部 Flash 并校验，再改写启动参数，下次上电由引导程序搬进运行区。

- `embedded_project/Y01/Board/`

### 主程序入口

芯片上电后的初始化和 Keil 工程配置，由 STM32CubeMX 生成框架。
图标：play
说明：程序启动后先初始化 ST 的库和时钟，再逐个初始化 ADC、定时器、SPI、串口、CAN 等外设，然后登记通信层要用的回调，调用公共初始化和板级初始化，之后就停在一个空循环里。外设初始化的代码由 STM32CubeMX 按工程配置生成，手写的部分只能放在 `USER CODE` 标记之间，否则重新生成时会被覆盖。Keil 工程和链接配置也在这里，它们决定几个版本的程序各放在 Flash 的什么位置。

- `embedded_project/Y01/Core/`
- `embedded_project/Y01/MDK-ARM/`
- `embedded_project/Y01/*.ioc`
- `embedded_project/keilkill.bat`

### 引导程序

上电先运行的小程序：检查并搬运新固件，再跳转到主程序。
图标：lock
敏感：引导程序出错会让板子无法启动或无法升级
说明：它先从内部 Flash 读一份启动参数，参数有效就按其中的启动方式走：要升级，就把外部 Flash 里的新固件校验后搬进内部运行区；正常启动，就先校验运行区的固件。校验或搬运失败时，改用外部备份区的固件恢复运行区。最后它检查主程序的栈地址是否合法，关掉自己用过的外设，再跳到主程序。它是单独编译的另一个程序，这里出错板子可能起不来，只能接烧录器重烧。

- `embedded_project/Y01/BootLoader/`

### 固件打包

把引导程序和主程序合成一个烧录文件并计算校验。
图标：box
说明：打包分两步。第一步把 Keil 编出的主程序拷过来，在末尾加上 CRC32 校验值，按时间命名；第二步把仓库里现成的引导程序和这个主程序拼成一个烧录文件，主程序默认放在地址 0x08100000，中间补 0xFF。引导程序和升级流程都靠末尾的校验值判断固件是否完好，所以发布的固件必须经过第一步。两步都是 Windows 批处理，第二步会在命令行里逐项询问文件名和地址。

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
说明：控制核心从不直接碰芯片，只调用这里约定的一组函数（读电流、写电压、读编码器、开关 PWM、存参数等）。每个运行平台各给一份实现，所以同一套控制代码既能上板子，也能在电脑上跑；新增一个硬件平台，就是把这组函数全部实现一遍。电机参数表是一组头文件，每个平台的配置里选一张，其中的数值成为参数数据库和控制环的初始值。上电自检要等母线电压稳定、两个编码器就绪、电流零点校准完，这三件没完成，控制环不会输出电压。

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

```calls 上电自检与电流校准
BusVoltageCheck | 板子启动时反复调用（经 `sys_bus_voltage_check`），母线电压连续正常约 0.5 秒才算稳定，欠压 2 秒或超时 3 秒就报错
  出 sys_msg.bus_voltage_check_flag: 稳定、出错或还不确定
HardwareSelfTestRun | 位置环中断里反复调用，直到电机端和负载端编码器都就绪，超时就报编码器错误
  出 sys_msg.hardware_self_test_status: 为真以后控制环才开始跑
  MotorCtlSmStateInit | 用编码器此刻的位置初始化控制状态，超时就从零开始
    进 motor_pos_now_p: 电机端编码器单圈值
  AppStatusScanFast | 自检期间也扫一遍故障
CurrentCalibrationStep | 第一次使能电机时调用（经 `sys_current_calibration_step`），零电压下采 1024 次电流取平均当零点
  出 sys_msg.current_calibration.drift: 三相电流的零点
  embedded_project/Y01/Board/bsp_api.c:bsp_set_phase_voltage | 板上实现：把三相电压按母线电压换成 PWM 占空比，这里给零电压
    进 voltage: 三相目标电压
  embedded_project/Y01/Board/bsp_api.c:bsp_set_pwm_state | 校准开始时打开 PWM，电流异常时立刻关掉
    进 state: 开或关
embedded_project/Y01/Board/bsp_api.c:bsp_get_phase_current | 控制环取电流时调用：板上实现，用 ADC 读数减去校准零点再换算成安培
  出 piabc: 三相电流
```

## Flash 存储库

把参数以键值形式存进 Flash 的开源库。
图标：database
外来：FlashDB 与 FAL 开源库（armink）

- `board_public/flash_db/`

## 电脑仿真驱动器

在电脑上运行的虚拟驱动器：同一套控制代码配上虚拟电机，通过 ZeroMQ 和上位机通信。
图标：cloud
说明：它把运动模式、参数数据库、控制环、控制算法和上位机调试通道编译成一个 Windows 程序，用一份“虚拟硬件函数”顶替真实板子：电压直接写给虚拟电机，电流从虚拟电机读回，自检和电流校准直接算通过。三个定时器每 1 毫秒各触发一次：一个连跑 20 次电流环和虚拟电机、10 次位置速度环，模拟 20kHz 和 10kHz 的节奏；一个跑 1 毫秒慢速任务；一个检查上位机有没有发来报文。上位机连本机 5555 端口，说的是和真板子一样的调试报文，所以调参、看波形可以先在电脑上走通。CANopen、EtherCAT 没有编进来，参数也不会真正存下来，这些只能在板子上试。

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

```calls 虚拟驱动器跑起来
SIL_project/main.c:main | 在电脑上启动虚拟驱动器：初始化参数、控制环和各应用，开 ZeroMQ，再开三个 1ms 定时器，按回车停止
  ZmqHandlerInit | 在本机 5555 端口开一个 ZeroMQ 连接，等上位机来连
    出 responder: 和上位机通信用的连接
  RegisterRecvCallback | 登记收到报文后交给谁：上位机调试通道的 `MavlinkRecvCallback`
    进 callback: 收包后要调用的函数
  StartHighPrecisionTimer | 用 Windows 定时器队列按周期回调
    进 periodMs: 周期，毫秒
MotorCtrlTimerCallback | 每 1ms 一次，通过定时器线程：连跑 20 次电流环和虚拟电机、10 次位置速度环，相当于 20kHz 和 10kHz
  CurrentLoopCtrl | 跑一次电流环
  SimPlantStep | 虚拟电机按电压算出下一刻的电流和位置
  PosSpeedLoopCtrl | 每两次电流环跑一次位置速度环
ZmqRecvTimerCallback | 每 1ms 一次，通过定时器线程：看上位机有没有发报文来
  ZmqAsyncRecv | 不等待地收一包，收到就交给登记过的函数
    进 responder: 和上位机的连接
    MavlinkRecvCallback | 上位机调试通道解包处理，通过函数指针
```

# 模型、契约与工具

不跑在驱动器里，但决定驱动器代码长什么样：Simulink 模型、产品契约、工厂服务定义和各种生成器。
图标：flask
说明：驱动器里很多代码是从这里来的：控制算法由 Simulink 模型生成，参数库、MAVLink 消息和 CANopen 对象字典由生成器从表格生成。这类生成出来的代码要改，得改源头再重新生成，否则下次生成会把手改的内容冲掉。产品契约和工厂服务则把“对外承诺了什么”写成机器能检查的定义，由自动检查守着。

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
说明：工程师在这里搭控制器，用被控对象模型代替真电机做闭环仿真，效果确认后由脚本生成 C 代码，覆盖到驱动器工程的控制算法目录。参数表格也会被脚本转成模型用的数据字典，所以模型和固件用的是同一份参数定义。这里的文件大多是 Simulink 模型，只能在 MATLAB 里打开；生成脚本里写的是 Windows 路径，要在 Windows 上运行。

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
说明：每个模型对应驱动器里的一段算法，看文件名，生成后就是“控制算法”里同名的 C 代码，比如电流环任务、电机位置传感器、速度观测、陷波滤波、各种辨识和轨迹规划。改算法要在这里改模型再生成，不要直接改生成出来的 C 代码。

- `simulink_project/controller/`

### 被控对象模型

虚拟电机、编码器、霍尔、电源等被控对象模型。
图标：home
说明：用模型代替真实世界：永磁同步电机、编码器、霍尔传感器、直流电源和开关检测，组合成一整套虚拟被控对象。控制器模型和它接在一起，就能在电脑上闭环仿真，不必上真电机。这套虚拟被控对象也会生成 C 代码，就是驱动器里的“虚拟电机”。

- `simulink_project/plant/`

### 仿真与测试

闭环仿真模型和针对单个模块的测试框架。
图标：flask
说明：仿真模型把电流环、速度环、位置环和 MIT 控制分别做成闭环，用来看响应、调参数；测试框架则把单个控制器模型拿出来单独喂输入、看输出。生成代码之前先在这里确认算法行为，这是项目验证控制算法的主要办法。

- `simulink_project/simulation/`
- `simulink_project/test_harness/`

### 模型库与数据字典

公共模块库、模型模板、数据字典和整体架构模型。
图标：grid
说明：公共模块库和模型模板让各个控制器用同一套积木搭。数据字典存参数和总线类型，其中伺服参数字典由脚本从参数表格生成，和固件的参数数据库同源。整体架构模型把各段算法装在一起，示例总模型是一键生成代码时的入口。

- `simulink_project/lib/`
- `simulink_project/data/`
- `simulink_project/architecture/`
- `simulink_project/module_code_gen/`

### 生成与调参脚本

MATLAB 脚本：生成代码并拷进工程，滤波器和 PID 计算，性能评估。
图标：note
说明：最常用的是一键生成：清掉旧代码，编译示例总模型，把生成的 C 文件拷到驱动器工程，再删掉每次都会变的时间和版本注释，免得每次生成都多出没有意义的改动。生成时还会把正弦、余弦换成项目自己数学库里的快速版本。另有一个脚本把参数表格转成模型的数据字典，还有一组计算工具，用来算陷波滤波器、输入整形、PID 参数和阶跃响应指标。

- `simulink_project/utilities/`

### 工程资源

MATLAB 工程的元数据文件。
图标：box
说明：MATLAB 打开这个工程时读的配置：哪些文件属于工程，路径和标签是什么。由 MATLAB 自己维护，平时不用看也不用手改；新建或移动模型时它会跟着变。

- `simulink_project/resources/`
- `simulink_project/*.prj`

## 产品契约

对外承诺的参数、信号、错误码的唯一权威定义，由它生成主机端表格和 C 头文件，并在合并前检查固件没有偏离。
图标：card
规矩：消费方只能锁定 ServoHub 的某个提交并使用生成物，不得另存一份可编辑的契约（contract/README.md，2026-11）
规矩：改契约字段的顺序：先改 product_contract.json，再校验、重新生成、改绑定、最后更新消费方锁定（contract/README.md，2026-11）
说明：一份 JSON 契约写明每个对外参数和信号的编号、类型、单位、读写权限，以及它在 CANopen 里的地址，另外还有错误码和告警码表。工具先检查契约本身，再生成给主机软件和固件用的文件，并对照固件的参数头文件和 CANopen 对象字典，确认字段还在、地址和类型没变。相关文件一改，合并检查就自动跑一遍，生成物过期或固件偏离契约都会让检查失败。目前只覆盖了一部分参数；错误码表是从上位机迁过来的，还没有和固件核对过。

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

```calls 一次完整检查
contract/tools/contract_tool.py:main | 合并检查或维护者在命令行运行，按命令决定只校验、重新生成还是全部检查
  进 args.command: 要做哪一步，合并检查用 all
  load_contract | 读根契约，再把参数、信号、诊断、能力四个分文件拼成一份
    出 返回: 合起来的整份契约
  validate | 逐项检查编号和名字不重复、类型和单位合法、换算规则自洽
    进 contract: 整份契约
  contract/tools/generators.py:generate | 把契约渲染成全部生成物；检查模式下只比对、不写文件
    进 check: 为真时只比对
    出 stale: 已经过期的生成文件
    render_outputs | 算出每个生成文件应有的内容：C 头文件、主机表格、EDS 片段、黄金向量和清单
  check_sources | 对照固件参数头文件和 CANopen 对象字典，确认字段还在、地址名称类型没变
    出 problems: 固件偏离契约的地方
    parse_eds | 把对象字典文件读成按地址查的表
  check_conversion_evidence | 确认已验证的单位换算在固件源码里找得到依据
  check_golden_vectors | 用契约的换算规则重算黄金向量，确认数值和字节都对
  check_mavlink_provenance | 确认 MAVLink 生成头文件没被悄悄换掉，消息编号和字段不变
```

## 工厂服务

研发和产线用的服务通道：定义工厂指令，并在电流环里录制有界波形供分析。
图标：lock
规矩：工厂服务编号不是产品契约编号，不得作为普通产品语义暴露（service/factory/README.md，2026-11）
规矩：未知的 schema 或协议版本一律拒绝（service/factory/README.md，2026-11）
说明：它和产品契约分开：编号只在研发和产线内部用，主机连上时先核对版本，不认识的版本一律拒绝。波形录制器设计成每个电流环周期调用一次，先把数据录在芯片内存里，录完再整段取走，不会每个周期都往总线上发。目前它默认不编译进固件，录制器还没挂进电流环，和主机之间的通信也没接通；某个诊断值没接上来源时，会回答“未就绪”，而不是给一个 0。

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

```calls 录一段电流环波形
FactoryServiceInit | 清空诊断快照，按电流环频率准备好录制器；固件里还没有地方调用它，目前只有测试调用
  进 base_sample_rate_hz: 电流环每秒运行的次数
  FactoryTraceRecorderInit | 录制器清零，回到空闲
FactoryServiceTraceConfigure | 工厂主机选通道、分频和触发方式；主机通道尚未接通，目前由测试调用
  进 config: 通道、分频、触发条件
  FactoryServiceSignalSupported | 确认每个通道都是 v1 支持的信号
  FactoryTraceRecorderConfigure | 正在录时拒绝改配置，否则存下
FactoryServiceTraceArm | 开始等待触发
  FactoryTraceRecorderArm | 清空缓冲，换一个新的录制编号
    出 recorder->capture_id: 这次录制的编号
FactoryServiceTracePushFocIsr | 每个电流环周期调用一次，挑出所选通道的值；设计上挂在电流环计算之后，目前还没挂上
  进 source: 这一周期的电流参考、电流反馈、相电流和母线电压
  FactoryServiceSignalValue | 按信号编号取出对应的数
  FactoryTraceRecorderPushIsr | 按分频取样；触发前只保留设定长度的历史，触发后录满后段就冻结
    出 recorder->state: 等待、已触发或录完
FactoryServiceTraceReadSample | 录完后按时间顺序一条条取出样本
  进 logical_index: 第几条样本
  出 sample: 这一条的各通道数值
```

```calls 重新生成编号
tools/factory_service_codegen/generate.py:main | 构建目标或开发者运行，generate 重新生成，check 只比对
  tools/factory_service_codegen/generate.py:generate | 读服务定义，写出或比对两个生成物
    进 check: 为真时只比对
    出 stale: 已经过期的生成文件
    render_header | 把能力、诊断、波形信号和 CANopen 对象编成 C 编号宏
    render_canopen | 写出 CANopen 绑定清单
```

## 代码生成工具

从参数表和 EDS 生成参数数据库、MAVLink 消息和 CANopen 对象字典的工具。
图标：spark
说明：参数数据库、MAVLink 消息和对象字典这些代码很长，又要彼此一致，所以不手写，而是从参数表格和 EDS 文件生成。参数和消息由仓库里现成的 Windows 程序生成，仓库里没有这些程序的源码；CANopen 对象字典由一个脚本调用开源工具 objdictgen 生成，同时生成 CiA402 头文件和对象说明文档。生成出来的文件不要手改；CANopen 生成默认不参与编译，要在 CMake 里专门打开。

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

```calls 重新生成对象字典
tools/canopen_codegen/generate.py:main | 读生成配置，按命令分派；generate 依次做下面两步
  进 args.command: generate、check、list 等
  _generate_canopen | 由 EDS 生成 CanFestival 的对象字典 C 代码
    进 source: 现在作为源头的 EDS 文件
    _find_odg | 找 objdictgen 程序：环境变量、本地虚拟环境或系统路径
    _run | 以子进程运行 objdictgen 的转换命令，它内部做什么量不到
  _write_profile_outputs | 由 CiA402 配置写出头文件和对象说明文档
    _render_profile_header | 拼出 CiA402 对象的 C 头文件
    _render_profile_doc | 拼出对象字典说明文档
```

## 项目说明与配置

仓库说明、许可证和编辑器设置。
图标：note
说明：两份说明文件目前还是代码托管平台给的空白模板，没有写项目内容，想了解项目要看各目录自己的说明。许可证是 MIT。编辑器里的编译和调试设置假定仓库旁边有一套 MinGW 工具链，换电脑要先改路径。这份部件清单也放在这里。

- `README.md`
- `README.en.md`
- `LICENSE`
- `.gitignore`
- `.gitattributes`
- `.vscode/`
- `docs/`
