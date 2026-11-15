# 函数调用链

这些链写的是代码怎么一层层调用，和 `parts.md` 里的流程图分开维护。三类差异都用“分支/情况”处理，相同的那一段只写一次：

- **通信方式**：编译时由 `version.h` 里的 `MINOR_VERSION` 选定（CAN+CiA402、CAN+ENCOS、CANopen+CiA402、EtherCAT+CiA402、CANFD 自定义）；上位机 MAVLink 串口在所有版本里都有，电脑仿真走 ZeroMQ。
- **编码器**：编译时由 `servo_system_cfg.h` 引入的那张 `motor_parameter_*.h` 里的 `ENCODER1_TYPE_OPTION` / `ENCODER2_TYPE_OPTION` 选定，经 `encoder_data[]` 里的函数指针 `init` / `read` / `process` 调用。
- **控制模式**：运行时分三处走：`set_app` 切换应用层模式，`AppRun` 跑应用层模式，`ModeRun` 跑控制层模式。

一个周期里依次走的是：电流环中断（链 7），位置环中断（链 4、5、6），1ms 任务（链 8）。外部指令（链 2、3）不直接动控制变量，只写参数数据库，由下一个位置环周期读走。

## 1. 上电初始化

从芯片或电脑程序启动，到控制环开始被定时触发为止。板子和电脑仿真的入口不同；通信外设按编译版本初始化。

```calls
分支 运行平台 | 板子烧 Y01 工程，电脑上编 SIL_project；两者共用控制核心，入口各写一份
  情况 Y01 板子 | STM32H743 上电后从 CubeMX 生成的 main 进入
    embedded_project/Y01/Core/Src/main.c:main | 依次初始化时钟和外设，登记回调，再初始化控制核心和板级驱动；之后所有工作都在中断里做，主循环为空。EtherCAT 版本例外：最后调 `ecat_main`（SSC 生成，初始化 LAN9252 和协议栈后在里面死循环跑 `MainLoop`，见链 2），不再回到 main。
      RegisterSetAppCallback | 把应用调度的 `set_app` 登记给参数数据库，以后切换运行模式经这个回调换应用；不登记的话 `set_app_Modes_of_operation` 会返回越界错误。
        进 callback: 要登记的函数，这里是 `set_app` | main 传入
        出 callback_registry.set_app_callback: 切换应用的回调 | 链 3 里 `set_app_Modes_of_operation` 调用
      RegisterSendCallback | 登记 MAVLink 回包用的发送函数，板子上是 `mavlink_send_data`（串口 DMA 发送，开了 CAN 透传时改走 CANFD 分包）。
      HalInit | 初始化控制核心：载入参数默认值、按所选电机参数表填好 `kAxis`、清空应用表、初始化状态检查、参数存储、示波器、抱闸和运动前置条件。
        展开 1
      BspInit | 启动板上外设：抱闸合闸，等母线电压稳定，校准 ADC，开 1ms 定时器、电流采样注入中断和 PWM（先关输出），再按版本初始化通信。位置环定时器这里不启动，等第一次电流环中断（链 7）再启动。
        embedded_project/Y01/Board/bsp_api.c:sys_bus_voltage_check | 循环等母线电压稳定；欠压超时会置母线电压错误。
        分支 通信方式 | `version.h` 里的 `MINOR_VERSION` 定出 `USE_CAN` / `USE_CANOPEN` / `USE_ECAT`
          情况 CAN 系列（CAN+CiA402、ENCOS、CANopen、CANFD 自定义） | 定义了 `USE_CAN`
            bsp_fdcan_init | 配置 FDCAN 外设和滤波器；失败则置通信初始化错误，不再初始化 CAN 应用。
            fdcan_app_init | 取得 FDCAN 设备句柄；CANopen 版本还会启动 CANopen 定时器、设节点号并进入 Operational。
          情况 EtherCAT | 定义了 `USE_ECAT`
            EcatTaskInit | 把 SYNC0 和 PDI 两根外部中断线配成软件可触发的中断，供 LAN9252 同步和过程数据使用。
        HAL_UART_Receive_DMA | 所有版本都打开上位机串口的 DMA 接收和空闲中断，收到一段字节后进链 2。
          进 pData: 接收缓冲 `mavlink_rx_buff` | 链 2 `HOST_UART_IRQ_TASK` 从这里取字节
  情况 电脑仿真 | Windows 上的 SIL 程序
    SIL_project/main.c:main | 同样登记回调、初始化控制核心，再连上 ZeroMQ，开三个 1ms 定时器；按回车退出。
      RegisterSetAppCallback | 同板子：登记 `set_app`。
      AppParamInit | 载入参数默认值。
      MotorCtrlInit | 按电机参数表填好 `kAxis`，并配好虚拟电机参数。
      AppInit | 把应用表里每个模式设为未启用、空闲。
      ZmqHandlerInit | 在本机 5555 端口建立 ZeroMQ 连接；失败则打印错误并退出程序。
      RegisterSendCallback | 登记 `ZmqSendData`，回包走 ZeroMQ。
      RegisterRecvCallback | 登记 `MavlinkRecvCallback`，`ZmqAsyncRecv` 收到字节后经它解包（链 2）。
      CreateHighPrecisionTimer | 分别建电机控制（链 4、7）、1ms 任务（链 8）、ZeroMQ 接收（链 2）三个定时器。
      StartHighPrecisionTimer | 以 1ms 周期启动定时器，回调在 Windows 定时器线程里运行。
```

## 2. 外部来的一帧报文：从中断到协议回调

上位机或总线主站发来的数据，从接收中断一直到落进参数数据库的写函数。不同通信方式的前半段各不相同；CiA402 对象之后的部分在链 2b，切换模式之后的部分在链 3。

```calls
分支 通信方式 | 上位机 MAVLink 所有版本都有；总线一侧由 `MINOR_VERSION` 定死一种
  情况 上位机 MAVLink | 调试软件读写几乎所有参数和控制环变量；三条线最后都交给同一个解包函数
    分支 字节从哪条线来 | 板子默认走串口；电脑仿真走 ZeroMQ；`IS_USE_CAN_PASSTHROUGH` 打开时走 CANFD 透传
      情况 串口 | 板子上的默认通道
        HOST_UART_IRQ_TASK | 串口空闲中断：停 DMA，把这一段字节交给解包，再重启 DMA；溢出错误时清标志并重启接收。
          出 mavlink_rx_buff: 这次收到的字节 | 交给 `MavlinkRecvCallback`
      情况 电脑仿真 ZeroMQ | SIL 程序的接收定时器每 1ms 调一次
        ZmqAsyncRecv | 非阻塞地从 ZeroMQ 取一段字节，有数据就经登记的接收回调（函数指针）交给 `MavlinkRecvCallback`。
          进 args: 轴数据 `kAxis` | 定时器创建时传入
          进 args_dw: 轴内部状态 `kAxisDw` | 定时器创建时传入
      情况 CAN 透传 | 默认关闭，打开后上位机报文改走 CANFD 的 FIFO1
        CAN_FDCAN_RX_FIFO1_CALLBACK | FDCAN 的 FIFO1 收到新消息时进来。
          进 hfdcan: 收到消息的 FDCAN 句柄 | HAL 中断传入
          进 RxFifo1ITs: 中断标志，只处理“新消息” | HAL 中断传入
          fdcan_app_fifo1_handle | 校验句柄后从 FIFO1 取出消息；句柄不对就置通信初始化错误返回。
            进 hfdcan: FDCAN 句柄 | `CAN_FDCAN_RX_FIFO1_CALLBACK`
            fdcan_passthrough_handle | 把 DLC 换成字节数；不带加速的帧当作 MAVLink 字节交给解包。
              进 bsp_fdcan: FDCAN 设备，含这一帧 | `fdcan_app_fifo1_handle`
    MavlinkRecvCallback | 逐字节拼 MAVLink 消息，拼成一条就停；连续失败超过阈值就清空通道重来。读请求把当前值打包回发；写消息在“通信控制权”属于上位机时才写入（增益、前馈等调试量除外），再回一份当前值。
      进 axis: 轴数据 | 中断或定时器传入 `kAxis`
      进 rx_data: 收到的字节 | 串口 DMA 缓冲、ZeroMQ 或 CAN 帧
      进 len: 字节数，超过缓冲区大小会被截断 | 调用方算出
      出 axis: 写消息直接改轴数据里的控制环变量 | 下一个控制环周期生效
      mavlink_parse_char | MAVLink 库的逐字节解析器，拼成完整且校验通过的一条消息时返回真。
        进 c: 一个字节 | `rx_data[i]`
        出 返回: 拼成一条完整消息时为真 | 消息存在 `msg` 里
      set_app_Controlword | 上位机写 `AppControlWord` 消息时调用：0 失能、1 使能、2 急停、3 清错；直接写进 `motor_enable`。
        进 val: 控制字 0..3，超出范围被拒绝 | 消息字段 `Controlword`
        出 axis->motor_ctl_sm_input.motor_enable: 控制层使能指令 | 链 6 `PosSpeedCtlLoopTask` 按上升沿处理
      set_app_Modes_of_operation | 上位机写 `AppOpMode` 消息时调用，切换运行模式，后续见链 3。
        进 val: 内部模式编号 | 消息字段或 `set_Modes_of_operation`
      set_app_Target_position | 上位机写 `AppMotionParam` 消息时调用，记下目标位置，等位置模式下一周期读取。
        进 val: 目标位置，负载端用户单位 | 消息字段 `Target_position`
  情况 CAN 总线 | 定义了 `USE_CAN` 的版本；FIFO0 收到的帧按编译进来的那一种协议处理
    CAN_FDCAN_RX_FIFO0_CALLBACK | FDCAN 的 FIFO0 收到新消息时进来。
      进 hfdcan: 收到消息的 FDCAN 句柄 | HAL 中断传入
      进 RxFifo0ITs: 中断标志，只处理“新消息” | HAL 中断传入
      fdcan_app_fifo0_handle | 校验句柄后从 FIFO0 取出消息，按编译版本交给对应协议；取消息失败直接返回。
        进 hfdcan: FDCAN 句柄 | 上面传入
        出 drv_can_app.p_fdcan: 里面的 `rx_header`、`rx_data` 是这一帧 | 交给下面的协议处理函数
        分支 CAN 上跑的协议 | `#if USE_CANOPEN / USE_ENCOS / USE_CUSTOM`，都没有时是 CAN+CiA402
          情况 CAN + CiA402 | `V_CAN_CIA402`，`version.h` 的默认版本；项目自定的简单帧格式
            fdcan_cia402_handle | 只收 ID 等于本机的帧；第 0 字节是读/写，1-2 字节是对象索引，3 是子索引，后面 1/2/4 字节是数据。读写完把结果原样格式回发。
              进 bsp_fdcan: FDCAN 设备，含这一帧 | `fdcan_app_fifo0_handle`
              出 bsp_fdcan->mg_counts: 收到本机报文的计数 | 上位机可读
              master_cia402_write_callback | 写请求交给 CiA402 对象分发，见链 2b。
              master_cia402_read_callback | 读请求按对象索引取值，填进回发帧。
                进 index: 对象索引 | 帧的第 1-2 字节
                出 data: 读出的值 | 填进回发帧
          情况 CANopen | `V_CANOPEN_CIA402`；CanFestival 协议栈
            fdcan_canopen_hendle | 把 FDCAN 帧转成 CanFestival 的 `Message` 交给协议栈；收到 SYNC（0x080）且节点在 Operational 时置“目标更新”标志，供同步位置模式取新目标。
              进 bsp_fdcan: FDCAN 设备，含这一帧 | `fdcan_app_fifo0_handle`
              canDispatch | CanFestival 按 COB-ID 分给 NMT、SYNC、PDO、SDO 等处理函数。
                进 d: 本机对象字典 `SlaveOD_Data` | 生成器生成
                进 m: 这一帧 CAN 消息 | `fdcan_canopen_hendle`
                proceedSDO | 处理 SDO 读写；一次传输完成后把数据写进对象字典。
                  进 m: SDO 帧 | `canDispatch`
                  SDOlineToObjdict | 把 SDO 线路里收齐的数据写进对象。
                    进 line: 收齐数据的 SDO 线路号 | `proceedSDO`
                    _setODentry | 查对象、检查权限和长度；对 0x2000 以上的对象，经函数指针 `OD_Write_Callback` 调用项目回调，用完即清空。
                      进 wIndex: 对象索引 | SDO 传输记录
                      进 pSourceData: 要写入的数据 | SDO 传输记录
                      出 返回: 中止码，0 为成功 | 回给主站
                      SlaveOD_Write_Callback | 由 `SlaveOD_scanIndexOD` 临时挂上的回调，直接转给 CiA402 对象分发，见链 2b。
                        进 index: 对象索引 | `_setODentry`
                        进 data: 要写入的数据 | `_setODentry`
                        master_cia402_write_callback | 见链 2b。
                proceedPDO | 处理 RPDO：按映射直接拷进对象字典变量，不经过项目回调。
              set_app_Target_update_state | 收到 SYNC 且节点在 Operational 时置真。
                进 val: 目标更新标志 | 这里传 true
                出 kAppMotionInfo.Target_update_state: 有新同步目标 | 链 5 `CspModeRun` 读取并清掉
          情况 CAN + ENCOS | `V_CAN_ENCOS`；协议还没实现
            fdcan_encos_handle | 目前是空函数，收到的帧被丢弃。
          情况 CANFD 自定义 | `V_CANFD_CUSTOM`；示例协议
            fdcan_custom_handle | 把收到的数据用本机 ID 原样回发，经典帧和 FD 帧分别用对应的发送函数。
  情况 EtherCAT | `V_ECAT_CIA402`；LAN9252 + SSC 协议栈。邮箱里的 SDO 在 `ecat_main` 的死循环里处理；过程数据映射 `APPL_OutputMapping` 仍是 SSC 生成的空模板
    MainLoop | SSC 协议栈主循环，由 `ecat_main` 反复调用。
      ECAT_Main | 检查 AL 状态和邮箱事件，有邮箱数据就处理。
        MBX_Main | 取出收到的邮箱报文并分给对应服务。
          MailboxServiceInd | 按邮箱类型分发，CoE 交给 `COE_ServiceInd`。
            进 pMbx: 收到的邮箱报文 | `MBX_Main`
            COE_ServiceInd | CoE 服务分发，SDO 请求交给 SDO 服务端。
              进 pCoeMbx: CoE 报文 | `MailboxServiceInd`
              SDOS_SdoInd | 解析 SDO 请求，下载（写）时调 `OBJ_Write`。
                进 pSdoInd: SDO 请求 | `COE_ServiceInd`
                OBJ_Write | 找到对象条目，经条目里的函数指针 `pObjEntry->Write` 调项目写回调。
                  进 index: 对象索引 | `SDOS_SdoInd`
                  进 pData: 主站写来的数据 | `SDOS_SdoInd`
                  出 返回: 中止码，0 为成功 | SDO 应答
                  communication/callback/ethercat/ecat_callback.c:EtherCAT_Write_0x6040_Callback | 每个可写对象一个这样的回调（这里以控制字为例）。上位机握有控制权时直接忽略；否则跳过 `master_cia402_write_callback`，直接调对应的 `OD_..._Write_Callback`，见链 2b。
                    进 subindex: 子索引 | SDO 请求
                    进 pData: 主站写来的数据 | SDO 请求
```

## 2b. CiA402 对象写入：控制字、运行模式、目标值

CAN+CiA402、CANopen 两种版本都从 `master_cia402_write_callback` 进来；EtherCAT 从各自的 `EtherCAT_Write_..._Callback` 直接进到下面的 `OD_..._Write_Callback`。主站写的是标准单位的对象，这里换算后写进参数数据库。

```calls
master_cia402_write_callback | 按对象索引分给每个对象的写回调。通信控制权在上位机时一律拒绝（返回“状态不允许”）；不支持写的索引返回“只读”。
  进 index: 对象索引，如 0x6040 | CAN 帧或 CANopen SDO
  进 subindex: 子索引 | 同上
  进 data: 主站写来的值 | 同上
  出 返回: CANopen 中止码，0 为成功 | 回给主站
  分支 写的是哪个对象 | `switch (index)`；下面列出驱动电机最关键的几个，其余对象同样是“OD 回调 → set_ → set_app_”三步
    情况 0x6040 控制字 | 主站让驱动器上电、使能、快停、复位故障
      OD_0x6040_Controlword_Write_Callback | 生成的对象回调，把值交给对象变量的写函数。
        进 value: 16 位控制字 | 主站
        set_Controlword | 存下控制字，立刻让 CiA402 状态机按它走一步。
          进 val: 控制字 | 上面传入
          出 kPDS_FSA.Controlword: 当前控制字 | `ControlwordChange_PDAS_FSA`、`PDS_FSA_Run` 读取
          ControlwordChange_PDAS_FSA | 按控制字的位组合算出状态机下一状态并更新状态字；bit8 写进暂停指令；bit4 上升沿通知轮廓位置模式“新目标生效”。真正改使能的是之后每周期的 `PDS_FSA_Run`（链 5）。
            set_app_Halt_running_cmd | 控制字 bit8：暂停运动。
              进 val: 1 暂停 | 控制字 bit8
            ControlwordBit4UpedgeCheck | 检测 bit4 上升沿。
              set_app_start_upedge_state | 告诉 `PpModeRun` 下一周期取新目标位置。
                进 state: 有上升沿时为真 | `ControlwordBit4UpedgeCheck`
                出 kPpMode.start_upedge_state: 新目标待取 | `PpModeRun` 下一周期
    情况 0x6060 运行模式 | 主站切换 PP/PV/PT/HM/CSP/CSV/CST/MIT
      OD_0x6060_Modes_of_operation_Write_Callback | 生成的对象回调。
        进 value: CiA402 模式编号，如 1=PP、8=CSP | 主站
        set_Modes_of_operation | 把 CiA402 模式编号翻成项目内部的模式编号。
          进 val: CiA402 模式编号 | 上面传入
          set_app_Modes_of_operation | 切换运行模式，后续见链 3。
            进 val: 内部模式编号 | 消息字段或 `set_Modes_of_operation`
    情况 0x607A 目标位置 | 位置类模式的目标
      OD_0x607A_Target_position_Write_Callback | 生成的对象回调。
        进 value: 主站写来的值 | `master_cia402_write_callback` 或 EtherCAT 写回调
        set_Target_position | 存下对象值并写进参数数据库。
          进 val: 目标位置，负载端脉冲 | 主站
          set_app_Target_position | 记下目标位置；PP 模式在 bit4 上升沿取用，CSP 模式在“目标更新”时取用（链 5）。
    情况 0x60FF 目标速度 | 速度类模式的目标
      OD_0x60FF_Target_velocity_Write_Callback | 生成的对象回调。
        进 value: 主站写来的值 | `master_cia402_write_callback` 或 EtherCAT 写回调
        set_Target_velocity | 把 P/s 换成 rpm 后写进参数数据库。
          进 val: 目标速度，负载端 P/s | 主站
          set_app_Target_velocity | 记下目标速度，PV/CSV 模式下一周期读取。
            进 val: 目标速度，rpm | `set_Target_velocity` 换算后
    情况 0x6071 目标力矩 | 力矩类模式的目标
      OD_0x6071_Target_torque_Write_Callback | 生成的对象回调。
        进 value: 主站写来的值 | `master_cia402_write_callback` 或 EtherCAT 写回调
        set_Target_torque | 把“额定电流的千分比”换成峰值电流安培后写进参数数据库。
          进 val: 目标力矩，额定值的千分比 | 主站
          set_app_Target_torque | 记下目标电流，PT/CST 模式下一周期读取。
            进 val: 目标电流，峰值安培 | `set_Target_torque` 换算后
```

## 3. 切换运行模式

上位机（链 2）或主站（链 2b）改运行模式时走这里。它同时定下两件事：控制层用位置、速度、力矩还是辨识环路（`motor_ctl_sm_config.mode`），以及应用层跑哪个应用（`set_app`）。

```calls
set_app_Modes_of_operation | 新模式和当前显示的模式相同就直接返回；否则记下新模式，按模式写控制层模式，再经回调换应用。回调没登记时返回越界错误；不认识的模式把控制层置为空闲。
  进 val: 内部模式编号（`MOTOR_CTL_SM_MODE_*`） | MAVLink 或 `set_Modes_of_operation`
  出 kAppOpMode.Modes_of_operation: 当前请求的运行模式 | 上位机和主站读回
  出 axis->motor_ctl_sm_config.mode: 控制层环路模式 | 链 6 `ModeRun` 按它分支
  set_app | 经 `callback_registry.set_app_callback` 调用（函数指针，上电时登记）。先停掉另一个正在跑的运动类应用，再调新应用的初始化：成功进 READY，失败进 ERROR；最后启用它。
    进 name: 要切换到的应用 | 下面各情况
    出 kAppInfo: 应用表里新应用 enable=true、状态 READY | 链 5 `AppRun` 下一周期启动它
    AppStop | 停掉正在运行的另一个运动类应用，调它的 stop 并置空闲。同一时间只留一个运动类应用。
      进 name: 要停的应用 | `set_app` 找到的另一个非空闲运动类应用
    分支 运行模式 | `val` 决定应用名和控制层模式；新应用的初始化经 `app_table` 里的函数指针 `init` 调用
      情况 PP 轮廓位置 | 应用 `APP_PP_MODE`，控制层位置模式
        PpModeInit | 清掉上升沿锁存，初始化位置规划器。
      情况 PV 轮廓速度 | 应用 `APP_PV_MODE`，控制层速度模式
        PvModeInit | 初始化速度规划器。
      情况 PT 轮廓力矩 | 应用 `APP_PT_MODE`，控制层力矩模式
        PtModeInit | 初始化力矩规划器，从当前 q 轴电流开始。
      情况 MIT | 应用 `APP_MIT_MODE`，控制层 MIT 模式
        MitModeInit | 初始化 MIT 目标。
      情况 HM 回零 | 应用 `APP_HM_MODE`，控制层用速度模式
        HomingModeInit | 初始化回零状态。
      情况 CSP 同步位置 | 应用 `APP_CSP_MODE`，控制层位置模式
        CspModeInit | 初始化插补规划器。
      情况 CSV 同步速度 | 应用 `APP_CSV_MODE`，控制层速度模式
        CsvModeInit | 初始化插补规划器。
      情况 CST 同步力矩 | 应用 `APP_CST_MODE`，控制层力矩模式
        CstModeInit | 初始化插补规划器。
      情况 方向辨识 | 应用 `APP_DIR_ID_MODE`，控制层方向辨识
        IdDirModeInit | 准备辨识电机相序方向。
      情况 电角度辨识 | 应用 `APP_ELEC_ANGLE_ID_MODE`
        IdElecAngleModeInit | 准备辨识电角度偏移。
      情况 电气参数辨识 | 应用 `APP_ELEC_ID_MODE`
        IdElecModeInit | 准备辨识电阻电感。
      情况 机械参数辨识 | 应用 `APP_MEC_ID_MODE`
        IdMecModeInit | 准备辨识惯量和阻尼。
      情况 极对数辨识 | 应用 `APP_POLE_PAIRS_ID_MODE`
        IdPolePairsModeInit | 准备辨识极对数。
      情况 力矩脉动与摩擦辨识 | 应用 `APP_TQ_FC_ID_MODE`
        IdTqFcModeInit | 准备辨识力矩脉动和摩擦。
```

## 4. 位置环中断：自检与读编码器

板子上位置环定时器约 10kHz 触发一次（第一次电流环中断之后才启动）；电脑仿真里由 1ms 定时器连跑 10 次代替。这一段负责把编码器读数准备好，然后进入链 5。

```calls
分支 运行平台 | 板子有真编码器；电脑仿真的位置直接由虚拟电机给出
  情况 Y01 板子 | 位置环定时器中断
    POSITION_LOOP_TIM_IRQ_TASK | 清中断标志、记录周期；EtherCAT 版本先数位置环周期，在同步帧对应的那一拍置“目标更新”。自检没完成时先初始化编码器并跑自检；之后每周期读、解析编码器，更新 PWM 就绪状态；自检完成才进位置环。
      出 kBspData.cs.pl_period_index: EtherCAT 同步周期内的位置环序号，仅 EtherCAT 版本 | 和 `latch_pl_period_index` 比较决定何时更新目标
      set_app_Target_update_state | 仅 EtherCAT 版本：到了同步对应的那一拍，告诉 CSP/CSV/CST 取新目标。
      EncoderDataInit | 只在自检完成前每周期调用：对两路编码器各调一次 `init` 函数指针。
        分支 编码器型号 | 电机参数表里的 `ENCODER1/2_TYPE_OPTION`，编译时填进 `encoder_data[].init`
          情况 增量 ABZ | 如 acm80 电机；只能接一端
            ABZ_Encoder_Init | 把定时器配成编码器模式、计数上限设为分辨率，开 Z 相捕获中断。
              进 enc_data: 这一路编码器的数据和配置 `encoder_data[id]` | `EncoderDataInit` / `EncoderDataRead` / `EncoderDataProcess` 循环传入
          情况 多摩川 RS485 | 如 wb40st（当前选用）、brt38m 的电机端
            TAMAGAWA_Encoder_Init | 设好帧长和请求命令；出过错就重置串口 DMA，防止帧错位。
              进 enc_data: 这一路编码器的数据和配置 `encoder_data[id]` | `EncoderDataInit` / `EncoderDataRead` / `EncoderDataProcess` 循环传入
          情况 MT68xx SPI | 如 gim6010
            MT68XX_Encoder_Init | 准备“读角度寄存器”的 SPI 发送帧。
              进 enc_data: 这一路编码器的数据和配置 `encoder_data[id]` | `EncoderDataInit` / `EncoderDataRead` / `EncoderDataProcess` 循环传入
          情况 KTM59xx SPI | 如 ktm59xx
            KTM59XX_Encoder_Init | 准备 SPI 请求帧。
              进 enc_data: 这一路编码器的数据和配置 `encoder_data[id]` | `EncoderDataInit` / `EncoderDataRead` / `EncoderDataProcess` 循环传入
          情况 BiSS-C | 如 smc40s、brt38m 的负载端
            BISSC_Encoder_Init | 准备 BiSS-C 时钟帧。
              进 enc_data: 这一路编码器的数据和配置 `encoder_data[id]` | `EncoderDataInit` / `EncoderDataRead` / `EncoderDataProcess` 循环传入
          情况 无编码器 | 第二路（负载端）没接时
            Encoder_None | 只把类型记为无。
              进 enc_data: 这一路编码器的数据和配置 `encoder_data[id]` | `EncoderDataInit` / `EncoderDataRead` / `EncoderDataProcess` 循环传入
      HardwareSelfTestRun | 只在自检完成前调用：两路编码器都就绪就读出单圈和多圈值初始化控制层状态，自检完成；超时则用 0 初始化并置编码器错误。
        出 sys_msg.hardware_self_test_status: 自检完成标志 | 本函数调用方和电流环中断据此决定跑不跑控制环
        embedded_project/Y01/Board/bsp_api.c:bsp_get_encoder_state | 查这一路编码器有没有收到有效数据。
          进 enc_id: 电机端或负载端 | `HardwareSelfTestRun`
          出 返回: 是否就绪 | 两路都就绪才算自检完成
        MotorCtlSmStateInit | 用编码器初值初始化位置传感器和速度观测器，使上电位置连续。
        AppStatusScanFast | 自检期间也扫描快速状态（如限位）。
      EncoderDataRead | 把上周期 DMA 收到的原始字节转存，再对每一路调 `read` 函数指针发起下一次读取，结果下周期才用。
        分支 编码器型号 | 同上，经 `encoder_data[].read`
          情况 增量 ABZ
            ABZ_Encoder_Data_Read | 直接读定时器计数。
              进 enc_data: 这一路编码器 | `EncoderDataRead` 循环传入
              出 enc_data->err_cnt: 启动传输失败时加一 | 连续出错会置编码器错误
          情况 多摩川 RS485
            TAMAGAWA_Encoder_Data_Read | 启动串口 DMA 接收并发出请求命令；启动失败计错误次数。
              进 enc_data: 这一路编码器 | `EncoderDataRead` 循环传入
              出 enc_data->err_cnt: 启动传输失败时加一 | 连续出错会置编码器错误
          情况 MT68xx SPI
            MT68XX_Encoder_Data_Read | 启动 SPI DMA 收发。
              进 enc_data: 这一路编码器 | `EncoderDataRead` 循环传入
              出 enc_data->err_cnt: 启动传输失败时加一 | 连续出错会置编码器错误
          情况 KTM59xx SPI
            KTM59XX_Encoder_Data_Read | 启动 SPI DMA 收发。
              进 enc_data: 这一路编码器 | `EncoderDataRead` 循环传入
              出 enc_data->err_cnt: 启动传输失败时加一 | 连续出错会置编码器错误
          情况 BiSS-C
            BISSC_Encoder_Data_Read | 启动 SPI DMA 收发。
              进 enc_data: 这一路编码器 | `EncoderDataRead` 循环传入
              出 enc_data->err_cnt: 启动传输失败时加一 | 连续出错会置编码器错误
          情况 无编码器
            Encoder_None | 什么也不做。
              进 enc_data: 这一路编码器的数据和配置 `encoder_data[id]` | `EncoderDataInit` / `EncoderDataRead` / `EncoderDataProcess` 循环传入
      EncoderDataProcess | 对每一路调 `process` 函数指针按协议解析，再统一处理多圈溢出、方向取反、负载端单圈当电机多圈的选项，最后分配给电机端和负载端，并检测过零。
        出 encoder_data: 电机端和负载端的单圈值 `single_cnt`、多圈值 `multi_turns` | 链 5 `bsp_get_encoder_cnt` 读取
        分支 编码器型号 | 同上，经 `encoder_data[].process`；校验失败连续多次会置编码器错误
          情况 增量 ABZ
            ABZ_Encoder_Data_Process | 检查两次 Z 相之间的计数偏差，太大计错；正常时计数即单圈值。
              进 enc_data: 这一路编码器，`data_raw` 是上周期收到的原始字节 | `EncoderDataProcess` 循环传入
              出 enc_data->a_single_raw: 解析出的单圈值，编码器计数 | `EncoderDataProcess` 再分给电机端或负载端
          情况 多摩川 RS485
            TAMAGAWA_Encoder_Data_Process | 异或校验和命令回显；出错计数，满了置错并重启串口 DMA；状态字段报警也置错；正常时拼出 24 位单圈值。
              进 enc_data: 这一路编码器，`data_raw` 是上周期收到的原始字节 | `EncoderDataProcess` 循环传入
              出 enc_data->a_single_raw: 解析出的单圈值，编码器计数 | `EncoderDataProcess` 再分给电机端或负载端
          情况 MT68xx SPI
            MT68XX_Encoder_Data_Process | CRC8 校验后拼出 21 位角度和状态位。
              进 enc_data: 这一路编码器，`data_raw` 是上周期收到的原始字节 | `EncoderDataProcess` 循环传入
              出 enc_data->a_single_raw: 解析出的单圈值，编码器计数 | `EncoderDataProcess` 再分给电机端或负载端
          情况 KTM59xx SPI
            KTM59XX_Encoder_Data_Process | CRC8 校验后取出角度和状态位。
              进 enc_data: 这一路编码器，`data_raw` 是上周期收到的原始字节 | `EncoderDataProcess` 循环传入
              出 enc_data->a_single_raw: 解析出的单圈值，编码器计数 | `EncoderDataProcess` 再分给电机端或负载端
              KTM59XX_Encoder_Crc8_Check | KTM 专用 CRC8。
                进 input: 整帧数据 | `KTM59XX_Encoder_Data_Process`
                出 返回: 校验是否通过 | 不通过计错误次数
          情况 BiSS-C
            BISSC_Encoder_Data_Process | 取出位置、错误位和警告位；CRC6 校验目前注释掉了。
              进 enc_data: 这一路编码器，`data_raw` 是上周期收到的原始字节 | `EncoderDataProcess` 循环传入
              出 enc_data->a_single_raw: 解析出的单圈值，编码器计数 | `EncoderDataProcess` 再分给电机端或负载端
          情况 无编码器
            Encoder_None | 什么也不做，负载端值记为 0。
              进 enc_data: 这一路编码器的数据和配置 `encoder_data[id]` | `EncoderDataInit` / `EncoderDataRead` / `EncoderDataProcess` 循环传入
        detect_abs_encoder_zero_crossing | 绝对式编码器检测单圈值有没有跨过零点。
      bsp_pwm_ready_state_updata | 根据 PWM 开关后经过的周期数更新“PWM 已就绪”。
      PosSpeedLoopCtrl | 自检完成后才调用，见链 5。
  情况 电脑仿真 | 电机控制定时器每 1ms 回调一次
    MotorCtrlTimerCallback | 一次回调里连跑 20 次电流环和虚拟电机，每两次跑一次位置环，模拟 20kHz/10kHz；没有编码器读取和自检。
      PosSpeedLoopCtrl | 见链 5。
```

## 5. 位置环：应用层

`PosSpeedLoopCtrl` 是位置环每周期的总入口：取编码器位置，跑应用层（故障检查、CiA402 状态机、当前运动模式），再交给控制层（链 6），最后按控制层状态开关 PWM、同步参数、采示波器。

```calls
PosSpeedLoopCtrl | 位置环每周期一次。控制权属于应用层时才跑应用层；控制层算完后：状态为使能且电流校准完成，就在“失能→使能”那一拍打开 PWM，校准没完成就继续校准；其他状态一律关 PWM。
  出 kAxis.motor_pos_sensor_input.enc_counts_now_p: 电机端单圈计数 | 链 6 `motor_pos_sensor`
  出 kAxis.load_pos_sensor_input.enc_counts_now_p: 负载端单圈计数 | 链 6 `load_pos_sensor`
  出 kAxis.motor_ctl_sm_input.hw_ready_state: PWM 已开且电流校准完成时为 1 | 下一周期 `AppRun` 和链 6 `ModeRun` 据此决定跑不跑
  分支 运行平台 | `bsp_get_encoder_cnt` 由哪份硬件实现提供
    情况 Y01 板子
      embedded_project/Y01/Board/bsp_api.c:bsp_get_encoder_cnt | 取链 4 解析好的单圈值。
        进 enc_id: 电机端或负载端 | `PosSpeedLoopCtrl`
        get_encoder_cnt | 返回 `encoder_data[enc_id].single_cnt`。
    情况 电脑仿真
      SIL_project/bsp_api_virtual.c:bsp_get_encoder_cnt | 从虚拟电机输出里取编码器计数。
        进 enc_id: 电机端或负载端 | `PosSpeedLoopCtrl`
  get_app_Internal_control_authority | 内部控制权：属于应用层时才跑下面的应用层；切到控制层直控（上位机调试控制层）时整段跳过。
  AppStatusCheck | 检查过流、过压、过温、跟随误差等应用层故障；有故障就把控制字改成失能。
    展开 1
  AppStatusScanFast | 扫描限位、到位等快速状态位。
  AppMotionPreConditionRun | 检查运动前置条件（抱闸、限位等）。
  Cia402ErrorCodeScan | 仅 CiA402 版本：把应用层故障按优先级翻成标准错误码。
  PDS_FSA_Run | 仅 CiA402 版本：有故障就转进故障状态，并按状态机当前状态写应用层控制字（`set_app_Controlword`），这是主站使能/失能真正生效的地方。
    展开 1
  AppRun | 遍历应用表，对启用的那个应用：READY 时调 start，成功进 RUNNING；RUNNING 时，硬件就绪才调 run，未就绪退回 READY；返回成功或失败就调 stop 并停用。start/run/stop 都经 `app_table` 里的函数指针调用。
    进 hw_ready_state: 上一周期得出的硬件就绪状态 | `kAxis.motor_ctl_sm_input.hw_ready_state`
    出 kAppInfo: 各应用的状态和结果 | 下一周期继续用
    分支 运动模式 | 链 3 `set_app` 启用的那个应用；这里只列每周期的 run
      情况 PP 轮廓位置 | 目标在控制字 bit4 上升沿（或上位机控制时每周期）取用
        PpModeRun | 使能时：取新目标，按有无负载端编码器和减速比换成电机端目标，暂停或抱闸未松时规划速度为 0；急停时按快停选项减速，零速后失能；失能时不规划。
          出 kPpMode.traj: 目标位置和速度、加减速 | `PosTrajectoryPlanningHandle`
          PosTrajectoryPlanningHandle | 梯形速度规划，算出这一周期的位置指令和前馈。
            进 pos_traj_data: 目标位置、轮廓速度、加减速 | `PpModeRun`
            出 axis->input_shaping_input.pos_cmd_p: 这一周期的位置指令 | 链 6 `PositionStep`
            出 axis->pos_speed_ctl_input.speed_tar_p_s: 速度前馈 | 链 6
          AppPositionTargetReachedStateCheck | 判断是否到位，更新状态位。
      情况 PV 轮廓速度
        PvModeRun | 按目标速度和加速度规划速度；急停和失能处理同 PP。
          SpeedTrajectoryPlanningHandle | 线性规划，算出这一周期的速度指令。
            出 axis->pos_speed_ctl_input.speed_tar_p_s: 速度指令 | 链 6 `SpeedStep`
      情况 PT 轮廓力矩
        PtModeRun | 按目标力矩和斜率规划电流。
          TorqueTrajectoryPlanningHandle | 线性规划，算出这一周期的 q 轴电流指令。
            出 axis->current_ctl_input.idq_tar_A: q 轴电流指令 | 链 7 电流环
      情况 MIT
        MitModeRun | 把位置、速度、力矩目标和两个增益交给控制层的 MIT 控制器。
          MitTrajectoryPlanningHandle | 直接写 MIT 控制器的输入和增益。
            出 axis->mit_ctl_input: 位置、速度、力矩目标 | 链 6 `MitCtlStep`
      情况 HM 回零
        HomingModeRun | 按回零方式跑回零过程，控制层用速度模式。
          SetMethodRun | 按回零方式选具体流程。
            进 method: 回零方式 | 参数数据库里的回零方式
      情况 CSP 同步位置 | 目标随同步帧更新
        CspModeRun | “目标更新”为真且抱闸已松时取新目标，换成电机端位置推进插补缓冲；急停和失能处理同 PP。
          set_app_Target_update_state | 取用新目标后清为假，等下一次同步（CANopen SYNC 或 EtherCAT 同步那一拍）再置真。
            进 val: 这里传 false | `CspModeRun`
          CspTrajectoryPlanningHandle | 在两次同步之间插补位置。
            出 axis->input_shaping_input.pos_cmd_p: 这一周期的位置指令 | 链 6 `PositionStep`
          AppPositionTargetReachedStateCheck | 判断是否到位。
      情况 CSV 同步速度
        CsvModeRun | 同步取新目标速度并插补。
          CsvTrajectoryPlanningHandle | 在两次同步之间插补速度。
            进 csv_traj_data: 目标速度 | `CsvModeRun`
            出 axis->pos_speed_ctl_input.speed_tar_p_s: 速度指令 | 链 6 `SpeedStep`
      情况 CST 同步力矩
        CstModeRun | 同步取新目标力矩并插补。
          CstTrajectoryPlanningHandle | 在两次同步之间插补电流。
            进 cst_traj_data: 目标电流 | `CstModeRun`
            出 axis->current_ctl_input.idq_tar_A: q 轴电流指令 | 链 7 电流环
      情况 方向辨识
        IdDirModeRun | 看控制层的方向辨识是否做完，做完返回成功并停下。
      情况 电角度辨识
        IdElecAngleModeRun | 看控制层的电角度辨识是否做完。
      情况 电气参数辨识
        IdElecModeRun | 看控制层的电气参数辨识是否做完。
      情况 机械参数辨识
        IdMecModeRun | 看控制层的机械参数辨识是否做完。
      情况 极对数辨识
        IdPolePairsModeRun | 看控制层的极对数辨识是否做完。
      情况 力矩脉动与摩擦辨识
        IdTqFcModeRun | 看控制层的力矩脉动与摩擦辨识是否做完。
  UpdateStatusword | 仅 CiA402 版本：按状态机状态和当前模式更新状态字，供主站读取。
  PosSpeedCtlLoopTask | 控制层，见链 6。
  embedded_project/Y01/Board/bsp_api.c:bsp_set_pwm_state | 失能到使能的那一拍开 PWM；不是使能状态（含故障）就关 PWM。电脑仿真用 `SIL_project/bsp_api_virtual.c` 里的同名函数，只记下开关状态。
    进 state: 开或关 | `PosSpeedLoopCtrl`
  embedded_project/Y01/Board/bsp_api.c:sys_current_calibration_step | 电脑仿真里的同名函数直接算校准通过。板子上：使能了但电流校准还没完成时，每周期走一步校准（50% 占空比采 1024 次取零点）；电流偏差过大或母线不稳会置错并关 PWM。
  AppParamSyncUser | 把控制层的实时变量同步进参数数据库，供上位机、主站和示波器读取。
  ScopeIsrHandle | 软件示波器按采样间隔取一次数据，等触发、采够点数。
```

## 6. 位置环：控制层

由 `PosSpeedLoopCtrl` 每周期调用一次。先把传感器读数变成位置和速度，再做保护检查，最后按控制层状态机和 `motor_ctl_sm_config.mode` 跑一步控制算法，写出电流环的目标。逐行笔记见 `calls-notes.md`。

```calls
PosSpeedCtlLoopTask | 不论什么状态都先更新位置速度和做保护；有故障直接进故障态并只等清错（0→3 上升沿）；收到失能立刻失能并清零输出；失能时等使能上升沿（0→1）做使能前初始化后转使能；使能且硬件就绪才跑当前模式。它只改状态，不开关 PWM。
  进 axis: 一根轴的全部控制变量 | `kAxis`
  进 axis->motor_ctl_sm_input.motor_enable: 使能指令 0 失能、1 使能、3 清错 | `set_app_Controlword`（链 2、链 5 `PDS_FSA_Run`）
  出 axis->motor_ctl_sm_output.state: 失能、使能或故障 | 链 5 `PosSpeedLoopCtrl` 据此开关 PWM
  出 axis->current_ctl_input: 电流环的目标和模式 | 链 7 电流环
  motor_pos_sensor | 生成代码：电机端编码器计数换成多圈绝对位置和电角度。
    出 enc_sum_p: 多圈绝对位置 | 写进 `pos_speed_ctl_input.pos_now_p`
  load_pos_sensor | 生成代码：负载端编码器同样处理。
  speed_obs_pll | 生成代码：锁相环观测器由位置估出速度。
  protect_check | 生成代码：对照限值检查电流、母线电压、温度、跟随误差和速度，有问题写错误码。
  ModeCheckInInit | 失能到使能那一拍调用：按模式复位辨识、往复、参考信号等的内部状态并把当前位置记作起点；位置、速度、力矩、MIT 模式不做事。
    进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
    进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
  ModeRun | 使能且硬件就绪时调用，按控制层模式跑一步。
    进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
    进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
    分支 控制层模式 | `axis->motor_ctl_sm_config.mode`，由链 3 `set_app_Modes_of_operation` 设定；回零和 CSV 用速度、CSP 用位置、CST 用力矩
      情况 位置 | PP、CSP
        PositionStep | 位置指令先做输入整形抑制末端振动，再跑位置速度环，最后做力矩补偿和电流指令滤波。
          进 axis->input_shaping_input.pos_cmd_p: 位置指令 | 链 5 规划器
          出 axis->pos_speed_ctl_output.iq_tar_A: q 轴电流目标 | 滤波后写给电流环
          input_shaping_ei | 生成代码：输入整形。
          pos_speed_ctl_pip | 生成代码：位置速度 PI/P 调节和扰动观测。
          TqFcComStep | 叠加力矩脉动与摩擦补偿。
            进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
            进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          CommandCurrentFilter | 陷波滤波后写入电流环的 q 轴目标。
            进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
            进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
      情况 速度 | PV、CSV、回零
        SpeedStep | 位置目标设成当前位置（不积累位置误差），按速度指令跑位置速度环，再做补偿和滤波。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          pos_speed_ctl_pip | 生成代码：速度模式下只跑速度环。
          TqFcComStep | 力矩补偿。
            进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
            进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          CommandCurrentFilter | 电流指令滤波。
            进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
            进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
      情况 力矩 | PT、CST
        TorqueStep | 不跑速度位置环：q 轴电流目标由应用层直接给，这里只填电角度和电角速度，并把电流环切到电流控制。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
      情况 MIT
        MitCtlStep | 用当前位置和速度跑 MIT 阻抗控制，结果作为 q 轴电流目标，再做补偿和滤波。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          mit_ctl | 生成代码：kp·位置误差 + kd·速度误差 + 前馈力矩。
          TqFcComStep | 力矩补偿。
            进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
            进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          CommandCurrentFilter | 电流指令滤波。
            进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
            进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
      情况 电压直控 | 调试用
        VoltageStep | 清零电流目标，电流环按给定电压直接输出。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
      情况 方向辨识
        DirectionIdStep | 跑方向辨识，做完自动更新相序方向。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          direction_id | 生成代码。
      情况 极对数辨识
        PolePairsIdStep | 跑极对数辨识，做完更新极对数。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          pole_pairs_id | 生成代码。
      情况 电角度辨识
        ElecAngleIdStep | 跑电角度辨识，做完更新电角度偏移。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          展开 1
      情况 电气参数辨识
        ElectricIdStep | 正弦注入辨识电阻电感。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          展开 1
      情况 机械参数辨识
        MecIdStep | chirp 或阶跃激励辨识惯量和阻尼。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          展开 1
      情况 力矩脉动与摩擦辨识
        TqFcIdStep | 速度环下采集电流辨识力矩脉动和摩擦。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
          展开 1
      情况 往复运动 | 调试用
        ReciprocalMotionStep | 生成往复位置指令并走位置环。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
      情况 参考信号 | 调试用
        ReferenceSignalStep | 按配置注入参考信号到电流、速度或位置环。
          进 axis: 轴数据 `kAxis` | `PosSpeedCtlLoopTask` 一路传下来
          进 axis_dw: 生成算法的内部状态 `kAxisDw` | 同上
      情况 空闲 | 模式未设置或不认识
```

## 7. 电流环中断

板子上 ADC 注入采样完成触发，约 20kHz；电脑仿真里由 1ms 定时器连跑 20 次。读电流、算电压、出 PWM。

```calls
分支 运行平台 | 入口和硬件实现不同，中间的控制算法相同
  情况 Y01 板子 | ADC 注入转换完成中断
    CURRENT_LOOP_IRQ_TASK | 只处理 U 相 ADC；自检完成且电流校准通过才跑电流环；第一次进来时启动位置环定时器（链 4 从此开始）。
      进 hadc: 触发中断的 ADC | HAL
      出 kBspData.pl_start_state: 位置环已启动 | 只启动一次
      embedded_project/Y01/Board/bsp_api.c:sys_get_hardware_self_test_status | 链 4 的自检是否完成。
      embedded_project/Y01/Board/bsp_api.c:sys_get_current_calibration_status | 电流零点校准是否完成。
      CurrentLoopCtrl | 见下。
      SimPlantStep | 仅定义了 `VIRTUAL_MOTOR_MODEL` 时：板上也跑虚拟电机，用于不接电机调试。
  情况 电脑仿真 | 电机控制定时器
    MotorCtrlTimerCallback | 每 1ms 连跑 20 次电流环和虚拟电机。
      CurrentLoopCtrl | 见下。
      SimPlantStep | 跑一步虚拟电机，给出电流和编码器读数。
CurrentLoopCtrl | 两个平台共用：读三相电流，跑电流环，输出三相电压。
  出 kAxis.current_ctl_input.iabc_now_A: 三相电流，安培 | `CurrentCtlLoopTask`，以及链 6 `protect_check`
  分支 读电流的硬件实现 | 板子与电脑各有一份 `bsp_get_phase_current`
    情况 Y01 板子
      embedded_project/Y01/Board/bsp_api.c:bsp_get_phase_current | ADC 读数减校准零点，换成安培。
        出 piabc: 三相电流 | `CurrentLoopCtrl`
    情况 电脑仿真
      SIL_project/bsp_api_virtual.c:bsp_get_phase_current | 从虚拟电机输出取电流。
        出 piabc: 三相电流 | `CurrentLoopCtrl`
  CurrentCtlLoopTask | 调生成的电流环算法。
    进 axis: 含电流目标、电角度、电流环模式 | 链 6 写入
    出 axis->current_ctl_output.uabc_tar_comp_V: 补偿后的三相目标电压 | `bsp_set_phase_voltage`
    current_ctl_loop_task | 生成代码：矢量控制，电流控制或电压直控两种模式。
  分支 出电压的硬件实现 | 板子与电脑各有一份 `bsp_set_phase_voltage`
    情况 Y01 板子
      embedded_project/Y01/Board/bsp_api.c:bsp_set_phase_voltage | 按母线电压把三相电压换成 PWM 占空比写进定时器。
        进 voltage: 三相目标电压，伏 | `CurrentLoopCtrl`
    情况 电脑仿真
      SIL_project/bsp_api_virtual.c:bsp_set_phase_voltage | 把电压写给虚拟电机输入。
        进 voltage: 三相目标电压，伏 | `CurrentLoopCtrl`
```

## 8. 1ms 慢速任务与协议定时器

板子上几个定时器共用一个中断入口，按定时器区分；电脑仿真只有 1ms 任务。

```calls
分支 运行平台
  情况 Y01 板子 | 定时器更新中断
    NRT_CAN_ECAT_IRQ_TASK | 按是哪个定时器触发分开处理。
      进 htim: 触发中断的定时器 | HAL
      分支 哪个定时器 | `htim->Instance`；CANopen 和 EtherCAT 定时器只在对应版本里有
        情况 1ms 任务定时器 | 所有版本
          UnrealTimeBase1ms | 见下。
        情况 CANopen 定时器 | `USE_CANOPEN`
          CANopen_DispatchFromISR | 清零计时，调 CanFestival 的定时调度（心跳、PDO 事件定时、SDO 超时）。
            TimeDispatch | CanFestival 定时器调度。
        情况 EtherCAT 定时器 | `USE_ECAT`
          ECAT_CheckTimer | SSC 协议栈的 1ms 计时：看门狗、DC 同步检查。
  情况 电脑仿真
    UnrealTimeCallback | 1ms 定时器回调。
      UnrealTimeBase1ms | 见下。
UnrealTimeBase1ms | 两个平台共用：滤波更新母线电压和各处温度，然后依次做慢速状态扫描、抱闸、指示灯、参数存储；正式版（`YH_RELEASE`）还推进固件升级。
  出 kAxis.current_ctl_input.dc_bus_now_V: 滤波后的母线电压 | 链 6 `protect_check`、链 7 电压换算
  出 kAxis.current_ctl_input.driver_temp: 滤波后的驱动器温度 | 链 6 `protect_check`
  展开 1
```
