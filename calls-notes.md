## motor_control/motor_ctl_sm.c:PosSpeedCtlLoopTask

每个位置环周期由 `PosSpeedLoopCtrl` 调一次。顺序固定：先把传感器读数变成位置和速度，再做保护检查，最后按状态机决定这一周期是清零输出、做使能前初始化，还是交给 `ModeRun` 跑当前模式。它自己不开关 PWM，只改 `motor_ctl_sm_output.state`；PWM 由调用方按这个状态去开关。

**一、读位置和速度（不论什么状态都跑）**

- `motor_pos_sensor(&axis->motor_pos_sensor_input, ...)`：处理电机端编码器。输入的单圈计数 `enc_counts_now_p` 是调用方刚用 `bsp_get_encoder_cnt(ENCODER_ID_MOTOR)` 填进去的，输出里的 `enc_sum_p` 是累计的多圈绝对位置。
- `axis->pos_speed_ctl_input.pos_now_p = axis->motor_pos_sensor_output.enc_sum_p;`：从这一行起，这一周期的“当前位置”才确定，后面的速度观测、保护和各模式都用它。
- `load_pos_sensor(...)`：同样处理负载端编码器。这里没有接着用它的输出，留给后面 `ModeRun` 里需要负载端位置的模式。
- `axis->speed_obs_pll_input.pos_now_p = axis->pos_speed_ctl_input.pos_now_p;` 然后 `speed_obs_pll(...)`：速度不是直接测的，是锁相环观测器从位置估出来的。
- `axis->pos_speed_ctl_input.speed_now_rad_s = axis->speed_obs_pll_output.ev_rad_s;`：估出的速度，单位 rad/s，写进位置速度环的输入。

传感器处理放在状态机前面，所以失能、甚至错误状态下位置和速度也一直在更新，上位机看到的位置不会停住，使能那一刻也不会有跳变。

**二、保护检查**

- `protect_check(&axis->current_ctl_input.iabc_now_A[0], ..., &error_code);`：拿三相电流、母线电压、驱动器温度、位置跟随误差和刚估出的速度，对照 `motor_ctl_sm_config` 里的限值检查，有问题就把错误码写进局部变量 `error_code`（开头初始化为 0）。
- 其中电流、电压、温度是电流环那边采样放进 `current_ctl_input` 的；`pos_ctl_error_p` 是**上一个周期**位置环算出的跟随误差，因为这一周期的模式还没跑。
- `if (error_code != MOTOR_CTL_SM_NONE_ERROR)`：一旦有错，不管原来是什么状态，直接改成 `MOTOR_CTL_SM_STATE_ERROR`，并记下错误码 `motor_ctl_sm_output.error`。

**三、错误状态：只等“清除错误”**

- `if (axis->motor_ctl_sm_output.state == MOTOR_CTL_SM_STATE_ERROR)`：这个块最后无条件 `return;`，所以错误状态下后面的失能处理和 `ModeRun` 都不会执行。电流环的目标值不在这里清零；电机停下来靠的是调用方看到状态不是使能，就 `bsp_set_pwm_state(PWM_DISABLE)` 关掉 PWM。
- `motor_enable == MOTOR_CTL_SM_MOTOR_CLEAR_ERROR && motor_enable_last == MOTOR_CTL_SM_MOTOR_DISABLE`：清除错误要的是指令从 0 变成 3 的那个“上升沿”，一直保持 3 不算。所以主站要先发 0 再发 3。
- 清除时做三件事：跟随误差 `pos_ctl_error_p` 归零（否则下一周期保护检查又会因为旧误差报错）、状态回到 `MOTOR_CTL_SM_STATE_DISABLE`、错误码归零。注意回到的是失能，不是使能，要转起来还得再发一次使能。
- `return` 之前先写 `motor_enable_last`：因为提前返回会跳过函数末尾那一行，这里不补上，上升沿就没法判断了。

**四、失能指令优先**

- `if (axis->motor_ctl_sm_input.motor_enable == MOTOR_CTL_SM_MOTOR_DISABLE)`：只要不在错误状态，收到 0 就马上进失能，不等当前模式走完。其他指令值（比如 2 紧急停车）在这里不处理，状态保持不变，交给应用层和模式自己去处理。

**五、失能状态：清零输出**

- 第一个 `if (state == MOTOR_CTL_SM_STATE_DISABLE)`：把电流环切到电压直接控制（`current_ctl_input.mode = 0`），dq 电流目标和三相电压目标全部写 0，跟随误差归零。这样即使 PWM 还开着，也只输出零电压；跟随误差归零是为了下次使能时保护检查不会拿旧误差误报。

**六、失能到使能的上升沿**

- 第二个 `if (state == MOTOR_CTL_SM_STATE_DISABLE)` 条件和上一个完全一样，只是拆开写：上一个块负责清零，这一个块负责等使能。
- `motor_enable == MOTOR_CTL_SM_MOTOR_ENABLE && motor_enable_last == MOTOR_CTL_SM_MOTOR_DISABLE`：同样只认 0→1 的上升沿。如果失能时指令一直停在 1（比如正转着被保护打断、清错后又发了 1），不会自己再使能，要先发 0 再发 1。
- `ModeCheckInInit(axis, axis_dw);`：按当前模式 `motor_ctl_sm_config.mode` 做使能前的初始化。位置、速度、力矩、MIT 模式这里是空的；各种辨识、往复运动、参考信号模式会复位自己的内部状态，并把当前位置 `enc_sum_p` 记作起点，所以使能那一刻不会往旧起点跳。
- 然后状态改成 `MOTOR_CTL_SM_STATE_ENABLE`。

**七、使能状态：跑当前模式**

- `if (axis->motor_ctl_sm_input.hw_ready_state)`：硬件就绪才调 `ModeRun`。`hw_ready_state` 由调用方在本函数返回**之后**更新（PWM 已打开且电流校准完成才为 1）。
- 所以刚使能的那个周期，`hw_ready_state` 还是上一周期的 0，`ModeRun` 不会跑；调用方看到状态从失能变成使能，这时才打开 PWM；下一周期 `hw_ready_state` 变成 1，模式才真正开始跑。中间这一周期的输出就是第五步清零的结果。
- 如果使能后硬件变成不就绪，`ModeRun` 不再跑，但 `current_ctl_input` 里不会被清零，保留的是最后一次 `ModeRun` 写进去的值。
- `ModeRun(axis, axis_dw);`：按 `motor_ctl_sm_config.mode` 用 `switch` 分到 `PositionStep`、`SpeedStep`、`TorqueStep` 或各种辨识模式，最后写出电流环的目标。

**八、记住这一周期的指令**

- `axis->motor_ctl_sm_output.motor_enable_last = axis->motor_ctl_sm_input.motor_enable;`：给下一周期判断上升沿用。错误状态走不到这里，所以第三步里单独写了一次。
