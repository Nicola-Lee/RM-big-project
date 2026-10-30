# 单轮 PID 调试接口

此接口默认关闭，开关在同目录 `wheel_pid_debug.hpp`。正常固件仍走原整车状态机。调试固件只允许选中底盘轮运行，其他三轮输入置零，yaw/pitch 持续下发失能；主循环、TIM6 1ms 入口与集中喂狗保持原方式。

## 开启与操作

1. 在 `Chassis/Instance/component/ins_pid.cpp` 的原参数数组内填写选中轮的 PID。四轮目前增益均为零，至少选中轮 kp 必须为正；接口不代填增益。其余轮 PID 尚未整定不会阻止本次单轮实验。
2. 修改共享头文件 `wheel_pid_debug.hpp`：`kWheelPidDebugEnabled = true`；`kMotorId` 选 1～4；`kTargetRadS` 默认 1.0（轮子输出轴 rad/s）；`kDurationMs` 默认 3000（允许 1～30000ms）。改变这些实验配置后必须重新编译、烧录两块板。
3. 因为新增了 cpp 文件，首次在两板工程分别重新运行 CMake 配置，再构建。不要修改 CubeMX 生成文件。头文件开关不是遥控器可切换的运行状态。
4. 将底盘可靠垫高、四轮离地。左拨杆上档；右拨杆下档；左右摇杆、拨轮回中，保持至少 100ms，并等待双板调试快照 `state = Armed`。上电时右拨杆已经在上档不会启动。
5. 将右拨杆拨到上档，运行一次。离开上档、移动任一摇杆或拨轮、左拨杆离开上档、遥控掉线、选中轮反馈过期、通信故障、控制周期中断或达到运行时长，都会结束/禁止这次实验。再试必须重新下档准备、再上档触发；持续上档不会重复运行。
6. 测试结束，改回 `kWheelPidDebugEnabled = false`，重新编译并烧录两板，恢复整车功能。

实际停止响应受遥控器帧、CAN 排队和接收超时影响。遥控/双板/电机反馈使用现有量级的 100ms 时效检查；“零电流停车”不等于机械瞬间停转。保持底盘悬空，不以机械碰撞测试保护。

## 参数与生产控制一致

- 四个 PID 对象仍来自 `GetPidMotorWheel()`，参数仍在原 `ins_pid.cpp`。积分限幅、总输出限幅没有调试专用副本。
- 轮速上限与四个速度前馈系数仍在 `Chassis/Instance/component/ins_robot.cpp` 的 `GetChassis()` 中配置。接口使用同一个 `Chassis` 对象。
- 正常路径为遥控指令、坐标旋转/麦轮逆解、轮目标速度；单轮调试在轮目标速度处接入，跳过整车运动学与整车加速度规划。它用于辨识单轮速度环，并不是整车运动曲线测试。
- 两种路径共用 `Chassis::applyWheelTargets()`：限幅后的轮目标速度乘原前馈系数，送入原 PID，总输出限幅后再经原电机 `setInput()` 硬限幅。
- 输出轴速度已经应用电机方向与减速比，不再除减速比。
- 调纯 PID 时，在原配置中把该轮前馈系数置零；当前四个前馈系数已经为零。加入前馈之后调试和正常模式同时使用新值。
- 零目标与正常全零速度命令一致：清 PID 历史并输出零电流，不执行闭环制动。非选中三轮每周期清历史、写零输入。
- 不要求无关的云台位置环、其他轮 PID 或 IMU Ready；IMU 前台采样与原看门狗监督继续运行。两路 CAN 健康要求仍保留，不能通过断开某一路 CAN 来绕过它。

## 双板隔离与握手

`0x213` 为云台调试命令，`0x214` 为底盘应答。原 `0x210～0x212` 格式和意义未修改。正常固件不处理调试帧；只给一块板刷调试固件无法完成握手。

8 字节调试帧包含 32 位实验配置摘要、16 位报文序号、8 位实验 session、8 位阶段/状态。摘要校验实验参数及协议版本；PID/前馈/限幅在底盘原配置中，云台不另存一份。重复帧不刷新时效，旧运行帧不能用于超时重同步。底盘必须先观察 Stop，再 Prepare，再同 session 的 Run。测试到期或故障后，继续收到 Run 不会重启。

运行许可帧与轮电流帧仍通过 `runtime::TrySend()`，撤销许可会请求取消尚在邮箱中的运动帧。邮箱忙只跳过本次发送，控制/通信进度记录继续执行，喂狗逻辑未改。

## 调试器观察

两块板都可直接观察 `robot::wheel_debug::g_wheel_debug_snapshot`，也提供 `GetSnapshot()` 临界区副本接口。无需在 1ms 中断内打印。

底盘快照包括 `motor_id`、`session`、`state`、`reason`、`elapsed_ms`、`target_rad_s`、`actual_rad_s`、`error_rad_s`、`feedforward`、`pid_output`、`raw_current`、`speed_limited`、`output_limited`、`feedback_valid`。

`pid_output` 是 PID 函数返回的、已经包含前馈和总限幅的值；`raw_current` 是电机硬限幅后的实际输入码。云台快照只提供握手/触发状态与时长，轮速数据应在底盘查看。

状态数字：Waiting=0、Armed=1、Running=2、Finished=3、Fault=4。原因枚举定义在共享头文件：Configuration 需检查选中轮 PID/限幅和两板实验配置；Feedback 需检查选中轮 CAN 反馈；PeerOffline 需检查另一块板固件和通信；Communication 查看原 runtime 诊断。

底盘另有 `g_wheel_debug_trace` 环形样本数组：运行中每 5ms 保存一个快照，停止时额外保存零输出快照；640 个槽覆盖默认 3 秒。`g_wheel_debug_trace_count` 是有效样本数，`g_wheel_debug_trace_head` 是下一写入位置。未写满时从 0 读取；写满后从 head 读取并循环到头。每次新实验清计数，停止后保留数据。更长实验保留末段样本。

不要通过单步/断点停住正在输出的控制任务来观察曲线；使用在线变量观察，或先停止实验再检查缓存。

## 修改位置

- 新增共享 `wheel_pid_debug.hpp`：默认开关、实验输入、编解码、时效与独立调试状态机。
- 新增共享 `wheel_pid_debug_port.hpp`：短临界区、非阻塞发送、周期检查。
- 新增两板 `Task/component/wheel_pid_debug.cpp`：底盘执行/采样，云台遥控触发/关节失能。
- 两板 `main_task.cpp`：调试初始化/控制分支；两板 `comm_task.cpp`：调试通信分支，保留任务完成记录。
- 两板 `src/HW_can.cpp`：仅调试模式接收分发新增 CAN ID。
- 底盘 `chassis.hpp/.cpp`：单轮入口与共享轮控制函数，正常麦轮逆解与全零停车行为不变。
- 共享 `pid.hpp`：只增加 `outputLimit()` 只读诊断接口，不改算法或参数。

未修改 CubeMX 文件、用户列出的受保护头文件、原 PID 数值、原前馈/限幅数值和电机方向/减速比。

## 验证边界

两板正常/调试模式均通过 ARM 编译与链接。纯状态机、协议时效、双板时序模拟及原运动学/状态机/看门狗检查使用 ARM 编译器的 constexpr/static_assert 验证。它们不能替代实车验证；尚未烧录、测量实际轮向、整定 PID 或验证负载下响应。已有链接器 RWX LOAD 段警告仍存在。
