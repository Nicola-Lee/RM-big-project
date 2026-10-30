# 底盘遥控接收与云台转发

## 实车来源与流程

实测底盘 USART3 的 DT7 有效帧计数持续增加；云台 USART3 没有收到遥控数据。
因此当前硬件配置使用：接收机 → 底盘 USART3 循环 DMA → CAN1 三片转发 → 云台原 DT7 解码器 → 原控制/调试入口。
云台仍计算 yaw/pitch 目标角和云台坐标系速度，再通过原双板协议向底盘发送速度。底盘接收遥控本身不授予电机运行许可。

共享开关在 `remote_control_forward.hpp` 的 `kReceiverOnChassis = true`。它描述接收机物理位置，与 `wheel_pid_debug.hpp` 的调试开关独立；两板必须使用相同版本、相同配置重新编译和烧录。

## 接收与转发逻辑

- 保留底盘全局诊断名称 `chassis_remote_probe`，现在用于正常/调试两种固件。USART3 使用原 Cube 配置，128 字节循环 DMA。
- HT、TC、IDLE 处理新增字节；第一次 IDLE 对齐后，拿到完整且有效的 18 字节帧立即解码、发布，不等待缓冲区写满。
- 检查五个模拟通道、两个拨杆、鼠标按钮取值。错误帧撤销有效输入并重新等 IDLE 对齐。
- 底盘 CAN1 标准数据帧 `0x215/0x216/0x217`，每片 8 字节：源有效帧序号低 8 位、版本/在线/急停/源数据年龄、原始数据 6 字节。三片合起来保留原 18 字节，包括鼠标和键盘数据。
- 1ms 通信入口每次最多提交一片。邮箱忙时直接返回转发函数，保留待发片段；后续通信任务和喂狗监督继续运行。
- 一组发送完成后不反复发送相同源计数，重复 CAN 帧不会让遥控器持续在线。新源帧在下一组采用。
- 源接收故障或 100ms 未得到有效 UART 帧，撤销旧组并每 10ms 尝试发送离线声明。UART 恢复在原主循环中的前台轮询执行，不在中断里等待 DMA 停止。

## 云台校验与控制边界

- 三片须同序号、20ms 内完整到齐，且格式有效；完整到齐才调用原 `DT7::decode()`，更新原 `remote_valid_frame_count`。
- 年龄以 8ms 为单位向上取整，结合拼帧耗时检查 100ms 新鲜度；这是接收时效检查，实际停车延迟还包含 UART 和 CAN 传输/排队及原许可链的响应。
- 缺片、错片、重复片不刷新有效时间；错误版本、非法帧、离线声明使输入失效。
- 每片都有右拨杆下档标志，可先收紧急停；解除需要一组新的完整有效帧。离线声明之后，相同或更旧的滞留分片不能立即恢复输入。
- 只有选中的物理来源能改变遥控输入；当前云台不启动本地 USART3 遥控 DMA。原 UART 恢复代码保留供日后改变物理配置使用。
- 原 `0x210～0x214` 协议、遥控映射、PID、前馈、限幅、轮向、正常状态机及单轮调试握手保持原实现。本次不修改 Cube 生成文件或受保护头文件。

## Ozone 检查顺序

先重新烧录两块板，分别打开对应新 ELF。保持底盘四轮悬空、右拨杆下档、左拨杆上档、摇杆和拨轮回中，再连续运行观察。

底盘通过 Global Data 添加：

- `(chassis_remote_probe).valid_frames`：持续增加。
- `(chassis_remote_probe).forward_packets`：持续增加，表示一组的三片已提交邮箱，并非云台已确认收到。
- `(chassis_remote_probe).forward_offline`：接收离线时增加。
- `(chassis_remote_probe).uart_errors`：观察是否出现接收故障。

云台通过 Global Data 添加：

- `remote_valid_frame_count`：持续增加，证明云台收到并重组了遥控帧。
- `remote_forward_online`：遥控在线时为 1；断开有效输入后应变为 0。
- `remote_forward_emergency`：右拨杆下档时为 1，完整收到其他档位后可变为 0。
- `remote_forward_rejected`：正常应不持续增长。
- `(g_wheel_debug_snapshot).state`、`.reason`：观察原单轮调试握手。

这些在线诊断由原遥控快照接口同步刷新。云台 `remote_rx_started = 0` 现在是预期行为，不能用它判断 CAN 遥控在线。
若底盘转发计数增加、云台计数不变，再排查两板 CAN1 接线、终端电阻、波特率和 `runtime_monitor` 总线故障；不要继续改串口引脚。

当前实验开关和 PID 参数保持用户现有值：调试开启，ID1，目标 1rad/s，运行 3000ms，ID1 kp=100、ki=kd=0。握手具备条件后，下档保持至少 100ms，等待 Armed，再上档仅启动一次。源码中 Finished/Fault 的停止原因可能在后续 Stop 周期被覆盖为等待状态，是已有诊断显示问题，不用它单独判断停车是否生效。

## 本次修改位置

| 文件 | 修改内容 |
|---|---|
| `RobotComponents/component/remote_control_forward.hpp` | 新增两板共用的帧校验、三片编码、重组、新鲜度与急停逻辑 |
| `Chassis/Task/component/remote_probe.hpp` | 原探针升级为遥控接收入口，增加恢复/转发接口与统计 |
| `Chassis/Task/component/remote_probe.cpp` | 原 DMA 接收基础上增加前台错误恢复与非阻塞 CAN 转发 |
| `Chassis/Task/component/main_task.cpp` | 原 WatchdogPoll 中调用接收恢复轮询 |
| `Chassis/Task/component/comm_task.cpp` | 正常/调试模式均初始化接收，并在原周期调用转发 |
| `Gimbal/Task/component/comm_task.hpp` | 声明转发接收入口 |
| `Gimbal/Task/component/comm_task.cpp` | 重组后接原解码器/快照，当前硬件配置停用本地 UART 遥控来源 |
| `Gimbal/src/HW_can.cpp` | 原 CAN1 回调增加 `0x215～0x217` 分发 |
| `RobotComponents/component/remote_control_forward.md` | 本说明与操作检查清单 |

已通过两板正常/调试模式 ARM 编译链接和协议、UART 拼帧、单轮握手、运动学、原状态机、看门狗相关编译期断言。原链接器 RWX LOAD 段警告仍存在。尚未代用户烧录或验证实际转发/电机响应。
