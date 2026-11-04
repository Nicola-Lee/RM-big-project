# 云台PID调试：先pitch，后yaw

## 配置与运行范围

共享配置在 `gimbal_pid_debug.hpp`：

```cpp
inline constexpr bool kEnabled = true;
inline constexpr Axis kAxis = Axis::Yaw;
inline constexpr bool kPitchHoldConfirmed = true;
inline constexpr PitchWaveform kPitchWaveform = PitchWaveform::Sine;
inline constexpr float kPitchSineCenterRad = 0.0f;
inline constexpr float kPitchSineAmplitudeRad = 0.34906585f; // 20度
inline constexpr uint32_t kPitchSinePeriodMs = 10000;
inline constexpr float kStepRad = 0.05235988f; // 阶跃模式/yaw测试的相对3度
inline constexpr YawWaveform kYawWaveform = YawWaveform::Sine;
inline constexpr float kYawSineAmplitudeRad = 1.04719755f; // 相对启动中心±60度
inline constexpr uint32_t kYawSinePeriodMs = 10000;
inline constexpr uint32_t kDurationMs = 30000;
```

`kEnabled` 与 `wheel_pid_debug.hpp` 的 `kWheelPidDebugEnabled` 不能同时为true，编译期会检查。
正常模式：两项均为false。单轮模式：仅单轮开关为true。云台模式：仅本开关为true。
改变轴、任一轴波形类型、正弦中心/幅值/周期、相对阶跃目标、时长、保持确认等共享实验配置后，重新编译并烧录两块板。仅修改原云台PID或云台补偿参数时，更新云台固件即可。

PID仍在 `Gimbal/Instance/component/ins_pid.cpp`，前馈、重力补偿、目标限速和模块转矩限幅仍在 `Gimbal/Instance/component/ins_robot.cpp`。
本接口不复制参数，不自动填写Kp，不改电机零点/方向，不改底盘原接收程序或Cube生成文件。
调pitch前在原pitch PID配置中填写正Kp；yaw PID可以继续为0。调yaw时两轴PID都需要满足原controlReady条件。原PID数值由用户整定，调试曲线不改变这些参数。

## 底盘与遥控链路

底盘UART接收、错误恢复、CAN1遥控转发继续运行。底盘调试分支每周期清除轮控制量并发送四轮零电流帧。
云台通过原CAN转发重组和DT7快照读取遥控器。新增0x218（云台命令）/0x219（底盘零输出确认），原0x210～0x217的格式不变。
新握手检查实验配置、序号、新鲜度和session；旧固件、只更新一块板或重复报文都不能完成本次运行许可。
底盘确认需要四轮反馈有效、四轮实际速度均不超过0.1rad/s和两路CAN健康，不要求底盘跟随PID就绪。云台单轴测试不依赖整车Working或IMU航向；原IMU采样和看门狗监督继续执行。

## 第一步：调pitch

1. 固定底盘并支撑pitch机构。右拨杆下档，左拨杆上档，四个摇杆通道和拨轮回中。
2. 满足配置与双板通信条件后，下档连续100ms开始准备。程序仅零转矩使能pitch，以获取真实反馈；yaw保持失能。
3. 等待云台调试快照state=2（Armed）。在5秒内把右拨杆拨到上档启动。准备完成后允许经过或短暂停留中档，期间保持零控制转矩；经过中档不会重置5秒期限。准备未完成时仍须保持下档。
4. 启动时捕获pitch当前位置作为模块内部起点；正弦请求按 `center + amplitude * sin(2π * elapsed / period)` 每1ms更新。当前以标定零点为中心，范围-20～+20度，周期10秒，30秒约3个周期。首周期仍捕获当前位置、零转矩，随后经原目标限速衔接到正弦请求。
5. 左拨杆离开上档、任一摇杆/拨轮离开中立、右拨杆离开上档、反馈丢失、总线故障、时序中断或达到时长，均结束输出。再次测试需重新下档准备。

启动位置与完整正弦范围都必须距已配置软件限位至少kPitchMarginRad（默认2度）；阶跃模式则检查启动位置及终点。运行中实际pitch进入这段边界区域也会停止。
角度使用原motor->angle()标定后的关节坐标，pitch向下为正；软件限位仍取原实车配置，代码不能判断机械限位是否已经实测。
下档准备、急停、正常结束或故障后均不提供保持转矩；pitch可能因重力下垂，整个测试过程需有适当机械支撑。不要用硬限位承接失能后的枪体。

## 第二步：调yaw，同时保持pitch

完成pitch在安全位置稳定保持的验证后，设置kAxis=Axis::Yaw及kPitchHoldConfirmed=true，再更新两板固件。
把pitch置于安全位置，下档准备、上档启动。本次启动位置会作为pitch保持角；不会直接追赶零角度。

程序先仅控制pitch。误差小于1度、实际速度小于0.05rad/s且输出未限幅，连续保持500ms后，才零转矩使能yaw。
取得yaw有效反馈后捕获yaw当前位置作为中心，从该时刻起按±60度正弦目标控制，程序计时10秒一周期；此时pitch继续使用原PID和重力补偿保持。YawWaveform::Step保留相对kStepRad阶跃。
pitch在5秒内无法稳定、保持误差超过5度、反馈失效或接近边界，会结束本次测试。yaw使能反馈等待上限3秒；30秒总时长从上档启动起计算，包含保持和使能阶段。
pitch保持确认是人为记录已完成整定，不替代运行中的误差、速度、反馈及限位检查。

## Ozone观察

在云台Global Data中找到g_gimbal_debug_snapshot后添加成员：

```cpp
(g_gimbal_debug_snapshot).state
(g_gimbal_debug_snapshot).reason
(g_gimbal_debug_snapshot).requested_rad
(g_gimbal_debug_snapshot).target_rad
(g_gimbal_debug_snapshot).actual_rad
(g_gimbal_debug_snapshot).error_rad
(g_gimbal_debug_snapshot).feedforward_nm
(g_gimbal_debug_snapshot).torque_nm
(g_gimbal_debug_snapshot).pitch_error_rad
(g_gimbal_debug_snapshot).output_limited
```

角度单位rad，转矩单位N*m。target_rad为原模块限速/限位后的目标，requested_rad为本次实验要求的关节角。
当前pitch实验使用绝对关节角正弦目标；`PitchWaveform::Step`保留原相对阶跃模式。初次整定可使用阶跃，正弦用于检查连续跟踪。pitch重力补偿使用原配置，两个系数仍需实测标定。

状态值：Waiting=0、Preparing=1、Armed=2、HoldingPitch=3、EnablingYaw=4、Running=5、Finished=6、Fault=7。
原因值：None=0、PreparationRequired=1、Configuration=2、RemoteOrSwitch=3、PeerOffline=4、Communication=5、Timing=6、MotorFault=7、FeedbackTimeout=8、PitchRange=9、PitchHold=10、Control=11、Released=12、Duration=13、PreparationTimeout=14、WheelMoving=15。
底盘同名快照只表示零输出握手，不代表云台已经开始运动。

云台另有g_gimbal_debug_trace和g_gimbal_debug_trace_count。每5ms采样，前639槽保留所选轴开始运动后的前约3.2秒，最后一槽保留停车零输出快照。它不是完整30秒记录；新实验会清计数。yaw测试的采样从yaw真正接入控制时开始。
调试中使用在线观察；需要检查缓存时，先结束输出，再检查样本。

## 修改范围

- 新增共享gimbal_pid_debug.hpp：可读配置、协议、纯状态机、诊断数据。
- 新增共享gimbal_pid_debug_port.hpp：短临界区、时序检查、非阻塞握手发送。
- 新增两板Task/component/gimbal_pid_debug.cpp：底盘零输出确认、云台测试与记录。
- 两板main_task.cpp/comm_task.cpp：在原初始化、1ms控制和通信入口增加互斥分支。
- 两板src/HW_can.cpp：分发新调试帧，原电机/遥控/正常协议分发继续保留。
- 云台gimbal.hpp/.cpp：新增选轴调试入口，与正常入口共用限速、PID、补偿、转矩限制及驱动输入路径；接入yaw时仅重置yaw历史，保留pitch保持历史。

编译与状态机检查不能代替实车方向、实际限位、保持能力和PID稳定性的验证。本接口不自动烧录或启动电机。

## ±20度正弦目标（2026-11-03）

本次只修改共享调试配置/目标计算、云台每周期目标更新和本说明。Yaw测试仍为相对阶跃，Pitch在Yaw测试期间保持捕获的固定安全角度。
这里的±20度相对原电机标定的pitch零点；不是相对每次启动姿态的±20度。当前零偏、方向、PID、补偿及±30度软件限位保持原配置。
10秒周期的峰值目标速度约0.2193rad/s，小于当前0.3rad/s限速。配置检查要求正弦峰值速度不超过原目标限速；过快周期会显示Configuration。
若启动姿态偏离正弦起点0度，内部目标会先经原限速衔接，开始一段时间可能不与requested_rad重合。
观察requested_rad（原始正弦）、target_rad（限速后的目标）、actual_rad（实际角度）、torque_nm和output_limited。实际能否达到±20度取决于实车负载与原PID/转矩限制，不自动增大输出限幅。
原640槽、5ms采样仍只保存开始后的约3.2秒及停车快照；在线观察可覆盖完整10秒周期，现有缓存不包含完整周期。

## 拨杆经过中档的修复（2026-11-04）

仅在Armed状态允许下档→中档→上档；上档启动前仍检查反馈、位置和双板握手。运行中离开上档仍立即停止，上电直接上档仍不能启动。本次更新了共享握手版本，两块板需同时更新固件。

## 准备阶段使能与停止快照修复（2026-11-04）

Preparing/Armed期间仅清零控制量，保持pitch零转矩使能以取得反馈；不会再每周期排队失能命令。
故障、结束及其他未使能状态继续失能，运行许可条件保持不变。
启动及发送前的反馈失效报FeedbackTimeout（8），位置不安全报PitchRange（9），启动握手失效报PeerOffline（4）。

云台额外提供g_gimbal_debug_stop_snapshot，进入Fault/Finished时保存一次。
停止后观察其state、reason、feedback_valid、pitch_actual_rad，区分真正越界与反馈失效。
它在下次进入停止状态时更新；重新准备不会清除上次停止记录。
本次没有更改实验配置、CAN帧或ConfigTag；两板已使用上一版拨杆修复固件时，只需更新云台。

## Yaw ±60度正弦测试（2026-11-04）

用户已确认完成pitch固定安全姿态保持验证，因此当前默认轴切为Yaw，kPitchHoldConfirmed=true。
目标为Wrap(本次yaw启动角+1.04719755*sin(2π*t/10))，t从yaw有效反馈建立、进入Running开始计时。
峰值目标速度约0.658rad/s，不超过当前yaw目标限速1rad/s；配置不匹配会禁止启动。
整次实验上限仍为上档后30秒，包含pitch保持和yaw使能等待，因此有效正弦时间可能不足3周期。
当前yaw、pitch PID和所有原限幅、补偿值保持用户配置，仍需实车整定。程序不自动烧录或启动。

跨±pi时绝对target_rad/actual_rad会回绕，画±60度曲线请使用：
```cpp
(g_gimbal_debug_snapshot).yaw_relative_target_rad
(g_gimbal_debug_snapshot).yaw_relative_actual_rad
(g_gimbal_debug_snapshot).yaw_center_rad
(g_gimbal_debug_snapshot).pitch_actual_rad
(g_gimbal_debug_snapshot).pitch_error_rad
(g_gimbal_debug_snapshot).torque_nm
(g_gimbal_debug_snapshot).output_limited
```
相对曲线单位仍为rad，峰值±1.0472rad；它只用于观察，控制仍使用原关节坐标、最短角误差和原限速。
pitch在yaw测试中保持捕获的固定角度。左拨杆上档、右拨杆下档准备，state=2后5秒内上档启动，state依次为3、4、5。
当前与旧固件的共享配置标签不同，两块板都需重新编译、烧录。
此前CSV时间轴上存在10秒配置对应约12秒周期的差异，本次未修改系统时基或Ozone时间轴。
