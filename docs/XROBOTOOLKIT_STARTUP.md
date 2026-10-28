# XRoboToolkit 头显遥操作启动手册

本文对应 `./start_teleop.sh --input-backend xrobotoolkit`：头显经 USB `adb reverse` 连接工作站上的 XRoboToolkit PC Service，`xrobotoolkit_bridge` 把手柄数据转成 OpenArmX 兼容的 ROS 2 话题，teleop 节点求解 IK 并驱动双臂 NERO。

上游主要支持 PICO；若使用 Quest，必须使用已适配并验证过的 XRoboToolkit 客户端，不能直接假设 PICO APK 可用。

## 1. 启动顺序

1. 打开头显，唤醒两只手柄，用 USB 数据线连接工作站。
2. 在头显内允许 USB 调试，确认 `adb devices -l` 显示 `device`。
3. 启动工作站 Robotics Service。
4. 建立 `adb reverse tcp:63901 tcp:63901`。
5. 打开头显 XRoboToolkit，PC Service 地址填 `127.0.0.1`，开启 Head、Controller、Send。
6. 先 `--shadow` 核对，再 `--execute` 真机。
7. 结束时先让机器人停在合适位置，再 `Ctrl-C`，最后关闭头显 Send 和 PC Service。

## 2. 依赖检查

```bash
command -v adb
test -f /opt/ros/humble/setup.bash && echo 'ROS OK'
test -f .runtime/install/agx/setup.bash && echo 'driver overlay OK'      # scripts/build.sh
test -f /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so && echo 'XR SDK OK'
LD_LIBRARY_PATH=/opt/apps/roboticsservice/SDK/x64 "$NEROX_PYTHON" -c 'import xrobotoolkit_sdk' && echo 'pybind OK'
ip -details link show can_arm_a; ip -details link show can_arm_b
```

## 3. 连接头显

```bash
adb start-server
adb devices -l
export XRT_ADB_SERIAL='YOUR_HEADSET_SERIAL'
adb -s "$XRT_ADB_SERIAL" get-state     # 应输出 device
```

| 状态 | 处理 |
| --- | --- |
| `unauthorized` | 戴上头显接受 USB 调试授权 |
| `offline` | 唤醒头显、重新插拔 |
| 无设备 | 检查数据线（需支持数据传输）、开发者模式、USB 权限 |

## 4. 启动 PC Robotics Service

```bash
pgrep -af '[R]oboticsServiceProcess' || bash /opt/apps/roboticsservice/runService.sh
ss -ltnp 'sport = :63901'
```

需在工作站本地桌面会话启动；出现 Qt/display 错误时不要在纯 SSH 会话里运行。

## 5. ADB reverse

```bash
adb -s "$XRT_ADB_SERIAL" reverse tcp:63901 tcp:63901
adb -s "$XRT_ADB_SERIAL" reverse --list      # 应包含 tcp:63901 tcp:63901
```

```text
头显 XRoboToolkit -> 头显 127.0.0.1:63901 -> USB/adb reverse -> 工作站 127.0.0.1:63901 -> RoboticsServiceProcess
```

是头显主动连 PC，所以用 `reverse`，不能用 `forward`。USB 不转发局域网自动发现广播，头显里必须手动填写 `127.0.0.1`。拔插 USB 或重启头显/ADB 后要重新建立映射。

## 6. 头显内设置

打开 XRoboToolkit → PC Service 输入 `127.0.0.1`（若有端口项填 `63901`），连接状态显示 `WORKING`。

| 选项 | 设置 |
| --- | --- |
| Tracking / Head | 开启（bridge 用头显水平朝向做坐标归一化） |
| Tracking / Controller | 开启 |
| Data & Control / Send | 开启 |
| Switch w/ A Button | 建议关闭，避免误按暂停发送 |

`WORKING` 不代表在发数据，还要确认 Send 已开、手柄被追踪、数据 FPS 在更新。

## 7. Shadow 核对

```bash
./start_teleop.sh --shadow --input-backend xrobotoolkit
```

- bridge 打印 `[xrt-bridge] ready: ...` 表示已收到有效手柄位姿；
- teleop 每 5 s 打印 `mode=shadow quest=... active=... solves=N publishes=0`；
- 按住 Grip 时 `active` 出现对应侧，`solves` 递增。

## 8. 真机执行

确认机械臂固定、扫掠空间净空、急停可用，没有其他进程占用同一 CAN：

```bash
./start_teleop.sh --execute --input-backend xrobotoolkit
# 加夹爪：
./start_teleop.sh --execute --input-backend xrobotoolkit \
  --with-tacclaw --tacclaw-clearance-confirmed --tacclaw-speed 50
```

`--tacclaw-speed` 必须与板端服务认证返回的 `limits.speed` 完全一致，否则启动时报 safety-limit mismatch。

## 9. 手柄操作

### Grip：机械臂离合

- 首次使用先完全松开双侧 Grip 至少 0.25 s；接管阈值 0.9。
- 左 Grip → `arm_b`，右 Grip → `arm_a`。
- 按下建立相对锚点，松开停止该侧新命令；再次按下重新锚定。
- 输入中断会撤销 Grip 授权；恢复后必须先松开 ≥0.25 s 再按下。
- 避免操作中突然转头或大幅移动身体；重新定位前先松开 Grip。

Grip 是手臂离合，不是急停，也不影响夹爪。

### Trigger：夹爪

`Trigger=0` → 全开 `1000`，`Trigger=1` → 全闭 `0`。不依赖 Grip。

### B / Y：预设动作

| 按钮 | 行为 |
| --- | --- |
| 右手 B | 双臂回关节零位 |
| 左手 Y | hands-up：J4 = 90°，其余关节归零 |

无需 Grip，不做环境碰撞规划；动作中按任意 Grip 立即取消并切回手柄控制。到位判定为所有关节误差 ≤1° 且持续 5 个周期。初次连通测试不要按 B/Y。

## 10. 停止

1. 把机器人移到合适位置，放下夹持物，松开 Grip。
2. 在启动终端 `Ctrl-C`，等待 `[nerox-session] stopped`。退出不会自动回零；execute 模式会关闭命令门并失能两臂。
3. 关闭头显 Send，退出 XRoboToolkit。
4. `adb -s "$XRT_ADB_SERIAL" reverse --remove tcp:63901`
5. 按需关闭 PC Service，再按设备规程下电。

## 11. 常见故障

| 现象 | 检查 |
| --- | --- |
| 头显连不上 `127.0.0.1` | PC 63901 是否监听、reverse 是否存在、USB 是否接在运行服务的主机上 |
| `WORKING` 但 bridge 不 ready | Head/Controller/Send 是否开启、手柄是否唤醒、头显是否休眠 |
| bridge 报 `input_fault` 并反复 reinitialize | 头显发送中断或时间戳停滞；恢复后需重新松开/按下 Grip |
| 恢复连接后按 Grip 不跟随 | 双侧完全松开 ≥0.25 s 再按紧 |
| 驱动一直等待 feedback | CAN 接口状态、机械臂上电、`NEROX_ARM_A_CAN` / `NEROX_ARM_B_CAN` |
| 左右或方向不对 | 先 shadow 核对；用 `--input-yaw-deg` 校准水平朝向 |
| TacClaw disabled 告警 | token 路径与权限、板端服务、`--tacclaw-speed` 是否与板端一致；夹爪故障不会停止机械臂 |
