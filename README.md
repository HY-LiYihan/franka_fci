# franka_fci

面向调用方的局域网请求示例见 [调用说明.md](调用说明.md)。

基于 `pylibfranka` 的 Franka **FR3** 机器人控制服务。项目的目标是在 FR3 所在的 Linux 工控机上运行一个控制进程，同时通过局域网向经过授权的客户端提供状态查询、关节运动、末端位姿运动和末端夹爪控制接口。

> **安全提示**
>
> 这是一个会驱动真实机器人运动的项目。任何联机测试都必须在机器人周围没有人员和障碍物、已经配置碰撞行为、急停可用且操作者能够立即切断动力的条件下进行。局域网 API 不是安全急停；急停和示教器上的安全机制始终优先。

## 当前状态

服务骨架、FastAPI 接口、线程隔离的命令管理器、fake 适配器和 `pylibfranka` 边界实现已经完成。默认仍建议先使用 fake 适配器和测试客户端验证部署配置；真实机器人控制必须按下方现场测试流程执行。详细约束见 [REQUIREMENTS.md](REQUIREMENTS.md)。

第一版明确采用以下边界：

- 机器人侧运行 Linux 主机、`pylibfranka` 和已启用 FCI 的 Franka 控制柜。
- 服务进程独占一个 `pylibfranka.Robot` 连接和一个实时控制循环。
- 局域网接口采用 HTTP/JSON；请求进入控制队列，由控制线程生成有限时长的轨迹。
- 网络线程不会在 `pylibfranka` 的实时回调中执行，也不会从回调中进行网络 I/O、磁盘 I/O 或阻塞日志操作。
- v0.1 开放带边界的 7 关节位置、笛卡尔位姿和 Franka Gripper 夹爪目标，不开放任意力矩、速度或原始实时 setpoint 转发。

## 架构

```text
┌──────────────────────┐             LAN / HTTP                ┌──────────────────────────┐
│  操作主机 / 客户端    │ ───────────────────────────────────▶ │  机器人主机               │
│  curl / SDK / UI      │ ◀────────── 状态与命令结果 ───────── │  franka_fci 服务          │
└──────────────────────┘                                      │  ├─ HTTP API              │
                                                              │  ├─ 命令校验与状态缓存     │
                                                              │  └─ pylibfranka 控制线程   │
                                                              └──────────┬───────────────┘
                                                                         │ FCI
                                                              ┌──────────▼───────────────┐
                                                              │  Franka 控制柜 / 机械臂    │
                                                              └──────────────────────────┘
```

HTTP 接口是监督控制面，不是实时伺服通道。需要连续遥操作或高频 setpoint 流时，应在后续版本中单独设计实时传输和 watchdog，不能把 HTTP 延迟直接带入机器人回调。

## 运行环境

### 机器人主机

- Ubuntu 22.04 或 24.04（以 `pylibfranka` 和实际控制柜的兼容矩阵为准）。
- Python 3.10–3.12；仓库默认使用 Python 3.11（见 `.python-version`）。
- 与控制柜位于可达网络中的有线网卡，并已启用 Franka FCI。
- 本项目现场机器人型号为 **Franka FR3**，拥有 7 个关节；末端夹爪通过控制柜的 Franka Gripper 接口连接。
- 笛卡尔位姿 API 的控制点是官方夹爪的 **gripper frame**，不是裸机械臂的 flange frame。
- 能够安装与机器人固件匹配的 `pylibfranka`/libfranka 版本。

### FR3 工控机网络

部署在对应工控机时，FCI 机器人控制柜的本地连接地址固定配置为 `172.16.0.2`。该地址是机器人侧连接地址，不是 HTTP API 的监听地址：

| 配置 | 值 | 说明 |
| --- | --- | --- |
| `FRANKA_ROBOT_IP` | `172.16.0.2` | 工控机连接 FR3 控制柜的 FCI 地址 |
| `FRANKA_API_HOST` | `127.0.0.1` | 默认仅本机访问 API；局域网控制时需显式改为工控机内网地址 |
| `FRANKA_API_PORT` | `8000` | 局域网 HTTP API 端口 |

工控机应通过独立有线网卡连接 `172.16.0.2`，并确认该网段不会与客户端 API 网段产生地址冲突。不要把 `172.16.0.2` 绑定为 API 服务监听地址。

### 开发或客户端主机

可以使用 macOS、Linux 或 Windows 编写客户端和运行脱机测试，但不能据此假定本机可以直接通过 FCI 控制机器人。真实控制服务应优先部署在经过实时性和网络配置验证的机器人主机上。

## 安装（uv）

在 FR3 工控机上安装 `uv` 后，从仓库根目录创建并同步环境：

```bash
uv python install 3.11
uv sync
```

运行命令时使用项目环境：

```bash
uv run pytest
uv run ruff check .
```

部署配置可从示例复制：

```bash
cp .env.example .env
```

`pyproject.toml` 是依赖真源，`uv.lock` 是部署锁定文件。`requirements.txt` 和 `requirements-dev.txt` 仅保留给尚未迁移到 uv 的外部工具使用；新开发统一使用 `uv sync`。`pylibfranka` 的版本需要和现场机器人固件、FCI 及底层 libfranka 兼容；如果现场约束不同，应在部署记录中锁定经过验证的版本，而不是在运行中自动升级。

## 配置约定（草案）

服务从 `.env` 或 `FRANKA_` 前缀环境变量读取配置，至少包括：

| 配置项 | 默认值 | 说明 |
| --- | --- | --- |
| `FRANKA_ROBOT_IP` | `172.16.0.2` | FR3 控制柜地址；部署配置仍应显式写出 |
| `FRANKA_ROBOT_MODEL` | `fr3` | 机器人型号；服务启动时校验为 `fr3` |
| `FRANKA_GRIPPER_ENABLED` | `true` | 是否启用末端 Franka Gripper 控制 |
| `FRANKA_GRIPPER_FLANGE_TRANSLATION_M` | 无 | flange 到 gripper 的标定平移 `[x,y,z]`，单位 m |
| `FRANKA_GRIPPER_FLANGE_QUATERNION_XYZW` | 无 | flange 到 gripper 的标定旋转，四元数顺序 xyzw |
| `FRANKA_API_HOST` | `127.0.0.1` | API 监听地址；暴露到局域网需显式改为内网地址或 `0.0.0.0` |
| `FRANKA_API_PORT` | `8000` | API 监听端口 |
| `FRANKA_STATE_STALE_AFTER_MS` | `250` | 状态超过该时长未更新即标记为过期 |
| `FRANKA_MAX_MOTION_DURATION_S` | `30` | 单条运动目标允许的最大时长 |
| `FRANKA_ALLOWED_ORIGINS` | 空 | 默认不启用跨域请求 |

密钥不得提交到仓库。生产网络还应通过主机防火墙限制来源地址；是否启用 TLS 应根据局域网边界和部署方式决定，不能把“在局域网内”当作身份认证。

## API 草案（v1）

接口前缀固定为 `/api/v1`。以下示例描述当前服务端契约。

查询健康状态：

```bash
curl http://robot-host:8000/api/v1/health
```

提交一次有限时长的关节位置运动（角度单位为 rad）：

```bash
curl -X POST http://robot-host:8000/api/v1/motions/joint-position \
  -H 'Content-Type: application/json' \
  -d '{
    "target_rad": [0.0, -0.4, 0.0, -2.0, 0.0, 1.6, 0.7],
    "duration_s": 3.0,
    "request_id": "demo-001"
  }'
```

目标接口：

| 方法 | 路径 | 作用 |
| --- | --- | --- |
| `GET` | `/api/v1/health` | 服务、FCI 连接、机器人模式和错误摘要 |
| `GET` | `/api/v1/state` | 读取带时间戳的最新机器人状态缓存 |
| `POST` | `/api/v1/motions/joint-position` | 提交 7 关节位置目标，返回 `command_id` |
| `POST` | `/api/v1/motions/cartesian-pose` | 提交基坐标系下的夹爪末端位姿目标，返回 `command_id` |
| `POST` | `/api/v1/gripper/homing` | 执行夹爪 homing，返回 `command_id` |
| `POST` | `/api/v1/gripper/move` | 将夹爪移动到指定开口宽度 |
| `POST` | `/api/v1/gripper/grasp` | 以指定宽度、速度和力执行夹持 |
| `GET` | `/api/v1/gripper/state` | 读取夹爪宽度、运动状态和错误 |
| `GET` | `/api/v1/commands/{command_id}` | 查询命令生命周期和失败原因 |
| `POST` | `/api/v1/stop` | 请求停止当前运动；不替代物理急停 |

运动请求成功接收时返回 `202 Accepted`，而不是等机器人运动结束。服务一次只允许一个活动运动命令；命令状态至少包含 `queued`、`running`、`succeeded`、`stopped`、`failed` 和 `rejected`。所有输入都必须有限、维度正确、单位明确，并同时通过服务端和 `pylibfranka`/机器人约束。

夹爪参数使用 SI 单位：开口宽度为 m，速度为 m/s，夹持力为 N。夹爪命令与机械臂运动共享命令状态、停止和审计机制；夹爪 homing 或 grasp 结果必须以 `pylibfranka` 返回结果为准。

### 夹爪末端位姿变换

API 的笛卡尔目标定义为 `T_base_gripper`，也就是官方夹爪末端相对于 FR3 `base` 坐标系的位姿。控制器内部根据现场标定的固定安装变换 `T_flange_gripper` 换算为 `pylibfranka` 所需的 flange 目标：

```text
T_base_gripper = T_base_flange · T_flange_gripper
T_base_flange_target = T_base_gripper_target · inverse(T_flange_gripper)
```

`T_flange_gripper` 必须描述“FR3 flange 坐标系到官方夹爪末端 frame”的变换，不能直接猜测或使用未经现场确认的默认值。平移使用 m，旋转使用 `quaternion_xyzw`；服务启动时校验四元数和变换的有限性。状态接口应同时返回 flange 和 gripper 位姿，便于验证变换方向。

## 设计原则

- **实时控制与网络解耦**：控制线程拥有机器人连接，API 线程只负责校验、排队和读取缓存。
- **默认拒绝运动**：未连接、状态过期、存在错误、已有活动命令或超出限制时，不接受新的运动。
- **可停止且可追踪**：每条命令都有唯一 ID、来源、开始/结束时间和结果；停止和异常路径要留下结构化日志。
- **单位显式**：关节位置/速度使用 rad、rad/s，笛卡尔位置使用 m，姿态使用 `quaternion_xyzw`；API 的 `frame_id` 是参考坐标系，目标工具坐标固定为 `gripper`，不得默认为裸 flange。
- **不自动清除安全错误**：错误恢复需要现场人员确认，服务不会通过重连或重试绕过急停、碰撞或控制柜故障。

## 开发路线

1. 实现 FR3 的 `pylibfranka` 适配层、夹爪适配层和机器人状态快照。
2. 实现轨迹/命令管理器，将 API 请求与实时控制线程隔离。
3. 实现 FastAPI 接口、网络边界配置和结构化日志。
4. 先用 fake robot/gripper adapter 完成脱机单元测试，再进行只读、停止、单步运动和夹爪 homing 的现场测试。
5. 补充工控机 systemd 部署、主机防火墙、版本兼容矩阵和客户端 SDK。

在完成真实机器人验证前，不应把服务绑定到公共网卡，也不应把本 README 中的 API 草案视为已经可用的控制接口。
