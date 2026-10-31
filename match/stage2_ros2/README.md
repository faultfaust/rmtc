# rmtc_stage2_tm —— ROS 2 Humble 节点与离线评测

本包把 `stage2_tm` 的核心决策/事件/LLM 代码包装成一个可 `colcon build` 的 ROS 2 包：

- 节点 `rmtc_bot`：订阅 `/team_<seat>/perception`，发布 `/team_<seat>/action` 和 `/team_<seat>/registration`；
- `LlmClient` 在 `Agent` 内部异步工作，不阻塞动作窗口；
- 附带 `rmtc_match4` 四车模拟器和 8 个离线回归目标；
- 附带与 `stage2_tm` 的 `parseLlmEvent` 兼容的 `mock_gateway.py`。

## 1. 目录

```text
stage2_ros2/
  CMakeLists.txt
  package.xml
  include/rmtc/*.h          # 与 stage2_tm 同步的核心头
  src/main.cpp              # ROS 2 薄壳
  src/agent.cpp ...         # 与 stage2_tm 同步的核心实现
  src/match4.cpp            # 离线四车模拟器
  test/*.cpp                # 规则/协议/LLM 回归
  launch/rmtc_bot.launch.py
  config/rmtc_bot.yaml
  scripts/mock_gateway.py
```

> **重要**：`include/` 与 `src/` 是 `stage2_tm` 的副本。修改核心代码时请改 `stage2_tm/`，再同步到这里，避免两套头文件 ODR 不一致：
>
> ```bash
> cp ../stage2_tm/rmtc/*.h include/rmtc/
> cp ../stage2_tm/rmtc/*.cpp src/rmtc/
> cp ../stage2_tm/{agent,combat,planner,world,match4}.cpp src/
> cp ../stage2_tm/{v12_check,damage_mission_check,stage2_probe,stage2_sim,stage2_llm_test,stage2_llm_e2e,stage2_ai_qualify_test,stage2_gateway_check}.cpp test/
> ```

## 2. 依赖

- Ubuntu 22.04 + ROS 2 Humble
- `rclcpp`、`std_msgs`、`ament_cmake`
- C++17 编译器（g++ 11+）
- Python 3（mock gateway / launch）

```bash
sudo apt install -y \
  ros-humble-rclcpp ros-humble-std-msgs ros-humble-ament-cmake \
  ros-humble-launch ros-humble-launch-ros \
  build-essential python3
```

## 3. 构建

```bash
mkdir -p ~/rmtc_ws/src
# 目录名用包名 rmtc_stage2_tm，scripts/build_clean.sh 按这个布局找 workspace
cp -r stage2_ros2 ~/rmtc_ws/src/rmtc_stage2_tm
cd ~/rmtc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rmtc_stage2_tm --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> 覆盖升级时请**整目录替换**（或解压 tarball 覆盖），不要只拷 `include`/`agent.cpp`。
> 头文件与 `src/rmtc/event_prompt.cpp` 不同步会出现
> `undefined reference to buildHandshakeRequestBody(int, std::string const&)`。

产物：

```text
install/rmtc_stage2_tm/lib/rmtc_stage2_tm/rmtc_bot
install/rmtc_stage2_tm/lib/rmtc_stage2_tm/rmtc_match4
install/rmtc_stage2_tm/lib/rmtc_stage2_tm/v12_check
...
```

### 3.1 编译报 `undefined reference to buildHandshakeRequestBody(int, std::string const&)`

**根因**：`include/rmtc/event_prompt.h` 和 `src/agent.cpp` 是新的（2 参数），
但 `src/rmtc/event_prompt.cpp` 还是旧版（只定义 1 参数），或者 `build/` 里旧
`event_prompt.cpp.o` 因 mtime 没被重编。

**处理**（从包目录，标准 `~/rmtc_ws/src/rmtc_stage2_tm` 布局）：

```bash
cd ~/rmtc_ws/src/rmtc_stage2_tm

# 先确认源码是 2 参数定义（应看到 const std::string& model）
grep -n "buildHandshakeRequestBody" src/rmtc/event_prompt.cpp
# 如果只有 1 参数：用最新 tarball 覆盖整个目录，不要只拷单个文件

# 强制重编（touch + 删除本包 build/install + colcon build + 符号校验）
bash scripts/build_clean.sh
```

手动等价命令：

```bash
cd ~/rmtc_ws/src/rmtc_stage2_tm
touch include/rmtc/*.h src/*.cpp src/rmtc/*.cpp test/*.cpp
rm -rf ~/rmtc_ws/build/rmtc_stage2_tm ~/rmtc_ws/install/rmtc_stage2_tm
cd ~/rmtc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rmtc_stage2_tm --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

验证 object 符号：

```bash
nm -C ~/rmtc_ws/build/rmtc_stage2_tm/CMakeFiles/rmtc_bot.dir/src/rmtc/event_prompt.cpp.o \
  | grep buildHandshakeRequestBody
# 期望看到：
# rmtc::buildHandshakeRequestBody(int, std::__cxx11::basic_string<...> const&)
```

如果仍只有 1 参数，说明 `src/rmtc/event_prompt.cpp` 是旧文件；用
`stage2_ros2.tar.gz` 重新覆盖后再 `scripts/build_clean.sh`。

## 4. 启动节点

```bash
source /opt/ros/humble/setup.bash
source ~/rmtc_ws/install/setup.bash

# 练习裁判（裁判教程 v1.1）：默认地址就是 127.0.0.1:8080
# 注意：裁判教程用的变量名是 RMTC_TEAM_TOKEN，不是 RMTC_AI_TOKEN
export RMTC_TEAM_TOKEN='practice-A'
# 可选：只有正式比赛换了网关地址时才需要覆盖
# export RMTC_AI_URL='http://<正式网关>:<port>/v1/chat/completions'
export ROS_DOMAIN_ID=0           # 按主办方要求

ros2 launch rmtc_stage2_tm rmtc_bot.launch.py seat:=A team_name:="My Team"
```

也可以直接跑可执行文件：

```bash
ros2 run rmtc_stage2_tm rmtc_bot --ros-args -p seat:=A -p team_name:="My Team"
```

参数/环境：

| 参数/变量 | 含义 |
|---|---|
| `seat` / `RMTC_SEAT` | 席位 A/B/C/D；环境变量优先 |
| `team_name` / `RMTC_TEAM_NAME` | 注册队名 1–96 字节，环境变量优先 |
| `RMTC_TEAM_TOKEN` | 裁判教程 v1.1 的席位令牌，例如 `practice-A`（**推荐**） |
| `RMTC_AI_TOKEN` | 兼容旧文档的令牌变量；与 `RMTC_TEAM_TOKEN` 二选一 |
| `RMTC_AI_URL` | 可选；默认 `http://127.0.0.1:8080/v1/chat/completions`。也可只填 `http://<gateway>:<port>`，代码会自动补 `/v1/chat/completions` |
| `RMTC_AI_MODEL` | 可选；默认 `deepseek-flash`，请求体顶层带 `model` |
| `RMTC_AI_TIMEOUT_MS` | 可选；事件请求总超时，默认 3000（6 个 Tick；2026-10-30 由 1800 调整） |
| `RMTC_AI_HANDSHAKE_TIMEOUT_MS` | 可选；资格握手总超时，默认 4500（冷启动更慢） |
| `RMTC_AI_QUOTA` | 可选；本地成功调用上限，默认 0=不设限（网关 429 兜底） |

> 令牌不会打印。**没有令牌时节点仍能跑本地策略，但不会挂 LLM，本场没有 AI 2xx，最终 M=0**；这不是“网关坏了”，而是没有鉴权令牌。
>
> **每场通常重新签发席位令牌**（协议 §5：其他场次令牌失效）。节点只在启动时读取
> `RMTC_TEAM_TOKEN`/`RMTC_AI_URL`，换场/换令牌必须重启节点；不要复用上一场进程，
> 否则会 401、本场没有 2xx、M=0。
>
> 裁判教程 v1.1 的本地地址是 `http://127.0.0.1:8080/v1/chat/completions`，令牌是 `practice-A/B/C/D`。如果程序运行在另一台机器，按教程做 SSH 端口转发 `ssh -N -L8080:127.0.0.1:8080 ...`。

## 5. 话题/QoS

| 话题 | 方向 | 类型 | QoS |
|---|---|---|---|
| `/team_<seat>/perception` | 订阅 | `std_msgs/String` | RELIABLE + VOLATILE + KeepLast(1) |
| `/team_<seat>/action` | 发布 | `std_msgs/String` | RELIABLE + VOLATILE + KeepLast(1) |
| `/team_<seat>/registration` | 发布 | `std_msgs/String` | RELIABLE + TRANSIENT_LOCAL + KeepLast(1)，启动即发、每 10s 重发 |
| `/referee/event_text` | 订阅 | `std_msgs/String`（纯文本） | RELIABLE + TRANSIENT_LOCAL + KeepLast(128) |
| `/referee/mission` | 订阅 | `std_msgs/String`（纯文本） | RELIABLE + TRANSIENT_LOCAL + KeepLast(1) |

公告话题独立于感知里的 `event_text` 缓存：感知缓存有容量上限，被挤出的公告
不会再进入 LLM；节点直接订阅公告话题并交给 `Agent::onRefereeEventText` /
`onRefereeMissionText`。任务的 `missions[]` 若缺 reach 坐标，Agent 会从
`mission_text`（或公告话题）里按关键词 + 坐标组兜底恢复，不依赖 LLM 是否成功。

感知/动作 JSON 原样透传；`tick`、`session_id`、动作字段由 `Agent` 解析和回填，节点不重写。

## 6. 网关联调

### 真实网关

```bash
# 练习裁判教程 v1.1：URL 可省略（默认 127.0.0.1:8080），只需令牌
export RMTC_TEAM_TOKEN='practice-A'
ros2 run rmtc_stage2_tm stage2_gateway_check --repeat=3 --timeout=5000

# 正式比赛换网关时：
# export RMTC_AI_URL='http://<gateway>:<port>/v1/chat/completions'
# export RMTC_TEAM_TOKEN='<正式席位令牌>'
```

当前 `http_transport.cpp` 只支持 `http://`；HTTPS 需要 TLS 反向代理或改传输层。请求体 ≤2 MiB、非流式、Bearer 鉴权；2xx 即消耗配额。额度按席位令牌全生命周期累计、每场上限赛前公告、**0 表示不限**；客户端默认 `RMTC_AI_QUOTA=0`，由网关 429 兜底。

> 请求体会自动带 `"model":"deepseek-flash"`、`"thinking":{"type":"disabled"}`、
> `"response_format":{"type":"json_object"}`、`"max_tokens":512`（事件）或 `16`
> （握手）。完整模板、system prompt 与示例见
> `docs/LLM请求模板_20261028.md`；节点日志会打印实际生效的 model。

### 本地 mock

```bash
# 终端 1（源码目录）
python3 scripts/mock_gateway.py --port 8080

# 终端 2：只设令牌，URL 用默认 8080
export RMTC_TEAM_TOKEN='practice-A'
ros2 run rmtc_stage2_tm stage2_gateway_check --repeat=2 --timeout=3000
```

也可以显式指定端口：

```bash
export RMTC_AI_URL='http://127.0.0.1:8080/v1/chat/completions'
export RMTC_AI_TOKEN='test-token'
ros2 run rmtc_stage2_tm stage2_gateway_check --repeat=2 --timeout=3000
```

该 mock 返回**顶层** `EventReport` JSON（`{"kind":...}`），与 `stage2_tm` 的 `parseLlmEvent` 兼容；不要用 `stage2_lab/tools/mock_gateway.py`（它返回 `{"events":[...]}`，是另一套 `AiBridge` 的格式）。

## 7. 测试

```bash
cd ~/rmtc_ws
colcon test --packages-select rmtc_stage2_tm
colcon test-result --verbose
```

或直接跑（2026-10-30 合并版 + 公告去重/解析状态：37 / 5 / 72 / 25 / **32** / 33 / 10 全过）：

```bash
./install/rmtc_stage2_tm/lib/rmtc_stage2_tm/v12_check                  # 37 条（含爆炸生命周期/机会主义 reach/解析状态）
./install/rmtc_stage2_tm/lib/rmtc_stage2_tm/damage_mission_check
./install/rmtc_stage2_tm/lib/rmtc_stage2_tm/stage2_probe
./install/rmtc_stage2_tm/lib/rmtc_stage2_tm/stage2_sim
./install/rmtc_stage2_tm/lib/rmtc_stage2_tm/stage2_llm_test
./install/rmtc_stage2_tm/lib/rmtc_stage2_tm/stage2_llm_e2e            # 33 条（含爆炸回收 C4/公告重放去重 C5）
./install/rmtc_stage2_tm/lib/rmtc_stage2_tm/stage2_ai_qualify
./install/rmtc_stage2_tm/lib/rmtc_stage2_tm/reach_event_check         # 10 条：LLM 全失败时 reach 仍完成
./install/rmtc_stage2_tm/lib/rmtc_stage2_tm/dns_check                 # 3 条：坏域名不拖死请求
```

> `stage2_sim` 场景 2 在当前 `Config` 默认 `combat_depth=5` 下会给出 `retreat`
> （move），而断言要求 defend/attack；该行为在合并前后逐位一致，属于仓库既有
> 基线差异（详见 MERGE_REPORT.md §5）。

离线高压：

```bash
./install/rmtc_stage2_tm/lib/rmtc_stage2_tm/rmtc_match4 \
  1000 500000 opp=hybrid spawn=2 rot=1 strict=1 eventaware=1
```

## 8. 决策原因诊断

实战/练习中若看到“来回走、不还手”，先开逐 Tick 决策日志（不改行为）：

```bash
export RMTC_DEBUG_DECISION=1
# 正常启动节点
```

日志格式：

```text
DEC tick=27 pos=(6,7) hp=80 r2=10 reason=resource target=... planV=... planC=... rate=... gate=1 action=move
```

- `reason=resource/explore/safe/attack/defend/retreat/event/no_target` 直接指出当前分支；
- `gate=1` 表示经济门在压攻击；
- `rate` 是经济门使用的采集边际率；
- `target` 连续变化 → 目标抖动；`target` 稳定但方向来回 → 路径回摆。

把 `DEC` 行（尤其异常 Tick 前后 20 Tick）发回来即可定位，不必再靠猜。

实战日志里出现连续 100+ Tick 的 `R_WAIT_NO_TARGET` 时，可临时开启兜底巡逻：

```bash
export RMTC_NO_TARGET_FALLBACK=1        # 默认 0
export RMTC_NO_TARGET_FALLBACK_TICKS=20 # 连续 20 Tick 才触发
export RMTC_NO_TARGET_FALLBACK_R2=100   # R2 已高则不巡逻
```

它只影响原本会原地等待的局面；3hybrid 1000 局均分 +0.3、前二 −0.2，
shadow 前二不变，所以默认不开，现场按对手/地图再决定。

资源耗尽后的猎杀默认开启（只在自己 R2 < 40 时触发，避免打扰正常局）：

```bash
export RMTC_POST_RESOURCE_HUNT=1      # 默认 1
export RMTC_POST_RESOURCE_HUNT_R2=40  # 默认 40
```

作用：所有已知资源拿完、连续 2 Tick no_target、HP>40 时，转向最近敌车；
没有敌车记忆就向地图中心走，不再原地等到终局。

## 9. 限制与注意

- **2026-10-30（合并版 = 主线 reach/LLM 修复 + 事件生命周期修复）**：
  - 【事件生命周期】爆炸改为瞬时事件：`parseLlmEvent` 把 `end_tick<0` 的爆炸
    归一化为 `activate_tick+1`；`refreshEvents` 在生效 Tick 后立即回收；
    `EventScorer` 在 `tick >= activate_tick` 不再给出 avoid；无 end 的兜底
    400 → 60 Tick。**这是 reach 失败的根因之一**：旧代码把 t=141 的爆炸
    当作永久危险区，t=348 仍把 A 锁在 x=3，任务点一步都走不过去。
  - 【reach 机会主义】任务点只差一步（`dsteps<=1`）时，即使还没到死线也先踩
    进去再发育。
  - 【reach 来源/顺序】死线不再挂在 `engage_gate` 下；冲刺窗口按真实步数提前
    （45 → max(45, 步数+10)，上限 90）；最后 15 Tick 有忽略软阻挡的直走兜底；
    reach 坐标不再只信 `missions[]`，缺 position 时从 `mission_text` /
    `/referee/mission` 恢复并缓存；`reach_target` 等类型也能识别。
  - 【LLM】事件默认超时 1800ms → **3000ms**；订阅 `/referee/event_text`、
    `/referee/mission`，公告不再受感知 `event_text` 缓存容量限制；传输层
    body 截断不再当 2xx；2xx 但 content 不可用可重试；校验失败不再额外停
    5 Tick（“已结束/已解除”状态行不再拖住真事件）。
  - 【公告去重/解析状态】新增本场持久 `seen_event_lines_`，同一行跨
    `perception.event_text`、`/referee/event_text`、TRANSIENT_LOCAL 重放只
    提交一次（协议 §3：重复收到公告不代表重复触发）。`parseLlmEvent` 增加
    OK/NO_EVENT/STALE/INVALID 四态：迟到的爆炸、`unknown` 状态行降为 INFO，
    不再刷 `event report rejected by validator`；只有真正的字段/坐标错误才 WARN。
  - 【LLM 剩余风险】DNS 解析改为带 1.5s 上限（+30s 失败冷却）的后台解析，
    getaddrinfo 卡死不再拖死 worker；LlmClient 改 2 个 worker（协议允许
    2 个在途调用），3s 事件请求不再堵住到期的资格握手。
  - 回归：`v12_check` 37、`stage2_llm_e2e` 33、`reach_event_check` 10、
    `dns_check` 3；其余 5/72/25/32，`stage2_sim` 2 场景全过。
- `stage2_ros2` 的核心代码是 `stage2_tm` 的副本；两边必须同步，详见第 1 节。
- **2026-10-29**：修复了 2xx 无 content 不算资格、429 `rate_limit` 不真正重试、
  公告提交前就标记导致丢失、配额硬编码 100、只填根地址 404 等问题；回归 193 条。
  详见 `docs/修复_20261029.md`。
- **2026-10-28**：`http_transport.cpp` 已改为整请求总截止 + 非阻塞 connect +
  收满 Content-Length 即返回；`LlmClient` 出队时握手优先。Windows 下旧代码把
  `SO_RCVTIMEO/SO_SNDTIMEO` 当 `timeval` 传（Winsock 只读 `tv_sec`，1800ms 变成
  1ms），已修；Linux 不受该单位问题影响。详见 `docs/LLM网关无法调用_诊断_20261027.md` §2.5。
- 评测脚本：`stage2_tm/tools/build_win.ps1`（编译 + 校验产物时间戳 + 跑回归）、
  `stage2_tm/tools/run_eval.ps1`（相对脚本定位、旧 exe 告警）、
  `stage2_tm/tools/run_eval.sh`（Linux）。不要把 `_build/` 写死成相对当前目录。
- `eventaware=1` 是 `match4` 的测试真值注入，**不是** LLM；真实节点走 `RMTC_AI_URL`。
- 真实网关事件公告延迟由 `parseLlmEvent` 的 `end_tick` 窗口校验；结果年龄上限 100 Tick（约 50s）。超过后 `LlmClient` 丢弃。
- `http_transport.cpp` 不支持 HTTPS/代理/重定向；正式网关若是 HTTPS，请前置 TLS 代理。
- 当前包没有 `ament_lint_auto` 依赖；如果需要 lint，自行添加 `ament_lint_auto`/`ament_lint_common`。
- 如果 `colcon build` 报 `rclcpp` 找不到，确认已经 `source /opt/ros/humble/setup.bash`。
- 若队友已有自己的 `main.cpp`/CMake，优先合并到他们的节点，避免两套节点抢同一席位话题。
