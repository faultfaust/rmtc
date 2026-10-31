# 合并版通信/事件下限检查（2026-10-30 晚）

## 1. “ignore”会不会影响比赛

这轮日志里的 ignore 分两类：

| 日志 | 含义 | 是否影响 |
|---|---|---|
| `stale event ignored (activate already past)` | 同一条爆炸/公告的重复或迟到副本；第一次已经在 t125 前 `event pooled` | **不影响**。爆炸是瞬时事件，t>125 时伤害已结算，再避也来不及；重复副本本来就不应再触发 |
| `event line has no event (ignored)` | 模型对“已结束/已解除/效果消失”类状态行返回 `unknown/0.0` | **不影响**。它不是新事件；这轮真正的 fortress/pepper/heal 都已 `event pooled` |

唯一的残余风险：如果模型把**真正的新事件**误判成 `unknown`，这条事件会被忽略，本地没有关键词兜底（协议 §8.1 禁止用正则/关键词替代 LLM）。这轮日志里真实事件都已入池，所以本局没有因此丢事件；要再加保险，只能在 `unknown` 时做一次更严格的 LLM 复核，代价是配额/延迟，默认不建议开。

结论：**本局的 ignore 不影响比赛**；它不是“事件类型非法”，也不是“解析器坏了”，而是重复公告/状态通知的正常分流。

## 2. 已做的通信下限加固

在原有“公告去重 + 四态解析”基础上，又补了 15 处：

1. **9×9 中心校验**：`parseRelative9x9` 要求第 5 行第 5 列为 `'1'`（协议 §3）；缺了直接丢弃该实体，避免用偏移的 `'2'` 算错绝对坐标。
2. **实体越界丢弃**：`self.position + 相对位移` 超出 `map_w/map_h` 的 robot/obstacle/resource 不再进入雷达/世界模型。
3. **tick/session 校验**：感知 `tick < 0` 或 `session_id` 为空直接拒绝，不发布 tick/session 非法的动作。
4. **动作编码兜底**：`MOVE_MOVE`/`MOVE_ATTACK` 若 direction 为空，降级为 `wait`，绝不发出缺 `direction` 的 E_SCHEMA 动作。
5. **数值有限性**：`numOr` 拒绝 `inf/nan`（损坏 JSON 的 `1e309` 等），避免污染坐标/伤害计算。
6. **在途公告解卡**：`event_pending_seq_` 超过 12 Tick 没有结果时主动丢弃当前行并继续。否则 `LlmClient::poll` 按 `max_age_ticks` 丢掉结果后，`event_pending_seq_` 永远为正，后续公告全部提交不出去。
7. **HTTP Host 头**：非 80 端口写 `Host: host:port`，符合 HTTP/1.1，兼容按端口做虚拟主机的网关。
8. **vision_radius 夹紧 0–4**：协议 §3 只允许 0–4；异常值不会让 World 把整图当可见。
9. **世界分裂激活 Tick 强制 WAIT**：§5.2 规定分裂在移动前击杀/强制复活，激活 Tick 再发 move 必然被 E_STATE 取消；现在直接发 wait。
10. **离线模拟器/测试的 9×9 重叠图**：`match4` 和 `v12_check` 的构造器原来把中心 `'1'` 覆盖成 `'2'`，与协议 §3 的“重叠时中心保留 1、无 2”相反；已统一改为 `else if`。
11. **洁身自好（thorns）接入 Combat**：新增 `Params::thorns_active`；攻击正在防御的敌人时 `expectedDamage=0`；`mustBreakDefense` 不再强制出手；`tryReachContest` / 无双 5×5 在 5×5 内有防御目标时也不再打，避免 0 伤害 + 自己 20 反伤。
12. **潘潘的神（vulnerable）接入 Combat**：新增 `Params::self_vulnerable`；`incomingDamage` 的普通受击按 40 计（防御 20）；`Agent::updateEffectState` 根据“上一 Tick 成功移动 + 触发窗口 [activate, activate+20)”维护绝对到期 Tick；thorns/vulnerable 生效时绕过深层搜索（深层仍按 20/10 名义值），改走已修正的启发式；无双/占位争夺在残血易伤时不再强行换命。
13. **敌方易伤接入**：`Radar::Sighting` 新增 `last_moved_tick` / `vulnerable_until`；只把“连续两帧目击、曼哈顿位移恰好 1”的敌车标记为在潘潘窗口内成功移动；`Combat::expectedDamage` 对易伤目标按 40/20 计算；有易伤敌人时同样绕过深层搜索，由启发式按目标结算。
14. **幻术激活 Tick 提前置位**：`updateEffectState` 在 `activate_tick` 当 Tick 检查自己是否在矩形内，若在则直接把 `illusion_until_` 设为 `end-1`；后续 Tick 即使没有移动回执，也不会被“无 mismatch”逻辑清掉，第一次 MOVE 就能正确反向。
15. **match4 潘潘计时校准**：评测器原来在整个 30 Tick 都允许触发，且按 `move+30` 记到期；已改为 `[activate, activate+20)` 触发、统一在 `activate+30` 到期，并按 `>` 判断到期，供 A/B 使用。

## 3. 通信逐项检查结果

| 项目 | 检查 | 结果 |
|---|---|---|
| 注册话题 | `/team_X/registration`，`{"team":"A","name":"..."}`，1–96 UTF-8 字节，无换行 | PASS（`sanitizeTeamName`） |
| 感知/动作 QoS | RELIABLE + VOLATILE + depth 1 | PASS |
| 公告/任务 QoS | RELIABLE + TRANSIENT_LOCAL；event depth 128，mission depth 1 | PASS |
| 感知 tick/session | 必须存在且类型正确；动作原样回填 | PASS（新增非空/非负校验） |
| 9×9 相对图 | 89 字符、8 个 `\n`、中心 `'1'`、至多一个 `'2'` | PASS（新增中心校验） |
| 实体坐标 | `self.position + 相对位移`，越界丢弃 | PASS（新增） |
| JSON 解析 | 递归下降、字符串转义/`\u`、数字、数组/对象、深度 64 | PASS |
| JSON 输出 | `jsonEscape` 处理引号/反斜杠/控制字符；UTF-8 原样 | PASS |
| 动作 JSON | `tick`/`session_id`/`action`/`direction` 与协议 §4 一致；≤4096 字节 | PASS（新增缺 direction 兜底） |
| 动作发布 | 每 Tick 一条；同 Tick 重发返回缓存动作；旧 Tick 丢弃 | PASS |
| LLM 请求 | `Content-Type`、`Authorization: Bearer`、`Content-Length`、`model`、`thinking.disabled`、`response_format=json_object`、`stream=false`、`temperature=0`、`max_tokens` | PASS |
| LLM 响应 | 2xx 解析 `choices[0].message.content`；空/坏 content 可重试；2xx 即计配额/资格 | PASS |
| 429/502/504 | 可重试；`quota_exceeded` 本场降级 | PASS |
| 503 | 不重试，保持本地 | PASS |
| 在途数 | 2 worker，协议允许 2 个在途 | PASS |
| 结果过期 | `max_age_ticks=100`；新增 Agent 侧 pending 解卡 | PASS（新增） |
| HTTP 响应 | 支持 Content-Length/读到关闭；超时/截断不冒充 2xx | PASS |
| 公告去重 | 跨感知/公开话题/TRANSIENT_LOCAL，本场持久 | PASS（本次修复） |
| 事件状态 | `OK/NO_EVENT/STALE/INVALID`，只有 INVALID WARN | PASS（本次修复） |

## 4. 仍需注意的规则/战术边界

以下不是协议/JSON 错误，也不会直接产生非法动作：

1. **天下无双**：5×5 攻击逻辑正确，但 `p.hp<45` 时主动放弃、且跳过保护目标；这是保守策略，不是规则错误。
2. **新场次公告早于首帧感知**：`onRefereeEventText` 可能先把新场公告入队，随后首帧感知触发换场清理，把队列清掉；这是边界风险，暂未改。
3. **敌方易伤的观测限制**：只有“连续两帧都看到、位移恰好 1 格”才判定为成功移动；隔帧、离开视野、复活/传送（位移 >1）不标记，避免高估。5×5 无双的伤害由裁判结算，代码只影响普通攻击估值。
4. **幻术迟到公告**：如果 LLM 在激活 Tick 之后才返回，代码无法还原“激活瞬间是否在矩形内”，只能靠第一次移动回执发现；会慢 1 Tick。公告按时到达时，激活 Tick 已能提前置位。

## 5. 仍需注意的边界

1. **LLM 把真事件判成 unknown**：只能靠 LLM 复核，不能本地关键词兜底。
2. **3 s 事件超时 vs 5 Tick 预告**：日志在 t299 出现过一次 `recv timeout`，重试 5 Tick 后在 t305 刚好接上 heal。若网关更慢，建议现场把 `RMTC_AI_TIMEOUT_MS` 设为 2000（给重试留余量），或确认裁判预告提前量。
3. **HTTPS/代理/重定向**：`http_transport.cpp` 不支持；协议给的是 `http://` 网关。
4. **chunked 响应**：当前按 Content-Length 或读到关闭；正式网关若改 chunked，需要补解码。

## 6. 构建/回归

```bash
cd /ceshi/stage2_ros2_merged
bash run_merged_tests.sh
```

预期 `v12_check` 37 条、`stage2_llm_e2e` 33 条全过，`REGRESSION PASS`。
