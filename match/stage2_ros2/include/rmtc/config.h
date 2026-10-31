#pragma once
#include <string>

namespace rmtc {

// 全部可调参数集中在这里。规划器（Planner）在构造时拿一份拷贝，运行期不再改动，
// 因此这些值等价于"编译期常量"，调参后必须重新构建。
//
// 参数分四组：
//   [时间/总量]  与赛制、资源总量相关，换阶段必须改
//   [视野/风险]  寻路与未知区通行的代价
//   [采集]       资源候选筛选、收尾保护
//   [探索]       未知区域的价值估计与动态放弃阈值
//   [规模]       精确解（Held-Karp）的使用边界
struct Config {
    // ---------- 时间与总量 ----------
    // 第二阶段 §3：动作 Tick 0–499，最终状态 tick=500（第一阶段为 100）。
    // 规划器的剩余时间 R = total_ticks - tick 全靠这个值；换成 500 Tick 后必须
    // 同步改这里，否则中局起 R 会被误判为 0、一直 WAIT。
    int total_ticks = 500;
    // §4.2：资源总量 = 12×10（普通） + 4×20（高价值） = 200。
    // hidden = total_value - 已得分 - 已知视野内资源，用来估计"还没看见的资源"。
    float total_value = 200.f;
    // 视野半径上限。协议允许 0–4，超出的值按此截断（防止异常感知把覆盖估计放大）。
    int vision_max = 4;

    // ---------- 寻路风险 ----------
    // 走未知格的额外代价权重：dijkstra 里每进入一个 UNKNOWN 格，risk += risk_weight。
    // 调大 → 更倾向于走已探明的路；调小 → 更愿意抄近路穿未知区。
    float risk_weight = 2.0f;
    // 是否允许规划路径穿过未知格。false 时只走已知空地（更慢但更安全）。
    bool allow_unknown_travel = true;

    // ---------- 自愈（第一阶段撞墙教训） ----------
    // 0=关闭自愈；1=最后 2 Tick 的保守判断；2=含撞墙后的保守窗口
    int heal_level = 2;
    // 连续失败多少次后，保守窗口从 3 Tick 延长到 8 Tick
    int fail_limit = 2;
    // 收尾保护：若"剩余时间 - 到达目标步数" ≤ 该值，只保留确认走得到的资源
    int strict_finish_ticks = 6;

    // ---------- 探索 ----------
    // 未知格的期望价值权重（乘在 hidden 上），是探索/采集权衡的主旋钮
    float explore_weight = 0.12f;
    // 探索价值的折扣时间窗（Tick）：越接近时间上限，远方的未知区越不值钱
    float explore_horizon = 12.f;
    // 固定放弃阈值（dynamic_stop=false 时使用）
    int explore_stop_R = 25;
    // 是否用"当前收集计划的总步数 + collect_margin"动态推出放弃阈值
    bool dynamic_stop = true;
    int collect_margin = 4;   // 动态阈值在计划步数上多留的余量
    int min_stop = 6;         // 动态阈值下限
    int max_stop = 25;        // 动态阈值上限
    // 剩余时间超过该值之前，探索强度按比例封顶（避免开局就满速探索）
    int explore_full_R = 45;
    // 最后 endgame_R Tick 若还有可达资源，就完全停止探索
    int endgame_R = 15;

    // ---------- 采集 ----------
    // true：用"整条收集计划的价值/步数"作为采集侧评分；
    // false：只用首站的 价值/距离（第一阶段 v1 口径，保留用于 A/B 对比）
    bool plan_level_ratio = true;
    // 目标迟滞：新目标评分必须超过旧目标评分 × switch_ratio 才切换，防止来回摇摆
    float switch_ratio = 1.15f;
    // 任务耦合（A/B 可选）：未完成的“收集 2 普通 / 1 高价值”任务，把对应资源
    // 价值放大。2026-10-27 的 500 局 3×hybrid A/B 中，1.8× 与 1.0× 的
    // 前二率差异在噪声内（74.2% vs 74.4%），所以默认保持 1.0（不启用）。
    float mission_coupling = 1.0f;
    // 目标提交时长上限（Tick）：一个目标最多锁定这么久，之后重新评估
    int submit_max = 10;
    // P6（2026-10-30）：目标不变时，若新第一步正好是上一步的反方向，
    // 且存在“更接近目标”的替代邻格，则改走替代方向，打断 A→B→A 回摆。
    // 只影响连续反向且仍有推进方向的局面；死路仍允许原路退。
    bool planner_anti_reverse = false;
    bool planner_path_commit = true;  // 目标不变时锁定完整路径，避免 first-step 抖动
    bool planner_progress_guard = true;  // 资源目标禁止第一步“远离”目标

    // ---------- 探索实现细节 ----------
    // true=前沿（连通块）规划器；false=逐格覆盖增益
    bool use_frontier = true;
    int min_region = 3;          // 小于该格数的未知连通块不当作前沿
    float region_gamma = 0.5f;   // 按连通块大小给未知格加权的指数（仅逐格增益用）
    float region_ref = 20.f;     // 加权参考块大小
    float corridor_gain = 0.5f;  // 距离带来的额外"沿途收益"系数
    int min_gain = 1;            // 手上还有资源时，低于该覆盖增益的目标不考虑
    // 资源目击时长加成：0 = 关闭。
    // 问题 1 的消融：0 与 0.1 在 200 局里分差 <0.06（噪声），CPU 差 0.6%，
    // 因此默认关闭，少一个参数也少一处每 Tick 的乘法。
    float res_age_bonus = 0.f;
    float age_scale = 20.f;      // 上述"越久越高"的时间尺度（Tick，关闭时无效）
    float high_value = 20.f;     // value ≥ 该值视为高价值资源（协议口径）
    float high_priority = 1.2f;  // 高价值资源的规划权重加成
    float region_traversal = 1.f; // 前沿规划器里穿越区块的代价系数

    // ---------- 精确解规模 ----------
    bool exact_op = true;        // 是否启用 Held-Karp 精确巡游
    int exact_max_n = 12;        // 最多几个资源点用精确解
    int exact_max_cells = 400;   // 地图格子数上限（再大就用曼哈顿距离近似）

    // ---------- 第二阶段（对抗环境） ----------
    // 移动因"占位/争格"失败后，该格短时回避的 Tick 数。
    // 只做时效回避（不进永久地形记忆），用于打破双方反复抢同一格的死锁。
    int occupied_avoid_ticks = 2;

    // 低血回避（Agent 层）：规划器给出的移动若会踏进敌方射程，
    // 在 HP ≤ 该值时改选一个不在威胁下的方向。0 表示关闭。
    int low_hp_avoid = 40;

    // 决策时间熔断（§13.9）：单 Tick 决策超过该毫秒数就改发 wait。
    // 0 表示关闭。正常决策约 0.07 ms，这里只防病态情况。
    int decision_budget_ms = 300;

    // 高价值资源的时间竞争门控（项目全景 §12.9 建议 1）：
    // 对高价值资源（value ≥ high_value），用最近目击敌车的"最后目击位置做
    // BFS 的步数 − 目击年龄"估计其到达时间（取所有新鲜目击的最小值，年龄越旧
    // 越悲观——敌人可能已经在路上）。敌人明显先到（t_enemy × race_margin <
    // t_me）时该资源不进候选，规划器自动转向下一处安全资源。
    // 普通资源不参与；无任何新鲜目击（如开局）时不启用。默认关闭用于 A/B。
    bool race_gate = false;
    float race_margin = 1.35f;  // 先到判定余量：t_enemy × margin < t_me 判负
    int race_max_age = 25;      // 目击年龄上限（Tick），超过不参与估计
    // P7（2026-10-30）：更窄的“明确敌人先到”放弃规则。只针对高价值资源，
    // 且敌人必须在目标旁（曼哈顿 ≤ radius）并比我们少至少 margin 步。
    bool high_resource_abandon = false;
    int high_resource_abandon_margin = 4;
    int high_resource_abandon_radius = 2;
    float high_resource_abandon_r2 = 40.f;  // 只在自己资源很少时让

    // reach 任务点被敌车占住时，改为走到相邻格逼近（而不是直接放弃）。
    // 用于 A/B 归因；false = 初稿行为（放弃并原地等待）。
    bool reach_approach_occupied = true;
    // reach 冲刺死线（§5.3）：剩余 Tick ≤ max(base, per_step×真实步数 + margin)
    // 时接管导航。2026-10-28 把三个常数提出来做 A/B：失败局里既有"到点被身体
    // 挡住"，也有"最后 45 Tick 才动身"；提前出发可能用 R2 换 reach 完成率。
    int reach_deadline_base = 160;
    int reach_deadline_per_step = 3;
    int reach_deadline_margin = 50;
    // 任务点被敌车占据时的四阶争夺（2026-10-25《占位解决参考》）：
    // A 射程内直接打；B 有限等待/走邻格；C 等不起时强冲；D 低血止损。
    // 默认关闭用于 A/B；生产 main 开启。
    bool reach_contest = false;

    // 战斗层多步期望最大搜索的深度（0=启发式，1–5=模拟 Tick 数）。
    // 实测（match4，40 局×5 对手 + 自博弈两套种子，A 席得分）：
    //   depth=1  自博弈最高（39.2/38.9，互刷 meta）但对战术/蹲守最差（45.2/42.0）
    //   depth=3  战术 53.7 / 蹲守 50.0 / 农夫 29.2
    //   depth=5  战术 55.5 / 蹲守 54.6 / 农夫 31.4 / 莽夫 55.4，自博弈 36.8/36.5
    //            —— 对全部异质对手 ≥ depth=3，自博弈也 +1.2
    // 开销：置换表 + 深层剪等待 + 80 万节点确定性预算；均值 70µs、峰值 1.1ms
    // （预算 500ms 的 0.2%）。默认 5；互刷 meta 现场可切 1（decision_type）。
    //
    // 2026-10-25 v9 复核（随机出生 + 四车同配，8 块）：
    // depth=5 4 块 top2 75.7%、R2 61.6、reach 84.4%；生产启发式 87.3%/92.9%/94.2%。
    // 原因是深度搜索只要视野内有敌人就接管，固定对手模型对被动
    // 对手也会防御/后撤（约 86 retreat + 57 defend Tick/局），而 D 在 500 已封顶，
    // 多打不涨分。库默认仍是 5（历史 A/B 兼容），生产 main 显式配 0。
    // 见 修改报告 §3.6.6；旧 81.3/87.7/68 是固定角/不对称口径，勿再用。
    // 生产 main.cpp 配 combat_depth=0 + reach_contest=true、engage_gate=false；
    // 深搜保留为可选能力（gate+depth5 84.0%，仍低于生产）。
    int combat_depth = 5;

    // ---- 战斗层两项盲点修复（审查 §三 #1/#2，均有 A/B 开关）----
    // 采集机会成本：交战每 Tick 扣除 Planner 采集计划的边际分（分/Tick，
    // 由 Agent 缓存 planValue/planCost 换算，上限 opp_cost_cap），净亏则脱离。
    // 实测（40 局×5 对手）：对避战流 +2.7、蹲守 +0.9、自博弈 +0.8，但对追击型
    // −3.2（脱离收入被高估——被追打时采不到）。**默认关闭**：需要 P2 对手模型
    // 区分"逃跑 vs 追击"后才能正确启用；管线已预留，可作 decision_type。
    bool combat_opp_cost = false;
    float opp_cost_cap = 1.2f;
    // 排名口径的相对战斗价值（A/B 开关，默认关闭以保持历史口径）：
    // 击杀 = 我方 D 边际 + 对手 60D 的相对损失；死亡 = 我方 D 边际 + 对手
    // 60D 的相对收益 + 回场。D 的边际按 15·min(D,500)/500 计算，因此
    // D≥500 后我方击杀/死亡不再加分，但 D 在 500–560 之间死亡仍有 1.5→0
    // 的分差（规则 §10.2 / v4.4 §6.2）。旧口径在 D>500 后仍按 1.8 计死亡，
    // 且击杀不计对手损失，会让一换一在价值函数里恒为负（偏保守）。
    // **只在 Combat::deepDecide 使用；生产 depth=0 的启发式路径不读取，
    // `rankvalue=1` 在生产是 no-op（2026-10-25 复评逐位相同）**。
    bool combat_rank_value = false;
    // 接战门（Agent 层，2026-10-25 建议书）：经济优先、防钓鱼、防抖动。
    // 三态：SAFE 不接管 Combat；ALERT 只用启发式且给射线加软代价；
    // ENGAGED 才允许深搜。D≥500 且无残血可杀时禁止主动接战。
    // 默认关闭用于 A/B/旧回归；生产 main.cpp **不启用**（启发式+争夺最优，见 §3.6.5），
    // 仅作为深搜的可选保护层（gate+depth5 仍远好于旧 depth5，但略低于启发式）。
    bool engage_gate = false;
    float alert_soft_cost = 6.0f;  // ALERT 射线格的寻路软代价（不封死）
    // 经济接战门（独立于旧 engage_gate）：当采集计划的边际收益足够高，
    // 且本 Tick 没有立即承伤/必杀机会时，先不接战，交给资源/任务链。
    // 目的：修“低 R2 局 D 反而更高”的战斗优先缺陷。
    // 2026-10-27 评测：3×hybrid 1000 局开启后 #1 44.3→47.2%、前二 70.4→73.5%，
    // R2 55.4→58.9；mix3/hunter/mirror 的 500 局配对也均为正，故默认开启。
    bool economy_gate = true;
    float economy_gate_rate = 0.6f;  // 点/Tick（planner_rate_ × 0.25）
    int economy_gate_hp = 40;        // 低于该血量仍允许自保/接战
    // 只有资源还没打满时才用经济门；R2 已接近 120 时继续攻击换 D。
    float economy_gate_r2 = 110.f;
    // P0（2026-10-29）：经济门的 planner_rate_ 必须来自“当前真的在执行的采集
    // 动作”，不能拿一个未被执行的采集计划（探索/等待时）压攻击。
    // 默认关闭用于 A/B；验证为正后翻默认。
    bool economy_gate_actual_action = false;
    // P1（2026-10-29）：只有资源目标在“持续接近”时才允许 planner_rate_ 进入
    // 经济门；连续 economy_gate_stall 个 Tick 没有缩短与目标的曼哈顿距离，
    // 视为路线未执行，经济门关闭（A 至少会还手，而不是对着空气绕圈）。
    bool economy_gate_progress = false;
    int economy_gate_stall = 3;
    // P1（2026-10-29）：探索目标粘性。ep.target 每 Tick 可能因覆盖率打平而
    // 换点，旧逻辑要求 ep.target == target_ 才沿用，导致探索来回摆。
    // 打开后：锁定未过期且旧探索格仍是未知格，就继续沿它走。
    bool explore_sticky = false;
    int explore_commit_min = 0;  // 新探索目标的最小锁定 Tick（0=不额外延长）
    // P3（2026-10-29）：1v1 承伤豁免的“窄口径”版本。只有当最近 3 Tick 真的
    // 掉过血、开火者仍在视野、且 HP 已不高于阈值时才关闭经济门；满血第一下
    // 仍可让资源（即“先吃一下再还手”）。全局 recentHit 豁免在 3hybrid 上
    // 掉前二 2.4pp，不能直接用。
    bool economy_gate_duel = false;
    int economy_gate_duel_hp = 80;
    // P1（2026-10-29）：路径回摆抑制。给“上一 Tick 所在格”加一点软代价，
    // 让等长最短路优先不立即回头；不是硬封死，死路仍可退。
    bool no_reverse = false;
    float no_reverse_cost = 4.0f;
    // P4（2026-10-30，来自实战日志）：no_target 连续多 Tick 时，Planner 会一直
    // 返回 WAIT（日志里出现 108/56/22 Tick 的原地等待，A 最终只有 22 分）。
    // 打开后：连续 N Tick no_target 且还有时间，强制做一次“最大视野增益”
    // 或向地图中心/角落巡逻的一步移动；只影响原本会原地等待的局面。
    bool no_target_fallback = false;
    int no_target_fallback_ticks = 2;
    float no_target_fallback_r2 = 100.f;  // R2 已高时不再兜底巡逻
    // P5（2026-10-30，来自实战资源耗尽日志）：所有已知资源被拿完后，
    // no_target 不应原地等到终局；转为猎杀/争夺（默认开启，只影响此时段）。
    bool post_resource_hunt = true;
    float post_resource_hunt_r2 = 40.f;  // 只在自己没拿到多少资源时猎杀
    // P2（2026-10-29）：经济门压攻击时，给敌车 1–2 格十字射线加软代价，
    // 让资源路线绕开枪口，而不是从 C 旁边走过。0=关闭。
    float threat_soft_cost = 0.f;
    int threat_soft_range = 4;   // 只对距我 ≤ 该值的敌车加射线软代价
    // D 值止盈：有效伤害 ≥500（15 分打满）后，除非能击杀/必死换命，
    // 启发式不再主动攻击；防御/撤退仍允许。默认关闭用于 A/B。
    bool d_cap_gate = true;

    // ---- v1.2 伤害任务（effective_damage）----
    // 伤害任务按“实际攻击伤害 + 击杀奖励 - 死亡扣除”后的有效伤害判定。
    // 任务未完成时不能套用 D≥500 的止盈，否则任务会被“降权”卡住；
    // 同时提高接战意愿：降低撤退/防御阈值、允许更小换血余量、不主动撤退。
    int damage_mission_defend_hp = 20;       // 正常 defend_hp=40；越低越敢打
    int damage_mission_attack_margin = -5;   // 正常 0；负数=允许以小血量差换血
    bool damage_mission_no_retreat = true;   // 伤害任务优先输出，不主动撤退
    // 防御性苟活：hp≤20 且有敌人，或 D≥500 且 R2≥120 时，禁止主动攻击，
    // 并用软阻挡迫使 Planner 远离敌人；默认关闭用于 A/B。
    bool defensive_mode = false;
    int defensive_hp = 20;
    int defensive_radius = 2;
    // 多敌射线：把"非交战敌车"本 Tick 能打到的格子（直线 1–2 格）在搜索的
    // 移动合法性里标记为不可进入——堵住"走向 A 踏进 B 射线"的结构性盲点。
    // 自博弈下多数交战是 1v1（第二敌不在交战区）故分数无变化，属预期；
    // 保留默认开启（混战站位正确性）。
    bool combat_ray_block = true;

    // ---- v1.2 事件利用（规则 §8.5 / §8.7）----
    // 天下无双：公告坐标 + 自己在堡垒内连续驻留 10 Tick → 推断已获得增益，
    // 之后 ATTACK 命中自身横纵 ±2 的 5×5 内全部存活敌人（§8.5，含斜向）。
    // 协议没有 buff 字段，只能靠公告坐标与自身位置推断；默认开启。
    bool unrivaled_attack = true;
    // 火爆辣椒：已站在点位 1–3 步内、且同排/同列有敌人时，主动踩点触发十字火线。
    // 触发者位于交点免疫（§8.7），所以 1 步踩点是“只赚不赔”的 50 环境伤害。
    // 默认开启；A/B 可用 pepper=0 关闭。
    bool pepper_trap = true;
    // 潘潘的神（§8.9）：事件生效后的前 20 Tick，成功移动会给该车附加易伤。
    // 开启后，探索/游走类移动改为原地防御；资源、任务、战斗、事件目标不受影响。
    // 这是“少动”而不是“不动”，避免为了躲易伤丢掉采集/任务。
    bool pan_caution = true;
    // 世界分裂（§8.6）：公告后提前离开十字线，并把临近生效的线标成软阻挡，
    // 避免“站在线上等死”或规划路径穿过即将生效的线。
    bool split_escape = true;
    int split_escape_horizon = 4;  // 生效前多少 Tick 开始避让/逃离

    // ---- 地形拓扑（咽喉/走廊，建议书《地形问题改进》）----
    // 总开关：false 时 World 仍会算拓扑（供诊断），但寻路与决策不用它。
    bool choke_enable = true;
    // 咽喉判定阈值 K：删除该格后"最小一侧"≥ K 才算战略咽喉。
    int choke_min_value = 8;
    // 寻路软惩罚系数：等长路径间避开咽喉。健康时 ×1，残血（hp≤choke_hp）时 ×5。
    float choke_w_corridor = 0.5f;
    int choke_hp = 40;
    // 残血避死门：hp ≤ choke_hp 时，规划第一步若踏入高价值咽喉且有别的
    // 非咽喉可行方向，改走该方向（建议书战术一的轻量版）。
    bool choke_lowhp_gate = true;

    // ---- AI 网关 ----
    // 网关统一模型 deepseek-flash；可用 RMTC_AI_MODEL 覆盖。
    // 请求体固定带 thinking.disabled + response_format.json_object
    // （见 rmtc/event_prompt.cpp），否则上游默认开思考，500ms 窗口来不及。
    std::string llm_model = "deepseek-flash";
    // 资格握手的单次总超时。现场网关冷启动可能比事件慢，可用
    // RMTC_AI_HANDSHAKE_TIMEOUT_MS 覆盖，不必改代码重编译。
    int llm_handshake_timeout_ms = 4500;
};
}  // namespace rmtc
