// rmtc/planner.cpp —— 采集/探索决策器（第一阶段主逻辑，第二阶段继续复用）
//
// 这个规划器只负责 MOVE/WAIT：怎么在有限剩余时间里，把"捡资源"和"开视野"
// 两件事的收益放在同一个标尺上比较，然后每 Tick 输出一步移动。
//
// 整体思路（阅读顺序建议按这个来）：
//   1. decide() 先做三件准备工作：
//        - 用 dijkstra 算出"到任意格子的步数/风险"，一份允许穿未知区、一份只走已知空地；
//        - 用 buildResourceCandidates 选出视野记忆里可达的资源；
//        - 用 planCollect / planExplore* 生成两种候选计划。
//   2. 两个计划各自算"价值/步数"的密度比，取大者；同分优先资源。
//   3. 目标有迟滞（switch_ratio）和锁定窗口（commit_until_），避免每 Tick 换目标来回抖。
//   4. 最后用 firstStep 取路径第一步；收尾阶段还会做"只走确认可达的空地"的保守切换。
//
// 四个防御性机制（都是第一阶段踩坑后加的，别轻易删）：
//   * 自愈：撞进看不见的格子后，把它记为障碍并进入 N Tick 保守模式（只用已知空地）；
//   * 收尾保护：剩余时间不够来回时，只保留"确认走得到"的资源；
//   * 视野半径按感知取真实值（0 是合法值，不能当成缺省 4，否则探索收益会被高估）；
//   * 所有内部缓冲复用，热路径零堆分配（用 alloc_probe 回归）。
//
// 第二阶段注意：本文件不处理 attack/defend——那是 combat.cpp 的职责；
// 这里的 decide 只会在 combat 层"不插手"时被调用。
#include "rmtc/planner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace rmtc {

namespace {

const float kInf = 1e9f;
const float kEps = 1e-4f;

inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// 相邻两格 → 方向编号。编号顺序必须与 kDirs / Combat 保持一致：
// 1=right 2=left 3=down 4=up（坐标系 x 向右、y 向下）。
inline int direction_from_step(int step, int start, int w) {
  const int sx = start % w, sy = start / w;
  const int nx = step % w, ny = step / w;
  if (nx == sx + 1 && ny == sy) return 1;  // 向右
  if (nx == sx - 1 && ny == sy) return 2;  // 向左
  if (ny == sy + 1 && nx == sx) return 3;  // 向下
  if (ny == sy - 1 && nx == sx) return 4;  // 向上
  return 0;
}

const char* kDirs[5] = {NULL, "right", "left", "down", "up"};

}  // namespace

const char* reason_name(int r) {
  switch (r) {
    case R_WAIT_DEAD: return "wait_dead";
    case R_WAIT_TIME: return "wait_time";
    case R_WAIT_ON_RES: return "collect_on_cell";
    case R_WAIT_NO_TARGET: return "no_target";
    case R_WAIT_ALREADY: return "already_here";
    case R_WAIT_NO_SAFE: return "no_safe_step";
    case R_MOVE_RESOURCE: return "resource";
    case R_MOVE_EXPLORE: return "explore";
    case R_MOVE_SAFE: return "safe";
    case R_MOVE_RETREAT: return "retreat";
    case R_MOVE_EVENT: return "event";
    case R_ACT_ATTACK: return "attack";
    case R_ACT_DEFEND: return "defend";
  }
  return "?";
}

const char* direction_name(int d) {
  if (d >= 1 && d <= 4) return kDirs[d];
  return NULL;
}

// 构造：配置只读拷贝；所有运行时状态显式初始化（避免依赖隐式零值）。
// 初始化顺序必须与 planner.h 的成员声明顺序一致（否则 GCC 报 -Wreorder）。
Planner::Planner(const Config& cfg)
    : cfg_(cfg), target_(-1), target_kind_(-1), last_dir_(0), last_target_(-1),
      self_r2_(0.f), path_target_(-1), target_ratio_(0.f), commit_until_(-1),
      attempt_tick_(-1), attempt_x_(0), attempt_y_(0),
      has_attempt_(false), cur_vision_(4), fail_streak_(0), total_failures_(0),
      conservative_until_(-1), plan_exact_calls_(0), plan_greedy_calls_(0) {}

// 换场次调用：只清"本场有效"的状态，缓冲容量保留（下一场继续复用）。
void Planner::reset() {
  target_ = -1;
  target_kind_ = -1;
  target_ratio_ = 0.f;
  commit_until_ = -1;
  last_dir_ = 0;
  last_target_ = -1;
  self_r2_ = 0.f;
  path_.clear();
  path_target_ = -1;
  attempt_tick_ = -1;
  has_attempt_ = false;
  fail_streak_ = 0;
  conservative_until_ = -1;
  plan_exact_calls_ = 0;
  plan_greedy_calls_ = 0;
  race_drops_ = 0;
}

// 登记"我这一 Tick 想进的格子"。
// 注意 MOVE 的语义：目标格 = 当前格 + 方向一步；只用于解释下一帧的 moved=false。
void Planner::noteAction(int tick, int x, int y, const char* direction) {
  if (direction == NULL) {
    has_attempt_ = false;
    return;
  }
  int dx = 0, dy = 0;
  if (!std::strcmp(direction, "up")) dy = -1;
  else if (!std::strcmp(direction, "down")) dy = 1;
  else if (!std::strcmp(direction, "left")) dx = -1;
  else if (!std::strcmp(direction, "right")) dx = 1;
  else {
    has_attempt_ = false;
    return;
  }
  attempt_tick_ = tick;
  attempt_x_ = x + dx;
  attempt_y_ = y + dy;
  has_attempt_ = true;
}

// 消化移动回执。
// 只处理"tick 对得上、且确实是 move"的回执；其它情况（攻击/防御回执、
// 上一场的残留）一律忽略，避免把别的动作解释到移动状态上。
// hardTerrain 由 Agent 归因后传入：地形性失败才写地形记忆并开保守模式。
void Planner::absorbReceipt(const Perception& p, World& world, bool hardTerrain) {
  if (!has_attempt_) return;
  const LastAction& la = p.last_action;
  if (!la.present) return;
  if (la.tick != attempt_tick_ || la.action != "move") {
    has_attempt_ = false;
    return;
  }
  if (!la.moved) {
    // moved=false：目标格进不去。任何失败都先放弃当前目标。
    target_ = -1;
    commit_until_ = -1;
    path_.clear();
    path_target_ = -1;
    if (hardTerrain) {
      // 地形性失败（看不见的格子撞墙）：写回记忆 + 保守模式自愈。
      // 第二阶段由 Agent 先分类：能被敌车占位/争格解释的失败不会走到这里。
      world.markBlocked(attempt_x_, attempt_y_);
      ++fail_streak_;
      ++total_failures_;
      // 任何一次"撞进去"都立刻进入 3 Tick 保守模式：只用已知空地行走。
      // 正常比赛里裁判会报全障碍，这条路径几乎不会触发；
      // 一旦触发，就是"信息缺失"的最后保险，宁可慢 3 Tick 也不再连撞。
      if (cfg_.heal_level >= 2) {
        int until = p.tick + 3;
        if (fail_streak_ >= cfg_.fail_limit) until = p.tick + 8;
        if (until > conservative_until_) conservative_until_ = until;
        if (fail_streak_ >= cfg_.fail_limit) fail_streak_ = 0;
      }
    }
  } else {
    fail_streak_ = 0;
  }
  has_attempt_ = false;
}

// 资源目击越久，价值略高：防止"视野里有资源但一直优先做别的事"把它忘了。
// 线性增长到 age_scale 封顶，倍率上限为 1 + res_age_bonus。
float Planner::ageBonus(int seenTick, int tick) const {
  if (cfg_.res_age_bonus <= 0.f || cfg_.age_scale <= 0.f) return 1.f;
  float age = static_cast<float>(tick - seenTick);
  if (age < 0.f) age = 0.f;
  float t = age / cfg_.age_scale;
  if (t > 1.f) t = 1.f;
  return 1.f + cfg_.res_age_bonus * t;
}

// ---------------------------------------------------------------------------
// 资源候选筛选。
// 输入依赖：dist_ 必须是"从本车当前位置出发"的距离场（decide 里刚算过）。
// 过滤条件：在剩余时间 R 内走得到（不可达或太远的直接丢弃）。
// 价值 = 原始价值 × 高价值加成 × 目击年龄加成；
// dist 与 idx 都参与排序，保证同一局面下的顺序完全确定（离线可复现）。
void Planner::buildResourceCandidates(const World& w, int R, int tick,
                                      const Radar* radar, bool wantNormal,
                                      bool wantHigh) {
  w.resourceList(res_all_);
  cands_.clear();
  for (std::size_t k = 0; k < res_all_.size(); ++k) {
    const World::ResCell& r = res_all_[k];
    // 敌人站在资源上：资源已进入对方的即时收集窗口，规划器不再把它当目标。
    // 这是“资源点被占→放弃”的分类处理；任务点仍由 Agent 的四阶争夺处理。
    if (w.softBlocked(r.idx)) continue;
    ResCand c;
    c.idx = r.idx;
    c.x = r.idx % w.w();
    c.y = r.idx / w.w();
    c.dist = dist_[r.idx];
    if (c.dist > static_cast<float>(R)) continue;
    c.high = (r.value >= cfg_.high_value);
    float v = r.value;
    if (c.high) v *= cfg_.high_priority;
    // 未完成任务所需的资源类型加权：只影响候选价值，不改变真实距离/可达性。
    if ((c.high && wantHigh) || (!c.high && wantNormal))
      v *= cfg_.mission_coupling;
    v *= ageBonus(r.seenTick, tick);
    c.value = v;
    cands_.push_back(c);
  }
  // 确定性排序：先按距离、再按索引
  std::sort(cands_.begin(), cands_.end(),
            [](const ResCand& a, const ResCand& b) {
              if (a.dist != b.dist) return a.dist < b.dist;
              return a.idx < b.idx;
            });

  // P7：明确敌人先到的高价值资源，直接不去。
  // 只认“敌人已经在目标附近（曼哈顿 ≤ radius）且比我们少 margin 步”的
  // 清晰局面；比全局 race_gate（1.35× 时间比）窄得多，避免正常争夺被误杀。
  if (cfg_.high_resource_abandon && radar != NULL &&
      self_r2_ < cfg_.high_resource_abandon_r2) {
    const std::vector<Radar::Sighting>& es = radar->all();
    for (std::size_t i = 0; i < cands_.size();) {
      ResCand& c = cands_[i];
      if (!c.high) {
        ++i;
        continue;
      }
      bool enemyFirst = false;
      for (std::size_t s = 0; s < es.size(); ++s) {
        const Radar::Sighting& sg = es[s];
        if (sg.last_tick < 0 || tick - sg.last_tick > 3) continue;
        const int ed = std::abs(sg.x - c.x) + std::abs(sg.y - c.y);
        if (ed <= cfg_.high_resource_abandon_radius &&
            ed + cfg_.high_resource_abandon_margin <=
                static_cast<int>(c.dist)) {
          enemyFirst = true;
          break;
        }
      }
      if (enemyFirst) {
        ++race_drops_;
        cands_.erase(cands_.begin() + i);
        continue;
      }
      ++i;
    }
  }

  // ---- race 门控（高价值资源的时间竞争，项目全景 §12.9 建议 1）----
  // 敌人到达时间估计 = 最后目击位置做 BFS 的步数 − 目击年龄（下限 0），
  // 即"假设敌人自目击以来以 1 格/Tick 朝该资源移动"的悲观估计；
  // 取所有新鲜目击的最小值——任何一个敌人可能先到都判负。
  // 敌人明显先到（t_enemy × race_margin < t_me）时剔除该候选。
  // 只作用于高价值资源；无新鲜目击（开局）不启用；普通资源不参与。
  if (!cfg_.race_gate || radar == NULL) return;
  const std::vector<Radar::Sighting>& es = radar->all();
  for (std::size_t s = 0; s < es.size(); ++s) {
    const Radar::Sighting& sg = es[s];
    if (sg.last_tick < 0) continue;
    const int age = tick - sg.last_tick;
    if (age > cfg_.race_max_age) continue;
    if (!w.inBounds(sg.x, sg.y)) continue;
    // allowUnknown=true：与本车规划同假设（未知格可通行，风险由别的机制管）。
    w.dijkstraSteps(w.idx(sg.x, sg.y), true, race_steps_, race_parent_);
    for (std::size_t i = 0; i < cands_.size();) {
      ResCand& c = cands_[i];
      if (!c.high) {
        ++i;
        continue;
      }
      const float d = race_steps_[c.idx];
      if (d >= kInf) {
        ++i;
        continue;
      }
      float t = d - static_cast<float>(age);
      if (t < 0.f) t = 0.f;
      if (t * cfg_.race_margin < c.dist) {
        ++race_drops_;
        cands_.erase(cands_.begin() + i);  // 保持距离序，不 swap
        continue;
      }
      ++i;
    }
  }
}

// ---------------------------------------------------------------------------
// 精确巡游：Held-Karp（状态压缩 DP）
//   dp[mask*n + i] = 从起点出发、恰好访问 mask 中的点、最后停在 i 的最短长度
// 只在"点数 ≤ exact_max_n 且地图 ≤ exact_max_cells"时启用；否则 dist 表
// 本身就是 O(n²) 次 dijkstra，开销会失控。
// 目标函数不是"最短"而是"价值最大、同价值取更短"，因此最后要遍历所有 mask。
// 回溯时按 dp 等式反推前驱，得到访问顺序；out.order 存的是 cands_ 的下标。
void Planner::planCollectExact(World& w, const std::vector<float>& d0,
                               const std::vector<float>& dij, int n, int R,
                               CollectPlan& out) {
  (void)w;
  const int total = 1 << n;
  dp_.assign(static_cast<std::size_t>(total) * n, kInf);
  std::vector<float>& dp = dp_;
  for (int i = 0; i < n; ++i) {
    if (d0[i] <= static_cast<float>(R)) {
      const std::size_t idx =
          (static_cast<std::size_t>(1) << i) * n + static_cast<std::size_t>(i);
      dp[idx] = d0[i];
    }
  }
  for (int mask = 1; mask < total; ++mask) {
    const std::size_t base = static_cast<std::size_t>(mask) * n;
    for (int i = 0; i < n; ++i) {
      const float cur = dp[base + i];
      if (cur >= kInf) continue;
      for (int j = 0; j < n; ++j) {
        if (mask & (1 << j)) continue;
        const float nd = cur + dij[static_cast<std::size_t>(i) * n + j];
        const std::size_t idx =
            static_cast<std::size_t>(mask | (1 << j)) * n + j;
        if (nd < dp[idx]) dp[idx] = nd;
      }
    }
  }
  float bestValue = -1.f;
  float bestCost = kInf;
  int bestMask = -1;
  int bestEnd = -1;
  for (int mask = 1; mask < total; ++mask) {
    const std::size_t base = static_cast<std::size_t>(mask) * n;
    float cost = kInf;
    int end = -1;
    for (int i = 0; i < n; ++i) {
      if (dp[base + i] < cost) {
        cost = dp[base + i];
        end = i;
      }
    }
    if (cost > static_cast<float>(R)) continue;
    float value = 0.f;
    for (int i = 0; i < n; ++i) {
      if (mask & (1 << i)) value += cands_[i].value;
    }
    if (value > bestValue + kEps ||
        (std::fabs(value - bestValue) <= kEps && cost < bestCost - kEps)) {
      bestValue = value;
      bestCost = cost;
      bestMask = mask;
      bestEnd = end;
    }
  }
  out.valid = false;
  out.order.clear();
  if (bestMask < 0) return;

  std::vector<int>& rev = rev_;
  rev.clear();
  int mask = bestMask;
  int i = bestEnd;
  int guard = 0;
  while (guard++ < n + 2) {
    rev.push_back(i);
    const int prevMask = mask ^ (1 << i);
    if (prevMask == 0) break;
    const std::size_t base = static_cast<std::size_t>(mask) * n;
    const float tgtCost = dp[base + i];
    int found = -1;
    for (int k = 0; k < n; ++k) {
      if (!(prevMask & (1 << k))) continue;
      const std::size_t pidx = static_cast<std::size_t>(prevMask) * n + k;
      if (dp[pidx] >= kInf) continue;
      const float c = dp[pidx] + dij[static_cast<std::size_t>(k) * n + i];
      if (std::fabs(c - tgtCost) <= 1e-3f) {
        found = k;
        break;
      }
    }
    if (found < 0) break;
    mask = prevMask;
    i = found;
  }
  std::reverse(rev.begin(), rev.end());
  if (rev.empty()) return;
  out.valid = true;
  out.value = bestValue;
  out.cost = bestCost;
  out.order = rev;
  out.first = cands_[rev[0]].idx;
}

// ---------------------------------------------------------------------------
// 多起点贪心链（"一步前瞻"的推广）：对每个可能的首站做贪心续接，
// 每步选"剩余价值 / 剩余距离"最大的下一个点，取总价值最大的链。
// 复杂度 O(n³)，n 较大或地图较大时用它替代精确解。
// 与精确解的取舍：实测两者分数接近（第一阶段 38.76 vs 38.80），
// 但精确解能给出可靠的"整条计划代价"，供 decide 里的动态放弃阈值使用。
void Planner::planCollectGreedy(const std::vector<float>& d0,
                                const std::vector<float>& dij, int n, int R,
                                CollectPlan& out) {
  out.valid = false;
  out.order.clear();
  if (n <= 0) return;
  float bestValue = -1.f;
  float bestCost = kInf;
  int bestFirst = -1;
  std::vector<int>& bestOrder = best_order_;
  std::vector<int>& order = order_;
  std::vector<char>& used = used_;
  bestOrder.clear();
  used.assign(n, 0);
  const float Rf = static_cast<float>(R);

  for (int s = 0; s < n; ++s) {
    if (d0[s] > Rf) continue;
    for (int i = 0; i < n; ++i) used[i] = 0;
    order.clear();
    float cost = d0[s];
    float value = cands_[s].value;
    int cur = s;
    used[s] = 1;
    order.push_back(s);
    for (;;) {
      int nxt = -1;
      float nxtRatio = -1.f;
      float nxtCost = 0.f;
      for (int j = 0; j < n; ++j) {
        if (used[j]) continue;
        const float stp = dij[static_cast<std::size_t>(cur) * n + j];
        const float c = cost + stp;
        if (c > Rf) continue;
        const float r = cands_[j].value / (stp + 0.5f);
        if (r > nxtRatio + 1e-6f ||
            (std::fabs(r - nxtRatio) <= 1e-6f && nxt >= 0 &&
             cands_[j].idx < cands_[nxt].idx)) {
          nxtRatio = r;
          nxt = j;
          nxtCost = c;
        }
      }
      if (nxt < 0) break;
      cost = nxtCost;
      value += cands_[nxt].value;
      used[nxt] = 1;
      order.push_back(nxt);
      cur = nxt;
    }
    if (value > bestValue + kEps ||
        (std::fabs(value - bestValue) <= kEps && cost < bestCost - kEps)) {
      bestValue = value;
      bestCost = cost;
      bestFirst = s;
      bestOrder = order;
    }
  }
  if (bestFirst < 0) return;
  out.valid = true;
  out.value = bestValue;
  out.cost = bestCost;
  out.order = bestOrder;
  out.first = cands_[bestFirst].idx;
}

// ---------------------------------------------------------------------------
// 收集计划入口：先补齐"资源点两两之间"的距离表，再按规模选算法。
// 规模判断：
//   * 地图 ≤ exact_max_cells：两两距离用真实最短路（绕障碍、含未知格风险）；
//   * 更大的地图：退回曼哈顿距离 × 1.15 的近似，避免 O(n²) 次 dijkstra 超时。
// 之后：点数 ≤ exact_max_n 用精确解，否则用贪心链。
// 计算结果写进 out，同时记录执行路径计数（plan_exact_calls_ / plan_greedy_calls_）。
void Planner::planCollect(World& w, int start, int R, CollectPlan& out) {
  (void)start;
  out.valid = false;
  out.value = 0.f;
  out.cost = 0.f;
  out.first = -1;
  out.order.clear();  // 只清空不清容量，避免每 Tick 重新分配
  const int n = static_cast<int>(cands_.size());
  if (n <= 0) return;
  d0_.resize(n);
  for (int i = 0; i < n; ++i) d0_[i] = cands_[i].dist;
  if (n == 1) {
    out.valid = true;
    out.value = cands_[0].value;
    out.cost = cands_[0].dist;
    out.first = cands_[0].idx;
    out.order.push_back(0);
    return;
  }

  // 两两距离：小图用最短路，大图退回曼哈顿（保证大图不会超时）
  const bool useExactDist = (w.cells() <= cfg_.exact_max_cells);
  dij_.assign(static_cast<std::size_t>(n) * static_cast<std::size_t>(n), 0.f);
  std::vector<float>& bdist = bdist_;
  std::vector<float>& brisk = brisk_;
  std::vector<int>& bparent = bparent_;
  for (int i = 0; i < n; ++i) {
    dij_[static_cast<std::size_t>(i) * n + i] = 0.f;
    if (i + 1 >= n) break;
    if (useExactDist) {
      w.dijkstra(cands_[i].idx, cfg_.risk_weight, true, bdist, brisk, bparent);
      for (int j = i + 1; j < n; ++j) {
        float d = bdist[cands_[j].idx];
        if (d >= kInf) d = 1e6f;
        dij_[static_cast<std::size_t>(i) * n + j] = d;
        dij_[static_cast<std::size_t>(j) * n + i] = d;
      }
    } else {
      for (int j = i + 1; j < n; ++j) {
        const int dx = std::abs(cands_[i].x - cands_[j].x);
        const int dy = std::abs(cands_[i].y - cands_[j].y);
        const float d = static_cast<float>(dx + dy) * 1.15f + 0.5f;
        dij_[static_cast<std::size_t>(i) * n + j] = d;
        dij_[static_cast<std::size_t>(j) * n + i] = d;
      }
    }
  }

  const bool canExact = cfg_.exact_op && n <= cfg_.exact_max_n &&
                        w.cells() <= cfg_.exact_max_cells;
  if (canExact) {
    planCollectExact(w, d0_, dij_, n, R, out);
    if (out.valid) {
      ++plan_exact_calls_;
      return;
    }
  }
  planCollectGreedy(d0_, dij_, n, R, out);
  ++plan_greedy_calls_;
}

// ---------------------------------------------------------------------------
// 前沿（连通块）探索。
//
// 做法：把未知格按 4 邻接切块，对每块找"从已探明空地进入它"的入口格，
// 用入口格的距离代替块的到达代价。评分：
//   expVal = explore_weight × hidden × (块大小 / 全部未知格数)
//   cost   = 到入口的步数 + region_traversal × sqrt(块大小)   // 大块需要更久逛完
//   kappa  = clamp((R - cost - 收尾预留) / explore_horizon, 0, 1)
//   ratio  = expVal × kappa × rate / cost
// kappa 的作用：剩余时间不足以支撑"跑过去 + 逛完"时，价值按比例衰减到 0。
// 第一阶段实测：这个模式比逐格增益略差（-0.16），保留它是因为它能处理
// "整块未知区被挡住、逐格评估会一直选边缘小缝"的情况。
void Planner::planExploreFrontier(World& w, int R, int tick, int visionR,
                                  float hidden, int unseen, float rate,
                                  float stopR, ExplorePlan& out) {
  (void)tick;
  (void)stopR;
  (void)visionR;
  out.valid = false;
  out.value = 0.f;
  out.cost = 0.f;
  out.ratio = 0.f;
  out.target = -1;
  out.region = -1;
  if (unseen <= 0 || hidden <= 0.f || rate <= 0.f) return;

  w.labelRegions(label_, region_size_, cfg_.min_region);
  const int regions = static_cast<int>(region_size_.size());
  if (regions <= 0) return;

  region_entry_.assign(regions, -1);
  region_dist_.assign(regions, kInf);

  const std::vector<unsigned char>& st = w.state();
  const int W = w.w(), H = w.h();
  for (int i = 0; i < W * H; ++i) {
    if (st[i] != World::UNKNOWN) continue;
    const int lab = label_[i];
    if (lab < 0) continue;
    const int x = i % W, y = i / W;
    int nb[4];
    int cnt = 0;
    if (x > 0) nb[cnt++] = i - 1;
    if (x + 1 < W) nb[cnt++] = i + 1;
    if (y > 0) nb[cnt++] = i - W;
    if (y + 1 < H) nb[cnt++] = i + W;
    for (int k = 0; k < cnt; ++k) {
      const int j = nb[k];
      if (st[j] != World::FREE) continue;
      const float d = dist_[j];
      if (d < region_dist_[lab]) {
        region_dist_[lab] = d;
        region_entry_[lab] = j;
      }
    }
  }

  const float lead = static_cast<float>(cfg_.strict_finish_ticks + 1);
  const float tau = cfg_.explore_horizon > 0.5f ? cfg_.explore_horizon : 0.5f;
  float best = -1.f;
  for (int r = 0; r < regions; ++r) {
    if (region_size_[r] <= 0 || region_entry_[r] < 0) continue;
    const float de = region_dist_[r];
    if (de > static_cast<float>(R)) continue;
    const float kappa =
        clampf((static_cast<float>(R) - de - lead) / tau, 0.f, 1.f);
    if (kappa <= 0.f) continue;
    const float sz = static_cast<float>(region_size_[r]);
    const float expVal =
        cfg_.explore_weight * hidden * (sz / static_cast<float>(unseen));
    const float traversal = cfg_.region_traversal * std::sqrt(sz);
    const float cost = de + traversal;
    if (cost <= 0.01f) continue;
    const float ratio = expVal * kappa * rate / cost;
    if (ratio > best + 1e-9f ||
        (std::fabs(ratio - best) <= 1e-9f && out.target >= 0 &&
         region_entry_[r] < out.target)) {
      best = ratio;
      out.valid = true;
      out.ratio = ratio;
      out.value = expVal * kappa * rate;
      out.cost = cost;
      out.target = region_entry_[r];
      out.region = r;
    }
  }
  if (out.valid && dist_[out.target] > static_cast<float>(R)) out.valid = false;
}

// ---------------------------------------------------------------------------
// 逐格覆盖增益（第一阶段 v1 的主探索算法，实测比前沿模式略好）。
//
// 对每个可达格 i 估"站在 i 能用视野覆盖掉多少未知格"：
//   gap   = hidden / unseen × explore_weight × rate      // 一个未知格的期望价值
//   value = gap × (gain(i) + corridor_gain × dist(i)) × kappa
//   ratio = value / dist(i)
// 其中 gain(i) 是半径 visionR 的方形窗口内的未知格数量（前缀和 O(1)）；
// corridor_gain 让"顺路多走几步"也有收益，避免总选贴着墙的格子。
//
// 区域加权（region_gamma>0）：把每个未知格的权重乘 (块大小/region_ref)^gamma，
// 这样"大块未知区的边缘"比"只隔一条缝的小块"更值得去。
// 第一阶段实测这项是负收益（-0.17），默认值保留但可以通过 region_gamma=0 关闭。
void Planner::planExploreGain(World& w, int R, int tick, int visionR,
                              float hidden, int unseen, float rate,
                              float stopR, bool hasRes, ExplorePlan& out) {
  (void)tick;
  (void)stopR;
  out.valid = false;
  out.value = 0.f;
  out.cost = 0.f;
  out.ratio = 0.f;
  out.target = -1;
  out.region = -1;
  if (unseen <= 0 || hidden <= 0.f || rate <= 0.f) return;

  if (cfg_.region_gamma > 0.01f) {
    w.labelRegions(label_, region_size_, cfg_.min_region);
    const int n = w.cells();
    cell_weight_.assign(n, 0.f);
    const std::vector<unsigned char>& st0 = w.state();
    const float ref = cfg_.region_ref > 1.f ? cfg_.region_ref : 20.f;
    for (int i = 0; i < n; ++i) {
      if (st0[i] != World::UNKNOWN) continue;
      const int lab = label_[i];
      if (lab < 0) continue;
      float sz = static_cast<float>(region_size_[lab]);
      if (sz < 1.f) sz = 1.f;
      float wt = std::pow(sz / ref, cfg_.region_gamma);
      if (wt < 0.25f) wt = 0.25f;
      if (wt > 4.f) wt = 4.f;
      cell_weight_[i] = wt;
    }
    w.coverageGainWeighted(visionR, &cell_weight_[0], gainf_);
  } else {
    w.coverageGainWeighted(visionR, NULL, gainf_);
  }

  const float gap =
      hidden / static_cast<float>(unseen) * cfg_.explore_weight * rate;
  const float lead = static_cast<float>(cfg_.strict_finish_ticks + 1);
  const float tau = cfg_.explore_horizon > 0.5f ? cfg_.explore_horizon : 0.5f;
  const int n = w.cells();
  const std::vector<unsigned char>& st = w.state();
  float best = -1.f;
  for (int i = 0; i < n; ++i) {
    if (st[i] == World::BLOCKED) continue;
    const float g = gainf_[i];
    // 手上没有可达资源时，连 1 格的增益也值得去（机会成本为 0）；
    // 否则 vision_radius=0 这种退化情形会被 min_gain 直接卡死、原地不动。
    if (g < static_cast<float>(hasRes ? cfg_.min_gain : 1)) continue;
    const float d = dist_[i];
    if (d > static_cast<float>(R)) continue;
    const float kappa =
        clampf((static_cast<float>(R) - d - lead) / tau, 0.f, 1.f);
    if (kappa <= 0.f) continue;
    const float val = gap * (g + cfg_.corridor_gain * d) * kappa;
    const float ratio = val / (d > 1e-6f ? d : 1.f);
    if (ratio > best + 1e-9f ||
        (std::fabs(ratio - best) <= 1e-9f && out.target >= 0 && i < out.target)) {
      best = ratio;
      out.valid = true;
      out.ratio = ratio;
      out.value = val;
      out.cost = d;
      out.target = i;
    }
  }
}

// ---------------------------------------------------------------------------
// 主决策：每 Tick 调用一次（由 Agent 在 combat 层不插手时调用）。
//
// 输入：World（地图/资源记忆，已被本帧感知更新）、Perception（本帧自身状态）。
// 输出：MOVE_MOVE 或 MOVE_WAIT；不会返回 ATTACK/DEFEND（那是 combat 的职责）。
//
// 步骤：
//   A. 早退：阵亡 / 时间耗尽 / 正站在资源格上（等裁判收集，避免原地抖动）
//   B. 两个距离场：允许穿未知区 / 只走已知空地；保守模式（自愈中）用后者
//   C. 估计 hidden（还没看见的资源价值）与未知格数
//   D. 生成采集计划 + 探索计划，各算价值密度
//   E. 比较密度选目标（带迟滞与锁定窗口），处理"无目标"
//   F. 取路径第一步，必要时切到保守路径；最后做收尾安全判断
Decision Planner::decide(World& w, const Perception& p, const Radar* radar) {
  Decision d;
  const int tick = p.tick;
  const int R = cfg_.total_ticks - tick;  // 剩余 Tick 数（第一/第二阶段由 Config 决定）
  self_r2_ = p.resource_value;

  // ---- A. 早退 ----
  if (!p.alive) {
    d.reason = R_WAIT_DEAD;
    return d;
  }
  if (R <= 0) {
    d.reason = R_WAIT_TIME;
    return d;
  }
  const int start = w.idx(p.x, p.y);
  // 站在资源格上：收集是到点自动结算的，这一 Tick 原地等待即可。
  // （第二阶段可以把这里改成"继续走"，因为收集不需要停留；保留等待是为了
  //   兼容第一阶段的节奏，且能避免刚踩上去就离开导致的视觉抖动。）
  if (w.resValue(start) > 0.f) {
    target_ = -1;
    commit_until_ = -1;
    d.reason = R_WAIT_ON_RES;
    return d;
  }

  // ---- B. 距离场 ----
  // conservative：自愈窗口内（或配置禁止穿未知区）时，本 Tick 只用已知空地。
  const bool conservative =
      (cfg_.heal_level >= 1) &&
      ((tick < conservative_until_) || !cfg_.allow_unknown_travel);
  cur_vision_ = p.vision_radius;
  if (cur_vision_ < 0) cur_vision_ = 0;
  if (cur_vision_ > cfg_.vision_max) cur_vision_ = cfg_.vision_max;

  // 咽喉软惩罚（建议书战术二）：只加在风险次级键上，步数保持真实值，
  // "剩余时间 R 内可达"的可行性判断不受影响；等长路径间优先绕开咽喉。
  const float chokeRisk =
      cfg_.choke_enable
          ? cfg_.choke_w_corridor * (p.hp <= cfg_.choke_hp ? 5.f : 1.f)
          : 0.f;
  w.dijkstra(start, cfg_.risk_weight, cfg_.allow_unknown_travel, dist_, risk_,
             parent_, chokeRisk);
  w.dijkstra(start, cfg_.risk_weight, false, dist_safe_, risk_safe_, parent_safe_,
             chokeRisk);
  // 保守模式（自愈）：本 Tick 只用"已知空地"作为可行域。
  // 用 swap 让后面所有逻辑无差别复用（O(1)，不复制数组）。
  if (conservative) {
    dist_.swap(dist_safe_);
    parent_.swap(parent_safe_);
  }
  // 视野半径就是 cur_vision_：0 是合法值（规则 §4.1 允许 0–4），
  // 绝不能把 0 当成"缺省 4"，否则会按 9x9 估算覆盖增益，探索目标持续高估。
  const int visionR = cur_vision_;

  // ---- C. 全局估计 ----
  int unk = 0, freeCells = 0, blk = 0;
  w.counts(unk, freeCells, blk);
  // hidden：规则给出资源总量，减去已得分与"记忆里还看得见的资源"，
  // 就是"还没被发现/被捡走的资源价值"，用来给探索定价值。
  float hidden = cfg_.total_value - p.resource_value - w.resValueSum();
  if (hidden < 0.f) hidden = 0.f;
  d.hidden = hidden;

  // ---- D. 采集计划 ----
  // 未完成的资源类任务：把对应类型资源加权，避免“只挑高分远点”导致
  // 2 普通 / 1 高价值任务白丢（任务 5 分/个，且影响同分排名）。
  bool wantNormal = false, wantHigh = false;
  for (std::size_t mi = 0; mi < p.missions.size(); ++mi) {
    const Mission& m = p.missions[mi];
    bool done = false;
    for (std::size_t k = 0; k < p.mission_completed_ids.size(); ++k)
      if (p.mission_completed_ids[k] == m.id) { done = true; break; }
    if (done) continue;
    if (m.type == "normal_resources" && m.target > 0) wantNormal = true;
    if (m.type == "high_resources" && m.target > 0) wantHigh = true;
  }
  buildResourceCandidates(w, R, tick, radar, wantNormal, wantHigh);
  // 收尾阶段：剩余时间极紧时，只保留"确认走得到"的资源。
  // 用原地压缩而不是另开缓冲 + swap —— swap 会把容量换走，下一 Tick 又要扩容。
  if (cfg_.strict_finish_ticks > 0 && !conservative) {
    std::size_t wpos = 0;
    for (std::size_t i = 0; i < cands_.size(); ++i) {
      const ResCand& c = cands_[i];
      if ((static_cast<float>(R) - c.dist) <=
          static_cast<float>(cfg_.strict_finish_ticks)) {
        if (dist_safe_[c.idx] > static_cast<float>(R)) continue;
      }
      if (wpos != i) cands_[wpos] = cands_[i];
      ++wpos;
    }
    cands_.resize(wpos);
  }

  CollectPlan& cp = cp_;
  planCollect(w, start, R, cp);

  // ---- E. 探索强度与动态放弃阈值 ----
  // 直觉：如果手上的收集计划本身就要花 need 步，那么"再跑远路去开视野"
  // 的机会成本很高——所以当剩余时间 R 逼近 need 时，探索强度降到 0。
  // stopR 就是这条"停止探索"的剩余时间水位线，默认在水位线以下不再探索。
  float stopR = static_cast<float>(cfg_.explore_stop_R);
  if (cfg_.dynamic_stop) {
    const float need = cp.valid ? cp.cost : 0.f;
    stopR = need + static_cast<float>(cfg_.collect_margin);
    if (stopR < static_cast<float>(cfg_.min_stop))
      stopR = static_cast<float>(cfg_.min_stop);
    if (stopR > static_cast<float>(cfg_.max_stop))
      stopR = static_cast<float>(cfg_.max_stop);
  }
  // rate 是探索强度的 0–1 斜坡：R 从 stopR 到 explore_full_R 之间线性上升。
  const float ramp = static_cast<float>(cfg_.explore_full_R) - stopR;
  float rate = (ramp > 6.f) ? (static_cast<float>(R) - stopR) / ramp : 1.f;
  rate = clampf(rate, 0.f, 1.f);
  if (!cp.valid) rate = 1.f;  // 没有可达资源：探索的机会成本为 0
  // 收尾独立模式：最后 endgame_R 个 Tick 若还有可达资源，就完全不再探索
  if (cfg_.endgame_R > 0 && R <= cfg_.endgame_R && cp.valid) rate = 0.f;
  d.rate = rate;
  d.stopR = stopR;

  // 收集侧的"价值密度"：
  //   plan_level  = 整条收集计划的价值 / 总步数（默认，见建议 2）
  //   first_leg   = 只算首站 v/d（v1 行为，保留用于 A/B）
  float ratioRes = 0.f;
  if (cp.valid && cp.cost > 0.01f) {
    ratioRes = cp.value / cp.cost;
    if (!cfg_.plan_level_ratio) {
      for (std::size_t i = 0; i < cands_.size(); ++i) {
        if (cands_[i].idx == cp.first) {
          ratioRes = cands_[i].value /
                     (cands_[i].dist > 0.1f ? cands_[i].dist : 1.f);
          break;
        }
      }
    }
  }
  d.ratioRes = ratioRes;
  d.planValue = cp.value;
  d.planCost = cp.cost;

  // ---- F. 探索候选 ----
  // 只有"还有未看见的资源价值、还有未知格、探索强度 > 0"时才评估探索。
  // 先试配置指定的算法；前沿模式失败时退回逐格增益，保证不会因为
  // "所有未知块都太小"而白白放弃探索。
  ExplorePlan ep;
  if (rate > 0.f && hidden > 0.f && unk > 0) {
    if (cfg_.use_frontier)
      planExploreFrontier(w, R, tick, visionR, hidden, unk, rate, stopR, ep);
    else
      planExploreGain(w, R, tick, visionR, hidden, unk, rate, stopR,
                      cp.valid && cp.first >= 0, ep);
    if (!ep.valid && cfg_.use_frontier)
      planExploreGain(w, R, tick, visionR, hidden, unk, rate, stopR,
                      cp.valid && cp.first >= 0, ep);
  }
  d.ratioExp = ep.ratio;
  d.exploreRatio = ep.ratio;

  // ---- G. 目标选择：刷新 or 沿用 ----
  // 沿用条件：锁定未过期，且旧目标仍可行（资源还在 / 探索目标仍是同一个）。
  bool useCommitted = false;
  if (target_ >= 0 && tick < commit_until_) {
    if (target_kind_ == 0) {
      if (w.resValue(target_) > 0.f &&
          dist_[target_] <= static_cast<float>(R) && dist_[target_] > 0.f) {
        useCommitted = true;
      }
    } else {
      if (ep.valid && ep.target == target_) {
        useCommitted = true;
      } else if (cfg_.explore_sticky && target_ >= 0 &&
                 target_ < w.w() * w.h() &&
                 w.state()[target_] == World::UNKNOWN &&
                 dist_[target_] <= static_cast<float>(R) &&
                 dist_[target_] > 0.f) {
        // 旧探索格仍是未知格且可达：继续沿它走，不被 ep.target 的
        // 覆盖率打平来回换点。
        useCommitted = true;
      }
    }
  }

  int tgt = -1;
  int kind = -1;
  float chosenRatio = 0.f;
  if (useCommitted) {
    tgt = target_;
    kind = target_kind_;
    chosenRatio = target_ratio_;
  } else {
    const bool haveRes = cp.valid && cp.first >= 0;
    const bool haveExp = ep.valid;
    if (!haveRes && !haveExp) {
      target_ = -1;
      commit_until_ = -1;
      d.reason = R_WAIT_NO_TARGET;
      return d;
    }
    if (haveRes && (!haveExp || ratioRes >= ep.ratio)) {
      tgt = cp.first;
      kind = 0;
      chosenRatio = ratioRes;
    } else {
      tgt = ep.target;
      kind = 1;
      chosenRatio = ep.ratio;
    }
    // 迟滞：旧目标仍可选时，只有新目标明显更优才切换。
    // 注意口径：必须用"选中它当时那个分数"（target_ratio_）来比，
    // 不能拿整条巡游的价值/步数 去和 单个资源的 v/d 比 —— 那是两个量纲。
    if (target_ >= 0 && target_ != tgt && commit_until_ > 0 && target_ratio_ > 0.f) {
      bool oldStillFeasible = false;
      if (target_kind_ == 0) {
        for (std::size_t i = 0; i < cands_.size(); ++i) {
          if (cands_[i].idx == target_) {
            oldStillFeasible = true;
            break;
          }
        }
      } else if (ep.valid && ep.target == target_) {
        oldStillFeasible = true;
      }
      if (oldStillFeasible && chosenRatio < cfg_.switch_ratio * target_ratio_) {
        tgt = target_;
        kind = target_kind_;
        chosenRatio = target_ratio_;
      }
    }
    target_ = tgt;
    target_kind_ = kind;
    target_ratio_ = chosenRatio;
    const float td0 = dist_[tgt];
    int commitLen = static_cast<int>(td0);
    if (kind == 1 && cfg_.explore_commit_min > commitLen)
      commitLen = cfg_.explore_commit_min;
    if (commitLen > cfg_.submit_max) commitLen = cfg_.submit_max;
    if (commitLen < 1) commitLen = 1;
    commit_until_ = tick + commitLen;
  }
  if (tgt < 0) {
    d.reason = R_WAIT_NO_TARGET;
    return d;
  }

  // ---- H. 输出第一步 ----
  // 三重保险：
  //   1. 目标不可达 → 放弃目标；
  //   2. 收尾阶段（剩余时间 - 步数 ≤ strict_finish_ticks）切到"只走已知空地"的路径；
  //   3. 最后 1–2 Tick 若第一步还是未知格且目标还远，直接放弃（失败没有补救机会）。
  float td = dist_[tgt];
  if (td >= kInf) {
    target_ = -1;
    commit_until_ = -1;
    d.reason = R_WAIT_NO_TARGET;
    return d;
  }
  const std::vector<int>* par = &parent_;
  bool safeMode = false;
  if (!conservative && cfg_.strict_finish_ticks > 0 &&
      (static_cast<float>(R) - td) <=
          static_cast<float>(cfg_.strict_finish_ticks) &&
      dist_safe_[tgt] <= static_cast<float>(R)) {
    safeMode = true;
  }
  if (safeMode) {
    par = &parent_safe_;
    td = dist_safe_[tgt];
  }
  if (!cfg_.planner_path_commit || tgt != path_target_) {
    path_.clear();
    path_target_ = -1;
  }
  int step = -1;
  if (cfg_.planner_path_commit && tgt == path_target_ && !path_.empty()) {
    if (path_.front() == start) path_.erase(path_.begin());
    if (!path_.empty()) {
      const int ni = path_.front();
      const int nx = ni % w.w(), ny = ni / w.w();
      const int sx = start % w.w(), sy = start / w.w();
      const int md = std::abs(nx - sx) + std::abs(ny - sy);
      if (md == 1 && w.state()[ni] != World::BLOCKED &&
          !w.softBlocked(ni) && dist_[ni] <= dist_[start] + 0.5f) {
        step = ni;
      } else {
        path_.clear();
        path_target_ = -1;
      }
    }
  }
  if (step < 0) {
    step = w.firstStep(*par, tgt, start);
    if (step >= 0 && cfg_.planner_path_commit) {
      // 锁定一条完整路径：下一 Tick 沿用同一序列，避免等长最短路
      // 在 (6,7)↔(6,8) 之间每 Tick 换第一步。
      path_.clear();
      int cur = tgt;
      int guard = 0;
      while (cur >= 0 && cur != start && guard++ < w.cells()) {
        path_.push_back(cur);
        cur = (*par)[cur];
      }
      if (cur == start) {
        std::reverse(path_.begin(), path_.end());
        if (!path_.empty() && path_.front() == start)
          path_.erase(path_.begin());
        path_target_ = tgt;
      } else {
        path_.clear();
        path_target_ = -1;
      }
    } else if (step >= 0) {
      path_.clear();
      path_target_ = -1;
    }
  }
  if (step < 0) {
    target_ = -1;
    commit_until_ = -1;
    path_.clear();
    path_target_ = -1;
    d.reason = R_WAIT_ALREADY;
    return d;
  }
  // P8：资源目标不允许第一步在曼哈顿距离上“远离”目标。
  // 这次日志里目标 (8,3) 在上方，A 却在 (6,7) 选 down 去 (6,8)；
  // 只要存在合法邻格更近，就强制改走更近的。
  if (cfg_.planner_progress_guard && target_kind_ == 0) {
    const int tx = tgt % w.w(), ty = tgt / w.w();
    const int sx = start % w.w(), sy = start / w.w();
    const int curM = std::abs(tx - sx) + std::abs(ty - sy);
    const int stepM = std::abs(tx - step % w.w()) + std::abs(ty - step / w.w());
    if (stepM > curM) {
      const int ddx[5] = {0, 1, -1, 0, 0};
      const int ddy[5] = {0, 0, 0, 1, -1};
      int altStep = -1;
      int altM = stepM;
      for (int nd = 1; nd <= 4; ++nd) {
        const int nx = sx + ddx[nd];
        const int ny = sy + ddy[nd];
        if (!w.inBounds(nx, ny)) continue;
        const int ni = w.idx(nx, ny);
        if (w.state()[ni] == World::BLOCKED || w.softBlocked(ni)) continue;
        const int m = std::abs(tx - nx) + std::abs(ty - ny);
        if (m < altM) {
          altM = m;
          altStep = ni;
        }
      }
      if (altStep >= 0) {
        step = altStep;
        path_.clear();
        path_target_ = -1;
      }
    }
  }
  const bool stepKnownFree = (w.state()[step] == World::FREE);
  if (!stepKnownFree && conservative) {
    // 保守模式下距离场本身就只走已知空地，走到这里说明无路可走
    target_ = -1;
    commit_until_ = -1;
    d.reason = R_WAIT_NO_SAFE;
    return d;
  }
  if (!stepKnownFree && !cfg_.allow_unknown_travel) {
    const int alt = w.firstStep(parent_safe_, tgt, start);
    if (alt >= 0 && w.state()[alt] == World::FREE) {
      step = alt;
      td = dist_safe_[tgt];
    }
  }
  // 最后 1-2 Tick：若目标还远且第一步进未知格，失败就没有补救 → 放弃
  if (cfg_.heal_level >= 1 && R <= 2 && !stepKnownFree &&
      dist_[tgt] > 1.5f && w.resValue(tgt) > 0.f) {
    target_ = -1;
    commit_until_ = -1;
    d.reason = R_WAIT_NO_SAFE;
    return d;
  }

  int outDir = direction_from_step(step, start, w.w());
  if (outDir == 0) {
    target_ = -1;
    commit_until_ = -1;
    d.reason = R_WAIT_ALREADY;
    return d;
  }
  // P6：同一目标下，若第一步正好是上一步的反方向，优先选一个更接近
  // 目标的替代邻格，打断 (6,7)↔(6,8) 这类等长最短路回摆。
  if (cfg_.planner_anti_reverse && tgt == last_target_ && last_dir_ != 0) {
    const int rev = (last_dir_ == 1) ? 2 : (last_dir_ == 2) ? 1
                    : (last_dir_ == 3) ? 4 : 3;
    if (outDir == rev) {
      const int ddx[5] = {0, 1, -1, 0, 0};
      const int ddy[5] = {0, 0, 0, 1, -1};
      int altDir = 0;
      float altDist = kInf;
      for (int nd = 1; nd <= 4; ++nd) {
        if (nd == rev) continue;
        const int nx = start % w.w() + ddx[nd];
        const int ny = start / w.w() + ddy[nd];
        if (!w.inBounds(nx, ny)) continue;
        const int ni = w.idx(nx, ny);
        if (w.state()[ni] == World::BLOCKED || w.softBlocked(ni)) continue;
        if (dist_[ni] >= dist_[start] - 0.5f) continue;
        if (dist_[ni] < altDist) {
          altDist = dist_[ni];
          altDir = nd;
        }
      }
      if (altDir != 0) {
        outDir = altDir;
        step = w.idx(start % w.w() + ddx[altDir], start / w.w() + ddy[altDir]);
      }
    }
  }
  d.kind = MOVE_MOVE;
  d.direction = kDirs[outDir];
  d.targetIdx = tgt;
  d.reason = (target_kind_ == 0)
                 ? R_MOVE_RESOURCE
                 : (w.state()[step] == World::FREE ? R_MOVE_EXPLORE : R_MOVE_SAFE);
  d.commitUntil = commit_until_;
  last_dir_ = outDir;
  last_target_ = tgt;
  return d;
}

}  // namespace rmtc

