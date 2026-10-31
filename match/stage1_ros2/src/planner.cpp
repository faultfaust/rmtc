// rmtc/planner.cpp —— 决策器实现（统一估值 + 滚动时域 + 自愈）
// [FIX] 版本：含语雀实测丢分问题的四项修复（开关见 config.h，标注 [FIX-A/B/C/D]）
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
  }
  return "?";
}

Planner::Planner(const Config& cfg)
    : cfg_(cfg), target_(-1), target_kind_(-1), target_ratio_(0.f),
      commit_until_(-1), attempt_tick_(-1), attempt_x_(0), attempt_y_(0),
      has_attempt_(false), cur_vision_(4), fail_streak_(0), total_failures_(0),
      conservative_until_(-1), plan_exact_calls_(0), plan_greedy_calls_(0),
      lam_(cfg.lam_init), j_tgt_(-1), j_est_(0.f), j_tick0_(0), excl_value_(0.f) {}

void Planner::reset() {
  target_ = -1;
  target_kind_ = -1;
  target_ratio_ = 0.f;
  commit_until_ = -1;
  attempt_tick_ = -1;
  has_attempt_ = false;
  fail_streak_ = 0;
  conservative_until_ = -1;
  plan_exact_calls_ = 0;
  plan_greedy_calls_ = 0;
  lam_ = cfg_.lam_init;   // [FIX-B]
  j_tgt_ = -1;            // [FIX-B]
  j_est_ = 0.f;           // [FIX-B]
  j_tick0_ = 0;           // [FIX-B]
  excl_value_ = 0.f;      // [FIX-D]
  excl_mask_.clear();     // [FIX-D]
}

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

void Planner::absorbReceipt(const Perception& p, World& world) {
  if (!has_attempt_) return;
  const LastAction& la = p.last_action;
  if (!la.present) return;
  if (la.tick != attempt_tick_ || la.action != "move") {
    has_attempt_ = false;
    return;
  }
  if (!la.moved) {
    // moved=false：目标格进不去（障碍/占位/越界）→ 写回记忆
    const bool terrain = la.error.empty() || la.error == "E_BLOCKED" ||
                         la.error == "E_OOB";
    if (terrain) {
      world.markBlocked(attempt_x_, attempt_y_);
      ++fail_streak_;
      ++total_failures_;
      target_ = -1;
      commit_until_ = -1;
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

float Planner::ageBonus(int seenTick, int tick) const {
  if (cfg_.res_age_bonus <= 0.f || cfg_.age_scale <= 0.f) return 1.f;
  float age = static_cast<float>(tick - seenTick);
  if (age < 0.f) age = 0.f;
  float t = age / cfg_.age_scale;
  if (t > 1.f) t = 1.f;
  return 1.f + cfg_.res_age_bonus * t;
}

// ---------------------------------------------------------------------------
void Planner::buildResourceCandidates(const World& w, int R, int tick) {
  w.resourceList(res_all_);
  cands_.clear();
  for (std::size_t k = 0; k < res_all_.size(); ++k) {
    const World::ResCell& r = res_all_[k];
    if (cfg_.fix_enclosed && r.idx < static_cast<int>(excl_mask_.size()) &&
        excl_mask_[r.idx]) {
      continue;  // [FIX-D] 围死资源不再进入候选
    }
    ResCand c;
    c.idx = r.idx;
    c.x = r.idx % w.w();
    c.y = r.idx / w.w();
    // [FIX-E] 收集规划用风险加权距离：未知格按 +risk_step_cost 步计价，
    // 防止"墙还没看见"时把穿墙捷径当成 2 步近路（实测绕行 18 tick 的根源）
    c.dist = cfg_.fix_riskdist && r.idx < static_cast<int>(eff_.size())
                 ? eff_[r.idx]
                 : dist_[r.idx];
    if (c.dist > static_cast<float>(R)) continue;
    float v = r.value;
    if (v >= cfg_.high_value) v *= cfg_.high_priority;
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
}

// ---------------------------------------------------------------------------
// 精确定向越野：Held-Karp
//   dp[mask*n + i] = 从起点出发、恰好访问 mask、最后停在 i 的最短长度
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
// 多起点贪心链（"一步前瞻"的推广）：对每个可能的首站做贪心续接，取总价值最大者。
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
        // [FIX-E] 两两距离同样按未知格加价，系数 (λ-1)：预测准时趋零
        if (cfg_.fix_riskdist) {
          const float lamEff = (cfg_.fix_lambda || cfg_.fix_endgame) ? lam_ : 1.f;
          d += brisk[cands_[j].idx] *
               ((lamEff - 1.f) * cfg_.risk_step_cost / cfg_.risk_weight);
        }
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
      // [FIX-G] 探索成本用风险加权距离：穿未知区的"近路"按实测绕路率加价
      const float d = (cfg_.fix_explore_eff && j < static_cast<int>(eff_.size()))
                          ? eff_[j]
                          : dist_[j];
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
// 逐格覆盖增益（v1 行为 + 区域加权）
// 区域加权：把每个未探索格的价值按它所属连通块的大小放大/缩小，
// 既保留"站在哪能看见最多"的局部视角，又避免为几条小缝隙来回跑。
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
    // [FIX-G] 探索成本用风险加权距离（预测准时 eff≈dist，无扰动）
    const float d = (cfg_.fix_explore_eff && i < static_cast<int>(eff_.size()))
                        ? eff_[i]
                        : dist_[i];
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
Decision Planner::decide(World& w, const Perception& p) {
  Decision d;
  const int tick = p.tick;
  const int R = cfg_.total_ticks - tick;

  if (!p.alive) {
    d.reason = R_WAIT_DEAD;
    return d;
  }
  if (R <= 0) {
    d.reason = R_WAIT_TIME;
    return d;
  }
  const int start = w.idx(p.x, p.y);
  // ---- [FIX-B] λ（行程可信度）更新：按"完整行程"统计 ----
  // 出发时记下预估距离，抵达目标时用 实际tick/预估 距离 修正 λ（EMA）。
  // λ=1 表示预测完全可信；λ 大表示未知区捷径严重失真（蛇形/竖柱）。
  // 加价系数用 (λ-1)：预测准的图加价自动趋零，不会伤害本来就准的巡游。
  // （必须放在"踩到资源提前 return"之前，否则抵达那拍检测不到。）
  if ((cfg_.fix_lambda || cfg_.fix_endgame) && j_tgt_ >= 0) {
    if (start == j_tgt_) {
      const int actual = tick - j_tick0_;
      const float ratio = static_cast<float>(actual) /
                          (j_est_ > 1.f ? j_est_ : 1.f);
      if (actual >= 2)
        lam_ = clampf(0.55f * lam_ + 0.45f * ratio, 1.f, 3.f);
      j_tgt_ = -1;
    } else if (target_ != j_tgt_) {
      j_tgt_ = -1;  // 中途换目标，本次行程作废
    }
  }
  if (w.resValue(start) > 0.f) {
    target_ = -1;
    commit_until_ = -1;
    d.reason = R_WAIT_ON_RES;
    return d;
  }

  const bool conservative =
      (cfg_.heal_level >= 1) &&
      ((tick < conservative_until_) || !cfg_.allow_unknown_travel);
  cur_vision_ = p.vision_radius;
  if (cur_vision_ < 0) cur_vision_ = 0;
  if (cur_vision_ > cfg_.vision_max) cur_vision_ = cfg_.vision_max;

  w.dijkstra(start, cfg_.risk_weight, cfg_.allow_unknown_travel, dist_, risk_, parent_);
  w.dijkstra(start, cfg_.risk_weight, false, dist_safe_, risk_safe_, parent_safe_);
  // 保守模式（自愈）：本 Tick 只用"已知空地"作为可行域。
  // 用 swap 让后面所有逻辑无差别复用（O(1)，不复制数组）。
  if (conservative) {
    dist_.swap(dist_safe_);
    parent_.swap(parent_safe_);
  }
  // 视野半径就是 cur_vision_：0 是合法值（规则 §4.1 允许 0–4），
  // 绝不能把 0 当成"缺省 4"，否则会按 9x9 估算覆盖增益，探索目标持续高估。
  const int visionR = cur_vision_;

  // ---- [FIX-F] 行程突变中止 ----
  // 墙被发现 → 目标实际距离远超出发时预估（如 2 步变 18 步）→ 立刻弃目标重规划，
  // 只损失 1~2 拍而不是走完 18 步冤枉路。预测准的图不触发，对原行为零扰动。
  if (cfg_.fix_abort && j_tgt_ >= 0 && target_ == j_tgt_ && !conservative &&
      dist_[j_tgt_] < 1e8f &&
      dist_[j_tgt_] > j_est_ * cfg_.abort_factor + 3.f) {
    target_ = -1;
    commit_until_ = -1;
    j_tgt_ = -1;
  }

  // ---- [FIX-E/G] 风险加权距离场：steps + risk/risk_weight * (lam-1) * risk_step_cost ----
  // E 用于收集规划距离，G 用于探索成本；加价系数乘 (λ-1)，预测准时趋零。
  // 保守模式 swap 过后 risk_ 就是安全场的 risk（全 0），eff 退化为纯步数，口径自洽
  if (cfg_.fix_riskdist || cfg_.fix_explore_eff) {
    const int nCells = w.cells();
    eff_.resize(nCells);
    const float lamEff = (cfg_.fix_lambda || cfg_.fix_endgame) ? lam_ : 1.f;
    const float k =
        (lamEff - 1.f) * cfg_.risk_step_cost /
        (cfg_.risk_weight > 0.1f ? cfg_.risk_weight : 1.f);
    for (int i = 0; i < nCells; ++i) eff_[i] = dist_[i] + risk_[i] * k;
  }

  // ---- [FIX-B] λ（行程可信度）更新：按"完整行程"统计 ----
  // 出发时记下预估距离，抵达目标时用 实际tick/预估 距离 修正 λ（EMA）。
  // λ=1 表示预测完全可信；λ 大表示未知区捷径严重失真（蛇形/竖柱）。
  // 加价系数用 (λ-1)：预测准的图加价自动趋零，不会伤害本来就准的巡游。
  if ((cfg_.fix_lambda || cfg_.fix_endgame) && j_tgt_ >= 0) {
    if (start == j_tgt_) {
      const int actual = tick - j_tick0_;
      const float ratio = static_cast<float>(actual) /
                          (j_est_ > 1.f ? j_est_ : 1.f);
      if (actual >= 2)
        lam_ = clampf(0.55f * lam_ + 0.45f * ratio, 1.f, 3.f);
      j_tgt_ = -1;
    } else if (target_ != j_tgt_) {
      j_tgt_ = -1;  // 中途换目标，本次行程作废
    }
  }

  // ---- [FIX-D] 围死资源剔除 ----
  // 判据：资源在"已知空地"上不可达，且它所在的已知空地连通域不与任何未知格相邻
  // （封死域不可能再长出路）。这类资源永远拿不到，剔除后不再吸引探索/巡游。
  if (cfg_.fix_enclosed && !conservative) {
    if (static_cast<int>(excl_mask_.size()) != w.cells()) {
      excl_mask_.assign(w.cells(), 0);
      excl_value_ = 0.f;
    }
    w.resourceList(res_all_);
    const int W = w.w(), H = w.h();
    const std::vector<unsigned char>& stS = w.state();
    for (std::size_t k = 0; k < res_all_.size(); ++k) {
      const int ri = res_all_[k].idx;
      if (excl_mask_[ri]) continue;
      if (dist_safe_[ri] < 1e8f) continue;  // 已知空地可达
      seal_vis_.assign(w.cells(), 0);
      seal_stack_.clear();
      seal_stack_.push_back(ri);
      seal_vis_[ri] = 1;
      bool sealed = true;
      while (!seal_stack_.empty()) {
        const int i = seal_stack_.back();
        seal_stack_.pop_back();
        const int x = i % W, y = i / W;
        int nb[4];
        int cnt = 0;
        if (x > 0) nb[cnt++] = i - 1;
        if (x + 1 < W) nb[cnt++] = i + 1;
        if (y > 0) nb[cnt++] = i - W;
        if (y + 1 < H) nb[cnt++] = i + W;
        for (int q = 0; q < cnt; ++q) {
          const int j = nb[q];
          if (stS[j] == World::UNKNOWN) { sealed = false; break; }
          if (stS[j] == World::FREE && !seal_vis_[j]) {
            seal_vis_[j] = 1;
            seal_stack_.push_back(j);
          }
        }
        if (!sealed) break;
      }
      if (sealed) {
        excl_mask_[ri] = 1;
        excl_value_ += res_all_[k].value;
      }
    }
  }

  int unk = 0, freeCells = 0, blk = 0;
  w.counts(unk, freeCells, blk);
  float hidden = cfg_.total_value - p.resource_value - w.resValueSum();
  if (cfg_.fix_enclosed) hidden += excl_value_;  // [FIX-D] 拿不到的不再计入"未发现"
  if (hidden < 0.f) hidden = 0.f;
  d.hidden = hidden;

  // ---- 资源候选 ----
  // [FIX-B] 预算口径：软距离 × λ ≤ R 才进候选（λ 随预测误差自适应，迷宫图自动放大）
  const int Rb = cfg_.fix_lambda
                     ? static_cast<int>(static_cast<float>(R) / lam_)
                     : R;
  buildResourceCandidates(w, Rb, tick);
  // 收尾阶段：剩余时间极紧时，只保留"确认走得到"的资源。
  // 用原地压缩而不是另开缓冲 + swap —— swap 会把容量换走，下一 Tick 又要扩容。
  if (cfg_.strict_finish_ticks > 0 && !conservative) {
    std::size_t wpos = 0;
    for (std::size_t i = 0; i < cands_.size(); ++i) {
      const ResCand& c = cands_[i];
      if ((static_cast<float>(R) - c.dist) <=
          static_cast<float>(cfg_.strict_finish_ticks)) {
        if (cfg_.fix_endgame) {
          // [FIX-C] 原版只认"已知空地确认可达"，迷宫里绕行距离长会误杀可达资源
          // → 38 分那种"提前停止"。放宽为：safe 可达 或 λ 放大后软可达，任一即可。
          const bool safeOk = dist_safe_[c.idx] <= static_cast<float>(R);
          const bool softOk = c.dist * lam_ <= static_cast<float>(R) + 0.5f;
          if (!safeOk && !softOk) continue;
        } else {
          if (dist_safe_[c.idx] > static_cast<float>(R)) continue;
        }
      }
      if (wpos != i) cands_[wpos] = cands_[i];
      ++wpos;
    }
    cands_.resize(wpos);
  }

  CollectPlan& cp = cp_;
  planCollect(w, start, Rb, cp);

  // ---- 动态放弃阈值（建议 5）----
  float stopR = static_cast<float>(cfg_.explore_stop_R);
  if (cfg_.dynamic_stop) {
    const float need = cp.valid ? cp.cost : 0.f;
    stopR = need + static_cast<float>(cfg_.collect_margin);
    if (stopR < static_cast<float>(cfg_.min_stop))
      stopR = static_cast<float>(cfg_.min_stop);
    if (stopR > static_cast<float>(cfg_.max_stop))
      stopR = static_cast<float>(cfg_.max_stop);
  }
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

  // ---- 探索候选 ----
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

  // ---- 目标提交：沿用上一目标 ----
  bool useCommitted = false;
  if (target_ >= 0 && tick < commit_until_) {
    if (target_kind_ == 0) {
      if (w.resValue(target_) > 0.f &&
          dist_[target_] <= static_cast<float>(R) && dist_[target_] > 0.f) {
        useCommitted = true;
      }
    } else {
      if (ep.valid && ep.target == target_) useCommitted = true;
    }
  }

  int tgt = -1;
  int kind = -1;
  float chosenRatio = 0.f;
  bool fallback = false;
  if (useCommitted) {
    tgt = target_;
    kind = target_kind_;
    chosenRatio = target_ratio_;
  } else {
    const bool haveRes = cp.valid && cp.first >= 0;
    const bool haveExp = ep.valid;
    if (!haveRes && !haveExp) {
      // ---- [FIX-A] 禁止原地等待：一阶段停下没有任何收益 ----
      // 没有预算内目标时，仍朝"综合分最高的已知资源"推进（哪怕估计赶不上，
      // 走近了每 Tick 重估，视野更新后可能变可行）；一个已知资源都没有时，
      // 朝最近的未知边界推进。每 Tick 重估，不锁死目标。
      if (cfg_.fix_never_idle) {
        int best = -1;
        float bv = -1.f;
        int bkind = 0;
        w.resourceList(res_all_);
        for (std::size_t k = 0; k < res_all_.size(); ++k) {
          const int ri = res_all_[k].idx;
          if (cfg_.fix_enclosed && ri < static_cast<int>(excl_mask_.size()) &&
              excl_mask_[ri]) continue;
          const float dd = dist_[ri];
          if (dd >= 1e8f) continue;
          float v = res_all_[k].value;
          if (v >= cfg_.high_value) v *= cfg_.high_priority;
          const float sc = v / (dd + 2.f);
          if (sc > bv) {
            bv = sc;
            best = ri;
            bkind = 0;
          }
        }
        if (best < 0 && unk > 0) {
          const int W2 = w.w(), H2 = w.h();
          const std::vector<unsigned char>& st2 = w.state();
          float bd = 1e8f;
          for (int yy = 0; yy < H2; ++yy) {
            for (int xx = 0; xx < W2; ++xx) {
              const int i = yy * W2 + xx;
              if (st2[i] != World::FREE) continue;
              bool adjU = false;
              if (xx > 0 && st2[i - 1] == World::UNKNOWN) adjU = true;
              else if (xx + 1 < W2 && st2[i + 1] == World::UNKNOWN) adjU = true;
              else if (yy > 0 && st2[i - W2] == World::UNKNOWN) adjU = true;
              else if (yy + 1 < H2 && st2[i + W2] == World::UNKNOWN) adjU = true;
              if (!adjU) continue;
              if (dist_[i] < bd) {
                bd = dist_[i];
                best = i;
                bkind = 1;
              }
            }
          }
        }
        if (best >= 0) {
          tgt = best;
          kind = bkind;
          chosenRatio = 0.f;
          target_ = tgt;
          target_kind_ = kind;
          target_ratio_ = 0.f;
          commit_until_ = tick + 1;  // 每 Tick 重估
          fallback = true;
        }
      }
      if (!fallback) {
        target_ = -1;
        commit_until_ = -1;
        d.reason = R_WAIT_NO_TARGET;
        return d;
      }
    }
    if (!fallback) {
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
      if (commitLen > cfg_.submit_max) commitLen = cfg_.submit_max;
      if (commitLen < 1) commitLen = 1;
      commit_until_ = tick + commitLen;
    }
  }
  if (tgt < 0) {
    d.reason = R_WAIT_NO_TARGET;
    return d;
  }

  // ---- [FIX-B] 记录行程起点（新选定的资源目标），供抵达时统计 实际/预估 比 ----
  if ((cfg_.fix_lambda || cfg_.fix_endgame) && !conservative &&
      tgt >= 0 && dist_[tgt] < 1e8f) {
    j_tgt_ = tgt;
    j_est_ = dist_[tgt];
    j_tick0_ = tick;
  }

  // ---- 输出第一步（含自愈阶梯）----
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
  int step = w.firstStep(*par, tgt, start);
  if (step < 0) {
    target_ = -1;
    commit_until_ = -1;
    d.reason = R_WAIT_ALREADY;
    return d;
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

  const int dir = direction_from_step(step, start, w.w());
  if (dir == 0) {
    target_ = -1;
    commit_until_ = -1;
    d.reason = R_WAIT_ALREADY;
    return d;
  }
  d.kind = MOVE_MOVE;
  d.direction = kDirs[dir];
  d.targetIdx = tgt;
  d.reason = (target_kind_ == 0)
                 ? R_MOVE_RESOURCE
                 : (stepKnownFree ? R_MOVE_EXPLORE : R_MOVE_SAFE);
  d.commitUntil = commit_until_;
  return d;
}

}  // namespace rmtc
