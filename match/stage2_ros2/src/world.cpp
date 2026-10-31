// rmtc/world.cpp —— 地图记忆、最短路与覆盖统计的实现
//
// 性能约束（第一阶段实测 0 次堆分配/Tick）：所有工作缓冲都是成员变量并复用。
// 如果想加新算法，请沿用这个做法——不要在 decide() 的热路径里声明 vector/string。
#include "rmtc/world.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <queue>
#include <utility>

namespace rmtc {

namespace {
const float kInf = 1e9f;
}  // namespace

World::World()
    : w_(0), h_(0), res_value_sum_(0.f), obstacles_version_(0),
      resources_version_(0) {
  reset(24, 24);  // 第二阶段标准地图 §4.1（实际尺寸每帧以感知 map 为准）
}

World::World(int w, int h)
    : w_(0), h_(0), res_value_sum_(0.f), obstacles_version_(0),
      resources_version_(0) {
  reset(w, h);
}

void World::reset(int w, int h) {
  if (w < 1) w = 1;
  if (h < 1) h = 1;
  w_ = w;
  h_ = h;
  const int n = w_ * h_;
  state_.assign(n, UNKNOWN);
  hard_block_.assign(n, 0);
  miss_count_.assign(n, 0);
  soft_block_.assign(n, 0);
  soft_cost_.assign(n, 0.f);
  res_val_.assign(n, 0.f);
  res_seen_.assign(n, 0);
  res_value_sum_ = 0.f;
  ++obstacles_version_;
  ++resources_version_;
  dist_buf_.assign(n, kInf);
  cost_buf_.assign(n, kInf);
  parent_buf_.assign(n, -1);
  pref_buf_.assign((w_ + 1) * (h_ + 1), 0);
  queue_buf_.assign(n, 0);
  // 拓扑缓存一并失效（换图/重置后必须重算）
  is_choke_.assign(n, 0);
  choke_value_.assign(n, 0);
  topology_valid_ = false;
  topo_free_mark_ = -1;
  topo_tick_mark_ = -1;
}

bool World::ensureSize(int w, int h) {
  if (w == w_ && h == h_) return false;
  reset(w, h);
  return true;
}

void World::updateFromPerception(const Perception& p) {
  if (p.map_w != w_ || p.map_h != h_) {
    // 换图/换场：地图尺寸变了，旧记忆全部失效，按新尺寸重建。
    reset(p.map_w, p.map_h);
  }

  // 视野是以自身为中心的方形（横纵各 ±vision_radius），不是圆形。
  const int r = p.vision_radius;
  const int x0 = std::max(0, p.x - r), x1 = std::min(w_ - 1, p.x + r);
  const int y0 = std::max(0, p.y - r), y1 = std::min(h_ - 1, p.y + r);

  // 1) 视野内：确认地形可视（UNKNOWN→FREE），并清空资源记忆。
  //    资源"看见即刷新"：这一步先全部清掉，第 2 步再把本帧真实存在的资源写回来。
  //    因此"视野内但没出现在 entities 里"等价于"被捡走了"，不需要额外的删除消息。
  for (int y = y0; y <= y1; ++y) {
    const int base = y * w_;
    for (int x = x0; x <= x1; ++x) {
      const int i = base + x;
      if (state_[i] != FREE && !hard_block_[i]) {
        // 2 帧迟滞：障碍实体偶发缺失一帧（9×9 解析失败/视野边缘）不能立刻
        // 把墙删掉；连续两帧“在视野内但未报障碍”才恢复 FREE。
        if (state_[i] == BLOCKED) {
          if (miss_count_[i] < 255) ++miss_count_[i];
          if (miss_count_[i] >= 2) state_[i] = FREE;
        } else {
          state_[i] = FREE;
        }
      }
      if (state_[i] == FREE) miss_count_[i] = 0;
      if (res_val_[i] > 0.f) {
        res_value_sum_ -= res_val_[i];
        res_val_[i] = 0.f;
        res_seen_[i] = 0;
        ++resources_version_;
      }
    }
  }

  // 2) 写入本帧实体。坐标已由 Agent 从 9×9 相对图换算成绝对坐标。
  for (std::size_t k = 0; k < p.entities.size(); ++k) {
    const Entity& e = p.entities[k];
    if (!inBounds(e.x, e.y)) continue;
    const int i = idx(e.x, e.y);
    if (e.kind == ENT_OBSTACLE) {
      if (state_[i] != BLOCKED) {
        state_[i] = BLOCKED;
        ++obstacles_version_;
      }
      miss_count_[i] = 0;
    } else if (e.kind == ENT_RESOURCE) {
      if (res_val_[i] <= 0.f) {
        res_val_[i] = e.value > 0.f ? e.value : (e.high ? 20.f : 10.f);
        res_seen_[i] = p.tick;
        res_value_sum_ += res_val_[i];
        ++resources_version_;
      }
    }
    // ENT_ROBOT 不会出现在 entities 里：敌车走 Perception::robots → Radar。
  }

  // 3) 拓扑脏检查：已探明格数变化超过 10%，或距上次计算 ≥50 Tick，重算割点。
  //    拓扑只是缓存（绝不写地形记忆），UNKNOWN 揭开后旧结果被整体覆盖。
  {
    int freeN = 0, blockedN = 0, unkN = 0;
    counts(freeN, unkN, blockedN);
    const int explored = freeN + blockedN;
    const bool dirty =
        !topology_valid_ ||
        (topo_free_mark_ >= 0 &&
         std::abs(explored - topo_free_mark_) * 10 >= w_ * h_) ||
        (topo_tick_mark_ >= 0 && p.tick - topo_tick_mark_ >= 50);
    if (dirty) analyzeTopology(p.tick);
  }
}

void World::markBlocked(int x, int y) {
  if (!inBounds(x, y)) return;
  const int i = idx(x, y);
  hard_block_[i] = 1;  // 永久：之后的 FREE 更新不会覆盖它
  miss_count_[i] = 0;
  if (state_[i] != BLOCKED) {
    state_[i] = BLOCKED;
    ++obstacles_version_;
  }
  if (res_val_[i] > 0.f) {
    res_value_sum_ -= res_val_[i];
    res_val_[i] = 0.f;
    res_seen_[i] = 0;
    ++resources_version_;
  }
}

void World::markFree(int x, int y) {
  if (!inBounds(x, y)) return;
  const int i = idx(x, y);
  if (state_[i] == UNKNOWN) state_[i] = FREE;
}

void World::clearSoft() {
  if (!soft_block_.empty()) std::memset(soft_block_.data(), 0, soft_block_.size());
  // 黄灯代价也是每 Tick 重设的临时状态，必须跟着清掉，不能污染下一 Tick。
  if (!soft_cost_.empty()) std::fill(soft_cost_.begin(), soft_cost_.end(), 0.f);
}

void World::setSoft(int idx) {
  if (idx >= 0 && idx < static_cast<int>(soft_block_.size())) soft_block_[idx] = 1;
}

void World::setSoftCost(int idx, float cost) {
  if (idx >= 0 && idx < static_cast<int>(soft_cost_.size()) && cost > 0.f)
    soft_cost_[idx] = cost;
}

float World::dropResource(int x, int y) {
  if (!inBounds(x, y)) return 0.f;
  const int i = idx(x, y);
  const float v = res_val_[i];
  if (v > 0.f) {
    res_value_sum_ -= v;
    res_val_[i] = 0.f;
    res_seen_[i] = 0;
    ++resources_version_;
  }
  return v;
}

void World::counts(int& unknown, int& freeCells, int& blocked) const {
  unknown = freeCells = blocked = 0;
  const int n = w_ * h_;
  for (int i = 0; i < n; ++i) {
    const unsigned char s = state_[i];
    if (s == UNKNOWN) ++unknown;
    else if (s == FREE) ++freeCells;
    else ++blocked;
  }
}

void World::resourceList(std::vector<ResCell>& out) const {
  out.clear();
  const int n = w_ * h_;
  for (int i = 0; i < n; ++i) {
    if (res_val_[i] > 0.f) {
      ResCell c;
      c.idx = i;
      c.value = res_val_[i];
      c.seenTick = res_seen_[i];
      out.push_back(c);
    }
  }
}

void World::dijkstra(int start, float riskWeight, bool allowUnknown,
                     std::vector<float>& steps, std::vector<float>& risk,
                     std::vector<int>& parent, float chokeRisk) const {
  const int n = w_ * h_;
  steps.assign(n, kInf);
  risk.assign(n, kInf);
  parent.assign(n, -1);
  cost_buf_.assign(n, kInf);
  if (start < 0 || start >= n) return;

  // 手写最小堆（成员复用，避免每 Tick 分配）。
  // 排序键：(步数+黄灯代价, 风险, 下标)。黄灯代价全 0 时退化为原“先比步数”，
  // 路径与历史完全一致；ALERT 时允许 1–2 格绕行，而不是把格子封死。
  heap_.clear();
  steps[start] = 0.f;
  risk[start] = 0.f;
  cost_buf_[start] = 0.f;
  HeapNode s0;
  s0.steps = 0;
  s0.cost = 0.f;
  s0.risk = 0.f;
  s0.idx = start;
  heap_.push_back(s0);
  std::push_heap(heap_.begin(), heap_.end(), HeapGreater());

  const int w = w_;
  while (!heap_.empty()) {
    std::pop_heap(heap_.begin(), heap_.end(), HeapGreater());
    const HeapNode top = heap_.back();
    heap_.pop_back();
    const int i = top.idx;
    // 过期堆项：代价更差，或代价相同但风险更差，直接跳过。
    if (top.cost > cost_buf_[i] + 1e-6f ||
        (std::abs(top.cost - cost_buf_[i]) <= 1e-6f &&
         top.risk > risk[i] + 1e-6f))
      continue;
    const int x = i % w;
    const int y = i / w;
    int nb[4];
    int cnt = 0;
    if (x > 0) nb[cnt++] = i - 1;
    if (x + 1 < w) nb[cnt++] = i + 1;
    if (y > 0) nb[cnt++] = i - w;
    if (y + 1 < h_) nb[cnt++] = i + w;
    for (int k = 0; k < cnt; ++k) {
      const int j = nb[k];
      const unsigned char st = state_[j];
      if (st == BLOCKED) continue;
      if (soft_block_[j]) continue;  // 敌车/占位：寻路绕行，但不进地形记忆
      if (st == UNKNOWN && !allowUnknown) continue;
      const int ns = top.steps + 1;                       // 真实步数永远 +1
      // 咽喉风险：只加在次级键（risk）上，不加进步数。
      // 步数保持真实值，"剩余时间 R 内可达"的可行性判断不会被扭曲；
      // 等长路径之间则优先绕开咽喉（健康时轻罚、残血时重罚）。
      float nr = top.risk + (st == UNKNOWN ? riskWeight : 0.f);
      if (chokeRisk > 0.f && isChoke(j)) nr += chokeRisk;
      // 黄灯软代价只加进“主代价”，不改 steps：允许绕 1–2 格，
      // 但绝不把格子变成不可通行（防止被敌人卡死在地图角落）。
      const float sc = (j < static_cast<int>(soft_cost_.size())) ? soft_cost_[j] : 0.f;
      const float nc = top.cost + 1.f + sc;
      if (nc < cost_buf_[j] - 1e-6f ||
          (std::abs(nc - cost_buf_[j]) <= 1e-6f && nr < risk[j] - 1e-6f)) {
        steps[j] = static_cast<float>(ns);
        risk[j] = nr;
        cost_buf_[j] = nc;
        parent[j] = i;
        HeapNode nd;
        nd.steps = ns;
        nd.cost = nc;
        nd.risk = nr;
        nd.idx = j;
        heap_.push_back(nd);
        std::push_heap(heap_.begin(), heap_.end(), HeapGreater());
      }
    }
  }
}

void World::dijkstraSteps(int start, bool allowUnknown,
                          std::vector<float>& steps,
                          std::vector<int>& parent, float chokeRisk) const {
  std::vector<float> dummy;
  dijkstra(start, 0.f, allowUnknown, steps, dummy, parent, chokeRisk);
}

int World::firstStep(const std::vector<int>& parent, int target, int start) const {
  if (target < 0 || target >= w_ * h_) return -1;
  if (target == start) return -1;
  int cur = target;
  int guard = 0;
  const int limit = w_ * h_ + 4;  // 防止 parent 成环时死循环
  while (guard++ < limit) {
    const int p = parent[cur];
    if (p < 0) return -1;        // 不可达
    if (p == start) return cur;  // cur 就是从起点迈出的第一步
    cur = p;
  }
  return -1;
}

void World::coverageGain(int r, std::vector<int>& gain) const {
  const int w = w_, h = h_;
  const int pw = w + 1;
  // 二维前缀和：pref[y+1][x+1] = 左上角 (0,0)-(x,y) 区域内的未知格数。
  pref_buf_.assign((w + 1) * (h + 1), 0);
  for (int y = 0; y < h; ++y) {
    int row = 0;
    const int base = y * w;
    const int pbase = (y + 1) * pw;
    const int pprev = y * pw;
    for (int x = 0; x < w; ++x) {
      if (state_[base + x] == UNKNOWN) ++row;
      pref_buf_[pbase + x + 1] = pref_buf_[pprev + x + 1] + row;
    }
  }
  gain.assign(w * h, 0);
  for (int y = 0; y < h; ++y) {
    const int y0 = std::max(0, y - r);
    const int y1 = std::min(h - 1, y + r);
    const int ptop = y0 * pw;
    const int pbot = (y1 + 1) * pw;
    for (int x = 0; x < w; ++x) {
      const int x0 = std::max(0, x - r);
      const int x1 = std::min(w - 1, x + r);
      // 容斥求方形窗口内未知格数量
      gain[y * w + x] = pref_buf_[pbot + x1 + 1] - pref_buf_[pbot + x0] -
                        pref_buf_[ptop + x1 + 1] + pref_buf_[ptop + x0];
    }
  }
}

void World::coverageGainWeighted(int r, const float* weights,
                                 std::vector<float>& gain) const {
  const int w = w_, h = h_;
  const int pw = w + 1;
  // 与 coverageGain 相同的窗口和，只是未知格按下标取权重（float 前缀和）。
  pref_f_.assign(static_cast<std::size_t>(w + 1) * (h + 1), 0.f);
  for (int y = 0; y < h; ++y) {
    float row = 0.f;
    const int base = y * w;
    const int pbase = (y + 1) * pw;
    const int pprev = y * pw;
    for (int x = 0; x < w; ++x) {
      const int i = base + x;
      if (state_[i] == UNKNOWN) row += (weights != NULL ? weights[i] : 1.f);
      pref_f_[pbase + x + 1] = pref_f_[pprev + x + 1] + row;
    }
  }
  gain.assign(w * h, 0.f);
  for (int y = 0; y < h; ++y) {
    const int y0 = std::max(0, y - r);
    const int y1 = std::min(h - 1, y + r);
    const int ptop = y0 * pw;
    const int pbot = (y1 + 1) * pw;
    for (int x = 0; x < w; ++x) {
      const int x0 = std::max(0, x - r);
      const int x1 = std::min(w - 1, x + r);
      gain[y * w + x] = pref_f_[pbot + x1 + 1] - pref_f_[pbot + x0] -
                        pref_f_[ptop + x1 + 1] + pref_f_[ptop + x0];
    }
  }
}

int World::labelRegions(std::vector<int>& label, std::vector<int>& size,
                        int minSize) const {
  const int n = w_ * h_;
  label.assign(n, -1);
  size.clear();
  int next = 0;
  // 迭代式 DFS（显式栈）：地图最大也就几百格，但这样不会因递归深度爆栈，
  // 栈/成员缓冲都是成员变量，可复用。
  std::vector<int>& stack = region_stack_;
  std::vector<int>& members = region_members_;
  for (int s = 0; s < n; ++s) {
    if (state_[s] != UNKNOWN || label[s] >= 0) continue;
    stack.clear();
    members.clear();
    stack.push_back(s);
    label[s] = next;
    while (!stack.empty()) {
      const int i = stack.back();
      stack.pop_back();
      members.push_back(i);
      const int x = i % w_;
      const int y = i / w_;
      int nb[4];
      int cnt = 0;
      if (x > 0) nb[cnt++] = i - 1;
      if (x + 1 < w_) nb[cnt++] = i + 1;
      if (y > 0) nb[cnt++] = i - w_;
      if (y + 1 < h_) nb[cnt++] = i + w_;
      for (int k = 0; k < cnt; ++k) {
        const int j = nb[k];
        if (state_[j] == UNKNOWN && label[j] < 0) {
          label[j] = next;
          stack.push_back(j);
        }
      }
    }
    const int count = static_cast<int>(members.size());
    if (count < minSize) {
      // 太小的块：只在成员上回改标记，避免 O(n) 重扫。
      for (std::size_t k = 0; k < members.size(); ++k) label[members[k]] = -2;
      size.push_back(0);
    } else {
      size.push_back(count);
    }
    ++next;
  }
  return next;
}

// ---------------------------------------------------------------------------
// 地形拓扑：迭代式 Tarjan 割点 + 咽喉价值（建议书《地形问题改进》§3）
//
// 图：state != BLOCKED 的格子（FREE+UNKNOWN）4 邻接。
// 咽喉价值 = 删除该割点后，其所在连通块分裂出的各侧规模的"最小一侧"
//   choke_value(v) = min over sides s of min(s, total-1-s)
// ≥ choke_min_value 的割点才标记为咽喉（两侧都够大才叫"战略要道"；
// 小口袋的口子没有封锁/躲避价值）。
//
// 全程显式栈（建议书警告 #3：防深递归爆栈），工作缓冲全部是成员变量。
// 复杂度：Tarjan O(V+E)；每个割点一次 BFS 分侧，≤ 割点数 × O(V)。
// 576 格实测 <0.1 ms，触发频率 ≤ 每 50 Tick 或探索 +10%。
int World::chokeCount() const {
  int c = 0;
  if (!topology_valid_) return 0;
  for (std::size_t i = 0; i < is_choke_.size(); ++i)
    if (is_choke_[i]) ++c;
  return c;
}

void World::analyzeTopology(int tick) {
  const int n = w_ * h_;
  // 缓冲按需初始化（reset 会同步 is_choke_，但 Tarjan 缓冲在这里兜底）
  if (static_cast<int>(is_choke_.size()) != n) {
    is_choke_.assign(n, 0);
    choke_value_.assign(n, 0);
  }
  if (static_cast<int>(t_order_.size()) != n) {
    t_dfn_.assign(n, 0);
    t_low_.assign(n, 0);
    t_parent_.assign(n, -1);
    t_order_.assign(n, -1);
    t_comp_.assign(n, 0);
    t_stack_.reserve(n);
    t_ap_.reserve(64);
  }
  std::fill(is_choke_.begin(), is_choke_.end(), 0);
  std::fill(choke_value_.begin(), choke_value_.end(), 0);
  t_ap_.clear();

  const int w = w_;
  // 邻居收集（割点检测需要按固定顺序访问子节点，保证确定性）
  // ---- 迭代式 Tarjan（根节点用"子树数 ≥2"判割点，其余用 low[child] >= dfn[v]）----
  std::fill(t_order_.begin(), t_order_.end(), -1);
  std::fill(t_parent_.begin(), t_parent_.end(), -1);
  int timer = 0;
  // 显式栈帧：交替压入"节点"与"下一个待访问邻居序号"两个 int
  std::vector<int>& stk = region_stack_;  // 复用成员缓冲（≤2n 个 int）
  stk.clear();

  for (int root = 0; root < n; ++root) {
    if (state_[root] == BLOCKED || t_order_[root] >= 0) continue;
    t_order_[root] = timer;
    t_dfn_[root] = t_low_[root] = timer++;
    t_parent_[root] = -1;
    int rootChildren = 0;
    stk.push_back(root);
    stk.push_back(0);  // nextK
    while (!stk.empty()) {
      const int nextK = stk.back();
      stk.pop_back();
      const int v = stk.back();
      const int x = v % w, y = v / w;
      int nb[4];
      int cnt = 0;
      if (x > 0 && state_[v - 1] != BLOCKED) nb[cnt++] = v - 1;
      if (x + 1 < w && state_[v + 1] != BLOCKED) nb[cnt++] = v + 1;
      if (y > 0 && state_[v - w] != BLOCKED) nb[cnt++] = v - w;
      if (y + 1 < h_ && state_[v + w] != BLOCKED) nb[cnt++] = v + w;
      if (nextK < cnt) {
        // 恢复帧并把推进后的 nextK 压回
        stk.push_back(nextK + 1);
        const int u = nb[nextK];
        if (t_order_[u] < 0) {
          t_parent_[u] = v;
          t_order_[u] = timer;
          t_dfn_[u] = t_low_[u] = timer++;
          if (v == root) ++rootChildren;
          stk.push_back(u);
          stk.push_back(0);
        } else if (u != t_parent_[v]) {
          if (t_dfn_[u] < t_low_[v]) t_low_[v] = t_dfn_[u];
        }
      } else {
        // v 的孩子都处理完：回溯并更新父节点 low
        stk.pop_back();  // 弹出 v
        const int par = t_parent_[v];
        if (par >= 0) {
          if (t_low_[v] < t_low_[par]) t_low_[par] = t_low_[v];
          if (par != root && t_low_[v] >= t_dfn_[par]) {
            // par 是割点（去重：同一割点可能被多个孩子触发）
            bool seen = false;
            for (std::size_t q = 0; q < t_ap_.size(); ++q)
              if (t_ap_[q] == par) seen = true;
            if (!seen) t_ap_.push_back(par);
          }
        }
      }
    }
    if (rootChildren >= 2) {
      bool seen = false;
      for (std::size_t q = 0; q < t_ap_.size(); ++q)
        if (t_ap_[q] == root) seen = true;
      if (!seen) t_ap_.push_back(root);
    }
  }

  // ---- 对每个割点算"删掉后的最小一侧规模"（BFS 分侧）----
  for (std::size_t ai = 0; ai < t_ap_.size(); ++ai) {
    const int v = t_ap_[ai];
    // 收集 v 的邻居作为各侧种子（同侧的邻居会在 BFS 中合并）
    int seeds[4];
    int seedN = 0;
    const int x = v % w, y = v / w;
    if (x > 0 && state_[v - 1] != BLOCKED) seeds[seedN++] = v - 1;
    if (x + 1 < w && state_[v + 1] != BLOCKED) seeds[seedN++] = v + 1;
    if (y > 0 && state_[v - w] != BLOCKED) seeds[seedN++] = v - w;
    if (y + 1 < h_ && state_[v + w] != BLOCKED) seeds[seedN++] = v + w;

    std::fill(t_comp_.begin(), t_comp_.end(), 0);
    t_comp_[v] = -1;  // v 本身已删除
    int totalReachable = 0;  // v 所在连通块的大小（含 v）
    int bestMin = -1;
    for (int s = 0; s < seedN; ++s) {
      const int seed = seeds[s];
      if (t_comp_[seed] != 0) continue;  // 已归入某一侧
      // BFS 这一侧
      t_stack_.clear();
      t_stack_.push_back(seed);
      t_comp_[seed] = 1;
      int sideSize = 0;
      for (std::size_t qi = 0; qi < t_stack_.size(); ++qi) {
        const int c = t_stack_[qi];
        ++sideSize;
        ++totalReachable;
        const int cx = c % w, cy = c / w;
        int nb2[4];
        int cnt2 = 0;
        if (cx > 0 && state_[c - 1] != BLOCKED) nb2[cnt2++] = c - 1;
        if (cx + 1 < w && state_[c + 1] != BLOCKED) nb2[cnt2++] = c + 1;
        if (cy > 0 && state_[c - w] != BLOCKED) nb2[cnt2++] = c - w;
        if (cy + 1 < h_ && state_[c + w] != BLOCKED) nb2[cnt2++] = c + w;
        for (int k = 0; k < cnt2; ++k) {
          const int u = nb2[k];
          if (t_comp_[u] != 0) continue;
          t_comp_[u] = 1;
          t_stack_.push_back(u);
        }
      }
      const int rest = totalReachable - sideSize;  // 其余各侧之和（不含 v）
      const int m = sideSize < rest ? sideSize : rest;
      if (bestMin < 0 || m > bestMin) bestMin = m;
      // 说明：咽喉价值取"最大一侧对"里的较小者——删除 v 后剩下 k 侧时，
      // 最坏分离规模由最大的 min(s_i, rest) 决定（k>2 时取该值更合理）。
    }
    if (bestMin >= 0 && bestMin > choke_value_[v]) choke_value_[v] = bestMin;
    // 是否标记为咽喉由调用方的 K 阈值决定（isChoke 由 Agent 侧过滤），
    // 这里先全部标记非零价值，K 过滤放查询端更灵活——但为了 O(1) 查询，
    // 直接按默认 K=8 标记（与 Config::choke_min_value 默认一致；若调参改 K，
    // 重新调用 analyzeTopology 前先改 World::choke_min_value_）。
    if (choke_value_[v] >= choke_min_value_) is_choke_[v] = 1;
    (void)totalReachable;
  }

  if (std::getenv("TOPO_DEBUG")) {
    std::printf("[topo] articulation points: %d\n", static_cast<int>(t_ap_.size()));
    int visitedN = 0;
    for (int q = 0; q < n; ++q) if (t_order_[q] >= 0) ++visitedN;
    std::printf("[topo] visited=%d / %d\n", visitedN, n);
    for (std::size_t q = 0; q < t_ap_.size() && q < 30; ++q)
      std::printf("  ap(%d,%d)\n", t_ap_[q] % w_, t_ap_[q] / w_);
  }
  topology_valid_ = true;
  int freeN = 0, blockedN = 0, unkN = 0;
  counts(freeN, unkN, blockedN);
  topo_free_mark_ = freeN + blockedN;
  topo_tick_mark_ = tick;
}

}  // namespace rmtc
