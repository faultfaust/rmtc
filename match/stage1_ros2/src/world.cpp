// rmtc/world.cpp ���� ��ͼ���䡢���·����������ʵ��
#include "rmtc/world.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <queue>
#include <utility>

namespace rmtc {

namespace {
const float kInf = 1e9f;

struct NodeLess {
  bool operator()(const std::pair<float, int>& a,
                  const std::pair<float, int>& b) const {
    return a.first > b.first;
  }
};
}  // namespace

World::World()
    : w_(0), h_(0), res_value_sum_(0.f), obstacles_version_(0),
      resources_version_(0) {
  reset(20, 20);
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
  res_val_.assign(n, 0.f);
  res_seen_.assign(n, 0);
  res_value_sum_ = 0.f;
  ++obstacles_version_;
  ++resources_version_;
  dist_buf_.assign(n, kInf);
  parent_buf_.assign(n, -1);
  pref_buf_.assign((w_ + 1) * (h_ + 1), 0);
  queue_buf_.assign(n, 0);
}

bool World::ensureSize(int w, int h) {
  if (w == w_ && h == h_) return false;
  reset(w, h);
  return true;
}

void World::updateFromPerception(const Perception& p) {
  if (p.map_w != w_ || p.map_h != h_) {
    // ���� ��8�������������֪��ĵ�ͼ�ߴ磬����������һ��
    reset(p.map_w, p.map_h);
  }
  const int r = p.vision_radius;
  const int x0 = std::max(0, p.x - r), x1 = std::min(w_ - 1, p.x + r);
  const int y0 = std::max(0, p.y - r), y1 = std::min(h_ - 1, p.y + r);

  // 1) �ɼ���ȷ֪���������� FREE������մ����ڵ���Դ���䣨�ɼ�����ֵ��
  for (int y = y0; y <= y1; ++y) {
    const int base = y * w_;
    for (int x = x0; x <= x1; ++x) {
      const int i = base + x;
      if (state_[i] != FREE && !hard_block_[i]) {
        state_[i] = FREE;
      }
      if (res_val_[i] > 0.f) {
        res_value_sum_ -= res_val_[i];
        res_val_[i] = 0.f;
        res_seen_[i] = 0;
        ++resources_version_;
      }
    }
  }

  // 2) ����ʵ��
  for (std::size_t k = 0; k < p.entities.size(); ++k) {
    const Entity& e = p.entities[k];
    if (!inBounds(e.x, e.y)) continue;
    const int i = idx(e.x, e.y);
    if (e.kind == ENT_OBSTACLE) {
      if (state_[i] != BLOCKED) {
        state_[i] = BLOCKED;
        ++obstacles_version_;
      }
    } else if (e.kind == ENT_RESOURCE) {
      if (res_val_[i] <= 0.f) {
        res_val_[i] = e.value > 0.f ? e.value : (e.high ? 20.f : 10.f);
        res_seen_[i] = p.tick;
        res_value_sum_ += res_val_[i];
        ++resources_version_;
      }
    }
    // ENT_ROBOT����һ�׶��޵��ˣ�����
  }
}

void World::markBlocked(int x, int y) {
  if (!inBounds(x, y)) return;
  const int i = idx(x, y);
  hard_block_[i] = 1;
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

namespace {
// ���ȶ���Ԫ�أ��ȱ� steps���ٱ� risk ���� �ֵ��������ھ�������Ⱦ
struct QNode {
  int steps;
  float risk;
  int idx;
};

struct QGreater {
  bool operator()(const QNode& a, const QNode& b) const {
    if (a.steps != b.steps) return a.steps > b.steps;
    if (a.risk != b.risk) return a.risk > b.risk;
    return a.idx > b.idx;
  }
};
}  // namespace

void World::dijkstra(int start, float riskWeight, bool allowUnknown,
                     std::vector<float>& steps, std::vector<float>& risk,
                     std::vector<int>& parent) const {
  const int n = w_ * h_;
  steps.assign(n, kInf);
  risk.assign(n, kInf);
  parent.assign(n, -1);
  if (start < 0 || start >= n) return;

  // �ֹ�ά������ѣ������ǳ�Ա����������ֻ����һ�Σ�֮��ÿ Tick ����䡣
  // �Ƚ����� std::priority_queue ��ȫһ�£�����˳�򣨺����У����ʵ����ͬ��
  heap_.clear();
  steps[start] = 0.f;
  risk[start] = 0.f;
  HeapNode s0;
  s0.steps = 0;
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
    if (top.steps > steps[i] || (top.steps == steps[i] && top.risk > risk[i] + 1e-6f))
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
      if (st == UNKNOWN && !allowUnknown) continue;
      const int ns = top.steps + 1;                       // ��ʵ��������Զ +1
      const float nr = top.risk + (st == UNKNOWN ? riskWeight : 0.f);
      if (ns < steps[j] ||
          (ns == steps[j] && nr < risk[j] - 1e-6f)) {
        steps[j] = static_cast<float>(ns);
        risk[j] = nr;
        parent[j] = i;
        HeapNode nd;
        nd.steps = ns;
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
                          std::vector<int>& parent) const {
  static const float kNoRisk[1] = {0.f};
  std::vector<float> dummy;
  dijkstra(start, 0.f, allowUnknown, steps, dummy, parent);
  (void)kNoRisk;
}

int World::firstStep(const std::vector<int>& parent, int target, int start) const {
  if (target < 0 || target >= w_ * h_) return -1;
  if (target == start) return -1;
  int cur = target;
  int guard = 0;
  const int limit = w_ * h_ + 4;
  while (guard++ < limit) {
    const int p = parent[cur];
    if (p < 0) return -1;
    if (p == start) return cur;
    cur = p;
  }
  return -1;
}

void World::coverageGain(int r, std::vector<int>& gain) const {
  const int w = w_, h = h_;
  const int pw = w + 1;
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
      gain[y * w + x] = pref_buf_[pbot + x1 + 1] - pref_buf_[pbot + x0] -
                        pref_buf_[ptop + x1 + 1] + pref_buf_[ptop + x0];
    }
  }
}

void World::coverageGainWeighted(int r, const float* weights,
                                 std::vector<float>& gain) const {
  const int w = w_, h = h_;
  const int pw = w + 1;
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
  // ����ʽջ�� 4 �ڽ� BFS����ͼҲ���ᱬջ����ջ���Ա���ó�Ա���壬�����
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
      // ̫С�Ŀ飺ֻ�ع��Լ��ǲ��֣�O(���С)���������� O(n^2)��
      for (std::size_t k = 0; k < members.size(); ++k) label[members[k]] = -2;
      size.push_back(0);
    } else {
      size.push_back(count);
    }
    ++next;
  }
  return next;
}

}  // namespace rmtc

