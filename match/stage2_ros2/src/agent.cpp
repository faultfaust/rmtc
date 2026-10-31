// rmtc/agent.cpp —— 感知 → 世界/敌情 → 决策 → 动作 的完整流水线实现
//
// 一个 Tick 的处理顺序（与 onPerception 里的注释一一对应）：
//   parse → session/tick 对齐 → 回执归因 → 更新世界 → 更新敌情
//   → 设置软阻挡 → 决策（combat → planner）→ 编码动作 → 登记本 Tick 尝试
//
// 这里不做任何 ROS 相关操作，所有输入输出都是 std::string，便于离线测试。
#include "rmtc/agent.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstring>

namespace rmtc {

// 感知坐标是否落在地图内（供测试与越界保护使用）。
bool insideMap(const Perception& p) {
    return p.map_w > 0 && p.map_h > 0 && p.x >= 0 && p.y >= 0 && p.x < p.map_w &&
           p.y < p.map_h;
}

namespace {

// ---- 容错取值：类型不符就用默认值 ----
// 这些 helper 是"感知解析不崩"的第一道防线：调用方不需要先判空判类型。
double numOr(const JsonValue* v, double def) {
    if (!v || !v->isNum()) return def;
    return std::isfinite(v->num) ? v->num : def;
}
bool boolOr(const JsonValue* v, bool def) {
    return (v && v->isBool()) ? v->b : def;
}
std::string strOr(const JsonValue* v, const std::string& def) {
    return (v && v->isStr()) ? v->str : def;
}
int intOr(const JsonValue* v, int def) {
    const double n = numOr(v, static_cast<double>(def));
    if (n < -1e9 || n > 1e9) return def;  // 离谱数值（如 1e300）直接退回默认
    return static_cast<int>(n + (n >= 0 ? 0.5 : -0.5));
}

// 方向字符串 → 位移。未识别的方向保持 (0,0)。
void dirDelta(const char* dir, int& dx, int& dy) {
    dx = 0;
    dy = 0;
    if (!dir) return;
    if (!std::strcmp(dir, "up")) dy = -1;
    else if (!std::strcmp(dir, "down")) dy = 1;
    else if (!std::strcmp(dir, "left")) dx = -1;
    else if (!std::strcmp(dir, "right")) dx = 1;
}

// 方向编号 → 位移（与 planner 一致：1=right 2=left 3=down 4=up）
const int kDirDx[5] = {0, 1, -1, 0, 0};
const int kDirDy[5] = {0, 0, 0, 1, -1};

// 方向取反（幻术用）
const char* flipName(const char* d) {
    if (!d) return NULL;
    if (!std::strcmp(d, "up")) return "down";
    if (!std::strcmp(d, "down")) return "up";
    if (!std::strcmp(d, "left")) return "right";
    if (!std::strcmp(d, "right")) return "left";
    return d;
}

// reach 任务类型判定。协议写的是 "reach"；练习裁判不同版本可能给
// "reach_target"/"reach_mission"，统一按“包含 reach”识别，避免任务被静默跳过。
bool isReachType(const std::string& type) {
    std::string k;
    k.reserve(type.size());
    for (std::size_t i = 0; i < type.size(); ++i)
        k.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(type[i]))));
    return k.find("reach") != std::string::npos;
}

// ---- 任务公告坐标抽取（只服务 reach，不解析随机事件）----
// 规则 §8.1 禁止用正则/关键词模板替代 LLM 理解“随机事件”；任务公告属于
// 另一类信息（协议 §3：mission_text 与随机事件文本分别处理），且这里的
// 结果只用于在结构化 missions 缺坐标时兜底，不参与任何事件判断。
bool readCoordInt(const std::string& s, std::size_t& i, int& v) {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    bool neg = false;
    if (i < s.size() && (s[i] == '-' || s[i] == '+')) {
        neg = (s[i] == '-');
        ++i;
    }
    if (i >= s.size() || s[i] < '0' || s[i] > '9') return false;
    long long n = 0;
    int digits = 0;
    while (i < s.size() && s[i] >= '0' && s[i] <= '9') {
        n = n * 10 + (s[i] - '0');
        ++i;
        if (++digits > 9) return false;
    }
    v = static_cast<int>(neg ? -n : n);
    return true;
}

// 在 [from,to) 里找 "a,b" / "[a, b]" / "(a,b)" / "（a，b）"。
bool findCoordPair(const std::string& s, std::size_t from, std::size_t to, int& x, int& y) {
    if (to > s.size()) to = s.size();
    for (std::size_t i = from; i < to; ++i) {
        std::size_t j = i;
        if (s[j] == '[' || s[j] == '(') ++j;
        int a = 0, b = 0;
        if (!readCoordInt(s, j, a)) continue;
        while (j < to && (s[j] == ' ' || s[j] == '\t')) ++j;
        if (j < to && s[j] == ',') {
            ++j;
        } else if (j + 3 <= to && s.compare(j, 3, "，") == 0) {
            j += 3;
        } else {
            continue;
        }
        if (!readCoordInt(s, j, b)) continue;
        x = a;
        y = b;
        return true;
    }
    return false;
}

bool extractReachCoord(const std::string& text, int map_w, int map_h, int& x, int& y) {
    if (text.empty()) return false;
    // 关键词：只有和“到达/进入/任务点”同句出现的数字才当作任务点。
    // 故意不把裸“坐标”当关键词——资源任务公告也可能写“资源坐标 [x,y]”，
    // 误把资源点当任务点会让整局走错方向。
    static const char* kKeywords[] = {"到达", "抵达", "进入", "前往", "任务点",
                                      "目标坐标", "reach"};
    std::size_t begin = std::string::npos;
    for (std::size_t k = 0; k < sizeof(kKeywords) / sizeof(kKeywords[0]); ++k) {
        const std::size_t p = text.find(kKeywords[k]);
        if (p != std::string::npos && (begin == std::string::npos || p < begin))
            begin = p;
    }
    if (begin == std::string::npos) return false;
    // 关键词后 160 字节内找第一组坐标（同一条任务公告通常很近）。
    const std::size_t end = begin + 160;
    if (!findCoordPair(text, begin, end, x, y)) return false;
    return x >= 0 && y >= 0 && x < map_w && y < map_h;
}

std::string trimLine(const std::string& in) {
    std::size_t b = 0, e = in.size();
    while (b < e && (in[b] == ' ' || in[b] == '\t' || in[b] == '\r' || in[b] == '\n')) ++b;
    while (e > b && (in[e - 1] == ' ' || in[e - 1] == '\t' || in[e - 1] == '\r' ||
                     in[e - 1] == '\n'))
        --e;
    return in.substr(b, e - b);
}

bool eventLineIn(const std::vector<std::string>& v, const std::string& key) {
    return std::find(v.begin(), v.end(), key) != v.end();
}

void rememberEventLine(std::vector<std::string>& v, const std::string& key) {
    if (!eventLineIn(v, key)) v.push_back(key);
}

// 事件失败退避上限（Tick）：旧值 30 Tick（15 s）对 20–50 Tick 的短事件来说
// 等于直接放弃。3 s 超时下次失败固定几 Tick 后重试更有意义。
const int kEventRetryMax = 10;
// 在途公告超过该 Tick 数仍没有结果：LlmClient::poll 可能因 max_age_ticks
// 丢弃了结果，或 seq/会话对不上。必须主动解卡，否则 event_pending_seq_ 永远
// 为正，后续公告全部提交不出去。
const int kEventPendingTimeout = 12;
// “2xx 但模型输出不合法”时同一公告行的重试次数（不含首次请求）。
const int kEventContentRetries = 2;

}  // namespace

Agent::Agent(const Config& cfg)
    : cfg_(cfg), world_(24, 24), planner_(cfg), last_tick_(-1) {
    // 战斗层深度来自配置（combat_depth=0 时退回纯启发式）
    Combat::Params cp = combat_.params();
    cp.depth = cfg.combat_depth;
    cp.opp_cost = cfg.combat_opp_cost;
    cp.ray_block = cfg.combat_ray_block;
    cp.opp_cost_cap = cfg.opp_cost_cap;
    cp.rank_value = cfg.combat_rank_value;
    cp.d_cap_gate = cfg.d_cap_gate;
    combat_.configure(cp);
    // 地形拓扑阈值同步给世界模型
    world_.setChokeMinValue(cfg.choke_min_value);
}

// ---------------------------------------------------------------------------
// 感知解析
//
// 三层处理：
//   1. 致命字段：tick / session_id / self.position，缺一个就无法对齐动作窗口，
//      返回 false → 本轮不提交（绝不猜 tick 或场次）。
//   2. 普通字段：缺失或类型不符取默认值，不影响本帧使用。
//   3. 实体：位置是 9×9 相对图，解析失败只丢该实体并记日志，不丢整帧。
bool Agent::parsePerception(const std::string& data, Perception& p) {
    JsonValue root;
    std::string err;
    if (!jsonParse(data, root, err) || !root.isObj()) {
        logf(1, "perception JSON invalid: " + err);
        return false;
    }
    const JsonValue* jt = root.get("tick");
    if (!jt || !jt->isNum()) {
        logf(1, "perception missing tick");
        return false;
    }
    p.tick = intOr(jt, 0);
    const JsonValue* js = root.get("session_id");
    if (!js || !js->isStr()) {
        logf(1, "perception missing session_id");
        return false;
    }
    p.session_id = js->str;
    if (p.tick < 0 || p.session_id.empty()) {
        logf(1, "perception tick/session_id invalid");
        return false;
    }

    const JsonValue* self = root.get("self");
    if (!self || !self->isObj()) {
        logf(1, "perception missing self");
        return false;
    }
    const JsonValue* pos = self->get("position");
    if (!pos || !pos->isArr() || pos->arr.size() < 2 ||
        !pos->arr[0].isNum() || !pos->arr[1].isNum()) {
        logf(1, "perception missing self.position");
        return false;
    }
    p.x = intOr(&pos->arr[0], 0);
    p.y = intOr(&pos->arr[1], 0);

    // ---- self 的其余字段 ----
    p.alive = boolOr(self->get("alive"), true);
    p.hp = intOr(self->get("hp"), 100);
    p.defending = boolOr(self->get("defending"), false);
    p.protected_ = boolOr(self->get("protected"), false);
    p.respawn_in = intOr(self->get("respawn_in"), 0);
    p.robot_id = strOr(self->get("robot_id"), "");
    const JsonValue* pr = self->get("position_reason");
    if (pr && pr->isStr()) {
        p.has_position_reason = true;
        p.position_reason = pr->str;
    }

    // ---- 上一 Tick 回执 ----
    // 只用于解释结果（是否移动成功/错误码），不能用来累加定位（规则 §4.1）。
    const JsonValue* la = self->get("last_action");
    if (la && la->isObj()) {
        p.last_action.present = true;
        p.last_action.tick = intOr(la->get("tick"), -1);
        p.last_action.action = strOr(la->get("action"), "");
        p.last_action.moved = boolOr(la->get("moved"), false);
        p.last_action.error = strOr(la->get("error"), "");
        p.last_action.effective_direction = strOr(la->get("effective_direction"), "");
    }

    // ---- 视野与地图 ----
    p.vision_radius = intOr(root.get("vision_radius"), 4);
    // 协议 §3：vision_radius 只允许 0–4；异常值夹紧，避免 World 把整图当可见。
    if (p.vision_radius < 0) p.vision_radius = 0;
    if (p.vision_radius > 4) p.vision_radius = 4;
    // map 缺失时**保持当前尺寸**，绝不能用 20 当默认值：
    // 那会让 updateFromPerception 把整张 24×24 记忆重置成 20×20，
    // 之后 self.position 可能越界（World::idx 不做边界检查）→ 堆越界。
    const JsonValue* map = root.get("map");
    p.map_w = intOr(map ? map->get("width") : NULL, world_.w());
    p.map_h = intOr(map ? map->get("height") : NULL, world_.h());
    if (p.map_w < 1 || p.map_w > 512) p.map_w = world_.w();
    if (p.map_h < 1 || p.map_h > 512) p.map_h = world_.h();

    // ---- 计分 ----
    const JsonValue* score = root.get("score");
    p.resource_value = static_cast<float>(numOr(score ? score->get("resource_value") : NULL, 0.0));
    p.effective_damage = static_cast<float>(numOr(score ? score->get("effective_damage") : NULL, 0.0));
    p.mission_completed = intOr(score ? score->get("mission_completed") : NULL, 0);
    p.invalid_actions = intOr(score ? score->get("invalid_actions") : NULL, 0);
    const JsonValue* mids = score ? score->get("mission_completed_ids") : NULL;
    if (mids && mids->isArr()) {
        for (std::size_t i = 0; i < mids->arr.size(); ++i) {
            if (mids->arr[i].isStr()) p.mission_completed_ids.push_back(mids->arr[i].str);
        }
    }

    // ---- 可见实体 ----
    // 每类实体的 position 都是独立的 9×9 相对图（不是合并视野图）。
    const JsonValue* ents = root.get("visible_entities");
    if (ents && ents->isArr()) {
        for (std::size_t i = 0; i < ents->arr.size(); ++i) {
            const JsonValue& e = ents->arr[i];
            if (!e.isObj()) continue;
            const std::string type = strOr(e.get("type"), "");
            const JsonValue* ep = e.get("position");
            if (!ep || !ep->isStr()) continue;
            int dx = 0, dy = 0;
            if (!parseRelative9x9(ep->str, dx, dy)) {
                logf(1, "bad 9x9 position string, entity skipped");
                continue;
            }
            // 相对图 → 绝对坐标：用本帧 self.position 做基准（规则 §4.1）。
            const int ex = p.x + dx;
            const int ey = p.y + dy;
            // 9x9 图在地图边缘会保留界外 0；若实体的 2 落在界外，
            // 直接丢弃，避免雷达/战斗层拿到负坐标或越界坐标。
            if (ex < 0 || ex >= p.map_w || ey < 0 || ey >= p.map_h) continue;
            if (type == "robot") {
                RobotView r;
                r.id = strOr(e.get("robot_id"), "");
                if (r.id.empty()) r.id = strOr(e.get("id"), "");
                r.team = strOr(e.get("team"), "");
                r.x = ex;
                r.y = ey;
                r.hp = intOr(e.get("hp"), 100);
                r.defending = boolOr(e.get("defending"), false);
                r.protected_ = boolOr(e.get("protected"), false);
                if (!r.id.empty()) p.robots.push_back(r);
            } else if (type == "obstacle") {
                Entity t;
                t.kind = ENT_OBSTACLE;
                t.x = ex;
                t.y = ey;
                p.entities.push_back(t);
            } else if (type == "resource_normal" || type == "resource_high") {
                Entity t;
                t.kind = ENT_RESOURCE;
                t.x = ex;
                t.y = ey;
                const float def = (type == "resource_high") ? 20.f : 10.f;
                t.value = static_cast<float>(numOr(e.get("value"), def));
                t.high = t.value >= 20.f;  // 协议口径：value >= 20 为高价值
                p.entities.push_back(t);
            }
            // 未知 type：前向兼容，忽略
        }
    }

    // ---- 任务（结构化 + 自然语言）----
    const JsonValue* missions = root.get("missions");
    if (missions && missions->isArr()) {
        for (std::size_t i = 0; i < missions->arr.size(); ++i) {
            const JsonValue& m = missions->arr[i];
            if (!m.isObj()) continue;
            Mission mi;
            mi.id = strOr(m.get("id"), "");
            mi.type = strOr(m.get("type"), "");
            mi.target = intOr(m.get("target"), 0);
            const JsonValue* mp = m.get("position");
            if (mp && mp->isArr() && mp->arr.size() >= 2 &&
                mp->arr[0].isNum() && mp->arr[1].isNum()) {
                mi.has_pos = true;
                mi.px = intOr(&mp->arr[0], 0);
                mi.py = intOr(&mp->arr[1], 0);
            }
            p.missions.push_back(mi);
        }
    }
    p.mission_text = strOr(root.get("mission_text"), "");
    p.event_text = strOr(root.get("event_text"), "");
    // reach 坐标兜底：结构化 missions 缺坐标时，从任务公告原文/本场缓存补齐。
    // 这一步只依赖感知与 /referee/mission，不依赖 LLM/事件解析是否成功。
    ensureReachMission(p);
    return true;
}

// 结构化 missions 是 reach 的第一来源（协议 §3.1）；但“某种情况下”
// 裁判只给了 mission_text（或 missions 里 reach 条目没有 position），
// 旧实现会直接跳过 reach 任务，直到终局都不去任务点。这里补三条来源：
//   1) 本帧 mission_text；
//   2) /referee/mission 最近一条原文；
//   3) 本场已经解析过的坐标缓存（公告只出现一帧也不会丢）。
void Agent::ensureReachMission(Perception& p) {
    bool havePos = false;
    int noPosIdx = -1;
    for (std::size_t i = 0; i < p.missions.size(); ++i) {
        if (!isReachType(p.missions[i].type)) continue;
        if (p.missions[i].has_pos) {
            havePos = true;
            break;
        }
        if (noPosIdx < 0) noPosIdx = static_cast<int>(i);
    }
    if (havePos) return;

    int rx = -1, ry = -1;
    bool ok = extractReachCoord(p.mission_text, p.map_w, p.map_h, rx, ry);
    if (!ok) ok = extractReachCoord(referee_mission_text_, p.map_w, p.map_h, rx, ry);
    const bool changed = ok && (!reach_hint_valid_ || rx != reach_hint_x_ ||
                                ry != reach_hint_y_);
    if (ok) {
        reach_hint_valid_ = true;
        reach_hint_x_ = rx;
        reach_hint_y_ = ry;
    } else if (reach_hint_valid_ && reach_hint_x_ >= 0 && reach_hint_y_ >= 0 &&
               reach_hint_x_ < p.map_w && reach_hint_y_ < p.map_h) {
        rx = reach_hint_x_;
        ry = reach_hint_y_;
        ok = true;
    }
    if (!ok) return;

    if (noPosIdx >= 0) {
        Mission& m = p.missions[static_cast<std::size_t>(noPosIdx)];
        m.has_pos = true;
        m.px = rx;
        m.py = ry;
        if (changed)
            logf(0, "reach coordinate recovered from mission text: (" +
                        std::to_string(rx) + "," + std::to_string(ry) + ")");
    } else {
        Mission m;
        m.id = "reach_target";
        m.type = "reach";
        m.target = 1;
        m.has_pos = true;
        m.px = rx;
        m.py = ry;
        p.missions.push_back(m);
        // 只在坐标首次出现/变化时记日志：文本每 Tick 都在时不要刷屏。
        if (changed)
            logf(0, "reach mission synthesized from mission text: (" +
                        std::to_string(rx) + "," + std::to_string(ry) + ")");
    }
}

void Agent::onRefereeEventText(const std::string& text) {
    // 公告话题逐条发送（协议 §3）；同一段文本可能因 TRANSIENT_LOCAL 重放
    // 再次到达，这里按文本去重，避免重复请求消耗配额。
    std::size_t begin = 0;
    while (begin <= text.size()) {
        const std::size_t nl = text.find('\n', begin);
        const std::string line =
            trimLine(text.substr(begin, nl == std::string::npos ? std::string::npos
                                                                : nl - begin));
        if (!line.empty()) {
            // 跨源/跨重放的本场永久去重：同样一行可能先由感知 event_text
            // 提交、也可能由 /referee/event_text 的 TRANSIENT_LOCAL 重放再次
            // 到达。只查 referee_events_ 是不够的——该行出队后就查不到了。
            if (!eventLineIn(seen_event_lines_, line) &&
                !eventLineIn(referee_events_, line) &&
                referee_events_.size() < 128) {
                rememberEventLine(seen_event_lines_, line);
                referee_events_.push_back(line);
            }
        }
        if (nl == std::string::npos) break;
        begin = nl + 1;
    }
}

void Agent::onRefereeMissionText(const std::string& text) {
    const std::string t = trimLine(text);
    if (!t.empty()) referee_mission_text_ = t;
}

// ---------------------------------------------------------------------------
// 回执归因：上一 Tick 的 move 失败（moved=false）属于哪种？
//   地形性（隐藏障碍/越界）→ RECEIPT_TERRAIN，写回障碍并触发保守自愈（第一阶段行为）；
//   占位性（敌车占格/多车争格）→ RECEIPT_OCCUPANCY，只放弃当前目标 + 短时回避该格。
//
// 为什么必须区分：抢格失败是**正常现象**（§6.1：多人争格时都失败，不罚分），
// 如果把对方站着的格子当成墙，地图记忆会被永久污染，之后再也不去那片区域。
// 判据基于本 Tick 新感知：失败格在视野内、且视野里没有障碍实体 → 判定为占位。
Agent::ReceiptKind Agent::classifyReceipt(const Perception& p) const {
    if (!attempted_.valid) return RECEIPT_NONE;
    const LastAction& la = p.last_action;
    if (!la.present || la.tick != attempted_.tick) return RECEIPT_NONE;
    if (la.action != "move" || la.moved) return RECEIPT_NONE;
    if (la.error == "E_OOB") return RECEIPT_TERRAIN;  // 走出地图一定是地形
    if (!world_.inBounds(attempted_.x, attempted_.y)) return RECEIPT_TERRAIN;
    const int dx = attempted_.x >= p.x ? attempted_.x - p.x : p.x - attempted_.x;
    const int dy = attempted_.y >= p.y ? attempted_.y - p.y : p.y - attempted_.y;
    const int vr = p.vision_radius > 0 ? p.vision_radius : 0;
    if (dx > vr || dy > vr) return RECEIPT_TERRAIN;  // 失败格看不见：按隐藏障碍保守处理
    for (std::size_t i = 0; i < p.entities.size(); ++i) {
        const Entity& e = p.entities[i];
        if (e.kind == ENT_OBSTACLE && e.x == attempted_.x && e.y == attempted_.y)
            return RECEIPT_TERRAIN;  // 视野里就是障碍：确认是地形
    }
    return RECEIPT_OCCUPANCY;  // 视野内是空格却进不去 → 被占位或争格
}

void Agent::rememberAvoid(int idx, int until) {
    for (std::size_t i = 0; i < avoid_.size(); ++i) {
        if (avoid_[i].idx == idx) {
            if (until > avoid_[i].until) avoid_[i].until = until;
            return;
        }
    }
    AvoidCell c;
    c.idx = idx;
    c.until = until;
    avoid_.push_back(c);
}

// ---------------------------------------------------------------------------
// 每 Tick 刷新软阻挡。
// 两种来源：
//   1. 本帧视野内的敌车（占位是实时的）；
//   2. 最近因占位/争格失败过的格子（短时回避，过期自动失效）。
// 两者都只影响本 Tick 寻路，不写地形记忆。
void Agent::applySoftBlocks(const Perception& p) {
    world_.clearSoft();
    for (std::size_t i = 0; i < p.robots.size(); ++i) {
        const RobotView& r = p.robots[i];
        if (!world_.inBounds(r.x, r.y)) continue;
        world_.setSoft(world_.idx(r.x, r.y));
    }
    // 已解析的事件危险区也作为软阻挡：寻路绕开，而不是走过去再逃。
    for (std::size_t i = 0; i < active_events_.size(); ++i) {
        const EventReport& e = active_events_[i].report;
        if (e.kind == EVK_EXPLOSION && e.activate_tick >= p.tick &&
            e.activate_tick - p.tick <= 6) {
            for (int y = e.y0; y <= e.y1; ++y)
                for (int x = e.x0; x <= e.x1; ++x)
                    if (world_.inBounds(x, y)) world_.setSoft(world_.idx(x, y));
        } else if (e.kind == EVK_SPLIT) {
            const bool active = e.activate_tick <= p.tick &&
                                (e.end_tick < 0 || p.tick < e.end_tick);
            const bool imminent = cfg_.split_escape && e.activate_tick > p.tick &&
                                  e.activate_tick - p.tick <= cfg_.split_escape_horizon;
            if (active || imminent) {
                // 十字线：x=x0 整列 + y=y0 整行；只标真实存在的那一条。
                if (e.x0 == e.x1) {
                    for (int y = 0; y < world_.h(); ++y)
                        if (world_.inBounds(e.x0, y)) world_.setSoft(world_.idx(e.x0, y));
                }
                if (e.y0 == e.y1) {
                    for (int x = 0; x < world_.w(); ++x)
                        if (world_.inBounds(x, e.y0)) world_.setSoft(world_.idx(x, e.y0));
                }
            }
        }
    }
    // 争格死锁的修复：如果双方每 Tick 都去抢同一格，裁判判双方都失败，
    // 双方就会永远卡住（实测 500 Tick 只走了 31 步）。让该格回避几 Tick，
    // 我车就会绕路，另一辆车跟进后死锁自然解开。
    std::size_t keep = 0;
    for (std::size_t i = 0; i < avoid_.size(); ++i) {
        if (avoid_[i].until < p.tick) continue;  // 过期丢弃
        const int idx = avoid_[i].idx;
        if (idx >= 0 && world_.inBounds(idx % world_.w(), idx / world_.w())) {
            world_.setSoft(idx);
        }
        if (keep != i) avoid_[keep] = avoid_[i];
        ++keep;
    }
    avoid_.resize(keep);
}

// ---------------------------------------------------------------------------
// 接战门：SAFE 不接管；ALERT 只用启发式并给射线软代价；ENGAGED 才允许深搜。
// 换场时 threat_level_/lock/damage 必须复位（见 onPerception）。
void Agent::updateThreatLevel(const Perception& p) {
    if (!cfg_.engage_gate) {
        threat_level_ = THREAT_SAFE;
        return;
    }
    int minDist = 99;
    bool approaching = false, aggressive = false, inAxis = false;
    const std::vector<Radar::Sighting>& all = radar_.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.last_tick < 0 || p.tick - s.last_tick > 1) continue;
        const int d = std::abs(s.x - p.x) + std::abs(s.y - p.y);
        if (d < minDist) minDist = d;
        if (d <= 4 && s.last_dist >= 0 && d < s.last_dist) approaching = true;
        if (s.last_attack_tick >= 0 && p.tick - s.last_attack_tick <= 20)
            aggressive = true;
        if (d <= 4 && (s.x == p.x || s.y == p.y)) inAxis = true;
    }
    if (minDist <= 2 || p.hp <= 40) {
        threat_level_ = THREAT_ENGAGED;
        ++gate_engaged_ticks_;
        combat_lock_until_ = p.tick + 5;
        return;
    }
    if (p.tick < combat_lock_until_) {
        if (threat_level_ == THREAT_SAFE) threat_level_ = THREAT_ALERT;
        if (threat_level_ == THREAT_ENGAGED) ++gate_engaged_ticks_;
        return;
    }
    if (minDist <= 4 && (approaching || aggressive || inAxis)) {
        threat_level_ = THREAT_ALERT;
    } else if (p.tick - last_damage_tick_ <= 3) {
        threat_level_ = THREAT_ALERT;
    } else {
        threat_level_ = THREAT_SAFE;
    }
}

// v1.2 §10.1：伤害任务按“实际攻击伤害 + 击杀奖励 - 死亡扣除”后的有效伤害判定。
// 只读结构化 missions（不解析 mission_text/公告，避免与 §8.1 的事件理解红线混淆）。
// 返回 true 表示本场存在尚未完成的伤害任务，需要关闭 D 止盈并提高接战意愿。
bool Agent::damageMissionPending(const Perception& p) const {
    for (std::size_t i = 0; i < p.missions.size(); ++i) {
        const Mission& m = p.missions[i];
        if (m.type != "effective_damage" && m.type != "damage" &&
            m.type != "damage_task")
            continue;
        bool done = false;
        if (!m.id.empty()) {
            for (std::size_t k = 0; k < p.mission_completed_ids.size(); ++k)
                if (p.mission_completed_ids[k] == m.id) { done = true; break; }
        }
        if (done) continue;
        // 裁判可能晚一个 Tick 下发 completed_ids；若已达 target，先按完成处理，
        // 不让“已打满但 ID 未到”继续触发激进模式。
        if (m.target > 0 && p.effective_damage >= static_cast<float>(m.target))
            continue;
        return true;
    }
    return false;
}

// reach 死线：R - 曼哈顿 < 15；比真实步数略早，安全方向。
bool Agent::reachDeadlineUrgent(const Perception& p) const {
    const Mission* reach = reachMission(p);
    if (!reach) return false;
    if (!world_.inBounds(reach->px, reach->py)) return false;
    const int d = std::abs(reach->px - p.x) + std::abs(reach->py - p.y);
    return (cfg_.total_ticks - p.tick) - d < 15;
}

// 一击必杀：本帧目击、残血、与我同轴 1-2 格、非保护。
bool Agent::hasEasyKill(const Perception& p) const {
    if (p.protected_) return false;
    const std::vector<Radar::Sighting>& all = radar_.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.last_tick != p.tick || s.protected_ || s.hp > 20) continue;
        if (s.x != p.x && s.y != p.y) continue;
        const int d = std::abs(s.x - p.x) + std::abs(s.y - p.y);
        if (d >= 1 && d <= 2) return true;
    }
    return false;
}

// ALERT：把“我→敌人”的直线（3-4 格）标成高代价，不封死。
void Agent::applyAlertSoftCost(const Perception& p) {
    if (!cfg_.engage_gate || threat_level_ != THREAT_ALERT) return;
    const std::vector<Radar::Sighting>& all = radar_.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.last_tick != p.tick || s.protected_) continue;
        const int dx = s.x - p.x, dy = s.y - p.y;
        if (dx != 0 && dy != 0) continue;
        const int d = std::abs(dx) + std::abs(dy);
        if (d < 3 || d > 4) continue;
        const int sx = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
        const int sy = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
        int maxK = d - 1;
        if (maxK > 3) maxK = 3;
        for (int k = 1; k <= maxK; ++k) {
            const int x = p.x + sx * k, y = p.y + sy * k;
            if (!world_.inBounds(x, y)) break;
            const int idx = world_.idx(x, y);
            if (world_.state()[idx] == World::BLOCKED) continue;
            if (world_.softBlocked(idx)) continue;
            world_.setSoftCost(idx, cfg_.alert_soft_cost);
        }
    }
}

// P2：经济门压攻击时，敌车 1–2 格十字射线加软代价，资源路线绕开枪口。
void Agent::applyThreatSoftCost(const Perception& p) {
    if (cfg_.threat_soft_cost <= 0.f) return;
    const std::vector<Radar::Sighting>& all = radar_.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.last_tick < 0 || p.tick - s.last_tick > 1 || s.protected_) continue;
        const int md = std::abs(s.x - p.x) + std::abs(s.y - p.y);
        if (md > cfg_.threat_soft_range) continue;
        for (int dir = 0; dir < 4; ++dir) {
            const int sx = (dir == 0) ? 1 : (dir == 1) ? -1 : 0;
            const int sy = (dir == 2) ? 1 : (dir == 3) ? -1 : 0;
            for (int k = 1; k <= 2; ++k) {
                const int x = s.x + sx * k, y = s.y + sy * k;
                if (!world_.inBounds(x, y)) break;
                const int idx = world_.idx(x, y);
                if (world_.state()[idx] == World::BLOCKED) continue;
                if (world_.softBlocked(idx)) continue;
                world_.setSoftCost(idx, cfg_.threat_soft_cost);
            }
        }
    }
}

// 防御性苟活：残血有敌，或 D/R2 已满的垃圾时间。
bool Agent::updateDefensiveMode(const Perception& p) {
    if (!cfg_.defensive_mode) return false;
    bool enemy = false;
    const std::vector<Radar::Sighting>& all = radar_.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.last_tick >= 0 && p.tick - s.last_tick <= 1) { enemy = true; break; }
    }
    if (p.hp <= cfg_.defensive_hp && enemy) return true;
    if (p.effective_damage >= 500.f && p.resource_value >= 120.f) return true;
    return false;
}

// 苟活时把敌人周围标记为软阻挡，迫使 Planner 绕开；不封死，留出退路。
void Agent::applyDefensiveSoftBlocks(const Perception& p) {
    if (!defensive_mode_) return;
    const int r = cfg_.defensive_radius > 0 ? cfg_.defensive_radius : 2;
    const std::vector<Radar::Sighting>& all = radar_.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.last_tick < 0 || p.tick - s.last_tick > 1) continue;
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx) {
                const int x = s.x + dx, y = s.y + dy;
                if (!world_.inBounds(x, y)) continue;
                if (x == p.x && y == p.y) continue;  // 不封自己的起点
                const int idx = world_.idx(x, y);
                if (world_.state()[idx] == World::BLOCKED) continue;
                world_.setSoft(idx);
            }
    }
}

// 任务点被占的四阶争夺：A 打 / B 等 / C 冲 / D 止损。
bool Agent::tryReachContest(const Perception& p, Decision& d) {
    if (!cfg_.reach_contest) return false;
    const Mission* reach = NULL;
    for (std::size_t i = 0; i < p.missions.size(); ++i) {
        if (isReachType(p.missions[i].type) && p.missions[i].has_pos) {
            reach = &p.missions[i];
            break;
        }
    }
    if (!reach) return false;
    for (std::size_t i = 0; i < p.mission_completed_ids.size(); ++i) {
        if (p.mission_completed_ids[i] == reach->id) {
            reach_contest_ = ContestedState();
            reach_contest_id_.clear();
            return false;
        }
    }
    if (!world_.inBounds(reach->px, reach->py)) return false;
    const int tgt = world_.idx(reach->px, reach->py);
    const Radar::Sighting* blocker = radar_.at(reach->px, reach->py, p.tick, 0);
    if (!blocker) {
        reach_contest_ = ContestedState();
        reach_contest_id_.clear();
        return false;
    }
    const int dist = std::abs(reach->px - p.x) + std::abs(reach->py - p.y);
    const int R = cfg_.total_ticks - p.tick;
    int threshold = cfg_.reach_deadline_per_step * dist + cfg_.reach_deadline_margin;
    if (threshold < cfg_.reach_deadline_base) threshold = cfg_.reach_deadline_base;
    if (R > threshold) return false;
    if (reach_contest_id_ != reach->id) {
        reach_contest_ = ContestedState();
        reach_contest_id_ = reach->id;
    }
    if (reach_contest_.abandon_until > p.tick) return false;
    if (reach_contest_.wait_ticks < 1000) ++reach_contest_.wait_ticks;
    if (p.hp <= 25 && dist > 2) {
        reach_contest_.abandon_until = p.tick + 40;
        return false;
    }
    int dx = reach->px - p.x, dy = reach->py - p.y;
    int dir = 0;
    if (dx == 0 && dy != 0) dir = dy > 0 ? 3 : 4;
    else if (dy == 0 && dx != 0) dir = dx > 0 ? 1 : 2;
    // §8.8：thorns 生效时攻击正在防御的占位者 = 0 伤害 + 自己 20 反伤；
    // 任务点被占时改为走位/等待，不做亏本攻击。
    const bool thorns_block = thorns_active_ && blocker->defending &&
                              !blocker->protected_;
    const bool canShoot =
        (dir != 0) && dist >= 1 && dist <= 2 && !blocker->protected_ &&
        !thorns_block;
    // §8.9：自己易伤时，占位者反击按 40；残血时不为 reach 强行换命。
    const bool vulnerable_block =
        (self_vulnerable_until_ > p.tick) && (p.hp <= 60);
    if (dist <= 2 && canShoot && !vulnerable_block && p.hp > 40) {
        d.kind = MOVE_ATTACK;
        d.direction = direction_name(dir);
        d.reason = R_ACT_ATTACK;
        d.target_id = blocker->id;
        int dmg = blocker->defending ? 10 : 20;
        if (blocker->vulnerable_until > p.tick) dmg *= 2;  // §8.9
        d.damage = static_cast<float>(dmg);
        d.targetIdx = tgt;
        ++contest_hits_;
        return true;
    }
    const bool urgent = (R < 15);
    const bool rush = p.hp > 40 && (urgent || reach_contest_.wait_ticks >= 10);
    if (!rush) return false;
    if (canShoot && !vulnerable_block) {
        d.kind = MOVE_ATTACK;
        d.direction = direction_name(dir);
        d.reason = R_ACT_ATTACK;
        d.target_id = blocker->id;
        int dmg = blocker->defending ? 10 : 20;
        if (blocker->vulnerable_until > p.tick) dmg *= 2;  // §8.9
        d.damage = static_cast<float>(dmg);
        d.targetIdx = tgt;
        ++contest_hits_;
        return true;
    }
    const int start = world_.idx(p.x, p.y);
    const std::size_t n = static_cast<std::size_t>(world_.cells());
    if (nav_dist_.size() != n) {
        nav_dist_.assign(n, 1e9f);
        nav_parent_.assign(n, -1);
    }
    const float navChoke =
        cfg_.choke_enable
            ? cfg_.choke_w_corridor * (p.hp <= cfg_.choke_hp ? 5.f : 1.f)
            : 0.f;
    world_.dijkstraSteps(start, true, nav_dist_, nav_parent_, navChoke);
    float dsteps = nav_dist_[tgt];
    int goal = tgt;
    if (dsteps >= 1e8f) {
        static const int kNx[4] = {1, -1, 0, 0};
        static const int kNy[4] = {0, 0, 1, -1};
        int best = -1;
        float bestD = 1e9f;
        for (int k = 0; k < 4; ++k) {
            const int nx = reach->px + kNx[k], ny = reach->py + kNy[k];
            if (!world_.inBounds(nx, ny)) continue;
            const int ni = world_.idx(nx, ny);
            if (world_.state()[ni] == World::BLOCKED) continue;
            if (world_.softBlocked(ni)) continue;
            if (nav_dist_[ni] < bestD) { bestD = nav_dist_[ni]; best = ni; }
        }
        if (best < 0) return false;
        goal = best;
        dsteps = nav_dist_[goal];
    }
    if (dsteps >= 1e8f) return false;
    const int step = world_.firstStep(nav_parent_, goal, start);
    if (step < 0 || step == start) return false;
    const int w = world_.w();
    const int nx = step % w, ny = step / w;
    int mdir = 0;
    if (nx == p.x + 1 && ny == p.y) mdir = 1;
    else if (nx == p.x - 1 && ny == p.y) mdir = 2;
    else if (nx == p.x && ny == p.y + 1) mdir = 3;
    else if (nx == p.x && ny == p.y - 1) mdir = 4;
    if (mdir == 0) return false;
    d.kind = MOVE_MOVE;
    d.direction = direction_name(mdir);
    d.reason = R_MOVE_RESOURCE;
    d.targetIdx = goal;
    ++contest_hits_;
    return true;
}

// ---------------------------------------------------------------------------
// 决策编排（网关清单 §4 注入点）：
//   L0 绝对避险（事件 Veto，覆盖一切）→ reach 冲刺 → 战斗 → reach 顺路
//   → 事件目标抢占（回血点/堡垒，价值 > 1.5）→ 采集/探索。
// 事件避险是每 Tick 的反应层决策（决策文本 §附录A L0），其余走原链路。
Decision Agent::decide(const Perception& p) {
    Decision d;
    if (!p.alive) {
        damage_mission_active_ = false;
        d.reason = R_WAIT_DEAD;  // 阵亡期间只能 wait（§7；其它动作会被 E_STATE 拒绝）
        return d;
    }
    // v1.2 §10.1：伤害任务未完成时，关闭 D 止盈并提高接战意愿。
    const bool damagePending = damageMissionPending(p);
    damage_mission_active_ = damagePending;
    if (damagePending) ++damage_mission_ticks_;
    updateThreatLevel(p);
    defensive_mode_ = updateDefensiveMode(p) && !damagePending;
    if (defensive_mode_) ++defensive_ticks_;

    // ---- L0 事件避险（Veto 级）----
    // 分裂线是“立即死亡 + 无视保护”，优先于普通矩形避险：先横移离轴。
    if (trySplitEscape(p, d)) return d;

    // EventScorer 已按剩余 Tick 与 escapeSteps 判定"来不来得及"——来不及时
    // advice.avoid 为 false（死也要换分，不做无效跑路，§5.4）。
    const EventAdvice* bestAvoid = NULL;
    const EventAdvice* bestTarget = NULL;
    for (std::size_t i = 0; i < active_events_.size(); ++i) {
        ActiveEvent& ae = active_events_[i];
        ae.advice = scorer_.score(ae.report, p, world_);
        if (ae.advice.avoid && (!bestAvoid || ae.advice.value < bestAvoid->value))
            bestAvoid = &ae.advice;  // 越负越致命
        if (ae.advice.has_target && ae.advice.value > 1.5 &&
            (!bestTarget || ae.advice.value > bestTarget->value))
            bestTarget = &ae.advice;
    }
    if (bestAvoid) {
        // 逃离方向：选一个"出危险区"的相邻格——优先远离矩形中心，
        // 次选任意不在危险区内的合法格；四面堵死则交还后续链路。
        int bestDir = 0;
        long long bestScore = -1;
        for (int dir = 1; dir <= 4; ++dir) {
            const int nx = p.x + kDirDx[dir];
            const int ny = p.y + kDirDy[dir];
            if (!world_.inBounds(nx, ny)) continue;
            const int ni = world_.idx(nx, ny);
            if (world_.state()[ni] == World::BLOCKED) continue;
            if (world_.softBlocked(ni)) continue;
            long long score = 0;
            bool safe = true;
            for (std::size_t i = 0; i < active_events_.size(); ++i) {
                const EventReport& r = active_events_[i].report;
                if (!active_events_[i].advice.avoid) continue;
                const bool inRect =
                    nx >= r.x0 && nx <= r.x1 && ny >= r.y0 && ny <= r.y1;
                if (inRect) safe = false;
                // 离矩形中心越远越好（十字/单点时 x0==x1 或按线处理）
                const int cx = (r.x0 + r.x1) / 2, cy = (r.y0 + r.y1) / 2;
                score += (std::abs(nx - cx) + std::abs(ny - cy)) * 10;
            }
            if (!safe) continue;
            if (score > bestScore) {
                bestScore = score;
                bestDir = dir;
            }
        }
        if (bestDir != 0) {
            d.kind = MOVE_MOVE;
            d.direction = direction_name(bestDir);
            d.reason = R_MOVE_EVENT;
            ++event_veto_ticks_;
            return d;
        }
        // 无路可逃：不做无效移动，落到后续链路（打/防/按计划，§5.4）
    }

    // ---- reach 任务导航（v2.4 §5.3 截止倒推 / §11 冲刺基调）----
    // ---- reach 死线（最高优先级；只在事件 Veto 之后）----
    // 2026-10-30：旧实现把这条挂在 cfg_.engage_gate 下，而生产 main 不启用
    // 接战门（false）→ “死线最高优先级”在正式配置里是死代码，reach 只有在
    // 最后 45 Tick 才可能压过战斗/事件利用。这里解耦：事件解析成功与否、
    // 接战门开不开，都不再影响 reach 死线的判定。
    const bool deadlineUrgent = reachDeadlineUrgent(p);
    if (deadlineUrgent && tryReach(p, d)) return d;

    // ---- 任务点被占：四阶争夺（A 打 / B 等 / C 冲 / D 止损）----
    if (cfg_.reach_contest && tryReachContest(p, d)) return d;

    // 冲刺窗口：至少最后 45 Tick；若真实最短路比 45 长（迷宫/绕行），
    // 按 dijkstra 步数自动提前，避免“45 Tick 只够曼哈顿距离”的假安全。
    const int R_left = cfg_.total_ticks - p.tick;
    int sprintR = 45;
    if (R_left <= 90) {
        int steps = reachPathSteps(p);
        if (steps <= 0) steps = last_reach_steps_;  // 本帧不可达时用上一帧的估计
        if (steps > 0 && steps + 10 > sprintR) sprintR = steps + 10;
        if (sprintR > 90) sprintR = 90;
    }
    if (R_left <= sprintR && tryReach(p, d)) return d;

    // 战斗层可能连续接管多个 Tick，规划器不一定每 Tick 都运行；陈旧的高
    // 采集率会让经济门一直压攻击。超过 8 Tick 未刷新就归零（宁可不压）。
    if (planner_rate_tick_ >= 0 && p.tick - planner_rate_tick_ > 8)
        planner_rate_ = 0.0;
    const double oppRate = planner_rate_ > 0.0 ? planner_rate_ * 0.25 : 0.0;
    double rAlt = 0.0;
    if (cfg_.combat_opp_cost && oppRate > 0.0) {
        rAlt = oppRate;
        if (rAlt > cfg_.opp_cost_cap) rAlt = cfg_.opp_cost_cap;
    }
    // 死线豁免：最后 20 Tick 且 reach 未完成时，经济门关闭，允许战斗层强冲。
    bool reachPending = false;
    for (std::size_t i = 0; i < p.missions.size(); ++i) {
        if (!isReachType(p.missions[i].type) || !p.missions[i].has_pos) continue;
        bool done = false;
        for (std::size_t k = 0; k < p.mission_completed_ids.size(); ++k)
            if (p.mission_completed_ids[k] == p.missions[i].id) done = true;
        if (!done) reachPending = true;
        break;
    }
    const bool economyDeadlineExempt = (R_left <= 20 && reachPending);
    // 诊断口径：经济门在"最近 3 Tick 掉过血 / 开火者仍在视野"时是否还压着攻击。
    // 2026-10-28 用新增的 shadow 对手做过承伤豁免 A/B：3×hybrid 1000 局
    // 前二 74.1%→71.7%、R2 −1.6，死亡不变（D 已封顶），故不启用豁免，只留计数。
    const bool recentHit = p.tick - last_damage_tick_ <= 3;
    const bool attackerNear = radar_.recentAttacker(p.tick, 3);
    const bool duelExempt =
        cfg_.economy_gate_duel && recentHit && attackerNear &&
        p.hp <= cfg_.economy_gate_duel_hp;
    const bool economyGateActive =
        cfg_.economy_gate && !damagePending &&
        !economyDeadlineExempt && !duelExempt &&
        p.resource_value < cfg_.economy_gate_r2 &&
        p.hp > cfg_.economy_gate_hp &&
        oppRate >= cfg_.economy_gate_rate && !hasEasyKill(p);
    last_economy_gate_ = economyGateActive;

    // v1.2 §8.5：已推断获得无双增益 → 5×5 内敌人都能打，优先于普通战斗层。
    if (tryUnrivaledAttack(p, d)) return d;

    // v1.2 §8.7：眼前没有 1–2 格同轴威胁时，踩辣椒点只赚不赔。
    if (tryPepperTrap(p, d)) return d;

    if (!deadlineUrgent) {
        if (cfg_.engage_gate && combat_base_depth_ < 0)
            combat_base_depth_ = combat_.params().depth;
        bool allowCombat = true;
        int combatDepth = cfg_.combat_depth;
        if (damagePending) {
            // 伤害任务优先：不因为“安全态/经济收益”把接战关掉。
            allowCombat = true;
            combatDepth = cfg_.combat_depth;
        } else if (cfg_.engage_gate) {
            const bool dMaxed = p.effective_damage >= 500.0f;
            const bool easyKill = hasEasyKill(p);
            if (threat_level_ == THREAT_ENGAGED) {
                allowCombat = true;
                combatDepth = (dMaxed && !easyKill) ? 0 : combat_base_depth_;
            } else if (threat_level_ == THREAT_ALERT) {
                allowCombat = !dMaxed || easyKill;
                combatDepth = 0;
            } else {
                allowCombat = false;
                combatDepth = 0;
            }
        }
        if (allowCombat) {
            rmtc::Combat::Params cp = combat_.params();
            cp.damage_mission = damagePending;
            cp.thorns_active = thorns_active_;
            cp.self_vulnerable = (self_vulnerable_until_ > p.tick);
            cp.enemy_vulnerable = false;
            for (std::size_t ei = 0; ei < radar_.all().size(); ++ei) {
                const Radar::Sighting& es = radar_.all()[ei];
                if (es.vulnerable_until > p.tick &&
                    p.tick - es.last_tick <= 1) {
                    cp.enemy_vulnerable = true;
                    break;
                }
            }
            cp.d_cap_gate = cfg_.d_cap_gate && !damagePending;
            cp.opp_cost = cfg_.combat_opp_cost && !damagePending;
            if (cfg_.engage_gate) cp.depth = combatDepth;
            combat_.configure(cp);
            Decision cd;
            if (combat_.decide(world_, radar_, p, cd, rAlt)) {
                // 高价值资源争夺豁免：攻击目标脚下有 20 分资源时，允许开火。
                const bool highValueContest =
                    cd.kind == MOVE_ATTACK && cd.targetIdx >= 0 &&
                    world_.resValue(cd.targetIdx) >= cfg_.high_value;
                const bool attackHold =
                    cd.kind == MOVE_ATTACK && !hasEasyKill(p) &&
                    (economyGateActive || defensive_mode_);
                if (attackHold && !(economyGateActive && highValueContest)) {
                    if (economyGateActive) {
                        ++economy_hold_ticks_;
                        if (recentHit || attackerNear) ++economy_hold_damaged_ticks_;
                    }
                } else {
                    d = cd;
                    return d;
                }
            }
        }
    }

    // 任务导航（reach 点）排在战斗之后、事件目标之前。
    // 2026-10-30：reach 死线/冲刺窗口已经提到战斗层之前（上面），所以事件
    // 利用（无双 5×5 / 辣椒踩点）不再可能把 reach 挤到最后 45 Tick 才动身。
    if (tryReach(p, d)) return d;

    // ---- 事件目标抢占（回血点/堡垒等，价值 > 1.5 才值得离开采集）----
    // 防止"为一个恩泽跑半张地图"：阈值 1.5 分 + 贪心一步（不接管长路线）。
    if (bestTarget) {
        int stepDir = 0;
        const int dx = bestTarget->tx - p.x;
        const int dy = bestTarget->ty - p.y;
        const int adx = dx > 0 ? dx : -dx;
        const int ady = dy > 0 ? dy : -dy;
        // 先走差距大的轴，再走另一轴（贪心两步内总有一个可行）
        int order[2];
        if (adx >= ady) {
            order[0] = dx > 0 ? 1 : 2;
            order[1] = dy > 0 ? 3 : 4;
        } else {
            order[0] = dy > 0 ? 3 : 4;
            order[1] = dx > 0 ? 1 : 2;
        }
        for (int k = 0; k < 2; ++k) {
            const int dir = order[k];
            const int nx = p.x + kDirDx[dir];
            const int ny = p.y + kDirDy[dir];
            if (!world_.inBounds(nx, ny)) continue;
            const int ni = world_.idx(nx, ny);
            if (world_.state()[ni] == World::BLOCKED) continue;
            if (world_.softBlocked(ni)) continue;
            stepDir = dir;
            break;
        }
        if (stepDir != 0) {
            d.kind = MOVE_MOVE;
            d.direction = direction_name(stepDir);
            d.reason = R_MOVE_EVENT;
            d.targetIdx = world_.idx(bestTarget->tx, bestTarget->ty);
            ++event_target_ticks_;
            return d;
        }
    }

    if (cfg_.engage_gate && !deadlineUrgent) applyAlertSoftCost(p);
    applyDefensiveSoftBlocks(p);
    if (economyGateActive) applyThreatSoftCost(p);
    // P1：给刚离开的格子加软代价，打散“等长最短路每 Tick 换边”的回摆。
    // 用软代价而不是硬阻挡，死路仍可原路退出。
    if (cfg_.no_reverse && last_p_.tick >= 0 && p.tick == last_p_.tick + 1) {
        const int dx = p.x - last_p_.x;
        const int dy = p.y - last_p_.y;
        if ((dx == 0 || dy == 0) && (dx != 0 || dy != 0)) {
            const int px = p.x - dx;
            const int py = p.y - dy;
            if (world_.inBounds(px, py)) {
                const int pi = world_.idx(px, py);
                if (!world_.softBlocked(pi))
                    world_.setSoftCost(pi, cfg_.no_reverse_cost);
            }
        }
    }
    d = planner_.decide(world_, p, &radar_);
    // 缓存采集计划边际率（供下一 Tick 的战斗层当机会成本）
    // P0：只有当前动作真的是“资源移动”时才缓存；探索/等待/安全移动没有
    // “放弃采集”的机会成本，不能拿未执行的采集计划压攻击。
    bool progressOk = true;
    if (cfg_.economy_gate_progress) {
        if (d.reason == R_MOVE_RESOURCE && d.targetIdx >= 0) {
            const int tx = d.targetIdx % world_.w();
            const int ty = d.targetIdx / world_.w();
            const int dist = (p.x > tx ? p.x - tx : tx - p.x) +
                             (p.y > ty ? p.y - ty : ty - p.y);
            if (d.targetIdx == econ_target_) {
                if (dist < econ_target_dist_) {
                    econ_stall_ticks_ = 0;
                } else {
                    ++econ_stall_ticks_;
                }
            } else {
                econ_target_ = d.targetIdx;
                // 目标本身在换：也算“没有按原计划推进”，不能把 stall 清零，
                // 否则每 2-3 Tick 换一次目标就永远触发不了进度门。
                ++econ_stall_ticks_;
            }
            econ_target_dist_ = dist;
            progressOk = econ_stall_ticks_ < cfg_.economy_gate_stall;
        } else {
            econ_target_ = -1;
            econ_target_dist_ = 999;
            econ_stall_ticks_ = 0;
            progressOk = false;
        }
    }
    const bool actualResourceAction =
        (!cfg_.economy_gate_actual_action || d.reason == R_MOVE_RESOURCE) &&
        progressOk;
    if (actualResourceAction && d.planCost > 0.5f && d.planValue > 0.f) {
        planner_rate_ = d.planValue / d.planCost;
        if (planner_rate_ > cfg_.opp_cost_cap / 0.25) planner_rate_ = cfg_.opp_cost_cap / 0.25;
        planner_rate_tick_ = p.tick;
    } else {
        planner_rate_ = 0.0;  // 没有正在执行的采集计划：交战没有机会成本
        planner_rate_tick_ = p.tick;
    }

    // ---- 规划结果的安全过滤（v2.4 §12.5）----
    // 规划器只把敌车格设为软阻挡，不知道"敌方 2 格十字"是危险区，
    // 所以它会主动走进枪口。这里在低血时改选一个不在威胁下的合法方向。
    if (d.kind == MOVE_MOVE && d.direction && p.hp <= cfg_.low_hp_avoid) {
        int dx = 0, dy = 0;
        dirDelta(d.direction, dx, dy);
        if (radar_.threatensCell(p.x + dx, p.y + dy, p.tick, 1)) {
            int bestDir = 0;
            int bestScore = -1000000;
            const std::vector<Radar::Sighting>& all = radar_.all();
            for (int dir = 1; dir <= 4; ++dir) {
                const int nx = p.x + kDirDx[dir];
                const int ny = p.y + kDirDy[dir];
                if (!world_.inBounds(nx, ny)) continue;
                const int ni = world_.idx(nx, ny);
                if (world_.state()[ni] == World::BLOCKED) continue;
                if (world_.softBlocked(ni)) continue;
                if (radar_.threatensCell(nx, ny, p.tick, 1)) continue;
                int minD = 999;
                for (std::size_t i = 0; i < all.size(); ++i) {
                    const Radar::Sighting& s = all[i];
                    if (s.last_tick < 0) continue;
                    if (p.tick - s.last_tick > 3) continue;
                    const int ddx = s.x - nx, ddy = s.y - ny;
                    const int d2 = (ddx < 0 ? -ddx : ddx) > (ddy < 0 ? -ddy : ddy)
                                       ? (ddx < 0 ? -ddx : ddx)
                                       : (ddy < 0 ? -ddy : ddy);
                    if (d2 < minD) minD = d2;
                }
                const int score = minD * 10 + (dir == 1 ? 0 : 0);
                if (score > bestScore) {
                    bestScore = score;
                    bestDir = dir;
                }
            }
            if (bestDir != 0) {
                d.direction = direction_name(bestDir);
                d.reason = R_MOVE_SAFE;
            }
        }
    }

    // ---- 残血避死门（地形建议书战术一的轻量版）----
    // hp ≤ choke_hp 且规划第一步要踏入高价值咽喉（两侧都 ≥K 的要道）时，
    // 若存在"非咽喉、不被威胁"的替代方向，改走替代方向——被堵在咽喉里
    // 可达集只剩 1–2 格，安全集容易为空（"走廊=死地"的数学依据）。
    // 健康时不拦：正常路线穿过咽喉是常有的事，惩罚已由寻路软处理。
    if (cfg_.choke_enable && cfg_.choke_lowhp_gate && d.kind == MOVE_MOVE &&
        d.direction && p.hp <= cfg_.choke_hp) {
        int dx = 0, dy = 0;
        dirDelta(d.direction, dx, dy);
        const int stepIdx = world_.idx(p.x + dx, p.y + dy);
        if (world_.isChoke(stepIdx) &&
            world_.chokeValue(stepIdx) >= cfg_.choke_min_value) {
            int bestDir = 0;
            int bestVal = -1;
            for (int dir = 1; dir <= 4; ++dir) {
                const int nx = p.x + kDirDx[dir];
                const int ny = p.y + kDirDy[dir];
                if (!world_.inBounds(nx, ny)) continue;
                const int ni = world_.idx(nx, ny);
                if (world_.state()[ni] == World::BLOCKED) continue;
                if (world_.softBlocked(ni)) continue;
                if (radar_.threatensCell(nx, ny, p.tick, 0)) continue;
                // 替代方向的价值 = 离咽喉价值（越大越好）+ 非咽喉加分
                const int val = (world_.isChoke(ni) ? 0 : 100) +
                                world_.chokeValue(ni);
                if (val > bestVal) {
                    bestVal = val;
                    bestDir = dir;
                }
            }
            if (bestDir != 0) {
                d.direction = direction_name(bestDir);
                d.reason = R_MOVE_SAFE;
            }
        }
    }
    // v1.2 §8.9：潘潘窗口内“成功移动”会附加易伤；探索/游走不是必要移动，
    // 改成原地防御（防御不触发易伤，且比 WAIT 多一点减伤）。资源/任务/战斗/
    // 事件目标的移动已经在前面提前 return，不会被这条改写。
    if (cfg_.pan_caution && panMovementWindow(p) &&
        d.kind == MOVE_MOVE && d.direction &&
        (d.reason == R_MOVE_EXPLORE || d.reason == R_MOVE_SAFE)) {
        d = Decision();
        d.kind = MOVE_DEFEND;
        d.reason = R_ACT_DEFEND;
        ++pan_holds_;
    }
    // P5/P4：no_target 兜底必须放在最后。
    // 资源耗尽 → 猎杀/争夺；仍无目标 → 可选巡逻。
    if (d.reason == R_WAIT_NO_TARGET) {
        if (tryPostResourceHunt(p, d)) return d;
        if (tryNoTargetFallback(p, d)) return d;
    }
    // 死线保底（最后一层）：走到这里说明战斗/事件/规划器都没有接管。
    // 最后 15 Tick 仍允许忽略软阻挡朝任务点迈一步，保证 reach 不会因为
    // “任务点被敌车占位/刚失败过被短时回避”而整场不尝试。
    if (R_left <= 15 && tryReachDirect(p, d)) return d;
    return d;
}

// ---------------------------------------------------------------------------
// reach 任务导航：只在"再不走就来不及"时接管（§5.3 的截止倒推）。
// 判据：剩余 Tick ≤ max(160, 3 × 真实步数 + 50)。
// 返回 true 表示已把 d 填成一步移动。
const Mission* Agent::reachMission(const Perception& p) const {
    for (std::size_t i = 0; i < p.missions.size(); ++i) {
        const Mission& m = p.missions[i];
        if (!isReachType(m.type) || !m.has_pos) continue;
        bool done = false;
        for (std::size_t k = 0; k < p.mission_completed_ids.size(); ++k)
            if (p.mission_completed_ids[k] == m.id) {
                done = true;
                break;
            }
        if (!done) return &m;
    }
    return NULL;
}

// 真实最短路步数（目标被敌车占住时退到最近的空格）。只服务冲刺窗口判断，
// 与 tryReach 的 dijkstra 参数保持一致（否则窗口会短于实际路线）。
int Agent::reachPathSteps(const Perception& p) {
    const Mission* reach = reachMission(p);
    if (!reach || !world_.inBounds(reach->px, reach->py)) return -1;
    const int start = world_.idx(p.x, p.y);
    const int tgt = world_.idx(reach->px, reach->py);
    const std::size_t n = static_cast<std::size_t>(world_.cells());
    if (nav_dist_.size() != n) {
        nav_dist_.assign(n, 1e9f);
        nav_parent_.assign(n, -1);
    }
    const float navChoke =
        cfg_.choke_enable
            ? cfg_.choke_w_corridor * (p.hp <= cfg_.choke_hp ? 5.f : 1.f)
            : 0.f;
    world_.dijkstraSteps(start, true, nav_dist_, nav_parent_, navChoke);
    float dsteps = nav_dist_[tgt];
    if (dsteps >= 1e8f && world_.softBlocked(tgt)) {
        static const int kNx[4] = {1, -1, 0, 0};
        static const int kNy[4] = {0, 0, 1, -1};
        for (int k = 0; k < 4; ++k) {
            const int nx = reach->px + kNx[k], ny = reach->py + kNy[k];
            if (!world_.inBounds(nx, ny)) continue;
            const int ni = world_.idx(nx, ny);
            if (world_.state()[ni] == World::BLOCKED) continue;
            if (world_.softBlocked(ni)) continue;
            if (nav_dist_[ni] < dsteps) dsteps = nav_dist_[ni];
        }
    }
    return dsteps < 1e8f ? static_cast<int>(dsteps) : -1;
}

bool Agent::tryReach(const Perception& p, Decision& d) {
    const Mission* reach = reachMission(p);
    if (!reach) {
        last_reach_steps_ = -1;
        return false;  // 无任务 / 已完成
    }
    if (!world_.inBounds(reach->px, reach->py)) return false;

    const int start = world_.idx(p.x, p.y);
    const int tgt = world_.idx(reach->px, reach->py);
    const std::size_t n = static_cast<std::size_t>(world_.cells());
    if (nav_dist_.size() != n) {
        nav_dist_.assign(n, 1e9f);
        nav_parent_.assign(n, -1);
    }
    // 任务导航同样带咽喉软惩罚（残血做任务也别钻死地）
    const float navChoke =
        cfg_.choke_enable
            ? cfg_.choke_w_corridor * (p.hp <= cfg_.choke_hp ? 5.f : 1.f)
            : 0.f;
    world_.dijkstraSteps(start, true, nav_dist_, nav_parent_, navChoke);
    float dsteps = nav_dist_[tgt];
    int goal = tgt;

    // 目标格被敌车占住（软阻挡）时，dijkstra 会直接判定不可达，整段任务被放弃
    // （实测：停在任务点 4 格外干等 10+ Tick）。正确做法是走到目标旁边最近的
    // 可达邻格——敌车一离开就能立刻踩进去，站在旁边也更容易打断占位者。
    if (dsteps >= 1e8f) {
        if (!cfg_.reach_approach_occupied) return false;  // A/B 开关：初稿行为
        if (!world_.softBlocked(tgt)) return false;  // 真不可达（被墙围死）
        static const int kNx[4] = {1, -1, 0, 0};
        static const int kNy[4] = {0, 0, 1, -1};
        int best = -1;
        float bestD = 1e9f;
        for (int k = 0; k < 4; ++k) {
            const int nx2 = reach->px + kNx[k];
            const int ny2 = reach->py + kNy[k];
            if (!world_.inBounds(nx2, ny2)) continue;
            const int ni = world_.idx(nx2, ny2);
            if (world_.state()[ni] == World::BLOCKED) continue;
            if (world_.softBlocked(ni)) continue;
            if (nav_dist_[ni] < bestD) {
                bestD = nav_dist_[ni];
                best = ni;
            }
        }
        if (best < 0) return false;  // 四周都被堵死：本 Tick 无法接近
        goal = best;
        dsteps = nav_dist_[goal];
    }
    last_reach_steps_ = (dsteps < 1e8f) ? static_cast<int>(dsteps) : -1;

    const int R = cfg_.total_ticks - p.tick;
    int threshold = cfg_.reach_deadline_per_step * static_cast<int>(dsteps) +
                    cfg_.reach_deadline_margin;
    if (threshold < cfg_.reach_deadline_base) threshold = cfg_.reach_deadline_base;
    if (R > threshold) {
        // 机会主义：任务点就在一步内，先踩了再走。5 分任务只差一步时，
        // 不该因为“还早”而走开（实测 A 在 t=40 到过 (9,15)，任务点 (9,14)）。
        if (goal != tgt || dsteps > 1.5f) return false;
    }

    const int step = world_.firstStep(nav_parent_, goal, start);
    if (step < 0 || step == start) return false;
    const int w = world_.w();
    const int nx = step % w, ny = step / w;
    int dir = 0;
    if (nx == p.x + 1 && ny == p.y) dir = 1;
    else if (nx == p.x - 1 && ny == p.y) dir = 2;
    else if (nx == p.x && ny == p.y + 1) dir = 3;
    else if (nx == p.x && ny == p.y - 1) dir = 4;
    if (dir == 0) return false;

    d.kind = MOVE_MOVE;
    d.direction = direction_name(dir);
    d.reason = R_MOVE_RESOURCE;  // 复用：去任务点也是"去目标"
    d.targetIdx = goal;
    return true;
}

// 死线保底：最后几 Tick 如果常规寻路仍给不出一步（任务点被敌车/回避格软封死，
// 或事件层连续接管），忽略软阻挡直接朝任务点走一格。墙/边界仍然尊重——
// 撞墙只是浪费一 Tick，不会比原地等死更差；而走到任务点旁边才能让裁判判完成。
bool Agent::tryReachDirect(const Perception& p, Decision& d) {
    const Mission* reach = reachMission(p);
    if (!reach || !world_.inBounds(reach->px, reach->py)) return false;
    if (p.x == reach->px && p.y == reach->py) return false;
    int bestDir = 0;
    int bestD = std::abs(p.x - reach->px) + std::abs(p.y - reach->py);
    for (int dir = 1; dir <= 4; ++dir) {
        const int nx = p.x + kDirDx[dir], ny = p.y + kDirDy[dir];
        if (!world_.inBounds(nx, ny)) continue;
        if (world_.state()[world_.idx(nx, ny)] == World::BLOCKED) continue;
        const int dist = std::abs(nx - reach->px) + std::abs(ny - reach->py);
        if (dist < bestD) {
            bestD = dist;
            bestDir = dir;
        }
    }
    if (bestDir == 0) return false;
    d = Decision();
    d.kind = MOVE_MOVE;
    d.direction = direction_name(bestDir);
    d.reason = R_MOVE_RESOURCE;
    d.targetIdx = world_.idx(p.x + kDirDx[bestDir], p.y + kDirDy[bestDir]);
    return true;
}

// P4：no_target 连续多 Tick 时的兜底巡逻。
// 实战日志里 A 在 (17,9)/(9,9)/(1,1) 分别原地 wait 108/56/22 Tick，
// 期间既没有资源目标也没有探索前沿；Planner 的“无目标”不是“没事可做”。
// 这里只做两步：
//   1) 若相邻格有未知视野增益，走增益最大的；
//   2) 否则按 中心→四角 的 waypoint 轮换巡逻，优先走可达的。
// 仍然尊重墙/软阻挡/威胁格；找不到合法方向才返回 false。
bool Agent::tryNoTargetFallback(const Perception& p, Decision& d) {
    if (!cfg_.no_target_fallback || !p.alive) return false;
    if (no_target_streak_ < cfg_.no_target_fallback_ticks) return false;
    const int R_left = cfg_.total_ticks - p.tick;
    if (R_left <= 30) return false;
    if (p.hp <= 50) return false;  // 残血不巡逻，交给 Combat 自保
    if (p.resource_value >= cfg_.no_target_fallback_r2) return false;  // R2 已高，等待更稳
    // 附近有敌车时不巡逻：no_target 时乱走容易送人头，等 Combat/防御更稳。
    const std::vector<Radar::Sighting>& all = radar_.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.last_tick < 0 || p.tick - s.last_tick > 2) continue;
        if (std::abs(s.x - p.x) + std::abs(s.y - p.y) <= 3) return false;
    }

    std::vector<int> gain;
    world_.coverageGain(p.vision_radius, gain);
    int bestDir = 0;
    long bestScore = -1;
    for (int dir = 1; dir <= 4; ++dir) {
        const int nx = p.x + kDirDx[dir];
        const int ny = p.y + kDirDy[dir];
        if (!world_.inBounds(nx, ny)) continue;
        const int ni = world_.idx(nx, ny);
        if (world_.state()[ni] != World::FREE) continue;
        if (world_.softBlocked(ni)) continue;
        if (radar_.threatensCell(nx, ny, p.tick, 1)) continue;
        const int g = ni < static_cast<int>(gain.size()) ? gain[ni] : 0;
        if (g <= 0) continue;
        const int cd = std::abs(nx - world_.w() / 2) + std::abs(ny - world_.h() / 2);
        const long score = static_cast<long>(g) * 1000 - cd;
        if (score > bestScore) {
            bestScore = score;
            bestDir = dir;
        }
    }
    if (bestDir != 0) {
        d = Decision();
        d.kind = MOVE_MOVE;
        d.direction = direction_name(bestDir);
        d.reason = R_MOVE_EXPLORE;
        d.targetIdx = world_.idx(p.x + kDirDx[bestDir], p.y + kDirDy[bestDir]);
        return true;
    }

    const int w = world_.w(), h = world_.h();
    std::vector<float> steps;
    std::vector<int> parent;
    world_.dijkstraSteps(world_.idx(p.x, p.y), true, steps, parent);

    // 优先去“最近的未知格”：这是 no_target 时真正能带来新资源信息的方向。
    // 只做一步，下一 Tick 若仍 no_target 会沿同一条最短路继续。
    int unknownGoal = -1;
    float unknownDist = 1e9f;
    for (int i = 0; i < world_.w() * world_.h(); ++i) {
        if (world_.state()[i] != World::UNKNOWN) continue;
        if (i >= static_cast<int>(steps.size())) continue;
        if (steps[i] < unknownDist) {
            unknownDist = steps[i];
            unknownGoal = i;
        }
    }
    if (unknownGoal >= 0 && unknownDist < 1e8f) {
        const int step = world_.firstStep(parent, unknownGoal, world_.idx(p.x, p.y));
        if (step >= 0 && step != world_.idx(p.x, p.y) &&
            world_.state()[step] == World::FREE) {
            const int nx = step % w, ny = step / w;
            int dir = 0;
            if (nx == p.x + 1 && ny == p.y) dir = 1;
            else if (nx == p.x - 1 && ny == p.y) dir = 2;
            else if (nx == p.x && ny == p.y + 1) dir = 3;
            else if (nx == p.x && ny == p.y - 1) dir = 4;
            if (dir != 0) {
                d = Decision();
                d.kind = MOVE_MOVE;
                d.direction = direction_name(dir);
                d.reason = R_MOVE_EXPLORE;
                d.targetIdx = unknownGoal;
                return true;
            }
        }
    }

    const int wps[5][2] = {{w / 2, h / 2},
                           {3, 3},
                           {w - 4, 3},
                           {3, h - 4},
                           {w - 4, h - 4}};
    std::vector<float> safeSteps;
    std::vector<int> safeParent;
    world_.dijkstraSteps(world_.idx(p.x, p.y), false, safeSteps, safeParent);
    for (int k = 0; k < 5; ++k) {
        const int idx = (no_target_patrol_idx_ + k) % 5;
        const int tx = wps[idx][0], ty = wps[idx][1];
        if (!world_.inBounds(tx, ty)) continue;
        const int ti = world_.idx(tx, ty);
        if (ti == world_.idx(p.x, p.y)) continue;
        int step = -1;
        if (ti < static_cast<int>(safeSteps.size()) && safeSteps[ti] < 1e8f)
            step = world_.firstStep(safeParent, ti, world_.idx(p.x, p.y));
        if (step < 0 && ti < static_cast<int>(steps.size()) && steps[ti] < 1e8f)
            step = world_.firstStep(parent, ti, world_.idx(p.x, p.y));
        if (step < 0 || step == world_.idx(p.x, p.y)) continue;
        if (world_.state()[step] != World::FREE) continue;
        int dir = 0;
        const int nx = step % w, ny = step / w;
        if (nx == p.x + 1 && ny == p.y) dir = 1;
        else if (nx == p.x - 1 && ny == p.y) dir = 2;
        else if (nx == p.x && ny == p.y + 1) dir = 3;
        else if (nx == p.x && ny == p.y - 1) dir = 4;
        if (dir == 0) continue;
        no_target_patrol_idx_ = (idx + 1) % 5;
        d = Decision();
        d.kind = MOVE_MOVE;
        d.direction = direction_name(dir);
        d.reason = R_MOVE_EXPLORE;
        d.targetIdx = ti;
        return true;
    }
    return false;
}

// P5：已知资源全部被拿完后，no_target 不能原地等到终局。
// 目标优先级：最近的非保护敌车（允许旧目击）→ 地图中心。
// 只走一步；下一 Tick 仍 no_target 会沿同一条最短路继续。
bool Agent::tryPostResourceHunt(const Perception& p, Decision& d) {
    if (!cfg_.post_resource_hunt || !p.alive) return false;
    if (no_target_streak_ < 2) return false;
    if (world_.resValueSum() > 0.5f) return false;
    if (p.resource_value >= cfg_.post_resource_hunt_r2) return false;
    if (cfg_.total_ticks - p.tick <= 30) return false;
    if (p.hp <= 40) return false;

    int goal = -1;
    int bestDist = 1000000;
    const std::vector<Radar::Sighting>& all = radar_.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.last_tick < 0 || p.tick - s.last_tick > 120) continue;
        if (s.hp <= 0 || s.protected_) continue;
        if (!world_.inBounds(s.x, s.y)) continue;
        const int dist = std::abs(s.x - p.x) + std::abs(s.y - p.y);
        if (dist < bestDist) {
            bestDist = dist;
            goal = world_.idx(s.x, s.y);
        }
    }
    if (goal < 0) goal = world_.idx(world_.w() / 2, world_.h() / 2);
    if (goal == world_.idx(p.x, p.y)) return false;

    std::vector<float> steps;
    std::vector<int> parent;
    world_.dijkstraSteps(world_.idx(p.x, p.y), true, steps, parent);
    if (goal >= static_cast<int>(steps.size()) || steps[goal] > 1e8f) return false;
    const int step = world_.firstStep(parent, goal, world_.idx(p.x, p.y));
    if (step < 0 || step == world_.idx(p.x, p.y)) return false;
    const int w = world_.w();
    const int nx = step % w, ny = step / w;
    int dir = 0;
    if (nx == p.x + 1 && ny == p.y) dir = 1;
    else if (nx == p.x - 1 && ny == p.y) dir = 2;
    else if (nx == p.x && ny == p.y + 1) dir = 3;
    else if (nx == p.x && ny == p.y - 1) dir = 4;
    if (dir == 0) return false;
    d = Decision();
    d.kind = MOVE_MOVE;
    d.direction = direction_name(dir);
    d.reason = R_MOVE_EXPLORE;  // 复用探索动作：仍是合法移动指令
    d.targetIdx = goal;
    return true;
}

// ---------------------------------------------------------------------------
// v1.2 §8.7 火爆辣椒：点位生效期间“成功移动进入点位”会在移动后触发整行整列
// 十字火线；交点上的车辆（触发者）免疫。已站在点上/WAIT/移动失败都不触发。
// 本函数只在“点位 1–3 步内 + 同排/同列有敌人”时接管动作，成本 1–3 步、收益 50。
bool Agent::tryPepperTrap(const Perception& p, Decision& d) {
    if (!cfg_.pepper_trap) return false;
    if (!p.alive || p.protected_) return false;
    if (p.hp < 50) return false;  // 残血不玩火
    const int R_left = cfg_.total_ticks - p.tick;
    if (R_left <= 45) return false;  // 任务冲刺期不为陷阱丢任务

    const EventReport* best = NULL;
    int bestDist = 999;
    for (std::size_t i = 0; i < active_events_.size(); ++i) {
        const EventReport& r = active_events_[i].report;
        if (r.kind != EVK_PEPPER) continue;
        if (r.activate_tick < 0 || p.tick < r.activate_tick) continue;
        if (r.end_tick >= 0 && p.tick >= r.end_tick) continue;
        if (!world_.inBounds(r.x0, r.y0)) continue;
        const int d = std::abs(r.x0 - p.x) + std::abs(r.y0 - p.y);
        if (d < bestDist) {
            bestDist = d;
            best = &r;
        }
    }
    if (!best) return false;

    const int px = best->x0, py = best->y0;  // 点位坐标（LLM 给的点）
    const int dist = bestDist;
    if (dist < 1 || dist > 3) return false;
    const int ti = world_.idx(px, py);
    if (world_.state()[ti] == World::BLOCKED) return false;
    if (world_.softBlocked(ti)) return false;  // 敌车占位/危险区

    // 线上必须真的有敌人；交点（点位）上的车免疫，不计为受害者。
    int victims = 0;
    for (std::size_t i = 0; i < p.robots.size(); ++i) {
        const RobotView& r = p.robots[i];
        if (r.hp <= 0 || r.protected_) continue;
        // 眼前 1–2 格同轴有敌人时先交给战斗层（攻击/防御优先），
        // 不为陷阱放弃本 Tick 的稳定伤害/保命动作。
        if ((r.x == p.x || r.y == p.y) &&
            std::abs(r.x - p.x) + std::abs(r.y - p.y) <= 2)
            return false;
        if (r.x == px && r.y == py) continue;
        if (r.x == px || r.y == py) ++victims;
    }
    if (victims <= 0) return false;

    if (dist == 1) {
        const int dir = (px == p.x + 1 && py == p.y) ? 1
                        : (px == p.x - 1 && py == p.y) ? 2
                        : (px == p.x && py == p.y + 1) ? 3
                        : (px == p.x && py == p.y - 1) ? 4 : 0;
        if (dir == 0) return false;
        d.kind = MOVE_MOVE;
        d.direction = direction_name(dir);
        d.reason = R_MOVE_EVENT;
        d.targetIdx = ti;
        ++pepper_traps_;
        return true;
    }

    // 距离 2–3：只沿差值较大的轴走一步，不为陷阱绕路；下 Tick 敌人若离开
    // 行列，下一次评估自然放弃。
    int sx = 0, sy = 0;
    if (px != p.x) sx = (px > p.x) ? 1 : -1;
    else sy = (py > p.y) ? 1 : -1;
    const int nx = p.x + sx, ny = p.y + sy;
    if (!world_.inBounds(nx, ny)) return false;
    const int ni = world_.idx(nx, ny);
    if (world_.state()[ni] == World::BLOCKED || world_.softBlocked(ni)) return false;
    const int dir = (sx == 1) ? 1 : (sx == -1) ? 2 : (sy == 1) ? 3 : (sy == -1) ? 4 : 0;
    if (dir == 0) return false;
    d.kind = MOVE_MOVE;
    d.direction = direction_name(dir);
    d.reason = R_MOVE_EVENT;
    d.targetIdx = ti;
    ++pepper_traps_;
    return true;
}

// v1.2 §8.6 世界分裂：十字线在生效 Tick 会直接击杀线上车辆（无视保护/防御）。
// 公告已给出线坐标时，提前 1–N Tick 横移离开轴线；已经生效时由 applySoftBlocks
// 把整条线标成软阻挡，本函数只负责“人还在线上”的最后一跳。
bool Agent::trySplitEscape(const Perception& p, Decision& d) {
    if (!cfg_.split_escape || !p.alive) return false;
    const EventReport* best = NULL;
    for (std::size_t i = 0; i < active_events_.size(); ++i) {
        const EventReport& r = active_events_[i].report;
        if (r.kind != EVK_SPLIT || r.activate_tick < 0) continue;
        if (r.end_tick >= 0 && p.tick >= r.end_tick) continue;
        const bool vertical = (r.x0 == r.x1);
        const bool horizontal = (r.y0 == r.y1);
        const bool onLine = (vertical && p.x == r.x0) ||
                            (horizontal && p.y == r.y0);
        if (!onLine) continue;
        // 规则 §5.2：分裂在移动前击杀/强制复活，激活 Tick 再移动已经来不及；
        // 本 Tick 只能 WAIT，不能发出一个注定被 E_STATE 取消的 move。
        if (r.activate_tick <= p.tick) {
            d = Decision();
            d.reason = R_WAIT_NO_SAFE;
            return true;
        }
        if (r.activate_tick > p.tick &&
            r.activate_tick - p.tick > cfg_.split_escape_horizon)
            continue;
        best = &r;
        break;
    }
    if (!best) return false;

    // 垂直线横移、水平线上下移；先试远离最近敌人的一侧，再试另一侧。
    const bool vertical = (best->x0 == best->x1);
    int dirs[2];
    if (vertical) { dirs[0] = 2; dirs[1] = 1; }   // left, right
    else { dirs[0] = 4; dirs[1] = 3; }            // up, down
    int bestDir = 0;
    int bestScore = -1000000;
    for (int k = 0; k < 2; ++k) {
        const int dir = dirs[k];
        const int nx = p.x + kDirDx[dir], ny = p.y + kDirDy[dir];
        if (!world_.inBounds(nx, ny)) continue;
        const int ni = world_.idx(nx, ny);
        if (world_.state()[ni] == World::BLOCKED) continue;
        if (world_.softBlocked(ni)) continue;
        int minEnemy = 999;
        const std::vector<Radar::Sighting>& all = radar_.all();
        for (std::size_t i = 0; i < all.size(); ++i) {
            const Radar::Sighting& s = all[i];
            if (s.last_tick < 0 || p.tick - s.last_tick > 3) continue;
            const int d2 = std::abs(s.x - nx) + std::abs(s.y - ny);
            if (d2 < minEnemy) minEnemy = d2;
        }
        const int score = minEnemy * 10 + (k == 0 ? 1 : 0);
        if (score > bestScore) {
            bestScore = score;
            bestDir = dir;
        }
    }
    if (bestDir == 0) return false;
    d.kind = MOVE_MOVE;
    d.direction = direction_name(bestDir);
    d.reason = R_MOVE_EVENT;
    d.targetIdx = world_.idx(p.x + kDirDx[bestDir], p.y + kDirDy[bestDir]);
    ++split_escapes_;
    return true;
}

// v1.2 §8.9 潘潘的神：事件生效后的前 20 Tick，成功移动会给该车附加易伤。
// 这里只判断“当前是否处于会附加易伤的移动窗口”，具体是否把探索移动改成
// 防御由 decide() 末尾执行；资源/任务/战斗/事件目标移动不受影响。
bool Agent::panMovementWindow(const Perception& p) const {
    for (std::size_t i = 0; i < active_events_.size(); ++i) {
        const EventReport& r = active_events_[i].report;
        if (r.kind != EVK_VULNERABLE || r.activate_tick < 0) continue;
        if (r.end_tick >= 0 && p.tick >= r.end_tick) continue;
        if (p.tick >= r.activate_tick && p.tick < r.activate_tick + 20) return true;
    }
    return false;
}

// v1.2 §8.5 无双增益的自我推断：协议不提供 buff 字段，只能由“公告给出的
// 3×3 堡垒坐标 + 自己连续 10 Tick 位于堡垒内”推断。v1.2 明确公告坐标优先
// （多数地图是 (11,11)-(13,13)，但允许特殊变化）；离开或死亡清空未完成进度，
// 已获得的增益保留到堡垒结束。
// v1.2 §8.8/§8.9：把事件效果折算成本 Tick 的 Combat 参数。
//
// 易伤的关键时序：规则说“新获得的易伤从下一 Tick 的战斗开始影响伤害”，
// 因此本 Tick 的 Combat 只看上一 Tick 是否成功移动过；本 Tick 决策里的
// MOVE 即使成功，也要等下一帧感知（last_action.moved）才计入。
void Agent::updateEffectState(const Perception& p) {
    if (self_vulnerable_until_ >= 0 && p.tick >= self_vulnerable_until_)
        self_vulnerable_until_ = -1;
    if (illusion_from_event_until_ >= 0 && p.tick > illusion_from_event_until_)
        illusion_from_event_until_ = -1;

    // §8.3：激活 Tick 已在矩形内的车，当 Tick 就生效。提前置位，避免
    // “本 Tick MOVE 没被反转→下一 Tick 才发现”而慢 1 Tick。
    for (std::size_t i = 0; i < active_events_.size(); ++i) {
        const EventReport& r = active_events_[i].report;
        if (r.kind != EVK_ILLUSION || r.activate_tick < 0) continue;
        if (p.tick != r.activate_tick || !p.alive) continue;
        if (p.x < r.x0 || p.x > r.x1 || p.y < r.y0 || p.y > r.y1) continue;
        const int until = (r.end_tick > 0) ? r.end_tick - 1
                                           : r.activate_tick + 49;
        if (until > illusion_from_event_until_)
            illusion_from_event_until_ = until;
    }
    if (illusion_from_event_until_ > p.tick &&
        illusion_from_event_until_ > illusion_until_)
        illusion_until_ = illusion_from_event_until_;

    if (p.last_action.present && p.last_action.action == "move" &&
        p.last_action.moved && p.last_action.tick == p.tick - 1) {
        for (std::size_t i = 0; i < active_events_.size(); ++i) {
            const EventReport& r = active_events_[i].report;
            if (r.kind != EVK_VULNERABLE || r.activate_tick < 0) continue;
            if (p.last_action.tick < r.activate_tick) continue;
            // 触发窗口 [activate, activate+20)；新易伤从下一 Tick 起生效。
            if (p.last_action.tick >= r.activate_tick + 20) continue;
            if (r.end_tick >= 0 && p.last_action.tick >= r.end_tick) continue;
            const int until = (r.end_tick >= 0) ? r.end_tick
                                                : r.activate_tick + 30;
            if (until > self_vulnerable_until_) self_vulnerable_until_ = until;
        }
    }

    thorns_active_ = false;
    for (std::size_t i = 0; i < active_events_.size(); ++i) {
        const EventReport& r = active_events_[i].report;
        if (r.kind == EVK_VULNERABLE && r.activate_tick >= 0) {
            // 敌方易伤：只认雷达“连续两帧、位移 1 格”的成功移动。
            radar_.markVulnerableMoves(r.activate_tick, r.end_tick);
        }
        if (r.kind != EVK_THORNS || r.activate_tick < 0) continue;
        if (p.tick < r.activate_tick) continue;
        if (r.end_tick >= 0 && p.tick >= r.end_tick) continue;
        thorns_active_ = true;
    }
}

void Agent::updateUnrivaledState(const Perception& p) {
    if (!p.alive) {
        unrivaled_stay_ = 0;  // 死亡清空驻留；unrivaled_until_ 保留
        return;
    }
    bool inside = false;
    int end_tick = -1;
    int activate_tick = -1;
    for (std::size_t i = 0; i < active_events_.size(); ++i) {
        const EventReport& r = active_events_[i].report;
        if (r.kind != EVK_FORTRESS) continue;
        if (r.activate_tick < 0 || p.tick < r.activate_tick) continue;
        if (r.end_tick >= 0 && p.tick >= r.end_tick) continue;
        if (p.x < r.x0 || p.x > r.x1 || p.y < r.y0 || p.y > r.y1) continue;
        inside = true;
        end_tick = r.end_tick;
        activate_tick = r.activate_tick;
        break;
    }
    if (!inside) {
        unrivaled_stay_ = 0;
        return;
    }
    ++unrivaled_stay_;
    if (unrivaled_stay_ >= 10 && unrivaled_until_ <= p.tick) {
        // end_tick=-1 时退回“堡垒从激活起共 50 Tick”的标准值；
        // 公告明确时应由模型给出 end_tick，不能假设不同场次同日程。
        unrivaled_until_ = (end_tick >= 0) ? end_tick
                                           : (activate_tick >= 0 ? activate_tick + 50
                                                                 : p.tick + 40);
        ++unrivaled_grants_;
    }
}

// v1.2 §8.5：获得增益后 ATTACK 命中自身横纵 ±2 的 5×5 内全部存活敌人
// （50 / 防御 10 / 保护 0，含斜向、不受障碍遮挡、方向不限制目标集合）。
// 协议仍要求动作带一个合法方向；这里取朝最近目标的方向，不影响命中集合。
bool Agent::tryUnrivaledAttack(const Perception& p, Decision& d) {
    if (!cfg_.unrivaled_attack) return false;
    if (!p.alive || p.protected_) return false;
    if (unrivaled_until_ <= p.tick) return false;
    if (p.hp < 45) return false;  // 残血先保命，交给战斗层
    // §8.9：自己易伤时普通受击 40；5x5 无双攻击是同时结算，不能假设先杀就免伤。
    if (self_vulnerable_until_ > p.tick && p.hp <= 60) return false;

    // §8.8：无双 5x5 会命中全部目标；只要 5x5 内有正在防御的敌人，
    // 就会吃 20 反伤，且无法通过选择方向避开。保守起见本 Tick 不用无双。
    if (thorns_active_) {
        for (std::size_t i = 0; i < p.robots.size(); ++i) {
            const RobotView& r = p.robots[i];
            if (r.hp <= 0 || r.protected_ || !r.defending) continue;
            if (std::abs(r.x - p.x) <= 2 && std::abs(r.y - p.y) <= 2)
                return false;
        }
    }

    int best = -1;
    int bestScore = -1;
    for (std::size_t i = 0; i < p.robots.size(); ++i) {
        const RobotView& r = p.robots[i];
        if (r.hp <= 0 || r.protected_) continue;
        const int dx = std::abs(r.x - p.x), dy = std::abs(r.y - p.y);
        if (dx > 2 || dy > 2) continue;
        // 优先一击必杀（HP≤50），其次血量越低越好。
        const int score = (r.hp <= 50 ? 1000 : 0) + (100 - r.hp);
        if (score > bestScore) {
            bestScore = score;
            best = static_cast<int>(i);
        }
    }
    if (best < 0) return false;

    const RobotView& tgt = p.robots[static_cast<std::size_t>(best)];
    if (!world_.inBounds(tgt.x, tgt.y)) return false;
    int dir = 4;
    if (tgt.x > p.x) dir = 1;
    else if (tgt.x < p.x) dir = 2;
    else if (tgt.y > p.y) dir = 3;
    d.kind = MOVE_ATTACK;
    d.direction = direction_name(dir);
    d.reason = R_ACT_ATTACK;
    d.targetIdx = world_.idx(tgt.x, tgt.y);
    ++unrivaled_attacks_;
    return true;
}

// ---------------------------------------------------------------------------
// 动作编码。格式严格对照协议 §4 的四个示例：
//   move   → {"tick":N,"session_id":"S","action":"move","direction":"right"}
//   attack → {"tick":N,"session_id":"S","action":"attack","direction":"up"}
//   defend → {"tick":N,"session_id":"S","action":"defend"}
//   wait   → {"tick":N,"session_id":"S","action":"wait"}
// tick / session_id 必须来自本帧感知，不能沿用上一场（协议 §4）。
std::string Agent::encodeAction(const Perception& p, const Decision& d) const {
    std::string s = "{\"tick\":";
    s += std::to_string(p.tick);
    s += ",\"session_id\":\"";
    s += jsonEscape(p.session_id);
    s += "\",\"action\":\"";
    if (d.kind == MOVE_MOVE) {
        if (!d.direction) {  // 防御：绝不让 action=move 缺 direction
            s += "wait\"}";
            return s;
        }
        s += "move\",\"direction\":\"";
        s += d.direction;
        s += "\"}";
    } else if (d.kind == MOVE_ATTACK) {
        if (!d.direction) {
            s += "wait\"}";
            return s;
        }
        s += "attack\",\"direction\":\"";
        s += d.direction;
        s += "\"}";
    } else if (d.kind == MOVE_DEFEND) {
        s += "defend\"}";
    } else {
        s += "wait\"}";
    }
    return s;
}

// ---------------------------------------------------------------------------
// AI 网关三件套（网关清单 §2/§3/§4；llm_ 未挂载时全部空转）
//
// 数据流：event_text 变化 → buildEventRequestBody → llm.submit（异步）
//        → 每 Tick pumpLlm 收割 → parseLlmEvent 硬校验 → 事件池
//        → refreshEvents 重打分 → decide 消费（避险 Veto / 目标抢占）。
void Agent::pumpLlm(const Perception& p) {
    if (!llm_) return;  // llm_disabled_ 仍要排空在途结果，不能直接 return
    LlmResult res;
    while (llm_->poll(p.tick, res)) {  // 同一 Tick 可能回来多个结果
        if (res.session != p.session_id) {
            logf(1, "ignored late LLM result from session " + res.session);
            continue;
        }
        last_llm_status_ = res.status;
        last_llm_error_ = res.error;
        const bool http2xx = res.status >= 200 && res.status < 300;
        // 在途公告的结果。旧实现“2xx 就推进前缀”，模型输出不合法时这一行
        // 会被永久丢弃（真实网关偶发空 content / 截断时尤其明显）；现在改为
        // 允许有限次重试同一行，只有明确不可重试（400/401/404/413…）才放弃。
        const bool pendingEvent = (event_pending_seq_ > 0 &&
                                   res.seq == event_pending_seq_ &&
                                   res.kind == LLM_EVENT);
        if (pendingEvent) {
            event_pending_seq_ = -1;
            event_pending_tick_ = -1;
        }
        // 协议 §5：资格以“本场本席位成功（2xx）调用记录”为准，不以模型内容
        // 是否能解析为准。旧实现只在 res.ok 时落资格；若网关 2xx 但 content
        // 为空/格式异常，会误判成没资格（M=0 风险）。
        if (http2xx && !ai_quota_secured_) {
            ai_quota_secured_ = true;
            logf(0, "AI qualification secured (kind=" +
                        std::to_string(res.kind) + ", " +
                        std::to_string(res.latency_ms) + "ms, tick " +
                        std::to_string(p.tick) + ")");
        }
        if (!res.ok) {
            // 2xx 但响应信封/content 不可用：按“内容不合法”处理（可重试），
            // 不能当成永久失败把公告扔了（网关偶发空 content 是真实故障模式）。
            const bool contentIssue = http2xx;
            if (res.status == 429 &&
                res.error.find("quota_exceeded") != std::string::npos) {
                llm_disabled_ = true;
                logf(2, "AI quota exhausted (429 quota_exceeded), local-only for rest of match");
            } else if (contentIssue) {
                logf(1, "AI 2xx but unusable content (" + res.error +
                            "), will retry the same announcement");
            } else if (res.retryable && res.kind == LLM_EVENT) {
                // 429 rate_limit/concurrency_limit、传输超时、502/504：
                // 保留公告前缀（失败的这一行下一轮才重发），指数退避，
                // 避免“每 5 Tick 重发全部历史行”把队列打满。
                const int wait = event_retry_delay_;
                event_retry_after_tick_ = p.tick + wait;
                if (event_retry_delay_ < kEventRetryMax) event_retry_delay_ *= 2;
                if (event_retry_delay_ > kEventRetryMax) event_retry_delay_ = kEventRetryMax;
                logf(1, "AI event call retryable failure (status " +
                            std::to_string(res.status) + "), retry in " +
                            std::to_string(wait) + " ticks");
            } else if (res.status == 503) {
                logf(2, "AI audit_unavailable (503): keep local play, contact judge");
            } else {
                logf(1, "AI call failed: status " + std::to_string(res.status) +
                            " (" + res.error + ")");
            }
            if (pendingEvent) {
                if (contentIssue) {
                    ++event_invalid_streak_;
                    if (++event_line_attempts_ <= kEventContentRetries) {
                        event_retry_after_tick_ = p.tick + 2;
                    } else {
                        const std::string what =
                            (event_pending_is_ref_ && !referee_events_.empty())
                                ? referee_events_.front()
                                : event_pending_prefix_;
                        logf(1, "event dropped after repeated unusable content: " +
                                    what.substr(0, 80));
                        advanceEventPrefix();
                    }
                } else if (res.retryable) {
                    // 保留该行：下一轮重发。
                } else {
                    // 永久性错误（401/404/413/400…）：放弃该行，避免永久卡住。
                    advanceEventPrefix();
                }
            }
            continue;
        }
        if (res.kind == LLM_HANDSHAKE) {
            if (pendingEvent) advanceEventPrefix();
            continue;
        }
        EventReport rep;
        EventParseStatus parse_status = EVENT_PARSE_INVALID;
        if (!parseLlmEvent(res.content, p.tick, world_.w(), world_.h(), 0.5f,
                           rep, &parse_status)) {
            std::string raw = res.content.substr(0, 240);
            for (std::size_t i = 0; i < raw.size(); ++i)
                if (raw[i] == '\n' || raw[i] == '\r') raw[i] = ' ';
            if (parse_status == EVENT_PARSE_INVALID) {
                // 真正的模型错误（字段/坐标/JSON 不合法）才值得 WARN。
                ++event_invalid_streak_;
                if (event_invalid_streak_ <= 3)
                    logf(1, "event report rejected by validator (raw=" + raw + ")");
            } else if (parse_status == EVENT_PARSE_STALE) {
                // 重复公告/迟到的爆炸：协议 §3 明确“重复收到不代表重复触发”，
                // 不是错误；降到 INFO，避免终端被同一行刷屏。
                event_invalid_streak_ = 0;
                logf(0, "stale event ignored (activate already past): " +
                            raw.substr(0, 120));
            } else {
                // unknown/低置信度：模型已经正确表示“这不是新事件”
                //（“已结束/已解除”等状态行），同样不是 validator 错误。
                event_invalid_streak_ = 0;
                logf(0, "event line has no event (ignored): " +
                            raw.substr(0, 120));
            }
            if (pendingEvent) {
                advanceEventPrefix();
            }
            continue;
        }
        event_invalid_streak_ = 0;
        event_retry_delay_ = 5;
        bool isDup = false;
        for (std::size_t i = 0; i < active_events_.size(); ++i) {
            ActiveEvent& ae = active_events_[i];
            if (ae.report.kind == rep.kind &&
                std::abs(ae.report.activate_tick - rep.activate_tick) <= 2) {
                isDup = true;
                ae.report = rep;
                break;
            }
        }
        if (!isDup) {
            ActiveEvent ae;
            ae.report = rep;
            active_events_.push_back(ae);
            logf(0, "event pooled: " + std::string(eventKindName(rep.kind)) +
                        " @t" + std::to_string(rep.activate_tick) +
                        " rect(" + std::to_string(rep.x0) + "," +
                        std::to_string(rep.y0) + ")-(" +
                        std::to_string(rep.x1) + "," + std::to_string(rep.y1) + ")");
        }
        if (pendingEvent) advanceEventPrefix();  // 成功：推进到下一行
    }
    // 未落资格告警：session +60 起每 25 Tick 一次。
    if (!ai_quota_secured_ && session_start_tick_ >= 0 &&
        p.tick - session_start_tick_ >= 60 &&
        (last_ai_warn_tick_ < 0 || p.tick - last_ai_warn_tick_ >= 25)) {
        last_ai_warn_tick_ = p.tick;
        logf(2, "AI QUALIFICATION NOT SECURED (rel=" +
                    std::to_string(p.tick - session_start_tick_) +
                    ", last status=" + std::to_string(last_llm_status_) +
                    ", err=" + last_llm_error_ + ")");
    }
}

// 测试专用：直接注入结构化事件，跳过 LLM/event_text 解析。
// 生产 src/main.cpp 不调用；只用于 match4 的事件感知 A/B。
void Agent::injectTestEvent(const EventReport& r) {
    if (r.kind == EVK_UNKNOWN) return;
    ++test_events_injected_;
    for (std::size_t i = 0; i < active_events_.size(); ++i) {
        ActiveEvent& ae = active_events_[i];
        if (ae.report.kind == r.kind &&
            std::abs(ae.report.activate_tick - r.activate_tick) <= 2) {
            ae.report = r;
            return;
        }
    }
    ActiveEvent ae;
    ae.report = r;
    active_events_.push_back(ae);
}

void Agent::submitEvents(const Perception& p) {
    if (!llm_ || llm_disabled_) return;

    // 在途公告兜底：结果可能被 LlmClient 按 max_age_ticks 丢掉，或 seq/会话
    // 对不上；不主动解卡会永久卡住后续所有公告。超时后丢弃当前行继续。
    if (event_pending_seq_ > 0 && event_pending_tick_ >= 0 &&
        p.tick - event_pending_tick_ > kEventPendingTimeout) {
        logf(1, "event request stuck for " +
                    std::to_string(p.tick - event_pending_tick_) +
                    " ticks, drop current announcement and continue");
        advanceEventPrefix();
    }

    // ---- 资格握手（session 相对窗口；任务 15 分的保险丝）----
    // +2…+120 每 10 Tick（最多 10 次），之后到 +360 每 40 Tick（最多 16 次）。
    // 握手是独立请求类型，客户端不做文本去重；单次 4.5s。
    if (!ai_quota_secured_) {
        const int rel = (session_start_tick_ >= 0) ? (p.tick - session_start_tick_)
                                                   : p.tick;
        bool due = false;
        if (rel >= 2 && rel <= 120) {
            due = (last_handshake_tick_ < 0) ||
                  (p.tick - last_handshake_tick_ >= 10);
        } else if (rel > 120 && rel <= 360) {
            due = (last_handshake_tick_ < 0) ||
                  (p.tick - last_handshake_tick_ >= 40);
        }
        if (due && handshake_attempts_ < 26) {
            last_handshake_tick_ = p.tick;
            ++handshake_attempts_;
            const long seq = llm_->submit(
                p.session_id, p.tick,
                buildHandshakeRequestBody(p.tick, cfg_.llm_model), NULL,
                LLM_HANDSHAKE, cfg_.llm_handshake_timeout_ms);
            if (seq > 0)
                logf(0, "handshake #" + std::to_string(handshake_attempts_) +
                            " submitted for AI quota");
            return;  // 本 Tick 只发保底，公告下一 Tick 再发
        }
    }

    // ---- 新公告提交（客户端按文本 hash 去重；同 Tick 多条公告逐条发）----
    // 可重试失败后的退避窗口内不重复发同一公告；握手不受影响。
    if (p.tick < event_retry_after_tick_) return;
    // 一次只允许一条公告在途：等它的 2xx/429/解析结果回来再推进前缀。
    // 旧实现每个新公告都把累计文本里的历史行全部重发，队列瞬间打满并
    // 触发 429；同时 2xx 但内容不合法时也没有退避。
    if (event_pending_seq_ > 0) return;

    // ---- 公开话题 /referee/event_text 的公告优先 ----
    // 协议 §3：公告逐条发送；感知里的 event_text 只是“近期公告”缓存，容量
    // 有限，被裁掉的历史行永远不会再出现在前缀里。这里先把话题收到的公告
    // 送进 LLM，不受感知缓存滚动影响（同一文本在 LlmClient 有去重）。
    if (!referee_events_.empty()) {
        const std::string line = referee_events_.front();
        const std::string body = buildEventRequestBody(p.tick, line, cfg_.llm_model);
        bool dup = false;
        const long seq = llm_->submit(p.session_id, p.tick, body, &dup);
        if (dup) {
            referee_events_.erase(referee_events_.begin());
            event_retry_after_tick_ = -1;
            return;
        }
        if (seq <= 0) {
            event_retry_after_tick_ = p.tick + 2;
            logf(1, "referee event submit deferred (queue/quota), retry after 2 ticks");
            return;
        }
        event_pending_seq_ = seq;
        event_pending_tick_ = p.tick;
        event_pending_is_ref_ = true;
        event_pending_prefix_.clear();
        event_line_attempts_ = 0;
        return;  // 只发这一条，等结果
    }

    if (p.event_text.empty() || p.event_text == last_event_text_) return;
    if (!last_event_text_.empty() &&
        p.event_text.compare(0, last_event_text_.size(), last_event_text_) != 0) {
        last_event_text_.clear();  // 文本被改写/换场：从头来
    }
    if (p.event_text.size() <= last_event_text_.size()) return;

    std::size_t begin = last_event_text_.size();
    for (;;) {
        std::size_t nl = p.event_text.find('\n', begin);
        std::string line = p.event_text.substr(
            begin, nl == std::string::npos ? std::string::npos : nl - begin);
        const std::size_t consumed =
            (nl == std::string::npos) ? p.event_text.size() : nl + 1;
        if (line.empty()) {
            last_event_text_ = p.event_text.substr(0, consumed);
            if (nl == std::string::npos) {
                event_retry_after_tick_ = -1;
                return;
            }
            begin = nl + 1;
            continue;
        }
        // 同一行可能已经由 /referee/event_text 提交/入队，或就是感知缓存里
        // 重复出现的近期公告。协议 §3 明确重复收到不代表事件重复触发；
        // 直接推进感知前缀，绝不再发第二次请求。
        const std::string key = trimLine(line);
        if (eventLineIn(seen_event_lines_, key)) {
            last_event_text_ = p.event_text.substr(0, consumed);
            event_retry_after_tick_ = -1;
            if (nl == std::string::npos) return;
            begin = nl + 1;
            continue;
        }
        const std::string body =
            buildEventRequestBody(p.tick, line, cfg_.llm_model);
        bool dup = false;
        const long seq = llm_->submit(p.session_id, p.tick, body, &dup);
        if (dup) {
            // 同一文本已经提交过：直接推进前缀，不发第二份。
            rememberEventLine(seen_event_lines_, key);
            last_event_text_ = p.event_text.substr(0, consumed);
            event_retry_after_tick_ = -1;
            return;
        }
        if (seq <= 0) {
            event_retry_after_tick_ = p.tick + 2;
            logf(1, "event submit deferred (queue/quota), retry after 2 ticks");
            return;
        }
        rememberEventLine(seen_event_lines_, key);
        event_pending_seq_ = seq;
        event_pending_tick_ = p.tick;
        event_pending_is_ref_ = false;
        event_pending_prefix_ = p.event_text.substr(0, consumed);
        event_line_attempts_ = 0;
        return;  // 只发这一条，等结果
    }
}

// 在途公告行处理完（成功/放弃）后推进：
//   * 感知前缀行：把前缀推进到该行末尾；
//   * /referee/event_text 行：从队列头弹出一条。
void Agent::advanceEventPrefix() {
    if (event_pending_is_ref_) {
        if (!referee_events_.empty()) referee_events_.erase(referee_events_.begin());
    } else if (!event_pending_prefix_.empty()) {
        last_event_text_ = event_pending_prefix_;
    }
    event_pending_prefix_.clear();
    event_pending_is_ref_ = false;
    event_pending_seq_ = -1;
    event_pending_tick_ = -1;
    event_line_attempts_ = 0;
    event_retry_after_tick_ = -1;
}

void Agent::refreshEvents(const Perception& p) {
    if (active_events_.empty()) return;
    // 过期清理：end_tick 已过的事件移除（有效期含生效 Tick 不含结束 Tick，§8.1）
    std::size_t keep = 0;
    for (std::size_t i = 0; i < active_events_.size(); ++i) {
        const EventReport& r = active_events_[i].report;
        // 爆炸是瞬时事件：end_tick=-1 只表示“没有结束 Tick”，不是“永久有效”。
        // 旧实现会保留 400 Tick，实测 A 在 t=348 被 t=141 的爆炸区锁死，
        // 之后整局不敢朝任务点走（reach 无法完成）。
        const bool expired =
            (r.end_tick >= 0 && p.tick >= r.end_tick) ||
            (r.kind == EVK_EXPLOSION && r.activate_tick >= 0 &&
             p.tick > r.activate_tick);
        // 无 end 的兜底从 400 收紧到 60：没有结束时间的事件不能常驻整局。
        const bool stale = r.activate_tick >= 0 &&
                           p.tick - r.activate_tick >
                               (r.end_tick >= 0 ? 400 : 60);
        if (expired || stale) continue;
        if (keep != i) active_events_[keep] = active_events_[i];
        ++keep;
    }
    active_events_.resize(keep);
}

// ---------------------------------------------------------------------------
// 主入口：一帧感知进，一条动作出。
std::string Agent::onPerception(const std::string& data) {
    // §13.9 时间熔断（兜底）：从收到感知开始计时，决策超预算就退化成本 Tick 的 wait。
    // 正常实测 ~0.07 ms/决策，这里只防病态输入或机器负载异常。
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    Perception p;
    if (!parsePerception(data, p)) return "";  // 解析失败：不臆测动作，按 WAIT 结算

    // ---- 场次切换：清空上一场的一切（§4）----
    // 规划目标、世界记忆、敌情、动作缓存都不能跨场使用；
    // 否则会把上一场的地图/目标当成本场真值。
    //
    // P1 修正：迟到的"上一场"感知也会满足 session_id != session_id_，
    // 若不区分，它会把状态整体倒回旧场并清空。所以把退役场次记下来，直接丢弃。
    if (has_session_ && p.session_id != session_id_) {
        if (retired0_ == p.session_id || retired1_ == p.session_id) {
            logf(1, "late frame from retired session ignored");
            return "";
        }
    }
    if (!has_session_ || p.session_id != session_id_) {
        const bool firstSession = !has_session_;  // 首帧不等于“换场”
        if (has_session_) {  // 把旧场次退役（只留最近两个）
            retired1_ = retired0_;
            retired0_ = session_id_;
            if (retired_n_ < 2) ++retired_n_;
        }
        session_id_ = p.session_id;
        has_session_ = true;
        last_tick_ = -1;
        cached_action_.clear();
        attempted_ = Attempt();
        avoid_.clear();
        world_.reset(p.map_w, p.map_h);
        radar_.reset();
        planner_.reset();
        planner_rate_ = 0.0;
        planner_rate_tick_ = -1;
        no_target_streak_ = 0;
        no_target_patrol_idx_ = 0;
        // AI/事件状态强清理（网关清单 §4 机制二）：换场必须串不了一滴旧数据
        active_events_.clear();
        last_event_text_.clear();
        event_veto_ticks_ = 0;
        event_target_ticks_ = 0;
        test_events_injected_ = 0;
        ai_quota_secured_ = false;
        llm_disabled_ = false;
        last_handshake_tick_ = -1;
        session_start_tick_ = p.tick;
        handshake_attempts_ = 0;
        last_llm_status_ = 0;
        last_ai_warn_tick_ = -1;
        last_llm_error_.clear();
        event_retry_after_tick_ = -1;
        event_retry_delay_ = 5;
        event_invalid_streak_ = 0;
        event_pending_seq_ = -1;
        event_pending_tick_ = -1;
        self_vulnerable_until_ = -1;
        thorns_active_ = false;
        illusion_from_event_until_ = -1;
        event_pending_prefix_.clear();
        event_pending_is_ref_ = false;
        event_line_attempts_ = 0;
        // 公告话题可能在本场第一帧感知之前就到（裁判开场即发任务/事件）。
        // 首场不清空，否则“只有话题、感知里没有”的任务/事件会被直接丢掉；
        // 真正换场时再清，避免上一场的未解析公告带到新场（新场会重发）。
        if (!firstSession) {
            referee_events_.clear();
            seen_event_lines_.clear();
            referee_mission_text_.clear();
        }
        reach_hint_valid_ = false;
        reach_hint_x_ = -1;
        reach_hint_y_ = -1;
        last_reach_steps_ = -1;
        threat_level_ = THREAT_SAFE;
        combat_lock_until_ = -1;
        last_damage_tick_ = -9999;
        gate_engaged_ticks_ = 0;
        economy_hold_ticks_ = 0;
        economy_hold_damaged_ticks_ = 0;
        defensive_ticks_ = 0;
        defensive_mode_ = false;
        contest_hits_ = 0;
        reach_contest_ = ContestedState();
        reach_contest_id_.clear();
        pepper_traps_ = 0;
        unrivaled_stay_ = 0;
        unrivaled_until_ = -1;
        unrivaled_grants_ = 0;
        unrivaled_attacks_ = 0;
        damage_mission_active_ = false;
        damage_mission_ticks_ = 0;
        split_escapes_ = 0;
        pan_holds_ = 0;
        if (llm_) llm_->resetSession();
        logf(0, "new session '" + p.session_id + "', map " +
                    std::to_string(p.map_w) + "x" + std::to_string(p.map_h));
    }

    // ---- Tick 对齐 ----
    // 过期感知直接丢弃（避免旧决策覆盖新状态）；
    // 同 Tick 重发说明裁判在补发（例如暂停恢复），回放缓存动作即可，
    // 绝对不能重复推进规划器，否则一次时间步会被算两次。
    if (last_tick_ >= 0 && p.tick < last_tick_) return "";
    if (p.tick == last_tick_) return cached_action_;
    last_tick_ = p.tick;

    // P0-2 修正：self.position 必须在图内。World::idx() 不做边界检查，
    // 一旦越界会直接越界读写 dist_/parent_（堆溢出）。
    if (!world_.inBounds(p.x, p.y)) {
        logf(2, "self.position out of map, skip planning");
        cached_action_.clear();
        return "";
    }

    // ---- 1) 给上一 Tick 的移动失败归因（必须在世界更新前，用本帧原始信息）----
    const ReceiptKind receipt = classifyReceipt(p);

    // ---- 2) 更新世界（地形/资源）----
    world_.updateFromPerception(p);
    // P0-2b 修正：updateFromPerception 在 map 尺寸变化时会 reset() 世界。
    // 上面的边界检查用的是**旧**尺寸，这里必须按新尺寸再验一次，
    // 否则 decide() 里的 world_.idx(p.x, p.y) 会在缩小后的地图上越界。
    if (!world_.inBounds(p.x, p.y)) {
        logf(2, "self.position out of map after resize, skip planning");
        cached_action_.clear();
        return "";
    }
    planner_.absorbReceipt(p, world_, receipt == RECEIPT_TERRAIN);
    if (receipt == RECEIPT_OCCUPANCY &&
        world_.inBounds(attempted_.x, attempted_.y)) {
        const int idx = world_.idx(attempted_.x, attempted_.y);
        rememberAvoid(idx, p.tick + cfg_.occupied_avoid_ticks);
    }

    // ---- 3) 更新敌情，并把"实体占位 + 短时回避"写进本 Tick 的软阻挡 ----
    radar_.update(p.tick, p.x, p.y, p.robots);
    // 掉血归因必须在 radar_.update 之后：markAttackers 依赖本 Tick 的目击。
    // 不再挂在 engage_gate 下：威胁分级/诊断计数都要用这份数据。
    if (last_p_.alive && p.alive && p.hp < last_p_.hp) {
        last_damage_tick_ = p.tick;
        radar_.markAttackers(p.tick, p.x, p.y);
    }
    applySoftBlocks(p);

    // ---- 3.5) 幻术补偿（v2.4 §10.2）----
    // 判据只依赖协议字段：上一条 move 指令 vs last_action.effective_direction。
    // 两者不一致 → 方向被反转了 → 之后 50 Tick 内发指令时取反。
    // 这是唯一能在没有 LLM 的情况下合规识别的事件效果。
    if (p.last_action.present && p.last_action.action == "move" && p.last_action.moved &&
        !p.last_action.effective_direction.empty() && !last_cmd_dir_.empty()) {
        if (p.last_action.effective_direction != last_cmd_dir_) {
            // 事件公告的 end_tick 是权威时长（练习图这次是 12 Tick）；
            // 旧代码硬编码 +50，会在效果结束后继续反向补偿。
            int until = p.tick + 50;
            int eventUntil = -1;
            for (std::size_t i = 0; i < active_events_.size(); ++i) {
                const EventReport& r = active_events_[i].report;
                if (r.kind != EVK_ILLUSION) continue;
                if (r.activate_tick < 0 || p.tick < r.activate_tick) continue;
                if (r.end_tick > 0 && p.tick >= r.end_tick) continue;
                if (r.end_tick > 0 && r.end_tick - 1 > eventUntil)
                    eventUntil = r.end_tick - 1;
            }
            if (eventUntil >= 0) until = eventUntil;
            illusion_until_ = until;
        } else {
            illusion_until_ = -1;
        }
    }

    // ---- 3.6) AI 网关：收割结果 + 提交新公告 + 开场保底（网关清单 §4）----
    pumpLlm(p);
    submitEvents(p);

    // ---- 4) 决策 ----
    refreshEvents(p);  // 事件池每 Tick 重打分（decide 里消费）
    updateEffectState(p);  // v1.2 §8.8/§8.9：thorns / 自身易伤
    updateUnrivaledState(p);  // v1.2 §8.5：公告坐标 + 连续驻留推断增益
    Decision d = decide(p);
    if (d.reason == R_WAIT_NO_TARGET) {
        ++no_target_streak_;
    } else {
        no_target_streak_ = 0;
    }

    // §13.9 时间熔断：超预算时放弃本次决策，改发 wait（永远合法）。
    // 只在异常情况下触发；正常路径不会走到这里。
    if (cfg_.decision_budget_ms > 0) {
        const long long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - t0)
                                      .count();
        if (elapsed > cfg_.decision_budget_ms) {
            logf(2, "decision over budget (" + std::to_string(elapsed) + " ms), fallback wait");
            d = Decision();
        }
    }
    if (d.kind == MOVE_MOVE && d.direction && p.tick <= illusion_until_) {
        d.direction = flipName(d.direction);  // 被反转 → 发反方向
    }
    if (d.kind == MOVE_MOVE && d.direction) {
        last_cmd_dir_ = d.direction;
    } else {
        last_cmd_dir_.clear();
    }

    // ---- 5) 编码并登记本 Tick 的尝试 ----
    const std::string action = encodeAction(p, d);
    if (d.kind == MOVE_MOVE && d.direction) {
        planner_.noteAction(p.tick, p.x, p.y, d.direction);
        int dx = 0, dy = 0;
        dirDelta(d.direction, dx, dy);
        attempted_.valid = true;
        attempted_.tick = p.tick;
        attempted_.x = p.x + dx;
        attempted_.y = p.y + dy;
    } else {
        // attack/defend/wait 没有"目标格"，下一帧的移动回执不可能指它们，
        // 清掉尝试记录可避免把旧失败归因到新动作上。
        attempted_.valid = false;
    }

    last_p_ = p;
    last_d_ = d;
    cached_action_ = action;
    return action;
}

}  // namespace rmtc
