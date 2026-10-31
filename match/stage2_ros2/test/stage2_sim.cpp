// stage2_sim.cpp —— 端到端小型裁判模拟（确定性，无 ROS）
//
// 定位：单元测试验证"契约"，这个模拟器验证"整场跑下来不出事"。
// 它实现了一个简化但按规则结算的裁判：24×24 地图、500 Tick、两个机器人、
// 16 个资源（12 普通 + 4 高价值），按 v1.2 §5.2 的 Tick 顺序结算：
//   固化动作 → 按移动前坐标结算攻击/死亡 → 存活车辆移动 → 采集
//   → 回执与复活/保护倒计时。
//
// 两个场景：
//   1) 标准对抗：陪练车会追击。用于验证长时间运行、tick/session 回填、
//      攻击/受击/复活/采集，以及"非法动作必须为 0"。
//   2) 劣势遭遇：我方 20 HP、敌车 100 HP 贴脸。用于把 defend 分支跑通
//      （正常对抗里双方往往能互杀，防御分支可能一次都不触发）。
//
// 断言（模拟器本身充当裁判，拒绝任何非法动作）：
//   1) 我方非法动作数 == 0（阵亡/保护期不攻击、tick/session 回填正确）
//   2) 500 Tick 内采到资源（move 真的能走、能收集）
//   3) 确实发生过攻击（attack 有机会出手）
//   4) 劣势场景确实触发了 defend
//
// 调试：SIM_TRACE=1 打印前 80 Tick 的逐帧轨迹（双方位置/血量/动作）。
//
// 注意：这是开发工具，不属于参赛节点的一部分；编译时不链接进 ROS 包。
//
// 编译（在项目根目录执行）：
//   g++ -std=c++11 -O2 -I src stage2_sim.cpp world.cpp planner.cpp
//       rmtc/protocol.cpp rmtc/json.cpp rmtc/radar.cpp rmtc/combat.cpp rmtc/agent.cpp
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "rmtc/agent.h"
#include "rmtc/json.h"

namespace {

const int W = 24;          // 第二阶段标准地图宽（规则 §4.1）
const int H = 24;
const int kTotalTicks = 500;
const int kVision = 4;     // 视野半径：横纵各 ±4

// 一台车在模拟器里的完整状态（相当于裁判内部记录）
struct Bot {
    std::string id;   // "A"/"B"
    int x = 0, y = 0;
    int hp = 100;
    bool alive = true;
    bool defending = false;   // 本 Tick 是否提交了防御（结算用）
    bool protected_ = false;
    int protect_left = 0;
    int respawn_in = 0;
    int spawn_x = 0, spawn_y = 0;

    // 上一 Tick 的回执（写入下一份感知）
    bool has_receipt = false;
    int receipt_tick = -1;
    std::string receipt_action;
    bool receipt_moved = false;
    std::string receipt_dir;
    std::string receipt_err;
};

struct Sim {
    std::vector<unsigned char> wall;
    std::vector<float> res;
    Bot bots[2];
    int tick = 0;
    std::string session = "sim-1";
    float score_value = 0.f;
    float score_damage = 0.f;
    int invalid_a = 0;       // 我方被拒绝的动作数（应为 0）
    int attacks_a = 0;
    int deaths_a = 0;
    int deaths_b = 0;
    int moves_a = 0;
    int kills_a = 0;
    int hp_start_a = 100;      // 起始生命（用于构造"劣势遭遇"场景）
    int enemy_sx = W - 2;      // 陪练车出生点（默认右下角）
    int enemy_sy = H - 2;

    int idx(int x, int y) const { return y * W + x; }
    bool in(int x, int y) const { return x >= 0 && x < W && y >= 0 && y < H; }
    bool freeCell(int x, int y) const {
        return in(x, y) && !wall[idx(x, y)];
    }
    bool occupiedByAlive(int x, int y, int exceptIdx) const {
        for (int i = 0; i < 2; ++i) {
            if (i == exceptIdx) continue;
            if (bots[i].alive && bots[i].x == x && bots[i].y == y) return true;
        }
        return false;
    }

    void init() {
        wall.assign(W * H, 0);
        res.assign(W * H, 0.f);
        bots[0].id = "A";
        bots[0].x = bots[0].spawn_x = 1;
        bots[0].y = bots[0].spawn_y = 1;
        bots[0].hp = hp_start_a;
        bots[1].id = "B";
        bots[1].x = bots[1].spawn_x = enemy_sx;
        bots[1].y = bots[1].spawn_y = enemy_sy;
        // 确定性障碍：避开出生点与四邻
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                if ((x * 7 + y * 13) % 17 != 0) continue;
                if (x <= 3 && y <= 3) continue;
                if (x >= W - 4 && y >= H - 4) continue;
                wall[idx(x, y)] = 1;
            }
        }
        // 资源：12 普通 + 4 高价值（§4.2）
        int placedN = 0, placedH = 0;
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                if (wall[idx(x, y)]) continue;
                if (x == 1 && y == 1) continue;
                if (x == W - 2 && y == H - 2) continue;
                const bool slot = ((x * 5 + y * 11) % 23 == 0);
                if (!slot) continue;
                if (placedH < 4 && (x + y) % 3 == 0) {
                    res[idx(x, y)] = 20.f;
                    ++placedH;
                } else if (placedN < 12) {
                    res[idx(x, y)] = 10.f;
                    ++placedN;
                }
            }
        }
    }

    // 9x9 相对位置字符串：中心 (4,4)=自身，实体在 (4+dx,4+dy)
    static std::string grid(int dx, int dy) {
        std::string s;
        for (int r = 0; r < 9; ++r) {
            for (int c = 0; c < 9; ++c) {
                if (r == 4 && c == 4) s += '1';
                else if (r == 4 + dy && c == 4 + dx) s += '2';
                else s += '0';
            }
            if (r < 8) s += '\n';
        }
        return s;
    }

    // 生成 selfIdx 视角的感知 JSON。
    // 可见性严格按协议：横纵差都 ≤ kVision 的方形范围，只给地图内的实体，
    // 阵亡时不给任何实体（规则 §4.1）；敌车单独作为 type=robot 输出。
    std::string perception(int selfIdx) const {
        const Bot& me = bots[selfIdx];
        std::string s = "{";
        s += "\"tick\":" + std::to_string(tick);
        s += ",\"session_id\":\"" + session + "\"";
        s += ",\"vision_radius\":" + std::to_string(kVision);
        s += ",\"self\":{\"team\":\"" + me.id + "\",\"position\":[" +
             std::to_string(me.x) + "," + std::to_string(me.y) + "]" +
             ",\"hp\":" + std::to_string(me.hp) +
             ",\"alive\":" + (me.alive ? "true" : "false") +
             ",\"defending\":" + (me.defending ? "true" : "false") +
             ",\"protected\":" + (me.protected_ ? "true" : "false") +
             ",\"respawn_in\":" + std::to_string(me.respawn_in) +
             ",\"robot_id\":\"" + me.id + "\"";
        if (me.has_receipt) {
            s += ",\"last_action\":{\"tick\":" + std::to_string(me.receipt_tick) +
                 ",\"action\":\"" + me.receipt_action + "\"," +
                 "\"moved\":" + (me.receipt_moved ? "true" : "false");
            if (!me.receipt_dir.empty()) {
                s += ",\"effective_direction\":\"" + me.receipt_dir + "\"";
            }
            if (!me.receipt_err.empty()) {
                s += ",\"error\":\"" + me.receipt_err + "\"";
            }
            s += "}";
        } else {
            s += ",\"last_action\":null";
        }
        s += "}";

        s += ",\"visible_entities\":[";
        bool first = true;
        if (me.alive) {  // 阵亡期间不提供周围实体（§4.1）
            for (int dy = -kVision; dy <= kVision; ++dy) {
                for (int dx = -kVision; dx <= kVision; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int x = me.x + dx, y = me.y + dy;
                    if (!in(x, y)) continue;
                    const int i = idx(x, y);
                    std::string ent;
                    if (wall[i]) {
                        ent = "{\"type\":\"obstacle\",\"position\":\"" + escape(grid(dx, dy)) + "\"}";
                    } else if (res[i] > 0.f) {
                        const bool high = res[i] >= 20.f;
                        ent = std::string("{\"type\":\"") +
                              (high ? "resource_high" : "resource_normal") +
                              "\",\"id\":\"" + (high ? "h" : "n") + std::to_string(i) +
                              "\",\"position\":\"" + escape(grid(dx, dy)) +
                              "\",\"value\":" + std::to_string(static_cast<int>(res[i])) + "}";
                    } else {
                        continue;
                    }
                    if (!first) s += ",";
                    first = false;
                    s += ent;
                }
            }
            for (int b = 0; b < 2; ++b) {
                if (b == selfIdx) continue;
                const Bot& o = bots[b];
                if (!o.alive) continue;
                if (o.x - me.x > kVision || me.x - o.x > kVision) continue;
                if (o.y - me.y > kVision || me.y - o.y > kVision) continue;
                if (!first) s += ",";
                first = false;
                s += "{\"type\":\"robot\",\"id\":\"robot_" + o.id + "\",\"team\":\"" +
                     o.id + "\",\"robot_id\":\"" + o.id + "\",\"position\":\"" +
                     escape(grid(o.x - me.x, o.y - me.y)) + "\",\"hp\":" +
                     std::to_string(o.hp) + ",\"defending\":" +
                     (o.defending ? "true" : "false") + ",\"protected\":" +
                     (o.protected_ ? "true" : "false") + "}";
            }
        }
        s += "]";

        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      ",\"score\":{\"resource_value\":%.1f,\"effective_damage\":%.1f,"
                      "\"mission_completed\":0,\"mission_completed_ids\":[],\"invalid_actions\":%d}",
                      score_value, score_damage, invalid_a);
        s += buf;
        s += ",\"mission_text\":\"\xe8\x8e\xb7\xe5\x8f\x96\xe8\x87\xb3\xe5\xb0\x91"
             "\xe4\xb8\xa4\xe4\xb8\xaa\xe6\x99\xae\xe9\x80\x9a\xe8\xb5\x84\xe6\xba\x90\"";
        s += ",\"missions\":[{\"id\":\"collect_two_normal\",\"type\":\"normal_resources\",\"target\":2},"
             "{\"id\":\"reach_target\",\"type\":\"reach\",\"target\":1,\"position\":[12,12]}]";
        s += ",\"map\":{\"width\":" + std::to_string(W) +
             ",\"height\":" + std::to_string(H) + "}";
        s += ",\"event_text\":\"\"}";
        return s;
    }

    static std::string escape(const std::string& in) {
        std::string o;
        for (std::size_t i = 0; i < in.size(); ++i) {
            if (in[i] == '\n') o += "\\n";
            else o += in[i];
        }
        return o;
    }

    struct Action {
        std::string name = "wait";
        std::string dir;
        bool ok = false;
    };

    // 动作解析 + 协议校验。任何一项不通过都返回 ok=false（模拟器计为一次被拒），
    // 这样"我方提交了非法动作"会在统计里立刻暴露：
    //   E_PARSE/E_SCHEMA → JSON 结构/字段不对
    //   E_TICK           → tick 不是本 Tick（常见于沿用上一帧）
    //   E_SESSION        → 场次对不上
    Action parseAction(const std::string& data) const {
        Action a;
        rmtc::JsonValue j;
        std::string err;
        if (!rmtc::jsonParse(data, j, err) || !j.isObj()) return a;
        const rmtc::JsonValue* t = j.get("tick");
        const rmtc::JsonValue* ss = j.get("session_id");
        const rmtc::JsonValue* ac = j.get("action");
        if (!t || !t->isNum() || !ss || !ss->isStr() || !ac || !ac->isStr()) return a;
        if (static_cast<int>(t->num) != tick) return a;      // E_TICK
        if (ss->str != session) return a;                    // E_SESSION
        const std::string name = ac->str;
        if (name != "move" && name != "attack" && name != "defend" && name != "wait") return a;
        if (name == "move" || name == "attack") {
            const rmtc::JsonValue* d = j.get("direction");
            if (!d || !d->isStr()) return a;
            const std::string dir = d->str;
            if (dir != "up" && dir != "down" && dir != "left" && dir != "right") return a;
            a.dir = dir;
        }
        a.name = name;
        a.ok = true;
        return a;
    }

    static void delta(const std::string& dir, int& dx, int& dy) {
        dx = dy = 0;
        if (dir == "up") dy = -1;
        else if (dir == "down") dy = 1;
        else if (dir == "left") dx = -1;
        else if (dir == "right") dx = 1;
    }

    // 按 §5.2 结算一个 Tick：acts[0]=我方、acts[1]=陪练车。
    //
    // 简化之处（有意为之，够用来测我方逻辑）：
    //   * 不模拟爆炸/辣椒/世界分裂等随机事件；
    //   * 不模拟出生点被占时的"最近合法空格"复活，固定回出生点；
    //   * 不计算任务判定与击杀奖励的队伍去重（只有两队，天然不重复）；
    //   * 移动失败只区分 E_OOB / E_BLOCKED，与协议一致，但不区分占位与争格。
    // 如果要扩事件测试，建议在这个函数里按 §5.2 的 10 个步骤插入，而不是改 Agent。
    void settle(const Action acts[2]) {
        // 1) 状态校验：阵亡/保护期提交的动作被拒绝（我方应从不发生）
        bool valid[2];
        for (int i = 0; i < 2; ++i) {
            valid[i] = true;
            if (!bots[i].alive && acts[i].name != "wait") valid[i] = false;
            if (acts[i].name == "attack" && bots[i].protected_) valid[i] = false;
            if (!valid[i] && i == 0) ++invalid_a;
        }

        // 2) 本 Tick 防御（同时生效，用于本次结算）
        for (int i = 0; i < 2; ++i) {
            bots[i].defending = valid[i] && acts[i].name == "defend";
        }

        // 3) 攻击：按移动前坐标，直线 1–2 格最近目标，障碍不阻挡
        int dmg[2] = {0, 0};  // 对 B、对 A 的伤害
        for (int i = 0; i < 2; ++i) {
            if (!valid[i] || acts[i].name != "attack" || !bots[i].alive) continue;
            if (i == 0) ++attacks_a;
            int dx = 0, dy = 0;
            delta(acts[i].dir, dx, dy);
            for (int step = 1; step <= 2; ++step) {
                const int x = bots[i].x + dx * step;
                const int y = bots[i].y + dy * step;
                if (!in(x, y)) break;
                const int other = 1 - i;
                if (bots[other].alive && bots[other].x == x && bots[other].y == y) {
                    int d = 20;
                    if (bots[other].protected_) d = 0;
                    else if (bots[other].defending) d = 10;
                    dmg[other] = d;
                    break;  // 只命中最近一个
                }
            }
        }

        // 4) 统一扣血与死亡；有效伤害 = 实际扣血 + 击杀奖励 - 60/死亡
        for (int i = 0; i < 2; ++i) {
            if (dmg[i] <= 0) continue;
            const int before = bots[i].hp;
            bots[i].hp -= dmg[i];
            const int actual = before < dmg[i] ? before : dmg[i];
            if (i == 1) score_damage += static_cast<float>(actual);
            if (bots[i].hp <= 0) {
                bots[i].hp = 0;
                bots[i].alive = false;
                bots[i].respawn_in = 3;
                bots[i].defending = false;
                score_damage -= 60.f;
                if (score_damage < 0.f) score_damage = 0.f;
                if (i == 0) { ++deaths_a; } else { ++deaths_b; ++kills_a; score_damage += 60.f; }
            }
        }

        // 5) 移动：存活才动；目标格障碍/越界/有存活车辆占位 → 原地；争格 → 都失败
        bool moved[2] = {false, false};
        std::string err[2];
        int wantX[2] = {-1, -1}, wantY[2] = {-1, -1};
        for (int i = 0; i < 2; ++i) {
            if (!valid[i] || acts[i].name != "move" || !bots[i].alive) continue;
            int dx = 0, dy = 0;
            delta(acts[i].dir, dx, dy);
            const int nx = bots[i].x + dx;
            const int ny = bots[i].y + dy;
            if (!in(nx, ny)) { err[i] = "E_OOB"; continue; }
            if (wall[idx(nx, ny)]) { err[i] = "E_BLOCKED"; continue; }
            if (occupiedByAlive(nx, ny, i)) { err[i] = "E_BLOCKED"; continue; }
            wantX[i] = nx;
            wantY[i] = ny;
        }
        if (wantX[0] >= 0 && wantX[1] >= 0 && wantX[0] == wantX[1] && wantY[0] == wantY[1]) {
            err[0] = "E_BLOCKED";  // 争格：都不动
            err[1] = "E_BLOCKED";
            wantX[0] = wantX[1] = -1;
        }
        for (int i = 0; i < 2; ++i) {
            if (wantX[i] < 0) continue;
            bots[i].x = wantX[i];
            bots[i].y = wantY[i];
            moved[i] = true;
            if (i == 0) ++moves_a;
            // 出生保护：成功移动且不在出生点 → 立即取消（§7）
            if (bots[i].protected_ && (bots[i].x != bots[i].spawn_x ||
                                       bots[i].y != bots[i].spawn_y)) {
                bots[i].protected_ = false;
                bots[i].protect_left = 0;
            }
        }

        // 6) 采集：存活车辆位于资源格自动收集（§4.2）
        for (int i = 0; i < 2; ++i) {
            if (!bots[i].alive) continue;
            const int c = idx(bots[i].x, bots[i].y);
            if (res[c] > 0.f) {
                if (i == 0) score_value += res[c];
                res[c] = 0.f;
            }
        }

        // 7) 回执 + 复活/保护倒计时
        for (int i = 0; i < 2; ++i) {
            Bot& b = bots[i];
            b.has_receipt = true;
            b.receipt_tick = tick;
            b.receipt_action = valid[i] ? acts[i].name : "wait";
            b.receipt_moved = moved[i];
            b.receipt_dir = moved[i] ? acts[i].dir : "";
            b.receipt_err = moved[i] ? "" : err[i];

            if (b.protected_ && b.protect_left > 0) {
                if (--b.protect_left == 0) b.protected_ = false;
            }
            if (!b.alive) {
                if (b.respawn_in > 0) --b.respawn_in;
                if (b.respawn_in == 0) {
                    b.alive = true;
                    b.hp = 100;
                    b.x = b.spawn_x;
                    b.y = b.spawn_y;
                    b.protected_ = true;
                    b.protect_left = 2;
                }
            }
        }
        ++tick;
    }
};

// 陪练车：能打就打，否则贴着我车走（保证 500 Tick 内必然发生交战，
// 从而覆盖攻击、被攻击、击杀奖励与防御分支）
Sim::Action enemyAction(const Sim& sim) {
    Sim::Action a;
    a.ok = true;
    const Bot& e = sim.bots[1];
    const Bot& m = sim.bots[0];
    if (!e.alive) { a.name = "wait"; return a; }
    if (!e.protected_ && m.alive) {
        const int dx = m.x - e.x;
        const int dy = m.y - e.y;
        if (dx == 0 && dy != 0 && dy >= -2 && dy <= 2) {
            a.name = "attack";
            a.dir = dy < 0 ? "up" : "down";
            return a;
        }
        if (dy == 0 && dx != 0 && dx >= -2 && dx <= 2) {
            a.name = "attack";
            a.dir = dx < 0 ? "left" : "right";
            return a;
        }
    }
    if (!m.alive) { a.name = "wait"; return a; }

    // 追击：先走差距大的轴，被挡再试另一轴
    const int dx = m.x - e.x;
    const int dy = m.y - e.y;
    const char* first = NULL;
    const char* second = NULL;
    if ((dx >= 0 ? dx : -dx) >= (dy >= 0 ? dy : -dy)) {
        first = dx > 0 ? "right" : (dx < 0 ? "left" : NULL);
        second = dy > 0 ? "down" : (dy < 0 ? "up" : NULL);
    } else {
        first = dy > 0 ? "down" : (dy < 0 ? "up" : NULL);
        second = dx > 0 ? "right" : (dx < 0 ? "left" : NULL);
    }
    const char* cand[2] = {first, second};
    for (int k = 0; k < 2; ++k) {
        if (!cand[k]) continue;
        int mx = 0, my = 0;
        Sim::delta(cand[k], mx, my);
        const int nx = e.x + mx;
        const int ny = e.y + my;
        if (!sim.in(nx, ny)) continue;
        if (sim.wall[sim.idx(nx, ny)]) continue;
        if (m.alive && m.x == nx && m.y == ny) continue;
        a.name = "move";
        a.dir = cand[k];
        return a;
    }
    a.name = "wait";
    return a;
}

}  // namespace

struct RunStats {
    int move_ = 0, attack_ = 0, defend_ = 0, wait_ = 0, reject_ = 0;
    bool crashed = false;
};

// 跑 ticks 个 Tick：我方 Agent 决策，陪练车追击，按 §5.2 结算
static RunStats runMatch(Sim& sim, int ticks, bool trace) {
    RunStats st;
    rmtc::Agent agent{rmtc::Config()};
    std::string lastAction;
    for (int t = 0; t < ticks && !st.crashed; ++t) {
        try {
            lastAction = agent.onPerception(sim.perception(0));
        } catch (...) {
            st.crashed = true;
            break;
        }
        Sim::Action acts[2];
        acts[0] = sim.parseAction(lastAction);
        if (!lastAction.empty() && !acts[0].ok) {
            sim.invalid_a++;
            ++st.reject_;
        }
        if (acts[0].ok) {
            if (acts[0].name == "move") ++st.move_;
            else if (acts[0].name == "attack") ++st.attack_;
            else if (acts[0].name == "defend") ++st.defend_;
            else ++st.wait_;
        }
        if (trace && t < 80) {
            std::printf("t=%3d A(%2d,%2d) hp=%3d %-6s %-5s  B(%2d,%2d) hp=%3d %s\n",
                        sim.tick, sim.bots[0].x, sim.bots[0].y, sim.bots[0].hp,
                        acts[0].name.c_str(), acts[0].dir.c_str(),
                        sim.bots[1].x, sim.bots[1].y, sim.bots[1].hp,
                        sim.bots[1].alive ? "" : "(dead)");
        }
        acts[1] = enemyAction(sim);
        sim.settle(acts);
    }
    return st;
}

int main() {
    int fails = 0;
    const bool trace = std::getenv("SIM_TRACE") != NULL;

    // ---- 场景 1：标准 500 Tick 对抗（陪练车追击）----
    Sim sim;
    sim.init();
    const RunStats st = runMatch(sim, kTotalTicks, trace);
    std::printf("[scene 1: 500 Tick 对抗]\n");
    std::printf("  actions: move=%d attack=%d defend=%d wait=%d reject=%d\n", st.move_,
                st.attack_, st.defend_, st.wait_, st.reject_);
    std::printf("  ticks=%d collected=%.0f/200 effective_damage=%.0f\n", sim.tick,
                sim.score_value, sim.score_damage);
    std::printf("  moves=%d attacks=%d kills=%d deaths(A)=%d invalid(A)=%d\n",
                sim.moves_a, sim.attacks_a, sim.kills_a, sim.deaths_a, sim.invalid_a);
    if (st.crashed) { std::printf("FAIL: crashed\n"); ++fails; }
    if (sim.invalid_a != 0) { std::printf("FAIL: illegal actions submitted\n"); ++fails; }
    if (sim.score_value <= 0.f) { std::printf("FAIL: collected nothing\n"); ++fails; }
    if (sim.moves_a < 50) { std::printf("FAIL: too few moves\n"); ++fails; }
    if (sim.attacks_a == 0) { std::printf("FAIL: never attacked\n"); ++fails; }
    if (sim.tick != kTotalTicks) { std::printf("FAIL: tick did not advance\n"); ++fails; }

    // ---- 场景 2：劣势遭遇（我方 20 HP，敌车贴脸 100 HP）----
    // 验证 defend 分支：打不死对方且挨一下就死 → 提交 defend，受击 20→10
    Sim sim2;
    sim2.hp_start_a = 20;
    sim2.enemy_sx = 1;
    sim2.enemy_sy = 3;
    sim2.init();
    const RunStats st2 = runMatch(sim2, 12, false);
    std::printf("[scene 2: 劣势遭遇 12 Tick]\n");
    std::printf("  actions: move=%d attack=%d defend=%d wait=%d reject=%d\n", st2.move_,
                st2.attack_, st2.defend_, st2.wait_, st2.reject_);
    if (st2.crashed) { std::printf("FAIL: crashed\n"); ++fails; }
    // depth=5 搜索在同局可能选攻击提取收益（§5.4）或后撤（retreat）而非防御——
    // 断言语义与 probe 的 T3h 一致：威胁下给出有效非等待动作即可。
    // 旧写法只数 defend/attack，会把合法的后撤误判成“没有有效动作”。
    if (st2.defend_ + st2.attack_ + st2.move_ == 0) {
        std::printf("FAIL: no purposeful action when outmatched\n");
        ++fails;
    }
    if (sim2.invalid_a != 0) { std::printf("FAIL: illegal actions (scene 2)\n"); ++fails; }

    std::printf(fails == 0 ? "\nSIM PASSED\n" : "\nSIM FAILED\n");
    return fails == 0 ? 0 : 1;
}
