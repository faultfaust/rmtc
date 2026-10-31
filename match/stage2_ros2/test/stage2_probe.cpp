// stage2_probe.cpp —— 第二阶段基础层离线单元测试（不需要 ROS）
//
// 定位：这是"改代码前先跑一遍、改完再跑一遍"的回归测试。它不测策略好坏，
// 只测**契约**：解析对不对、动作编码合不合协议、状态机有没有被破坏。
//
// 覆盖：
//   T1  9×9 相对位置图解析（含同格 [0,0]、长度非法）
//   T2  协议 PDF 的感知示例逐字段解析（tick/session/实体/任务/回执/地图）
//   T3  attack / defend / wait / move 全部分支（含保护、挡后方、可击杀优先）
//   T4  敌车离开视野后，靠记忆判定威胁（防御）与记忆过期
//   T5  采集移动（走向资源 → 逐步靠近 → 收集后继续探索）
//   T6  Tick/场次对齐（过期丢弃、同 Tick 回放、换场清空、坏输入不崩）
//   T7  敌车占位 → 寻路绕行
//   T8  移动失败归因（占位不写墙 / E_OOB 按地形处理）
//   T12 高价值资源时间竞争门（race_gate：明显先到的剔除、更远保留、raceDrops 计数）
//
// 新增测试的写法：
//   1. 用 PBuilder 拼一帧感知（tick 递增，实体用相对位移描述）；
//   2. agent.onPerception(p.build()) 拿到动作 JSON；
//   3. parseAct() 转成 Act 结构，用 CHECK 断言 action/direction/tick/session；
//   4. 需要看内部状态时直接用 agent.world() / agent.radar() 等访问器。
//
// 编译（在 src 目录下执行）：
//   g++ -std=c++11 -O2 -I . stage2_probe.cpp world.cpp planner.cpp
//       rmtc/protocol.cpp rmtc/json.cpp rmtc/radar.cpp rmtc/combat.cpp rmtc/agent.cpp
#include <cstdio>
#include <string>
#include <vector>

#include "rmtc/agent.h"
#include "rmtc/json.h"
#include "rmtc/protocol.h"
#include "rmtc/radar.h"

// 最简断言：累计通过/失败数，最后统一打印（不引入 gtest，保持零依赖）。
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg)                                        \
    do {                                                        \
        if (cond) { ++g_pass; }                                 \
        else { ++g_fail; std::printf("FAIL: %s\n", msg); }      \
    } while (0)

// 把普通字符串转成 JSON 字符串字面量（用于拼 position 里的换行）。
static std::string jsonStr(const std::string& s) {
    return "\"" + rmtc::jsonEscape(s) + "\"";
}

// 构造协议里的 9x9 相对位置图：'1' 固定中心，'2' 在 (4+dx, 4+dy)。
// dx/dy 是"相对本车的位移"，与地图轴一致（x 向右、y 向下）。
static std::string grid9(int dx, int dy) {
    std::string out;
    for (int r = 0; r < 9; ++r) {
        for (int c = 0; c < 9; ++c) {
            char ch = '0';
            if (r == 4 && c == 4) ch = '1';
            else if (r == 4 + dy && c == 4 + dx) ch = '2';
            out += ch;
        }
        if (r < 8) out += '\n';
    }
    return out;
}

// 感知构造器：坐标用相对位移（和协议一致），内部转 9x9 图。
// 链式写法：
//   PBuilder p; p.tick = 10; p.robot(1, 0, 60).res(2, 0, 10);
//   std::string json = p.build();
// 注意：这里的 x/y 是"本车绝对坐标"，robot/res/obs 的参数是相对本车的位移。
struct PBuilder {
    int tick = 1;
    int x = 10, y = 10;
    int hp = 100;
    bool alive = true;
    bool prot = false;
    int vision = 4;
    int mapw = 24, maph = 24;
    float res_value = 0.f;
    std::string session = "s1";
    std::string last_action;  // 完整 JSON 或空（= null）
    std::vector<std::string> ents;
    std::string missions_json;

    PBuilder& robot(int dx, int dy, int hpR, bool def = false, bool protR = false,
                    const char* id = "B") {
        ents.push_back("{\"type\":\"robot\",\"id\":\"robot_" + std::string(id) +
                       "\",\"team\":\"" + std::string(id) + "\",\"robot_id\":\"" +
                       std::string(id) + "\",\"position\":" + jsonStr(grid9(dx, dy)) +
                       ",\"hp\":" + std::to_string(hpR) +
                       ",\"defending\":" + (def ? "true" : "false") +
                       ",\"protected\":" + (protR ? "true" : "false") + "}");
        return *this;
    }
    PBuilder& res(int dx, int dy, int val) {
        ents.push_back("{\"type\":\"" + std::string(val >= 20 ? "resource_high" : "resource_normal") +
                       "\",\"id\":\"r01\",\"position\":" + jsonStr(grid9(dx, dy)) +
                       ",\"value\":" + std::to_string(val) + "}");
        return *this;
    }
    PBuilder& obs(int dx, int dy) {
        ents.push_back("{\"type\":\"obstacle\",\"position\":" + jsonStr(grid9(dx, dy)) + "}");
        return *this;
    }

    std::string build() const {
        std::string s = "{";
        s += "\"tick\":" + std::to_string(tick);
        s += ",\"session_id\":" + jsonStr(session);
        s += ",\"vision_radius\":" + std::to_string(vision);
        s += ",\"self\":{\"team\":\"A\",\"position\":[" + std::to_string(x) + "," +
             std::to_string(y) + "],\"hp\":" + std::to_string(hp) +
             ",\"alive\":" + (alive ? "true" : "false") +
             ",\"defending\":false,\"protected\":" + (prot ? "true" : "false") +
             ",\"respawn_in\":0,\"robot_id\":\"A\",\"last_action\":" +
             (last_action.empty() ? std::string("null") : last_action) + "}";
        s += ",\"visible_entities\":[";
        for (std::size_t i = 0; i < ents.size(); ++i) {
            if (i) s += ",";
            s += ents[i];
        }
        s += "]";
        char score[128];
        std::snprintf(score, sizeof(score),
                      ",\"score\":{\"resource_value\":%.1f,\"effective_damage\":0,"
                      "\"mission_completed\":0,\"mission_completed_ids\":[],\"invalid_actions\":0}",
                      res_value);
        s += score;
        s += ",\"mission_text\":\"\"";
        s += ",\"missions\":" + (missions_json.empty() ? std::string("[]") : missions_json);
        s += ",\"map\":{\"width\":" + std::to_string(mapw) +
             ",\"height\":" + std::to_string(maph) + "}";
        s += ",\"event_text\":\"\"}";
        return s;
    }
};

struct Act {
    bool ok = false;
    int tick = -999;
    std::string session, action, dir;
};

static Act parseAct(const std::string& s) {
    Act a;
    rmtc::JsonValue j;
    std::string err;
    if (!rmtc::jsonParse(s, j, err) || !j.isObj()) return a;
    const rmtc::JsonValue* t = j.get("tick");
    const rmtc::JsonValue* ac = j.get("action");
    if (!t || !t->isNum() || !ac || !ac->isStr()) return a;
    a.ok = true;
    a.tick = static_cast<int>(t->num);
    a.action = ac->str;
    const rmtc::JsonValue* d = j.get("direction");
    if (d && d->isStr()) a.dir = d->str;
    const rmtc::JsonValue* ss = j.get("session_id");
    if (ss && ss->isStr()) a.session = ss->str;
    return a;
}

int main() {
    // ---------------- T1: 9x9 相对图解析 ----------------
    {
        int dx = 99, dy = 99;
        CHECK(rmtc::parseRelative9x9(grid9(1, -2), dx, dy) && dx == 1 && dy == -2,
              "T1 grid9(1,-2) roundtrip");
        CHECK(rmtc::parseRelative9x9(grid9(0, 0), dx, dy) && dx == 0 && dy == 0,
              "T1 overlap -> [0,0]");
        CHECK(!rmtc::parseRelative9x9("000", dx, dy), "T1 bad size rejected");
    }

    // ---------------- T2: 协议 PDF 感知示例逐字段 ----------------
    {
        rmtc::Agent agent{rmtc::Config()};
        PBuilder p;
        p.tick = 148;
        p.session = "example-match";
        p.x = 8;
        p.y = 8;
        p.hp = 70;
        p.robot(1, 1, 60);   // robot_B -> (9,9)
        p.res(0, 1, 10);     // n09     -> (8,9)
        p.res(1, -2, 20);    // h01     -> (9,6)
        p.obs(2, 1);         // obstacle-> (10,9)
        p.res_value = 30.f;
        p.last_action =
            "{\"tick\":147,\"action\":\"move\",\"moved\":true,\"effective_direction\":\"right\"}";
        p.missions_json =
            "[{\"id\":\"collect_two_normal\",\"type\":\"normal_resources\",\"target\":2},"
            "{\"id\":\"collect_high_value\",\"type\":\"high_resources\",\"target\":1},"
            "{\"id\":\"reach_target\",\"type\":\"reach\",\"target\":1,\"position\":[12,12]}]";
        const std::string act = agent.onPerception(p.build());
        Act a = parseAct(act);
        CHECK(a.ok && a.tick == 148 && a.session == "example-match",
              "T2 action echoes tick+session");
        CHECK(agent.lastTick() == 148, "T2 lastTick");
        rmtc::World& w = agent.world();
        CHECK(w.w() == 24 && w.h() == 24, "T2 map size");
        CHECK(w.resValue(w.idx(8, 9)) == 10.f && w.resValue(w.idx(9, 6)) == 20.f,
              "T2 resources at abs coords");
        CHECK(w.state()[w.idx(10, 9)] == rmtc::World::BLOCKED, "T2 obstacle blocked");
        CHECK(agent.radar().all().size() == 1 && agent.radar().all()[0].x == 9 &&
                  agent.radar().all()[0].y == 9 && agent.radar().all()[0].hp == 60,
              "T2 radar sighting");
        const rmtc::Perception& lp = agent.lastPerception();
        CHECK(lp.hp == 70 && lp.resource_value == 30.f && lp.mission_completed_ids.empty(),
              "T2 self/score fields");
        CHECK(lp.last_action.present && lp.last_action.tick == 147 &&
                  lp.last_action.moved && lp.last_action.effective_direction == "right",
              "T2 last_action receipt");
        CHECK(lp.missions.size() == 3 && lp.missions[2].has_pos &&
                  lp.missions[2].px == 12 && lp.missions[2].py == 12,
              "T2 missions parsed");
    }

    // ---------------- T3: attack / defend / wait 判定 ----------------
    {
        rmtc::Agent agent{rmtc::Config()};
        // (a) 邻格敌车，未防御 → attack right，20 伤害
        PBuilder p1;
        p1.tick = 10;
        p1.robot(1, 0, 60);
        Act a = parseAct(agent.onPerception(p1.build()));
        CHECK(a.ok && a.action == "attack" && a.dir == "right", "T3a attack adjacent");
        // (b) 距离 2 同行 → attack
        PBuilder p2;
        p2.tick = 11;
        p2.robot(2, 0, 40);
        a = parseAct(agent.onPerception(p2.build()));
        CHECK(a.ok && a.action == "attack" && a.dir == "right", "T3b attack range2");
        // (c) 敌车防御中 → 仍攻击（10 伤害）
        PBuilder p3;
        p3.tick = 12;
        p3.robot(0, -1, 50, true);
        a = parseAct(agent.onPerception(p3.build()));
        CHECK(a.ok && a.action == "attack" && a.dir == "up", "T3c attack defending");
        // (d) 敌车出生保护 → 0 伤害不出手（避免无意义攻击）
        PBuilder p4;
        p4.tick = 13;
        p4.robot(0, 1, 50, false, true);
        a = parseAct(agent.onPerception(p4.build()));
        CHECK(a.ok && a.action != "attack", "T3d protected target no attack");
        // (e) 自己出生保护 → 不能 attack（E_STATE），交给规划器
        PBuilder p5;
        p5.tick = 14;
        p5.prot = true;
        p5.robot(1, 0, 60);
        a = parseAct(agent.onPerception(p5.build()));
        CHECK(a.ok && a.action != "attack", "T3e self protected no attack");
        // (f) 斜角敌车不在攻击线 → 不攻击
        PBuilder p6;
        p6.tick = 15;
        p6.robot(1, 1, 60);
        a = parseAct(agent.onPerception(p6.build()));
        CHECK(a.ok && a.action != "attack", "T3f diagonal no attack");
        // (g) 阵亡 → 只能 wait
        PBuilder p7;
        p7.tick = 16;
        p7.alive = false;
        a = parseAct(agent.onPerception(p7.build()));
        CHECK(a.ok && a.action == "wait", "T3g dead -> wait");
        // (h) 低血 + 打不死：
        //   启发式（depth=0）：防御能多活一 Tick → defend；
        //   深度搜索（默认 depth=3）：防御后 hp5，下一 Tick 双方位置不变仍同轴，
        //   攻击按移动前坐标结算 → 挡不住 → 死亡不可避免，按 §5.4 改为攻击提取收益。
        {
            rmtc::Config cfgH;
            cfgH.combat_depth = 0;
            rmtc::Agent heur{cfgH};
            PBuilder p8h;
            p8h.tick = 17;
            p8h.hp = 15;
            p8h.robot(-1, 0, 80);
            Act ah = parseAct(heur.onPerception(p8h.build()));
            CHECK(ah.ok && ah.action == "defend", "T3h heuristic save-hp defend");
        }
        PBuilder p8;
        p8.tick = 17;
        p8.hp = 15;
        p8.robot(-1, 0, 80);
        a = parseAct(agent.onPerception(p8.build()));
        // 深度搜索在"防御只能拖一 Tick"的局面上会给出攻击或横移脱轴
        // （两者期望相近，都优于原地无效防御），断言只要求有效非等待动作。
        CHECK(a.ok && a.action != "wait", "T3h deep: purposeful action under threat");
        // (h2) 出生保护期间被威胁也不防御：§7 保护期 DEFEND 无额外收益
        PBuilder p8b;
        p8b.tick = 18;
        p8b.hp = 15;
        p8b.prot = true;
        p8b.robot(-1, 0, 80);
        a = parseAct(agent.onPerception(p8b.build()));
        CHECK(a.ok && a.action != "defend", "T3h2 no defend while protected");
        // (h3) 低血但这一击能打死对方 → 先手攻击而不是防御
        PBuilder p8c;
        p8c.tick = 19;
        p8c.hp = 15;
        p8c.robot(-1, 0, 10);
        a = parseAct(agent.onPerception(p8c.build()));
        CHECK(a.ok && a.action == "attack", "T3h3 kill beats defend");
        // (i) 空图无敌人 → 规划器探索移动
        PBuilder p9;
        p9.tick = 20;
        a = parseAct(agent.onPerception(p9.build()));
        CHECK(a.ok && a.action == "move", "T3i explore move");
        // (j) 最近目标是保护中的敌车 → 挡住后方目标，本次不攻击（§6.2）
        rmtc::Agent agent2{rmtc::Config()};
        PBuilder pj;
        pj.tick = 21;
        pj.robot(1, 0, 60, false, true, "B");  // 保护中，距离 1
        pj.robot(2, 0, 60, false, false, "C");  // 距离 2，被前方挡住
        a = parseAct(agent2.onPerception(pj.build()));
        CHECK(a.ok && a.action != "attack", "T3j protected nearest blocks rear");
        // (k) 500 Tick 赛制：中局（tick=450）仍有剩余时间，不能退化成 WAIT
        rmtc::Agent agent3{rmtc::Config()};
        PBuilder pk;
        pk.tick = 450;
        pk.x = 12;
        pk.y = 12;
        a = parseAct(agent3.onPerception(pk.build()));
        CHECK(a.ok && a.action == "move", "T3k mid-game tick 450 still moves");
    }

    // ---------------- T4: 记忆威胁不用于防御（防"对空气防御"） ----------------
    {
        rmtc::Agent agent{rmtc::Config()};
        PBuilder p1;
        p1.tick = 20;
        p1.robot(2, 0, 60);  // 距离 2 同行：本 Tick 直接攻击
        Act a = parseAct(agent.onPerception(p1.build()));
        CHECK(a.action == "attack", "T4 fresh attack");
        // tick21：敌车已离开视野（不在本帧实体里）。防御只对"本 Tick 真正落下"
        // 的攻击有效，而能打到我的敌人一定在视野内（射程 2 << 视野 4）。
        // 所以这里**不能**因为陈旧记忆而防御，否则会对着空气原地站桩。
        PBuilder p2;
        p2.tick = 21;
        p2.hp = 30;
        p2.last_action = "{\"tick\":20,\"action\":\"attack\",\"moved\":false}";
        a = parseAct(agent.onPerception(p2.build()));
        CHECK(a.ok && a.action != "defend", "T4 stale threat does not defend");
        // 记忆继续老化，同样不防御（威胁由本帧视野重新确认）
        PBuilder p3;
        p3.tick = 23;
        p3.hp = 30;
        a = parseAct(agent.onPerception(p3.build()));
        CHECK(a.ok && a.action != "defend", "T4 threat expires");
    }

    // ---------------- T5: 采集移动 + 到点收集 ----------------
    {
        rmtc::Agent agent{rmtc::Config()};
        PBuilder p1;
        p1.tick = 30;
        p1.x = 5;
        p1.y = 5;
        p1.res(2, 0, 10);  // 资源在 (7,5)
        Act a = parseAct(agent.onPerception(p1.build()));
        CHECK(a.ok && a.action == "move" && a.dir == "right", "T5 step toward resource");
        PBuilder p2;
        p2.tick = 31;
        p2.x = 6;
        p2.y = 5;
        p2.res(1, 0, 10);
        p2.last_action = "{\"tick\":30,\"action\":\"move\",\"moved\":true}";
        a = parseAct(agent.onPerception(p2.build()));
        CHECK(a.ok && a.action == "move" && a.dir == "right", "T5 second step");
        // 到达 (7,5)：资源已被收走（score 增加、实体消失）→ 继续探索
        PBuilder p3;
        p3.tick = 32;
        p3.x = 7;
        p3.y = 5;
        p3.res_value = 10.f;
        p3.last_action = "{\"tick\":31,\"action\":\"move\",\"moved\":true}";
        a = parseAct(agent.onPerception(p3.build()));
        CHECK(a.ok && a.action == "move", "T5 collected, explore on");
    }

    // ---------------- T6: tick/session 对齐 ----------------
    {
        rmtc::Agent agent{rmtc::Config()};
        PBuilder p1;
        p1.tick = 40;
        p1.robot(1, 0, 60);
        const std::string first = agent.onPerception(p1.build());
        CHECK(!first.empty(), "T6 first action");
        // 过期 Tick：丢弃
        PBuilder stale;
        stale.tick = 39;
        CHECK(agent.onPerception(stale.build()).empty(), "T6 stale tick dropped");
        // 同 Tick 重发：回放缓存（不重复推进策略）
        CHECK(agent.onPerception(p1.build()) == first, "T6 same tick replays cache");
        // 换场次：清空敌情/世界/规划
        PBuilder p2;
        p2.tick = 5;
        p2.session = "s2";
        CHECK(!agent.onPerception(p2.build()).empty(), "T6 new session accepted");
        CHECK(agent.sessionId() == "s2" && agent.lastTick() == 5, "T6 session switched");
        CHECK(agent.radar().all().empty(), "T6 radar cleared");
        int unk = 0, fr = 0, bl = 0;
        agent.world().counts(unk, fr, bl);
        CHECK(unk == 24 * 24 - 81, "T6 world memory cleared");  // 只剩本帧 9x9 视野
        // 坏输入：不崩溃、不影响状态
        CHECK(agent.onPerception("not json").empty(), "T6 garbage dropped");
        CHECK(agent.onPerception("{\"tick\":\"x\"}").empty(), "T6 bad tick type dropped");
        CHECK(agent.onPerception("{\"tick\":6}").empty(), "T6 missing session dropped");
        CHECK(agent.lastTick() == 5, "T6 state intact after bad input");
    }

    // ---------------- T7: 敌车软阻挡 → 寻路绕行 ----------------
    {
        rmtc::Agent agent{rmtc::Config()};
        rmtc::Combat::Params cp;  // 关闭战斗层，单测寻路
        cp.enabled = false;
        agent.combat().configure(cp);
        PBuilder p1;
        p1.tick = 50;
        p1.robot(1, 0, 60);   // 敌车占 (11,10)
        p1.res(3, 0, 10);     // 资源在 (13,10)：直线被敌车挡住
        Act a = parseAct(agent.onPerception(p1.build()));
        CHECK(a.ok && a.action == "move" && a.dir != "right", "T7 path avoids enemy cell");
    }

    // ---------------- T8: move 失败回执分类（占位 ≠ 障碍） ----------------
    {
        rmtc::Agent agent{rmtc::Config()};
        rmtc::Combat::Params cp;
        cp.enabled = false;
        agent.combat().configure(cp);
        // 向已知空地 (6,5) 移动失败，且本 Tick 视野内该格无障碍、被敌车占用
        PBuilder p1;
        p1.tick = 60;
        p1.x = 5;
        p1.y = 5;
        p1.res(3, 0, 10);  // 目标 (8,5)，第一步 right -> (6,5)
        Act a = parseAct(agent.onPerception(p1.build()));
        CHECK(a.action == "move" && a.dir == "right", "T8 attempt right");
        PBuilder p2;
        p2.tick = 61;
        p2.x = 5;
        p2.y = 5;
        p2.robot(1, 0, 60);  // 敌车正好站在 (6,5)
        p2.res(3, 0, 10);
        p2.last_action = "{\"tick\":60,\"action\":\"move\",\"moved\":false}";
        a = parseAct(agent.onPerception(p2.build()));
        const int cell = agent.world().idx(6, 5);
        CHECK(agent.world().state()[cell] == rmtc::World::FREE,
              "T8 occupied cell NOT marked blocked");
        CHECK(a.action == "move" && a.dir != "right", "T8 detours around occupant");
        // 对照：越界失败（E_OOB）一定是地形性失败 → 走硬失败路径不崩溃
        rmtc::Agent agent2{rmtc::Config()};
        rmtc::Combat::Params cp2;
        cp2.enabled = false;
        agent2.combat().configure(cp2);
        PBuilder r1;
        r1.tick = 71;
        r1.x = 23;
        r1.y = 10;  // 地图右边缘
        r1.vision = 4;
        r1.res(-3, 0, 10);  // 资源在 (20,10)，远离边界，规划向左走（不越界）
        Act ar = parseAct(agent2.onPerception(r1.build()));
        CHECK(ar.action == "move" && ar.dir == "left", "T8 edge walk left");
        PBuilder r2;
        r2.tick = 72;
        r2.x = 23;
        r2.y = 10;
        r2.res(-3, 0, 10);
        // 伪造 tick71 的移动失败回执（error=E_OOB）：走地形性失败路径，
        // 尝试格被写回障碍 + 进入保守模式，流程不崩溃
        r2.last_action = "{\"tick\":71,\"action\":\"move\",\"moved\":false,\"error\":\"E_OOB\"}";
        ar = parseAct(agent2.onPerception(r2.build()));
        CHECK(ar.ok, "T8 E_OOB receipt processed");
    }

    // ---------------- T9: 地形拓扑（割点/咽喉，确定性单元测试） ----------------
    {
        // 手造地图：10×10，x=5 一整列墙，只在 y=5 留一个单格走廊。
        // 删掉走廊格后左右两侧各 50/40 格 → min=40 ≥ K=8 → 咽喉。
        rmtc::World w(10, 10);
        for (int y = 0; y < 10; ++y)
            for (int x = 0; x < 10; ++x)
                w.markFree(x, y);
        for (int y = 0; y < 10; ++y)
            if (y != 5) w.markBlocked(5, y);
        w.analyzeTopology(0);

        CHECK(w.topologyValid(), "T9 topology computed");
        const int gap = w.idx(5, 5);
        CHECK(w.isChoke(gap), "T9 corridor gap is a choke point");
        CHECK(w.chokeValue(gap) == 40, "T9 choke value = min(50,40) = 40");
        // 开阔区中部的格子不应是咽喉
        CHECK(!w.isChoke(w.idx(2, 2)), "T9 open cell not choke");
        CHECK(!w.isChoke(w.idx(7, 7)), "T9 open cell (right side) not choke");
        // 割点共 3 个：走廊格 + 两侧翼格（删翼格同样把半区与走廊分离，
        // 这是正确的图论结果——"柄"结构的三个关节点）
        CHECK(w.chokeCount() == 3, "T9 gap + both flanks are chokes");
        CHECK(w.isChoke(w.idx(4, 5)) && w.isChoke(w.idx(6, 5)),
              "T9 flank cells are chokes");

        // 未知区参与建图：右侧全部 UNKNOWN 时走廊仍是割点（两侧仍连通）
        rmtc::World w2(10, 10);
        for (int y = 0; y < 10; ++y)
            if (y != 5) w2.markBlocked(5, y);   // 左半保持 UNKNOWN
        for (int x = 6; x < 10; ++x)
            for (int y = 0; y < 10; ++y) w2.markFree(x, y);
        w2.analyzeTopology(0);
        CHECK(w2.isChoke(w2.idx(5, 5)), "T9 choke with unknown half");

        // 寻路软惩罚：从左到右的等长路径必须经过走廊（无替代路），
        // 惩罚只应改变风险值，不改变步数。
        std::vector<float> steps0, stepsP, risk0, riskP;
        std::vector<int> par0, parP;
        w.dijkstra(w.idx(0, 5), 2.0f, true, steps0, risk0, par0, 0.f);
        w.dijkstra(w.idx(0, 5), 2.0f, true, stepsP, riskP, parP, 5.f);
        const int right = w.idx(9, 5);
        CHECK(steps0[right] == stepsP[right] && steps0[right] == 9.f,
              "T9 choke penalty keeps true step count");
        CHECK(riskP[right] > risk0[right], "T9 choke penalty raises risk");

        // 咽喉两侧多格开口 → 不再是咽喉（两侧不再被单点分离）
        rmtc::World w3(10, 10);
        for (int y = 0; y < 10; ++y)
            for (int x = 0; x < 10; ++x) w3.markFree(x, y);
        for (int y = 0; y < 10; ++y)
            if (y != 4 && y != 5) w3.markBlocked(5, y);  // 两格开口
        w3.analyzeTopology(0);
        CHECK(!w3.isChoke(w3.idx(5, 4)) && !w3.isChoke(w3.idx(5, 5)),
              "T9 two-cell opening is not a choke");
    }

    // ---------------- T10: 接战门三态 + 黄灯软代价 ----------------
    {
        rmtc::Config cfg;
        cfg.engage_gate = true;
        cfg.combat_depth = 0;
        rmtc::Agent a(cfg);

        PBuilder p;
        p.x = 10;
        p.y = 10;

        // 斜向 4 格：不逼近、不同轴、未开火 → SAFE
        p.tick = 1;
        p.ents.clear();
        p.robot(3, 1, 100);
        a.onPerception(p.build());
        CHECK(a.threatLevel() == 0, "T10a diagonal far -> SAFE");

        // 同一敌人从 4 格到 3 格：approaching → ALERT
        p.tick = 2;
        p.ents.clear();
        p.robot(2, 1, 100);
        a.onPerception(p.build());
        CHECK(a.threatLevel() == 1, "T10b approaching -> ALERT");

        // 贴到 1 格：ENGAGED
        p.tick = 3;
        p.ents.clear();
        p.robot(1, 0, 100);
        a.onPerception(p.build());
        CHECK(a.threatLevel() == 2, "T10c adjacent -> ENGAGED");

        // 敌人离开，但 5 Tick 防抖锁内保持 ENGAGED
        p.tick = 4;
        p.ents.clear();
        a.onPerception(p.build());
        CHECK(a.threatLevel() == 2, "T10d lock keeps ENGAGED");

        // 锁过期且无新威胁 → SAFE
        p.tick = 9;
        a.onPerception(p.build());
        CHECK(a.threatLevel() == 0, "T10e lock expired -> SAFE");
    }

    {
        // ALERT 黄灯：同轴 3 格敌人 → 射线格加软代价，但不是硬阻挡
        rmtc::Config cfg;
        cfg.engage_gate = true;
        cfg.combat_depth = 0;
        rmtc::Agent a(cfg);
        PBuilder p;
        p.tick = 1;
        p.x = 10;
        p.y = 10;
        p.robot(3, 0, 100);
        a.onPerception(p.build());
        CHECK(a.threatLevel() == 1, "T10f same-axis 3 -> ALERT");
        const int idx = a.world().idx(11, 10);
        CHECK(a.world().softCost(idx) > 0.f, "T10g alert ray has soft cost");
        CHECK(!a.world().softBlocked(idx) &&
                  a.world().state()[idx] != rmtc::World::BLOCKED,
              "T10h alert ray is soft, not a hard block");
    }

    // ---------------- T11: 任务点被占的四阶争夺 ----------------
    {
        rmtc::Config cfg;
        cfg.reach_contest = true;
        cfg.engage_gate = false;
        cfg.combat_depth = 0;
        rmtc::Agent a(cfg);
        PBuilder p;
        p.tick = 400;  // R=100，进入 reach 窗口
        p.x = 10;
        p.y = 10;
        p.missions_json =
            "[{\"id\":\"reach_target\",\"type\":\"reach\",\"target\":1,"
            "\"position\":[10,12]}]";
        p.robot(0, 2, 100);  // 敌车站在任务点上
        Act a1 = parseAct(a.onPerception(p.build()));
        CHECK(a1.ok && a1.action == "attack" && a1.dir == "down",
              "T11a contest attacks blocker on target");
    }

    // ---------------- T12: 高价值资源时间竞争门（race_gate） ----------------
    // 视野半径 4 → 实体必须放在 ±4 内；下列坐标都在 9×9 图里。
    {
        // 高价值资源在 (12,10)，本车 (10,10)：t_me = 2，密度 24/2 必然优先生效；
        // 敌车本 Tick 目击在 (13,10)：t_enemy = 1 → 1×1.35 < 2，判负剔除。
        // race 关：走向资源；race 开：候选被剔除（且 raceDrops=1），不再向右。
        PBuilder p;
        p.tick = 30;
        p.x = 10;
        p.y = 10;
        p.res(2, 0, 20);     // (12,10) 高价值
        p.robot(3, 0, 100);  // (13,10) 敌车，距资源 1 步，距本车 3 步（不触发战斗）

        rmtc::Config cfgOff;
        cfgOff.race_gate = false;
        cfgOff.combat_depth = 0;
        rmtc::Agent off(cfgOff);
        Act aOff = parseAct(off.onPerception(p.build()));
        CHECK(aOff.ok && aOff.action == "move" && aOff.dir == "right" &&
                  off.planner().raceDrops() == 0,
              "T12a race off: heads to high-value resource");

        rmtc::Config cfgOn;
        cfgOn.race_gate = true;
        cfgOn.combat_depth = 0;
        rmtc::Agent on(cfgOn);
        Act aOn = parseAct(on.onPerception(p.build()));
        CHECK(aOn.ok && !(aOn.action == "move" && aOn.dir == "right") &&
                  on.planner().raceDrops() == 1,
              "T12b race on: lost race dropped, not heading right");

        // 敌车更远（(7,10)，距资源 5 步）：t_enemy=5，5×1.35 > 2，保留候选。
        PBuilder p2;
        p2.tick = 30;
        p2.x = 10;
        p2.y = 10;
        p2.res(2, 0, 20);      // (12,10)
        p2.robot(-3, 0, 100);  // (7,10)，距本车 3 步
        rmtc::Agent far(cfgOn);
        Act aFar = parseAct(far.onPerception(p2.build()));
        CHECK(aFar.ok && aFar.action == "move" && aFar.dir == "right" &&
                  far.planner().raceDrops() == 0,
              "T12c enemy clearly farther: candidate kept");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
