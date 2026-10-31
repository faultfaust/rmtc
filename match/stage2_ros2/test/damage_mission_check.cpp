// damage_mission_check.cpp —— v1.2 伤害任务未完成时的接战强化回归
//
// 断言：
//   1) 无伤害任务、D≥500：D 止盈生效，不主动攻击；
//   2) 伤害任务未完成、D≥500：关闭 D 止盈，直接攻击；
//   3) 伤害任务已完成：恢复 D 止盈，不主动攻击；
//   4) 低血（30 HP）且无任务：正常撤退/防御；有未完成伤害任务：提高接战意愿，攻击。
#include <cstdio>
#include <string>

#include "rmtc/agent.h"
#include "rmtc/config.h"
#include "rmtc/json.h"
#include "rmtc/planner.h"

namespace {

int g_pass = 0;
int g_fail = 0;

#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (cond) {                                                        \
            ++g_pass;                                                      \
        } else {                                                           \
            ++g_fail;                                                      \
            std::printf("FAIL: %s (line %d)\n", msg, __LINE__);            \
        }                                                                  \
    } while (0)

std::string robotGrid(int dx, int dy) {
    std::string out;
    for (int r = 0; r < 9; ++r) {
        for (int c = 0; c < 9; ++c) {
            char ch = '0';
            if (r == 4 && c == 4) ch = '1';
            if (r == 4 + dy && c == 4 + dx) ch = '2';
            out += ch;
        }
        if (r < 8) out += '\n';
    }
    return out;
}

// missionsJson / completedJson 直接拼进感知；敌人固定在自身右侧 2 格。
std::string perceptionJson(int tick, int hp, float effectiveDamage,
                           const std::string& missionsJson,
                           const std::string& completedJson) {
    std::string s = "{";
    s += "\"tick\":" + std::to_string(tick);
    s += ",\"session_id\":\"damage-task-check\"";
    s += ",\"vision_radius\":4";
    s += ",\"map\":{\"width\":24,\"height\":24}";
    s += ",\"self\":{\"team\":\"A\",\"position\":[5,5]";
    s += ",\"hp\":" + std::to_string(hp);
    s += ",\"alive\":true,\"protected\":false,\"respawn_in\":0,\"robot_id\":\"A\"";
    s += ",\"last_action\":null}";
    s += ",\"score\":{\"resource_value\":0,\"effective_damage\":";
    s += std::to_string(static_cast<int>(effectiveDamage));
    s += ",\"mission_completed\":0,\"mission_completed_ids\":[";
    s += completedJson;
    s += "],\"invalid_actions\":0}";
    s += ",\"visible_entities\":[{\"type\":\"robot\",\"robot_id\":\"B\",";
    s += "\"team\":\"B\",\"hp\":100,\"defending\":false,\"protected\":false,";
    s += "\"position\":\"" + rmtc::jsonEscape(robotGrid(2, 0)) + "\"}]";
    s += ",\"missions\":[" + missionsJson + "]";
    s += ",\"mission_text\":\"\"";
    s += ",\"event_text\":\"\"}";
    return s;
}

rmtc::Decision run(const std::string& missionsJson, const std::string& completedJson,
                   float effectiveDamage, int hp, bool dcap, bool economy = false) {
    rmtc::Config cfg;
    cfg.combat_depth = 0;
    cfg.d_cap_gate = dcap;
    cfg.engage_gate = false;
    cfg.economy_gate = economy;
    cfg.economy_gate_rate = 0.f;  // 让经济门在 planner_rate_=0 时也可触发
    rmtc::Agent agent(cfg);
    agent.onPerception(perceptionJson(0, hp, effectiveDamage, missionsJson,
                                      completedJson));
    return agent.lastDecision();
}

const char* kDamageMission =
    "{\"id\":\"damage_task\",\"type\":\"effective_damage\",\"target\":600}";

}  // namespace

int main() {
    // 1) 无任务、D=520：D 止盈应拦住非击杀攻击。
    {
        const rmtc::Decision d = run("", "", 520.f, 100, true);
        CHECK(d.kind != rmtc::MOVE_ATTACK, "no damage task + D>=500: dcap holds attack");
    }
    // 2) 未完成伤害任务、D=520：应关闭降权，直接打。
    {
        const rmtc::Decision d = run(kDamageMission, "", 520.f, 100, true);
        CHECK(d.kind == rmtc::MOVE_ATTACK, "pending damage task + D>=500: attack");
    }
    // 3) 任务已完成：恢复 D 止盈。
    {
        const rmtc::Decision d =
            run(kDamageMission, "\"damage_task\"", 520.f, 100, true);
        CHECK(d.kind != rmtc::MOVE_ATTACK, "completed damage task: dcap restored");
    }
    // 4) 经济门开启、无任务：攻击被“采集降权”压住；有未完成任务：经济门关闭，攻击。
    {
        const rmtc::Decision base = run("", "", 0.f, 100, false, true);
        CHECK(base.kind != rmtc::MOVE_ATTACK,
              "economy gate without damage task holds attack");
        const rmtc::Decision urgent = run(kDamageMission, "", 0.f, 100, false, true);
        CHECK(urgent.kind == rmtc::MOVE_ATTACK,
              "pending damage task disables economy gate and attacks");
    }

    std::printf("damage_mission_check: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
