// reach_fail_check.cpp —— 问题 (1)(2) 的定向回归：
//   1) LLM 完全失败（传输层永远连不上）+ 每 Tick 都有事件公告 时，
//      reach 仍然必须被导航并完成（reach 不能依赖事件解析成功）；
//   2) 结构化 missions 里 reach 没有 position（只有 mission_text）时，
//      仍能从任务公告恢复坐标；
//   3) reach 类型写成 reach_target 也能识别；
//   4) /referee/event_text 话题注入的公告能绕过感知缓存进入 LLM 事件池；
//   5) 事件请求默认总超时必须是 3000ms（旧值 1800ms 会稳定超时）。
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "rmtc/agent.h"
#include "rmtc/json.h"
#include "rmtc/llm.h"

namespace {

int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (cond) { ++g_pass; }                                            \
        else { ++g_fail; std::printf("FAIL: %s\n", msg); }                 \
    } while (0)

// 本测试把 total_ticks 设小，让 reach 从第一帧就进入“再不走就来不及”窗口，
// 这样不用先跑 300 Tick 采集就能验证导航是否依赖 LLM。
int g_total_ticks = 120;

std::string perc(int tick, int x, int y, const std::string& mission_text,
                 const std::string& missions, const std::string& event_text) {
    std::string s = "{\"tick\":" + std::to_string(tick);
    s += ",\"session_id\":\"reach-check\"";
    s += ",\"vision_radius\":4";
    s += ",\"map\":{\"width\":24,\"height\":24}";
    s += ",\"self\":{\"team\":\"A\",\"position\":[" + std::to_string(x) + "," +
         std::to_string(y) + "],\"hp\":100,\"alive\":true,\"protected\":false,";
    s += "\"respawn_in\":0,\"robot_id\":\"A\",\"last_action\":null}";
    s += ",\"visible_entities\":[]";
    s += ",\"score\":{\"resource_value\":0,\"effective_damage\":0,";
    s += "\"mission_completed\":0,\"mission_completed_ids\":[],\"invalid_actions\":0}";
    s += ",\"mission_text\":\"" + rmtc::jsonEscape(mission_text) + "\"";
    s += ",\"missions\":[" + missions + "]";
    s += ",\"event_text\":\"" + rmtc::jsonEscape(event_text) + "\"}";
    return s;
}

void applyAction(const std::string& action, int& x, int& y) {
    rmtc::JsonValue j;
    std::string err;
    if (!rmtc::jsonParse(action, j, err) || !j.isObj()) return;
    const rmtc::JsonValue* a = j.get("action");
    const rmtc::JsonValue* d = j.get("direction");
    if (!a || !a->isStr() || a->str != "move" || !d || !d->isStr()) return;
    if (d->str == "up") --y;
    else if (d->str == "down") ++y;
    else if (d->str == "left") --x;
    else if (d->str == "right") ++x;
}

// 永远连不上的传输层：模拟网关不可达/超时（status=0，可重试）。
class FailTransport : public rmtc::LlmTransport {
public:
    std::atomic<int> calls{0};
    bool post(const std::string&, const std::string&, const std::string&, int,
              int& status, std::string&, std::string& err) override {
        ++calls;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        status = 0;
        err = "connection failed";
        return false;
    }
};

struct RunResult {
    int final_x = -1, final_y = -1;
    int first_reason = -1;
    bool first_is_reach_move = false;
    int events_pooled = 0;
    int last_status = -1;
};

// 从 (1,1) 走到 (22,22)：reach 坐标只给在 mission_text / missions 里。
RunResult runReach(const std::string& mission_text, const std::string& missions,
                   bool feed_events, bool inject_referee_event) {
    rmtc::Config cfg;
    cfg.total_ticks = g_total_ticks;
    rmtc::Agent agent(cfg);
    FailTransport tp;
    rmtc::LlmClient llm;
    rmtc::LlmConfig lc;
    lc.timeout_ms = 300;
    lc.max_queue = 8;
    llm.configure(lc);
    llm.setEndpoint("http://127.0.0.1:1/v1/chat/completions", "t");
    llm.setTransport(&tp);
    llm.start();
    agent.attachLlm(&llm);
    if (inject_referee_event)
        agent.onRefereeEventText("Tick 3: 在 [3,3] 生成回血点，持续 60 Tick。");

    RunResult r;
    int x = 1, y = 1;
    std::string event_text;
    for (int t = 0; t < g_total_ticks; ++t) {
        if (feed_events) {
            event_text += "Tick " + std::to_string(t) +
                          ": 随机事件预告：20 Tick 后 [5,5]-[8,8] 发生爆炸。\n";
        }
        const std::string action =
            agent.onPerception(perc(t, x, y, mission_text, missions, event_text));
        const rmtc::Decision& d = agent.lastDecision();
        if (t == 0) {
            r.first_reason = static_cast<int>(d.reason);
            r.first_is_reach_move =
                d.kind == rmtc::MOVE_MOVE && d.reason == rmtc::R_MOVE_RESOURCE;
        }
        if (!action.empty()) applyAction(action, x, y);
        if (t % 20 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    r.final_x = x;
    r.final_y = y;
    r.events_pooled = agent.activeEventCount();
    r.last_status = agent.lastLlmStatus();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    llm.stop();
    return r;
}

// 正常网关的假传输：固定返回一条合法事件 JSON（验证 referee 队列）。
class OkTransport : public rmtc::LlmTransport {
public:
    bool post(const std::string&, const std::string&, const std::string&, int,
              int& status, std::string& resp, std::string&) override {
        status = 200;
        resp = "{\"choices\":[{\"message\":{\"content\":"
               "\"{\\\"kind\\\":\\\"heal\\\",\\\"x0\\\":3,\\\"y0\\\":3,"
               "\\\"x1\\\":3,\\\"y1\\\":3,\\\"activate_tick\\\":5,"
               "\\\"end_tick\\\":60,\\\"confidence\\\":0.9}\"}}]}";
        return true;
    }
};

}  // namespace

int main() {
    // ---- 0) 默认超时 ----
    CHECK(rmtc::LlmConfig().timeout_ms == 3000,
          "Z1 default event timeout is 3000ms (was 1800ms)");

    // ---- 1) LLM 完全失败 + 公告持续到达：reach 仍要导航并完成 ----
    {
        const RunResult r = runReach("比赛结束前到达指定任务坐标 [22,22]。", "", true, false);
        CHECK(r.first_is_reach_move,
              "A1 first decision navigates to reach even though LLM fails");
        CHECK(r.final_x == 22 && r.final_y == 22,
              "A2 reach completed by mission_text fallback (no structured missions)");
        CHECK(r.events_pooled == 0, "A3 (control) LLM never produced an event");
        CHECK(r.last_status == 0, "A4 (control) last LLM status is transport failure");
    }
    // ---- 2) 结构化 reach 无 position：仍从任务公告恢复坐标 ----
    {
        const std::string missions =
            "{\"id\":\"reach_target\",\"type\":\"reach\",\"target\":1}";
        const RunResult r =
            runReach("获得至少两个普通资源；到达 [22,22]。", missions, false, false);
        CHECK(r.first_is_reach_move, "B1 reach recovered from mission_text before combat");
        CHECK(r.final_x == 22 && r.final_y == 22,
              "B2 reach completed when missions[] lacks position");
    }
    // ---- 3) reach 类型写成 reach_target 也能识别 ----
    {
        const std::string missions =
            "{\"id\":\"r1\",\"type\":\"reach_target\",\"target\":1,\"position\":[22,22]}";
        const RunResult r = runReach("", missions, false, false);
        CHECK(r.final_x == 22 && r.final_y == 22,
              "C1 tolerant reach type (reach_target) navigates");
    }
    // ---- 4) /referee/event_text 绕过感知缓存进入事件池 ----
    {
        rmtc::Config cfg;
        rmtc::Agent agent(cfg);
        OkTransport tp;
        rmtc::LlmClient llm;
        rmtc::LlmConfig lc;
        lc.timeout_ms = 500;
        llm.configure(lc);
        llm.setEndpoint("http://127.0.0.1:9/v1/chat/completions", "t");
        llm.setTransport(&tp);
        llm.start();
        agent.attachLlm(&llm);
        // 公告在第一帧感知之前到达（裁判开场即发），且感知 event_text 始终为空。
        agent.onRefereeEventText("Tick 3: 在 [3,3] 生成回血点，持续 60 Tick。");
        bool pooled = false;
        for (int t = 0; t < 40 && !pooled; ++t) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            agent.onPerception(perc(t, 10, 10, "", "", ""));
            if (agent.activeEventCount() > 0) pooled = true;
        }
        CHECK(pooled, "D1 referee event_text topic feeds the LLM event pool");
        CHECK(agent.quotaSecured(), "D2 2xx from event topic counts as AI qualification");
        llm.stop();
    }
    std::printf("reach_event_check: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
