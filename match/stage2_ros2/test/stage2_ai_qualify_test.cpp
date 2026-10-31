// stage2_ai_qualify_test.cpp —— AI 资格（M=0）状态机的离线回归
//
// 背景：规则 §10.2 要求“本场本席位至少一次真实 AI 成功调用”，否则任务 15 分归零。
// 旧实现有三个真实缺陷：
//   1) 握手请求体固定，LlmClient 按文本去重 → 6 次“重试”实际只发 1 次；
//   2) 握手窗口用绝对 tick 5–20 → 换场/重连在 tick 100+ 时完全跳过；
//   3) 只把回复里含 "OK" 的结果算资格 → 2xx 的“收到”/事件成功不算。
// 本测试用 FakeTransport 钉死这三条，以及“任一 2xx 事件也能落资格”。
//
// 编译（在 cpp 目录）：
//   g++ -std=c++11 -O2 -I . -o ../build/qualify_test \
//       stage2_ai_qualify_test.cpp world.cpp planner.cpp rmtc/protocol.cpp \
//       rmtc/json.cpp rmtc/radar.cpp rmtc/event.cpp rmtc/event_prompt.cpp \
//       rmtc/llm.cpp combat.cpp agent.cpp
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "rmtc/agent.h"
#include "rmtc/json.h"
#include "rmtc/llm.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg)                                   \
    do {                                                   \
        if (cond) { ++g_pass; }                            \
        else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
    } while (0)

namespace {

class FakeTransport : public rmtc::LlmTransport {
public:
    // 两个 worker 并发 post：计数必须是原子的（否则 T7 偶发丢计数）。
    std::atomic<int> handshakeCalls{0};
    std::atomic<int> eventCalls{0};
    int failHandshakeFirst = 0;   // 前 N 次握手返回连接失败
    bool failHandshakeForever = false;
    bool failEvent = false;
    int eventFailFirst = 0;       // 前 N 次事件返回 429 rate_limit（可重试）
    int handshakeStatus = 200;    // 可模拟 401/429/500
    std::string handshakeReply = "OK";
    bool emptyContent = false;    // 2xx 但 choices[0].message.content 缺失
    int latency_ms = 1;
    std::atomic<int> lastTimeoutMs{0};
    std::atomic<int> handshakeTimeoutMs{0};

    bool post(const std::string&, const std::string& body, const std::string&,
              int timeout_ms, int& status, std::string& response, std::string& err) {
        const bool isHandshake = body.find("请回复 OK") != std::string::npos;
        lastTimeoutMs = timeout_ms;
        if (isHandshake) handshakeTimeoutMs = timeout_ms;
        std::this_thread::sleep_for(std::chrono::milliseconds(latency_ms));
        if (isHandshake) {
            ++handshakeCalls;
            if (failHandshakeForever || handshakeCalls <= failHandshakeFirst) {
                status = 0;
                err = "connection refused";
                return false;
            }
            if (handshakeStatus < 200 || handshakeStatus >= 300) {
                status = handshakeStatus;
                err = "http error";
                return true;
            }
            status = 200;
            response = emptyContent ? "{\"choices\":[{\"message\":{}}]}"
                                    : wrap(handshakeReply);
            return true;
        }
        ++eventCalls;
        if (eventCalls <= eventFailFirst) {
            status = 429;
            response = "{\"error\":{\"code\":\"rate_limit\"}}";
            return true;
        }
        if (failEvent) {
            status = 500;
            err = "server error";
            return true;
        }
        status = 200;
        // 一个合法事件（activate_tick 取未来值，避免“过去 Tick”拒绝）。
        response = wrap(
            "{\"kind\":\"explosion\",\"x0\":4,\"y0\":4,\"x1\":9,\"y1\":9,"
            "\"activate_tick\":400,\"end_tick\":-1,\"confidence\":0.9}");
        return true;
    }

private:
    static std::string wrap(const std::string& content) {
        return "{\"choices\":[{\"message\":{\"content\":\"" +
               rmtc::jsonEscape(content) + "\"}}]}";
    }
};

std::string makePerc(int tick, const std::string& session,
                     const std::string& eventText) {
    std::string s = "{\"tick\":" + std::to_string(tick);
    s += ",\"session_id\":\"" + session + "\"";
    s += ",\"vision_radius\":4";
    s += ",\"self\":{\"team\":\"A\",\"position\":[10,10],\"hp\":100,"
         "\"alive\":true,\"defending\":false,\"protected\":false,"
         "\"respawn_in\":0,\"robot_id\":\"A\",\"last_action\":null}";
    s += ",\"visible_entities\":[]";
    s += ",\"score\":{\"resource_value\":0,\"effective_damage\":0,"
         "\"mission_completed\":0,\"mission_completed_ids\":[],"
         "\"invalid_actions\":0}";
    s += ",\"mission_text\":\"\",\"missions\":[]";
    s += ",\"map\":{\"width\":24,\"height\":24}";
    s += ",\"event_text\":\"" + rmtc::jsonEscape(eventText) + "\"}";
    return s;
}

struct Harness {
    FakeTransport tr;
    rmtc::LlmClient client;
    rmtc::Agent agent;

    Harness() : agent(rmtc::Config()) {
        rmtc::LlmConfig cfg;
        cfg.timeout_ms = 100;
        cfg.max_queue = 8;
        cfg.max_age_ticks = 40;
        cfg.quota = 50;
        client.configure(cfg);
        client.setEndpoint("http://127.0.0.1:1/v1/chat/completions", "test-token");
        client.setTransport(&tr);
        client.start();
        agent.attachLlm(&client);
    }

    void tick(int t, const std::string& session, const std::string& eventText = "") {
        agent.onPerception(makePerc(t, session, eventText));
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
};

}  // namespace

int main() {
    // T1：前两次握手失败，第三次成功 —— 证明重试真的发出去了（不是被去重吞掉）
    {
        Harness h;
        h.tr.failHandshakeFirst = 2;
        for (int t = 0; t <= 60; ++t) h.tick(t, "s1");
        CHECK(h.agent.quotaSecured(), "T1 qualification secured after retries");
        CHECK(h.tr.handshakeCalls >= 3, "T1 at least 3 handshake calls hit transport");
        CHECK(h.agent.handshakeAttempts() >= 3, "T1 attempt counter >= 3");
    }

    // T2：2xx 但回复不是 "OK"（例如“收到”）也要落资格；不得被当成事件解析
    {
        Harness h;
        h.tr.handshakeReply = "收到";
        for (int t = 0; t <= 10; ++t) h.tick(t, "s1");
        CHECK(h.agent.quotaSecured(), "T2 non-OK 2xx handshake secures");
        CHECK(h.tr.handshakeCalls == 1, "T2 only one handshake needed");
        CHECK(h.agent.activeEventCount() == 0, "T2 handshake not parsed as event");
    }

    // T2b：2xx 但 content 缺失 —— 协议按“成功调用记录”算资格，不能因为
    // 本地解析失败就判 M=0。旧实现只在 res.ok 时落资格，会误判。
    {
        Harness h;
        h.tr.emptyContent = true;
        for (int t = 0; t <= 10; ++t) h.tick(t, "s1");
        CHECK(h.agent.quotaSecured(),
              "T2b 2xx without content still secures qualification");
        CHECK(h.tr.handshakeCalls == 1,
              "T2b qualification stops further handshakes");
    }

    // T3：握手一直失败，但事件请求 2xx —— 任何成功调用都能落资格
    {
        Harness h;
        h.tr.failHandshakeForever = true;
        for (int t = 0; t <= 40; ++t)
            h.tick(t, "s1", t >= 20 ? "爆炸公告" : "");
        CHECK(h.agent.quotaSecured(), "T3 successful event call secures qualification");
        CHECK(h.tr.eventCalls >= 1, "T3 event request reached transport");
        CHECK(h.agent.activeEventCount() >= 1, "T3 event parsed into pool");
    }

    // T3b：事件 429 rate_limit 可重试 —— Agent 清公告去重、退避 5 Tick 后
    // 重新提交，且客户端必须已把 hash 从 seen_ 移除，否则会被吞。
    {
        Harness h;
        h.tr.eventFailFirst = 1;
        for (int t = 0; t <= 6; ++t) h.tick(t, "s1");
        for (int t = 7; t <= 30; ++t) h.tick(t, "s1", "爆炸公告");
        CHECK(h.tr.eventCalls >= 2,
              "T3b rate_limit event retried at transport");
        CHECK(h.agent.activeEventCount() >= 1,
              "T3b retry eventually pools the event");
    }

    // T4：tick 100 才换到新 session —— 握手窗口必须相对 session 起点，不能跳过
    {
        Harness h;
        for (int t = 0; t <= 10; ++t) h.tick(t, "s1");
        CHECK(h.agent.quotaSecured(), "T4 first session secured");
        const int before = h.tr.handshakeCalls;
        for (int t = 100; t <= 140; ++t) h.tick(t, "s2");
        CHECK(h.agent.quotaSecured(), "T4 new session re-secured");
        CHECK(h.tr.handshakeCalls > before, "T4 handshake fired after late session switch");
    }

    // T5：一直失败也不能崩；重试次数应覆盖 early+late 窗口
    {
        Harness h;
        h.tr.failHandshakeForever = true;
        for (int t = 0; t <= 140; ++t) h.tick(t, "s1");
        CHECK(!h.agent.quotaSecured(), "T5 stays unsecured when all calls fail");
        CHECK(h.tr.handshakeCalls >= 8, "T5 retries continue past tick 20");
        CHECK(h.agent.handshakeAttempts() >= 8, "T5 attempt counter grows");
        CHECK(h.agent.lastLlmStatus() == 0, "T5 last status reflects transport failure");
        CHECK(!h.agent.llmDisabled(), "T5 transport failure does not permanently disable");
    }

    // T6：握手结果晚到也不能被“结果过期”丢掉；事件结果过期仍要丢。
    //     同时验证握手使用 4.5s 独立超时（事件仍是 1.8s 级别）。
    {
        FakeTransport tr;
        tr.latency_ms = 50;
        rmtc::LlmClient c;
        rmtc::LlmConfig cfg;
        cfg.timeout_ms = 100;
        cfg.max_age_ticks = 5;
        cfg.quota = 10;
        c.configure(cfg);
        c.setEndpoint("http://127.0.0.1:1/v1/chat/completions", "t");
        c.setTransport(&tr);
        c.start();

        bool dup = false;
        const long hs = c.submit("s", 0, "请回复 OK。", &dup,
                                 rmtc::LLM_HANDSHAKE, 4500);
        CHECK(hs > 0 && !dup, "T6 handshake accepted");
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        rmtc::LlmResult r;
        const bool got = c.poll(1000, r);  // now_tick 远大于 max_age_ticks
        CHECK(got && r.ok && r.kind == rmtc::LLM_HANDSHAKE,
              "T6 late handshake result still delivered");
        CHECK(tr.handshakeTimeoutMs == 4500, "T6 handshake uses 4.5s timeout");

        const long ev = c.submit("s", 0, "event text", &dup,
                                 rmtc::LLM_EVENT, 100);
        CHECK(ev > 0, "T6 event accepted");
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        rmtc::LlmResult r2;
        const bool got2 = c.poll(1000, r2);
        CHECK(!got2 && c.droppedStale() >= 1, "T6 stale event result dropped");
        c.stop();
    }

    // T7：同文本的两次握手都必须真正发出去（绕过事件去重）。
    {
        FakeTransport tr;
        tr.latency_ms = 1;
        rmtc::LlmClient c;
        rmtc::LlmConfig cfg;
        cfg.timeout_ms = 100;
        cfg.quota = 10;
        c.configure(cfg);
        c.setTransport(&tr);
        c.start();
        bool dup = false;
        const long a = c.submit("s", 1, "请回复 OK。", &dup, rmtc::LLM_HANDSHAKE, 100);
        const long b = c.submit("s", 2, "请回复 OK。", &dup, rmtc::LLM_HANDSHAKE, 100);
        CHECK(a > 0 && b > 0 && !dup, "T7 handshake dedupe bypass: both accepted");
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        CHECK(tr.handshakeCalls == 2, "T7 two handshakes hit transport");
        c.stop();
    }

    // T8：换场必须恢复配额。同一 ROS 进程连打多场时，上一场用满 quota
    //     不能把下一场的 AI 调用全部拒绝（否则第二场直接 M=0）。
    {
        FakeTransport tr;
        tr.latency_ms = 1;
        rmtc::LlmClient c;
        rmtc::LlmConfig cfg;
        cfg.timeout_ms = 100;
        cfg.max_age_ticks = 40;
        cfg.quota = 1;
        c.configure(cfg);
        c.setTransport(&tr);
        c.start();

        bool dup = false;
        CHECK(c.submit("m1", 1, "event text 1", &dup, rmtc::LLM_EVENT, 100) > 0,
              "T8 first match accepted");
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        rmtc::LlmResult r;
        bool got = false;
        for (int i = 0; i < 20 && !got; ++i) {
            got = c.poll(2, r);
            if (!got) std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        CHECK(got && c.quotaLeft() == 0, "T8 quota consumed in first match");
        c.resetSession();
        CHECK(c.quotaLeft() == 1, "T8 quota restored on new session");
        CHECK(c.submit("m2", 1, "event text 2", &dup, rmtc::LLM_EVENT, 100) > 0,
              "T8 new match accepted after reset");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        c.stop();
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
