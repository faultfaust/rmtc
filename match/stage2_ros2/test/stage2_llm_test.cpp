// stage2_llm_test.cpp —— 事件量化 + 异步 LLM 客户端的离线测试
//
// 不需要网络：用 FakeTransport 模拟网关的延迟、429、连接失败。
// 验证三件事：
//   1) submit/poll 都不阻塞（主线程每 Tick 调用是安全的）；
//   2) 去重 / 配额 / 时效 / 换场隔离都按设计工作；
//   3) 事件评分器把 8 类事件折算成分值，并能给出"避开/前往"建议。
//
// 编译（在 cpp 目录）：
//   g++ -std=c++11 -O2 -I . stage2_llm_test.cpp rmtc/event.cpp rmtc/llm.cpp -o ../build/llm_test
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "rmtc/event.h"
#include "rmtc/llm.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg)                                   \
    do {                                                   \
        if (cond) { ++g_pass; }                            \
        else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
    } while (0)

// 假传输层：固定延迟 + 固定状态码，可模拟超时/限流。
class FakeTransport : public rmtc::LlmTransport {
public:
    int latency_ms = 5;
    int status = 200;
    bool connect = true;
    std::atomic<int> calls{0};
    std::string failBody;  // 非 2xx 的响应体（429 错误码放在这里）

    bool post(const std::string&, const std::string& body, const std::string&,
              int timeout_ms, int& st, std::string& response, std::string& err) {
        ++calls;
        if (!connect) {
            err = "connection refused";
            st = 0;
            return false;
        }
        int sleep = latency_ms;
        if (timeout_ms > 0 && sleep > timeout_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
            err = "timeout";
            st = 0;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
        st = status;
        if (status >= 200 && status < 300) {
            response = "{\"choices\":[{\"message\":{\"content\":\"ok:" + body + "\"}}]}";
        } else {
            response = failBody;  // 上层用 body 里的 code 区分 quota/rate_limit
            err = "http error";
        }
        return true;
    }
};

int main() {
    // ---------------- 1) 非阻塞 + 去重 + 结果 ----------------
    {
        FakeTransport tr;
        tr.latency_ms = 60;
        rmtc::LlmClient c;
        rmtc::LlmConfig cfg;
        cfg.timeout_ms = 500;
        cfg.max_age_ticks = 40;
        cfg.quota = 10;
        c.configure(cfg);
        c.setEndpoint("http://127.0.0.1:8080/v1/chat/completions", "practice-A");
        c.setTransport(&tr);
        c.start();

        const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        bool dup = false;
        const long seq1 = c.submit("s1", 100, "event text A", &dup);
        const long seq2 = c.submit("s1", 101, "event text A", &dup);
        const long submit_us = static_cast<long>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0)
                .count());
        CHECK(seq1 > 0, "L1 submit accepted");
        CHECK(seq2 == 0 && dup, "L1 duplicate text deduped");
        CHECK(submit_us < 5000, "L1 submit is non-blocking (<5ms)");

        // 主线程"每 Tick 轮询"：立刻 poll 应该是空的
        rmtc::LlmResult r;
        CHECK(!c.poll(100, r), "L1 poll empty while inflight");
        // 等结果
        bool got = false;
        for (int i = 0; i < 100 && !got; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            got = c.poll(100, r);
        }
        CHECK(got && r.ok && r.seq == seq1 && r.session == "s1" && r.tick == 100,
              "L1 result matched");
        CHECK(r.content.find("ok:") == 0, "L1 content passed through");
        CHECK(c.quotaLeft() == 9 && c.deduped() == 1, "L1 quota/dedup counters");
        c.stop();
    }

    // ---------------- 2) 配额耗尽 ----------------
    {
        FakeTransport tr;
        tr.latency_ms = 1;
        rmtc::LlmClient c;
        rmtc::LlmConfig cfg;
        cfg.quota = 1;
        cfg.timeout_ms = 200;
        c.configure(cfg);
        c.setTransport(&tr);
        c.start();
        CHECK(c.submit("s", 1, "x1") > 0, "L2 first accepted");
        rmtc::LlmResult r;
        for (int i = 0; i < 100; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            if (c.poll(2, r)) break;
        }
        CHECK(r.ok && c.quotaLeft() == 0, "L2 quota consumed");
        CHECK(c.submit("s", 3, "x2") == 0, "L2 quota exhausted rejects");
        c.stop();
    }

    // ---------------- 3) 429 不消耗配额 ----------------
    {
        FakeTransport tr;
        tr.status = 429;
        tr.latency_ms = 1;
        rmtc::LlmClient c;
        rmtc::LlmConfig cfg;
        cfg.quota = 5;
        c.configure(cfg);
        c.setTransport(&tr);
        c.start();
        CHECK(c.submit("s", 10, "rate limited") > 0, "L3 accepted");
        rmtc::LlmResult r;
        bool got = false;
        for (int i = 0; i < 100 && !got; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            got = c.poll(10, r);
        }
        CHECK(got && !r.ok && r.status == 429, "L3 429 reported");
        CHECK(c.quotaLeft() == 5 && c.failed() == 1, "L3 429 keeps quota");
        c.stop();
    }

    // ---------------- 3b) 429 rate_limit：可重试 + 释放去重 hash ----------------
    {
        FakeTransport tr;
        tr.status = 429;
        tr.latency_ms = 1;
        tr.failBody = "{\"error\":{\"code\":\"rate_limit\"}}";
        rmtc::LlmClient c;
        rmtc::LlmConfig cfg;
        cfg.quota = 5;
        c.configure(cfg);
        c.setTransport(&tr);
        c.start();
        CHECK(c.submit("s", 10, "retryable event") > 0, "L3b accepted");
        rmtc::LlmResult r;
        bool got = false;
        for (int i = 0; i < 100 && !got; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            got = c.poll(10, r);
        }
        CHECK(got && !r.ok && r.status == 429 && r.retryable,
              "L3b rate_limit marked retryable");
        // 旧实现把文本 hash 留在 seen_，第二次 submit 会被当成重复直接吞掉。
        CHECK(c.submit("s", 11, "retryable event") > 0,
              "L3b retryable failure releases dedup hash");
        c.stop();
    }

    // ---------------- 4) 超时 + 过期丢弃 ----------------
    {
        FakeTransport tr;
        tr.latency_ms = 200;  // 超过下面的 timeout
        rmtc::LlmClient c;
        rmtc::LlmConfig cfg;
        cfg.timeout_ms = 20;
        cfg.max_age_ticks = 5;
        c.configure(cfg);
        c.setTransport(&tr);
        c.start();
        c.submit("s", 10, "slow event");
        rmtc::LlmResult r;
        bool got = false;
        for (int i = 0; i < 100 && !got; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            got = c.poll(10, r);
        }
        CHECK(got && !r.ok, "L4 timeout reported");
        // 再提交一条，但用很晚的 now_tick 去取 → 过期
        c.resetSession();
        c.submit("s", 10, "another event");
        bool staleGot = false;
        for (int i = 0; i < 100; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            if (c.poll(1000, r)) { staleGot = true; break; }
        }
        CHECK(!staleGot && c.droppedStale() >= 1, "L4 stale result dropped");
        c.stop();
    }

    // ---------------- 5) 事件评分器 ----------------
    {
        rmtc::World w(24, 24);
        rmtc::Perception p;
        p.tick = 100;
        p.x = 10;
        p.y = 10;
        p.hp = 100;
        p.effective_damage = 100.f;
        p.alive = true;

        rmtc::EventScorer sc;
        rmtc::EventReport rep;

        // 爆炸：站在矩形内、来不及走、会被炸死
        rep.kind = rmtc::EVK_EXPLOSION;
        rep.x0 = 8; rep.y0 = 8; rep.x1 = 12; rep.y1 = 12;
        p.hp = 60;  // 60 HP + 80 爆炸 = 必死
        rep.activate_tick = 101;  // 只剩 1 Tick，离边界 3 格
        rep.confidence = 0.9f;
        rmtc::EventAdvice a = sc.score(rep, p, w);
        CHECK(a.avoid && a.value < -3.0, "E1 lethal blast -> avoid, big negative");

        // 爆炸：离得远
        p.x = 1; p.y = 1;
        a = sc.score(rep, p, w);
        CHECK(!a.avoid && a.value == 0.0, "E2 outside blast -> neutral");
        p.hp = 100;

        // 回血点：低血且不远
        p.x = 10; p.y = 10; p.hp = 30;
        rep.kind = rmtc::EVK_HEAL;
        rep.x0 = 12; rep.y0 = 10; rep.x1 = 12; rep.y1 = 10;
        rep.activate_tick = 110;
        rep.end_tick = 130;
        a = sc.score(rep, p, w);
        CHECK(a.value > 0.5 && a.has_target && a.tx == 12 && a.ty == 10,
              "E3 heal point targeted");

        // 无双：D 已满 → 不值得冒险
        p.hp = 100; p.effective_damage = 520.f;
        rep.kind = rmtc::EVK_FORTRESS;
        rep.x0 = 11; rep.y0 = 11; rep.x1 = 13; rep.y1 = 13;
        rep.activate_tick = 120; rep.end_tick = 170;
        a = sc.score(rep, p, w);
        CHECK(a.value < 0.0, "E4 fortress with D capped -> negative");

        // 世界分裂：站在线上且来不及走
        rep.kind = rmtc::EVK_SPLIT;
        rep.x0 = 10; rep.y0 = 5; rep.x1 = 10; rep.y1 = 20;
        rep.activate_tick = 101;
        a = sc.score(rep, p, w);
        CHECK(a.avoid, "E5 on split line -> avoid");

        // 低置信度：不采纳
        rep.kind = rmtc::EVK_EXPLOSION;
        rep.confidence = 0.2f;
        a = sc.score(rep, p, w);
        CHECK(a.value == 0.0 && !a.avoid, "E6 low confidence ignored");
    }

    // ---------------- 6) 评分器耗时 ----------------
    {
        rmtc::World w(24, 24);
        rmtc::Perception p;
        p.tick = 100; p.x = 10; p.y = 10; p.hp = 60; p.alive = true;
        rmtc::EventScorer sc;
        rmtc::EventReport rep;
        rep.kind = rmtc::EVK_HEAL;
        rep.x0 = 12; rep.y0 = 10; rep.x1 = 12; rep.y1 = 10;
        rep.activate_tick = 110; rep.end_tick = 130; rep.confidence = 0.9f;
        const int kIter = 200000;
        const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        double sink = 0;
        for (int i = 0; i < kIter; ++i) {
            p.tick = 100 + (i & 7);   // 让输入变化，防止编译器把整段循环提出去
            p.hp = 40 + (i & 31);
            sink += sc.score(rep, p, w).value;
        }
        const long us = static_cast<long>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0)
                .count());
        std::printf("event scorer: %.3f us/call (sink=%.0f)\n",
                    static_cast<double>(us) / kIter, sink);
        CHECK(us / kIter < 5, "E7 scorer < 5us per call");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
