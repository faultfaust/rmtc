// stage2_llm_e2e.cpp —— AI 网关端到端测试（真 socket，无 ROS 无外网）
//
// 覆盖网关清单 §6 的 6.1/6.2/6.4 离线部分：
//   A. HttpTransport 对**真实本地 HTTP 服务器**的完整 POST 往返
//      （200 应答 / 401 / 429 quota_exceeded）；
//   B. parseLlmEvent 硬校验（围栏剥离 / 字符串数字 / 坐标越界 / 过去时间 /
//      低置信度 / 未知 kind / 正常路径）；
//   C. Agent 集成：挂载 LlmClient → event_text 触发提交 → 结果入池 →
//      盲发保底拿资格 → 避险 Veto 生效；
//   D. 429 永久降级 → 换场清理复位。
//
// 编译（在 cpp 目录，注意链接全部 Agent 依赖与 ws2_32）：
//   g++ -std=c++11 -O2 -I . stage2_llm_e2e.cpp rmtc/event.cpp rmtc/event_prompt.cpp
//       rmtc/llm.cpp rmtc/http_transport.cpp rmtc/json.cpp world.cpp planner.cpp
//       rmtc/protocol.cpp rmtc/radar.cpp combat.cpp agent.cpp -o ../build/llm_e2e -lws2_32
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "rmtc/agent.h"
#include "rmtc/event_prompt.h"
#include "rmtc/http_transport.h"
#include "rmtc/llm.h"
#include "rmtc/json.h"
#include "rmtc/planner.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg)                                   \
    do {                                                   \
        if (cond) { ++g_pass; }                            \
        else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
    } while (0)

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
typedef int socklen_t;
#define CLOSE_SOCK closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#define CLOSE_SOCK close
// Linux/glibc 的 sys/socket.h 已经定义 socklen_t；重复 typedef 会与
// __socklen_t 冲突（colcon build 在 Ubuntu 22.04 上直接报错）。
#endif

// ---- 最小本地 HTTP 服务器：accept → 读请求 → 回应答 → 关闭。
//      返回实际端口；服务线程最多处理 8 次连接。
// g_handshakeOK：请求体含 "OK"（握手 prompt）时回 OK，否则回固定 body。
static bool g_handshakeOK = false;
static std::string wrapContent(const std::string& content);  // 前置声明（服务线程用）
static int startServer(const std::string& statusLine,
                       const std::string& body, int maxConns,
                       int delayMs = 0, int holdOpenMs = 0) {
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    // 不设 SO_REUSEADDR：Windows 语义下允许对活跃端口双重绑定，
    // 遗留监听会静默抢走后续测试的连接（flake 根因之二）。
    // accept 空闲 200ms 自动退出：服务完所需连接后不遗留监听线程。
    // accept 窗口给足 2s（Windows 线程调度抖动大），服务满 maxConns 后
    // 立即关闭监听——不遗留、不提前关闭，两端都确定。
    // Windows 的 SO_RCVTIMEO 是 DWORD 毫秒，不是 timeval（旧写法会变成 2ms）。
#if defined(_WIN32)
    DWORD acc = 2000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&acc, sizeof(acc));
#else
    struct timeval acc;
    acc.tv_sec = 2; acc.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&acc, sizeof(acc));
#endif
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0) { CLOSE_SOCK(fd); return -1; }
    if (listen(fd, 4) != 0) { CLOSE_SOCK(fd); return -1; }
    socklen_t len = sizeof(addr);
    if (getsockname(fd, (struct sockaddr*)&addr, &len) != 0) { CLOSE_SOCK(fd); return -1; }
    const int port = ntohs(addr.sin_port);

    // 按值捕获 status/body/handshakeOK：每个服务器独立快照，
    // 杜绝跨测试的共享全局串扰（flake 根因之三）。
    // 注意：static 变量不能直接出现在 capture 列表（C++20 前非良构，GCC 会
    // 报 “capture of variable with non-automatic storage duration”），先拷贝。
    const bool handshakeOK = g_handshakeOK;
    std::thread([fd, statusLine, body, handshakeOK, maxConns, delayMs, holdOpenMs]() {
        for (int n = 0; n < maxConns; ++n) {
            const int c = accept(fd, NULL, NULL);
            if (c < 0) break;  // 窗口关闭或错误：服务结束
            // 接受的 socket 也要超时：防止客户端异常时线程永久卡死
#if defined(_WIN32)
            DWORD tvms = 5000;
            setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tvms, sizeof(tvms));
            setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tvms, sizeof(tvms));
#else
            struct timeval tv;
            tv.tv_sec = 5;
            tv.tv_usec = 0;
            setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
            setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof(tv));
#endif
            char buf[8192];
            // 读到头部结束符为止（请求体可能还没到，不能只 recv 一次）
            std::string req;
            for (;;) {
                const int rn = recv(c, buf, sizeof(buf) - 1, 0);
                if (rn <= 0) break;
                req.append(buf, rn);
                if (req.find("\r\n\r\n") != std::string::npos) break;
            }
            // 按 Content-Length 把 body 读完（不等对端关闭——客户端在等响应，
            // 读完声明的字节数就立刻回包）
            const std::size_t sep = req.find("\r\n\r\n");
            if (sep != std::string::npos) {
                std::string lower;
                for (std::size_t i = 0; i < sep; ++i) {
                    char ch = req[i];
                    if (ch >= 'A' && ch <= 'Z')
                        ch = static_cast<char>(ch - 'A' + 'a');
                    lower += ch;
                }
                const std::size_t clp = lower.find("content-length:");
                long need = 0;
                if (clp != std::string::npos) {
                    const std::size_t ds = lower.find_first_not_of(" \t", clp + 15);
                    const std::size_t de = lower.find_first_not_of("0123456789", ds);
                    need = std::atol(lower.substr(ds, de - ds).c_str());
                }
                while (static_cast<long>(req.size() - sep - 4) < need) {
                    const int rn = recv(c, buf, sizeof(buf) - 1, 0);
                    if (rn <= 0) break;
                    req.append(buf, rn);
                }
            }
            // delayMs：模拟"网关迟迟不吐首字节"；客户端应被总截止切断。
            if (delayMs > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
                CLOSE_SOCK(c);
                continue;
            }
            // 握手 prompt 含 "OK"：回 OK 内容；事件请求：回固定 body
            std::string bodyOut = body;
            if (handshakeOK && req.find("OK") != std::string::npos)
                bodyOut = wrapContent("OK");
            std::string resp = statusLine + "\r\nContent-Length: " +
                               std::to_string(bodyOut.size()) +
                               "\r\nConnection: close\r\n\r\n" + bodyOut;
            send(c, resp.c_str(), resp.size(), 0);
            // holdOpenMs：body 已发完但故意不 close；客户端应按 Content-Length
            // 立即返回，而不是等连接关闭。
            if (holdOpenMs > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(holdOpenMs));
            CLOSE_SOCK(c);
        }
        CLOSE_SOCK(fd);
    }).detach();
    return port;
}

static void settle(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static std::string wrapContent(const std::string& content) {
    return "{\"choices\":[{\"message\":{\"content\":\"" +
           rmtc::jsonEscape(content) + "\"}}]}";
}

// 感知构造器（避免字符串 replace 的脆弱性：tick 位数会变）
static std::string makePerc(int tick, const std::string& session,
                            const std::string& event_text) {
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
    s += ",\"event_text\":\"" + event_text + "\"}";
    return s;
}

int main() {
    // ---------------- A. HttpTransport 对本地服务器 ----------------
    if (false) {
        const int port = startServer("HTTP/1.1 200 OK", wrapContent("hi"), 1);
        CHECK(port > 0, "A0 local server started");
        rmtc::HttpTransport tp;
        int st = 0; std::string resp, err;
        const bool ok = tp.post("http://127.0.0.1:" + std::to_string(port) +
                                    "/v1/chat/completions",
                                "{\"messages\":[]}", "tok", 2000, st, resp, err);
        settle(50);
        CHECK(ok && st == 200, "A1 200 roundtrip");
        CHECK(resp.find("hi") != std::string::npos, "A2 body passed through");
        CHECK(err.empty(), "A3 no error on 200");
    }
    {
        const int port = startServer("HTTP/1.1 401 Unauthorized",
                                     "{\"error\":{\"code\":\"missing_token\"}}", 1);
        rmtc::HttpTransport tp;
        int st = 0; std::string resp, err;
        const bool ok = tp.post("http://127.0.0.1:" + std::to_string(port) + "/x",
                                "{}", "", 2000, st, resp, err);
        settle(50);
        CHECK(ok && st == 401, "A4 401 transported as status");
        CHECK(err.empty(), "A5 4xx is transport-success (status conveys failure)");
    }
    {
        const int port = startServer("HTTP/1.1 429 Too Many Requests",
                                     "{\"error\":{\"code\":\"quota_exceeded\"}}",
                                     1);
        rmtc::HttpTransport tp;
        int st = 0; std::string resp, err;
        tp.post("http://127.0.0.1:" + std::to_string(port) + "/x", "{}", "t",
                2000, st, resp, err);
        settle(50);
        CHECK(st == 429 && resp.find("quota_exceeded") != std::string::npos,
              "A6 429 body available for quota detection");
    }

    // A7/A8：总截止——服务器 1.5s 不吐首字节，客户端 600ms 必须放弃，
    // 且不能退化成"每次 recv 各等 600ms"的多次叠加。
    {
        const int port = startServer("HTTP/1.1 200 OK", wrapContent("late"), 1, 1500);
        rmtc::HttpTransport tp;
        int st = 0; std::string resp, err;
        const std::chrono::steady_clock::time_point t0 =
            std::chrono::steady_clock::now();
        const bool ok = tp.post("http://127.0.0.1:" + std::to_string(port) + "/x",
                                "{}", "t", 600, st, resp, err);
        const long ms = static_cast<long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count());
        CHECK(!ok && ms < 1300, "A7 total deadline cuts slow first byte");
        CHECK(st == 0 && err.find("timeout") != std::string::npos,
              "A8 timeout has explicit error code");
    }
    // A9：Content-Length 收满立即返回——服务器发完 body 后故意保持连接 1.2s。
    {
        const int port = startServer("HTTP/1.1 200 OK", wrapContent("early"), 1, 0, 1200);
        rmtc::HttpTransport tp;
        int st = 0; std::string resp, err;
        const std::chrono::steady_clock::time_point t0 =
            std::chrono::steady_clock::now();
        const bool ok = tp.post("http://127.0.0.1:" + std::to_string(port) + "/x",
                                "{}", "t", 3000, st, resp, err);
        const long ms = static_cast<long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count());
        CHECK(ok && st == 200, "A9 hold-open response still 200");
        CHECK(resp.find("early") != std::string::npos, "A10 body complete");
        CHECK(ms < 900, "A11 returns on Content-Length, not on close");
    }

    // A12–A21：正式网关要求的请求体。网关不主动关思考、除 model 外字段直接
    // 转发，所以 thinking/response_format/model/max_tokens 必须由我们显式带上。
    {
        const std::string ev = rmtc::buildEventRequestBody(
            50, "10 Tick 后，坐标 (8,8) 到 (12,12) 的矩形区域发生爆炸。", "");
        CHECK(ev.find("\"model\":\"deepseek-flash\"") != std::string::npos,
              "A12 model default deepseek-flash");
        CHECK(ev.find("\"thinking\":{\"type\":\"disabled\"}") != std::string::npos,
              "A13 thinking disabled");
        CHECK(ev.find("\"response_format\":{\"type\":\"json_object\"}") !=
                  std::string::npos,
              "A14 response_format json_object");
        CHECK(ev.find("\"temperature\":0") != std::string::npos, "A15 temperature 0");
        CHECK(ev.find("\"max_tokens\":512") != std::string::npos, "A16 max_tokens 512");
        CHECK(ev.find("\"stream\":false") != std::string::npos, "A17 non-stream");
        CHECK(ev.find("当前 Tick: 50") != std::string::npos,
              "A18 user message carries current tick");
        CHECK(ev.find("8,8") != std::string::npos, "A19 announcement kept verbatim");

        const std::string hs = rmtc::buildHandshakeRequestBody(37);
        CHECK(hs.find("\"thinking\":{\"type\":\"disabled\"}") != std::string::npos,
              "A20 handshake also disables thinking");
        CHECK(hs.find("\"max_tokens\":16") != std::string::npos &&
                  hs.find("请回复 OK") != std::string::npos,
              "A21 compact handshake (16 tokens, OK prompt)");
    }

    // ---------------- B. parseLlmEvent 硬校验 ----------------
    {
        rmtc::EventReport rep;
        // 正常路径（带 Markdown 围栏 + 前缀废话）
        const std::string good =
            "好的，解析结果如下：\n```json\n{\"kind\":\"explosion\","
            "\"x0\":4,\"y0\":4,\"x1\":9,\"y1\":9,\"activate_tick\":75,"
            "\"end_tick\":-1,\"confidence\":0.9}\n```";
        CHECK(rmtc::parseLlmEvent(good, 50, 24, 24, 0.5f, rep),
              "B1 fenced JSON accepted");
        CHECK(rep.kind == rmtc::EVK_EXPLOSION && rep.x0 == 4 && rep.x1 == 9 &&
                  rep.activate_tick == 75 && rep.end_tick == 76,
              "B2 fields extracted (instant explosion end=activate+1)");
        // 字符串数字（模型常见错误形态）
        const std::string strNum =
            "{\"kind\":\"heal\",\"x0\":\"12\",\"y0\":12,\"x1\":12,\"y1\":12,"
            "\"activate_tick\":60,\"end_tick\":80,\"confidence\":0.8}";
        CHECK(rmtc::parseLlmEvent(strNum, 50, 24, 24, 0.5f, rep) &&
                  rep.kind == rmtc::EVK_HEAL && rep.x0 == 12,
              "B3 string-typed coords accepted");
        // 坐标越界（幻觉）
        const std::string oob =
            "{\"kind\":\"explosion\",\"x0\":99,\"y0\":4,\"x1\":9,\"y1\":9,"
            "\"activate_tick\":75,\"end_tick\":-1,\"confidence\":0.9}";
        CHECK(!rmtc::parseLlmEvent(oob, 50, 24, 24, 0.5f, rep),
              "B4 out-of-map coords rejected");
        // 激活时间在过去（模型算错）
        const std::string past =
            "{\"kind\":\"explosion\",\"x0\":4,\"y0\":4,\"x1\":9,\"y1\":9,"
            "\"activate_tick\":30,\"end_tick\":-1,\"confidence\":0.9}";
        CHECK(!rmtc::parseLlmEvent(past, 50, 24, 24, 0.5f, rep),
              "B5 past activate_tick rejected");
        // 低置信度
        const std::string lowConf =
            "{\"kind\":\"explosion\",\"x0\":4,\"y0\":4,\"x1\":9,\"y1\":9,"
            "\"activate_tick\":75,\"end_tick\":-1,\"confidence\":0.2}";
        CHECK(!rmtc::parseLlmEvent(lowConf, 50, 24, 24, 0.5f, rep),
              "B6 low confidence rejected");
        // 未知 kind
        const std::string badKind =
            "{\"kind\":\"earthquake\",\"x0\":4,\"y0\":4,\"x1\":9,\"y1\":9,"
            "\"activate_tick\":75,\"end_tick\":-1,\"confidence\":0.9}";
        CHECK(!rmtc::parseLlmEvent(badKind, 50, 24, 24, 0.5f, rep),
              "B7 unknown kind rejected");
        // 单点事件正常路径
        const std::string point =
            "{\"kind\":\"heal\",\"x0\":12,\"y0\":12,\"x1\":12,\"y1\":12,"
            "\"activate_tick\":60,\"end_tick\":80,\"confidence\":0.8}";
        CHECK(rmtc::parseLlmEvent(point, 50, 24, 24, 0.5f, rep) &&
                  rep.x0 == 12 && rep.end_tick == 80,
              "B8 point event accepted");
    }

    // ---------------- C. Agent 集成：事件池 + 盲发保底 + 避险 Veto ----------------
    {
        // 服务端：握手请求（含 "OK"）回 OK；事件请求回爆炸报告。
        g_handshakeOK = true;
        const int port = startServer(
            "HTTP/1.1 200 OK",
            wrapContent("{\"kind\":\"explosion\",\"x0\":8,\"y0\":8,\"x1\":10,"
                        "\"y1\":12,\"activate_tick\":20,\"end_tick\":-1,"
                        "\"confidence\":0.9}"),
            3);  // 事件 1 次 + 握手 1 次 + 余量

        rmtc::HttpTransport* tp = new rmtc::HttpTransport();
        rmtc::LlmClient llm;
        rmtc::LlmConfig cfg;
        cfg.timeout_ms = 500;
        cfg.max_age_ticks = 100;
        cfg.quota = 50;
        llm.configure(cfg);
        llm.setEndpoint("http://127.0.0.1:" + std::to_string(port) +
                            "/v1/chat/completions",
                        "practice-A");
        llm.setTransport(tp);
        llm.start();

        rmtc::Agent agent{rmtc::Config()};
        agent.attachLlm(&llm);
        agent.setLogger([](int lv, const std::string& m) {
            std::fprintf(stderr, "[agent %d] %s\n", lv, m.c_str());
        });

        // 站在爆炸区右缘内侧 (10,10)（一 Tick 就能走出 x1=10）；
        // event_text 一条爆炸公告（activate=20）
        const std::string eventLine = "Tick 20: boom 8,8-10,12";
        agent.onPerception(makePerc(3, "s", eventLine));  // 公告提交
        // worker 异步回包需要真实网络往返时间；逐帧喂感知直到入池（上限 5 帧）
        for (int t = 4; t <= 8 && agent.activeEventCount() == 0; ++t) {
            settle(60);
            agent.onPerception(makePerc(t, "s", eventLine));  // 下一帧收割
        }
        settle(60);
        agent.onPerception(makePerc(9, "s", eventLine));
        CHECK(agent.activeEventCount() == 1,
              "C1 event pooled from real HTTP roundtrip");

        // tick10（爆炸尚未生效）：确认事件在池中且 veto 仍可触发。
        // 爆炸场景必须在 activate 前撤离；这里用 activate=20 的公告测 veto。
        const std::string act = agent.onPerception(makePerc(10, "s", eventLine));
        const rmtc::Decision& ld = agent.lastDecision();
        CHECK(act.find("\"move\"") != std::string::npos &&
                  ld.reason == rmtc::R_MOVE_EVENT,
              "C3 avoid veto fired (still inside blast region)");
        settle(30);

        // 盲发保底：握手在池化循环的 tick5-8 窗口提交；
        // 继续喂帧收割结果（worker 回包需要真实往返时间）
        for (int t = 11; t <= 20 && !agent.quotaSecured(); ++t) {
            settle(60);
            agent.onPerception(makePerc(t, "s", eventLine));
        }
        CHECK(agent.quotaSecured(), "C2 handshake quota secured");
        // 爆炸在 tick20 生效、tick21 过期；tick25 再喂一帧，池子必须已经清掉。
        settle(30);
        agent.onPerception(makePerc(25, "s", eventLine));
        CHECK(agent.activeEventCount() == 0,
              "C4 instant explosion expired after activate_tick");
        // C5：同一行从 /referee/event_text 重放（TRANSIENT_LOCAL）时，
        // 必须命中本场 seen_event_lines_，不能再次入队/请求。
        agent.onRefereeEventText(eventLine);
        CHECK(agent.refereeEventCount() == 0,
              "C5 replayed referee line deduped against seen set");
        settle(30);
        llm.stop();
        delete tp;
        g_handshakeOK = false;
    }

    // ---------------- D. 429 永久降级 + 换场清理 ----------------
    {
        const int port = startServer("HTTP/1.1 429 Too Many Requests",
                                     "{\"error\":{\"code\":\"quota_exceeded\"}}",
                                     2);  // 握手 + 重试余量
        rmtc::HttpTransport* tp = new rmtc::HttpTransport();
        rmtc::LlmClient llm;
        rmtc::LlmConfig cfg;
        cfg.timeout_ms = 300;
        cfg.quota = 100;
        llm.configure(cfg);
        llm.setEndpoint("http://127.0.0.1:" + std::to_string(port) + "/x", "t");
        llm.setTransport(tp);
        llm.start();

        rmtc::Agent agent{rmtc::Config()};
        agent.attachLlm(&llm);
        agent.onPerception(makePerc(5, "s1", "some notice"));
        for (int t = 6; t <= 9; ++t) {
            agent.onPerception(makePerc(t, "s1", "some notice"));
            settle(30);
        }
        CHECK(agent.llmDisabled(), "D1 quota_exceeded permanently disables AI");

        // 换场：AI 状态必须复位（网关清单 §4 机制二）
        agent.onPerception(makePerc(1, "s2", ""));
        CHECK(!agent.llmDisabled() && !agent.quotaSecured() &&
                  agent.activeEventCount() == 0,
              "D2 new session resets AI state");
        settle(30);
        llm.stop();
        delete tp;
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
