// dns_check.cpp —— 验证 DNS 不再无限阻塞请求：数字 IP 走快速路径，
// 域名解析在总截止内完成/失败，坏域名不会拖死 3s 的动作窗口。
#include <chrono>
#include <cstdio>
#include <string>

#include "rmtc/http_transport.h"

using Clock = std::chrono::steady_clock;

static long msSince(Clock::time_point t0) {
    return static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                 Clock::now() - t0).count());
}

int main() {
    int pass = 0, fail = 0;
    auto check = [&](bool ok, const char* msg) {
        if (ok) ++pass; else { ++fail; std::printf("FAIL: %s\n", msg); }
    };

    rmtc::HttpTransport tp;
    int st = 0;
    std::string resp, err;

    // 1) 数字 IP（快速路径，不触发 DNS）：本机端口 1 没有监听 → 连接被拒/超时，
    //    但总耗时必须受 3000ms 总截止约束（不会无限等待）。
    {
        Clock::time_point t0 = Clock::now();
        tp.post("http://127.0.0.1:1/x", "{}", "t", 3000, st, resp, err);
        const long ms = msSince(t0);
        check(ms <= 3100 && err.find("dns") == std::string::npos,
              "numeric IP bounded by request deadline (no DNS)");
        std::printf("numeric: %ldms err=%s\n", ms, err.c_str());
    }
    // 2) localhost：走 DNS 线程路径，解析应成功（错误是 connect 而不是 dns）。
    {
        Clock::time_point t0 = Clock::now();
        tp.post("http://localhost:1/x", "{}", "t", 3000, st, resp, err);
        const long ms = msSince(t0);
        check(ms <= 3100 && err.find("dns") == std::string::npos,
              "localhost resolved then connect bounded");
        std::printf("localhost: %ldms err=%s\n", ms, err.c_str());
    }
    // 3) 坏域名：解析失败应快速返回（有 30s 冷却，后续更快），不能拖到 3s。
    {
        Clock::time_point t0 = Clock::now();
        tp.post("http://no-such-host-rmtc.invalid:1/x", "{}", "t", 3000, st, resp, err);
        const long ms = msSince(t0);
        check(ms < 1800 && err.find("dns") != std::string::npos,
              "bad host fails fast with dns error");
        std::printf("badhost: %ldms err=%s\n", ms, err.c_str());
    }
    std::printf("dns_check: %d passed, %d failed\n", pass, fail);
    return fail == 0 ? 0 : 1;
}
