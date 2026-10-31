// main.cpp —— ROS 2 Humble 薄壳
//
// 只做四件事：
//   1. 读取席位/队名（ROS 参数优先，环境变量可覆盖）；
//   2. 订阅 /team_<seat>/perception、发布 /team_<seat>/action；
//   3. 发布 /team_<seat>/registration（RELIABLE + TRANSIENT_LOCAL，周期重发）；
//   4. 把感知 JSON 原样交给 rmtc::Agent；Agent 内部完成事件/LLM/决策。
//
// 决策与 AI 都不在这一层：
//   * Agent 在 onPerception 里异步 submit/poll LLM，绝不阻塞动作窗口；
//   * 本节点只负责 ROS QoS/话题/参数/注册。
//
// 环境变量：
//   RMTC_SEAT         席位 A/B/C/D（覆盖 ROS 参数 seat）
//   RMTC_TEAM_NAME    队名（覆盖 ROS 参数 team_name，1–96 字节）
//   RMTC_AI_URL       统一 AI 网关完整端点，例如
//                     http://127.0.0.1:8000/v1/chat/completions
//                     （空 = 不挂 LLM，本地策略；任务分会有 M=0 风险）
//   RMTC_AI_TOKEN     本场本席位令牌
//   RMTC_AI_MODEL     可选；默认 deepseek-flash
//   RMTC_AI_TIMEOUT_MS            可选；事件请求总超时，默认 3000ms
//   RMTC_AI_HANDSHAKE_TIMEOUT_MS  可选；资格握手总超时，默认 4500ms
//   RMTC_AI_QUOTA                 可选；本地成功调用上限，默认 0=不设限（网关 429 兜底）
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cstdlib>
#include <exception>
#include <functional>
#include <memory>
#include <string>

#include "rmtc/agent.h"
#include "rmtc/http_transport.h"
#include "rmtc/json.h"
#include "rmtc/llm.h"

using std::placeholders::_1;

namespace {

std::string envOr(const char* name, const std::string& fallback = "") {
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : fallback;
}

// 现场可调的整数环境变量（夹到安全范围，非法值回落默认）。
int envInt(const char* name, int fallback, int lo, int hi) {
    const std::string s = envOr(name);
    if (s.empty()) return fallback;
    char* end = NULL;
    const long v = std::strtol(s.c_str(), &end, 10);
    if (!end || *end != '\0') return fallback;
    if (v < lo) return lo;
    if (v > hi) return hi;
    return static_cast<int>(v);
}

// 协议 §2：队名 1–96 个 UTF-8 字节，不得含换行/回车。
// 按完整字符复制，避免在 96 字节处切断多字节序列；非法字节直接丢弃。
std::string sanitizeTeamName(const std::string& in) {
    std::string out;
    for (std::size_t i = 0; i < in.size();) {
        const unsigned char c = static_cast<unsigned char>(in[i]);
        std::size_t len = 1;
        if (c < 0x80) len = 1;
        else if ((c & 0xE0) == 0xC0) len = 2;
        else if ((c & 0xF0) == 0xE0) len = 3;
        else if ((c & 0xF8) == 0xF0) len = 4;
        else { ++i; continue; }
        if (i + len > in.size()) break;
        bool ok = true;
        for (std::size_t k = 1; k < len; ++k) {
            if ((static_cast<unsigned char>(in[i + k]) & 0xC0) != 0x80)
                ok = false;
        }
        if (!ok) { ++i; continue; }
        if (c == '\n' || c == '\r') { i += len; continue; }
        if (out.size() + len > 96) break;
        out.append(in, i, len);
        i += len;
    }
    return out.empty() ? "RMTC Bot" : out;
}

std::string validSeat(const std::string& s) {
    if (s.size() == 1 && s[0] >= 'A' && s[0] <= 'D') return s;
    return "A";
}

}  // namespace

class RmtcBot : public rclcpp::Node {
public:
    RmtcBot() : Node("rmtc_bot") {
        std::string seat = this->declare_parameter<std::string>("seat", "A");
        seat = envOr("RMTC_SEAT", seat);
        seat = validSeat(seat);

        std::string team_name =
            this->declare_parameter<std::string>("team_name", "RMTC Bot");
        team_name = sanitizeTeamName(envOr("RMTC_TEAM_NAME", team_name));
        debug_decision_ = envInt("RMTC_DEBUG_DECISION", 0, 0, 1) != 0;

        // 先建 LLM（不接管 Agent 所有权），再建 Agent 并挂载。
        // 练习裁判默认地址是 http://127.0.0.1:8080/v1/chat/completions；
        // 正式比赛用 RMTC_AI_URL 覆盖。令牌同时兼容裁判教程里的
        // RMTC_TEAM_TOKEN 和本仓库早期文档的 RMTC_AI_TOKEN。
        const std::string ai_url = envOr(
            "RMTC_AI_URL", "http://127.0.0.1:8080/v1/chat/completions");
        const std::string ai_token =
            envOr("RMTC_TEAM_TOKEN", envOr("RMTC_AI_TOKEN"));
        const std::string ai_model = envOr("RMTC_AI_MODEL");
        // 现场网关延迟与练习网关可能不同：不改代码，用环境变量调总超时。
        // 事件超时默认 3000ms（6 个 Tick；异步执行，不阻塞动作窗口）。
        // 旧默认 1800ms 对真实网关偏紧：上游排队/冷启动时会稳定超时，
        // 公告反复重试直到事件过期。握手默认 4500ms（冷启动更慢）。
        const int ai_timeout_ms = envInt("RMTC_AI_TIMEOUT_MS", 3000, 200, 60000);
        const int ai_hs_timeout_ms =
            envInt("RMTC_AI_HANDSHAKE_TIMEOUT_MS", 4500, 200, 60000);
        // 协议：额度按席位令牌全生命周期累计，每场上限赛前公告，0=不限。
        // 默认 0（不自己设上限，交给网关 429 兜底），可用 RMTC_AI_QUOTA 收紧。
        const int ai_quota = envInt("RMTC_AI_QUOTA", 0, 0, 100000);
        if (!ai_token.empty()) {
            http_transport_.reset(new rmtc::HttpTransport());
            llm_.reset(new rmtc::LlmClient());
            rmtc::LlmConfig lc;
            lc.timeout_ms = ai_timeout_ms;
            lc.max_queue = 8;
            lc.max_age_ticks = 100;
            lc.quota = ai_quota;
            llm_->configure(lc);
            llm_->setEndpoint(ai_url, ai_token);
            // 关键：LlmClient 不持有 transport；不挂 HttpTransport 时
            // worker 会直接返回 status=0 / "no transport"，一个网络包都不会发。
            llm_->setTransport(http_transport_.get());
            llm_->start();
        }

        rmtc::Config cfg;
        if (!ai_model.empty()) cfg.llm_model = ai_model;
        cfg.llm_handshake_timeout_ms = ai_hs_timeout_ms;
        // 实战日志（2026-10-30）：资源在 Tick 100 前被拿完后，A 在
        // R_WAIT_NO_TARGET 里原地等到 500。默认仍关；现场可用环境变量开启
        // 兜底巡逻，确认不会明显增加死亡后再考虑默认。
        cfg.no_target_fallback =
            envInt("RMTC_NO_TARGET_FALLBACK", 0, 0, 1) != 0;
        cfg.no_target_fallback_ticks =
            envInt("RMTC_NO_TARGET_FALLBACK_TICKS", 20, 1, 100);
        cfg.no_target_fallback_r2 = static_cast<float>(
            envInt("RMTC_NO_TARGET_FALLBACK_R2", 100, 0, 200));
        // P5：资源耗尽且自己 R2 很低时，转猎杀/争夺；默认开启。
        cfg.post_resource_hunt =
            envInt("RMTC_POST_RESOURCE_HUNT", 1, 0, 1) != 0;
        cfg.post_resource_hunt_r2 = static_cast<float>(
            envInt("RMTC_POST_RESOURCE_HUNT_R2", 40, 0, 120));
        // P6：目标不变时锁定完整路径，避免 first-step 抖动；默认开。
        cfg.planner_path_commit =
            envInt("RMTC_PATH_COMMIT", 1, 0, 1) != 0;
        cfg.planner_progress_guard =
            envInt("RMTC_PROGRESS_GUARD", 1, 0, 1) != 0;
        // P7：只在“敌人已在高价值资源旁且明显先到”时放弃；默认关。
        cfg.high_resource_abandon =
            envInt("RMTC_HIGH_RESOURCE_ABANDON", 0, 0, 1) != 0;
        cfg.race_gate = envInt("RMTC_RACE_GATE", 0, 0, 1) != 0;
        agent_.reset(new rmtc::Agent(cfg));
        agent_->setLogger([this](int level, const std::string& msg) {
            if (level == 0) RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
            else if (level == 1) RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
            else RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        });
        if (llm_) agent_->attachLlm(llm_.get());

        // 协议 §5.1：感知/动作 RELIABLE + VOLATILE + KeepLast(1)；
        // depth=1 避免 ROS 队列堆积过期帧，只处理最新消息。
        rclcpp::QoS qos(1);
        qos.reliable().durability_volatile();
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/team_" + seat + "/perception", qos,
            std::bind(&RmtcBot::onPerception, this, _1));
        pub_ = this->create_publisher<std_msgs::msg::String>(
            "/team_" + seat + "/action", qos);

        // 协议 §3：随机事件 / 任务公告是独立的公开纯文本话题。
        // 只依赖感知里的 event_text 会受“近期公告缓存”容量限制——公告一旦
        // 被挤出缓存，就再也不会进入 LLM（表现为“事件没解析出来”）。这里
        // 直接订阅公告话题并转交 Agent，与感知缓存解耦。
        rclcpp::QoS qev(128);
        qev.reliable().transient_local();
        sub_event_ = this->create_subscription<std_msgs::msg::String>(
            "/referee/event_text", qev,
            [this](const std_msgs::msg::String::SharedPtr m) {
                if (agent_) agent_->onRefereeEventText(m->data);
            });
        rclcpp::QoS qmission(1);
        qmission.reliable().transient_local();
        sub_mission_ = this->create_subscription<std_msgs::msg::String>(
            "/referee/mission", qmission,
            [this](const std_msgs::msg::String::SharedPtr m) {
                if (agent_) agent_->onRefereeMissionText(m->data);
            });

        // 协议 §2：注册 RELIABLE + TRANSIENT_LOCAL，启动即发，之后每 10s 重发。
        rclcpp::QoS qreg(1);
        qreg.reliable().transient_local();
        pub_reg_ = this->create_publisher<std_msgs::msg::String>(
            "/team_" + seat + "/registration", qreg);
        std_msgs::msg::String reg;
        reg.data = "{\"team\":\"" + seat + "\",\"name\":\"" +
                   rmtc::jsonEscape(team_name) + "\"}";
        pub_reg_->publish(reg);
        reg_timer_ = this->create_wall_timer(
            std::chrono::seconds(10), [this, reg]()  {
                pub_reg_->publish(reg);
            });

        RCLCPP_INFO(this->get_logger(),
                    "RMTC stage2 bot started: seat=%s team=%s ai=%s token=%s",
                    seat.c_str(), team_name.c_str(),
                    llm_ ? ai_url.c_str() : "(not attached)",
                    ai_token.empty() ? "missing" : "set");
        RCLCPP_INFO(this->get_logger(),
                    "no-target fallback=%d ticks=%d r2=%.0f post-hunt=%d r2<%.0f "
                    "path-commit=%d high-abandon=%d race=%d progress-guard=%d",
                    cfg.no_target_fallback ? 1 : 0,
                    cfg.no_target_fallback_ticks,
                    cfg.no_target_fallback_r2,
                    cfg.post_resource_hunt ? 1 : 0,
                    cfg.post_resource_hunt_r2,
                    cfg.planner_path_commit ? 1 : 0,
                    cfg.high_resource_abandon ? 1 : 0,
                    cfg.race_gate ? 1 : 0,
                    cfg.planner_progress_guard ? 1 : 0);
        if (llm_) {
            RCLCPP_INFO(this->get_logger(),
                        "AI model=%s (request carries thinking.disabled + "
                        "response_format=json_object)",
                        cfg.llm_model.c_str());
            RCLCPP_INFO(this->get_logger(),
                        "AI timeout event=%dms handshake=%dms quota=%d "
                        "(0=gateway-enforced)",
                        ai_timeout_ms, ai_hs_timeout_ms, ai_quota);
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "No AI token: set RMTC_TEAM_TOKEN (practice-A/B/C/D) "
                        "or RMTC_AI_TOKEN. Local-only mode will make M=0.");
        }
    }

private:
    void onPerception(const std_msgs::msg::String::SharedPtr msg) {
        std::string action;
        try {
            action = agent_->onPerception(msg->data);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Agent exception: %s", e.what());
            return;
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Agent unknown exception");
            return;
        }
        if (action.empty()) return;  // 解析失败/旧帧/阵亡空动作：不发布
        std_msgs::msg::String out;
        out.data = action;
        pub_->publish(out);

        const int tick = agent_->lastTick();
        if (debug_decision_) {
            const rmtc::Decision& d = agent_->lastDecision();
            const rmtc::Perception& p = agent_->lastPerception();
            RCLCPP_INFO(this->get_logger(),
                        "DEC tick=%d pos=(%d,%d) hp=%d r2=%.0f reason=%s "
                        "target=%d planV=%.1f planC=%.1f rate=%.2f gate=%d action=%s",
                        tick, p.x, p.y, p.hp, p.resource_value,
                        rmtc::reason_name(d.reason), d.targetIdx, d.planValue,
                        d.planCost, agent_->plannerRate(),
                        agent_->economyGateActive() ? 1 : 0, action.c_str());
        }
        if (tick >= 0 && tick - last_log_tick_ >= 50) {
            last_log_tick_ = tick;
            RCLCPP_INFO(this->get_logger(),
                        "tick=%d qualified=%d aiDisabled=%d handshakes=%d "
                        "lastHttp=%d handshake=%d err=%s",
                        tick, agent_->quotaSecured() ? 1 : 0,
                        agent_->llmDisabled() ? 1 : 0,
                        agent_->handshakeAttempts(), agent_->lastLlmStatus(),
                        agent_->handshakeAttempts(), agent_->lastLlmError().c_str());
        }
    }

    // 所有指针的析构顺序：agent_ → llm_ → http_transport_。
    // LlmClient 只保存 HttpTransport 的裸指针，transport 必须晚于它析构。
    std::unique_ptr<rmtc::HttpTransport> http_transport_;
    std::unique_ptr<rmtc::LlmClient> llm_;
    std::unique_ptr<rmtc::Agent> agent_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_event_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mission_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_reg_;
    rclcpp::TimerBase::SharedPtr reg_timer_;
    int last_log_tick_ = -1;
    bool debug_decision_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RmtcBot>());
    rclcpp::shutdown();
    return 0;
}
