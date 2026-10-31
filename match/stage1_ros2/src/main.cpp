#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <iostream>

#include "rmtc/world.h"
#include "rmtc/planner.h"
#include "rmtc/config.h"

using json = nlohmann::json;
using std::placeholders::_1;
class RmtcBot : public rclcpp::Node {
public:
    RmtcBot() : Node("rmtc_bot") {
        // 第一阶段席位是 A
        std::string seat = "A";

        // 初始化你的世界和决策器
        world_ = std::make_unique<rmtc::World>(20, 20);
        config_ = rmtc::Config();
        planner_ = std::make_unique<rmtc::Planner>(config_);

        // 订阅感知 (QoS 必须是 Reliable, Volatile, KeepLast(1))
        rclcpp::QoS qos(1);
        qos.reliable().durability_volatile();
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/team_" + seat + "/perception", qos, std::bind(&RmtcBot::on_perception, this, _1));

        // 发布动作
        pub_ = this->create_publisher<std_msgs::msg::String>("/team_" + seat + "/action", qos);

        RCLCPP_INFO(this->get_logger(), "RMTC 机器人节点已启动，等待感知...");
    }

private:
    void on_perception(const std_msgs::msg::String::SharedPtr msg) {
        try {
            // 1. 解析 JSON
            json j = json::parse(msg->data);

            rmtc::Perception p;
            p.tick = j["tick"];
            p.session_id = j["session_id"];
            p.vision_radius = j["vision_radius"];
            p.x = j["self"]["position"][0];
            p.y = j["self"]["position"][1];
            p.alive = j["self"]["alive"];
            p.map_w = j["map"]["width"];
            p.map_h = j["map"]["height"];
            p.resource_value = j["score"]["resource_value"];
            // 解析实体 (需要将 9x9 相对坐标转化为绝对坐标)
            for (auto& ent : j["visible_entities"]) {
                rmtc::Entity e;
                std::string pos_str = ent["position"];
                int dx = 0, dy = 0;
                if (parse9x9(pos_str, dx, dy)) {
                    e.x = p.x + dx;
                    e.y = p.y + dy;
                    if (ent["type"] == "obstacle") e.kind = rmtc::ENT_OBSTACLE;
                    else if (ent["type"] == "resource_normal") { e.kind = rmtc::ENT_RESOURCE; e.value = 10; }
                    else if (ent["type"] == "resource_high") { e.kind = rmtc::ENT_RESOURCE; e.value = 20; }
                    p.entities.push_back(e);
                }
            }

            // 处理上一 Tick 的移动回执 (防撞墙)
            if (!j["self"]["last_action"].is_null()) {
                p.last_action.present = true;
                p.last_action.tick = j["self"]["last_action"]["tick"];
                p.last_action.action = j["self"]["last_action"]["action"];
                p.last_action.moved = j["self"]["last_action"]["moved"];
            }

            // ===== 场次门控（重置逻辑的核心） =====
            // 协议要求：收到新的 session_id 时清空上一场的规划、定位与动作缓存。
            // 世界记忆必须整体作废——updateFromPerception 只在地图尺寸变化时
            // 自动重置，"同尺寸地图重开/换配置"时旧障碍与旧资源会残留成
            // 幽灵目标；上一场撞墙学的硬障碍、规划器的目标/保守窗口也一并失效。
            if (p.session_id != session_id_) {
                if (!session_id_.empty()) {
                    RCLCPP_INFO(this->get_logger(),
                        "检测到新场次 %s（旧场次 %s），重置世界记忆与规划器",
                        p.session_id.c_str(), session_id_.c_str());
                } else {
                    RCLCPP_INFO(this->get_logger(), "接入场次 %s", p.session_id.c_str());
                }
                session_id_ = p.session_id;
                last_tick_ = -1;
                cached_action_.clear();
                world_->reset(p.map_w, p.map_h);
                planner_->reset();
            }

            // ===== Tick 门控 =====
            // 过期帧（晚到的旧感知）直接丢弃，不参与决策；
            // 同 Tick 重发的感知只重发本 Tick 已缓存的动作，不重复推进策略。
            if (p.tick < last_tick_) {
                RCLCPP_WARN(this->get_logger(), "丢弃过期帧 tick=%d（已处理到 %d）",
                            p.tick, last_tick_);
                return;
            }
            if (p.tick == last_tick_ && !cached_action_.empty()) {
                std_msgs::msg::String cached_msg;
                cached_msg.data = cached_action_;
                pub_->publish(cached_msg);
                return;
            }

            // 2. 更新世界模型
            world_->updateFromPerception(p);
            planner_->absorbReceipt(p, *world_);

            // 3. 做出决策
            rmtc::Decision d = planner_->decide(*world_, p);

            // 4. 构造并发布动作 JSON
            json action_j;
            action_j["tick"] = p.tick;
            action_j["session_id"] = p.session_id;

            if (d.kind == rmtc::MOVE_MOVE) {
                action_j["action"] = "move";
                action_j["direction"] = d.direction; // d.direction 就是 "up", "down" 等
                planner_->noteAction(p.tick, p.x, p.y, d.direction);
            } else {
                action_j["action"] = "wait";
            }

            std_msgs::msg::String action_msg;
            action_msg.data = action_j.dump();
            pub_->publish(action_msg);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "解析或决策失败: %s", e.what());
        }
    }

    // 解析协议里的 9x9 字符串
    bool parse9x9(const std::string& s, int& dx, int& dy) {
        dx = 0; dy = 0;
        if (s.size() != 89) return false;
        int row = 0, col = 0;
        for (char c : s) {
            if (c == '\n') { row++; col = 0; continue; }
            if (c == '2') { dx = col - 4; dy = row - 4; }
            col++;
        }
        return true;
    }

    std::unique_ptr<rmtc::World> world_;
    rmtc::Config config_;
    std::unique_ptr<rmtc::Planner> planner_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    // 场次与 Tick 门控状态（新场次时全部重置）
    std::string session_id_;
    int last_tick_ = -1;
    std::string cached_action_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RmtcBot>());
    rclcpp::shutdown();
    return 0;
}
