#pragma once
#include <string>
#include <vector>
namespace rmtc {
    enum EntityKind { ENT_OBSTACLE, ENT_RESOURCE, ENT_ROBOT };

    struct Entity {
        EntityKind kind;
        int x, y;
        float value = 0.f;
        bool high = false;
    };

    struct LastAction {
        bool present = false;
        int tick = -1;
        std::string action;
        std::string error;
        bool moved = false;
    };

    struct Perception {
        std::string session_id;
        int tick = 0;
        int x = 0, y = 0;
        bool alive = true;
        int vision_radius = 4;
        int map_w = 20, map_h = 20;
        float resource_value = 0.f;
        std::vector<Entity> entities;
        LastAction last_action;
    };
} // namespace rmtc