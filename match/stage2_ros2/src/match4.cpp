// match4.cpp —— 第二阶段四车同场模拟器 v2（含 8 种随机事件）
//
// 结算顺序严格按规则 v1.2 §5.2：
//   1 清理到期效果、激活事件（爆炸/幻术/世界分裂在移动前判定）
//   2 处理死亡；分裂强制复活者本 Tick WAIT
//   3 固化动作、设置本 Tick 防御
//   4 按移动前坐标汇总攻击/反伤/伤害修饰，扣血，分配实际伤害与击杀奖励，处理死亡
//   5 按战斗后存活车占位移动（幻术方向、临时墙）
//   6 结算入点与移动触发（辣椒火线统一结算），处理死亡
//   7 累计堡垒驻留，结算回血
//   8 先加伤害与奖励，再按死亡次数扣 60，最低 0
//   9 收集资源、检查任务、更新复活与保护倒计时
//
// v1.2 关键差异（本文件已实现）：
//   * §7 复活：出生点优先，按曼哈顿距离“上→右→下→左”找最近合法格；
//     保护期离开落点立即取消；增益/负面效果跨死亡保留；
//   * §8.5 天下无双：生成区域可由公告指定（fortress=0/1/2），不再写死中心；
//   * §8.7 辣椒：同 Tick 多条火线统一结算，交点免疫，按剩余 HP 比例分摊；
//   * §8.8 洁身自好：反伤击杀奖励给防御方，且不放大易伤倍率；
//   * §10.2 每次死亡各扣 60（用计数而非 bool）。
//
// v9 评测口径（2026-10-25）：
//   * 默认生产配置：depth=0、reach_contest=true、engage_gate=false；
//   * 默认随机出生 spawn=2（0=四角 1=四边中点）；rot=1 只建议配 spawn=0；
//   * 默认四车同配：A/B/C/D 的 Agent 都用生产基线；A/B 开关只叠加在 A；
//   * selfcheck=1 / repro=1 / csv=1 / strict=1 见 main。
//   * eventaware=1..4 是测试专用真值事件注入（绕过 LLM），生产不调用。
// 用法： match4 <局数> [seed0] [v] [noevent] [opp=...] [spawn=0/1/2] ...
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rmtc/agent.h"
#include "rmtc/config.h"
#include "rmtc/json.h"
#include "rmtc/protocol.h"
#include "rmtc/revive.h"

namespace {

const int W = 24, H = 24;
const int TICKS = 500;
const int VISION = 4;
const int NORMAL_N = 12, HIGH_N = 4;
const int NORMAL_V = 10, HIGH_V = 20;

// 事件类型
enum {
    EV_EXPLOSION = 0,
    EV_ILLUSION,
    EV_HEAL,
    EV_FORTRESS,
    EV_SPLIT,
    EV_PEPPER,
    EV_THORNS,
    EV_VULNERABLE,
    EV_KIND_N
};
const char* kEvName[EV_KIND_N] = {"explosion", "illusion", "heal", "fortress",
                                  "split",     "pepper",   "thorns", "vulnerable"};

// 测试专用：模拟器事件类型 → Agent 的结构化事件报告类型。
rmtc::EventKind toEventKind(int type) {
    switch (type) {
        case EV_EXPLOSION: return rmtc::EVK_EXPLOSION;
        case EV_ILLUSION: return rmtc::EVK_ILLUSION;
        case EV_HEAL: return rmtc::EVK_HEAL;
        case EV_FORTRESS: return rmtc::EVK_FORTRESS;
        case EV_SPLIT: return rmtc::EVK_SPLIT;
        case EV_PEPPER: return rmtc::EVK_PEPPER;
        case EV_THORNS: return rmtc::EVK_THORNS;
        case EV_VULNERABLE: return rmtc::EVK_VULNERABLE;
        default: return rmtc::EVK_UNKNOWN;
    }
}

struct Rng {
    unsigned long long s;
    explicit Rng(unsigned long long seed) : s(seed ? seed : 88172645463325252ULL) {}
    unsigned int next() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return static_cast<unsigned int>(s >> 16);
    }
    int range(int n) { return n <= 0 ? 0 : static_cast<int>(next() % static_cast<unsigned>(n)); }
    double unit() { return (next() & 0xFFFF) / 65536.0; }
};

struct Res {
    int x, y, value;
    bool taken;
};

struct Event {
    int type;
    int tAct;      // 激活 Tick
    int dur;       // 持续 Tick（一次性事件为 0）
    int x0, y0, x1, y1;  // 矩形/点/十字
    int n;         // 参数（回血 Tick 数等）
};

// 对手策略池（问题 6 + 建议书《对手策略池搭建》）：除 A 外可让 B/C/D 换成
// 脚本策略或降配 Agent，检验策略在异质对手下的鲁棒性。
enum {
    OPP_AGENT = 0,   // 四车都用完整的 Agent（默认，自博弈基线 = mirror）
    OPP_COLLECTOR,   // 纯采集：只捡资源，不主动打架
    OPP_RUSHER,      // 激进抢资源：射程内攻击，否则抢最近资源，近距离才追人
    OPP_CAMPER,      // 蹲守流：守中心，敌人进射程就打
    OPP_RANDOM,      // 随机流：随机移动/攻击/防御
    OPP_TACTICIAN,   // 战术流：算换血、躲射线、劣势撤退、无战事时采集
    OPP_FARMER,      // 农夫流：只在低血/近身时躲，否则持续采集（v5 farm 口径）
    OPP_MCAMP,       // 任务蹲守：完成自己的 reach 前守点，完成后转采集
    OPP_MAD,         // 疯狗流：80% 乱走（不避障）+ 20% 见敌就打
    OPP_PAST,        // 过去的自己：完整 Agent + 初稿配置（降维旧版）
    OPP_MIX,         // 策略池（旧口径）：每局抽一种模式，B/C/D 三家同策略
    OPP_MIX3,        // 策略池（真实口径）：B/C/D 每席位独立抽取，允许强 Agent 对手
    OPP_MIX1,        // 1 个同级 Agent + 抢匪 + 任务党（v9 高强度混合）
    OPP_MIXREAL,     // 农夫 + 抢匪 + 任务党（v9 真实混合，全脚本）
    // 2026-10-26 池子校准：按队友 match_het 的对手口径补三种风格，
    // 只做"同一对手、两套评测台"的交叉验证，不进 mix/mix3 池（保持历史口径可比）。
    OPP_HUNTER,      // 纯追击：不采集、不做任务，只咬最近的敌人
    OPP_HYBRID,      // 采集+追击混合：敌人≤4 格且（敌残血或我血厚）就转追击
    OPP_SHADOW,      // 影子蹭血（2026-10-28 新增）：只盯 A，贴同轴 1–2 格每 Tick 刮痧，
                     // 不抢资源、不做任务；专门测"经济门压攻击时被白打"的攻击面。
    OPP_MISSION      // 任务优先：先去 reach 点，之后转采集，从不主动攻击
};
const int kMixPool[] = {OPP_RUSHER, OPP_FARMER, OPP_MCAMP, OPP_MAD,
                        OPP_TACTICIAN, OPP_PAST};
const char* kMixName[] = {"rusher", "farmer", "mcamp", "mad", "tactician", "past"};
const double kMixWeight[] = {0.20, 0.15, 0.15, 0.15, 0.20, 0.15};

// mix3：真实赛场三家对手是异构的，且很可能有一家是和我们同级的 Agent。
// 旧 mix 把同一种策略复制给 B/C/D，会放大"三个农夫"这类极端局，且完全
// 排除了强 Agent 对手——两项都会让排名估计失真。这里按席位独立抽取。
// 权重是"对真实赛场的先验"：20% 概率每个席位是同级 Agent，其余按脚本分布。
const int kMix3Pool[] = {OPP_AGENT,  OPP_RUSHER,   OPP_FARMER, OPP_MCAMP,
                         OPP_MAD,    OPP_TACTICIAN, OPP_PAST};
const char* kMix3Name[] = {"agent", "rusher", "farmer", "mcamp",
                           "mad",   "tactician", "past"};
const double kMix3Weight[] = {0.20, 0.10, 0.15, 0.10, 0.10, 0.20, 0.15};

const char* oppName(int p) {
    switch (p) {
        case OPP_AGENT: return "agent";
        case OPP_COLLECTOR: return "collector";
        case OPP_RUSHER: return "rusher";
        case OPP_CAMPER: return "camper";
        case OPP_RANDOM: return "random";
        case OPP_TACTICIAN: return "tactician";
        case OPP_FARMER: return "farmer";
        case OPP_MCAMP: return "mcamp";
        case OPP_MAD: return "mad";
        case OPP_PAST: return "past";
        case OPP_MIX: return "mix";
        case OPP_MIX3: return "mix3";
        case OPP_MIX1: return "mix1";
        case OPP_MIXREAL: return "mixreal";
        case OPP_HUNTER: return "hunter";
        case OPP_HYBRID: return "hybrid";
        case OPP_SHADOW: return "shadow";
        case OPP_MISSION: return "mission";
    }
    return "?";
}

// 命令行名称 → 脚本编号（未识别返回 -1）。只认"单席位可用"的具体策略，
// 不认 mix/mix3/mix1/mixreal（那是整局抽签，不是某一席位的固定风格）。
int policyFromName(const std::string& s) {
    static const int kOk[] = {OPP_AGENT, OPP_COLLECTOR, OPP_RUSHER, OPP_CAMPER, OPP_RANDOM,
                              OPP_TACTICIAN, OPP_FARMER, OPP_MCAMP, OPP_MAD, OPP_PAST,
                              OPP_HUNTER, OPP_HYBRID, OPP_SHADOW, OPP_MISSION};
    for (std::size_t i = 0; i < sizeof(kOk) / sizeof(kOk[0]); ++i)
        if (s == oppName(kOk[i])) return kOk[i];
    return -1;
}

struct Bot {
    int team = 0;
    char id = 'A';
    int policy = OPP_AGENT;
    int defendStreak = 0;   // 连续防御次数（检测"无限防御"用）
    int maxStreak = 0;      // 本局最长连续防御
    int x = 0, y = 0, sx = 0, sy = 0;
    int hp = 100;
    bool alive = true;
    int respawnAt = -1;
    int protectUntil = -1;
    int protectX = -1, protectY = -1;  // 本次复活的落点（离开它才取消保护）
    bool defending = false;
    bool defendedLast = false;
    bool forcedWait = false;

    // 增益/负面
    bool wushuang = false;
    int wushuangUntil = -1;
    int dwell = 0;          // 堡垒连续驻留
    int vulnerableUntil = -1;
    int healUntil = -1;
    int healTicks = 0;

    int lastTick = -1;
    std::string lastAction;
    bool lastMoved = false;
    std::string lastDir;

    double r2 = 0, dmg = 0;
    int kills = 0, deaths = 0;
    int normalGot = 0, highGot = 0;
    bool reachDone = false, twoNormalDone = false, highDone = false;
    bool damageDone = false;   // v1.2 伤害任务（test-only：damagetask=N 开启）
    int damageTarget = 0;      // 0=本场无伤害任务
    int reachTick = -1;   // 首次踩到 reach 点的 Tick（未完成 = -1）

    std::unique_ptr<rmtc::Agent> agent;

    int missions() const {
        return (twoNormalDone ? 1 : 0) + (highDone ? 1 : 0) + (reachDone ? 1 : 0) +
               ((damageTarget > 0 && damageDone) ? 1 : 0);
    }
    double score() const {
        double r = 30.0 * (r2 / 120.0 > 1.0 ? 1.0 : r2 / 120.0);
        double d = 15.0 * (dmg / 500.0 > 1.0 ? 1.0 : dmg / 500.0);
        return r + d + 5.0 * missions();
    }
};

struct Match {
    Rng rng;
    unsigned long long seed_;
    unsigned char blocked[W * H];
    int wallUntil[W * H];   // 分裂十字墙：>tick 表示该格是墙
    std::vector<Res> res;
    std::vector<Event> events;
    int missionX, missionY;
    int spawnMode_;         // 0=四角 1=四边中点 2=随机空格
    int spawns_[4][2];
    bool corridorMap_;
    bool eventsOn;
    bool fairMap_;          // 资源/障碍四象限镜像（模拟“资源分配公平”的正式图）
    int fortressMode_;      // 0=中心（历史口径） 1=偏移 2=随机合法 3×3（v1.2 特殊图）
    int damageTaskTarget_;  // test-only：>0 时本场公布伤害任务（有效伤害阈值）

    Match(unsigned long long seed, bool ev, bool corridorMap, int spawnMode,
          int fortressMode = 0, int damageTaskTarget = 0, bool fairMap = false)
        : rng(seed), seed_(seed), missionX(0), missionY(0), spawnMode_(spawnMode),
          corridorMap_(corridorMap), eventsOn(ev), fairMap_(fairMap),
          fortressMode_(fortressMode), damageTaskTarget_(damageTaskTarget) {
        std::memset(blocked, 0, sizeof(blocked));
        std::memset(wallUntil, 0, sizeof(wallUntil));
        const int corners[4][2] = {{1, 1}, {W - 2, 1}, {1, H - 2}, {W - 2, H - 2}};
        for (int i = 0; i < 4; ++i) {
            spawns_[i][0] = corners[i][0];
            spawns_[i][1] = corners[i][1];
        }
        build(corridorMap);
        chooseSpawns();
        if (eventsOn) buildEvents(fortressMode);
    }

    int idx(int x, int y) const { return y * W + x; }
    bool inb(int x, int y) const { return x >= 0 && x < W && y >= 0 && y < H; }
    bool wall(int x, int y) const { return !inb(x, y) || blocked[idx(x, y)] != 0; }
    bool wallAt(int x, int y, int tick) const {
        if (!inb(x, y)) return true;
        return blocked[idx(x, y)] != 0 || wallUntil[idx(x, y)] > tick;
    }
    int spawnX(int i) const { return spawns_[i & 3][0]; }
    int spawnY(int i) const { return spawns_[i & 3][1]; }

    // 出生点选择必须与地图生成解耦：不能消耗 rng，否则 spawn=0/2 会得到
    // 不同的障碍/资源图，固定角与随机出生就无法做配对比较。
    // 随机出生用独立 Rng(seed^常量)，且只挑空格、非资源、非任务点。
    bool spawnCandidateOk(int x, int y) const {
        if (wall(x, y)) return false;
        if (x == missionX && y == missionY) return false;
        for (std::size_t i = 0; i < res.size(); ++i)
            if (!res[i].taken && res[i].x == x && res[i].y == y) return false;
        int freeN = 0;
        if (inb(x + 1, y) && !wall(x + 1, y)) ++freeN;
        if (inb(x - 1, y) && !wall(x - 1, y)) ++freeN;
        if (inb(x, y + 1) && !wall(x, y + 1)) ++freeN;
        if (inb(x, y - 1) && !wall(x, y - 1)) ++freeN;
        return freeN >= 2;
    }

    // 在目标点附近找一个合法出生格（四边中点模式用；随机图上边缘可能是墙）。
    void nearestFreeSpawn(int& x, int& y) const {
        for (int r = 0; r <= 6; ++r) {
            for (int dy = -r; dy <= r; ++dy)
                for (int dx = -r; dx <= r; ++dx) {
                    if (std::abs(dx) + std::abs(dy) > r) continue;
                    const int nx = x + dx, ny = y + dy;
                    if (spawnCandidateOk(nx, ny)) { x = nx; y = ny; return; }
                }
        }
    }

    void chooseSpawns() {
        if (corridorMap_ || spawnMode_ == 0) return;  // 四角（建图时已清空 3×3）
        if (spawnMode_ == 1) {
            const int edge[4][2] = {{W / 2, 1}, {W - 2, H / 2}, {W / 2, H - 2}, {1, H / 2}};
            for (int i = 0; i < 4; ++i) {
                int x = edge[i][0], y = edge[i][1];
                nearestFreeSpawn(x, y);
                spawns_[i][0] = x;
                spawns_[i][1] = y;
            }
            return;
        }
        // 随机空格：独立 RNG，四家互异；优先拉开距离，避免开局挤在一起。
        Rng srng(seed_ ^ 0x9E3779B97F4A7C15ULL);
        for (int i = 0; i < 4; ++i) {
            int bx = -1, by = -1;
            for (int guard = 0; guard < 4000; ++guard) {
                const int x = 1 + srng.range(W - 2);
                const int y = 1 + srng.range(H - 2);
                if (!spawnCandidateOk(x, y)) continue;
                bool dup = false;
                for (int k = 0; k < i; ++k) {
                    if (spawns_[k][0] == x && spawns_[k][1] == y) dup = true;
                    if (std::abs(spawns_[k][0] - x) + std::abs(spawns_[k][1] - y) < 4)
                        dup = true;
                }
                if (dup) continue;
                bx = x; by = y; break;
            }
            if (bx < 0) {
                for (int y = 1; y < H - 1 && bx < 0; ++y)
                    for (int x = 1; x < W - 1; ++x) {
                        if (!spawnCandidateOk(x, y)) continue;
                        bool dup = false;
                        for (int k = 0; k < i; ++k)
                            if (spawns_[k][0] == x && spawns_[k][1] == y) dup = true;
                        if (!dup) { bx = x; by = y; break; }
                    }
            }
            if (bx >= 0) { spawns_[i][0] = bx; spawns_[i][1] = by; }
        }
    }

    // corridorMap=true：象限地图——四个 10×10 开阔象限，仅由单格走廊相连。
    // 这是建议书《地形问题改进》§5 要求的"确定性走廊场景"：
    // 每个象限内咽喉价值 ≈ 整个其它象限的规模（≥8），避死/绕行才有发挥空间。
    void buildCorridor() {
        // 单列/单行墙（双列墙会让"开门"变成双格通道，门内两格互为冗余，
        // 整张图反而没有割点——这是实测踩过的坑）：
        //   竖墙 x=11 全高；横墙 y=11 全宽
        //   三条单格门：上 (11,5)、下 (11,18)、左 (5,11)
        //   → 每个门都是割点，删除后两侧象限断开，咽喉价值 ≈ 对侧象限规模。
        for (int y = 0; y < H; ++y) blocked[idx(11, y)] = 1;
        for (int x = 0; x < W; ++x) blocked[idx(x, 11)] = 1;
        blocked[idx(11, 5)] = 0;
        blocked[idx(11, 18)] = 0;
        blocked[idx(5, 11)] = 0;
        // 资源：每象限 2 普通 + 1 高价值
        const int qx[2] = {5, 18};
        const int qy[2] = {5, 18};
        int placed = 0;
        for (int q = 0; q < 4 && placed < NORMAL_N; ++q) {
            const int cx = qx[q % 2], cy = qy[q / 2];
            for (int k = 0; k < 2 && placed < NORMAL_N; ++k) {
                const int x = cx + (k == 0 ? -2 : 2), y = cy + (k == 0 ? -1 : 1);
                if (!wall(x, y)) {
                    Res r; r.x = x; r.y = y; r.value = NORMAL_V; r.taken = false;
                    res.push_back(r); ++placed;
                }
            }
        }
        for (int q = 0; q < 4 && placed < NORMAL_N + HIGH_N; ++q) {
            const int cx = qx[q % 2], cy = qy[q / 2];
            const int x = cx, y = cy + (q < 2 ? -3 : 3);
            if (!wall(x, y)) {
                Res r; r.x = x; r.y = y; r.value = HIGH_V; r.taken = false;
                res.push_back(r); ++placed;
            }
        }
        // 任务点放在对角象限中心（必然跨走廊）
        missionX = 18; missionY = 18;
    }

    // 公平图：障碍与资源做四象限镜像，四家出生点到资源的结构完全一致。
    // 用于回答“正式比赛保证资源公平后是否改善”，不是正式赛地图生成器。
    void buildFair() {
        // 只生成左上象限，再镜像到右上/左下/右下。
        for (int y = 0; y < H / 2; ++y)
            for (int x = 0; x < W / 2; ++x) {
                if (rng.unit() >= 0.08) continue;
                const int xs[4] = {x, W - 1 - x, x, W - 1 - x};
                const int ys[4] = {y, y, H - 1 - y, H - 1 - y};
                for (int k = 0; k < 4; ++k)
                    if (inb(xs[k], ys[k])) blocked[idx(xs[k], ys[k])] = 1;
            }
        // 四角出生区清空，保证 spawn=0 可用。
        for (int k = 0; k < 4; ++k)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    const int x = spawns_[k][0] + dx, y = spawns_[k][1] + dy;
                    if (inb(x, y)) blocked[idx(x, y)] = 0;
                }

        // 左上象限放 3 普通 + 1 高价值，镜像后恰好 12 普通 + 4 高价值。
        struct Base { int x, y, v; };
        Base base[4];
        int made = 0, guard = 0;
        while (made < 4 && guard++ < 5000) {
            const int x = 1 + rng.range(W / 2 - 2);
            const int y = 1 + rng.range(H / 2 - 2);
            if (wall(x, y)) continue;
            bool bad = false;
            for (int k = 0; k < 4; ++k)
                if (x == spawns_[k][0] && y == spawns_[k][1]) bad = true;
            for (int k = 0; k < made; ++k)
                if (base[k].x == x && base[k].y == y) bad = true;
            if (bad) continue;
            base[made].x = x;
            base[made].y = y;
            base[made].v = (made < 3) ? NORMAL_V : HIGH_V;
            ++made;
        }
        for (int k = 0; k < made; ++k) {
            const int xs[4] = {base[k].x, W - 1 - base[k].x, base[k].x,
                               W - 1 - base[k].x};
            const int ys[4] = {base[k].y, base[k].y, H - 1 - base[k].y,
                               H - 1 - base[k].y};
            for (int q = 0; q < 4; ++q) {
                if (!inb(xs[q], ys[q])) continue;
                // 资源格必须是可站的；镜像落在墙上时清掉该墙，保持资源总量 200。
                if (wall(xs[q], ys[q])) blocked[idx(xs[q], ys[q])] = 0;
                Res r;
                r.x = xs[q];
                r.y = ys[q];
                r.value = base[k].v;
                r.taken = false;
                res.push_back(r);
            }
        }
        // 任务点放正中；中心若是墙，找最近的合法格。
        missionX = W / 2;
        missionY = H / 2;
        if (wall(missionX, missionY)) nearestFreeSpawn(missionX, missionY);
    }

    void build(bool corridorMap) {
        if (corridorMap) { buildCorridor(); return; }
        if (fairMap_) { buildFair(); return; }
        for (int i = 0; i < W * H; ++i)
            if (rng.unit() < 0.08) blocked[i] = 1;
        // 固定四角始终清空 3×3，保证 spawn=0 与随机出生共享同一张图；
        // 随机出生只换车位，不再改动障碍/资源。
        for (int k = 0; k < 4; ++k)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    const int x = spawns_[k][0] + dx, y = spawns_[k][1] + dy;
                    if (inb(x, y)) blocked[idx(x, y)] = 0;
                }
        int placed = 0, guard = 0;
        while (placed < NORMAL_N + HIGH_N && guard++ < 5000) {
            const int x = rng.range(W), y = rng.range(H);
            if (wall(x, y)) continue;
            bool bad = false;
            for (int k = 0; k < 4; ++k)
                if (x == spawns_[k][0] && y == spawns_[k][1]) bad = true;
            for (std::size_t i = 0; i < res.size(); ++i)
                if (res[i].x == x && res[i].y == y) bad = true;
            if (bad) continue;
            Res r;
            r.x = x; r.y = y;
            r.value = (placed < NORMAL_N) ? NORMAL_V : HIGH_V;
            r.taken = false;
            res.push_back(r);
            ++placed;
        }
        for (int guard2 = 0; guard2 < 2000; ++guard2) {
            const int x = rng.range(W), y = rng.range(H);
            if (!wall(x, y)) { missionX = x; missionY = y; break; }
        }
    }

    // v1.2 §8.5：天下无双的生成区域“多数为 (11,11)-(13,13)，但部分地图存在
    // 特殊变化，以赛时自然语言公告与话题为准”。模拟器不能永远只测中心：
    //   mode 0 = 中心（历史口径，保持 A/B 可比）
    //   mode 1 = 固定偏移（确定性地验证“非中心”路径）
    //   mode 2 = 随机合法 3×3（高压力测试）
    void fortressRect(int mode, int& x0, int& y0) const {
        if (mode == 0) {
            x0 = W / 2 - 1;
            y0 = H / 2 - 1;
            return;
        }
        if (mode == 1) {
            x0 = W / 2 + 3;
            y0 = H / 2 - 3;
            if (x0 + 2 >= W) x0 = W - 3;
            if (y0 < 0) y0 = 0;
            return;
        }
        // 独立 RNG：不能消耗主 rng，否则同一 seed 的障碍/资源/其它事件会变，
        // 中心模式与特殊模式就无法配对比较。
        Rng frng(seed_ ^ 0xD1B54A32D192ED03ULL);
        for (int guard = 0; guard < 200; ++guard) {
            const int x = 1 + frng.range(W - 3);
            const int y = 1 + frng.range(H - 3);
            bool allFree = true;
            for (int dy = 0; dy < 3 && allFree; ++dy)
                for (int dx = 0; dx < 3; ++dx)
                    if (wall(x + dx, y + dy)) { allFree = false; break; }
            if (allFree) {
                x0 = x;
                y0 = y;
                return;
            }
        }
        x0 = W / 2 - 1;
        y0 = H / 2 - 1;
    }

    // 4 个事件，类型随机、时间随机（激活 Tick 分布在 80–420）、位置随机
    void buildEvents(int fortressMode) {
        const int kindOrder[EV_KIND_N] = {EV_EXPLOSION, EV_ILLUSION, EV_HEAL, EV_FORTRESS,
                                          EV_SPLIT,     EV_PEPPER,   EV_THORNS, EV_VULNERABLE};
        int used[EV_KIND_N] = {0};
        int made = 0, guard = 0;
        while (made < 4 && guard++ < 200) {
            int k = rng.range(EV_KIND_N);
            if (used[k]) continue;
            used[k] = 1;
            Event e;
            e.type = kindOrder[k];
            e.tAct = 80 + rng.range(340);
            e.dur = 0;
            e.n = 0;
            // 随机矩形（小于全图）
            int w = 3 + rng.range(5), h = 3 + rng.range(5);
            int x0 = rng.range(W - w), y0 = rng.range(H - h);
            e.x0 = x0; e.y0 = y0; e.x1 = x0 + w - 1; e.y1 = y0 + h - 1;
            switch (e.type) {
                case EV_EXPLOSION: e.dur = 0; break;
                case EV_ILLUSION: e.dur = 50; break;
                case EV_HEAL: e.dur = 20; e.x1 = e.x0; e.y1 = e.y0; break;
                case EV_FORTRESS: {
                    e.dur = 50;
                    int fx = 0, fy = 0;
                    fortressRect(fortressMode, fx, fy);
                    e.x0 = fx; e.y0 = fy;
                    e.x1 = fx + 2; e.y1 = fy + 2;
                    break;
                }
                case EV_SPLIT: e.dur = 30; e.x1 = e.x0; e.y1 = e.y0; break;  // 用 x0 作十字线
                case EV_PEPPER: e.dur = 30; e.x1 = e.x0; e.y1 = e.y0; break;
                case EV_THORNS: e.dur = 20; break;
                case EV_VULNERABLE: e.dur = 30; break;
                default: break;
            }
            events.push_back(e);
            ++made;
        }
    }

    // 高压力机制测试：8 类事件都出现一次，且时间/位置可控。
    // 这不是正式赛程，只用于证明 v1.2 的结算分支确实会被触发。
    void buildDenseEvents() {
        events.clear();
        int fx = 0, fy = 0;
        fortressRect(fortressMode_, fx, fy);
        Event e;
        e = Event(); e.type = EV_EXPLOSION; e.tAct = 80; e.dur = 1;
        e.x0 = 9; e.y0 = 9; e.x1 = 14; e.y1 = 14; events.push_back(e);
        e = Event(); e.type = EV_ILLUSION; e.tAct = 120; e.dur = 50;
        e.x0 = 6; e.y0 = 6; e.x1 = 17; e.y1 = 17; events.push_back(e);
        e = Event(); e.type = EV_HEAL; e.tAct = 160; e.dur = 20;
        e.x0 = 12; e.y0 = 12; e.x1 = 12; e.y1 = 12; events.push_back(e);
        e = Event(); e.type = EV_FORTRESS; e.tAct = 200; e.dur = 50;
        e.x0 = fx; e.y0 = fy; e.x1 = fx + 2; e.y1 = fy + 2; events.push_back(e);
        e = Event(); e.type = EV_SPLIT; e.tAct = 260; e.dur = 30;
        e.x0 = 12; e.y0 = 12; e.x1 = 12; e.y1 = 12; events.push_back(e);
        e = Event(); e.type = EV_PEPPER; e.tAct = 300; e.dur = 30;
        e.x0 = 12; e.y0 = 12; e.x1 = 12; e.y1 = 12; events.push_back(e);
        e = Event(); e.type = EV_THORNS; e.tAct = 350; e.dur = 20;
        e.x0 = 0; e.y0 = 0; e.x1 = W - 1; e.y1 = H - 1; events.push_back(e);
        e = Event(); e.type = EV_VULNERABLE; e.tAct = 400; e.dur = 30;
        e.x0 = 0; e.y0 = 0; e.x1 = W - 1; e.y1 = H - 1; events.push_back(e);
    }

    bool resourceAt(int x, int y, int& value) const {
        for (std::size_t i = 0; i < res.size(); ++i)
            if (!res[i].taken && res[i].x == x && res[i].y == y) { value = res[i].value; return true; }
        return false;
    }
    void takeAt(int x, int y) {
        for (std::size_t i = 0; i < res.size(); ++i)
            if (!res[i].taken && res[i].x == x && res[i].y == y) { res[i].taken = true; return; }
    }
};

std::string jsonStr(const std::string& s) {
    std::string o = "\"";
    for (std::size_t i = 0; i < s.size(); ++i) {
        const char c = s[i];
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (c == '\n') o += "\\n";
        else if (c == '\r') o += "\\r";
        else if (c == '\t') o += "\\t";
        else o += c;
    }
    o += "\"";
    return o;
}

std::string grid9(int dx, int dy) {
    std::string out;
    const int tx = 4 + dx, ty = 4 + dy;
    for (int r = 0; r < 9; ++r) {
        for (int c = 0; c < 9; ++c) {
            char ch = '0';
            if (r == 4 && c == 4) ch = '1';
            // 协议 §3：实体与自身重叠时中心保留 '1'，不能覆盖成 '2'。
            else if (r == ty && c == tx) ch = '2';
            out += ch;
        }
        if (r < 8) out += '\n';
    }
    return out;
}

std::string buildPerception(const Match& m, const std::vector<Bot>& bots, int self,
                            int tick, const std::string& session) {
    const Bot& me = bots[self];
    std::string s = "{";
    s += "\"tick\":" + std::to_string(tick);
    s += ",\"session_id\":" + jsonStr(session);
    s += ",\"vision_radius\":" + std::to_string(VISION);
    s += ",\"self\":{\"team\":" + jsonStr(std::string(1, me.id)) +
         ",\"position\":[" + std::to_string(me.x) + "," + std::to_string(me.y) + "]" +
         ",\"hp\":" + std::to_string(me.alive ? me.hp : 0) +
         ",\"alive\":" + (me.alive ? "true" : "false") +
         ",\"defending\":" + (me.defendedLast ? "true" : "false") +
         ",\"protected\":" + (tick <= me.protectUntil ? "true" : "false") +
         ",\"respawn_in\":" + std::to_string(me.alive ? 0 : (me.respawnAt - tick)) +
         ",\"robot_id\":" + jsonStr(std::string(1, me.id)) + ",\"last_action\":";
    if (me.lastTick < 0 || me.lastAction.empty()) {
        s += "null";
    } else {
        s += "{\"tick\":" + std::to_string(me.lastTick) + ",\"action\":" +
             jsonStr(me.lastAction) + ",\"moved\":" + (me.lastMoved ? "true" : "false");
        if (me.lastMoved && !me.lastDir.empty())
            s += ",\"effective_direction\":" + jsonStr(me.lastDir);
        s += "}";
    }
    s += "}";

    s += ",\"visible_entities\":[";
    bool first = true;
    if (me.alive) {
        for (int t = 0; t < 4; ++t) {
            if (t == self || !bots[t].alive) continue;
            const int dx = bots[t].x - me.x, dy = bots[t].y - me.y;
            if (dx < -VISION || dx > VISION || dy < -VISION || dy > VISION) continue;
            if (!first) s += ",";
            first = false;
            s += "{\"type\":\"robot\",\"id\":" + jsonStr(std::string("robot_") + bots[t].id) +
                 ",\"team\":" + jsonStr(std::string(1, bots[t].id)) +
                 ",\"robot_id\":" + jsonStr(std::string(1, bots[t].id)) +
                 ",\"position\":" + jsonStr(grid9(dx, dy)) +
                 ",\"hp\":" + std::to_string(bots[t].hp) +
                 ",\"defending\":" + (bots[t].defendedLast ? "true" : "false") +
                 ",\"protected\":" + (tick <= bots[t].protectUntil ? "true" : "false") + "}";
        }
        for (std::size_t i = 0; i < m.res.size(); ++i) {
            const Res& r = m.res[i];
            if (r.taken) continue;
            const int dx = r.x - me.x, dy = r.y - me.y;
            if (dx < -VISION || dx > VISION || dy < -VISION || dy > VISION) continue;
            if (!first) s += ",";
            first = false;
            s += "{\"type\":\"" + std::string(r.value >= 20 ? "resource_high" : "resource_normal") +
                 "\",\"id\":\"r\",\"position\":" + jsonStr(grid9(dx, dy)) +
                 ",\"value\":" + std::to_string(r.value) + "}";
        }
        for (int dy = -VISION; dy <= VISION; ++dy)
            for (int dx = -VISION; dx <= VISION; ++dx) {
                const int x = me.x + dx, y = me.y + dy;
                if (!m.inb(x, y) || !m.wallAt(x, y, tick)) continue;
                if (!first) s += ",";
                first = false;
                s += "{\"type\":\"obstacle\",\"position\":" + jsonStr(grid9(dx, dy)) + "}";
            }
    }
    s += "]";

    char score[192];
    std::snprintf(score, sizeof(score),
                  ",\"score\":{\"resource_value\":%.1f,\"effective_damage\":%.1f,"
                  "\"mission_completed\":%d,\"mission_completed_ids\":[",
                  me.r2, me.dmg, me.missions());
    s += score;
    // 已完成任务 ID 必须逐条回填（协议 §3：在 score 对象内部）：
    // 否则 Agent 会以为 reach 还没做，完成后仍反复往任务点跑。
    bool firstM = true;
    if (me.twoNormalDone) { s += "\"collect_two_normal\""; firstM = false; }
    if (me.highDone) { if (!firstM) s += ","; s += "\"collect_high_value\""; firstM = false; }
    if (me.reachDone) { if (!firstM) s += ","; s += "\"reach_target\""; firstM = false; }
    if (me.damageTarget > 0 && me.damageDone) {
        if (!firstM) s += ",";
        s += "\"damage_task\"";
    }
    s += "],\"invalid_actions\":0}";
    s += ",\"mission_text\":\"\"";
    s += ",\"missions\":[";
    s += "{\"id\":\"collect_two_normal\",\"type\":\"normal_resources\",\"target\":2},";
    s += "{\"id\":\"collect_high_value\",\"type\":\"high_resources\",\"target\":1},";
    s += "{\"id\":\"reach_target\",\"type\":\"reach\",\"target\":1,\"position\":[" +
         std::to_string(m.missionX) + "," + std::to_string(m.missionY) + "]}";
    if (me.damageTarget > 0) {
        s += ",{\"id\":\"damage_task\",\"type\":\"effective_damage\",\"target\":" +
             std::to_string(me.damageTarget) + "}";
    }
    s += "]";
    s += ",\"map\":{\"width\":" + std::to_string(W) + ",\"height\":" + std::to_string(H) + "}";
    s += ",\"event_text\":\"\"}";  // 本模拟器不给自然语言公告（无 LLM 时无法合规解析）
    return s;
}

struct Act {
    std::string action, dir;
    bool ok = false;
};

std::string fieldOf(const std::string& j, const char* key) {
    const std::string pat = std::string("\"") + key + "\":\"";
    const std::size_t p = j.find(pat);
    if (p == std::string::npos) return std::string();
    const std::size_t b = p + pat.size();
    const std::size_t e = j.find('"', b);
    if (e == std::string::npos) return std::string();
    return j.substr(b, e - b);
}

Act parseAction(const std::string& j) {
    Act a;
    a.action = fieldOf(j, "action");
    a.dir = fieldOf(j, "direction");
    a.ok = a.action == "move" || a.action == "attack" || a.action == "defend" ||
           a.action == "wait";
    if (a.action == "move" || a.action == "attack")
        if (a.dir != "up" && a.dir != "down" && a.dir != "left" && a.dir != "right") a.ok = false;
    return a;
}

void delta(const std::string& d, int& dx, int& dy) {
    dx = dy = 0;
    if (d == "up") dy = -1;
    else if (d == "down") dy = 1;
    else if (d == "left") dx = -1;
    else if (d == "right") dx = 1;
}

std::string flip(const std::string& d) {
    if (d == "up") return "down";
    if (d == "down") return "up";
    if (d == "left") return "right";
    if (d == "right") return "left";
    return d;
}

// ---- 脚本对手用的最短路（BFS）----
// 返回从 (x,y) 朝目标 (tx,ty) 走的第一步方向；不可达返回空串。
// 只考虑静态墙、当前临时墙、存活车占位（与裁判的移动合法性一致）。
std::string stepToward(const Match& m, const std::vector<Bot>& bots, int self,
                       int tx, int ty, int tick) {
    const Bot& me = bots[self];
    if (me.x == tx && me.y == ty) return std::string();
    if (!m.inb(tx, ty)) return std::string();
    std::vector<int> parent(W * H, -2);
    std::vector<int> queue;
    queue.reserve(W * H);
    const int start = m.idx(me.x, me.y);
    parent[start] = -1;
    queue.push_back(start);
    const int goal = m.idx(tx, ty);
    for (std::size_t qi = 0; qi < queue.size(); ++qi) {
        const int cur = queue[qi];
        if (cur == goal) break;
        const int cx = cur % W, cy = cur / W;
        const int nb[4][2] = {{cx + 1, cy}, {cx - 1, cy}, {cx, cy + 1}, {cx, cy - 1}};
        for (int k = 0; k < 4; ++k) {
            const int nx = nb[k][0], ny = nb[k][1];
            if (!m.inb(nx, ny)) continue;
            if (m.wallAt(nx, ny, tick)) continue;
            const int ni = m.idx(nx, ny);
            if (parent[ni] != -2) continue;
            bool occupied = false;
            for (int t = 0; t < 4; ++t) {
                if (t == self || !bots[t].alive) continue;
                if (bots[t].x == nx && bots[t].y == ny) occupied = true;
            }
            // 目标格允许"到达"（追击时目标就是敌人所在格），但不可作为中转。
            // 修 2026-10-26：旧版连目标格也排除，导致追求敌人的脚本永远 BFS
            // 失败 → 原地 wait，追逐型对手被系统性削弱（hunter/rusher/hybrid 都受影响）。
            if (occupied && ni != goal) continue;
            parent[ni] = cur;
            queue.push_back(ni);
        }
    }
    if (parent[goal] == -2) return std::string();
    int cur = goal;
    while (parent[cur] != start && parent[cur] >= 0) cur = parent[cur];
    const int nx = cur % W, ny = cur / W;
    if (nx == me.x + 1 && ny == me.y) return "right";
    if (nx == me.x - 1 && ny == me.y) return "left";
    if (nx == me.x && ny == me.y + 1) return "down";
    if (nx == me.x && ny == me.y - 1) return "up";
    return std::string();
}

// 攻击方向：若敌人在同行/同列的 1–2 格内，返回朝它的方向；否则空串。
std::string attackDirTo(const Bot& me, const Bot& target) {
    if (me.x != target.x && me.y != target.y) return std::string();
    const int dx = target.x - me.x, dy = target.y - me.y;
    if (dx == 0 && dy == 0) return std::string();
    if (dx != 0 && dy != 0) return std::string();
    if (dx < -2 || dx > 2 || dy < -2 || dy > 2) return std::string();
    if (dx > 0) return "right";
    if (dx < 0) return "left";
    if (dy > 0) return "down";
    return "up";
}

// 最近的未取资源；没有返回 -1。脚本对手仍然用全局信息（全知），
// 这是测试台的已知偏差，但在两边脚本对比时至少规则一致。
int nearestResourceIndex(const Match& m, const Bot& me) {
    int best = -1, bestD = 999;
    for (std::size_t i = 0; i < m.res.size(); ++i) {
        if (m.res[i].taken) continue;
        const int d = std::abs(m.res[i].x - me.x) + std::abs(m.res[i].y - me.y);
        if (d < bestD) { bestD = d; best = static_cast<int>(i); }
    }
    return best;
}

// 脚本对手的动作生成。刻意写得"像人"：追击不看血量交换、采集不还手。
std::string scriptedAction(const Match& m, const std::vector<Bot>& bots, int self,
                           int tick, unsigned long long seed) {
    const Bot& me = bots[self];
    if (!me.alive) return "wait";
    // 最近的存活敌人
    int near = -1, nearD = 999;
    for (int t = 0; t < 4; ++t) {
        if (t == self || !bots[t].alive) continue;
        const int d = std::abs(bots[t].x - me.x) + std::abs(bots[t].y - me.y);
        if (d < nearD) { nearD = d; near = t; }
    }
    switch (me.policy) {
        case OPP_COLLECTOR: {
            int best = -1, bestD = 999;
            for (std::size_t i = 0; i < m.res.size(); ++i) {
                if (m.res[i].taken) continue;
                const int d = std::abs(m.res[i].x - me.x) + std::abs(m.res[i].y - me.y);
                if (d < bestD) { bestD = d; best = static_cast<int>(i); }
            }
            if (best < 0) return "wait";
            const std::string dir = stepToward(m, bots, self, m.res[best].x, m.res[best].y, tick);
            return dir.empty() ? "wait" : "move " + dir;
        }
        case OPP_RUSHER: {
            // 抢资源型激进对手：有射程先打；敌人贴近且血厚才继续追；
            // 否则去最近资源。旧版只追人、不采集，A 可以靠资源白赢。
            if (near >= 0) {
                const std::string ad = attackDirTo(me, bots[near]);
                if (!ad.empty() && tick > me.protectUntil) return "attack " + ad;
                if (nearD <= 4 && me.hp > 50) {
                    const std::string dir =
                        stepToward(m, bots, self, bots[near].x, bots[near].y, tick);
                    if (!dir.empty()) return "move " + dir;
                }
            }
            const int best = nearestResourceIndex(m, me);
            if (best >= 0) {
                const std::string dir =
                    stepToward(m, bots, self, m.res[best].x, m.res[best].y, tick);
                if (!dir.empty()) return "move " + dir;
            }
            if (near >= 0) {
                const std::string dir =
                    stepToward(m, bots, self, bots[near].x, bots[near].y, tick);
                if (!dir.empty()) return "move " + dir;
            }
            return "wait";
        }
        case OPP_FARMER: {
            // 旧版满血也见人就跑，经济和路线都太弱。v5 farm 的口径是
            // 只在低血（hp≤40）且敌人≤3 格时撤，否则继续抢最近资源。
            if (near >= 0 && nearD <= 3 && me.hp <= 40) {
                const int awayX = me.x < bots[near].x ? 0 : (W - 1);
                const int awayY = me.y < bots[near].y ? 0 : (H - 1);
                const std::string dir = stepToward(m, bots, self, awayX, awayY, tick);
                if (!dir.empty()) return "move " + dir;
            }
            const int best = nearestResourceIndex(m, me);
            if (best < 0) return "wait";
            const std::string dir =
                stepToward(m, bots, self, m.res[best].x, m.res[best].y, tick);
            return dir.empty() ? "wait" : "move " + dir;
        }
        case OPP_MCAMP: {
            // 任务蹲守：完成自己的 reach 前守点；完成之后转采集。
            // 旧版永远站在任务点，三个 mcamp 会把 A 的 reach 长期压到 74.5%，
            // 这是脚本永久占位 artifact，不是真实任务对手。
            if (me.reachDone) {
                const int best = nearestResourceIndex(m, me);
                if (best >= 0) {
                    const std::string dir =
                        stepToward(m, bots, self, m.res[best].x, m.res[best].y, tick);
                    if (!dir.empty()) return "move " + dir;
                }
                return "wait";
            }
            if (near >= 0) {
                const std::string ad = attackDirTo(me, bots[near]);
                if (!ad.empty() && tick > me.protectUntil) return "attack " + ad;
            }
            if (me.x == m.missionX && me.y == m.missionY) return "wait";
            const std::string dir = stepToward(m, bots, self, m.missionX, m.missionY, tick);
            return dir.empty() ? "wait" : "move " + dir;
        }
        case OPP_HUNTER: {
            // 纯追击（队友 match_het 口径）：不采集、不做任务，目标恒为最近敌人。
            // 它几乎不拿资源，作用是"高压威胁"，不是分数竞争者。
            if (near >= 0) {
                const std::string ad = attackDirTo(me, bots[near]);
                if (!ad.empty() && tick > me.protectUntil) return "attack " + ad;
                const std::string dir =
                    stepToward(m, bots, self, bots[near].x, bots[near].y, tick);
                if (!dir.empty()) return "move " + dir;
            }
            return "wait";
        }
        case OPP_HYBRID: {
            // 采集 + 追人混合（最接近真实赛场）：射程内打；敌人≤4 格且
            // （敌血≤60 或我血≥70）时转追击；否则继续抢最近资源。
            if (near >= 0) {
                const std::string ad = attackDirTo(me, bots[near]);
                if (!ad.empty() && tick > me.protectUntil) return "attack " + ad;
                const bool worth = (nearD <= 4) && (bots[near].hp <= 60 || me.hp >= 70);
                if (worth) {
                    const std::string dir =
                        stepToward(m, bots, self, bots[near].x, bots[near].y, tick);
                    if (!dir.empty()) return "move " + dir;
                }
            }
            const int best = nearestResourceIndex(m, me);
            if (best >= 0) {
                const std::string dir =
                    stepToward(m, bots, self, m.res[best].x, m.res[best].y, tick);
                if (!dir.empty()) return "move " + dir;
            }
            return "wait";
        }
        case OPP_SHADOW: {
            // 影子蹭血：只盯 A（席位 0），目标不是击杀而是"贴同轴 1–2 格每 Tick 打 20"。
            // 它不抢资源、不做任务，专门检验经济门在承伤时是否还压着攻击。
            const Bot& prey = bots[0];
            if (!prey.alive) return "wait";
            const std::string ad = attackDirTo(me, prey);
            if (!ad.empty() && tick > me.protectUntil) return "attack " + ad;
            const int dx = prey.x - me.x, dy = prey.y - me.y;
            const int man = std::abs(dx) + std::abs(dy);
            // 已同轴 1–2 格但打不了（多半是对方出生保护）：原地卡位，别让开。
            if (man <= 2 && (dx == 0 || dy == 0)) return "wait";
            // 斜角贴脸：优先走一步到"同轴 1–2 格"的攻击位，而不是踩进对方格。
            if (man <= 2) {
                static const int sdx[4] = {1, -1, 0, 0};
                static const int sdy[4] = {0, 0, 1, -1};
                for (int k = 0; k < 4; ++k) {
                    const int nx = me.x + sdx[k], ny = me.y + sdy[k];
                    if (!m.inb(nx, ny) || m.wallAt(nx, ny, tick)) continue;
                    bool occupied = false;
                    for (int t = 0; t < 4; ++t)
                        if (t != self && bots[t].alive && bots[t].x == nx && bots[t].y == ny)
                            occupied = true;
                    if (occupied) continue;
                    const int nd = std::abs(nx - prey.x) + std::abs(ny - prey.y);
                    if ((nx == prey.x || ny == prey.y) && nd >= 1 && nd <= 2)
                        return std::string("move ") + (sdx[k] > 0 ? "right" : sdx[k] < 0 ? "left" :
                                                       sdy[k] > 0 ? "down" : "up");
                }
            }
            const std::string dir = stepToward(m, bots, self, prey.x, prey.y, tick);
            return dir.empty() ? "wait" : "move " + dir;
        }
        case OPP_MISSION: {
            // 任务优先：先到 reach 点，之后转采集；从不主动攻击。
            // 与 mcamp 的区别：mcamp 完成自己的 reach 前"守点"（占位 artifact），
            // mission 只把任务点当第一个目标，不蹲守。
            if (!me.reachDone) {
                if (me.x == m.missionX && me.y == m.missionY) return "wait";
                const std::string dir =
                    stepToward(m, bots, self, m.missionX, m.missionY, tick);
                return dir.empty() ? "wait" : "move " + dir;
            }
            const int best = nearestResourceIndex(m, me);
            if (best < 0) return "wait";
            const std::string dir =
                stepToward(m, bots, self, m.res[best].x, m.res[best].y, tick);
            return dir.empty() ? "wait" : "move " + dir;
        }
        case OPP_MAD: {
            // 疯狗：80% 乱走（不避障，裁判会拒绝撞墙）、20% 见敌就打。
            // 专测裁判的争格/卡位/拒绝逻辑与 Agent 的容错。
            unsigned long long h = seed * 0x2545F4914F6CDD1DULL +
                                   static_cast<unsigned long long>(tick) * 0x9E3779B97F4A7C15ULL +
                                   static_cast<unsigned long long>(self) * 0xBF58476D1CE4E5B9ULL;
            h ^= h >> 33;
            const unsigned r = static_cast<unsigned>(h >> 11);
            const char* dirs[4] = {"up", "down", "left", "right"};
            if (r % 100 < 80) return std::string("move ") + dirs[r % 4];
            if (near >= 0) {
                const std::string ad = attackDirTo(me, bots[near]);
                if (!ad.empty()) return "attack " + ad;
            }
            return std::string("move ") + dirs[r % 4];
        }
        case OPP_CAMPER: {
            if (near >= 0) {
                const std::string ad = attackDirTo(me, bots[near]);
                if (!ad.empty() && tick > me.protectUntil) return "attack " + ad;
                if (me.hp <= 40 && nearD <= 3) return "defend";
            }
            const int campX = W / 2, campY = H / 2;
            // 到点后原地等。不要一直 DEFEND：那是脚本策略的"无限防御"，
            // 会把 maxDefendStreak 统计污染成"Agent 的问题"。
            if (me.x == campX && me.y == campY) return "wait";
            const std::string dir = stepToward(m, bots, self, campX, campY, tick);
            return dir.empty() ? "wait" : "move " + dir;
        }
        case OPP_TACTICIAN: {
            // 战术型对手：比莽夫聪明——会算换血、会躲射线、劣势会撤。
            if (near < 0) {
                // 没敌人时也做点正事：去最近的资源（比干等更接近真实对手）
                int best = -1, bestD = 999;
                for (std::size_t i = 0; i < m.res.size(); ++i) {
                    if (m.res[i].taken) continue;
                    const int d = std::abs(m.res[i].x - me.x) + std::abs(m.res[i].y - me.y);
                    if (d < bestD) { bestD = d; best = static_cast<int>(i); }
                }
                if (best < 0) return "wait";
                const std::string dir =
                    stepToward(m, bots, self, m.res[best].x, m.res[best].y, tick);
                return dir.empty() ? "wait" : "move " + dir;
            }
            const Bot& foe = bots[near];
            const std::string ad = attackDirTo(me, foe);
            if (!ad.empty() && tick > me.protectUntil) {
                // 同轴对峙：血量优势或能击杀 → 打；劣势且致命 → 躲（横移出轴）
                if (foe.hp <= 20 || me.hp > 25) return "attack " + ad;
                if (me.hp <= 20) {
                    // 横移脱轴：与敌人同行 → 上下；同列 → 左右
                    const bool sameRow = (me.y == foe.y);
                    const char* perp[2] = {sameRow ? "up" : "left",
                                           sameRow ? "down" : "right"};
                    for (int k = 0; k < 2; ++k) {
                        int dx = 0, dy = 0;
                        delta(perp[k], dx, dy);
                        if (!m.wallAt(me.x + dx, me.y + dy, tick)) {
                            bool occ = false;
                            for (int t = 0; t < 4; ++t)
                                if (t != self && bots[t].alive &&
                                    bots[t].x == me.x + dx && bots[t].y == me.y + dy)
                                    occ = true;
                            if (!occ) return std::string("move ") + perp[k];
                        }
                    }
                }
                return "defend";
            }
            // 不在同轴：残血拉开，健康则逼近
            if (me.hp <= 30) {
                const int awayX = me.x < foe.x ? 0 : (W - 1);
                const int awayY = me.y < foe.y ? 0 : (H - 1);
                const std::string dir = stepToward(m, bots, self, awayX, awayY, tick);
                return dir.empty() ? "wait" : "move " + dir;
            }
            const std::string dir = stepToward(m, bots, self, foe.x, foe.y, tick);
            return dir.empty() ? "wait" : "move " + dir;
        }
        default: {
            // OPP_RANDOM：确定性地按 (seed,tick,self) 取伪随机
            unsigned long long h = seed * 0x9E3779B97F4A7C15ULL +
                                   static_cast<unsigned long long>(tick) * 0xBF58476D1CE4E5B9ULL +
                                   static_cast<unsigned long long>(self) * 0x94D049BB133111EBULL;
            h ^= h >> 31;
            h *= 0x7FB5D329728EA185ULL;
            h ^= h >> 27;
            const unsigned r = static_cast<unsigned>(h);
            const char* dirs[4] = {"up", "down", "left", "right"};
            if (r % 100 < 55) return std::string("move ") + dirs[r % 4];
            if (r % 100 < 80 && near >= 0) {
                const std::string ad = attackDirTo(me, bots[near]);
                if (!ad.empty()) return "attack " + ad;
            }
            if (r % 100 < 90) return "defend";
            return "wait";
        }
    }
}

// 静态可达性（忽略车辆占位）：只判断任务点是否被墙围死。
bool staticReachable(const Match& m, int x0, int y0, int tx, int ty, int tick) {
    if (!m.inb(tx, ty)) return false;
    if (m.wallAt(tx, ty, tick)) return false;
    std::vector<char> seen(W * H, 0);
    std::vector<int> queue;
    queue.push_back(m.idx(x0, y0));
    seen[m.idx(x0, y0)] = 1;
    for (std::size_t qi = 0; qi < queue.size(); ++qi) {
        const int cur = queue[qi];
        if (cur == m.idx(tx, ty)) return true;
        const int cx = cur % W, cy = cur / W;
        const int nb[4][2] = {{cx + 1, cy}, {cx - 1, cy}, {cx, cy + 1}, {cx, cy - 1}};
        for (int k = 0; k < 4; ++k) {
            const int nx = nb[k][0], ny = nb[k][1];
            if (!m.inb(nx, ny) || m.wallAt(nx, ny, tick)) continue;
            const int ni = m.idx(nx, ny);
            if (seen[ni]) continue;
            seen[ni] = 1;
            queue.push_back(ni);
        }
    }
    return false;
}

// "move right" / "attack up" / "defend" / "wait" → Act
Act parseCmd(const std::string& c) {
    Act a;
    if (c == "defend") { a.action = "defend"; a.ok = true; return a; }
    if (c == "wait" || c.empty()) { a.action = "wait"; a.ok = true; return a; }
    const std::size_t sp = c.find(' ');
    if (sp == std::string::npos) { a.action = "wait"; a.ok = true; return a; }
    a.action = c.substr(0, sp);
    a.dir = c.substr(sp + 1);
    a.ok = (a.action == "move" || a.action == "attack") &&
           (a.dir == "up" || a.dir == "down" || a.dir == "left" || a.dir == "right");
    if (!a.ok) { a.action = "wait"; a.dir.clear(); a.ok = true; }
    return a;
}

struct Stat {
    double sumScore = 0, sumR2 = 0, sumD = 0, sumMissions = 0;
    double sumScoreA = 0;   // 只统计 A 席（异质对手时用）
    long deaths = 0, kills = 0, attacks = 0, defends = 0, moves = 0, waits = 0;
    int maxDefendStreak = 0;  // 单次最长连续防御（检测"无限防御"）
    int games = 0;
    long mTwo = 0, mHigh = 0, mReach = 0;
    long aTwo = 0, aHigh = 0, aReach = 0;  // 只统计 A 席
    long evDamage = 0, evDeaths = 0, evHits = 0;  // evHits=真正影响到车的次数
    long evPepperTriggers = 0;                  // 踩中辣椒点次数（机制计数）
    long evHitsA = 0, evDamageA = 0, evDeathsA = 0;  // 只统计 A 席
    long evVetoA = 0, evTargetA = 0, evInjectA = 0;  // A 的事件决策/注入计数
    long evTypeHits[EV_KIND_N] = {0};   // 按事件类型（所有席位）
    long evTypeDamage[EV_KIND_N] = {0};
    long evTypeDeaths[EV_KIND_N] = {0};
    long contestHitsA = 0, gateTicksA = 0, economyHoldsA = 0, defensiveA = 0;
    long economyHoldHitA = 0;  // 经济门在"最近掉过血/被开火"时仍压住攻击的 Tick 数
    long pepperTrapsA = 0, unrivaledGrantsA = 0, unrivaledAttacksA = 0;
    long damageMissionTicksA = 0;   // 伤害任务未完成、强化接战的 Tick 数
    long splitEscapesA = 0, panHoldsA = 0;  // 事件利用计数（A 席）
    long dcapBlocksA = 0;                          // D 值止盈拦下的攻击次数
    long raceDropsA = 0;                           // race 门剔除的高价值候选数
    int spawnBad = 0;                              // 出生格重叠/在墙内（应为 0）
    std::vector<unsigned long long> traceHashes;  // 每局轨迹哈希（自检/复现）
    // ---- 建议书《对手策略池》§5 要求的监控指标 ----
    long deadlockTicks = 0;   // 僵局 Tick 总数（连续 ≥30 无移动无攻击）
    int deadlockCount = 0;    // 僵局次数（每满 30 Tick 计一次）
    long invalidA = 0;        // A 席提交后被裁判拒绝的动作数（应为 0）
    long aTimeUs = 0;         // A 席决策累计耗时（µs）
    long aTimeMaxUs = 0;      // A 席单 Tick 最大耗时（µs）
    long chokeDeathsA = 0;    // A 席死于咽喉格的次数（地形回归指标）
    // mix 模式按对手类型归因 A 的表现
    double mixScoreA[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int mixGames[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // A 席专项：死亡/击杀与两个计分通道的上限命中率（对照 v4.4 §2.2/§6.2）
    long aDeaths = 0, aKills = 0;
    long aDeathsToWushuang = 0;  // A 死亡中，致死批次里含"无双"敌车的次数（#4 暴露量）
    double aR2Sum = 0, aDSum = 0;
    int aR2CapGames = 0;   // A 的 R2 ≥ 120（资源通道封顶）
    int aDCapGames = 0;    // A 的 D ≥ 500（对抗通道封顶）
    int aDamageDone = 0;   // A 完成伤害任务的局数（test-only damagetask=N）
    int aTieGames = 0;     // A 与某家完全同分（规则要求加赛）
    // A 的任务诊断（v4.4 §9：各任务完成率、完成 Tick、失败原因）
    long aReachDone = 0, aReachTickSum = 0;
    long aReachFail = 0, aReachWalled = 0, aReachNoTime = 0, aReachContested = 0;
    long aLateDeaths = 0;  // A 在 tick≥400 的死亡次数（任务失败常见原因）
    int aReason[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int aTargetSwitches = 0;  // A 的决策目标格变化次数（抖动诊断）
    int aMaxNoTargetRun = 0;  // A 连续 no_target 的最长长度（区分"长等待"vs"目标抖动"）
    long aNoTargetRuns = 0;   // A 进入 no_target 的次数
    // 席位轮换诊断：按 A 本局实际使用的出生角（0=左上,1=右上,2=左下,3=右下）
    double rotScoreA[4] = {0, 0, 0, 0};
    int rotRankSumA[4] = {0, 0, 0, 0};
    int rotTop2A[4] = {0, 0, 0, 0};
    int rotGames[4] = {0, 0, 0, 0};
    // mix3 按席位抽签归因：模式在 B/C/D 出现的席位次数 + 此时 A 的得分/前二
    int mix3SeatCount[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int mix3PresenceGames[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double mix3PresenceScoreA[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int mix3PresenceTop2A[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // 每队（席位）的得分/胜率/排名统计：席位固定策略，跨局可比。
    // mix 模式下 B/C/D 每局抽不同策略，按席位统计则反映"抽签运气+策略池平均"，
    // 所以 mix 下同时给出按对手类型的归因（mixScoreA）。
    double teamScore[4] = {0, 0, 0, 0};
    int teamRankSum[4] = {0, 0, 0, 0};  // 排名和（1 最优），平均 = RankSum/games
    int teamWin[4] = {0, 0, 0, 0};      // 第 1 名次数
    int teamTop2[4] = {0, 0, 0, 0};     // 前 2 名次数（晋级线）
    int samples = 0;
};

// 单局排名（规则 §10.1：总分 → 获计分资格任务数 → 资源总价值 → 有效伤害
// （含 500 以上部分）→ 无效动作少者）。完全同分时规则要求加赛/裁定，并明确
// "不按消息到达先后、席位字母或画面顺序决定胜负"——因此这里不再用席位字母
// 打破平局，完全同分返回相同名次（competition ranking）。
inline bool botRanksAbove(const std::vector<Bot>& bots, int a, int b) {
    const double sa = bots[a].score(), sb = bots[b].score();
    if (sa > sb + 1e-9) return true;
    if (sa < sb - 1e-9) return false;
    if (bots[a].missions() != bots[b].missions())
        return bots[a].missions() > bots[b].missions();
    if (bots[a].r2 != bots[b].r2) return bots[a].r2 > bots[b].r2;
    if (bots[a].dmg != bots[b].dmg) return bots[a].dmg > bots[b].dmg;
    return false;  // 完全同分：并列
}

inline bool botFullyTied(const std::vector<Bot>& bots, int a, int b) {
    return !botRanksAbove(bots, a, b) && !botRanksAbove(bots, b, a);
}

inline int matchRankOf(const std::vector<Bot>& bots, int self) {
    int rank = 1;
    for (int j = 0; j < 4; ++j) {
        if (j == self) continue;
        if (botRanksAbove(bots, j, self)) ++rank;
    }
    return rank;
}

// 一场对局的全部可调选项（原来 9 个散参收拢成一个结构）
struct RunOpts {
    int oppPolicy = OPP_AGENT;
    int lookahead = 0;
    float resAgeBonus = 0.f;
    bool reachFix = true;
    // 评测默认 = 生产配置（src/main.cpp）：depth=0、contest=true、gate=false。
    // legacy=1 才回到 Config() 的历史默认（depth=5、contest=false），用于复现旧数据。
    int depth = 0;
    bool depthAOnly = false;  // true = depth 只作用于 A 席（异构对手的 A/B 消融）
    bool rankValue = false;   // A 席使用排名口径的相对战斗价值（只改 A，便于 A/B）
    bool gate = false;        // A 席启用旧接战门（三态 + 软代价；深搜保护层）
    bool economy = true;     // A 席启用经济接战门（2026-10-27 起默认开启）
    int econPlan = -1;       // A 席经济门 P0：-1=配置默认，0/1=强制
    bool dcap = true;        // A 席启用 D≥500 止盈
    bool race = false;        // A 席启用高价值资源时间竞争门
    bool defense = false;     // A 席启用防御性苟活
    float highPriority = 1.2f; // A 席高价值资源规划权重（默认同 Config）
    int eventAware = 0;       // 测试专用：0=全盲，1=仅 A 注入真值事件，2=所有 Agent
    int eventLead = 10;       // 事件激活前多少 Tick 注入公告（测试专用；10 比 20/30 略好）
    bool eventDense = false;  // 测试专用：8 类事件固定排程（验证机制真的触发）
    int fortressMode = 0;     // v1.2 §8.5：0=中心 1=偏移 2=随机合法 3×3
    int pepper = -1;          // A 席辣椒陷阱：-1=配置默认，0/1=强制
    int unrivaled = -1;       // A 席无双 5×5：-1=配置默认，0/1=强制
    int panCaution = -1;      // A 席潘潘少动：-1=配置默认，0/1=强制
    int splitEscape = -1;     // A 席分裂提前逃离：-1=配置默认，0/1=强制
    int reachBase = -1;       // A 席 reach 死线基准（-1=配置默认）
    int reachMargin = -1;     // A 席 reach 死线余量（-1=配置默认）
    int scenario = 0;         // 1=辣椒定点场景（A 一步踩点，B 同行受火线）
    int damageTask = 0;       // test-only：>0 时给所有车公布 effective_damage 任务
    float missionCoupling = 1.0f;  // A 席任务资源加权（mission=1 开启 1.8×）
    bool fairMap = false;          // test-only：四象限镜像资源/障碍，模拟公平图
    bool contest = true;      // A 席启用任务点被占的四阶争夺（生产默认）
    bool zeroMissionA = false;  // 压力测试：A 的 AI 资格失败 → M=0（规则 §10.2）
    bool spawnRot = false; // 席位轮换：每局把 A 的出生角按拉丁方旋转
    int spawnMode = 2;     // 0=四角 1=四边中点 2=随机空格（v9 默认）
    bool chokeOn = true;   // 地形拓扑开关（A 与 Agent 型对手同步）
    bool corridorMap = false;  // 象限走廊地图（地形回归场景）
    bool oppCost = false;      // 跟随 Config 默认（关闭，见 config.h 说明）；oppcost=1 开启
    bool rayBlock = true;      // 多敌射线
    bool legacy = false;       // 历史口径：Config 默认（depth=5/无争夺/四角）
    bool csv = false;          // 逐局 CSV（配对 A/B 用）
    bool selfcheck = false;    // 配置自检：两种配置必须产生不同轨迹
    bool repro = false;        // 同配置重跑必须逐位一致
    bool strict = false;       // 样本/非法动作/机制不触发时返回非零
    int seatPick[3] = {-1, -1, -1};  // B/C/D 单独指定脚本（-1=跟随 oppPolicy）
    std::string check = "contest";  // selfcheck 的正对照：contest/depth/gate/economy/dcap/race/m0/event
};

// "过去的自己"：完整 Agent + 初稿（修改报告之前）的降维配置
rmtc::Config pastConfig() {
    rmtc::Config cfg;
    cfg.combat_depth = 0;             // 初稿没有深度搜索
    cfg.reach_approach_occupied = false;  // 初稿对被占任务点直接放弃
    cfg.res_age_bonus = 0.1f;         // 初稿默认
    return cfg;
}

// mix 模式：按权重确定性抽取本局对手类型（种子绑定，可复现）
int pickMix(unsigned long long seed) {
    unsigned long long h = seed * 0x9E3779B97F4A7C15ULL + 0xD1B54A32D192ED03ULL;
    h ^= h >> 32;
    h *= 0xCBF29CE484222325ULL;
    h ^= h >> 29;
    const double u = (h & 0xFFFF) / 65536.0;
    double acc = 0;
    const int poolN = static_cast<int>(sizeof(kMixPool) / sizeof(kMixPool[0]));
    for (int k = 0; k < poolN; ++k) {
        acc += kMixWeight[k];
        if (u < acc) return kMixPool[k];
    }
    return kMixPool[poolN - 1];
}

// mix3：按 (seed, 席位) 独立抽取，三家对手可以不同。
// 只依赖 seed/seat，不依赖地图生成顺序，因此可与同一 seed 的舞台配对比较。
int pickMix3(unsigned long long seed, int seat) {
    unsigned long long h = seed * 0x9E3779B97F4A7C15ULL +
                           static_cast<unsigned long long>(seat + 1) * 0xD1B54A32D192ED03ULL;
    h ^= h >> 30;
    h *= 0xBF58476D1CE4E5B9ULL;
    h ^= h >> 27;
    h *= 0x94D049BB133111EBULL;
    h ^= h >> 31;
    const double u = (h & 0xFFFF) / 65536.0;
    const int poolN = static_cast<int>(sizeof(kMix3Pool) / sizeof(kMix3Pool[0]));
    double acc = 0;
    for (int k = 0; k < poolN; ++k) {
        acc += kMix3Weight[k];
        if (u < acc) return kMix3Pool[k];
    }
    return kMix3Pool[poolN - 1];
}

// oppPolicy：0=四车全 Agent（自博弈）；1..4=B/C/D 使用对应脚本策略。
// lookahead：传给 Agent 的 Combat::Params::lookahead（0=原启发式）。
// resAgeBonus：资源目击加成（问题 1 的消融开关，0 = 关闭）。
void runMatch(unsigned long long seed, int gameNo, Stat& st, bool verbose, bool eventsOn,
              const RunOpts& opt) {
    // 本局 A 的事件计数（CSV 用；st 是跨局累计）
    long gameEvHitsA = 0, gameEvDamageA = 0, gameEvDeathsA = 0;
    bool healCounted[4] = {false, false, false, false};  // 每局回血点首次拾取
    bool evInjected[4][8] = {{false}};  // 事件感知测试：每车每事件只注入一次
    // mix（旧口径）：本局 B/C/D 统一用抽到的模式（便于按模式归因 A 的表现）
    const int oppPolicy = (opt.oppPolicy == OPP_MIX) ? pickMix(seed) : opt.oppPolicy;
    // mix3（真实口径）：三家对手按席位独立抽取，允许不同模式、允许强 Agent
    int seatPolicy[4] = {OPP_AGENT, oppPolicy, oppPolicy, oppPolicy};
    for (int i = 0; i < 3; ++i)
        if (opt.seatPick[i] >= 0) seatPolicy[i + 1] = opt.seatPick[i];
    if (opt.oppPolicy == OPP_MIX3) {
        for (int i = 1; i < 4; ++i) seatPolicy[i] = pickMix3(seed, i);
    } else if (opt.oppPolicy == OPP_MIX1) {
        // v9 的高强度混合：恰有一个同级 Agent，另两家是抢匪 + 任务蹲守。
        // Agent 所在席位按 seed 轮转，避免把"对手强度"误读成席位效应。
        const int agentSeat = 1 + static_cast<int>(seed % 3);
        const int scripts[2] = {OPP_RUSHER, OPP_MCAMP};
        int k = 0;
        for (int i = 1; i < 4; ++i)
            seatPolicy[i] = (i == agentSeat) ? OPP_AGENT : scripts[k++];
    } else if (opt.oppPolicy == OPP_MIXREAL) {
        // v9 的真实混合：农夫 + 抢匪 + 任务党，全脚本；角色按 seed 轮转。
        const int roles[3] = {OPP_FARMER, OPP_RUSHER, OPP_MCAMP};
        const int off = static_cast<int>(seed % 3);
        for (int i = 1; i < 4; ++i) seatPolicy[i] = roles[(i - 1 + off) % 3];
    }
    // 席位轮换：每局把出生角整体旋转。调度必须同时满足两点：
    //   1) 每 4 局内四个角各出现一次（平衡）；
    //   2) 角的分配不能与 seed 低两位绑定——xorshift 用连续种子时，seed%4
    //      会与地图前几个随机数相关，实测会出现"BR 角系统性更好"的假偏差。
    // 用拉丁方 (r + floor(k/4)) % 4：组内仍是四角各一次，跨组则每个角会
    // 轮到四个不同的 seed 余数。
    const int spawnRot = opt.spawnRot ? (((gameNo % 4) + (gameNo / 4)) % 4) : 0;
    Match m(seed, eventsOn, opt.corridorMap, opt.spawnMode, opt.fortressMode,
            opt.damageTask, opt.fairMap);
    if (eventsOn && opt.eventDense) m.buildDenseEvents();
    // 真值地形的咽喉标记（走廊死亡统计用；Agent 自己的记忆世界与此独立）
    rmtc::World truth(24, 24);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            if (m.blocked[m.idx(x, y)]) truth.markBlocked(x, y);
            else truth.markFree(x, y);
    truth.analyzeTopology(0);

    // 生产默认：所有 Agent 都用 depth=0 + reach_contest=true。
    // legacy=1 回到 Config() 的历史默认；depthAOnly 只影响 A/B 的 depth 归因。
    const int baseDepth = opt.legacy
                              ? (opt.depth >= 0 ? opt.depth : rmtc::Config().combat_depth)
                              : opt.depth;
    const int oppDepth = opt.depthAOnly
                             ? (opt.legacy ? rmtc::Config().combat_depth : 0)
                             : baseDepth;
    const bool baseContest = !opt.legacy;

    std::vector<Bot> bots(4);
    for (int i = 0; i < 4; ++i) {
        Bot& b = bots[i];
        b.team = i;
        b.id = static_cast<char>('A' + i);
        b.policy = (i == 0 || seatPolicy[i] <= 0) ? OPP_AGENT : seatPolicy[i];
        const int corner = (i + spawnRot) % 4;
        b.x = b.sx = m.spawnX(corner);
        b.y = b.sy = m.spawnY(corner);
        b.damageTarget = m.damageTaskTarget_;
        // Agent 型选手的配置：A 用当版；"过去的自己"用初稿降维配置
        rmtc::Config cfg;
        if (b.policy == OPP_PAST) {
            cfg = pastConfig();
        } else {
            cfg.res_age_bonus = opt.resAgeBonus;
            cfg.reach_approach_occupied = opt.reachFix;
            cfg.choke_enable = opt.chokeOn;
            cfg.combat_opp_cost = opt.oppCost;
            cfg.combat_ray_block = opt.rayBlock;
            // 生产基线：所有 Agent 都是启发式 + 争夺；A 席再叠加 A/B 开关。
            cfg.combat_depth = (i == 0) ? baseDepth : oppDepth;
            cfg.reach_contest = (i == 0) ? opt.contest : baseContest;
            // 经济门只叠加在 A 席（生产里只有我方进程会用这个门）；
            // 脚本对手不读 Config，Agent 型对手保持基线，A/B 才是单变量。
            if (i == 0) {
                cfg.combat_rank_value = opt.rankValue;
                cfg.engage_gate = opt.gate;
                cfg.economy_gate = opt.economy;
                if (opt.econPlan >= 0)
                    cfg.economy_gate_actual_action = (opt.econPlan != 0);
                cfg.d_cap_gate = opt.dcap;
                cfg.race_gate = opt.race;
                cfg.defensive_mode = opt.defense;
                cfg.high_priority = opt.highPriority;
                cfg.mission_coupling = opt.missionCoupling;
                if (opt.pepper >= 0) cfg.pepper_trap = (opt.pepper != 0);
                if (opt.unrivaled >= 0) cfg.unrivaled_attack = (opt.unrivaled != 0);
                if (opt.panCaution >= 0) cfg.pan_caution = (opt.panCaution != 0);
                if (opt.splitEscape >= 0) cfg.split_escape = (opt.splitEscape != 0);
                if (opt.reachBase > 0) cfg.reach_deadline_base = opt.reachBase;
                if (opt.reachMargin >= 0) cfg.reach_deadline_margin = opt.reachMargin;
            }
        }
        b.agent.reset(new rmtc::Agent(cfg));
        // PAST 永不覆盖 depth：旧代码把 main 解析出的 depth=5 也套到 PAST 上，
        // 使"过去的自己"实际是"当版 Agent + 弱化 reach"，对手池因此偏强。
        // 这里只保留 lookahead 的覆盖；depth 已通过 cfg 在构造时生效。
        if (b.policy != OPP_PAST && opt.lookahead != 0) {
            rmtc::Combat::Params cp = b.agent->combat().params();
            cp.lookahead = opt.lookahead;
            b.agent->combat().configure(cp);
        }
    }
    // 测试专用：定点场景。
    //   scenario=1 辣椒：A 一步踩点，B 同行受火线；
    //   scenario=2 无双：A 在非中心 (2,2)-(4,4) 堡垒驻留 10 Tick，随后 B 进入 5×5；
    //   scenario=3 洁身自好：A 攻击防御中的 B，反伤致死，B 应拿到击杀奖励。
    if (opt.scenario >= 1 && opt.scenario <= 3) {
        for (int y = 1; y <= 12; ++y)
            for (int x = 1; x <= 12; ++x) m.blocked[m.idx(x, y)] = 0;
        m.events.clear();
        Event pe;
        pe.type = (opt.scenario == 1) ? EV_PEPPER
                  : (opt.scenario == 2) ? EV_FORTRESS : EV_THORNS;
        pe.tAct = 0;
        pe.dur = 500;
        pe.n = 0;
        if (opt.scenario == 1) {
            bots[0].x = bots[0].sx = 5; bots[0].y = bots[0].sy = 5;
            bots[1].x = bots[1].sx = 9; bots[1].y = bots[1].sy = 5;
            bots[2].x = bots[2].sx = 10; bots[2].y = bots[2].sy = 10;
            bots[3].x = bots[3].sx = 15; bots[3].y = bots[3].sy = 15;
            pe.x0 = 6; pe.y0 = 5; pe.x1 = 6; pe.y1 = 5;
        } else if (opt.scenario == 2) {
            bots[0].x = bots[0].sx = 3; bots[0].y = bots[0].sy = 3;
            bots[1].x = bots[1].sx = 4; bots[1].y = bots[1].sy = 4;
            bots[2].x = bots[2].sx = 10; bots[2].y = bots[2].sy = 10;
            bots[3].x = bots[3].sx = 15; bots[3].y = bots[3].sy = 15;
            pe.x0 = 2; pe.y0 = 2; pe.x1 = 4; pe.y1 = 4;
        } else {
            bots[0].x = bots[0].sx = 5; bots[0].y = bots[0].sy = 5;
            bots[0].hp = 20;
            bots[1].x = bots[1].sx = 6; bots[1].y = bots[1].sy = 5;
            bots[2].x = bots[2].sx = 10; bots[2].y = bots[2].sy = 10;
            bots[3].x = bots[3].sx = 15; bots[3].y = bots[3].sy = 15;
            pe.x0 = 0; pe.y0 = 0; pe.x1 = W - 1; pe.y1 = H - 1;
        }
        m.events.push_back(pe);
    }

    for (int i = 0; i < 4; ++i) {
        if (m.wall(bots[i].sx, bots[i].sy)) ++st.spawnBad;
        for (int j = i + 1; j < 4; ++j)
            if (bots[i].sx == bots[j].sx && bots[i].sy == bots[j].sy) ++st.spawnBad;
    }
    const std::string session = "m" + std::to_string(seed);
    std::vector<Act> acts(4);
    std::vector<int> dmgTo(4, 0);
    std::vector<std::vector<int> > contrib(4);
    std::vector<std::vector<int> > dmgBy(4);
    std::vector<int> thornTo(4, 0);

    // v1.2 §7：普通复活/分裂立即复活共用同一选点规则。传 grantProtect=false
    // 表示世界分裂的立即复活（规则未授予普通复活的 2 Tick 保护）。
    const auto respawnBot = [&](int i, int tick, bool grantProtect) {
        Bot& b = bots[i];
        int rx = b.sx, ry = b.sy;
        rmtc::chooseRespawnCell(
            b.sx, b.sy, W, H, tick,
            [&](int x, int y, int t) { return m.wallAt(x, y, t); },
            [&](int x, int y) {
                for (int k = 0; k < 4; ++k)
                    if (k != i && bots[k].alive && bots[k].x == x && bots[k].y == y)
                        return true;
                return false;
            },
            rx, ry);
        b.alive = true;
        b.hp = 100;
        b.x = rx;
        b.y = ry;
        b.respawnAt = -1;
        b.dwell = 0;
        if (grantProtect) {
            b.protectUntil = tick + 1;
            b.protectX = rx;
            b.protectY = ry;
        } else {
            b.protectUntil = tick - 1;
            b.protectX = b.protectY = -1;
        }
    };
    int deadlockStreak = 0;
    int dist450 = -1, occ450 = 0, prevTarget = -1;  // A 的任务/抖动诊断
    int noTargetRun = 0, maxNoTargetRun = 0;        // A 连续 no_target 长度
    long noTargetRuns = 0;
    unsigned long long trace = 1469598103934665603ULL;  // FNV-1a，配置自检/复现用

    for (int tick = 0; tick < TICKS; ++tick) {
        // 本 Tick 开局存活快照（死亡位置统计用：死亡发生在事件/战斗/火线多阶段）
        bool aliveAtStart[4];
        for (int i = 0; i < 4; ++i) aliveAtStart[i] = bots[i].alive;
        // 本 Tick 的所有死亡（含爆炸/分裂/辣椒）都要在末尾按“每次死亡 60”
        // 扣有效伤害；用计数而不是 bool，否则同一 Tick 先被分裂击杀、立即
        // 复活后又被攻击击杀时，第二次死亡会漏扣。
        int died[4] = {0, 0, 0, 0};
        bool moved[4] = {false, false, false, false};
        // ---- 1) 清理到期效果、激活事件 ----
        bool illus[4] = {false, false, false, false};
        bool thorns = false;
        bool vulnActive = false;
        int vulnTriggerEnd = -1;  // 潘潘：前 20 Tick 移动才触发
        int vulnEnd = -1;         // 潘潘：统一在事件生效后第 30 Tick 到期
        int fortX0 = -1, fortY0 = -1, fortX1 = -1, fortY1 = -1;
        bool fortActive = false;
        for (std::size_t k = 0; k < m.events.size(); ++k) {
            const Event& e = m.events[k];
            const bool active = tick >= e.tAct && tick < e.tAct + (e.dur > 0 ? e.dur : 1);
            if (!active) continue;
            if (e.type == EV_THORNS) thorns = true;
            if (e.type == EV_VULNERABLE) {
                vulnActive = true;
                vulnTriggerEnd = e.tAct + 20;
                vulnEnd = e.tAct + (e.dur > 0 ? e.dur : 30);
            }
            if (e.type == EV_FORTRESS) {
                fortActive = true;
                fortX0 = e.x0; fortY0 = e.y0; fortX1 = e.x1; fortY1 = e.y1;
            }
            if (e.type == EV_ILLUSION) {
                for (int i = 0; i < 4; ++i)
                    if (bots[i].alive && bots[i].x >= e.x0 && bots[i].x <= e.x1 &&
                        bots[i].y >= e.y0 && bots[i].y <= e.y1) {
                        illus[i] = true;
                        if (tick == e.tAct) {
                            ++st.evHits;
                            ++st.evTypeHits[e.type];
                            if (i == 0) { ++st.evHitsA; ++gameEvHitsA; }
                        }
                    }
            }
            if (e.type == EV_SPLIT && tick == e.tAct) {
                for (int y = 0; y < H; ++y) m.wallUntil[m.idx(e.x0, y)] = e.tAct + e.dur;
                for (int x = 0; x < W; ++x) m.wallUntil[m.idx(x, e.y0)] = e.tAct + e.dur;
            }
            if (e.type == EV_EXPLOSION && tick == e.tAct) {
                for (int i = 0; i < 4; ++i) {
                    Bot& b = bots[i];
                    if (!b.alive || tick <= b.protectUntil) continue;
                    if (b.x < e.x0 || b.x > e.x1 || b.y < e.y0 || b.y > e.y1) continue;
                    const int d = b.hp > 80 ? 80 : b.hp;
                    b.hp -= 80;
                    st.evDamage += d;
                    ++st.evHits;
                    ++st.evTypeHits[e.type];
                    st.evTypeDamage[e.type] += d;
                    if (i == 0) { ++st.evHitsA; st.evDamageA += d; ++gameEvHitsA; gameEvDamageA += d; }
                    if (b.hp <= 0) {
                        b.hp = 0; b.alive = false; b.deaths++; b.respawnAt = tick + 3;
                        b.dwell = 0; st.evDeaths++; ++died[i];
                        ++st.evTypeDeaths[e.type];
                        if (i == 0) { ++st.evDeathsA; ++gameEvDeathsA; }
                    }
                }
            }
            if (e.type == EV_SPLIT && tick == e.tAct) {
                for (int i = 0; i < 4; ++i) {
                    Bot& b = bots[i];
                    if (!b.alive) continue;
                    if (b.x != e.x0 && b.y != e.y0) continue;
                    b.deaths++; st.evDeaths++; ++died[i]; ++st.evHits;
                    ++st.evTypeHits[e.type];
                    ++st.evTypeDeaths[e.type];
                    if (i == 0) { ++st.evHitsA; ++st.evDeathsA; ++gameEvHitsA; ++gameEvDeathsA; }
                    respawnBot(i, tick, false);
                    b.forcedWait = true;
                }
            }
        }
        for (int i = 0; i < 4; ++i) {
            Bot& b = bots[i];
            if (!b.alive && b.respawnAt == tick) {
                respawnBot(i, tick, true);
                // v1.2 §7：已获得/负面的效果跨死亡与复活保留，绝对到期时间不顺延；
                // 因此这里**不能**清 vulnerableUntil / healUntil。
            }
        }

        // ---- 感知 → 动作 ----
        bool anyInvalidA = false;
        for (int i = 0; i < 4; ++i) {
            if (bots[i].policy == OPP_AGENT || bots[i].policy == OPP_PAST) {
                // 测试专用：事件感知模式。把模拟器真值事件结构化注入 Agent，
                // 绕过 event_text/LLM；只用于 A/B，生产不走这条路径。
                if (opt.eventAware && bots[i].agent &&
                    (i == 0 || opt.eventAware == 2 || opt.eventAware == 4)) {
                    const bool harmfulOnly = opt.eventAware >= 3;
                    const int lead = opt.eventLead > 0 ? opt.eventLead : 0;
                    for (std::size_t k = 0; k < m.events.size() && k < 8; ++k) {
                        if (evInjected[i][k]) continue;
                        const Event& e = m.events[k];
                        if (harmfulOnly && e.type != EV_EXPLOSION && e.type != EV_SPLIT)
                            continue;
                        int injTick = e.tAct - lead;
                        // 首帧（tick=0）会建立 session 并清空事件池；若公告恰好
                        // 在 tick 0 生效，注入必须延到 tick 1，否则会被清掉。
                        if (injTick <= 0) injTick = 1;
                        if (tick < injTick || tick > e.tAct + 1) continue;
                        rmtc::EventReport rep;
                        rep.kind = toEventKind(e.type);
                        rep.x0 = e.x0; rep.y0 = e.y0;
                        rep.x1 = e.x1; rep.y1 = e.y1;
                        rep.activate_tick = e.tAct;
                        rep.end_tick = e.tAct + (e.dur > 0 ? e.dur : 1);
                        rep.confidence = 1.0f;
                        bots[i].agent->injectTestEvent(rep);
                        evInjected[i][k] = true;
                        if (i == 0) ++st.evInjectA;
                    }
                }
                const std::string per = buildPerception(m, bots, i, tick, session);
                // A 席计时（建议书 §5 指标 4：单 Tick 决策耗时，预算 500ms）
                const std::chrono::steady_clock::time_point ta =
                    std::chrono::steady_clock::now();
                const std::string out = bots[i].agent->onPerception(per);
                if (i == 0) {
                    const long us = static_cast<long>(
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now() - ta)
                            .count());
                    st.aTimeUs += us;
                    if (us > st.aTimeMaxUs) st.aTimeMaxUs = us;
                }
                acts[i] = parseAction(out);
                if (!acts[i].ok) {
                    acts[i].action = "wait";
                    if (i == 0) anyInvalidA = true;
                }
            } else {
                acts[i] = parseCmd(scriptedAction(m, bots, i, tick, seed));
            }
        }
        if (opt.scenario == 1) {
            for (int i = 1; i < 4; ++i) acts[i] = parseCmd("wait");
            // tick 0 先让 A 等待，避免它在事件注入前自己走到辣椒点上；
            // tick 1 起事件已进入 Agent，踩点由 tryPepperTrap 接管。
            if (tick == 0) acts[0] = parseCmd("wait");
        } else if (opt.scenario == 2) {
            for (int i = 1; i < 4; ++i) acts[i] = parseCmd("wait");
            // 前 11 Tick 保持在堡垒内（事件在 tick 1 注入，tick 1..10 恰好 10 次）；
            // tick 11 起放开，验证 5×5 攻击是否接管。
            if (tick <= 10) acts[0] = parseCmd("wait");
        } else if (opt.scenario == 3) {
            // tick 0：A 攻击右侧防御中的 B；之后 A 等待，避免反复送死。
            // 反伤应让 A 阵亡，且 B 作为防御方拿到 60 击杀奖励（v1.2 §8.8）。
            acts[0] = (tick == 0) ? parseCmd("attack right") : parseCmd("wait");
            acts[1] = parseCmd("defend");
            acts[2] = parseCmd("wait");
            acts[3] = parseCmd("wait");
        }
        if (anyInvalidA) ++st.invalidA;
        // 轨迹哈希：动作 + 位置 + HP。只用于自检/复现，不参与决策。
        trace ^= static_cast<unsigned long long>(tick + 1);
        trace *= 1099511628211ULL;
        for (int i = 0; i < 4; ++i) {
            const std::string& s = acts[i].action;
            for (std::size_t c = 0; c < s.size(); ++c) {
                trace ^= static_cast<unsigned char>(s[c]);
                trace *= 1099511628211ULL;
            }
            for (std::size_t c = 0; c < acts[i].dir.size(); ++c) {
                trace ^= static_cast<unsigned char>(acts[i].dir[c]);
                trace *= 1099511628211ULL;
            }
            trace ^= static_cast<unsigned long long>(bots[i].x * 131 + bots[i].y * 17 +
                                                     bots[i].hp);
            trace *= 1099511628211ULL;
        }
        // A 席决策原因直方图 + 目标格切换次数（对照 v4.4 §8 抖动控制）
        if (bots[0].policy == OPP_AGENT) {
            const rmtc::Decision& ld = bots[0].agent->lastDecision();
            if (ld.reason >= 0 && ld.reason < 13) ++st.aReason[ld.reason];
            if (ld.targetIdx >= 0) {
                if (prevTarget >= 0 && ld.targetIdx != prevTarget) ++st.aTargetSwitches;
                prevTarget = ld.targetIdx;
            }
            if (ld.reason == rmtc::R_WAIT_NO_TARGET) {
                if (noTargetRun == 0) ++noTargetRuns;
                ++noTargetRun;
                if (noTargetRun > maxNoTargetRun) maxNoTargetRun = noTargetRun;
            } else {
                noTargetRun = 0;
            }
        }
        // ---- 3) 设置防御 ----
        for (int i = 0; i < 4; ++i)
            bots[i].defending = bots[i].alive && !bots[i].forcedWait &&
                                acts[i].action == "defend";
        for (int i = 0; i < 4; ++i) {
            dmgTo[i] = 0;
            thornTo[i] = 0;
            contrib[i].clear();
            dmgBy[i].clear();
        }

        // ---- 4) 攻击结算（移动前坐标；§6.2 / §8.5 / §8.8 / §8.9）----
        for (int i = 0; i < 4; ++i) {
            Bot& a = bots[i];
            if (!a.alive || a.forcedWait || acts[i].action != "attack") continue;
            if (tick <= a.protectUntil) continue;
            if (a.wushuang && tick <= a.wushuangUntil) {
                // §8.5：5×5 全部存活敌人；防御目标 10、保护 0、易伤 ×2。
                // §8.8：每个“正在防御且无保护”的目标分别反伤 20，目标本身免伤。
                for (int t = 0; t < 4; ++t) {
                    if (t == i || !bots[t].alive) continue;
                    const int dx = bots[t].x - a.x, dy = bots[t].y - a.y;
                    if (dx < -2 || dx > 2 || dy < -2 || dy > 2) continue;
                    const bool prot = (tick <= bots[t].protectUntil);
                    if (thorns && bots[t].defending && !prot) {
                        thornTo[i] += 20;
                        contrib[i].push_back(t);  // 防御方参与攻击者死亡的分奖
                        dmgBy[i].push_back(0);    // 反伤不计普通攻击贡献
                        ++st.evHits;
                        ++st.evTypeHits[EV_THORNS];
                        if (i == 0) ++st.evHitsA;
                        continue;
                    }
                    int d = prot ? 0 : (bots[t].defending ? 10 : 50);
                    if (d > 0 && bots[t].vulnerableUntil > tick) d *= 2;
                    if (d > 0) {
                        dmgTo[t] += d;
                        contrib[t].push_back(i);
                        dmgBy[t].push_back(d);
                    }
                }
                continue;
            }
            int dx, dy;
            delta(acts[i].dir, dx, dy);
            int tx = a.x, ty = a.y, hit = -1;
            for (int s = 1; s <= 2; ++s) {
                tx += dx; ty += dy;
                if (!m.inb(tx, ty)) break;
                for (int t = 0; t < 4; ++t) {
                    if (t == i || !bots[t].alive) continue;
                    if (bots[t].x == tx && bots[t].y == ty) hit = t;
                }
                if (hit >= 0) break;
            }
            if (hit < 0) continue;
            if (thorns && bots[hit].defending && tick > bots[hit].protectUntil) {
                thornTo[i] += 20;          // 反伤：目标免伤，攻击者掉血
                contrib[i].push_back(hit); // v1.2 §8.8：防御方参与攻击者死亡的分奖
                dmgBy[i].push_back(0);     // 反伤不计普通攻击贡献
                ++st.evHits;
                ++st.evTypeHits[EV_THORNS];
                if (i == 0) ++st.evHitsA;
                continue;
            }
            int d = 20;
            if (tick <= bots[hit].protectUntil) d = 0;
            else if (bots[hit].defending) d = 10;
            if (bots[hit].vulnerableUntil > tick) d *= 2;
            if (d > 0) {
                dmgTo[hit] += d;
                contrib[hit].push_back(i);
                dmgBy[hit].push_back(d);
            }
        }

        // 扣血 + 死亡 + 分奖（dmgBy 保存每个来源的名义伤害，用于按比例分实际伤害）
        for (int i = 0; i < 4; ++i) {
            if (!bots[i].alive) continue;
            const int total = dmgTo[i] + thornTo[i];
            if (total <= 0) continue;
            const int hpBefore = bots[i].hp;
            const int applied = total < hpBefore ? total : hpBefore;
            if (dmgTo[i] > 0) {
                for (std::size_t k = 0; k < contrib[i].size() && k < dmgBy[i].size(); ++k) {
                    const double act = dmgBy[i][k] * applied / static_cast<double>(total);
                    bots[contrib[i][k]].dmg += act;
                }
            }
            bots[i].hp = hpBefore - applied;
            if (bots[i].hp > 0) continue;
            bots[i].hp = 0;
            bots[i].alive = false;
            bots[i].deaths++;
            bots[i].respawnAt = tick + 3;
            bots[i].dwell = 0;
            ++died[i];
            std::vector<int> uniq;
            for (std::size_t k = 0; k < contrib[i].size(); ++k) {
                bool seen = false;
                for (std::size_t q = 0; q < uniq.size(); ++q)
                    if (uniq[q] == contrib[i][k]) seen = true;
                if (!seen) uniq.push_back(contrib[i][k]);
            }
            if (!uniq.empty()) {
                const double share = 60.0 / static_cast<double>(uniq.size());
                for (std::size_t q = 0; q < uniq.size(); ++q) {
                    bots[uniq[q]].dmg += share;
                    bots[uniq[q]].kills++;
                }
            }
            // 诊断：A 的这次死亡是否有"无双"敌车参与（用于评估 #4 的暴露量）。
            if (i == 0) {
                for (std::size_t q = 0; q < uniq.size(); ++q) {
                    const Bot& src = bots[uniq[q]];
                    if (src.wushuang && tick <= src.wushuangUntil) {
                        ++st.aDeathsToWushuang;
                        break;
                    }
                }
            }
        }

        // ---- 5) 移动（幻术反方向、临时墙、争格）----
        std::vector<int> want(4, -1);
        for (int i = 0; i < 4; ++i) {
            Bot& b = bots[i];
            if (!b.alive || b.forcedWait || acts[i].action != "move") continue;
            std::string d = acts[i].dir;
            if (illus[i]) d = flip(d);
            int dx, dy;
            delta(d, dx, dy);
            const int nx = b.x + dx, ny = b.y + dy;
            if (m.wallAt(nx, ny, tick)) continue;
            bool occupied = false;
            for (int t = 0; t < 4; ++t) {
                if (t == i || !bots[t].alive) continue;
                if (bots[t].x == nx && bots[t].y == ny) occupied = true;
            }
            if (occupied) continue;
            want[i] = ny * W + nx;
        }
        for (int i = 0; i < 4; ++i) {
            if (want[i] < 0) continue;
            int clash = 0;
            for (int t = 0; t < 4; ++t) if (want[t] == want[i]) ++clash;
            if (clash > 1) continue;  // 多车争格全部失败
            bots[i].x = want[i] % W;
            bots[i].y = want[i] / W;
            moved[i] = true;
            // §7：保护期内成功移动且离开“本次复活的落点” → 立即取消保护。
            if (tick <= bots[i].protectUntil &&
                (bots[i].x != bots[i].protectX || bots[i].y != bots[i].protectY)) {
                bots[i].protectUntil = tick - 1;
            }
        }

        // ---- 6) 辣椒火线（v1.2 §8.7：同 Tick 全部火线统一结算）----
        {
            int pepperNominal[4] = {0, 0, 0, 0};
            std::vector<int> pepperSrc[4];  // 每条火线的触发者（按席位去重前）
            for (std::size_t k = 0; k < m.events.size(); ++k) {
                const Event& e = m.events[k];
                if (e.type != EV_PEPPER) continue;
                if (tick < e.tAct || tick >= e.tAct + e.dur) continue;
                for (int i = 0; i < 4; ++i) {
                    // 只有“本 Tick 成功移动进入点位”才触发；WAIT/移动失败不算。
                    if (!bots[i].alive || !moved[i]) continue;
                    if (bots[i].x != e.x0 || bots[i].y != e.y0) continue;
                    ++st.evPepperTriggers;
                    // 十字火线：交点（点位）上的车免疫；同行/同列其他车各受 50。
                    // 多条火线对同一目标的名义伤害累加，再按剩余 HP 比例分摊。
                    for (int t = 0; t < 4; ++t) {
                        if (!bots[t].alive || t == i) continue;
                        if (bots[t].x == e.x0 && bots[t].y == e.y0) continue;
                        if (bots[t].x != e.x0 && bots[t].y != e.y0) continue;
                        if (tick <= bots[t].protectUntil) continue;  // 出生保护免疫
                        pepperNominal[t] += 50;
                        pepperSrc[t].push_back(i);
                    }
                }
            }
            for (int t = 0; t < 4; ++t) {
                if (!bots[t].alive || pepperNominal[t] <= 0) continue;
                const int actual = pepperNominal[t] < bots[t].hp ? pepperNominal[t] : bots[t].hp;
                if (actual <= 0) continue;
                st.evDamage += actual;
                ++st.evHits;
                ++st.evTypeHits[EV_PEPPER];
                st.evTypeDamage[EV_PEPPER] += actual;
                if (t == 0) {
                    ++st.evHitsA;
                    st.evDamageA += actual;
                    ++gameEvHitsA;
                    gameEvDamageA += actual;
                }
                bots[t].hp -= actual;
                if (bots[t].hp > 0) continue;
                bots[t].hp = 0;
                bots[t].alive = false;
                bots[t].deaths++;
                bots[t].respawnAt = tick + 3;
                bots[t].dwell = 0;
                ++died[t];
                ++st.evDeaths;
                ++st.evTypeDeaths[EV_PEPPER];
                if (t == 0) { ++st.evDeathsA; ++gameEvDeathsA; }
                // 致死批次的合格触发者按队伍去重后平分 60；已阵亡触发者不再分奖。
                std::vector<int> uniq;
                for (std::size_t q = 0; q < pepperSrc[t].size(); ++q) {
                    const int src = pepperSrc[t][q];
                    if (!bots[src].alive || src == t) continue;
                    bool seen = false;
                    for (std::size_t z = 0; z < uniq.size(); ++z)
                        if (uniq[z] == src) seen = true;
                    if (!seen) uniq.push_back(src);
                }
                if (!uniq.empty()) {
                    const double share = 60.0 / static_cast<double>(uniq.size());
                    for (std::size_t q = 0; q < uniq.size(); ++q) {
                        bots[uniq[q]].dmg += share;
                        bots[uniq[q]].kills++;
                    }
                }
            }
        }

        // ---- 7) 堡垒驻留 + 回血 ----
        for (int i = 0; i < 4; ++i) {
            Bot& b = bots[i];
            if (!b.alive) continue;
            if (fortActive && b.x >= fortX0 && b.x <= fortX1 && b.y >= fortY0 &&
                b.y <= fortY1) {
                b.dwell++;
                if (b.dwell >= 10 && (!b.wushuang || b.wushuangUntil < tick)) {
                    b.wushuang = true;
                    b.wushuangUntil = 10000;  // 堡垒结束时统一失效
                    ++st.evHits;
                    ++st.evTypeHits[EV_FORTRESS];
                    if (i == 0) { ++st.evHitsA; ++gameEvHitsA; }
                }
            } else {
                b.dwell = 0;
            }
            if (tick <= b.healUntil && b.hp < 100) {
                b.hp += 10;
                if (b.hp > 100) b.hp = 100;
            }
        }
        if (!fortActive) {
            for (int i = 0; i < 4; ++i) bots[i].wushuang = false;
        }
        // 恩泽：移动后站在点上则获得/刷新
        for (std::size_t k = 0; k < m.events.size(); ++k) {
            const Event& e = m.events[k];
            if (e.type != EV_HEAL || tick < e.tAct || tick >= e.tAct + e.dur) continue;
            for (int i = 0; i < 4; ++i) {
                if (!bots[i].alive || !moved[i]) continue;
                if (bots[i].x == e.x0 && bots[i].y == e.y0) {
                    bots[i].healUntil = tick + 9;
                    if (!healCounted[i]) {
                        healCounted[i] = true;
                        ++st.evHits;
                        ++st.evTypeHits[e.type];
                        if (i == 0) { ++st.evHitsA; ++gameEvHitsA; }
                    }
                }
            }
        }
        // 易伤：**仅当"神的"事件生效时**，前 20 Tick 成功移动的车获得易伤；
        // 统一在事件生效后第 30 Tick 到期（不是各车 move+30）。
        if (vulnActive && tick < vulnTriggerEnd) {
            bool anyVuln = false;
            for (int i = 0; i < 4; ++i) {
                if (!moved[i]) continue;
                bots[i].vulnerableUntil = vulnEnd;
                anyVuln = true;
            }
            if (anyVuln) {
                ++st.evHits;
                ++st.evTypeHits[EV_VULNERABLE];
                if (moved[0]) ++st.evHitsA;
            }
        }

        // ---- 8) D 死亡惩罚（v1.2 §7/§10.2：每次死亡各扣 60，最低 0）----
        for (int i = 0; i < 4; ++i)
            if (died[i] > 0) {
                bots[i].dmg -= 60.0 * static_cast<double>(died[i]);
                if (bots[i].dmg < 0) bots[i].dmg = 0;
            }

        // ---- 9) 收集 / 任务 ----
        for (int i = 0; i < 4; ++i) {
            Bot& b = bots[i];
            if (!b.alive) continue;
            int v = 0;
            if (m.resourceAt(b.x, b.y, v)) {
                m.takeAt(b.x, b.y);
                b.r2 += v;
                if (v >= 20) b.highGot++; else b.normalGot++;
            }
            if (!b.twoNormalDone && b.normalGot >= 2) b.twoNormalDone = true;
            if (!b.highDone && b.highGot >= 1) b.highDone = true;
            if (!b.reachDone && b.x == m.missionX && b.y == m.missionY) {
                b.reachDone = true;
                b.reachTick = tick;
            }
            // v1.2 §10.1：伤害任务按结算后的有效伤害判定；仅对存活车辆检查，
            // 完成后不因随后死亡下降而撤销。
            if (b.damageTarget > 0 && b.dmg >= static_cast<double>(b.damageTarget))
                b.damageDone = true;
        }

        // 回执
        bool tickMoved = false, tickAttacked = false;
        for (int i = 0; i < 4; ++i) {
            Bot& b = bots[i];
            const bool didMove = moved[i];
            b.lastTick = tick;
            b.lastAction = acts[i].action;
            b.lastMoved = didMove;
            b.lastDir = didMove ? acts[i].dir : std::string();
            b.defendedLast = b.defending;
            b.forcedWait = false;
            if (didMove) tickMoved = true;
            if (acts[i].action == "attack" && b.alive) tickAttacked = true;
            if (acts[i].action == "attack") st.attacks++;
            else if (acts[i].action == "defend") st.defends++;
            else if (acts[i].action == "move") st.moves++;
            else st.waits++;
            // 连续防御计数：用于检测"双方无限防御"的对峙
            if (acts[i].action == "defend") {
                ++b.defendStreak;
                if (b.defendStreak > b.maxStreak) b.maxStreak = b.defendStreak;
                if (b.policy == OPP_AGENT && b.defendStreak > st.maxDefendStreak)
                    st.maxDefendStreak = b.defendStreak;
            } else {
                b.defendStreak = 0;
            }
        }
        if (verbose && tick > 445 && bots[0].policy == OPP_AGENT) {
            const rmtc::Decision& ld = bots[0].agent->lastDecision();
            std::printf("  t=%3d A(%2d,%2d) hp=%3d act=%-6s dir=%-5s reason=%s dReach=%d\n",
                        tick, bots[0].x, bots[0].y, bots[0].hp, acts[0].action.c_str(),
                        acts[0].dir.c_str(), rmtc::reason_name(ld.reason),
                        std::abs(bots[0].x - m.missionX) + std::abs(bots[0].y - m.missionY));
            const rmtc::World& w0 = bots[0].agent->world();
            if (w0.inBounds(m.missionX, m.missionY))
                std::printf("        tgtState=%d softTgt=%d\n",
                            (int)w0.state()[w0.idx(m.missionX, m.missionY)],
                            (int)w0.softBlocked(w0.idx(m.missionX, m.missionY)));
        }
        if (verbose && tick % 50 == 0) {
            std::printf("t=%3d ", tick);
            for (int i = 0; i < 4; ++i)
                std::printf("%c(%2d,%2d)%3d%s ", bots[i].id, bots[i].x, bots[i].y,
                            bots[i].alive ? bots[i].hp : 0, bots[i].alive ? "" : "*");
            std::printf("\n");
        }

        // ---- 僵局检测（建议书 §5 指标 3：连续 30 Tick 无移动且无攻击）----
        if (tickMoved || tickAttacked) {
            deadlockStreak = 0;
        } else {
            ++deadlockStreak;
            ++st.deadlockTicks;
            if (deadlockStreak >= 30) {
                ++st.deadlockCount;
                deadlockStreak = 0;
            }
        }
        // ---- 走廊死亡 / 晚期死亡统计 ----
        for (int i = 0; i < 4; ++i) {
            if (aliveAtStart[i] && !bots[i].alive && i == 0) {
                if (tick >= 400) ++st.aLateDeaths;
                if (truth.isChoke(truth.idx(bots[i].x, bots[i].y))) ++st.chokeDeathsA;
            }
        }
        // 任务失败归因的现场快照（tick=450：距截止还有 50 Tick）
        if (tick == 450) {
            dist450 = std::abs(bots[0].x - m.missionX) + std::abs(bots[0].y - m.missionY);
            for (int t = 1; t < 4; ++t) {
                if (!bots[t].alive) continue;
                if (std::abs(bots[t].x - m.missionX) + std::abs(bots[t].y - m.missionY) <= 2)
                    occ450 = 1;
            }
        }
    }

    // AI 资格压力测试：规则 §10.2 规定本场没有一次真实 AI 成功调用则 M=0，
    // 且原始任务完成数不能用于同分。这里在计分/排名前临时清空 A 的任务标记，
    // 统计完再恢复，以便同时报告"实际完成率"和"M=0 时的名次"。
    const bool saveTwo = bots[0].twoNormalDone;
    const bool saveHigh = bots[0].highDone;
    const bool saveReach = bots[0].reachDone;
    const bool saveDamage = bots[0].damageDone;
    if (opt.zeroMissionA) {
        bots[0].twoNormalDone = false;
        bots[0].highDone = false;
        bots[0].reachDone = false;
        bots[0].damageDone = false;
    }
    const int rankA = matchRankOf(bots, 0);
    for (int i = 0; i < 4; ++i) {
        st.sumScore += bots[i].score();
        st.sumR2 += bots[i].r2;
        st.sumD += bots[i].dmg;
        st.sumMissions += bots[i].missions();
        st.deaths += bots[i].deaths;
        st.kills += bots[i].kills;
        st.mTwo += bots[i].twoNormalDone ? 1 : 0;
        st.mHigh += bots[i].highDone ? 1 : 0;
        st.mReach += bots[i].reachDone ? 1 : 0;
        st.samples++;
        // 每队每局得分 + 席位排名（规则 §10.1 同分序列；完全同分并列）
        const int r = (i == 0) ? rankA : matchRankOf(bots, i);
        st.teamScore[i] += bots[i].score();
        st.teamRankSum[i] += r;
        st.teamWin[i] += (r == 1) ? 1 : 0;
        st.teamTop2[i] += (r <= 2) ? 1 : 0;
    }
    const double scoreAStat = bots[0].score();  // M=0 时的得分（若开启压力测试）
    // 轨迹哈希收尾：把最终得分/名次也纳入，M=0 这类只改计分的开关也能被自检发现。
    for (int i = 0; i < 4; ++i) {
        trace ^= static_cast<unsigned long long>(static_cast<long long>(bots[i].score() * 1000.0));
        trace *= 1099511628211ULL;
        trace ^= static_cast<unsigned long long>(bots[i].missions());
        trace *= 1099511628211ULL;
    }
    trace ^= static_cast<unsigned long long>(rankA);
    trace *= 1099511628211ULL;
    st.traceHashes.push_back(trace);
    if (bots[0].agent) {
        st.contestHitsA += bots[0].agent->contestHits();
        st.gateTicksA += bots[0].agent->gateEngagedTicks();
        st.economyHoldsA += bots[0].agent->economyHoldTicks();
        st.economyHoldHitA += bots[0].agent->economyHoldDamagedTicks();
        st.defensiveA += bots[0].agent->defensiveTicks();
        st.dcapBlocksA += bots[0].agent->combat().dcapBlocks();
        st.raceDropsA += bots[0].agent->planner().raceDrops();
        st.evVetoA += bots[0].agent->eventVetoTicks();
        st.evTargetA += bots[0].agent->eventTargetTicks();
        st.pepperTrapsA += bots[0].agent->pepperTrapCount();
        st.unrivaledGrantsA += bots[0].agent->unrivaledGrants();
        st.unrivaledAttacksA += bots[0].agent->unrivaledAttackCount();
        st.damageMissionTicksA += bots[0].agent->damageMissionTicks();
        st.splitEscapesA += bots[0].agent->splitEscapeCount();
        st.panHoldsA += bots[0].agent->panHoldCount();
    }
    if (opt.zeroMissionA) {
        bots[0].twoNormalDone = saveTwo;
        bots[0].highDone = saveHigh;
        bots[0].reachDone = saveReach;
        bots[0].damageDone = saveDamage;
    }
    st.sumScoreA += scoreAStat;
    st.aTwo += bots[0].twoNormalDone ? 1 : 0;
    st.aHigh += bots[0].highDone ? 1 : 0;
    st.aReach += bots[0].reachDone ? 1 : 0;
    st.aDeaths += bots[0].deaths;
    st.aKills += bots[0].kills;
    st.aR2Sum += bots[0].r2;
    st.aDSum += bots[0].dmg;
    st.aNoTargetRuns += noTargetRuns;
    if (maxNoTargetRun > st.aMaxNoTargetRun) st.aMaxNoTargetRun = maxNoTargetRun;
    if (bots[0].r2 >= 120.0) ++st.aR2CapGames;
    if (bots[0].dmg >= 500.0) ++st.aDCapGames;
    if (bots[0].damageTarget > 0 && bots[0].damageDone) ++st.aDamageDone;
    for (int j = 1; j < 4; ++j)
        if (botFullyTied(bots, 0, j)) { ++st.aTieGames; break; }
    // A 的 reach 任务：完成 Tick + 失败原因分桶（v4.4 §9.4）
    if (bots[0].reachDone) {
        ++st.aReachDone;
        st.aReachTickSum += bots[0].reachTick;
    } else {
        ++st.aReachFail;
        if (!staticReachable(m, bots[0].sx, bots[0].sy, m.missionX, m.missionY, TICKS))
            ++st.aReachWalled;
        else if (dist450 > 45)
            ++st.aReachNoTime;
        else if (occ450)
            ++st.aReachContested;
    }
    // 出生位置诊断：按 A 实际起点所在象限分桶（随机出生下比"席位角"更有意义）。
    {
        const int corner = (bots[0].sy >= H / 2 ? 2 : 0) + (bots[0].sx >= W / 2 ? 1 : 0);
        ++st.rotGames[corner];
        st.rotScoreA[corner] += scoreAStat;
        st.rotRankSumA[corner] += rankA;
        st.rotTop2A[corner] += (rankA <= 2) ? 1 : 0;
    }
    ++st.games;
    if (opt.csv) {
        std::printf("CSV,%llu,%d,%.3f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%d,%d,%d\n",
                    seed, rankA, scoreAStat, bots[0].r2, bots[0].dmg,
                    bots[0].missions(), bots[0].reachDone ? 1 : 0,
                    bots[0].deaths, bots[0].kills, bots[0].sx, bots[0].sy,
                    bots[0].agent ? bots[0].agent->contestHits() : 0,
                    bots[0].agent ? bots[0].agent->gateEngagedTicks() : 0,
                    bots[0].twoNormalDone ? 1 : 0,
                    bots[0].highDone ? 1 : 0,
                    gameEvHitsA, gameEvDamageA, gameEvDeathsA,
                    bots[0].agent ? bots[0].agent->eventVetoTicks() : 0,
                    bots[0].agent ? bots[0].agent->eventTargetTicks() : 0,
                    bots[0].agent ? bots[0].agent->testEventsInjected() : 0);
    }
    // mix（旧口径）：把本局 A 的得分归因到抽到的对手类型
    if (opt.oppPolicy == OPP_MIX) {
        const int poolN = static_cast<int>(sizeof(kMixPool) / sizeof(kMixPool[0]));
        for (int k = 0; k < poolN; ++k) {
            if (kMixPool[k] == oppPolicy) {
                st.mixScoreA[k] += bots[0].score();
                st.mixGames[k] += 1;
                break;
            }
        }
    }
    // mix3：按席位记录抽到的模式，并统计"该模式在场时 A 的表现"（席位暴露口径，
    // 不是互斥分组：一局三个对手会各记一次）。
    if (opt.oppPolicy == OPP_MIX3) {
        for (int i = 1; i < 4; ++i) {
            const int p = bots[i].policy;
            if (p < 0 || p >= 12) continue;
            ++st.mix3SeatCount[p];
            ++st.mix3PresenceGames[p];
            st.mix3PresenceScoreA[p] += scoreAStat;
            st.mix3PresenceTop2A[p] += (rankA <= 2) ? 1 : 0;
        }
    }
    if (verbose) {
        std::printf("\n--- seed %llu mission=(%d,%d) events:", seed, m.missionX, m.missionY);
        for (std::size_t k = 0; k < m.events.size(); ++k)
            std::printf(" %s@%d", kEvName[m.events[k].type], m.events[k].tAct);
        std::printf(" ---\n");
        for (int i = 0; i < 4; ++i) {
            const Bot& b = bots[i];
            // 可达性诊断：忽略其他车，只按静态墙判断任务点是否连通
            const bool reachable = staticReachable(m, b.sx, b.sy, m.missionX, m.missionY, TICKS);
            std::printf("  %c: score=%6.2f R2=%5.0f D=%6.1f M=%d kills=%2d deaths=%2d reach=%d dReach=%d reachable=%d maxDef=%d\n",
                        b.id, b.score(), b.r2, b.dmg, b.missions(), b.kills, b.deaths,
                        b.reachDone ? 1 : 0,
                        std::abs(b.x - m.missionX) + std::abs(b.y - m.missionY),
                        reachable ? 1 : 0, b.maxStreak);
        }
    }
}

void runAll(int n, unsigned long long s0, bool eventsOn, const RunOpts& opt, Stat& st) {
    for (int k = 0; k < n; ++k) runMatch(s0 + k, k, st, false, eventsOn, opt);
}

// 95% Wilson 区间：名次是 0/1 变量，不能只看点估计。
double wilsonLow(int k, int n) {
    if (n <= 0) return 0.0;
    const double z = 1.959963984540054;
    const double p = static_cast<double>(k) / n;
    const double den = 1.0 + z * z / n;
    const double centre = p + z * z / (2.0 * n);
    const double rad = z * std::sqrt((p * (1.0 - p) + z * z / (4.0 * n)) / n);
    return (centre - rad) / den;
}
double wilsonHigh(int k, int n) {
    if (n <= 0) return 0.0;
    const double z = 1.959963984540054;
    const double p = static_cast<double>(k) / n;
    const double den = 1.0 + z * z / n;
    const double centre = p + z * z / (2.0 * n);
    const double rad = z * std::sqrt((p * (1.0 - p) + z * z / (4.0 * n)) / n);
    return (centre + rad) / den;
}

bool traceSame(const Stat& a, const Stat& b) {
    if (a.traceHashes.size() != b.traceHashes.size()) return false;
    for (std::size_t i = 0; i < a.traceHashes.size(); ++i)
        if (a.traceHashes[i] != b.traceHashes[i]) return false;
    return true;
}

bool parseLongArg(const char* s, long& out) {
    if (!s || !*s) return false;
    char* end = NULL;
    const long v = std::strtol(s, &end, 10);
    if (!end || *end != '\0') return false;
    out = v;
    return true;
}
bool parseIntArg(const char* s, int& out) {
    long v = 0;
    if (!parseLongArg(s, v) || v < -1000000 || v > 1000000) return false;
    out = static_cast<int>(v);
    return true;
}
}  // namespace

int main(int argc, char** argv) {
    int n = 20;
    unsigned long long s0 = 1000ULL;
    if (argc > 1) {
        int v = 0;
        if (!parseIntArg(argv[1], v) || v <= 0) {
            std::fprintf(stderr, "invalid match count: %s\n", argv[1]);
            return 2;
        }
        n = v;
    }
    if (argc > 2) {
        char* end = NULL;
        s0 = std::strtoull(argv[2], &end, 10);
        if (!end || *end != '\0') {
            std::fprintf(stderr, "invalid seed: %s\n", argv[2]);
            return 2;
        }
    }

    bool verbose = false, eventsOn = true;
    RunOpts opt;
    bool depthExplicit = false, contestExplicit = false, spawnExplicit = false;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "v") verbose = true;
        else if (a == "noevent") eventsOn = false;
        else if (a == "opp=agent") opt.oppPolicy = OPP_AGENT;
        else if (a == "opp=collector") opt.oppPolicy = OPP_COLLECTOR;
        else if (a == "opp=rusher") opt.oppPolicy = OPP_RUSHER;
        else if (a == "opp=camper") opt.oppPolicy = OPP_CAMPER;
        else if (a == "opp=random") opt.oppPolicy = OPP_RANDOM;
        else if (a == "opp=tactician") opt.oppPolicy = OPP_TACTICIAN;
        else if (a == "opp=farmer") opt.oppPolicy = OPP_FARMER;
        else if (a == "opp=mcamp") opt.oppPolicy = OPP_MCAMP;
        else if (a == "opp=mad") opt.oppPolicy = OPP_MAD;
        else if (a == "opp=past") opt.oppPolicy = OPP_PAST;
        else if (a == "opp=mix") opt.oppPolicy = OPP_MIX;
        else if (a == "opp=mix3") opt.oppPolicy = OPP_MIX3;
        else if (a == "opp=mix1") opt.oppPolicy = OPP_MIX1;
        else if (a == "opp=mixreal") opt.oppPolicy = OPP_MIXREAL;
        else if (a == "opp=hunter") opt.oppPolicy = OPP_HUNTER;
        else if (a == "opp=hybrid") opt.oppPolicy = OPP_HYBRID;
        else if (a == "opp=shadow") opt.oppPolicy = OPP_SHADOW;
        else if (a == "opp=mission") opt.oppPolicy = OPP_MISSION;
        else if (a.compare(0, 6, "seatB=") == 0) opt.seatPick[0] = policyFromName(a.substr(6));
        else if (a.compare(0, 6, "seatC=") == 0) opt.seatPick[1] = policyFromName(a.substr(6));
        else if (a.compare(0, 6, "seatD=") == 0) opt.seatPick[2] = policyFromName(a.substr(6));
        else if (a == "rot=1") opt.spawnRot = true;   // 席位/出生点轮换
        else if (a == "rot=0") opt.spawnRot = false;
        else if (a == "depthA=1") opt.depthAOnly = true;  // depth 只改 A 席
        else if (a == "depthA=0") opt.depthAOnly = false;
        else if (a == "rankvalue=1") opt.rankValue = true;   // 排名口径 VF（只改 A）
        else if (a == "rankvalue=0") opt.rankValue = false;
        else if (a == "gate=1") opt.gate = true;             // 旧接战门（只改 A）
        else if (a == "gate=0") opt.gate = false;
        else if (a == "economy=1") opt.economy = true;       // 经济接战门（只改 A）
        else if (a == "economy=0") opt.economy = false;
        else if (a.compare(0, 9, "econplan=") == 0) opt.econPlan = atoi(a.c_str() + 9);
        else if (a == "dcap=1") opt.dcap = true;             // D≥500 止盈（只改 A）
        else if (a == "dcap=0") opt.dcap = false;
        else if (a == "race=1") opt.race = true;             // 高价值资源时间竞争门（只改 A）
        else if (a == "race=0") opt.race = false;
        else if (a == "defense=1") opt.defense = true;       // 防御性苟活（只改 A）
        else if (a == "defense=0") opt.defense = false;
        else if (a.compare(0, 6, "highp=") == 0) opt.highPriority = static_cast<float>(atof(a.c_str() + 6));
        else if (a.compare(0, 11, "eventaware=") == 0) opt.eventAware = atoi(a.c_str() + 11);
        else if (a.compare(0, 10, "eventlead=") == 0) opt.eventLead = atoi(a.c_str() + 10);
        else if (a == "evdense=1") opt.eventDense = true;
        else if (a == "evdense=0") opt.eventDense = false;
        else if (a.compare(0, 9, "fortress=") == 0) opt.fortressMode = atoi(a.c_str() + 9);
        else if (a.compare(0, 7, "pepper=") == 0) opt.pepper = atoi(a.c_str() + 7);
        else if (a.compare(0, 10, "unrivaled=") == 0) opt.unrivaled = atoi(a.c_str() + 10);
        else if (a.compare(0, 4, "pan=") == 0) opt.panCaution = atoi(a.c_str() + 4);
        else if (a.compare(0, 6, "split=") == 0) opt.splitEscape = atoi(a.c_str() + 6);
        else if (a.compare(0, 10, "reachbase=") == 0) opt.reachBase = atoi(a.c_str() + 10);
        else if (a.compare(0, 12, "reachmargin=") == 0) opt.reachMargin = atoi(a.c_str() + 12);
        else if (a == "pepperduel=1") opt.scenario = 1;
        else if (a == "unrivaledduel=1") opt.scenario = 2;
        else if (a == "thorneduel=1") opt.scenario = 3;
        else if (a.compare(0, 8, "mission=") == 0) {
            const int v = atoi(a.c_str() + 8);
            opt.missionCoupling = v ? 1.8f : 1.0f;
        }
        else if (a.compare(0, 11, "damagetask=") == 0) opt.damageTask = atoi(a.c_str() + 11);
        else if (a == "fair=1") opt.fairMap = true;
        else if (a == "fair=0") opt.fairMap = false;
        else if (a == "contest=1") { opt.contest = true; contestExplicit = true; }
        else if (a == "contest=0") { opt.contest = false; contestExplicit = true; }
        else if (a == "m0=1") opt.zeroMissionA = true;       // A 的 AI 资格失败压力测试
        else if (a == "m0=0") opt.zeroMissionA = false;
        else if (a == "csv" || a == "csv=1") opt.csv = true;
        else if (a == "selfcheck=1") opt.selfcheck = true;
        else if (a == "repro=1") opt.repro = true;
        else if (a == "strict=1") opt.strict = true;
        else if (a.compare(0, 6, "check=") == 0) opt.check = a.substr(6);
        else if (a == "prod=0" || a == "legacy=1") {
            opt.legacy = true;
            if (!depthExplicit) opt.depth = -1;
            if (!contestExplicit) opt.contest = false;
            if (!spawnExplicit) opt.spawnMode = 0;
        } else if (a == "prod=1" || a == "legacy=0") {
            opt.legacy = false;
            if (!depthExplicit) opt.depth = 0;
            if (!contestExplicit) opt.contest = true;
            if (!spawnExplicit) opt.spawnMode = 2;
        } else if (a.compare(0, 6, "spawn=") == 0) {
            int v = 0;
            if (!parseIntArg(a.c_str() + 6, v) || v < 0 || v > 2) {
                std::fprintf(stderr, "spawn must be 0/1/2\n");
                return 2;
            }
            opt.spawnMode = v;
            spawnExplicit = true;
        } else if (a.compare(0, 10, "lookahead=") == 0) {
            int v = 0;
            if (!parseIntArg(a.c_str() + 10, v) || v < 0 || v > 3) {
                std::fprintf(stderr, "lookahead must be 0..3\n");
                return 2;
            }
            opt.lookahead = v;
        } else if (a.compare(0, 6, "depth=") == 0) {
            int v = 0;
            if (!parseIntArg(a.c_str() + 6, v) || v < 0 || v > 5) {
                std::fprintf(stderr, "depth must be 0..5\n");
                return 2;
            }
            opt.depth = v;
            depthExplicit = true;
        } else if (a == "age=0") opt.resAgeBonus = 0.f;
        else if (a == "age=1") opt.resAgeBonus = 0.1f;  // 复现旧默认，用于消融
        else if (a == "reachfix=0") opt.reachFix = false;
        else if (a == "choke=0") opt.chokeOn = false;
        else if (a == "choke=1") opt.chokeOn = true;
        else if (a == "corridor") opt.corridorMap = true;   // 走廊地图
        else if (a == "oppcost=1") opt.oppCost = true;      // 开采集机会成本（decision_type 候选）
        else if (a == "oppcost=0") opt.oppCost = false;
        else if (a == "rayblock=0") opt.rayBlock = false;   // 关多敌射线
        else {
            std::fprintf(stderr, "unknown option: %s\n", a.c_str());
            return 2;
        }
    }

    // ---- 配置自检 / 复现自检（v9 口径：开关必须改变轨迹）----
    if (opt.scenario != 0 && opt.eventAware == 0) opt.eventAware = 1;
    if (opt.selfcheck || opt.repro) {
        // 20 局会出现“机制触发但动作序列完全相同”的假阴性（尤其 contest），
        // 用 100 局做正对照；仍失败说明该开关在当前场景没有可观测行为改变。
        const int scN = 100;
        RunOpts base = opt;
        base.selfcheck = false;
        base.repro = false;
        base.csv = false;
        RunOpts alt = base;
        bool altEvents = eventsOn;
        if (opt.repro) {
            // 同配置跑两遍，轨迹必须逐位一致。
        } else if (opt.check == "contest") {
            alt.contest = !base.contest;
        } else if (opt.check == "depth") {
            alt.depth = (base.depth == 0) ? 5 : 0;
            alt.depthAOnly = true;
        } else if (opt.check == "gate") {
            alt.gate = !base.gate;
        } else if (opt.check == "economy") {
            alt.economy = !base.economy;
        } else if (opt.check == "dcap") {
            alt.dcap = !base.dcap;
        } else if (opt.check == "race") {
            alt.race = !base.race;
        } else if (opt.check == "m0") {
            alt.zeroMissionA = !base.zeroMissionA;
        } else if (opt.check == "event") {
            altEvents = !eventsOn;
        } else {
            std::fprintf(stderr,
                         "unknown check=%s (contest/depth/gate/economy/dcap/race/m0/event)\n",
                         opt.check.c_str());
            return 2;
        }
        Stat a, b;
        runAll(scN, s0, eventsOn, base, a);
        runAll(scN, s0, opt.repro ? eventsOn : altEvents, alt, b);
        const bool tracesDiffer = !traceSame(a, b);
        bool mech = true;
        if (a.spawnBad > 0 || b.spawnBad > 0) mech = false;
        if (eventsOn && a.evHits <= 0) mech = false;
        if (altEvents && b.evHits <= 0) mech = false;
        if (base.contest && a.contestHitsA <= 0) mech = false;
        if (alt.contest && b.contestHitsA <= 0) mech = false;
        if (base.gate && a.gateTicksA <= 0) mech = false;
        if (alt.gate && b.gateTicksA <= 0) mech = false;
        if (base.economy && a.economyHoldsA <= 0) mech = false;
        if (alt.economy && b.economyHoldsA <= 0) mech = false;
        if (base.dcap && a.dcapBlocksA <= 0) mech = false;
        if (alt.dcap && b.dcapBlocksA <= 0) mech = false;
        if (base.race && a.raceDropsA <= 0) mech = false;
        if (alt.race && b.raceDropsA <= 0) mech = false;
        std::printf("%s n=%d seed=[%llu,%llu] check=%s traceDiff=%d "
                    "eventsHits=%ld contestHitsA=%ld gateTicksA=%ld "
                    "altContestHits=%ld altGateTicks=%ld econHolds=%ld altEconHolds=%ld "
                    "dcapBlocks=%ld altDcapBlocks=%ld raceDrops=%ld/%ld mech=%d\n",
                    opt.repro ? "repro" : "selfcheck", scN, s0, s0 + scN - 1,
                    opt.repro ? "same" : opt.check.c_str(), tracesDiffer ? 1 : 0,
                    a.evHits, a.contestHitsA, a.gateTicksA,
                    b.contestHitsA, b.gateTicksA, a.economyHoldsA, b.economyHoldsA,
                    a.dcapBlocksA, b.dcapBlocksA, a.raceDropsA, b.raceDropsA, mech ? 1 : 0);
        const bool pass = opt.repro ? !tracesDiffer : (tracesDiffer && mech);
        std::printf("%s %s\n", opt.repro ? "repro" : "selfcheck", pass ? "PASS" : "FAIL");
        return pass ? 0 : 4;
    }

    Stat st;
    for (int k = 0; k < n; ++k) runMatch(s0 + k, k, st, verbose, eventsOn, opt);
    const double N = static_cast<double>(st.samples);
    if (st.samples != 4 * n) {
        std::fprintf(stderr, "sample accounting error: got %d, expected %d\n",
                     st.samples, 4 * n);
        return 3;
    }
    static const char* kSpawnName[3] = {"corners", "edge-mid", "random"};
    const char* spawnLabel = opt.corridorMap ? "corners(corridor)" : kSpawnName[opt.spawnMode];
    std::printf("matches=%d samples=%d seed=[%llu,%llu] events=%s opp=%s "
                "spawn=%s rot=%d lookahead=%d depth=%d%s rv=%d gate=%d economy=%d "
                "econplan=%d dcap=%d race=%d defense=%d highp=%.2f eventaware=%d eventlead=%d "
                "contest=%d m0=%d choke=%d map=%s legacy=%d "
                "evdense=%d fortress=%d pepper=%d unrivaled=%d pan=%d split=%d "
                "damagetask=%d mission=%.1f fair=%d\n",
                n, st.samples, s0, s0 + n - 1, eventsOn ? "on" : "off",
                oppName(opt.oppPolicy), spawnLabel, opt.spawnRot ? 1 : 0,
                opt.lookahead, opt.depth, opt.depthAOnly ? "(A-only)" : "",
                opt.rankValue ? 1 : 0, opt.gate ? 1 : 0, opt.economy ? 1 : 0,
                opt.econPlan < 0 ? (rmtc::Config().economy_gate_actual_action ? 1 : 0)
                                 : (opt.econPlan ? 1 : 0),
                opt.dcap ? 1 : 0, opt.race ? 1 : 0, opt.defense ? 1 : 0, opt.highPriority,
                opt.eventAware, opt.eventLead,
                opt.contest ? 1 : 0,
                opt.zeroMissionA ? 1 : 0, opt.chokeOn ? 1 : 0,
                opt.corridorMap ? "corridor" : "random", opt.legacy ? 1 : 0,
                opt.eventDense ? 1 : 0, opt.fortressMode, opt.pepper, opt.unrivaled,
                opt.panCaution, opt.splitEscape, opt.damageTask, opt.missionCoupling,
                opt.fairMap ? 1 : 0);
    std::printf("score      mean=%7.3f  (A=%.3f)\n", st.sumScore / N,
                st.sumScoreA / st.games);
    std::printf("resource   mean=%7.3f\n", st.sumR2 / N);
    std::printf("damage     mean=%7.3f\n", st.sumD / N);
    std::printf("missions   mean=%7.3f\n", st.sumMissions / N);
    std::printf("deaths     per-bot=%.2f   kills per-bot=%.2f\n", st.deaths / N, st.kills / N);
    std::printf("actions    atk=%ld def=%ld move=%ld wait=%ld  maxDefendStreak=%d\n",
                st.attacks, st.defends, st.moves, st.waits, st.maxDefendStreak);
    std::printf("missions   two_normal=%.2f high=%.2f reach=%.2f (A: %.2f/%.2f/%.2f)\n",
                st.mTwo / N, st.mHigh / N, st.mReach / N,
                (double)st.aTwo / st.games, (double)st.aHigh / st.games,
                (double)st.aReach / st.games);
    std::printf("events     hits=%ld damage=%ld deaths=%ld (per-match %.1f / %.1f)\n",
                st.evHits, st.evDamage, st.evDeaths, st.evDamage / (N / 4.0),
                st.evDeaths / (N / 4.0));
    std::printf("eventsA    hits=%ld damage=%ld deaths=%ld (per-match %.2f / %.2f)\n",
                st.evHitsA, st.evDamageA, st.evDeathsA,
                (double)st.evDamageA / st.games, (double)st.evDeathsA / st.games);
    std::printf("eventsType");
    for (int k = 0; k < EV_KIND_N; ++k)
        std::printf(" %s=%ld/%ld/%ld", kEvName[k], st.evTypeHits[k],
                    st.evTypeDamage[k], st.evTypeDeaths[k]);
    std::printf("\n");
    std::printf("eventsDecision vetoA=%ld targetA=%ld injectedA=%ld "
                "pepperTrig=%ld pepperTrapA=%ld unrivaledGrantA=%ld unrivaledAtkA=%ld "
                "damageTaskTicksA=%ld splitEscA=%ld panHoldA=%ld\n",
                st.evVetoA, st.evTargetA, st.evInjectA, st.evPepperTriggers,
                st.pepperTrapsA, st.unrivaledGrantsA, st.unrivaledAttacksA,
                st.damageMissionTicksA, st.splitEscapesA, st.panHoldsA);
    // 建议书《对手策略池》§5 的监控指标
    std::printf("watchdog   deadlocks=%d (%ld ticks)  invalidA=%ld  chokeDeathsA=%ld\n",
                st.deadlockCount, st.deadlockTicks, st.invalidA, st.chokeDeathsA);
    std::printf("time       A mean=%.1fus  max=%ldus (budget 500000us)\n",
                static_cast<double>(st.aTimeUs) / st.games / TICKS, st.aTimeMaxUs);
    std::printf("mechanism  contestHitsA=%ld gateTicksA=%ld economyHoldsA=%ld "
                "economyHoldHitA=%ld dcapBlocksA=%ld raceDropsA=%ld defensiveA=%ld "
                "(0 表示开关没有进入决策路径)\n",
                st.contestHitsA, st.gateTicksA, st.economyHoldsA, st.economyHoldHitA,
                st.dcapBlocksA, st.raceDropsA, st.defensiveA);
    // A 席主指标（v4.4 §1.1：前二率第一、#1 率第二、平均分仅诊断）
    std::printf("rankA      win=%d/%d (%.3f) top2=%d/%d (%.3f) avgRank=%.2f exactTies=%d\n",
                st.teamWin[0], st.games, (double)st.teamWin[0] / st.games,
                st.teamTop2[0], st.games, (double)st.teamTop2[0] / st.games,
                (double)st.teamRankSum[0] / st.games, st.aTieGames);
    std::printf("rankCI     win95=[%.3f,%.3f] top2_95=[%.3f,%.3f]\n",
                wilsonLow(st.teamWin[0], st.games), wilsonHigh(st.teamWin[0], st.games),
                wilsonLow(st.teamTop2[0], st.games), wilsonHigh(st.teamTop2[0], st.games));
    std::printf("A          deaths=%.2f kills=%.2f  R2=%.1f cap120=%d  D=%.1f cap500=%d dmgTask=%d "
                "deathsToWushuang=%ld\n",
                (double)st.aDeaths / st.games, (double)st.aKills / st.games,
                st.aR2Sum / st.games, st.aR2CapGames, st.aDSum / st.games, st.aDCapGames,
                st.aDamageDone, st.aDeathsToWushuang);
    std::printf("reachA     done=%ld/%d avgTick=%.0f  fail=%ld (walled=%ld noTime=%ld contested=%ld) lateDeaths=%ld\n",
                st.aReachDone, st.games,
                st.aReachDone ? (double)st.aReachTickSum / st.aReachDone : 0.0,
                st.aReachFail, st.aReachWalled, st.aReachNoTime, st.aReachContested,
                st.aLateDeaths);
    std::printf("reasonsA   ");
    for (int r = 0; r < 13; ++r) {
        if (st.aReason[r] == 0) continue;
        std::printf("%s=%d ", rmtc::reason_name(r), st.aReason[r]);
    }
    std::printf(" targetSwitch=%.1f/game noTargetRuns=%.1f/game maxNoTargetRun=%d\n",
                (double)st.aTargetSwitches / st.games,
                (double)st.aNoTargetRuns / st.games, st.aMaxNoTargetRun);
    if (opt.oppPolicy == OPP_MIX) {
        const int poolN = static_cast<int>(sizeof(kMixPool) / sizeof(kMixPool[0]));
        for (int k = 0; k < poolN; ++k) {
            if (st.mixGames[k] == 0) continue;
            std::printf("mixPartner %-9s games=%3d  A=%6.3f\n", kMixName[k], st.mixGames[k],
                        st.mixScoreA[k] / st.mixGames[k]);
        }
    }
    if (opt.oppPolicy == OPP_MIX3) {
        std::printf("---- mix3 pool (seat draws) ----\n");
        const int poolN = static_cast<int>(sizeof(kMix3Pool) / sizeof(kMix3Pool[0]));
        for (int k = 0; k < poolN; ++k) {
            const int p = kMix3Pool[k];
            if (st.mix3SeatCount[p] == 0) continue;
            const double g = static_cast<double>(st.mix3PresenceGames[p]);
            std::printf("mix3Partner %-9s seats=%3d (%.1f%%)  A=%6.3f  top2=%.3f\n",
                        kMix3Name[k], st.mix3SeatCount[p],
                        100.0 * st.mix3SeatCount[p] / (3.0 * st.games),
                        st.mix3PresenceScoreA[p] / g,
                        st.mix3PresenceTop2A[p] / g);
        }
    }
    if (opt.spawnMode != 0 || opt.spawnRot) {
        static const char* kCorner[4] = {"TL", "TR", "BL", "BR"};
        std::printf("---- spawn distribution (A, by start quadrant) ----\n");
        for (int c = 0; c < 4; ++c) {
            if (st.rotGames[c] == 0) continue;
            std::printf("quadrant %s games=%3d  score=%6.3f  avgRank=%.2f  top2=%d\n",
                        kCorner[c], st.rotGames[c], st.rotScoreA[c] / st.rotGames[c],
                        (double)st.rotRankSumA[c] / st.rotGames[c], st.rotTop2A[c]);
        }
    }
    // ---- 每队每局得分 + 结束时排名对比（按规则 §3.3 同分序列）----
    // 席位标签：A 恒为本方 Agent；其余按 oppPolicy 标注
    char seatLabel[4][24];
    std::snprintf(seatLabel[0], sizeof(seatLabel[0]), "A(ours)");
    for (int i = 1; i < 4; ++i)
        std::snprintf(seatLabel[i], sizeof(seatLabel[i]), "%c(%s)", 'A' + i,
                      oppName(opt.oppPolicy));
    if (opt.oppPolicy == OPP_MIX) {
        // mix 下 B/C/D 每局策略不同，席位标签注明"抽签"
        for (int i = 1; i < 4; ++i)
            std::snprintf(seatLabel[i], sizeof(seatLabel[i]), "%c(mixDraw)", 'A' + i);
    } else if (opt.oppPolicy == OPP_MIX3) {
        for (int i = 1; i < 4; ++i)
            std::snprintf(seatLabel[i], sizeof(seatLabel[i]), "%c(mix3Draw)", 'A' + i);
    } else if (opt.oppPolicy == OPP_MIX1 || opt.oppPolicy == OPP_MIXREAL) {
        for (int i = 1; i < 4; ++i)
            std::snprintf(seatLabel[i], sizeof(seatLabel[i]), "%c(mixDraw)", 'A' + i);
    }
    // 名次口径排序：平均排名升序（主指标），再按场均得分降序。
    int order[4] = {0, 1, 2, 3};
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j) {
            const double ra = static_cast<double>(st.teamRankSum[order[i]]) / st.games;
            const double rb = static_cast<double>(st.teamRankSum[order[j]]) / st.games;
            bool swap = rb < ra - 1e-9;
            if (!swap && std::abs(rb - ra) <= 1e-9 &&
                st.teamScore[order[j]] > st.teamScore[order[i]])
                swap = true;
            if (swap) { const int t = order[i]; order[i] = order[j]; order[j] = t; }
        }
    std::printf("---- team standings (%d games) ----\n", st.games);
    for (int r = 0; r < 4; ++r) {
        const int i = order[r];
        std::printf("#%d %-12s score=%6.3f  avgRank=%.2f  win=%d  top2=%d (%.1f%%)\n",
                    r + 1, seatLabel[i], st.teamScore[i] / st.games,
                    static_cast<double>(st.teamRankSum[i]) / st.games,
                    st.teamWin[i], st.teamTop2[i],
                    100.0 * st.teamTop2[i] / st.games);
    }
    // v9 的"机制触发"门槛：事件/争夺/接战门开了却一次没触发，说明开关没接上，
    // 这组数据不能拿去做结论。strict=1 时直接返回非零，便于脚本拦截。
    const bool dataValid = (st.invalidA == 0) && (st.spawnBad == 0);
    const bool mechEvents = !eventsOn || st.evHits > 0;
    const bool mechContest = !opt.contest || st.contestHitsA > 0;
    const bool mechGate = !opt.gate || st.gateTicksA > 0;
    std::printf("valid      data=%d invalidA=%ld spawnBad=%d eventsHits=%ld "
                "contestHitsA=%ld gateTicksA=%ld mech=%d\n",
                dataValid ? 1 : 0, st.invalidA, st.spawnBad, st.evHits, st.contestHitsA,
                st.gateTicksA, (mechEvents && mechContest && mechGate) ? 1 : 0);
    // strict 只拦"数据坏了"（非法动作/出生格重叠）；机制计数是诊断，
    // 例如三个抢匪根本不占任务点，contestHits=0 是合法结果，不能判无效。
    if (opt.strict && !dataValid) return 5;
    return 0;
}
