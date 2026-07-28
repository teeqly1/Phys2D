// phys2d v2 - tables, templated kernels, command line, scripting, console, benchmarks.
#include "phys2d/Extras.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace phys2d {

// ================================================================ триг-таблица (65536)
namespace {

constexpr size_t kTrigEntries = 65536;

struct TrigTable {
    std::vector<float> sinTable;
    TrigTable() {
        sinTable.resize(kTrigEntries + 1);
        for (size_t i = 0; i <= kTrigEntries; ++i)
            sinTable[i] = (float)std::sin(2.0 * PI * (double)i / (double)kTrigEntries);
    }
};

const TrigTable& trigTable() {
    static const TrigTable t;
    return t;
}

inline real lookupSin(real angle) {
    const TrigTable& t = trigTable();
    real turns = angle / (2.0 * PI);
    turns -= std::floor(turns);
    const real f = turns * (real)kTrigEntries;
    const size_t i = (size_t)f;
    const real frac = f - (real)i;
    const real a = t.sinTable[i];
    const real b = t.sinTable[i + 1];
    return a + (b - a) * frac;   // линейная интерполяция между градациями
}

} // namespace

real tableSin(real angle) { return lookupSin(angle); }
real tableCos(real angle) { return lookupSin(angle + PI * 0.5); }
void tableSinCos(real angle, real& s, real& c) {
    s = lookupSin(angle);
    c = lookupSin(angle + PI * 0.5);
}
size_t trigTableEntries() { return kTrigEntries; }

// ================================================================ кэш контактов
void ContactPointCache::store(uint64_t key, const CachedContact& c) {
    CachedContact entry = c;
    entry.key = key;
    entry.valid = true;
    m_slots[(size_t)(key % kSlots)] = entry;
}

const CachedContact* ContactPointCache::lookup(uint64_t key, real angleA, real angleB,
                                               real angleTolerance) const {
    const CachedContact& slot = m_slots[(size_t)(key % kSlots)];
    if (!slot.valid || slot.key != key) return nullptr;
    // Инвалидация по повороту: геометрия контакта больше не актуальна.
    if (std::fabs(slot.angleA - angleA) > angleTolerance) return nullptr;
    if (std::fabs(slot.angleB - angleB) > angleTolerance) return nullptr;
    return &slot;
}

void ContactPointCache::invalidateOlderThan(uint32_t frame) {
    for (CachedContact& c : m_slots)
        if (c.valid && c.frame < frame) c.valid = false;
}

void ContactPointCache::clear() {
    for (CachedContact& c : m_slots) c = CachedContact{};
}

size_t ContactPointCache::occupancy() const {
    size_t n = 0;
    for (const CachedContact& c : m_slots) if (c.valid) ++n;
    return n;
}

// ================================================================ шаблонные ядра
template <typename T>
T VecT<T>::length() const { return std::sqrt(x * x + y * y); }

template <typename T>
VecT<T> VecT<T>::normalized() const {
    const T len = length();
    return (len > T(1e-12)) ? VecT<T>(x / len, y / len) : VecT<T>();
}

template struct VecT<float>;
template struct VecT<double>;

template <typename T, int Flags>
void ContactKernel<T, Flags>::solve(VecT<T>& va, T& wa, VecT<T>& vb, T& wb,
                                    const VecT<T>& ra, const VecT<T>& rb,
                                    const VecT<T>& normal, T invMassA, T invMassB,
                                    T invIA, T invIB, T bias) {
    // Относительная скорость в точке контакта.
    const VecT<T> pa(va.x - wa * ra.y, va.y + wa * ra.x);
    const VecT<T> pb(vb.x - wb * rb.y, vb.y + wb * rb.x);
    const VecT<T> rel(pb.x - pa.x, pb.y - pa.y);

    T vn = rel.dot(normal);

    if (Flags & KERNEL_RESTITUTION) {
        if (vn < T(-1)) vn += restitution * vn;
    }

    T lambda = -normalMass * (vn + bias);

    if (Flags & KERNEL_WARMSTART) {
        const T old = normalImpulse;
        normalImpulse = std::max(old + lambda, T(0));
        lambda = normalImpulse - old;
    } else {
        lambda = std::max(lambda, T(0));
        normalImpulse += lambda;
    }

    VecT<T> impulse(normal.x * lambda, normal.y * lambda);

    if (Flags & KERNEL_FRICTION) {
        const VecT<T> tangent(-normal.y, normal.x);
        const T vt = rel.dot(tangent);
        T lambdaT = -tangentMass * vt;
        const T maxFriction = friction * normalImpulse;
        const T oldT = tangentImpulse;
        tangentImpulse = std::max(-maxFriction, std::min(oldT + lambdaT, maxFriction));
        lambdaT = tangentImpulse - oldT;
        impulse.x += tangent.x * lambdaT;
        impulse.y += tangent.y * lambdaT;
    }

    va.x -= impulse.x * invMassA;
    va.y -= impulse.y * invMassA;
    wa -= invIA * (ra.x * impulse.y - ra.y * impulse.x);

    vb.x += impulse.x * invMassB;
    vb.y += impulse.y * invMassB;
    wb += invIB * (rb.x * impulse.y - rb.y * impulse.x);
}

template <typename T, int Flags>
const char* ContactKernel<T, Flags>::signature() {
    static const std::string sig = std::string(sizeof(T) == 8 ? "double" : "float") +
                                   "/f" + std::to_string(Flags);
    return sig.c_str();
}

template struct ContactKernel<float, 0>;
template struct ContactKernel<float, 1>;
template struct ContactKernel<float, 2>;
template struct ContactKernel<float, 3>;
template struct ContactKernel<float, 4>;
template struct ContactKernel<float, 5>;
template struct ContactKernel<float, 6>;
template struct ContactKernel<float, 7>;
template struct ContactKernel<double, 0>;
template struct ContactKernel<double, 1>;
template struct ContactKernel<double, 2>;
template struct ContactKernel<double, 3>;
template struct ContactKernel<double, 4>;
template struct ContactKernel<double, 5>;
template struct ContactKernel<double, 6>;
template struct ContactKernel<double, 7>;

const char* kernelSignature(int flags, bool doublePrecision) {
    const int f = flags & 7;
    if (doublePrecision) {
        switch (f) {
            case 0: return ContactKernel<double, 0>::signature();
            case 1: return ContactKernel<double, 1>::signature();
            case 2: return ContactKernel<double, 2>::signature();
            case 3: return ContactKernel<double, 3>::signature();
            case 4: return ContactKernel<double, 4>::signature();
            case 5: return ContactKernel<double, 5>::signature();
            case 6: return ContactKernel<double, 6>::signature();
            default: return ContactKernel<double, 7>::signature();
        }
    }
    switch (f) {
        case 0: return ContactKernel<float, 0>::signature();
        case 1: return ContactKernel<float, 1>::signature();
        case 2: return ContactKernel<float, 2>::signature();
        case 3: return ContactKernel<float, 3>::signature();
        case 4: return ContactKernel<float, 4>::signature();
        case 5: return ContactKernel<float, 5>::signature();
        case 6: return ContactKernel<float, 6>::signature();
        default: return ContactKernel<float, 7>::signature();
    }
}

// ================================================================ командная строка
CommandLineOptions parseCommandLine(int argc, char** argv) {
    CommandLineOptions o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* def) -> std::string {
            if (i + 1 < argc && argv[i + 1][0] != '-') return argv[++i];
            return def;
        };

        if (arg == "--steps") o.steps = std::atoi(next("600").c_str());
        else if (arg == "--dt") o.dt = std::atof(next("0.016666").c_str());
        else if (arg == "--threads") o.threads = std::atoi(next("0").c_str());
        else if (arg == "--bodies") o.bodies = std::atoi(next("1000").c_str());
        else if (arg == "--scene") o.scene = next("");
        else if (arg == "--json") o.outputJson = next("scene.json");
        else if (arg == "--report") o.report = next("report.md");
        else if (arg == "--script") o.script = next("");
        else if (arg == "--benchmark") o.runBenchmark = true;
        else if (arg == "--window") o.headless = false;
        else if (arg == "--no-profile") o.profile = false;
        else if (arg.rfind("--", 0) == 0) {
            const size_t eq = arg.find('=');
            if (eq != std::string::npos) o.extra[arg.substr(2, eq - 2)] = arg.substr(eq + 1);
            else o.extra[arg.substr(2)] = "1";
        }
    }
    return o;
}

std::string commandLineHelp() {
    return
        "phys2d v2\n"
        "  --steps N        число шагов симуляции (600)\n"
        "  --dt X           фиксированный шаг (1/60)\n"
        "  --threads N      потоки (0 = авто)\n"
        "  --bodies N       тел в тестовой сцене\n"
        "  --scene NAME     имя сцены бенчмарка\n"
        "  --benchmark      прогнать все сцены и собрать отчёт\n"
        "  --json FILE      сохранить состояние в JSON\n"
        "  --report FILE    сохранить отчёт по производительности (markdown)\n"
        "  --script FILE    выполнить скрипт перед стартом\n"
        "  --window         оконный режим (если собрано с рендером)\n"
        "  --no-profile     отключить профайлинг\n";
}

// ================================================================ скрипты
namespace {

std::vector<std::string> splitTokens(const std::string& line) {
    std::vector<std::string> tokens;
    std::istringstream ss(line);
    std::string t;
    while (ss >> t) tokens.push_back(t);
    return tokens;
}

std::string trim(const std::string& s) {
    const size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    const size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

} // namespace

void ScriptEngine::attach(World& w) {
    SimulationSystem::attach(w);

    registerCommand("gravity", [this](const std::vector<std::string>& args) -> std::string {
        if (args.size() < 2) return "usage: gravity X Y";
        m_world->setGravity(Vec2(std::atof(args[0].c_str()), std::atof(args[1].c_str())));
        return "ok";
    });
    registerCommand("spawn", [this](const std::vector<std::string>& args) -> std::string {
        if (args.size() < 3) return "usage: spawn circle|box X Y [size]";
        BodyDef def;
        def.type = BodyType::Dynamic;
        const real size = (args.size() > 3) ? std::atof(args[3].c_str()) : 0.4;
        def.shape = (args[0] == "box") ? Shape::box(size, size) : Shape::circle(size * 0.5);
        def.position = Vec2(std::atof(args[1].c_str()), std::atof(args[2].c_str()));
        def.name = "script_body";
        const BodyId id = m_world->createBody(def);
        return "body " + std::to_string(id);
    });
    registerCommand("iterations", [this](const std::vector<std::string>& args) -> std::string {
        if (args.empty()) return "usage: iterations N [positionN]";
        m_world->config().solver.velocityIterations = std::atoi(args[0].c_str());
        if (args.size() > 1) m_world->config().solver.positionIterations = std::atoi(args[1].c_str());
        return "ok";
    });
    registerCommand("integrator", [this](const std::vector<std::string>& args) -> std::string {
        if (args.empty()) return "usage: integrator euler|verlet|rk4";
        if (args[0] == "verlet") m_world->setIntegrator(Integrator::Verlet);
        else if (args[0] == "rk4") m_world->setIntegrator(Integrator::RK4);
        else m_world->setIntegrator(Integrator::SymplecticEuler);
        return "ok";
    });
    registerCommand("module", [this](const std::vector<std::string>& args) -> std::string {
        if (args.size() < 2) return "usage: module BIT 0|1";
        const uint32_t bit = 1u << (uint32_t)std::atoi(args[0].c_str());
        m_world->enableModule(bit, std::atoi(args[1].c_str()) != 0);
        return "ok";
    });
    registerCommand("step", [this](const std::vector<std::string>& args) -> std::string {
        const int n = args.empty() ? 1 : std::atoi(args[0].c_str());
        for (int i = 0; i < n; ++i) m_world->step();
        return "stepped " + std::to_string(n);
    });
    registerCommand("stats", [this](const std::vector<std::string>&) -> std::string {
        std::ostringstream out;
        out << "bodies " << m_world->bodyCount() << ", contacts " << m_world->contactCount()
            << ", awake " << m_world->awakeCount() << ", energy " << m_world->totalEnergy();
        return out.str();
    });
    registerCommand("save", [this](const std::vector<std::string>& args) -> std::string {
        const std::string path = args.empty() ? "scene.json" : args[0];
        return m_world->saveToFile(path) ? ("saved " + path) : "save failed";
    });
    registerCommand("print", [](const std::vector<std::string>& args) -> std::string {
        std::string out;
        for (const std::string& a : args) out += a + " ";
        return out;
    });
}

void ScriptEngine::registerCommand(const std::string& cmdName,
                                   std::function<std::string(const std::vector<std::string>&)> fn) {
    m_cmds[cmdName] = std::move(fn);
}

void ScriptEngine::setVariable(const std::string& varName, real value) { m_vars[varName] = value; }

real ScriptEngine::variable(const std::string& varName) const {
    const auto it = m_vars.find(varName);
    return (it == m_vars.end()) ? 0.0 : it->second;
}

// Простой рекурсивный вычислитель: + - * / и скобки, имена переменных.
real ScriptEngine::evaluate(const std::string& expr) const {
    struct Parser {
        const std::string& s;
        size_t i = 0;
        const std::unordered_map<std::string, real>& vars;

        void skip() { while (i < s.size() && std::isspace((unsigned char)s[i])) ++i; }

        real parseExpr() {
            real value = parseTerm();
            for (;;) {
                skip();
                if (i < s.size() && (s[i] == '+' || s[i] == '-')) {
                    const char op = s[i++];
                    const real rhs = parseTerm();
                    value = (op == '+') ? value + rhs : value - rhs;
                } else return value;
            }
        }
        real parseTerm() {
            real value = parseAtom();
            for (;;) {
                skip();
                if (i < s.size() && (s[i] == '*' || s[i] == '/')) {
                    const char op = s[i++];
                    const real rhs = parseAtom();
                    value = (op == '*') ? value * rhs : (std::fabs(rhs) > 1e-12 ? value / rhs : 0.0);
                } else return value;
            }
        }
        real parseAtom() {
            skip();
            if (i < s.size() && s[i] == '(') {
                ++i;
                const real v = parseExpr();
                skip();
                if (i < s.size() && s[i] == ')') ++i;
                return v;
            }
            if (i < s.size() && (std::isalpha((unsigned char)s[i]) || s[i] == '_')) {
                const size_t start = i;
                while (i < s.size() && (std::isalnum((unsigned char)s[i]) || s[i] == '_')) ++i;
                const std::string ident = s.substr(start, i - start);
                const auto it = vars.find(ident);
                return (it == vars.end()) ? 0.0 : it->second;
            }
            const size_t start = i;
            while (i < s.size() && (std::isdigit((unsigned char)s[i]) || s[i] == '.' ||
                                    s[i] == '-' || s[i] == '+' || s[i] == 'e' || s[i] == 'E')) {
                if ((s[i] == '-' || s[i] == '+') && i != start && s[i - 1] != 'e' && s[i - 1] != 'E') break;
                ++i;
            }
            return (start == i) ? 0.0 : std::atof(s.substr(start, i - start).c_str());
        }
    };

    Parser p{expr, 0, m_vars};
    return p.parseExpr();
}

std::string ScriptEngine::runLine(const std::string& rawLine) {
    std::string line = trim(rawLine);
    if (line.empty() || line[0] == '#' || line.rfind("--", 0) == 0) return "";

    // Присваивание: name = выражение
    const size_t eq = line.find('=');
    if (eq != std::string::npos && line.find("==") == std::string::npos) {
        const std::string lhs = trim(line.substr(0, eq));
        const std::string rhs = trim(line.substr(eq + 1));
        if (!lhs.empty() && (std::isalpha((unsigned char)lhs[0]) || lhs[0] == '_') &&
            lhs.find(' ') == std::string::npos) {
            m_vars[lhs] = evaluate(rhs);
            return lhs + " = " + std::to_string(m_vars[lhs]);
        }
    }

    std::vector<std::string> tokens = splitTokens(line);
    if (tokens.empty()) return "";

    // Условное исполнение: if <expr> then <command>
    if (tokens[0] == "if") {
        const size_t thenPos = line.find(" then ");
        if (thenPos == std::string::npos) return "usage: if EXPR then COMMAND";
        const real cond = evaluate(line.substr(2, thenPos - 2));
        return (cond != 0.0) ? runLine(line.substr(thenPos + 6)) : "";
    }
    // Цикл: repeat N <command>
    if (tokens[0] == "repeat" && tokens.size() >= 3) {
        const int n = std::min(100000, std::max(0, (int)evaluate(tokens[1])));
        const size_t pos = line.find(tokens[2]);
        std::string out;
        for (int i = 0; i < n; ++i) out = runLine(line.substr(pos));
        return out;
    }

    const auto cmd = m_cmds.find(tokens[0]);
    if (cmd == m_cmds.end()) return "unknown command: " + tokens[0];

    std::vector<std::string> args(tokens.begin() + 1, tokens.end());
    for (std::string& a : args) {
        const auto v = m_vars.find(a);
        if (v != m_vars.end()) a = std::to_string(v->second);
    }
    return cmd->second(args);
}

std::string ScriptEngine::execute(const std::string& source) {
    std::istringstream ss(source);
    std::string line, out;
    while (std::getline(ss, line)) {
        const std::string r = runLine(line);
        if (!r.empty()) out += r + "\n";
    }
    return out;
}

bool ScriptEngine::executeFile(const std::string& path, std::string* out) {
    std::ifstream f(path);
    if (!f) { PHYS2D_ERROR(ErrorCode::IoFailure, path); return false; }
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string result = execute(ss.str());
    if (out) *out = result;
    return true;
}

void ScriptEngine::update(real) {}

// ================================================================ консоль
void Console::pushCommand(const std::string& cmd) { m_queue.push_back(cmd); }

bool Console::pollFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    bool any = false;
    while (std::getline(f, line)) {
        if (!trim(line).empty()) { m_queue.push_back(line); any = true; }
    }
    f.close();
    if (any) std::ofstream(path, std::ios::trunc);   // очищаем очередь команд
    return any;
}

void Console::update(real) {
    if (!script || m_queue.empty()) return;
    for (const std::string& cmd : m_queue) {
        const std::string result = script->execute(cmd);
        m_log.push_back(cmd + " -> " + trim(result));
        if (m_log.size() > 512) m_log.erase(m_log.begin());
    }
    m_queue.clear();
}

// ================================================================ бенчмарки
namespace {

void addGround(World& w, real halfWidth = 30.0) {
    BodyDef def;
    def.type = BodyType::Static;
    def.shape = Shape::box(halfWidth * 2.0, 1.0);
    def.position = Vec2(0.0, -0.5);
    def.material.staticFriction = 0.8;
    def.name = "ground";
    w.createBody(def);

    for (int side = -1; side <= 1; side += 2) {
        BodyDef wall;
        wall.type = BodyType::Static;
        wall.shape = Shape::box(1.0, 40.0);
        wall.position = Vec2(halfWidth * side, 20.0);
        wall.name = "wall";
        w.createBody(wall);
    }
}

BodyId spawn(World& w, const Shape& s, const Vec2& p, real density = 1.0) {
    BodyDef def;
    def.type = BodyType::Dynamic;
    def.shape = s;
    def.position = p;
    def.material.density = density;
    return w.createBody(def);
}

} // namespace

BenchmarkSuite::BenchmarkSuite() {
    name = "phys2d.benchmarks";

    addScene("circles_1000", [](World& w) {
        addGround(w);
        for (int i = 0; i < 1000; ++i)
            spawn(w, Shape::circle(0.14 + 0.0001 * (i % 40)),
                  Vec2(-24.0 + (i % 60) * 0.8, 1.0 + (i / 60) * 0.7));
    });

    addScene("boxes_500", [](World& w) {
        addGround(w);
        for (int i = 0; i < 500; ++i)
            spawn(w, Shape::box(0.35, 0.35), Vec2(-20.0 + (i % 50) * 0.8, 1.0 + (i / 50) * 0.6));
    });

    addScene("pyramid", [](World& w) {
        addGround(w);
        const int rows = 28;
        for (int row = 0; row < rows; ++row)
            for (int col = 0; col <= row; ++col)
                spawn(w, Shape::box(0.5, 0.5),
                      Vec2((col - row * 0.5) * 0.54, (rows - row) * 0.55 + 0.5));
    });

    addScene("polygons_200", [](World& w) {
        addGround(w);
        for (int i = 0; i < 200; ++i)
            spawn(w, Shape::regularPolygon(3 + (i % 6), 0.3, (real)i * 0.1),
                  Vec2(-18.0 + (i % 40) * 0.9, 1.0 + (i / 40) * 1.1));
    });

    addScene("capsules_150", [](World& w) {
        addGround(w);
        for (int i = 0; i < 150; ++i)
            spawn(w, Shape::capsule(0.16, 0.6), Vec2(-16.0 + (i % 30) * 1.1, 1.0 + (i / 30) * 1.0));
    });

    addScene("mixed_stack", [](World& w) {
        addGround(w);
        for (int i = 0; i < 600; ++i) {
            const Vec2 p(-20.0 + (i % 40) * 1.0, 1.0 + (i / 40) * 0.8);
            switch (i % 4) {
                case 0: spawn(w, Shape::circle(0.25), p); break;
                case 1: spawn(w, Shape::box(0.45, 0.45), p); break;
                case 2: spawn(w, Shape::capsule(0.15, 0.5), p); break;
                default: spawn(w, Shape::regularPolygon(5, 0.3, 0.0), p); break;
            }
        }
    });

    addScene("chains", [](World& w) {
        addGround(w);
        for (int chain = 0; chain < 20; ++chain) {
            BodyId prev = INVALID_BODY;
            for (int link = 0; link < 15; ++link) {
                const Vec2 p(-18.0 + chain * 1.9, 24.0 - link * 0.6);
                const BodyId id = spawn(w, Shape::box(0.2, 0.5), p);
                if (link == 0) w.body(id)->setType(BodyType::Static);
                if (prev != INVALID_BODY)
                    w.createRevolute(prev, id, Vec2(0.0, -0.3), Vec2(0.0, 0.3));
                prev = id;
            }
        }
    });

    addScene("ragdolls", [](World& w) {
        addGround(w);
        for (int i = 0; i < 12; ++i) {
            RagdollConfig cfg;
            cfg.position = Vec2(-20.0 + i * 3.4, 8.0);
            cfg.scale = 1.0;
            createRagdoll(w, cfg);
        }
    });

    addScene("fracture", [](World& w) {
        addGround(w);
        for (int i = 0; i < 120; ++i)
            spawn(w, Shape::box(0.8, 0.8), Vec2(-15.0 + (i % 20) * 1.5, 2.0 + (i / 20) * 1.4));
        for (int i = 0; i < 20; ++i) {
            const BodyId id = spawn(w, Shape::circle(0.4), Vec2(-12.0 + i * 1.3, 26.0), 6.0);
            w.body(id)->velocity = Vec2(0.0, -28.0);
        }
    });

    addScene("tumbler", [](World& w) {
        BodyDef def;
        def.type = BodyType::Kinematic;
        def.shape = Shape::gear(12, 6.0, 8.0);
        def.position = Vec2(0.0, 10.0);
        def.angularVelocity = 1.2;
        def.name = "tumbler";
        w.createBody(def);
        addGround(w);
        for (int i = 0; i < 400; ++i)
            spawn(w, Shape::circle(0.2), Vec2(-4.0 + (i % 20) * 0.42, 6.0 + (i / 20) * 0.45));
    });

    addScene("terrain_run", [](World& w) {
        buildNoiseTerrain(w, -30.0, 30.0, 0.9, 3.5, 17);
        for (int i = 0; i < 300; ++i)
            spawn(w, Shape::circle(0.22), Vec2(-24.0 + (i % 30) * 1.6, 12.0 + (i / 30) * 0.8));
    });
}

void BenchmarkSuite::addScene(const std::string& sceneName, SceneBuilder fn) {
    m_scenes.emplace_back(sceneName, std::move(fn));
}

std::vector<std::string> BenchmarkSuite::sceneNames() const {
    std::vector<std::string> names;
    for (const auto& s : m_scenes) names.push_back(s.first);
    return names;
}

BenchmarkResult BenchmarkSuite::run(const std::string& sceneName, int steps, real dt) {
    BenchmarkResult r;
    r.scene = sceneName;
    r.steps = steps;

    const auto it = std::find_if(m_scenes.begin(), m_scenes.end(),
                                 [&](const std::pair<std::string, SceneBuilder>& s) { return s.first == sceneName; });
    if (it == m_scenes.end()) {
        PHYS2D_ERROR(ErrorCode::NotFound, sceneName);
        r.stable = false;
        return r;
    }

    WorldConfig cfg;
    cfg.fixedTimeStep = dt;
    World w(cfg);
    it->second(w);
    r.bodies = (int)w.bodyCount();

    const real startEnergy = w.totalEnergy();
    std::vector<double> times;
    times.reserve((size_t)steps);
    double contactsSum = 0.0;

    for (int i = 0; i < steps; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        w.step(dt);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        times.push_back(ms);
        contactsSum += (double)w.contactCount();
    }

    const ProfileData& pd = w.profile();
    r.broadMs = pd.msAverage[(size_t)Stage::BroadPhase];
    r.narrowMs = pd.msAverage[(size_t)Stage::NarrowPhase];
    r.solveMs = pd.msAverage[(size_t)Stage::SolveVelocity] + pd.msAverage[(size_t)Stage::SolvePosition];

    for (double t : times) {
        r.totalMs += t;
        r.maxStepMs = std::max(r.maxStepMs, t);
    }
    r.avgStepMs = times.empty() ? 0.0 : r.totalMs / (double)times.size();
    r.contactsAvg = times.empty() ? 0.0 : contactsSum / (double)times.size();

    std::vector<double> sorted = times;
    std::sort(sorted.begin(), sorted.end());
    if (!sorted.empty()) r.p95StepMs = sorted[(size_t)((double)sorted.size() * 0.95) % sorted.size()];

    const real endEnergy = w.totalEnergy();
    r.energyDrift = (double)(endEnergy - startEnergy);
    r.stable = std::isfinite((double)endEnergy) && std::fabs(r.energyDrift) < 1e12;
    return r;
}

std::vector<BenchmarkResult> BenchmarkSuite::runAll(int steps, real dt) {
    std::vector<BenchmarkResult> results;
    for (const auto& s : m_scenes) results.push_back(run(s.first, steps, dt));
    return results;
}

std::string BenchmarkSuite::report(const std::vector<BenchmarkResult>& results) const {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "# phys2d v2 - отчёт по производительности\n\n";
    out << "| Сцена | Тел | Шагов | Средний шаг, мс | p95, мс | Макс, мс | Broad | Narrow | Solve | Контакты | FPS | Стабильно |\n";
    out << "|---|---|---|---|---|---|---|---|---|---|---|---|\n";

    double totalMs = 0.0;
    for (const BenchmarkResult& r : results) {
        totalMs += r.totalMs;
        const double fps = (r.avgStepMs > 0.0) ? 1000.0 / r.avgStepMs : 0.0;
        out << "| " << r.scene << " | " << r.bodies << " | " << r.steps << " | "
            << r.avgStepMs << " | " << r.p95StepMs << " | " << r.maxStepMs << " | "
            << r.broadMs << " | " << r.narrowMs << " | " << r.solveMs << " | "
            << r.contactsAvg << " | " << fps << " | " << (r.stable ? "да" : "нет") << " |\n";
    }
    out << "\nВсего сцен: " << results.size() << ", суммарное время: " << totalMs << " мс\n";
    out << "Таблицы: тригонометрия " << trigTableEntries() << " градаций, материалов "
        << materialCount() << ", пар " << materialCount() * materialCount()
        << ", статические таблицы " << (staticTableBytes() / 1024) << " КБ\n";
    return out.str();
}

std::string BenchmarkSuite::csv(const std::vector<BenchmarkResult>& results) const {
    std::ostringstream out;
    out << "scene,bodies,steps,avg_ms,p95_ms,max_ms,broad_ms,narrow_ms,solve_ms,contacts,energy_drift,stable\n";
    for (const BenchmarkResult& r : results) {
        out << r.scene << "," << r.bodies << "," << r.steps << "," << r.avgStepMs << ","
            << r.p95StepMs << "," << r.maxStepMs << "," << r.broadMs << "," << r.narrowMs << ","
            << r.solveMs << "," << r.contactsAvg << "," << r.energyDrift << ","
            << (r.stable ? 1 : 0) << "\n";
    }
    return out.str();
}

} // namespace phys2d
