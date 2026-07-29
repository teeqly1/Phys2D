

## 📄 README.md


# Phys2D — физический движок для игр, симуляций и безумных экспериментов

**Phys2D** — это высокопроизводительный 2D-физический движок на C++ с открытым исходным кодом. Он создан для тех, кто хочет добавить в свои игры реалистичную физику, кровь, разрушения, жидкости, электричество, мягкие тела и многое другое.

Движок был написан с нуля одним разработчиком за 4 месяца и уже обгоняет Box2D в ряде бенчмарков. Он распространяется бесплатно, но с простым условием: не продавай его как отдельный продукт, а если модифицируешь — укажи автора.

---

## 🔗 Навигация
- **[Original README](README_EN.md)** — original readme in English
- **[Phys2D for games](README_GAME_EN.md)** - creating games with physics, blood, and destruction in english
- **[Phys2D interactive scene](README_REALTIME_EN.md)** — testing physics in real time english docs
- **[Phys2D documentation (English)](DOCS_EN.md)** - full technical documentation in English
- **[Phys2D для игр](README_GAME.md)** — создание игр с физикой, кровью и разрушением
- **[Phys2D документация (English)](DOCS.md)** — полная техническая документация на русском
- **[Phys2D интерактивная сцена](README_REALTIME.md)** — тестирование физики в реальном времени

---

## 🚀 Что умеет Phys2D

| Область | Возможности |
|---------|-------------|
| **Твёрдые тела** | Круги, прямоугольники, многоугольники, капсулы. SAT, GJK+EPA, непрерывная детекция коллизий (CCD), лучи. |
| **Материалы** | 50 встроенных материалов (сталь, стекло, бетон, резина, дерево, лёд, ткань, нитинол и др.). Таблица 50x50 пар с трением, восстановлением и проводимостью. |
| **Джойнты** | Расстояние, шарнир, пружина, трос, угловой, сварка, мотор, призматический, шкив, шестерня, цепная линия, разрушаемые соединения. |
| **Разрушение** | Вороновская фрагментация, радиальный разлом, точное сохранение импульса (до 10⁻¹⁵), поколения осколков. |
| **Мягкие тела** | Mass-spring, co-rotational FEM, гибридная модель, внутреннее давление, разрыв связей, взаимодействие с твёрдыми телами. |
| **Жидкости (SPH)** | Плотность, давление, вязкость, поверхностное натяжение, волны, связь с твёрдыми телами. |
| **Электричество** | Ток по воде (резисторная сеть), электроды, удары током, джоулев нагрев, электрокинетика, дуга, пьезоэффект, трибозаряд. |
| **Поля** | Гравитация N-тел (алгоритм Барнса-Хата), электростатика, магнетизм, ветер (шум Перлина), Кориолис, центробежная сила, приливные силы, световое давление. |
| **Термодинамика** | Теплопроводность, нагрев трением, тепловое расширение, фазовые переходы, пьезоэффект. |
| **Материаловедение** | Усталость (Баскин), ползучесть (Нортон), релаксация напряжений, вязкоупругость (Кельвин-Фойгт), нелинейная упругость, пластичность, память формы (нитинол), ударная вязкость, коррозия, окисление, радиация, абляция. |
| **Поверхности** | Адгезия, когезия, капиллярные мостики, диффузия, осмос. |
| **Кровь** | Капли, декали, лужи, раны с пульсацией, цвет от кислорода, высыхание, смазывание, капание, отпечатки на телах. |
| **Сенсоры** | Зоны-триггеры с событиями Enter/Stay/Exit, фильтрация по слоям. |
| **Инструменты** | JSON-сериализация, импорт/экспорт (Box2D, Bullet, OBJ), скриптовый язык, консоль, профайлер, бенчмарки, дебаг-отрисовка. |
| **Оптимизация** | SIMD (AVX/SSE), пул потоков, кэш контактов, триг-таблица 65536 градаций, ранний выход из итераций, острова. |

---

## 📦 Что на борту

```
phys2d/
├── include/phys2d/          # Заголовочные файлы (API)
│   ├── World.h              # Ядро: тела, джойнты, шаг симуляции
│   ├── Body.h               # Твёрдые тела
│   ├── Shape.h              # Формы (круг, бокс, полигон, капсула)
│   ├── Collision.h          # SAT, GJK+EPA, манифолды, CCD
│   ├── Constraints.h        # Базовые джойнты
│   ├── Extras.h             # Расширенный API (всё остальное)
│   ├── Advanced.h           # Advanced Lab (ветер, ток, материалы, осколки)
│   ├── Blood.h              # Система крови
│   ├── phys2d_c_api.h       # C-API для FFI
│   └── ...
├── src/                     # Исходники
│   ├── World.cpp
│   ├── Body.cpp
│   ├── Collision.cpp
│   ├── Constraints.cpp
│   ├── Extras_*.cpp         # Разбивка по модулям
│   ├── Advanced.cpp
│   ├── Blood.cpp
│   └── ...
├── examples/                # Демки
│   ├── demo_v2.cpp          # Полная демонстрация всего API
│   ├── realtime_scene.cpp   # Интерактивная сцена (Win32)
│   ├── advanced_lab.cpp     # Advanced Lab (блок 1)
│   ├── sandbox_game.cpp     # Gore Lab — песочница с кровью
│   ├── test_scene.cpp       # Тестовая сцена (1000+ тел)
│   └── ...
├── README.md                # Этот файл
├── README_EN.md             # Документация на английском
├── README_GAME.md           # Для игроделов
├── README_REALTIME.md       # Интерактивная сцена
├── DOCS.md                  # Полная документация по фичам и API
├── LICENSE                  # Лицензия
└── CMakeLists.txt           # Сборка
```

---

## 🛠 Быстрый старт

### Сборка (Windows, MinGW)

```bash
cd /d/projects/phys2d
cmake -B build -G "MinGW Makefiles" \
  -DCMAKE_MAKE_PROGRAM=mingw32-make \
  -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
  -DCMAKE_BUILD_TYPE=Release \
  -DPHYS2D_NATIVE=ON -DPHYS2D_SIMD_AVX=ON -DPHYS2D_FMA=ON \
  -DPHYS2D_BIG_BINARY=ON -DPHYS2D_TABLE_MB=34
cmake --build build -j$(nproc)

# Запуск демок
./build/phys2d_demo_v2.exe
./build/phys2d_demo_v2.exe --benchmark
./build/phys2d_realtime.exe
```

### Без CMake (одной строкой)

```bash
g++ -std=c++17 -O3 -march=native -Iinclude examples/demo_v2.cpp src/*.cpp -pthread -o demo_v2.exe
```

---

## 🧪 Пример кода

```cpp
#include "phys2d/Extras.h"
using namespace phys2d;

Engine engine;

// Включаем модули
engine.fracture().enabled = true;
engine.softBodies().enabled = true;
engine.fluid().enabled = true;
engine.fields().enabled = true;
engine.fields().flags = FIELD_WIND | FIELD_NBODY;

// Шаг симуляции
for (int i = 0; i < 600; ++i) {
    engine.step(1.0 / 60.0);
}

std::puts(engine.statusLine().c_str());
```

---

## 📜 Лицензия

**Phys2D is FREE to use in any project** (including commercial games).  
**You may NOT sell this engine as a standalone product.**  
**If you modify the source code, you must credit the original author (teeqly1).**

[Полный текст лицензии](LICENSE)

---

## 👤 Автор

**[teeqly1](https://github.com/teeqly1)** — разработчик, создатель Phys2D.

---

## ⭐ Поддержать проект

- ⭐ Поставь звезду на GitHub
- 🐛 Сообщай о багах через Issues
- 📖 Пиши документацию и примеры
- 🎮 Делай игры на Phys2D и делись ими
```

---

Have a build.
