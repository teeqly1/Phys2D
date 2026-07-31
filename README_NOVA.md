EN/RU DOCS

RU:

# PHYS2D NOVA - пакет расширений (12 подсистем)

Все 12 пунктов из списка реализованы как модуль `nova` поверх существующего phys2d.
Ничего в старом коде не сломано: это новый заголовок + 4 файла реализации + демо.

## Файлы

```
include/phys2d/Nova.h      - весь публичный API (namespace phys2d::nova)
src/Nova_core.cpp          - материалы, 2.5D-слои, генерация уровней, редактор, энергопрофили
src/Nova_sim.cpp           - ткань/волосы, DEM-частицы, погода, транспорт, океан
src/Nova_net.cpp           - сетевая физика, акустика, NovaSuite
src/Nova_script.cpp        - Lua-подобная VM (лексер + парсер + интерпретатор), 28 нативных функций
examples/nova_demo.cpp     - демонстрация всех 12 подсистем с замерами
```

## Сборка (MSYS2 MINGW64)

```bash
cd /d/projects/phys2d
mkdir -p build
g++ -std=c++17 -O2 -march=native -Iinclude examples/nova_demo.cpp src/*.cpp -pthread -o build/nova_demo.exe
./build/nova_demo.exe --steps 240
```

ВНИМАНИЕ: `src/embedded_tables.S` в командную строку g++ не добавлять.

## 1. Сетевая физика (Replication)

* `ServerAuthority` - авторитетный сервер: очередь входов, `step(dt)`, кольцевая история снапшотов (по умолчанию 64 тика).
* `SnapshotCodec` - differential encoding: квантование (позиция 1/512 м, угол 1/2048 рад, скорость 1/256 м/с) + zigzag varint + пропуск не изменившихся тел.
  Замер на 40 телах x 240 тиков: **270 720 B -> 69 616 B, экономия 74.3%**, средний пакет 290 B.
* `ClientPredictor` - client-side prediction: локальные входы применяются сразу, при получении снапшота выполняется реконсиляция и переигрывание неподтверждённых входов. Ошибка реконсиляции в демо **0.0009 м**.
* `InterpolationBuffer` - буфер интерполяции для рендера (задержка по умолчанию 100 мс).
* Lag compensation: `rewindQueryPoint(tick, point)` откатывает мир на прошлый тик, делает проверку попадания и восстанавливает состояние (в демо hitscan на тике 220 = HIT).

## 2. 2.5D

`DepthWorld`: у каждого тела есть слой с параметрами `z`, `thickness`, `parallax`, `tint`, `fog`.
Физика остаётся 2D, но столкновения между слоями отключаются через `setContactFilter` (`linkLayers(a, b, false)`).
`drawOrder()` возвращает тела отсортированными от дальних к ближним, `scaleFor(layer)` даёт масштаб перспективы (0.77 для заднего плана, 1.18 для переднего).

## 3. Визуальный редактор

`EditorCore`: пауза (`setPaused`), выбор тела мышью (`pick`), перетаскивание (`drag` - в паузе телепорт, в игре скоростное следование), удаление и дублирование, палитра из 9 объектов (дерево/сталь/стекло/камень/резина/платформа/рампа/ткань/песок, `addPaletteItem` для своих), live-настройка гравитации, трения, упругости, итераций солвера и `timeScale`, undo/redo на 32 шага через JSON, `saveScene`/`loadScene`, `statusLine()` для строки состояния.

## 4. Материалы

48 материалов, у каждого:
* физика (плотность, трение статическое/динамическое, упругость);
* внешний вид: `color`, `accent`, `roughness`, `gloss`, `emissive` (лава светится), `transparency` (стекло 0.85);
* звук: базовая частота, затухание, поглощение, частота шага;
* разрушение: `FractureStyle` = Shards / Splinters (дерево - щепки) / Dent (металл - вмятина) / Crumble / Shatter (стекло - осколки, 26 шт) / Tear / Melt (лава), плюс `toughness` и `shardCount`.

`impactHz(a, b, impulse)` даёт частоту удара: стекло/сталь при 40 Н*с -> 1895 Гц, дуб/дуб -> 501 Гц.

## 5. Процедурная генерация

* `ProcGen::buildCave` - клеточный автомат (правило 4-5, 5 проходов сглаживания, герметичные границы), горизонтальные ряды клеток сливаются в один бокс: 96x48 -> **296 тел** вместо 4608.
* `ProcGen::buildMaze` - DFS-раскопка с шагом 2 и опциональным braid (удаление тупиков): 21x15 -> 72 тела.
* `ProcGen::buildCity` - кварталы: асфальт, стены, перекрытия каждые 3.2 м, стеклянные окна (динамические), крыши, обломки: 81 тело.
* `ProcGen::findSpawn` - поиск безопасной точки появления.

## 6. Скрипты

`LuaVM` - самостоятельная реализация Lua-подобного языка (внешних зависимостей нет): комментарии `--`, строки, числа с экспонентой, `local`, `if/elseif/else`, `while`, `repeat/until`, числовой `for`, `function`, `return`, `break`, операторы `and/or/not`, конкатенация `..`, `^`, защита от переполнения стека (128) и вечных циклов (5 млн итераций).

28 нативных функций: `print`, `tostring`, `tonumber`, `math.sin/cos/tan/sqrt/abs/floor/ceil/min/max/atan2/random`, `set_gravity`, `body_count`, `material_count`, `material_name`, `spawn_box`, `spawn_circle`, `spawn_static`, `body_x`, `body_y`, `body_speed`, `apply_force`, `apply_impulse`, `remove_body`, `step`. Свои функции добавляются через `registerFunction`.

Ограничение: табличных конструкторов `{}` нет.

## 7. Ткани и волосы

`ClothSystem` - Verlet-интегрирование, структурные + диагональные связи, разрыв по `tearFactor`, ветер с турбулентностью, столкновения с телами, пришивание к телам (`pinToBody`).
Готовые формы: `createFlag`, `createCurtain`, `createHair` (волосы получают ослабленный ветер и мягкие связи через узел), `createRopeStrand`.
В демо: 3 куска, 156 узлов, 0 разрывов, флаг колышется на ветру (6.5, 0).

## 8. DEM-частицы и погода

`GrainSystem` - контакт Герца `Fn = k*d^1.5 - c*vn*sqrt(d)` + касательное трение, хеш-сетка (ячейка 0.2 м), взаимодействие с телами, взрывы (`applyExplosion`).
Виды: песок, гравий, снег, искры, дым (всплывает), угли, дождь, град.
`WeatherSystem`: Clear / Rain / Snowfall / Hail / Sandstorm с интенсивностью и ветром.
В демо: 1856 живых частиц, 523 контакта за шаг, 1440 капель дождя.

## 9. Транспорт

* `WheeledVehicle` - подвеска-стойка: пружина `k` считается из массы и заданной просадки (`ridePercent`), демпфер по коэффициенту от критического, отбойник, проекция колеса вдоль оси стойки после шага (колёса не выпадают). В демо: пружина 222 671 Н/м, демпфер 16 600 Н*с/м, максимум 46 км/ч, ход подвески использован на 60%.
* `TrackedVehicle` - танк: корпус, башня на шарнире, опорные катки, тяга по мощности, поворот разностью гусениц. В демо: 7.7 м/с, давление на грунт 72.7 кПа.
* `Aircraft` - крылья с площадью, наклоном кривой подъёмной силы, углом сваливания и рулями; учёт ветра, срыв потока, индуктивное сопротивление `cD = cD0 + 0.06*cL^2`; режим вертолёта (`rotorcraft`).

## 10. Океан

`Ocean` - сумма волн Герстнера, спектр Пирсона-Московица (`makeSea(waves, windSpeed)`), орбитальные скорости, мелководное усиление у берега (`setShore`), индекс обрушения.
Плавучесть по Архимеду считается по 8 полоскам корпуса + гидродинамическое сопротивление + угловое демпфирование. В демо: 9 волн, 6 плавающих тел, качка корпуса 1.20 м.

## 11. Мобильная оптимизация

`detectCpu()` определяет SSE2/SSE4/AVX/AVX2/NEON и число ядер, `simdBackend()` возвращает активный бэкенд (на ARM автоматически neon).
`PowerManager` - три профиля: Desktop / Mobile / Battery. Считает активность мира (доля не спящих тел), растягивает шаг при простое, снижает число итераций солвера и пропускает кадры рендера.
Замер: Desktop ~26.8 Вт (120/120 кадров), Mobile ~3.1 Вт (60/120), Battery ~1.6 Вт (60/120).

## 12. Акустика

`Acoustics`: реверберация по Сабину (`RT60 = 0.161*V/(S*a)`, поглощение берётся из материала стен), затухание по расстоянию, окклюзия (луч слушатель-источник против стен), доплер по проекции скоростей, звук шага зависит от поверхности.
Замер: RT60 0.70 с при 3 стенах; ближний удар 1895 Гц gain 1.000 задержка 6 мс; тот же удар за бетонной стеной - в 127 раз тише; шаг по снегу 40 Гц; источник, удаляющийся на 60 м/с, - 0.85x по частоте.

## NovaSuite

`NovaSuite` подключает все подсистемы к одному миру одним вызовом `attachAll(world)`, обновляет их `update(dt)` и печатает сводку `report()`.


EN:

# PHYS2D NOVA - Extension Pack (12 Subsystems)

All 12 items on the list are implemented as the `nova` module on top of the existing phys2d.
Nothing in the old code is broken: this is a new header + 4 implementation files + a demo.

## Files

```
include/phys2d/Nova.h - the entire public API (namespace phys2d::nova)
src/Nova_core.cpp - materials, 2.5D layers, level generation, editor, energy profiles
src/Nova_sim.cpp - cloth/hair, DEM particles, weather, vehicles, ocean
src/Nova_net.cpp - network physics, acoustics, NovaSuite
src/Nova_script.cpp - Lua-like VM (lexer + parser + interpreter), 28 native functions
examples/nova_demo.cpp - demonstration of all 12 subsystems with measurements
```

## Build (MSYS2 MINGW64)

```bash
cd /d/projects/phys2d
mkdir -p build
g++ -std=c++17 -O2 -march=native -Iinclude examples/nova_demo.cpp src/*.cpp -pthread -o build/nova_demo.exe
./build/nova_demo.exe --steps 240
```

WARNING: Do not add `src/embedded_tables.S` to the g++ command line.

## 1. Network Physics (Replication)

* `ServerAuthority` - authority server: input queue, `step(dt)`, circular snapshot history (default 64 ticks).
* `SnapshotCodec` - differential encoding: quantization (position 1/512 m, angle 1/2048 rad, velocity 1/256 m/s) + zigzag varint + skip unchanged bodies.
Measurement on 40 bodies x 240 ticks: **270,720 B -> 69,616 B, 74.3% savings**, average packet size 290 B.
* `ClientPredictor` - client-side prediction: local inputs are applied immediately; upon receiving a snapshot, reconciliation and replay of unconfirmed inputs are performed. Reconciliation error in the demo was **0.0009 m**.
* `InterpolationBuffer` - interpolation buffer for rendering (default latency is 100 ms).
* Lag compensation: `rewindQueryPoint(tick, point)` rolls the world back to the previous tick, performs a hit check, and restores the state (in the demo, hitscan at tick 220 = HIT).

## 2. 2.5D

`DepthWorld`: Each body has a layer with parameters `z`, `thickness`, `parallax`, `tint`, and `fog`.
Physics remains 2D, but collisions between layers are disabled using `setContactFilter` (`linkLayers(a, b, false)`).
`drawOrder()` returns bodies sorted from farthest to nearest, `scaleFor(layer)` returns the perspective scale (0.77 for the background, 1.18 for the foreground).

## 3. Visual Editor

`EditorCore`: pause (`setPaused`), mouse body selection (`pick`), dragging (`drag` - teleport when paused, high-speed tracking in-game), deletion and duplication, 9-object palette (wood/steel/glass/stone/rubber/platform/ramp/fabric/sand, `addPaletteItem` for custom items), live adjustment of gravity, friction, elasticity, solver iterations, and `timeScale`, undo/redo in 32 steps via JSON, `saveScene`/`loadScene`, `statusLine()` for the status bar.

## 4. Materials

48 materials, each with:
* physics (density, static/dynamic friction, elasticity);
* Appearance: `color`, `accent`, `roughness`, `gloss`, `emissive` (lava glows), `transparency` (glass 0.85);
* Sound: base frequency, attenuation, absorption, step frequency;
* Destruction: `FractureStyle` = Shards / Splinters (wood - splinters) / Dent (metal - dents) / Crumble / Shatter (glass - shards, 26 pcs) / Tear / Melt (lava), plus `toughness` and `shardCount`.

`impactHz(a, b, impulse)` gives the impact frequency: glass/steel at 40 N*s -> 1895 Hz, oak/oak -> 501 Hz.

## 5. Procedural Generation

* `ProcGen::buildCave` - cellular automaton (4-5 rule, 5 smoothing passes, tight boundaries), horizontal rows of cells merge into a single box: 96x48 -> **296 bodies** instead of 4608.
* `ProcGen::buildMaze` - DFS excavation with a step of 2 and optional braid (dead-end removal): 21x15 -> 72 bodies.
* `ProcGen::buildCity` - blocks: asphalt, walls, slabs every 3.2 m, glass windows (dynamic), roofs, debris: 81 bodies.
* `ProcGen::findSpawn` - search for a safe spawn point.

## 6. Scripts

`LuaVM` is a standalone implementation of a Lua-like language (no external dependencies): `--` comments, strings, numbers with exponents, `local`, `if/elseif/else`, `while`, `repeat/until`, numeric `for`, `function`, `return`, `break`, `and/or/not` operators, `..` concatenation, `^`, stack overflow protection (128) and infinite loops (5 million iterations).

28 native functions: `print`, `tostring`, `tonumber`, `math.sin/cos/tan/sqrt/abs/floor/ceil/min/max/atan2/random`, `set_gravity`, `body_count`, `material_count`, `material_name`, `spawn_box`, `spawn_circle`, `spawn_static`, `body_x`, `body_y`, `body_speed`, `apply_force`, `apply_impulse`, `remove_body`, `step`. Custom functions can be added via `registerFunction`.

Limitation: no table constructors `{}`.

## 7. Fabric and Hair

`ClothSystem` - Verlet integration, structural + diagonal connections, tearFactor break, wind with turbulence, collisions with bodies, pinning to bodies (`pinToBody`).
Pre-made shapes: `createFlag`, `createCurtain`, `createHair` (hair receives attenuated wind and soft connections via knot), `createRopeStrand`.
Demo: 3 pieces, 156 knots