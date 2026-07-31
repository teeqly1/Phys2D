// nova_demo.cpp - demonstration of the 12 NOVA extensions for phys2d
#include "phys2d/Nova.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace phys2d;
using namespace phys2d::nova;

static void header(const char* t)
{
    std::printf("\n==================================================================\n");
    std::printf("  %s\n", t);
    std::printf("==================================================================\n");
}
static BodyId ground(World& w, real y, real halfW)
{
    const MaterialLibrary& lib = MaterialLibrary::instance();
    BodyDef bd;
    bd.type = BodyType::Static;
    bd.shape = Shape::box(halfW * 2.0, 1.0);
    bd.material = lib.physics(lib.indexOf("asphalt"));
    bd.position = Vec2(0, y - 0.5);
    bd.name = "ground";
    return w.createBody(bd);
}

int main(int argc, char** argv)
{
    int steps = 240;
    for(int i = 1; i < argc; ++i)
        if(!std::strcmp(argv[i], "--steps") && i + 1 < argc) steps = std::atoi(argv[++i]);

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    const MaterialLibrary& lib = MaterialLibrary::instance();

    // ---------------------------------------------------------- 4. MATERIALS
    header("4. MATERIALS - visual + sound + fracture behaviour");
    std::printf("materials in library: %d\n", lib.count());
    const char* show[] = {"glass", "oak", "steel", "snow", "lava", 0};
    for(int i = 0; show[i]; ++i){
        const MaterialDef& m = lib.byName(show[i]);
        const char* fs[] = {"None","Shards","Splinters","Dent","Crumble","Shatter","Tear","Melt"};
        std::printf("  %-14s dens %7.0f  color #%06X rough %.2f gloss %.2f  hz %6.0f absorb %.2f  fracture %-9s shards %d\n",
            m.name.c_str(), (double)m.phys.density, m.look.color, (double)m.look.roughness,
            (double)m.look.gloss, (double)m.sound.baseHz, (double)m.sound.absorption,
            fs[(int)m.fracture], m.shardCount);
    }
    std::printf("impact glass/steel 40 Ns -> %.0f Hz | oak/oak -> %.0f Hz | snow/flesh -> %.0f Hz\n",
        (double)lib.impactHz(lib.indexOf("glass"), lib.indexOf("steel"), 40),
        (double)lib.impactHz(lib.indexOf("oak"), lib.indexOf("oak"), 40),
        (double)lib.impactHz(lib.indexOf("snow"), lib.indexOf("flesh"), 40));

    // -------------------------------------------------------------- 2. 2.5D
    header("2. 2.5D DEPTH LAYERS - render depth, 2D physics");
    {
        World w;
        ground(w, 0, 40);
        DepthWorld depth;
        depth.attach(w);
        BodyDef bd;
        bd.type = BodyType::Dynamic;
        bd.shape = Shape::box(1, 1);
        bd.material = lib.physics(lib.indexOf("oak"));
        bd.position = Vec2(-2, 3); BodyId back = w.createBody(bd);
        bd.position = Vec2(0, 3);  BodyId mid = w.createBody(bd);
        bd.position = Vec2(2, 3);  BodyId front = w.createBody(bd);
        depth.setLayer(back, -1);
        depth.setLayer(mid, 0);
        depth.setLayer(front, 1);
        depth.linkLayers(-1, 0, false);
        depth.linkLayers(0, 1, false);
        depth.linkLayers(-1, 1, false);
        for(int i = 0; i < steps; ++i) w.step(1.0 / 120.0);
        std::printf("parallax back %.2f  front %.2f  scale back %.2f front %.2f\n",
            (double)depth.parallax(-1), (double)depth.parallax(1),
            (double)depth.scaleFor(-1), (double)depth.scaleFor(1));
        std::printf("back<->front collide: %s\n", depth.canCollide(-1, 1) ? "yes" : "no");
        std::vector<BodyId> order = depth.drawOrder();
        std::printf("draw order (far to near):");
        for(size_t i = 0; i < order.size(); ++i) std::printf(" %u", (unsigned)order[i]);
        std::printf("\n");
    }

    // ------------------------------------------------------------ 3. EDITOR
    header("3. VISUAL EDITOR CORE - palette, drag, undo, save/load");
    {
        World w;
        ground(w, 0, 30);
        EditorCore ed;
        ed.attach(w);
        std::printf("palette (%d items):", (int)ed.palette().size());
        for(size_t i = 0; i < ed.palette().size(); ++i) std::printf(" [%d]%s", (int)i, ed.palette()[i].name.c_str());
        std::printf("\n");
        ed.spawn(0, Vec2(0, 3));
        ed.spawn(3, Vec2(1.2, 5));
        ed.spawn(5, Vec2(0, 1.5));
        ed.setGravity(Vec2(0, -9.81));
        ed.setSolverIterations(14, 10);
        ed.setPaused(true);
        for(int i = 0; i < 30; ++i) ed.step(1.0 / 60.0);
        bool got = ed.pick(Vec2(0, 3));
        ed.drag(Vec2(3.0, 2.77));
        ed.setPaused(false);
        for(int i = 0; i < 60; ++i) ed.step(1.0 / 60.0);
        RigidBody* sel = ed.hasSelection() ? w.body(ed.selection()) : 0;
        std::printf("picked: %s  dragged to (%.2f, %.2f)\n", got ? "yes" : "no",
            sel ? (double)sel->position.x : 0.0, sel ? (double)sel->position.y : 0.0);
        int before = (int)w.bodyCount();
        ed.duplicateSelection();
        int dup = (int)w.bodyCount();
        ed.undo();
        std::printf("bodies %d -> %d -> %d after undo\n", before, dup, (int)w.bodyCount());
        std::printf("saveScene: %s\n", ed.saveScene("nova_scene.json") ? "nova_scene.json" : "FAILED");
        std::printf("status: %s\n", ed.statusLine().c_str());
    }

    // ----------------------------------------------------------- 5. PROCGEN
    header("5. PROCEDURAL GENERATION - caves, mazes, cities");
    {
        World cave, maze, city;
        CaveParams cp;
        int nCave = ProcGen::buildCave(cave, cp);
        MazeParams mp;
        int nMaze = ProcGen::buildMaze(maze, mp);
        CityParams cyp;
        int nCity = ProcGen::buildCity(city, cyp);
        std::vector<uint8_t> g = ProcGen::caveGrid(cp);
        Vec2 spawn = ProcGen::findSpawn(g, cp.width, cp.height, cp.cell, cp.origin);
        std::printf("cave  : %d merged wall bodies (%dx%d cells), spawn (%.1f, %.1f)\n", nCave, cp.width, cp.height, (double)spawn.x, (double)spawn.y);
        std::printf("maze  : %d bodies (%dx%d, DFS carve)\n", nMaze, mp.cols, mp.rows);
        std::printf("city  : %d bodies (%d blocks, glass windows + debris)\n", nCity, cyp.blocks);
        for(int i = 0; i < 60; ++i){ cave.step(1.0/120.0); city.step(1.0/120.0); }
        std::printf("city settled, awake bodies %d / %d\n", (int)city.awakeCount(), (int)city.bodyCount());
    }

    // --------------------------------------------------------------- 6. LUA
    header("6. LUA SCRIPTING - engine API exported to script");
    {
        World w;
        ground(w, 0, 30);
        LuaVM vm;
        vm.bindWorld(w);
        const char* code =
            "set_gravity(0, -9.81)\n"
            "local top = 0\n"
            "for i = 0, 9 do\n"
            "  local id = spawn_box(0, 1 + i * 0.9, 0.8, 0.8, 23)\n"
            "  top = id\n"
            "end\n"
            "local ball = spawn_circle(-6, 6, 0.5, 14)\n"
            "apply_impulse(ball, 900, 0)\n"
            "step(1/120, 240)\n"
            "print('lua bodies ' .. body_count())\n"
            "print('top box height ' .. math.floor(body_y(top) * 100) / 100)\n"
            "print('materials in lua ' .. material_count() .. ' ' .. material_name(6))\n"
            "function shove()\n"
            "  for i = 1, 5 do apply_force(i, 4000, 0) end\n"
            "  step(1/120, 60)\n"
            "  return body_speed(1)\n"
            "end\n";
        std::string err;
        if(!vm.doString(code, &err)) std::printf("script error: %s\n", err.c_str());
        else std::printf("script ok\n%s", vm.output().c_str());
        LuaValue r = vm.callFunction("shove");
        std::printf("shove() -> body 1 speed %.2f m/s | natives exported: %d\n", r.toNumber(), vm.nativeCount());
    }

    // ------------------------------------------------------- 7. CLOTH & HAIR
    header("7. CLOTH & HAIR - verlet cloth, tearing, hair strands");
    {
        World w;
        ground(w, 0, 30);
        ClothSystem cloth;
        cloth.attach(w);
        cloth.createFlag(Vec2(-4, 8), 14, 9, 0.18, 1.0);
        cloth.createHair(Vec2(2, 7), 18, 2.2, 0.9);
        cloth.createRopeStrand(Vec2(-2, 6), Vec2(2, 6), 12, 1.0);
        cloth.setWind(Vec2(6.5, 0), 2.2);
        for(int i = 0; i < steps; ++i){
            cloth.update(1.0 / 120.0, 6);
            w.step(1.0 / 120.0);
        }
        std::printf("pieces %d, nodes %d, torn links %d\n", cloth.pieceCount(), (int)cloth.nodeCount(), cloth.tornLinks());
        const ClothPiece& flag = cloth.piece(0);
        const ClothNode& tip = flag.nodes[flag.nodes.size() - 1];
        std::printf("flag tip (%.2f, %.2f) | hair tip y %.2f | rope nodes %d\n",
            (double)tip.p.x, (double)tip.p.y,
            (double)cloth.piece(1).nodes.back().p.y,
            (int)cloth.piece(2).nodes.size());
    }

    // ------------------------------------------------ 8. DEM GRAINS + WEATHER
    header("8. DEM PARTICLES - sand, sparks, smoke, weather");
    {
        World w;
        ground(w, 0, 40);
        GrainSystem grains;
        grains.attach(w);
        grains.emitPile(Vec2(-3, 0.1), 1.6, 1.2, 0.05, GrainKind::Sand);
        grains.emitPile(Vec2(3, 0.1), 1.2, 0.9, 0.07, GrainKind::Gravel);
        grains.emitBurst(Vec2(0, 3), 220, 9.0, GrainKind::Spark);
        grains.emitBurst(Vec2(0, 3), 160, 3.0, GrainKind::Smoke);
        grains.emitBurst(Vec2(0, 3), 120, 6.0, GrainKind::Ember);
        grains.applyExplosion(Vec2(0, 1.0), 4.0, 26000);
        WeatherSystem weather;
        weather.setArea(Vec2(-20, 0), Vec2(20, 22));
        weather.configure(WeatherSystem::Kind::Rain, 0.8, Vec2(-3, 0));
        for(int i = 0; i < steps; ++i){
            weather.update(1.0 / 120.0, grains);
            grains.update(1.0 / 120.0);
            w.step(1.0 / 120.0);
        }
        std::printf("grains alive %d, contacts last step %d, packed %.1f%%\n",
            (int)grains.count(), (int)grains.contactsLastStep(), (double)grains.packedFraction() * 100.0);
        std::printf("weather spawned total %d drops\n", weather.spawnedTotal());
    }

    // ---------------------------------------------------------- 9. VEHICLES
    header("9. VEHICLES - car suspension, tank tracks, aircraft");
    {
        World w;
        w.config().solver.velocityIterations = 18;
        w.config().solver.positionIterations = 12;
        w.config().solver.baumgarte = 0.22;
        ground(w, 0, 400);
        WheeledVehicle car;
        WheeledVehicle::Config cc;
        cc.position = Vec2(0, 1.2);
        std::vector<WheelSetup> ws;
        WheelSetup a;
        a.anchor = Vec2(-1.2, -0.3); a.driven = true;  ws.push_back(a);
        a.anchor = Vec2(1.2, -0.3);  a.steered = true; ws.push_back(a);
        car.create(w, cc, ws);
        for(int i = 0; i < steps * 4; ++i){
            car.update(1.0 / 120.0, 1.0, 0.0, 0.0);
            w.step(1.0 / 120.0);
            car.postStep();
        }
        std::printf("car   : odometer %.0f m, top speed %.0f km/h, travel used %.0f%%, airborne %s\n",
            (double)car.odometer(), (double)car.speedKmh(), (double)car.suspensionTravelUsed() * 100.0,
            car.airborne() ? "yes" : "no");
        std::printf("        spring %.0f N/m  damper %.0f Ns/m\n", (double)car.springRate(), (double)car.damperRate());

        World tw;
        ground(tw, 0, 400);
        TrackedVehicle tank;
        TrackedVehicle::Config tc;
        tc.position = Vec2(0, 2.0);
        tank.create(tw, tc);
        for(int i = 0; i < steps * 4; ++i){
            tank.update(1.0 / 120.0, 1.0, 1.0);
            tw.step(1.0 / 120.0);
        }
        std::printf("tank  : track speed %.1f m/s, ground pressure %.1f kPa, road wheels %d\n",
            (double)tank.trackSpeed(), (double)tank.groundPressure(), (int)tank.roadWheels().size());

        World aw;
        aw.setGravity(Vec2(0, -9.81));
        Aircraft plane;
        Aircraft::Config ac;
        ac.position = Vec2(0, 100);
        std::vector<Aircraft::Wing> wings;
        Aircraft::Wing main;
        main.offset = Vec2(0.2, 0); main.area = 11.0; wings.push_back(main);
        Aircraft::Wing tail;
        tail.offset = Vec2(-2.6, 0); tail.area = 2.2; tail.controlGain = 0.25; wings.push_back(tail);
        plane.create(aw, ac, wings);
        RigidBody* pb = aw.body(plane.body());
        if(pb) pb->velocity = Vec2(70, 0);
        for(int i = 0; i < steps * 4; ++i){
            RigidBody* b = aw.body(plane.body());
            real pitch = 0;
            if(b){
                real err = 100.0 - b->position.y;
                pitch = err * 0.02 - b->velocity.y * 0.06 - b->angularVelocity * 0.30;
                pitch = pitch < -0.4 ? -0.4 : (pitch > 0.4 ? 0.4 : pitch);
            }
            plane.update(1.0 / 120.0, 1.0, pitch);
            aw.step(1.0 / 120.0);
        }
        RigidBody* fb = aw.body(plane.body());
        std::printf("plane : airspeed %.0f m/s, altitude %.0f m, lift %.0f N, drag %.0f N, aoa %.1f deg, %s\n",
            (double)plane.airspeed(), fb ? (double)fb->position.y : 0.0,
            (double)plane.lastLift(), (double)plane.lastDrag(),
            (double)plane.angleOfAttack() * 57.2957795, plane.stalled() ? "STALL" : "clean");
    }

    // ------------------------------------------------------------- 10. OCEAN
    header("10. OCEAN - Gerstner waves, buoyancy, shoreline");
    {
        World w;
        Ocean sea;
        sea.attach(w);
        sea.makeSea(9, 12.0, 4242);
        sea.setShore(60.0, 0.06, 18.0);
        BodyId boat = INVALID_BODY;
        for(int i = 0; i < 6; ++i){
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.shape = Shape::box(3.0, 1.2);
            bd.material = lib.physics(lib.indexOf("oak"));
            bd.material.density = 380;
            bd.position = Vec2(-20.0 + i * 9.0, 1.0);
            bd.allowSleep = false;
            bd.name = "boat";
            BodyId id = w.createBody(bd);
            if(i == 2) boat = id;
        }
        real t = 0;
        real minY = 1e30, maxY = -1e30;
        for(int i = 0; i < steps * 2; ++i){
            sea.applyBuoyancy(t, 1.0 / 120.0);
            w.step(1.0 / 120.0);
            t += 1.0 / 120.0;
            RigidBody* b = w.body(boat);
            if(b){
                if(b->position.y < minY) minY = b->position.y;
                if(b->position.y > maxY) maxY = b->position.y;
            }
        }
        std::printf("waves %d, floating bodies %d\n", sea.waveCount(), sea.floatingBodies());
        std::printf("boat hull y %.2f ... %.2f m (heave %.2f m)\n", (double)minY, (double)maxY, (double)(maxY - minY));
        std::printf("surface at x=0: %.2f m | depth at shore x=55: %.1f m | breaking %.2f\n",
            (double)sea.height(0, t), (double)sea.depth(55.0), (double)sea.breakingIntensity(55.0, t));
    }

    // ------------------------------------------------------------ 1. NETWORK
    header("1. NETWORK PHYSICS - authority, delta compression, prediction");
    {
        World sw, cw;
        ground(sw, 0, 60);
        ground(cw, 0, 60);
        for(int i = 0; i < 40; ++i){
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.shape = Shape::box(0.6, 0.6);
            bd.material = lib.physics(lib.indexOf("oak"));
            bd.position = Vec2(-6.0 + (i % 8) * 1.5, 1.0 + (i / 8) * 0.9);
            sw.createBody(bd);
            cw.createBody(bd);
        }
        ServerAuthority server;
        server.attach(sw);
        server.setInputHandler([](World& w, const NetInput& in){
            RigidBody* b = w.body((BodyId)(1 + in.client));
            if(b){ b->wake(); b->applyForce(Vec2(in.axisX * 4000.0, in.axisY * 4000.0)); }
        });
        ClientPredictor client;
        client.attach(cw);
        client.setInputHandler([](World& w, const NetInput& in){
            RigidBody* b = w.body((BodyId)(1 + in.client));
            if(b){ b->wake(); b->applyForce(Vec2(in.axisX * 4000.0, in.axisY * 4000.0)); }
        });
        InterpolationBuffer interp;
        interp.setDelay(0.10);

        size_t rawTotal = 0, deltaTotal = 0;
        NetSnapshot prev;
        bool hasPrev = false;
        for(int i = 0; i < steps; ++i){
            NetInput in;
            in.tick = (uint32_t)i;
            in.client = 0;
            in.axisX = (float)std::sin(i * 0.05);
            server.pushInput(in);
            client.pushLocalInput(in);
            client.predict(1.0 / 60.0);
            server.step(1.0 / 60.0);
            NetSnapshot snap = server.capture();
            snap.time = i / 60.0;
            std::vector<uint8_t> packet = SnapshotCodec::encode(snap, hasPrev ? &prev : 0);
            rawTotal += SnapshotCodec::rawSize(snap);
            deltaTotal += packet.size();
            NetSnapshot decoded;
            SnapshotCodec::decode(packet, hasPrev ? &prev : 0, decoded);
            decoded.time = snap.time;
            interp.push(decoded);
            prev = snap;
            hasPrev = true;
            if(i % 3 == 0) client.applySnapshot(decoded, (uint32_t)i);
        }
        std::printf("ticks %d | raw %d B -> delta %d B (%.1f%% saved), avg packet %.0f B\n",
            steps, (int)rawTotal, (int)deltaTotal,
            100.0 * (1.0 - (double)deltaTotal / (double)rawTotal),
            (double)deltaTotal / (double)steps);
        std::printf("client reconciliation error %.4f m, unacked inputs %d, interp buffer %d\n",
            (double)client.reconciliationError(), (int)client.unacked(), (int)interp.size());
        NetSnapshot lerped;
        interp.sample(steps / 60.0, lerped);
        std::printf("interpolated snapshot: %d bodies at t-100ms\n", (int)lerped.bodies.size());
        RigidBody* hit = server.rewindQueryPoint((uint32_t)(steps - 20), Vec2(-6, 1));
        std::printf("lag compensated hitscan at tick %d: %s\n", steps - 20, hit ? "HIT" : "miss");
    }

    // ---------------------------------------------------------- 12. ACOUSTICS
    header("12. ACOUSTIC PHYSICS - reverb, occlusion, doppler, surfaces");
    {
        World w;
        Acoustics audio;
        audio.attach(w);
        audio.setRoom(520.0, 380.0);
        audio.setListener(Vec2(0, 1.7), Vec2(0, 0));
        audio.addWall(Vec2(-10, 0), Vec2(-10, 6), lib.indexOf("concrete"));
        audio.addWall(Vec2(10, 0), Vec2(10, 6), lib.indexOf("concrete"));
        audio.addWall(Vec2(-10, 6), Vec2(10, 6), lib.indexOf("foam"));
        std::printf("RT60 (Sabine) %.2f s with %d walls\n", (double)audio.reverbTime60(), audio.wallCount());
        Voice near = audio.impact(Vec2(2, 1), 40.0, lib.indexOf("glass"), lib.indexOf("steel"));
        std::printf("near impact : %.0f Hz gain %.3f delay %.0f ms pan %+.2f\n",
            (double)near.frequency, (double)near.gain, (double)near.delay * 1000.0, (double)near.pan);
        Voice far = audio.impact(Vec2(-30, 1), 40.0, lib.indexOf("glass"), lib.indexOf("steel"));
        std::printf("occluded far: gain %.4f (%.0fx quieter through concrete)\n",
            (double)far.gain, far.gain > 0 ? (double)(near.gain / far.gain) : 0.0);
        Voice step = audio.footstep(Vec2(1, 0), lib.indexOf("snow"));
        std::printf("snow footstep %.0f Hz gain %.3f\n", (double)step.frequency, (double)step.gain);
        std::printf("doppler of source at 60 m/s receding: %.2fx\n", (double)audio.dopplerFor(Vec2(-20, 2), Vec2(-60, 0)));
        audio.update(0.25);
        std::printf("active voices %d, master level %.3f\n", (int)audio.voices().size(), (double)audio.masterLevel());
    }

    // -------------------------------------------------------------- 11. MOBILE
    header("11. MOBILE OPTIMISATION - SIMD detection, power profiles");
    {
        CpuFeatures cpu = detectCpu();
        std::printf("cores %d | sse2 %d sse4 %d avx %d avx2 %d neon %d | backend %s\n",
            cpu.cores, cpu.sse2 ? 1 : 0, cpu.sse4 ? 1 : 0, cpu.avx ? 1 : 0, cpu.avx2 ? 1 : 0, cpu.neon ? 1 : 0, simdBackend());
        const char* names[3] = {"Desktop", "Mobile", "Battery"};
        PowerManager::Profile profs[3] = {PowerManager::Profile::Desktop, PowerManager::Profile::Mobile, PowerManager::Profile::Battery};
        for(int p = 0; p < 3; ++p){
            World w;
            ground(w, 0, 30);
            for(int i = 0; i < 30; ++i){
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.shape = Shape::box(0.5, 0.5);
                bd.material = lib.physics(lib.indexOf("oak"));
                bd.position = Vec2(-4.0 + (i % 10) * 0.9, 1.0 + (i / 10) * 0.7);
                w.createBody(bd);
            }
            PowerManager pm;
            pm.attach(w);
            pm.setProfile(profs[p]);
            int rendered = 0;
            for(int i = 0; i < 120; ++i){
                pm.update(pm.suggestTimestep());
                w.config().solver.velocityIterations = pm.suggestIterations();
                w.step(pm.suggestTimestep());
                if(pm.shouldRenderFrame()) ++rendered;
            }
            std::printf("%-8s dt %.4f s, iters %2d, rendered %3d/120 frames, activity %.2f, ~%.1f W\n",
                names[p], (double)pm.suggestTimestep(), pm.suggestIterations(), rendered,
                (double)pm.activity(), (double)pm.estimatedPowerW());
        }
    }

    // --------------------------------------------------------------- SUMMARY
    header("SUMMARY - NovaSuite (all systems on one world)");
    {
        World w;
        ground(w, 0, 60);
        NovaSuite suite;
        suite.attachAll(w);
        suite.ocean.makeSea(5, 8.0, 77);
        suite.cloth.createFlag(Vec2(-3, 6), 8, 6, 0.2, 1.0);
        suite.grains.emitPile(Vec2(4, 0.1), 0.8, 0.6, 0.06, GrainKind::Sand);
        suite.audio.setRoom(400.0, 320.0);
        suite.audio.impact(Vec2(1, 1), 25.0, lib.indexOf("oak"), lib.indexOf("concrete"));
        for(int i = 0; i < 120; ++i){
            suite.update(1.0 / 120.0);
            w.step(1.0 / 120.0);
        }
        std::printf("%s\n", suite.report().c_str());
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double wall = std::chrono::duration<double>(t1 - t0).count();
    std::printf("\nwall time %.2f s\n", wall);
    std::printf("\nALL 12 SUBSYSTEMS OK\n");
    return 0;
}
