/* phys2d - C API for calling the engine from other languages (FFI friendly). */
#ifndef PHYS2D_C_API_H
#define PHYS2D_C_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct p2World* p2WorldHandle;
typedef void*           p2ConstraintHandle;
typedef uint32_t        p2BodyId;

typedef struct { double x, y; } p2Vec2;
typedef struct { double minX, minY, maxX, maxY; } p2AABB;

typedef enum { P2_STATIC = 0, P2_DYNAMIC = 1, P2_KINEMATIC = 2 } p2BodyType;
typedef enum { P2_SYMPLECTIC_EULER = 0, P2_VERLET = 1, P2_RK4 = 2 } p2Integrator;

typedef struct {
    double   gravityX, gravityY;
    int      integrator;
    int      velocityIterations;
    int      positionIterations;
    int      substeps;
    int      threadCount;
    double   fixedTimeStep;
    double   gridCellSize;
    uint32_t modules;
    size_t   arenaBytes;
} p2WorldConfig;

typedef struct {
    p2BodyId a, b;
    double   normalX, normalY;
    double   pointX, pointY;
    double   penetration;
    double   normalImpulse;
    double   timeOfImpact;
    int      phase;              /* 0 = begin, 1 = persist, 2 = end */
} p2CollisionEvent;

typedef void (*p2CollisionCallback)(const p2CollisionEvent* event, void* userData);

/* ---- world ------------------------------------------------------------- */
p2WorldConfig p2DefaultConfig(void);
p2WorldHandle p2CreateWorld(const p2WorldConfig* config);
void          p2DestroyWorld(p2WorldHandle world);
void          p2Step(p2WorldHandle world, double dt);
void          p2SetGravity(p2WorldHandle world, double x, double y);
void          p2SetIntegrator(p2WorldHandle world, int integrator);
void          p2EnableModule(p2WorldHandle world, uint32_t flag, int enabled);
uint32_t      p2Modules(p2WorldHandle world);
void          p2Clear(p2WorldHandle world);

/* ---- bodies ------------------------------------------------------------ */
p2BodyId p2CreateCircle(p2WorldHandle world, int type, double x, double y,
                        double radius, double density);
p2BodyId p2CreateBox(p2WorldHandle world, int type, double x, double y,
                     double width, double height, double angle, double density);
p2BodyId p2CreatePolygon(p2WorldHandle world, int type, double x, double y,
                         const p2Vec2* verts, int count, double density);
p2BodyId p2CreateCapsule(p2WorldHandle world, int type, double x, double y,
                         double radius, double length, double angle, double density);
void     p2DestroyBody(p2WorldHandle world, p2BodyId body);

int    p2GetTransform(p2WorldHandle world, p2BodyId body, double* x, double* y, double* angle);
int    p2SetTransform(p2WorldHandle world, p2BodyId body, double x, double y, double angle);
int    p2GetVelocity(p2WorldHandle world, p2BodyId body, double* vx, double* vy, double* omega);
int    p2SetVelocity(p2WorldHandle world, p2BodyId body, double vx, double vy, double omega);
int    p2ApplyForce(p2WorldHandle world, p2BodyId body, double fx, double fy);
int    p2ApplyImpulseAtPoint(p2WorldHandle world, p2BodyId body,
                             double ix, double iy, double px, double py);
int    p2SetMaterial(p2WorldHandle world, p2BodyId body, double restitution,
                     double staticFriction, double dynamicFriction);
int    p2GetAABB(p2WorldHandle world, p2BodyId body, p2AABB* out);
double p2GetMass(p2WorldHandle world, p2BodyId body);
double p2GetInertia(p2WorldHandle world, p2BodyId body);
int    p2IsSleeping(p2WorldHandle world, p2BodyId body);
void   p2Wake(p2WorldHandle world, p2BodyId body);

/* ---- constraints ------------------------------------------------------- */
p2ConstraintHandle p2CreateDistance(p2WorldHandle world, p2BodyId a, p2BodyId b,
                                    double lax, double lay, double lbx, double lby, double length);
p2ConstraintHandle p2CreateRevolute(p2WorldHandle world, p2BodyId a, p2BodyId b,
                                    double lax, double lay, double lbx, double lby);
p2ConstraintHandle p2CreateSpring(p2WorldHandle world, p2BodyId a, p2BodyId b,
                                  double lax, double lay, double lbx, double lby,
                                  double length, double stiffness, double damping);
p2ConstraintHandle p2CreateRope(p2WorldHandle world, p2BodyId a, p2BodyId b,
                                double lax, double lay, double lbx, double lby, double maxLength);
p2ConstraintHandle p2CreateAngular(p2WorldHandle world, p2BodyId a, p2BodyId b,
                                   double minAngle, double maxAngle);
void p2DestroyConstraint(p2WorldHandle world, p2ConstraintHandle constraint);

/* ---- events, stats, IO -------------------------------------------------- */
void   p2SetContactCallback(p2WorldHandle world, int phase, p2CollisionCallback cb, void* userData);
size_t p2BodyCount(p2WorldHandle world);
size_t p2ContactCount(p2WorldHandle world);
size_t p2ArenaBytes(p2WorldHandle world);
double p2StageMilliseconds(p2WorldHandle world, int stage);
double p2TotalEnergy(p2WorldHandle world);
int    p2ProfileString(p2WorldHandle world, char* buffer, int bufferSize);
int    p2SaveToFile(p2WorldHandle world, const char* path);
int    p2LoadFromFile(p2WorldHandle world, const char* path);
char*  p2ToJson(p2WorldHandle world, int pretty);
int    p2FromJson(p2WorldHandle world, const char* json);
void   p2FreeString(char* str);

/* ---- debug -------------------------------------------------------------- */
void p2SetDebugLayers(p2WorldHandle world, uint32_t layers);
int  p2BuildDebugSvg(p2WorldHandle world, const char* path, double viewSize);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PHYS2D_C_API_H */
