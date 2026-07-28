#include "phys2d/Body.h"

namespace phys2d {

void RigidBody::setMassData(real m, real I) {
    mass    = std::max(m, 0.0);
    inertia = std::max(I, 0.0);
    if (type == BodyType::Dynamic && mass > EPSILON) {
        invMass = 1.0 / mass;
    } else {
        invMass = 0.0;
        if (type != BodyType::Dynamic) { mass = 0.0; }
    }
    if (type == BodyType::Dynamic && !fixedRotation && inertia > EPSILON) {
        invInertia = 1.0 / inertia;
    } else {
        invInertia = 0.0;
    }
}

void RigidBody::computeMassFromShape() {
    const MassData md = shape.computeMass(material.density);
    localCenter = md.center;
    setMassData(md.mass, md.inertia);
}

void RigidBody::setType(BodyType t) {
    type = t;
    if (t == BodyType::Dynamic) {
        computeMassFromShape();
        wake();
    } else {
        invMass = 0.0;
        invInertia = 0.0;
        velocity = (t == BodyType::Static) ? Vec2() : velocity;
        if (t == BodyType::Static) angularVelocity = 0.0;
    }
}

void RigidBody::updateVertices() {
    const Transform xf = transform();
    if (worldVertices.size() != localVertices.size())
        worldVertices.resize(localVertices.size());
    for (size_t i = 0; i < localVertices.size(); ++i)
        worldVertices[i] = xf.apply(localVertices[i]);
    centerOfMass   = xf.apply(localCenter);
    boundingRadius = shape.boundingRadius;
}

void RigidBody::updateAABB(real margin) {
    aabb = shape.computeAABB(transform());
    fatAABB = (margin > 0.0) ? aabb.expanded(margin) : aabb;
}

} // namespace phys2d
