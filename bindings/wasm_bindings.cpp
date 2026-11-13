// Emscripten embind bindings: class PhysicsWorld implementing web/src/engine/types.ts.
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "physics/world.hpp"

using emscripten::val;
using namespace physics;

namespace {

double getNumber(const val& obj, const char* key, double fallback) {
  val v = obj[key];
  if (v.isUndefined() || v.isNull()) {
    return fallback;
  }
  return v.as<double>();
}

bool getBool(const val& obj, const char* key, bool fallback) {
  val v = obj[key];
  if (v.isUndefined() || v.isNull()) {
    return fallback;
  }
  return v.as<bool>();
}

#ifdef PHYSICS_BINDINGS_COLLISION_FALLBACK
// Temporary stand-ins until World gains the collision API (see CMakeLists.txt).
BodyId fallbackQueryPoint(const World& world, Vec2 p) {
  const std::vector<std::int32_t>& ids = world.bodyIds();
  for (std::int32_t packed : ids) {
    BodyId id = BodyId::fromInt(packed);
    const RigidBody* b = world.body(id);
    if (b == nullptr) {
      continue;
    }
    Vec2 local = b->worldToLocal(p);
    for (const Fixture& f : b->shapes()) {
      const Shape& s = f.shape;
      if (s.type() == ShapeType::Circle) {
        Vec2 d = local - s.center();
        if (d.x * d.x + d.y * d.y <= s.radius() * s.radius()) {
          return id;
        }
        continue;
      }
      bool inside = true;
      for (int i = 0; i < s.vertexCount(); ++i) {
        Vec2 d = local - s.vertex(i);
        if (d.x * s.normal(i).x + d.y * s.normal(i).y > 0) {
          inside = false;
          break;
        }
      }
      if (inside) {
        return id;
      }
    }
  }
  return BodyId();
}
#endif

class PhysicsWorld {
 public:
  PhysicsWorld() = default;

  void setGravity(double x, double y) { world_.setGravity(Vec2(x, y)); }
  void setMouseMaxSpeed(double speed) { world_.setMouseMaxSpeed(speed); }

  int addBody(const val& def) {
    BodyDef bd;
    std::string type = def["type"].as<std::string>();
    if (type == "static") {
      bd.type = BodyType::Static;
    } else if (type == "kinematic") {
      bd.type = BodyType::Kinematic;
    } else {
      bd.type = BodyType::Dynamic;
    }
    bd.position = Vec2(getNumber(def, "x", 0), getNumber(def, "y", 0));
    bd.angle = getNumber(def, "angle", 0);
    bd.linearDamping = getNumber(def, "linearDamping", 0);
    bd.angularDamping = getNumber(def, "angularDamping", 0);

    Fixture fx;
    val shape = def["shape"];
    std::string kind = shape["kind"].as<std::string>();
    if (kind == "circle") {
      fx.shape = Shape::circle(getNumber(shape, "radius", 0.5));
    } else if (kind == "box") {
      fx.shape = Shape::box(getNumber(shape, "w", 1), getNumber(shape, "h", 1));
    } else {
      std::vector<Vec2> verts;
      val vs = shape["vertices"];
      unsigned n = vs["length"].as<unsigned>();
      for (unsigned i = 0; i < n; ++i) {
        val v = vs[i];
        verts.push_back(Vec2(v["x"].as<double>(), v["y"].as<double>()));
      }
      fx.shape = Shape::polygon(verts);
    }
    if (!fx.shape.isValid()) {
      return -1;
    }
    fx.density = getNumber(def, "density", 1);
    fx.friction = getNumber(def, "friction", 0.5);
    fx.restitution = getNumber(def, "restitution", 0);
    fx.filter.group = static_cast<int>(getNumber(def, "collisionGroup", 0));

    BodyId id = world_.createBody(bd);
    if (!world_.addShape(id, fx)) {
      world_.destroyBody(id);
      return -1;
    }
    return id.toInt();
  }

  void removeBody(int id) { world_.destroyBody(BodyId::fromInt(id)); }

  void setBodyTransform(int id, double x, double y, double angle) {
    world_.setBodyTransform(BodyId::fromInt(id), Vec2(x, y), angle);
  }

  void setBodyVelocity(int id, double vx, double vy, double w) {
    world_.setBodyVelocity(BodyId::fromInt(id), Vec2(vx, vy), w);
  }

  int addRevoluteJoint(const val& def) {
    RevoluteJointDef jd;
    jd.bodyA = BodyId::fromInt(def["bodyA"].as<int>());
    jd.bodyB = BodyId::fromInt(def["bodyB"].as<int>());
    jd.anchor = Vec2(getNumber(def, "anchorX", 0), getNumber(def, "anchorY", 0));
    jd.enableLimit = getBool(def, "enableLimit", false);
    jd.lowerAngle = getNumber(def, "lowerAngle", 0);
    jd.upperAngle = getNumber(def, "upperAngle", 0);
    jd.enableMotor = getBool(def, "enableMotor", false);
    jd.motorSpeed = getNumber(def, "motorSpeed", 0);
    jd.maxMotorTorque = getNumber(def, "maxMotorTorque", 0);
    jd.collideConnected = getBool(def, "collideConnected", false);
    return world_.createRevoluteJoint(jd).toInt();
  }

  void updateRevoluteJoint(int id, const val& patch) {
    RevoluteJoint* j = world_.revoluteJoint(JointId::fromInt(id));
    if (j == nullptr) {
      return;
    }
    if (patch.hasOwnProperty("lowerAngle") || patch.hasOwnProperty("upperAngle")) {
      j->setLimits(getNumber(patch, "lowerAngle", j->lowerAngle()), getNumber(patch, "upperAngle", j->upperAngle()));
    }
    if (patch.hasOwnProperty("enableLimit")) {
      j->setLimitEnabled(getBool(patch, "enableLimit", j->limitEnabled()));
    }
    if (patch.hasOwnProperty("enableMotor")) {
      j->setMotorEnabled(getBool(patch, "enableMotor", false));
    }
    if (patch.hasOwnProperty("motorSpeed")) {
      j->setMotorSpeed(getNumber(patch, "motorSpeed", 0));
    }
    if (patch.hasOwnProperty("maxMotorTorque")) {
      j->setMaxMotorTorque(getNumber(patch, "maxMotorTorque", 0));
    }
  }

  int addDistanceJoint(const val& def) {
    DistanceJointDef jd;
    jd.bodyA = BodyId::fromInt(def["bodyA"].as<int>());
    jd.bodyB = BodyId::fromInt(def["bodyB"].as<int>());
    jd.anchorA = Vec2(getNumber(def, "anchorAX", 0), getNumber(def, "anchorAY", 0));
    jd.anchorB = Vec2(getNumber(def, "anchorBX", 0), getNumber(def, "anchorBY", 0));
    jd.length = getNumber(def, "length", -1);
    jd.hertz = getNumber(def, "frequencyHz", 0);
    jd.dampingRatio = getNumber(def, "dampingRatio", 0);
    jd.collideConnected = getBool(def, "collideConnected", false);
    return world_.createDistanceJoint(jd).toInt();
  }

  void removeJoint(int id) { world_.destroyJoint(JointId::fromInt(id)); }

  int createMouseJoint(int bodyId, double x, double y, double maxForce) {
    return world_.createMouseJoint(BodyId::fromInt(bodyId), Vec2(x, y), maxForce).toInt();
  }

  void setMouseTarget(int jointId, double x, double y) { world_.setMouseTarget(JointId::fromInt(jointId), Vec2(x, y)); }

  int queryPoint(double x, double y) {
#ifdef PHYSICS_BINDINGS_COLLISION_FALLBACK
    return fallbackQueryPoint(world_, Vec2(x, y)).toInt();
#else
    return world_.queryPoint(Vec2(x, y)).toInt();
#endif
  }

  void step(double frameDt) {
#ifdef PHYSICS_BINDINGS_COLLISION_FALLBACK
    auto t0 = std::chrono::steady_clock::now();
    world_.advance(frameDt);
    std::chrono::duration<double, std::milli> ms = std::chrono::steady_clock::now() - t0;
    fallbackStepMs_ = ms.count();
#else
    world_.advance(frameDt);
#endif
  }

  val getBodyStates() {
    const std::vector<float>& v = world_.bodyStates();
    return val(emscripten::typed_memory_view(v.size(), v.data()));
  }

  val getBodyIds() {
    const std::vector<std::int32_t>& v = world_.bodyIds();
    return val(emscripten::typed_memory_view(v.size(), v.data()));
  }

  val getJointAnchors() {
    const std::vector<float>& v = world_.jointAnchors();
    return val(emscripten::typed_memory_view(v.size(), v.data()));
  }

  val getContacts() {
#ifdef PHYSICS_BINDINGS_COLLISION_FALLBACK
    return val(emscripten::typed_memory_view(fallbackContacts_.size(), fallbackContacts_.data()));
#else
    const std::vector<float>& v = world_.contactPoints();
    return val(emscripten::typed_memory_view(v.size(), v.data()));
#endif
  }

  val getStats() {
    val o = val::object();
    o.set("bodies", static_cast<double>(world_.bodyCount()));
#ifdef PHYSICS_BINDINGS_COLLISION_FALLBACK
    o.set("contacts", 0.0);
    o.set("stepMs", fallbackStepMs_);
#else
    o.set("contacts", static_cast<double>(world_.contactCount()));
    o.set("stepMs", world_.lastStepMs());
#endif
    o.set("kineticEnergy", world_.kineticEnergy());
    o.set("potentialEnergy", world_.potentialEnergy());
    return o;
  }

  void destroy() {}

 private:
  World world_;
#ifdef PHYSICS_BINDINGS_COLLISION_FALLBACK
  double fallbackStepMs_ = 0;
  std::vector<float> fallbackContacts_;
#endif
};

}  // namespace

EMSCRIPTEN_BINDINGS(physics_engine) {
  emscripten::class_<PhysicsWorld>("PhysicsWorld")
      .constructor<>()
      .function("setGravity", &PhysicsWorld::setGravity)
      .function("setMouseMaxSpeed", &PhysicsWorld::setMouseMaxSpeed)
      .function("addBody", &PhysicsWorld::addBody)
      .function("removeBody", &PhysicsWorld::removeBody)
      .function("setBodyTransform", &PhysicsWorld::setBodyTransform)
      .function("setBodyVelocity", &PhysicsWorld::setBodyVelocity)
      .function("addRevoluteJoint", &PhysicsWorld::addRevoluteJoint)
      .function("updateRevoluteJoint", &PhysicsWorld::updateRevoluteJoint)
      .function("addDistanceJoint", &PhysicsWorld::addDistanceJoint)
      .function("removeJoint", &PhysicsWorld::removeJoint)
      .function("createMouseJoint", &PhysicsWorld::createMouseJoint)
      .function("setMouseTarget", &PhysicsWorld::setMouseTarget)
      .function("queryPoint", &PhysicsWorld::queryPoint)
      .function("step", &PhysicsWorld::step)
      .function("getBodyStates", &PhysicsWorld::getBodyStates)
      .function("getBodyIds", &PhysicsWorld::getBodyIds)
      .function("getJointAnchors", &PhysicsWorld::getJointAnchors)
      .function("getContacts", &PhysicsWorld::getContacts)
      .function("getStats", &PhysicsWorld::getStats)
      .function("destroy", &PhysicsWorld::destroy);
}
