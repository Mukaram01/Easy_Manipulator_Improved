#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <iostream>

#include "TouchingBulletContacts.hpp"
using namespace dart;
struct IgnoreAll : collision::CollisionFilter {
  bool ignoresCollision(const collision::CollisionObject*,
                        const collision::CollisionObject*) const override {
    return true;
  }
};
int main() {
  try {
    std::weak_ptr<collision::CollisionDetector> lifetime;
    for (int trial = 0; trial < 20; ++trial) {
      auto detector = std::make_shared<workcell::TouchingBulletContacts>();
      lifetime = detector;
      auto clone = detector->cloneWithoutCollisionObjects();
      dynamics::SimpleFrame a, b;
      a.setShape(std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()));
      b.setShape(std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()));
      a.setTranslation(Eigen::Vector3d(1000, 0, 0));
      b.setTranslation(Eigen::Vector3d(1001 - 1e-8, 0, 0));
      auto group = detector->createCollisionGroup(&a, &b);
      collision::CollisionResult result;
      if (!detector->collide(group.get(), collision::CollisionOption(true, 1000, nullptr),
                             &result) ||
          result.getNumContacts() != 4)
        throw std::runtime_error("accurate BOX face contacts absent");
      for (const auto& c : result.getContacts()) {
        if (std::abs(c.penetrationDepth - 1e-8) > 1e-11 ||
            std::abs(std::abs(c.normal.x()) - 1) > 1e-12 || !result.inCollision(&a) ||
            !result.inCollision(&b))
          throw std::runtime_error("BOX precision/normal/membership defect");
        if (c.collisionObject1->getCollisionDetector() != detector.get() ||
            c.collisionObject2->getCollisionDetector() != detector.get())
          throw std::runtime_error("borrowed BOX object owner");
      }
      if (detector->collide(group.get(),
                            collision::CollisionOption(true, 1000, std::make_shared<IgnoreAll>()),
                            &result) ||
          result.getNumContacts())
        throw std::runtime_error("original filter ignored");
      if (!group->collide()) throw std::runtime_error("default binary BOX query false");
      for (bool detail : {false, true})
        for (std::size_t limit : {1u, 2u, 3u, 4u, 5u}) {
          if (!group->collide(collision::CollisionOption(detail, limit, nullptr), &result) ||
              result.getNumContacts() != std::min(limit, std::size_t(4)))
            throw std::runtime_error("stock BOX contact cap mismatch");
        }
      dynamics::SimpleFrame c, d;
      c.setShape(std::make_shared<dynamics::SphereShape>(.5));
      d.setShape(std::make_shared<dynamics::SphereShape>(.5));
      c.setTranslation(Eigen::Vector3d(0, 0, 0));
      d.setTranslation(Eigen::Vector3d(.9, 0, 0));
      group->addShapeFramesOf(&c, &d);
      for (bool detail : {false, true})
        for (std::size_t limit : {1u, 2u, 3u, 4u, 5u, 6u}) {
          if (!group->collide(collision::CollisionOption(detail, limit, nullptr), &result) ||
              result.getNumContacts() != std::min(limit, std::size_t(5)))
            throw std::runtime_error("mixed Bullet/BOX global cap mismatch");
        }
      group->removeShapeFrame(&c);
      group->removeShapeFrame(&d);
      if (detector->collide(group.get(), collision::CollisionOption(true, 0, nullptr), &result) ||
          result.getNumContacts())
        throw std::runtime_error("zero contact budget ignored");
      b.setTranslation(Eigen::Vector3d(1001 + 1e-8, 0, 0));
      if (detector->collide(group.get(), collision::CollisionOption(true, 1000, nullptr), &result))
        throw std::runtime_error("separated BOX counted");
      group->removeShapeFrame(&b);
      if (detector->collide(group.get(), collision::CollisionOption(true, 1000, nullptr),
                            &result) ||
          group->getNumShapeFrames() != 1)
        throw std::runtime_error("removed BOX survives");
    }
    if (!lifetime.expired()) throw std::runtime_error("physical group detector ownership cycle");
    std::cout << "BOX precision/filter/budgets/membership/removal/clone/lifetime PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
