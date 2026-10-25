// The built dartsim plugin sets the workcell contact compliance (DART contact CFM 1e-4) exactly when the
// workcell physical contact detector is selected, and leaves DART's default otherwise.
// usage: native_contact_compliance_test <candidate dartsim plugin> <bullet|fcl>
#include <gz/plugin/Loader.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <dart/constraint/ContactConstraint.hpp>
#include <sdf/World.hh>
#include <iostream>
#include <string>
using namespace ignition;
struct Features : physics::FeatureList<physics::sdf::ConstructSdfWorld, physics::CollisionDetector> {};
void require(bool b, const char* message) { if (!b) throw std::runtime_error(message); }
int run(int argc, char** argv) {
  require(argc == 3, "need candidate library and detector");
  const std::string detector = argv[2];
  plugin::Loader loader; loader.LoadLib(argv[1]);
  auto engine = physics::RequestEngine3d<Features>::From(loader.Instantiate("ignition::physics::dartsim::Plugin"));
  require(engine != nullptr, "engine unavailable");
  sdf::World worldSdf; worldSdf.SetName("w");
  auto world = engine->ConstructWorld(worldSdf); require(world != nullptr, "world construction");
  const double before = dart::constraint::ContactConstraint::getConstraintForceMixing();
  require(before == 1e-5, "unexpected DART default contact CFM");
  world->SetCollisionDetector(detector);
  require(world->GetCollisionDetector() == detector, "detector not selected");
  const double after = dart::constraint::ContactConstraint::getConstraintForceMixing();
  require(after == (detector == "bullet" ? 1e-4 : before), "contact compliance differs from the workcell model");
  return 0;
}
int main(int argc, char** argv) {
  try { const int code = run(argc, argv); std::cout << "PASS " << (argc > 2 ? argv[2] : "") << "\n"; return code; }
  catch (const std::exception& e) { std::cerr << "FAIL " << e.what() << "\n"; return 1; }
}
