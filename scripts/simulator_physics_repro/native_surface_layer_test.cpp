// The built dartsim plugin applies authored contact surface layers (<surface><contact><ode><min_depth>) per
// contact pair: the reported (corrected) depth is max(0, depth - layer) with the pair's layer the smaller of
// its two surfaces' layers, so a pair with an unlayered surface stays rigid; an unsupported layer is rejected.
// usage: native_surface_layer_test <candidate dartsim plugin> <both|one|none|rejected>
#include <gz/plugin/Loader.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/dartsim/World.hh>
#include <dart/simulation/World.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <iostream>
#include <string>
using namespace ignition;
struct Features : physics::FeatureList<physics::sdf::ConstructSdfWorld, physics::sdf::ConstructSdfModel,
                                       physics::ForwardStep, physics::CollisionDetector,
                                       physics::dartsim::RetrieveWorld> {};
void require(bool b, const char* message) { if (!b) throw std::runtime_error(message); }
// Two 25 mm boxes overlapping 50 um along z; the lower one static.
std::string box(const std::string& name, double z, bool fixed, const std::string& layer) {
  return "<model name='" + name + "'><static>" + (fixed ? "true" : "false") + "</static><pose>0 0 " + std::to_string(z) +
         " 0 0 0</pose><link name='link'><inertial><mass>0.02</mass></inertial><collision name='c'><geometry><box>"
         "<size>0.025 0.025 0.025</size></box></geometry><surface><contact><ode>" +
         (layer.empty() ? "" : "<min_depth>" + layer + "</min_depth>") + "</ode></contact></surface></collision></link></model>";
}
int run(int argc, char** argv) {
  require(argc == 3, "need candidate library and mode");
  const std::string mode = argv[2];
  plugin::Loader loader; loader.LoadLib(argv[1]);
  auto engine = physics::RequestEngine3d<Features>::From(loader.Instantiate("ignition::physics::dartsim::Plugin"));
  require(engine != nullptr, "engine unavailable");
  sdf::World worldSdf; worldSdf.SetName("w"); worldSdf.SetGravity(math::Vector3d::Zero);
  auto world = engine->ConstructWorld(worldSdf); require(world != nullptr, "world construction");
  world->SetCollisionDetector("bullet");
  const std::string upper = mode == "both" ? "0.0001" : mode == "rejected" ? "0.002" : mode == "one" ? "0.0001" : "";
  const std::string lower = mode == "both" ? "0.0001" : "";
  sdf::Root a, b;
  require(a.LoadSdfString("<sdf version='1.9'>" + box("lower", 0, true, lower) + "</sdf>").empty(), "lower SDF");
  require(b.LoadSdfString("<sdf version='1.9'>" + box("upper", 0.025 - 5e-5, false, upper) + "</sdf>").empty(), "upper SDF");
  require(world->ConstructModel(*a.Model()) != nullptr, "lower construction");
  if (mode == "rejected") {
    try { world->ConstructModel(*b.Model()); } catch (const std::invalid_argument& e) {
      require(std::string(e.what()).find("contact surface layer") != std::string::npos, "wrong rejection");
      return 0;
    }
    throw std::runtime_error("unsupported layer accepted");
  }
  require(world->ConstructModel(*b.Model()) != nullptr, "upper construction");
  physics::ForwardStep::Input in; physics::ForwardStep::Output out; physics::ForwardStep::State state;
  world->Step(out, state, in);
  const auto& contacts = world->GetDartsimWorld()->getLastCollisionResult().getContacts();
  require(contacts.size() == 4, "face contacts absent");
  const double expected = mode == "both" ? 0. : 5e-5;     // both layered: within the layer; otherwise rigid
  for (const auto& c : contacts)
    require(std::abs(c.penetrationDepth - expected) < 1e-9, "reported depth differs from the pair's layer rule");
  return 0;
}
int main(int argc, char** argv) {
  try { const int code = run(argc, argv); std::cout << "PASS " << (argc > 2 ? argv[2] : "") << "\n"; return code; }
  catch (const std::exception& e) { std::cerr << "FAIL " << e.what() << "\n"; return 1; }
}
